"""Generic single-arm queue skills (pick / place / Cartesian cache goto).

单臂段使用 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice`（由结构化配置叠层解析）。
笛卡尔回程用 ``robot.cache_ee_pose`` + ``single_arm.goto_cache_pose``。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from geometry_msgs.msg import Pose
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    euler_rpy_to_quat_xyzw,
    pose_from_tuple,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmTarget,
    SendMode,
    StageTarget,
    assign_to_arm,
    build_single_arm_pick_sequence,
    build_single_arm_place_sequence,
    build_single_arm_return_home_sequence,
)

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_object_pose_from_service,
)
from robot_action_composer.task_runtime.object_binding import resolve_pick_object_binding
from robot_action_composer.task_runtime.object_resolution_replay import resolve_object_pose_for_task

from robot_action_composer.motion_generation.tasks.pick_place import (  # pyright: ignore[reportMissingImports]
    apply_object_local_offset_to_pose,
    resolve_place_skill_from_entity,
)
from robot_action_composer.motion_generation.tasks.object_orientation import (  # pyright: ignore[reportMissingImports]
    compose_aligned_ee_orientation,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext, queue_primary_ee_frame_id
from robot_action_composer.task_runtime.ee_motion_frame import (
    clone_pose,
    current_ee_orientation_xyzw,
    param_vec3,
    pose_quat_xyzw,
    read_current_ee_pose_in_motion_frame,
    set_pose_quat_xyzw,
)
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.config.single_arm import (
    QueueSingleArmSlice,
    QueueSlicePlace,
    overlay_queue_single_arm_pick_from_params,
    overlay_queue_single_arm_place_from_params,
    overlay_queue_single_arm_from_params,
)
from robot_action_composer.task_runtime.types import ExecutionMeta


def _arm_side_and_ee_prefix(qt: QueueSingleArmSlice) -> tuple[ArmSide, bool, str]:
    """按 ``qt.common.arm`` 选择工作臂与 EE 前缀。"""
    arm = qt.common.arm.strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right', got {qt.common.arm!r}")
    source_is_right = arm == "right"
    arm_side = ArmSide.RIGHT if source_is_right else ArmSide.LEFT
    ee_prefix = "right_ee" if source_is_right else "left_ee"
    return arm_side, source_is_right, ee_prefix


def _stamped_mode(ctx: QueueRuntimeContext) -> SendMode:
    return SendMode.STAMPED if ctx.use_stamped else SendMode.UNSTAMPED


_ROS_ARM_MOVEL_DURATION_PARAM = "movel_duration"


def _apply_arm_movel_duration(ctx: QueueRuntimeContext, *, duration: Any, label: str) -> None:
    """在单臂笛卡尔段执行前写 ``arm_controller.movel_duration``。"""
    if duration is None:
        return
    try:
        d = float(duration)
    except (TypeError, ValueError):
        print(f"[{label}] WARN: arm_movel_duration must be numeric, got {duration!r}")
        return
    iface = ctx.interface
    node = str(getattr(iface, "arm_controller", "") or "").strip()
    if not node:
        print(
            f"[{label}] WARN: arm_movel_duration={d} but arm_controller is empty; "
            "configure unified_arm_joint_controller_topic / left_arm_joint_controller_topic on ROS2RobotInterface."
        )
        return
    ok = iface.set_node_parameters(full_node_name=node, parameters={_ROS_ARM_MOVEL_DURATION_PARAM: d})
    if ok:
        print(f"[{label}] Set {node}.{_ROS_ARM_MOVEL_DURATION_PARAM} = {d}")
    else:
        print(f"[{label}] WARN: failed to set {node}.{_ROS_ARM_MOVEL_DURATION_PARAM} = {d}")


def _pick_execution_frame_id(ctx: QueueRuntimeContext, motion_frame_id: Any) -> str:
    base_f = str(ctx.frame_id).strip() or "base_link"
    if motion_frame_id is None:
        return base_f
    raw = str(motion_frame_id).strip()
    return raw or base_f


def _pick_tf_timeout(raw: Any) -> float:
    if raw is None:
        return 2.0
    try:
        return float(raw)
    except (TypeError, ValueError):
        return 2.0


def _is_bimanual_ee_cache(raw: Mapping[str, Any]) -> bool:
    """双臂缓存：``robot.cache_ee_pose`` + ``which: both`` 写入的 ``{left, right}``（无顶层 ``pose``）。"""
    return "left" in raw and "right" in raw and "pose" not in raw


def _scratch_pose_entry_to_geometry_pose(entry: Mapping[str, Any]) -> Pose:
    """解析 :func:`robot.cache_ee_pose` 写入的 ``{"pose": {...}, "gripper": ...}``。"""
    pose_d = entry.get("pose")
    if not isinstance(pose_d, Mapping):
        raise ValueError("cached entry missing 'pose' object (use robot.cache_ee_pose)")
    pos = pose_d.get("position")
    ori = pose_d.get("orientation")
    if not isinstance(pos, Mapping) or not isinstance(ori, Mapping):
        raise ValueError("scratch pose expects position and orientation mappings")
    pose = Pose()
    pose.position.x = float(pos["x"])
    pose.position.y = float(pos["y"])
    pose.position.z = float(pos["z"])
    pose.orientation.x = float(ori["x"])
    pose.orientation.y = float(ori["y"])
    pose.orientation.z = float(ori["z"])
    pose.orientation.w = float(ori["w"])
    return pose


def _binding_params_for_pick(params: Mapping[str, Any], pk: Any) -> dict[str, Any]:
    """Build resolve_pick_object_binding input; preserve explicit offset key presence from ``params``."""
    out: dict[str, Any] = {}
    for key in ("object_prim_path", "object_position_offset", "object_key", "grasp_id", "grasp_prim_path"):
        if key in params:
            out[key] = params[key]
    for key in ("object_prim_path", "object_key", "grasp_id", "grasp_prim_path"):
        if key in out:
            continue
        val = getattr(pk, key, None)
        if val is None:
            continue
        if isinstance(val, str) and not val.strip():
            continue
        out[key] = val
    return out


def _clone_pose(pose: Pose) -> Pose:
    out = Pose()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    out.orientation.x = float(pose.orientation.x)
    out.orientation.y = float(pose.orientation.y)
    out.orientation.z = float(pose.orientation.z)
    out.orientation.w = float(pose.orientation.w)
    return out


def _resolve_object_target_pose_from_pick_like_params(
    *,
    ctx: QueueRuntimeContext,
    object_prim_path: str,
    object_position_offset: tuple[float, float, float],
    motion_frame_id: Any,
    tf_lookup_timeout: Any,
    label: str,
) -> tuple[Pose, Pose, str]:
    """Resolve grasp target + object orientation from ``object_prim_path``.

    Returns ``(grasp_pose, object_orient_pose, exec_frame)``. Orientation for
    ``compose_aligned_ee_orientation`` must come from the rigid-body prim pose
    (after TF), not from the grasp-offset position.
    """
    exec_f = _pick_execution_frame_id(ctx, motion_frame_id)
    arm_side = ctx.task_cfg.common.arm.strip().lower()
    object_role = "pick_target" if label == "single_arm.pick" else "move_to_object_target"
    if ctx.object_resolution is not None:
        obj_pose = resolve_object_pose_for_task(
            ctx,
            object_prim_path=object_prim_path,
            include_orientation=True,
            arm_side=arm_side,
            object_role=object_role,
            entity_state_timeout=SERVICE_CALL_TIMEOUT,
            retries=SERVICE_CALL_RETRIES,
            retry_delay=SERVICE_RETRY_DELAY,
        )
    else:
        obj_pose = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            object_prim_path,
            include_orientation=True,
        )
    if exec_f != ctx.frame_id:
        iface = ctx.interface
        if not hasattr(iface, "transform_pose"):
            raise TypeError(f"{label}: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)")
        transformed = iface.transform_pose(
            obj_pose,
            str(ctx.frame_id),
            exec_f,
            timeout=_pick_tf_timeout(tf_lookup_timeout),
        )
        if transformed is None:
            raise RuntimeError(
                f"{label}: TF {ctx.frame_id!r} -> {exec_f!r} failed for "
                f"object_prim_path={object_prim_path!r}; check motion_frame_id and TF."
            )
        obj_pose = transformed
    orient_pose = _clone_pose(obj_pose)
    grasp_pose = apply_object_local_offset_to_pose(_clone_pose(obj_pose), object_position_offset)
    return grasp_pose, orient_pose, exec_f


def skill_pick(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(f"single_arm.pick expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}")
    qt = overlay_queue_single_arm_pick_from_params(ctx.task_cfg, params)
    arm_side, _source_is_right, _ee_prefix = _arm_side_and_ee_prefix(qt)
    pk = qt.pick
    path, object_position_offset = resolve_pick_object_binding(
        _binding_params_for_pick(params, pk),
        objects=ctx.objects,
        active_object=ctx.active_object,
        cache=ctx.grasp_offset_cache,
    )
    prepare_offset = pk.prepare_offset
    pick_clearance = pk.pick_clearance
    ee_base_orientation = pk.ee_base_orientation
    motion_frame_id = pk.motion_frame_id
    tf_lookup_timeout = pk.tf_lookup_timeout
    arm_movel_duration = pk.arm_movel_duration
    # common.use_object_orientation=true 时等价于 ee_orientation_frame=object（兼容旧字段）
    ee_frame = str(pk.ee_orientation_frame or "motion")
    if bool(qt.common.use_object_orientation) and ee_frame in ("", "motion", "base", "world"):
        ee_frame = "object"
    ori_mode = str(pk.object_orientation_mode or "yaw")
    if not path:
        raise ValueError("object_prim_path is required for skill single_arm.pick")
    _apply_arm_movel_duration(ctx, duration=arm_movel_duration, label="single_arm.pick")
    gripper_open = float(ctx.gripper_open)
    gripper_closed = (
        float(pk.gripper_closed) if pk.gripper_closed is not None else float(ctx.gripper_closed)
    )
    target, object_orient, exec_f = _resolve_object_target_pose_from_pick_like_params(
        ctx=ctx,
        object_prim_path=path,
        object_position_offset=object_position_offset,
        motion_frame_id=motion_frame_id,
        tf_lookup_timeout=tf_lookup_timeout,
        label="single_arm.pick",
    )
    ee_orientation = compose_aligned_ee_orientation(
        ee_base_orientation,
        object_orient,
        ee_orientation_frame=ee_frame,
        object_orientation_mode=ori_mode,
        aligned_object_yaw=pk.aligned_object_yaw,
        aligned_object_roll=pk.aligned_object_roll,
        object_prim_path=path,
    )
    retreat_offset = (
        rotate_vector_by_quat(pk.ee_lift_offset, ee_orientation)
        if pk.ee_lift_offset is not None
        else None
    )
    retreat_xyz = (
        rotate_vector_by_quat(pk.ee_retreat_offset, ee_orientation)
        if pk.ee_retreat_offset is not None
        else None
    )
    arm_seq = build_single_arm_pick_sequence(
        target_pose=target,
        ee_base_orientation=ee_orientation,
        prepare_offset=prepare_offset,
        pick_clearance=pick_clearance,
        retreat_offset=retreat_offset,
        retreat_xyz=retreat_xyz,
        retreat_open_gripper=pk.retreat_open_gripper,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="TaskQ-Pick",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.task_cfg = replace(ctx.task_cfg, common=qt.common, pick=pk)
    ctx.gripper_for_return_home = gripper_open if pk.retreat_open_gripper else gripper_closed
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ pick timeout",
    )


def skill_move_to_object(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    """Move one arm to an object-relative Cartesian target pose (no grasp sequence)."""
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(
            f"single_arm.move_to_object expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}"
        )
    qt = overlay_queue_single_arm_pick_from_params(ctx.task_cfg, params)
    arm_side, _source_is_right, _ee_prefix = _arm_side_and_ee_prefix(qt)
    pk = qt.pick
    if not pk.object_prim_path:
        raise ValueError("object_prim_path is required for skill single_arm.move_to_object")

    _apply_arm_movel_duration(ctx, duration=pk.arm_movel_duration, label="single_arm.move_to_object")
    target, object_orient, exec_f = _resolve_object_target_pose_from_pick_like_params(
        ctx=ctx,
        object_prim_path=pk.object_prim_path,
        object_position_offset=pk.object_position_offset,
        motion_frame_id=pk.motion_frame_id,
        tf_lookup_timeout=pk.tf_lookup_timeout,
        label="single_arm.move_to_object",
    )

    # Target orientation: 对齐标定下的 ee_base，可选按物体 yaw/姿态左乘。
    ee_frame = str(pk.ee_orientation_frame or "motion")
    if bool(qt.common.use_object_orientation) and ee_frame in ("", "motion", "base", "world"):
        ee_frame = "object"
    ee_orientation = compose_aligned_ee_orientation(
        pk.ee_base_orientation,
        object_orient,
        ee_orientation_frame=ee_frame,
        object_orientation_mode=str(pk.object_orientation_mode or "yaw"),
        aligned_object_yaw=pk.aligned_object_yaw,
        aligned_object_roll=pk.aligned_object_roll,
        object_prim_path=pk.object_prim_path,
    )
    target.orientation.x = float(ee_orientation[0])
    target.orientation.y = float(ee_orientation[1])
    target.orientation.z = float(ee_orientation[2])
    target.orientation.w = float(ee_orientation[3])

    grip_v = float(params["gripper"]) if params.get("gripper") is not None else ctx.gripper_for_return_home
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=target,
        gripper=grip_v,
        stage_name="TaskQ-MoveToObject",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.task_cfg = replace(ctx.task_cfg, common=qt.common, pick=pk)
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ move_to_object timeout",
    )


def skill_place(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(f"single_arm.place expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}")
    qt = overlay_queue_single_arm_place_from_params(ctx.task_cfg, params)
    arm_side, _source_is_right, _ee_prefix = _arm_side_and_ee_prefix(qt)
    pl = qt.place
    explicit_orientation = pl.ee_base_orientation is not None
    pl2 = resolve_place_skill_from_entity(
        pl,
        base_world_pos=ctx.base_world_pos,
        base_world_quat=ctx.base_world_quat,
        ctx=ctx,
    )
    iface = ctx.interface
    handler = iface.left_arm_handler if arm_side == ArmSide.LEFT else iface.right_arm_handler
    if handler is None:
        raise TypeError("single_arm.place requires arm handler")

    exec_f = _pick_execution_frame_id(ctx, pl2.motion_frame_id)
    base_f = str(ctx.frame_id).strip() or "base_link"
    if not explicit_orientation:
        ori, exec_f = current_ee_orientation_xyzw(
            iface, handler, base_f, params, label="single_arm.place"
        )
        pl2 = replace(pl2, ee_base_orientation=ori)

    if pl2.place_position is None:
        raise ValueError(
            "place_position is required for single_arm.place "
            "(set via skill params, object_prim_path, or skill_defaults.single_arm.place)"
        )
    if pl2.ee_base_orientation is None:
        raise ValueError("ee_base_orientation is required for single_arm.place")

    _apply_arm_movel_duration(ctx, duration=pl2.arm_movel_duration, label="single_arm.place")
    if exec_f != base_f:
        if not hasattr(iface, "transform_pose"):
            raise TypeError("single_arm.place: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)")
        tft = _pick_tf_timeout(pl2.tf_lookup_timeout)
        if explicit_orientation:
            pose_in = pose_from_tuple(pl2.place_position, pl2.ee_base_orientation)
            transformed = iface.transform_pose(pose_in, base_f, exec_f, timeout=tft)
            if transformed is None:
                raise RuntimeError(
                    f"single_arm.place: TF {base_f!r} -> {exec_f!r} failed for place pose; "
                    "check motion_frame_id and TF."
                )
            pl2 = replace(
                pl2,
                place_position=(
                    float(transformed.position.x),
                    float(transformed.position.y),
                    float(transformed.position.z),
                ),
                ee_base_orientation=(
                    float(transformed.orientation.x),
                    float(transformed.orientation.y),
                    float(transformed.orientation.z),
                    float(transformed.orientation.w),
                ),
            )
        else:
            pose_in = Pose()
            pose_in.position.x = float(pl2.place_position[0])
            pose_in.position.y = float(pl2.place_position[1])
            pose_in.position.z = float(pl2.place_position[2])
            pose_in.orientation.w = 1.0
            transformed = iface.transform_pose(pose_in, base_f, exec_f, timeout=tft)
            if transformed is None:
                raise RuntimeError(
                    f"single_arm.place: TF {base_f!r} -> {exec_f!r} failed for place position; "
                    "check motion_frame_id and TF."
                )
            pl2 = replace(
                pl2,
                place_position=(
                    float(transformed.position.x),
                    float(transformed.position.y),
                    float(transformed.position.z),
                ),
            )
    ctx.task_cfg = replace(ctx.task_cfg, common=qt.common, place=pl2)
    gripper_open = (
        float(pl2.gripper_open) if pl2.gripper_open is not None else float(ctx.gripper_open)
    )
    arm_seq = build_single_arm_place_sequence(
        place_position=pl2.place_position,
        place_orientation=pl2.ee_base_orientation,
        place_axis=(pl2.ee_place_axis if pl2.ee_place_axis is not None else qt.pick.ee_pick_axis),
        prepare_offset=pl2.prepare_offset,
        place_insert_clearance=pl2.place_insert_clearance,
        ee_retreat_offset=pl2.ee_retreat_offset,
        gripper_open=gripper_open,
        stage_prefix="TaskQ-Place",
        start_index=1,
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.gripper_for_return_home = gripper_open
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ place timeout",
    )


def skill_goto_cache_pose(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """笛卡尔运动到 ``ctx.scratch[key]`` 中的缓存末端位姿（由 ``robot.cache_ee_pose`` 写入）。

    若缓存为双臂结构（``which: both``），须指定 ``side: left|right`` 或依赖 ``task_cfg.common.arm`` 选侧。
    双臂同步回程请用 ``dual_arm.goto_cache_pose``。
    """
    key = str(params.get("key", "saved_ee_pose")).strip() or "saved_ee_pose"
    raw = ctx.scratch_get(key)
    if not isinstance(raw, Mapping):
        raise ValueError(
            f"single_arm.goto_cache_pose: cache[{key!r}] missing or not a mapping; "
            "run robot.cache_ee_pose with the same key earlier in the queue"
        )
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side: ArmSide
    stamped_ee_side: str | None = None  # 双臂缓存时按侧取该臂 EE frame_id
    if _is_bimanual_ee_cache(raw):
        side = params.get("side")
        if side is None or str(side).strip() == "":
            side = qt.common.arm
        side = str(side).strip().lower()
        if side not in ("left", "right"):
            raise ValueError(
                "single_arm.goto_cache_pose: bimanual cache requires params.side left|right "
                "or task_cfg.common.arm"
            )
        sub = raw.get(side)
        if not isinstance(sub, Mapping):
            raise ValueError(
                f"single_arm.goto_cache_pose: bimanual cache[{key!r}][{side!r}] missing or invalid"
            )
        entry = sub
        arm_side = ArmSide.RIGHT if side == "right" else ArmSide.LEFT
        stamped_ee_side = side
    else:
        entry = raw
        arm_side, _sir, _pfx = _arm_side_and_ee_prefix(qt)

    home_pose = _scratch_pose_entry_to_geometry_pose(entry)
    gripper_override = params.get("gripper")
    if gripper_override is not None:
        grip_v = float(gripper_override)
    else:
        grip_v = float(entry.get("gripper", ctx.gripper_for_return_home))
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=home_pose,
        gripper=grip_v,
        stage_name="TaskQ-GotoCachePose",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    if ctx.use_stamped:
        if stamped_ee_side is not None:
            h = (
                ctx.interface.right_arm_handler
                if stamped_ee_side == "right"
                else ctx.interface.left_arm_handler
            )
            ee_fid = (h.frame_id if h else None) or ctx.frame_id
        else:
            ee_fid = queue_primary_ee_frame_id(ctx)
        for st in stages:
            st.frame_id = ee_fid
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ goto cache pose timeout",
    )


def skill_move_to_pose(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """笛卡尔空间运动到指定末端位姿。

    params:
        arm (str): ``left`` 或 ``right``，必填。
        position (dict | list): 目标位置，``{x, y, z}`` 或 ``[x, y, z]``（米），必填。
        orientation (dict | list): 目标姿态四元数，``{x, y, z, w}`` 或 ``[x, y, z, w]``，必填。
        motion_frame_id (str, 可选): 位姿所在的参考帧，默认 ``ctx.frame_id``（通常为 ``arm_base``）。
        gripper (float, 可选): 目标夹爪开合值，默认 ``ctx.gripper_for_return_home``（上一步结束时的夹爪状态）。
        arm_movel_duration (float, 可选): 写入 ``arm_controller.movel_duration``。
    """
    arm = str(params.get("arm", "")).strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(
            f"single_arm.move_to_pose: 'arm' must be 'left' or 'right', got {params.get('arm')!r}"
        )

    pos_raw = params.get("position")
    ori_raw = params.get("orientation")
    pose = Pose()
    if isinstance(pos_raw, Mapping):
        pose.position.x = float(pos_raw["x"])
        pose.position.y = float(pos_raw["y"])
        pose.position.z = float(pos_raw["z"])
    elif isinstance(pos_raw, (list, tuple)) and len(pos_raw) == 3:
        pose.position.x = float(pos_raw[0])
        pose.position.y = float(pos_raw[1])
        pose.position.z = float(pos_raw[2])
    else:
        raise ValueError(
            "single_arm.move_to_pose: 'position' must be {x,y,z} or [x,y,z]"
        )

    if isinstance(ori_raw, Mapping):
        pose.orientation.x = float(ori_raw["x"])
        pose.orientation.y = float(ori_raw["y"])
        pose.orientation.z = float(ori_raw["z"])
        pose.orientation.w = float(ori_raw["w"])
    elif isinstance(ori_raw, (list, tuple)) and len(ori_raw) == 4:
        # Quaternion list order is [x, y, z, w].
        pose.orientation.x = float(ori_raw[0])
        pose.orientation.y = float(ori_raw[1])
        pose.orientation.z = float(ori_raw[2])
        pose.orientation.w = float(ori_raw[3])
    else:
        raise ValueError(
            "single_arm.move_to_pose: 'orientation' must be {x,y,z,w} or [x,y,z,w]"
        )

    grip_v = float(params["gripper"]) if params.get("gripper") is not None else ctx.gripper_for_return_home
    motion_frame_id = str(
        params.get("motion_frame_id", ctx.frame_id) or ctx.frame_id
    ).strip() or ctx.frame_id

    _apply_arm_movel_duration(
        ctx, duration=params.get("arm_movel_duration"), label="single_arm.move_to_pose"
    )

    arm_side = ArmSide.RIGHT if arm == "right" else ArmSide.LEFT
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=pose,
        gripper=grip_v,
        stage_name="TaskQ-MoveToPose",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    if ctx.use_stamped:
        for st in stages:
            st.frame_id = motion_frame_id
    print(
        f"[MoveToPose] arm={arm} motion_frame_id={motion_frame_id} "
        f"pos=({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}) "
        f"ori=({pose.orientation.x:.3f},{pose.orientation.y:.3f},"
        f"{pose.orientation.z:.3f},{pose.orientation.w:.3f})"
    )
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=motion_frame_id,
        warn_prefix="TaskQ move_to_pose timeout",
    )


def skill_move_relative(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """相对当前末端位姿平移并旋转（在 ``motion_frame_id`` 下解释增量）。

    params:
        arm (str): ``left`` 或 ``right``，必填。
        position_delta (list): ``[dx, dy, dz]``（米），在 ``motion_frame_id`` 下叠加到当前位置，默认 ``[0,0,0]``。
        orientation_delta_rpy (list): ``[roll, pitch, yaw]``（弧度），左乘当前姿态四元数，默认 ``[0,0,0]``。
        motion_frame_id (str, 可选): 增量与位姿计算坐标系，默认与末端反馈 frame 一致。
        gripper (float, 可选): 目标夹爪值；**未写则不发夹爪指令**（保持当前开合）。
        arm_movel_duration (float, 可选): 写入 ``arm_controller.movel_duration``。
    """
    label = "single_arm.move_relative"
    arm = str(params.get("arm", "")).strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(f"{label}: 'arm' must be 'left' or 'right', got {params.get('arm')!r}")

    iface = ctx.interface
    handler = iface.right_arm_handler if arm == "right" else iface.left_arm_handler
    if handler is None:
        raise TypeError(f"{label}: {arm} arm handler is not available")
    base_f = str(ctx.frame_id).strip() or "base_link"
    p0, motion_frame = read_current_ee_pose_in_motion_frame(
        iface, handler, base_f, params, label=label
    )

    dx, dy, dz = param_vec3(params, "position_delta", (0.0, 0.0, 0.0))
    dr, dp, dyaw = param_vec3(params, "orientation_delta_rpy", (0.0, 0.0, 0.0))
    try:
        min_abs = float(params.get("min_abs_delta", 1e-4))
    except (TypeError, ValueError):
        min_abs = 1e-4
    try:
        min_abs_ori = float(params.get("min_abs_orientation_rpy", 1e-4))
    except (TypeError, ValueError):
        min_abs_ori = 1e-4

    move = max(abs(dx), abs(dy), abs(dz)) >= min_abs
    has_ori = max(abs(dr), abs(dp), abs(dyaw)) >= min_abs_ori
    if not (move or has_ori):
        return [], ExecutionMeta(
            send_mode=_stamped_mode(ctx),
            frame_id=motion_frame,
            warn_prefix=f"TaskQ {label} timeout",
        )

    _apply_arm_movel_duration(ctx, duration=params.get("arm_movel_duration"), label=label)

    p1 = clone_pose(p0)
    if move:
        p1.position.x += dx
        p1.position.y += dy
        p1.position.z += dz
    if has_ori:
        qd = quat_normalize(euler_rpy_to_quat_xyzw(dr, dp, dyaw))
        q_new = quat_normalize(quat_multiply(qd, pose_quat_xyzw(p1)))
        set_pose_quat_xyzw(p1, q_new)

    skip_gripper = params.get("gripper") is None
    grip_v = float(params["gripper"]) if not skip_gripper else 0.0
    arm_side = ArmSide.RIGHT if arm == "right" else ArmSide.LEFT
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=p1,
        gripper=grip_v,
        stage_name="TaskQ-MoveRelative",
        skip_gripper_command=skip_gripper,
    )
    stages = assign_to_arm(arm_seq, arm_side)
    if ctx.use_stamped:
        for st in stages:
            st.frame_id = motion_frame
    print(
        f"[MoveRelative] arm={arm} motion_frame_id={motion_frame} "
        f"delta_pos=({dx:.3f},{dy:.3f},{dz:.3f}) delta_rpy=({dr:.3f},{dp:.3f},{dyaw:.3f}) "
        f"target=({p1.position.x:.3f},{p1.position.y:.3f},{p1.position.z:.3f})"
    )
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=motion_frame,
        warn_prefix="TaskQ move_relative timeout",
    )


def register_single_arm_skills() -> None:
    register_skill("single_arm.pick", skill_pick)
    register_skill("single_arm.move_to_object", skill_move_to_object)
    register_skill("single_arm.place", skill_place)
    register_skill("single_arm.goto_cache_pose", skill_goto_cache_pose)
    register_skill("single_arm.move_to_pose", skill_move_to_pose)
    register_skill("single_arm.move_relative", skill_move_relative)


register_single_arm_skills()
