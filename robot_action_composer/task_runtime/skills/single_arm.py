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
from robot_action_composer.task_runtime.object_resolution_replay import resolve_object_pose_for_task

from robot_action_composer.motion_generation.tasks.pick_place import (  # pyright: ignore[reportMissingImports]
    apply_object_local_offset_to_pose,
    resolve_place_skill_from_entity,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext, queue_primary_ee_frame_id
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


def _resolve_object_target_pose_from_pick_like_params(
    *,
    ctx: QueueRuntimeContext,
    object_prim_path: str,
    object_position_offset: tuple[float, float, float],
    motion_frame_id: Any,
    tf_lookup_timeout: Any,
    label: str,
) -> tuple[Pose, str]:
    """Resolve object pose (+ local offset) into execution frame, mirroring ``single_arm.pick``."""
    exec_f = _pick_execution_frame_id(ctx, motion_frame_id)
    arm_side = ctx.task_cfg.common.arm.strip().lower()
    object_role = "pick_target" if label == "single_arm.pick" else "move_to_object_target"
    if ctx.object_resolution is not None:
        target = resolve_object_pose_for_task(
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
        target = get_object_pose_from_service(
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
            target,
            str(ctx.frame_id),
            exec_f,
            timeout=_pick_tf_timeout(tf_lookup_timeout),
        )
        if transformed is None:
            raise RuntimeError(
                f"{label}: TF {ctx.frame_id!r} -> {exec_f!r} failed for object pose; "
                "check motion_frame_id and TF."
            )
        target = transformed
    target = apply_object_local_offset_to_pose(target, object_position_offset)
    return target, exec_f


def skill_pick(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(f"single_arm.pick expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}")
    qt = overlay_queue_single_arm_pick_from_params(ctx.task_cfg, params)
    arm_side, _source_is_right, _ee_prefix = _arm_side_and_ee_prefix(qt)
    pk = qt.pick
    path = pk.object_prim_path
    object_position_offset = pk.object_position_offset
    prepare_offset = pk.prepare_offset
    pick_clearance = pk.pick_clearance
    ee_base_orientation = pk.ee_base_orientation
    motion_frame_id = pk.motion_frame_id
    tf_lookup_timeout = pk.tf_lookup_timeout
    arm_movel_duration = pk.arm_movel_duration
    retreat_offset = (
        rotate_vector_by_quat(pk.ee_lift_offset, ee_base_orientation)
        if pk.ee_lift_offset is not None
        else None
    )
    retreat_xyz = (
        rotate_vector_by_quat(pk.ee_retreat_offset, ee_base_orientation)
        if pk.ee_retreat_offset is not None
        else None
    )
    if not path:
        raise ValueError("object_prim_path is required for skill single_arm.pick")
    _apply_arm_movel_duration(ctx, duration=arm_movel_duration, label="single_arm.pick")
    gripper_open = float(ctx.gripper_open)
    gripper_closed = (
        float(pk.gripper_closed) if pk.gripper_closed is not None else float(ctx.gripper_closed)
    )
    target, exec_f = _resolve_object_target_pose_from_pick_like_params(
        ctx=ctx,
        object_prim_path=path,
        object_position_offset=object_position_offset,
        motion_frame_id=motion_frame_id,
        tf_lookup_timeout=tf_lookup_timeout,
        label="single_arm.pick",
    )
    arm_seq = build_single_arm_pick_sequence(
        target_pose=target,
        ee_base_orientation=ee_base_orientation,
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
    target, exec_f = _resolve_object_target_pose_from_pick_like_params(
        ctx=ctx,
        object_prim_path=pk.object_prim_path,
        object_position_offset=pk.object_position_offset,
        motion_frame_id=pk.motion_frame_id,
        tf_lookup_timeout=pk.tf_lookup_timeout,
        label="single_arm.move_to_object",
    )

    # Target orientation is explicitly driven by pick-like ee_base_orientation.
    target.orientation.x = float(pk.ee_base_orientation[0])
    target.orientation.y = float(pk.ee_base_orientation[1])
    target.orientation.z = float(pk.ee_base_orientation[2])
    target.orientation.w = float(pk.ee_base_orientation[3])

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
    pl2 = resolve_place_skill_from_entity(
        pl,
        base_world_pos=ctx.base_world_pos,
        base_world_quat=ctx.base_world_quat,
        ctx=ctx,
    )
    if pl2.ee_base_orientation is None:
        pl2 = replace(pl2, ee_base_orientation=qt.pick.ee_base_orientation)
    if pl2.place_position is None:
        raise ValueError(
            "place_position is required for single_arm.place "
            "(set via skill params, object_prim_path, or skill_defaults.single_arm.place)"
        )
    if pl2.ee_base_orientation is None:
        raise ValueError("ee_base_orientation is required for single_arm.place")
    _apply_arm_movel_duration(ctx, duration=pl2.arm_movel_duration, label="single_arm.place")
    exec_f = _pick_execution_frame_id(ctx, pl2.motion_frame_id)
    if exec_f != ctx.frame_id:
        iface = ctx.interface
        if not hasattr(iface, "transform_pose"):
            raise TypeError("single_arm.place: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)")
        pose_in = pose_from_tuple(pl2.place_position, pl2.ee_base_orientation)
        transformed = iface.transform_pose(
            pose_in,
            str(ctx.frame_id),
            exec_f,
            timeout=_pick_tf_timeout(pl2.tf_lookup_timeout),
        )
        if transformed is None:
            raise RuntimeError(
                f"single_arm.place: TF {ctx.frame_id!r} -> {exec_f!r} failed for place pose; "
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


def _clone_pose(p: Pose) -> Pose:
    q = Pose()
    q.position.x = p.position.x
    q.position.y = p.position.y
    q.position.z = p.position.z
    q.orientation.x = p.orientation.x
    q.orientation.y = p.orientation.y
    q.orientation.z = p.orientation.z
    q.orientation.w = p.orientation.w
    return q


def _param_vec3(
    params: Mapping[str, Any],
    key: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    v = params.get(key)
    if v is None:
        return default
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"{key} must be a length-3 list [x, y, z]")
    return (float(v[0]), float(v[1]), float(v[2]))


def _pose_quat_xyzw(p: Pose) -> tuple[float, float, float, float]:
    return (
        float(p.orientation.x),
        float(p.orientation.y),
        float(p.orientation.z),
        float(p.orientation.w),
    )


def _set_pose_quat_xyzw(p: Pose, q: tuple[float, float, float, float]) -> None:
    x, y, z, w = q
    p.orientation.x = x
    p.orientation.y = y
    p.orientation.z = z
    p.orientation.w = w


def _single_arm_pose_in_motion_frame(
    iface: Any,
    pose_raw: Pose,
    pose_frame: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[Pose, str]:
    raw_mf = params.get("motion_frame_id")
    if raw_mf is None or (isinstance(raw_mf, str) and not str(raw_mf).strip()):
        motion_frame = pose_frame
    else:
        motion_frame = str(raw_mf).strip()
    try:
        tft = float(params.get("tf_lookup_timeout", 2.0))
    except (TypeError, ValueError):
        tft = 2.0
    if motion_frame == pose_frame:
        return _clone_pose(pose_raw), motion_frame
    if not hasattr(iface, "transform_pose"):
        raise TypeError(f"{label}: motion_frame_id requires ROS2RobotInterface.transform_pose (TF buffer)")
    out = iface.transform_pose(pose_raw, pose_frame, motion_frame, timeout=tft)
    if out is None:
        raise RuntimeError(
            f"{label}: TF {pose_frame!r} -> {motion_frame!r} failed (timeout={tft}s)."
        )
    return out, motion_frame


def skill_move_relative(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """相对当前末端位姿平移并旋转（在 ``motion_frame_id`` 下解释增量）。

    params:
        arm (str): ``left`` 或 ``right``，必填。
        position_delta (list): ``[dx, dy, dz]``（米），在 ``motion_frame_id`` 下叠加到当前位置，默认 ``[0,0,0]``。
        orientation_delta_rpy (list): ``[roll, pitch, yaw]``（弧度），左乘当前姿态四元数，默认 ``[0,0,0]``。
        motion_frame_id (str, 可选): 增量与位姿计算坐标系，默认与末端反馈 frame 一致。
        gripper (float, 可选): 目标夹爪值，默认 ``ctx.gripper_for_return_home``。
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
    p0_raw = handler.get_pose()
    if p0_raw is None:
        raise RuntimeError(f"{label}: could not read current {arm} EE pose")

    gf = getattr(handler, "get_frame_id", None)
    pose_frame = str(gf()).strip() if callable(gf) and gf() else str(ctx.frame_id).strip() or "base_link"
    p0, motion_frame = _single_arm_pose_in_motion_frame(iface, p0_raw, pose_frame, params, label=label)

    dx, dy, dz = _param_vec3(params, "position_delta", (0.0, 0.0, 0.0))
    dr, dp, dyaw = _param_vec3(params, "orientation_delta_rpy", (0.0, 0.0, 0.0))
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

    p1 = _clone_pose(p0)
    if move:
        p1.position.x += dx
        p1.position.y += dy
        p1.position.z += dz
    if has_ori:
        qd = quat_normalize(euler_rpy_to_quat_xyzw(dr, dp, dyaw))
        q_new = quat_normalize(quat_multiply(qd, _pose_quat_xyzw(p1)))
        _set_pose_quat_xyzw(p1, q_new)

    grip_v = float(params["gripper"]) if params.get("gripper") is not None else ctx.gripper_for_return_home
    arm_side = ArmSide.RIGHT if arm == "right" else ArmSide.LEFT
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=p1,
        gripper=grip_v,
        stage_name="TaskQ-MoveRelative",
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
