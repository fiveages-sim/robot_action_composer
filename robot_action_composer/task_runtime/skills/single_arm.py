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
    pose_from_tuple,
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

from robot_action_composer.isaac_sim import get_object_pose_from_service  # pyright: ignore[reportMissingImports]

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
    ee_pick_axis = pk.ee_pick_axis
    ee_pick_direction_vector = pk.ee_pick_direction_vector
    motion_frame_id = pk.motion_frame_id
    tf_lookup_timeout = pk.tf_lookup_timeout
    arm_movel_duration = pk.arm_movel_duration
    retreat_offset = (
        rotate_vector_by_quat(pk.ee_lift_offset, ee_base_orientation)
        if pk.ee_lift_offset is not None
        else (0.0, 0.0, 0.0)
    )
    retreat_xyz = (
        rotate_vector_by_quat(pk.ee_retreat_offset, ee_base_orientation)
        if pk.ee_retreat_offset is not None
        else None
    )
    if not path:
        raise ValueError("object_prim_path is required for skill single_arm.pick")
    _apply_arm_movel_duration(ctx, duration=arm_movel_duration, label="single_arm.pick")
    exec_f = _pick_execution_frame_id(ctx, motion_frame_id)
    target = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path,
        include_orientation=True,
    )
    if exec_f != ctx.frame_id:
        iface = ctx.interface
        if not hasattr(iface, "transform_pose"):
            raise TypeError("single_arm.pick: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)")
        transformed = iface.transform_pose(
            target,
            str(ctx.frame_id),
            exec_f,
            timeout=_pick_tf_timeout(tf_lookup_timeout),
        )
        if transformed is None:
            raise RuntimeError(
                f"single_arm.pick: TF {ctx.frame_id!r} -> {exec_f!r} failed for object pose; "
                "check motion_frame_id and TF."
            )
        target = transformed
    target = apply_object_local_offset_to_pose(target, object_position_offset)
    arm_seq = build_single_arm_pick_sequence(
        target_pose=target,
        ee_base_orientation=ee_base_orientation,
        prepare_offset=prepare_offset,
        pick_clearance=pick_clearance,
        ee_pick_axis=ee_pick_axis,
        ee_pick_direction_vector=ee_pick_direction_vector,
        retreat_offset=retreat_offset,
        retreat_xyz=retreat_xyz,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="TaskQ-Pick",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.task_cfg = replace(ctx.task_cfg, common=qt.common, pick=pk)
    ctx.gripper_for_return_home = ctx.gripper_closed
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ pick timeout",
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
    arm_seq = build_single_arm_place_sequence(
        place_position=pl2.place_position,
        place_orientation=pl2.ee_base_orientation,
        place_axis=(pl2.ee_place_axis if pl2.ee_place_axis is not None else qt.pick.ee_pick_axis),
        prepare_offset=pl2.prepare_offset,
        place_insert_clearance=pl2.place_insert_clearance,
        ee_retreat_offset=pl2.ee_retreat_offset,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="TaskQ-Place",
        start_index=1,
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.gripper_for_return_home = ctx.gripper_open
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
    stage_name = str(params.get("stage_name", "TaskQ-GotoCachePose"))
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=home_pose,
        gripper=grip_v,
        stage_name=stage_name,
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


def register_single_arm_skills() -> None:
    register_skill("single_arm.pick", skill_pick)
    register_skill("single_arm.place", skill_place)
    register_skill("single_arm.goto_cache_pose", skill_goto_cache_pose)


register_single_arm_skills()
