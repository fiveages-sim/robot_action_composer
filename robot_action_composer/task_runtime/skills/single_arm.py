"""Generic single-arm queue skills (pregrasp / pick / place / Cartesian cache goto).

单臂段使用 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice`（由扁平 preset 解析）。
笛卡尔回程用 ``robot.cache_ee_pose`` + ``single_arm.goto_cache_pose``。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmStage,
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
    _apply_target_pose_offset,
    resolve_place_skill_from_entity,
)
from robot_action_composer.ros_interface_utils import arm_handler_pose_or_raise  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext, queue_primary_ee_frame_id
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.config.single_arm import (
    QueueSingleArmSlice,
    QueueSlicePlace,
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


def skill_pregrasp(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side, source_is_right, ee_prefix = _arm_side_and_ee_prefix(qt)
    handler = ctx.interface.right_arm_handler if source_is_right else ctx.interface.left_arm_handler
    home = arm_handler_pose_or_raise(handler, label=ee_prefix)
    pregrasp_target = ArmTarget(pose=home, gripper=ctx.gripper_open)
    stages = assign_to_arm([ArmStage("TaskQ-pregrasp", pregrasp_target)], arm_side)
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=queue_primary_ee_frame_id(ctx),
        warn_prefix="TaskQ pregrasp timeout",
    )


def skill_pick(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(f"single_arm.pick expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}")
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side, _source_is_right, _ee_prefix = _arm_side_and_ee_prefix(qt)
    pk = qt.pick
    path = pk.source_object_entity_path
    target_pose_offset = pk.target_pose_offset
    approach_clearance = pk.approach_clearance
    grasp_clearance = pk.grasp_clearance
    grasp_orientation = pk.grasp_orientation
    grasp_direction = pk.grasp_direction
    grasp_direction_vector = pk.grasp_direction_vector
    grasp_offset = pk.grasp_offset
    retreat_direction_extra = pk.retreat_direction_extra
    retreat_offset = pk.retreat_offset
    if not path:
        raise ValueError("source_object_entity_path is required for skill single_arm.pick")
    target = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path,
        include_orientation=False,
    )
    target = _apply_target_pose_offset(target, target_pose_offset)
    arm_seq = build_single_arm_pick_sequence(
        target_pose=target,
        approach_clearance=approach_clearance,
        grasp_clearance=grasp_clearance,
        grasp_orientation=grasp_orientation,
        grasp_direction=grasp_direction,
        grasp_direction_vector=grasp_direction_vector,
        grasp_offset=grasp_offset,
        retreat_direction_extra=retreat_direction_extra,
        retreat_offset=retreat_offset,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="TaskQ-Pick",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.task_cfg = replace(ctx.task_cfg, pick=pk)
    ctx.gripper_for_return_home = ctx.gripper_closed
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ pick timeout",
    )


def skill_place(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    if not isinstance(ctx.task_cfg, QueueSingleArmSlice):
        raise TypeError(f"single_arm.place expects QueueSingleArmSlice on ctx.task_cfg, got {type(ctx.task_cfg)}")
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side, source_is_right, ee_prefix = _arm_side_and_ee_prefix(qt)
    pl = qt.place
    if not pl.run_place_before_return:
        ctx.task_cfg = replace(ctx.task_cfg, place=pl)
        return [], ExecutionMeta(
            send_mode=_stamped_mode(ctx),
            frame_id=ctx.frame_id,
            warn_prefix="TaskQ place skipped",
        )
    pl2 = resolve_place_skill_from_entity(
        pl,
        base_world_pos=ctx.base_world_pos,
        base_world_quat=ctx.base_world_quat,
        current_obs=None,
        ee_prefix_for_orientation_fallback=ee_prefix,
    )
    if pl2.place_orientation is None:
        h = ctx.interface.right_arm_handler if source_is_right else ctx.interface.left_arm_handler
        pose = h.get_pose() if h else None
        if pose is not None:
            pl2 = replace(
                pl2,
                place_orientation=(
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ),
            )
    if pl2.place_position is None or pl2.place_orientation is None:
        raise ValueError(
            "place_position and place_orientation are required when run_place_before_return=True "
            "(set via skill params, place_object_entity_path, or skill_defaults.single_arm.place)"
        )
    ctx.task_cfg = replace(ctx.task_cfg, place=pl2)
    arm_seq = build_single_arm_place_sequence(
        place_position=pl2.place_position,
        place_orientation=pl2.place_orientation,
        place_direction=pl2.place_direction,
        place_direction_vector=pl2.place_direction_vector,
        place_approach_clearance=pl2.place_approach_clearance,
        place_insert_clearance=pl2.place_insert_clearance,
        post_release_retract_offset=pl2.post_release_retract_offset,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="TaskQ-Place",
        start_index=1,
    )
    stages = assign_to_arm(arm_seq, arm_side)
    ctx.gripper_for_return_home = ctx.gripper_open
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
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
    register_skill("single_arm.pregrasp", skill_pregrasp)
    register_skill("single_arm.pick", skill_pick)
    register_skill("single_arm.place", skill_place)
    register_skill("single_arm.goto_cache_pose", skill_goto_cache_pose)


register_single_arm_skills()
