"""Generic single-arm queue skills (pregrasp / pick / place / Cartesian home / MoveJ home).

单臂段使用 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice`（由扁平 preset 解析）。

``joint.movej_return_initial`` / ``single_arm.movej_return_initial`` 使用 Runner 缓存的初始关节角。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any

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
from robot_action_composer.motion_generation.tasks.movej_return import movej_return_to_initial_state  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext, queue_primary_ee_frame_id
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.config.single_arm import (
    QueueSingleArmSlice,
    QueueSlicePlace,
    overlay_queue_single_arm_from_params,
)
from robot_action_composer.task_runtime.types import ExecutionMeta


def _arm_execution_fields(
    qt: QueueSingleArmSlice,
    ctx: QueueRuntimeContext,
) -> tuple[ArmSide, bool, str, Any]:
    """按 ``qt.common.arm`` 选择工作臂、EE 前缀与 home（双臂任务用 left/right_home_pose）。"""
    arm = qt.common.arm.strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right', got {qt.common.arm!r}")
    source_is_right = arm == "right"
    arm_side = ArmSide.RIGHT if source_is_right else ArmSide.LEFT
    ee_prefix = "right_ee" if source_is_right else "left_ee"
    if ctx.left_home_pose is not None and ctx.right_home_pose is not None:
        home = ctx.right_home_pose if source_is_right else ctx.left_home_pose
    else:
        home = ctx.source_home_pose
    return arm_side, source_is_right, ee_prefix, home


def _stamped_mode(ctx: QueueRuntimeContext) -> SendMode:
    return SendMode.STAMPED if ctx.use_stamped else SendMode.UNSTAMPED


def skill_pregrasp(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side, _sir, _pfx, home = _arm_execution_fields(qt, ctx)
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
    arm_side, _source_is_right, _ee_prefix, _home = _arm_execution_fields(qt, ctx)
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
    arm_side, source_is_right, ee_prefix, _home = _arm_execution_fields(qt, ctx)
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


def skill_movej_return_initial(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """关节空间回到 Runner 连接时缓存的初始关节角（手臂 + 躯干）。"""
    rcfg = ctx.robot_cfg
    moved = movej_return_to_initial_state(
        interface=ctx.interface,
        left_initial_positions=ctx.left_initial_joint_positions,
        right_initial_positions=ctx.right_initial_joint_positions,
        body_initial_positions=ctx.body_initial_joint_positions,
        arrival_timeout=rcfg.arrival_timeout,
        arrival_poll=rcfg.arrival_poll,
        sim_time=ctx.sim_time,
    )
    if not moved:
        print("[WARN] MoveJ return-to-initial skipped: no valid cached joints or motion failed.")
    return [], ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ movej return",
    )


def skill_return_home(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    qt = overlay_queue_single_arm_from_params(ctx.task_cfg, params)
    arm_side, _sir, _pfx, home = _arm_execution_fields(qt, ctx)
    arm_seq = build_single_arm_return_home_sequence(
        home_pose=home,
        gripper=ctx.gripper_for_return_home,
        stage_name="TaskQ-ReturnHomeHold",
    )
    stages = assign_to_arm(arm_seq, arm_side)
    if ctx.use_stamped:
        ee_fid = queue_primary_ee_frame_id(ctx)
        for st in stages:
            if "ReturnHome" in st.name:
                st.frame_id = ee_fid
    return stages, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ return home timeout",
    )


def register_single_arm_skills() -> None:
    register_skill("single_arm.pregrasp", skill_pregrasp)
    register_skill("single_arm.pick", skill_pick)
    register_skill("single_arm.place", skill_place)
    register_skill("single_arm.return_home", skill_return_home)
    register_skill("single_arm.movej_return_initial", skill_movej_return_initial)
    register_skill("joint.movej_return_initial", skill_movej_return_initial)


register_single_arm_skills()
