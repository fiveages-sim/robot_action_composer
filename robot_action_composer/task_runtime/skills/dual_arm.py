"""Dual-arm queue skills (``dual_arm.*``): carry + handover sync + cache goto."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmTarget,
    SendMode,
    StageTarget,
)

from robot_action_composer.isaac_sim import get_object_pose_from_service  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.tasks.bimanual_carry import (  # pyright: ignore[reportMissingImports]
    BimanualCarryTaskConfig,
    build_bimanual_carry_record_sequence,
    slice_carry_stages_for_queue,
)
from robot_action_composer.motion_generation.tasks.bimanual_parallel_pick import (  # pyright: ignore[reportMissingImports]
    BimanualParallelPickTaskConfig,
    build_bimanual_parallel_pick_record_sequence,
    parallel_pick_cfg_from_params,
    slice_parallel_pick_stages_for_queue,
)
from robot_action_composer.motion_generation.tasks.handover import build_handover_sync_sequence  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.pick_place import _apply_target_pose_offset  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.skills.single_arm import (
    _is_bimanual_ee_cache,
    _scratch_pose_entry_to_geometry_pose,
)
from robot_action_composer.task_runtime.types import ExecutionMeta


def _dual_mode(ctx: QueueRuntimeContext) -> SendMode:
    return SendMode.DUAL_ARM_STAMPED if ctx.use_stamped else SendMode.UNSTAMPED


def _require_carry(ctx: QueueRuntimeContext) -> BimanualCarryTaskConfig:
    c = ctx.carry_task_cfg
    if c is None:
        raise TypeError("dual_arm.carry* skills require carry_task_cfg (bimanual / carry fields in YAML)")
    return c


def _carry_full_stages(ctx: QueueRuntimeContext, cfg: BimanualCarryTaskConfig, object_center: Any) -> list[StageTarget]:
    return build_bimanual_carry_record_sequence(
        carry_task_cfg=cfg,
        object_center=object_center,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
    )


def _require_handover_sync(ctx: QueueRuntimeContext):
    h = ctx.handover_sync
    if h is None:
        raise TypeError(
            "dual_arm.handover_sync requires handover_sync (merged from skill_defaults.dual_arm.handover; "
            "need handover_position + orientations). Pick side arm comes from common.arm / single_arm.pick."
        )
    return h


def _require_parallel_pick_cfg(params: Mapping[str, Any]) -> BimanualParallelPickTaskConfig:
    return parallel_pick_cfg_from_params(params)


def _resolve_parallel_pick_target_poses(
    ctx: QueueRuntimeContext, cfg: BimanualParallelPickTaskConfig,
) -> dict[str, Any]:
    left_pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.left_pick.source_object_entity_path,
        include_orientation=False,
    )
    right_pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.right_pick.source_object_entity_path,
        include_orientation=False,
    )
    left_pose = _apply_target_pose_offset(left_pose, cfg.left_pick.target_pose_offset)
    right_pose = _apply_target_pose_offset(right_pose, cfg.right_pick.target_pose_offset)
    return {"left": left_pose, "right": right_pose}


def _parallel_pick_full_stages(
    ctx: QueueRuntimeContext,
    cfg: BimanualParallelPickTaskConfig,
    target_poses: Mapping[str, Any],
) -> list[StageTarget]:
    left_pose = target_poses.get("left")
    right_pose = target_poses.get("right")
    if left_pose is None or right_pose is None:
        raise ValueError("parallel pick requires both left and right target poses")
    return build_bimanual_parallel_pick_record_sequence(
        task_cfg=cfg,
        left_target_pose=left_pose,
        right_target_pose=right_pose,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
    )


def skill_carry_approach(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Approach → Forward → CloseIn（夹爪张开）。写入 ``ctx.carry_object_center`` 供后续段复用。"""
    cfg = _require_carry(ctx)
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.source_object_entity_path,
        include_orientation=False,
    )
    ctx.carry_object_center = object_center
    full = _carry_full_stages(ctx, cfg, object_center)
    approach, _g, _lr = slice_carry_stages_for_queue(full)
    return approach, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry approach timeout",
    )


def skill_carry_grasp(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Grasp（闭合）。须先于本块执行 ``dual_arm.carry_approach`` 以填充 ``ctx.carry_object_center``。"""
    cfg = _require_carry(ctx)
    oc = ctx.carry_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.source_object_entity_path,
            include_orientation=False,
        )
        ctx.carry_object_center = oc
    full = _carry_full_stages(ctx, cfg, oc)
    _a, grasp, _lr = slice_carry_stages_for_queue(full)
    return grasp, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry grasp timeout",
    )


def skill_carry_lift_retreat(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Lift → Retreat。"""
    cfg = _require_carry(ctx)
    oc = ctx.carry_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.source_object_entity_path,
            include_orientation=False,
        )
        ctx.carry_object_center = oc
    full = _carry_full_stages(ctx, cfg, oc)
    _a, _g, lift_retreat = slice_carry_stages_for_queue(full)
    return lift_retreat, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry lift/retreat timeout",
    )


def skill_carry(ctx: QueueRuntimeContext, _params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：一次执行全部 6 段（兼容旧队列；新任务推荐 ``carry_approach`` / ``grasp`` / ``lift_retreat``）。"""
    cfg = _require_carry(ctx)
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.source_object_entity_path,
        include_orientation=False,
    )
    ctx.carry_object_center = object_center
    stages = _carry_full_stages(ctx, cfg, object_center)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry timeout",
    )


def skill_parallel_pick_approach(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
    ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    approach, _g, _r = slice_parallel_pick_stages_for_queue(full)
    return approach, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick approach timeout",
    )


def skill_parallel_pick_grasp(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = ctx.parallel_pick_target_poses
    if target_poses is None:
        target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
        ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    _a, grasp, _r = slice_parallel_pick_stages_for_queue(full)
    return grasp, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick grasp timeout",
    )


def skill_parallel_pick_retreat(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = ctx.parallel_pick_target_poses
    if target_poses is None:
        target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
        ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    _a, _g, retreat = slice_parallel_pick_stages_for_queue(full)
    return retreat, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick retreat timeout",
    )


def skill_parallel_pick(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
    ctx.parallel_pick_target_poses = dict(target_poses)
    stages = _parallel_pick_full_stages(ctx, cfg, target_poses)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick timeout",
    )


def skill_goto_cache_pose(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂同步笛卡尔运动到 ``robot.cache_ee_pose``（``which: both``）写入的左右缓存位姿。"""
    key = str(params.get("key", "saved_ee_pose")).strip() or "saved_ee_pose"
    raw = ctx.scratch_get(key)
    if not isinstance(raw, Mapping) or not _is_bimanual_ee_cache(raw):
        raise ValueError(
            f"dual_arm.goto_cache_pose: cache[{key!r}] must be bimanual "
            "(use robot.cache_ee_pose with which: both)"
        )
    left_sub = raw.get("left")
    right_sub = raw.get("right")
    if not isinstance(left_sub, Mapping) or not isinstance(right_sub, Mapping):
        raise ValueError(
            f"dual_arm.goto_cache_pose: cache[{key!r}] needs valid left and right entries"
        )

    def _grip(sub: Mapping[str, Any], side_key: str) -> float:
        sk = f"{side_key}_gripper"
        if params.get(sk) is not None:
            return float(params[sk])
        if params.get("gripper") is not None:
            return float(params["gripper"])
        return float(sub.get("gripper", ctx.gripper_for_return_home))

    left_pose = _scratch_pose_entry_to_geometry_pose(left_sub)
    right_pose = _scratch_pose_entry_to_geometry_pose(right_sub)
    gl = _grip(left_sub, "left")
    gr = _grip(right_sub, "right")

    stages = [
        StageTarget(
            name=str(params.get("stage_name", "TaskQ-DualGotoCachePose")),
            left=ArmTarget(pose=left_pose, gripper=gl),
            right=ArmTarget(pose=right_pose, gripper=gr),
        )
    ]
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm goto cache pose timeout",
    )


def skill_handover_sync(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂同步交接（``build_handover_sequence``）；抓取 / 放置由 ``single_arm.pick`` / ``place`` 承担。"""
    hcfg = _require_handover_sync(ctx)
    arm = ctx.task_cfg.common.arm.strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(f"common.arm must be 'left' or 'right' for handover pick side, got {arm!r}")
    source_is_right = arm == "right"
    stages = build_handover_sync_sequence(
        sync_cfg=hcfg,
        source_is_right=source_is_right,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="Handover",
    )
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        left_arrival_guard_stage="Handover-1-SyncMove" if source_is_right else None,
        warn_prefix="TaskQ dual_arm handover sync timeout",
    )


def register_dual_arm_skills() -> None:
    register_skill("dual_arm.carry_approach", skill_carry_approach)
    register_skill("dual_arm.carry_grasp", skill_carry_grasp)
    register_skill("dual_arm.carry_lift_retreat", skill_carry_lift_retreat)
    register_skill("dual_arm.carry", skill_carry)
    register_skill("dual_arm.parallel_pick_approach", skill_parallel_pick_approach)
    register_skill("dual_arm.parallel_pick_grasp", skill_parallel_pick_grasp)
    register_skill("dual_arm.parallel_pick_retreat", skill_parallel_pick_retreat)
    register_skill("dual_arm.parallel_pick", skill_parallel_pick)
    register_skill("dual_arm.handover_sync", skill_handover_sync)
    register_skill("dual_arm.goto_cache_pose", skill_goto_cache_pose)


register_dual_arm_skills()
