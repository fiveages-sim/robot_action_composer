#!/usr/bin/env python3
"""Bimanual carry task config and Cartesian sequence for task-queue execution."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_bimanual_carry_sequence,
    carry_prepare_offset_is_active,
)

# ``build_bimanual_carry_sequence``：可选 Approach + Forward + CloseIn | Grasp | 可选 Lift / Retreat
CARRY_APPROACH_STAGE_COUNT_MAX = 3
CARRY_GRASP_STAGE_COUNT = 1
CARRY_TAIL_STAGE_COUNT_MAX = 2
CARRY_TOTAL_STAGES_MAX = (
    CARRY_APPROACH_STAGE_COUNT_MAX + CARRY_GRASP_STAGE_COUNT + CARRY_TAIL_STAGE_COUNT_MAX
)


@dataclass(frozen=True)
class BimanualCarryTaskConfig:
    """双臂对称搬运几何（扁平 YAML / ``dual_arm.carry`` 合并到 ``MergedQueueConfig.carry``）。"""

    source_object_entity_path: str
    # 相对物体中心：左右手在横向（典型为 Y）上的半间距分量，与 carry_xyz[1] 相加得抓取半宽
    carry_half_span_y: float
    carry_left_orientation: tuple[float, float, float, float]
    carry_right_orientation: tuple[float, float, float, float]
    # 预闭合前：相对抓取点的后退/侧移/抬升（独立 Approach 段）；不写 / 全 0 则省略 Approach
    carry_prepare_offset: tuple[float, float, float] | None = None
    # 仅 Approach/Forward：在半间距上再张开的余量（米）
    carry_approach_clearance_y: float = 0.0
    # 物体中心到名义抓取点的平移（物体坐标系）
    carry_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 不写 / ``null``：不生成该段；另一段仍按 (0,0,0) 参与合成其终点
    carry_lift_xyz: tuple[float, float, float] | None = None
    carry_retreat_xyz: tuple[float, float, float] | None = None
    # 可选：双臂 movel 时长，执行前写入 ``arm_controller.movel_duration``（类比 ``body_movej_duration``）
    arm_movel_duration: float | None = None


def carry_approach_stage_count(cfg: BimanualCarryTaskConfig) -> int:
    """无预接近偏移时为 2（Forward+CloseIn）；否则为 3（Approach+Forward+CloseIn）。"""
    return 3 if carry_prepare_offset_is_active(cfg.carry_prepare_offset) else 2


def format_bimanual_carry_task_cfg_summary(
    scene: str, task_cfg: BimanualCarryTaskConfig,
) -> str:
    return (
        f"[Scene] {scene} -> {task_cfg.source_object_entity_path}, "
        f"carry_half_span_y={task_cfg.carry_half_span_y}, "
        f"carry_prepare_offset={task_cfg.carry_prepare_offset}, "
        f"carry_lift_xyz={task_cfg.carry_lift_xyz!r}, carry_retreat_xyz={task_cfg.carry_retreat_xyz!r}, "
        f"arm_movel_duration={task_cfg.arm_movel_duration!r}"
    )


def slice_carry_stages_for_queue(
    full: list[StageTarget],
    *,
    approach_stage_count: int,
) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    """前 ``approach_stage_count`` + 1 段为 approach / grasp；余下 0～2 段为可选 lift / retreat。"""
    if approach_stage_count not in (2, 3):
        raise ValueError(f"approach_stage_count must be 2 or 3, got {approach_stage_count}")
    i = approach_stage_count
    j = i + CARRY_GRASP_STAGE_COUNT
    if len(full) < j:
        raise ValueError(f"carry sequence too short: need >= {j} stages, got {len(full)}")
    tail = len(full) - j
    if tail > CARRY_TAIL_STAGE_COUNT_MAX:
        raise ValueError(f"carry tail too long: at most {CARRY_TAIL_STAGE_COUNT_MAX} lift/retreat, got {tail}")
    return full[0:i], full[i:j], full[j:]


def build_bimanual_carry_record_sequence(
    *,
    carry_task_cfg: BimanualCarryTaskConfig,
    object_center: Pose,
    gripper_open: float,
    gripper_closed: float,
    output_frame_id: str | None = None,
) -> list[StageTarget]:
    """Build the full bimanual carry sequence as StageTarget list."""
    return build_bimanual_carry_sequence(
        object_center=object_center,
        carry_half_span_y=carry_task_cfg.carry_half_span_y,
        carry_prepare_offset=carry_task_cfg.carry_prepare_offset,
        carry_approach_clearance_y=carry_task_cfg.carry_approach_clearance_y,
        carry_xyz=carry_task_cfg.carry_xyz,
        carry_left_orientation=carry_task_cfg.carry_left_orientation,
        carry_right_orientation=carry_task_cfg.carry_right_orientation,
        carry_lift_xyz=carry_task_cfg.carry_lift_xyz,
        carry_retreat_xyz=carry_task_cfg.carry_retreat_xyz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        output_frame_id=output_frame_id,
    )
