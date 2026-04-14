#!/usr/bin/env python3
"""Bimanual carry task config and Cartesian sequence for task-queue execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_bimanual_carry_sequence,
)

# ``build_bimanual_carry_sequence`` 阶段顺序：Approach, Forward, CloseIn | Grasp | Lift, Retreat
CARRY_APPROACH_STAGE_COUNT = 3
CARRY_GRASP_STAGE_COUNT = 1
CARRY_LIFT_RETREAT_STAGE_COUNT = 2
CARRY_TOTAL_STAGES = CARRY_APPROACH_STAGE_COUNT + CARRY_GRASP_STAGE_COUNT + CARRY_LIFT_RETREAT_STAGE_COUNT


@dataclass(frozen=True)
class BimanualCarryTaskConfig:
    """双臂对称搬运几何（扁平 YAML / ``dual_arm.carry`` 合并到 ``MergedQueueConfig.carry``）。"""

    source_object_entity_path: str
    # 相对物体中心：左右手在横向（典型为 Y）上的半间距分量，与 carry_grasp_xyz[1] 相加得抓取半宽
    carry_half_span_y: float
    # 预闭合前：相对抓取点的后退/侧移/抬升（Approach→Forward 段）
    carry_pregrasp_xyz: tuple[float, float, float]
    carry_left_orientation: tuple[float, float, float, float]
    carry_right_orientation: tuple[float, float, float, float]
    carry_lift_xyz: tuple[float, float, float]
    carry_retreat_xyz: tuple[float, float, float]
    # 仅 Approach/Forward：在半间距上再张开的余量（米）
    carry_approach_clearance_y: float = 0.0
    # 物体中心到名义抓取点的平移（物体坐标系）
    carry_grasp_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    object_xyz_random_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


def format_bimanual_carry_task_cfg_summary(
    scene: str, task_cfg: BimanualCarryTaskConfig,
) -> str:
    return (
        f"[Scene] {scene} -> {task_cfg.source_object_entity_path}, "
        f"carry_half_span_y={task_cfg.carry_half_span_y}, "
        f"carry_pregrasp_xyz={task_cfg.carry_pregrasp_xyz}, "
        f"carry_lift_xyz={task_cfg.carry_lift_xyz}, carry_retreat_xyz={task_cfg.carry_retreat_xyz}"
    )


def slice_carry_stages_for_queue(full: list[StageTarget]) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    """将完整 6 段搬运序列切成队列用的三段（与 :data:`CARRY_TOTAL_STAGES` 一致）。"""
    if len(full) != CARRY_TOTAL_STAGES:
        raise ValueError(f"expected {CARRY_TOTAL_STAGES} carry stages, got {len(full)}")
    i = CARRY_APPROACH_STAGE_COUNT
    j = i + CARRY_GRASP_STAGE_COUNT
    return full[0:i], full[i:j], full[j:]


def build_bimanual_carry_record_sequence(
    *,
    carry_task_cfg: BimanualCarryTaskConfig,
    object_center: Pose,
    gripper_open: float,
    gripper_closed: float,
) -> list[StageTarget]:
    """Build the full bimanual carry sequence as StageTarget list."""
    return build_bimanual_carry_sequence(
        object_center=object_center,
        carry_half_span_y=carry_task_cfg.carry_half_span_y,
        carry_pregrasp_xyz=carry_task_cfg.carry_pregrasp_xyz,
        carry_approach_clearance_y=carry_task_cfg.carry_approach_clearance_y,
        carry_grasp_xyz=carry_task_cfg.carry_grasp_xyz,
        carry_left_orientation=carry_task_cfg.carry_left_orientation,
        carry_right_orientation=carry_task_cfg.carry_right_orientation,
        carry_lift_xyz=carry_task_cfg.carry_lift_xyz,
        carry_retreat_xyz=carry_task_cfg.carry_retreat_xyz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
    )
