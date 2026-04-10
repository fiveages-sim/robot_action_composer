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
    source_object_entity_path: str
    lateral_offset: float
    approach_offset: tuple[float, float, float]
    left_orientation: tuple[float, float, float, float]
    right_orientation: tuple[float, float, float, float]
    lift_offset: tuple[float, float, float]
    retreat_offset: tuple[float, float, float]
    lateral_clearance: float = 0.0
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    object_xyz_random_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


def format_bimanual_carry_task_cfg_summary(
    scene: str, task_cfg: BimanualCarryTaskConfig,
) -> str:
    return (
        f"[Scene] {scene} -> {task_cfg.source_object_entity_path}, "
        f"lateral_offset={task_cfg.lateral_offset}, "
        f"approach_offset={task_cfg.approach_offset}, "
        f"lift_offset={task_cfg.lift_offset}, retreat_offset={task_cfg.retreat_offset}"
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
        lateral_offset=carry_task_cfg.lateral_offset,
        approach_offset=carry_task_cfg.approach_offset,
        lateral_clearance=carry_task_cfg.lateral_clearance,
        grasp_offset=carry_task_cfg.grasp_offset,
        left_orientation=carry_task_cfg.left_orientation,
        right_orientation=carry_task_cfg.right_orientation,
        lift_offset=carry_task_cfg.lift_offset,
        retreat_offset=carry_task_cfg.retreat_offset,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
    )
