#!/usr/bin/env python3
"""Bimanual parallel pick config and synchronized Cartesian sequence builders."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_single_arm_pick_sequence,
    compose_bimanual_synchronized_sequence,
)
from robot_action_composer.task_runtime.merge.flat_presets import kwargs_for_dataclass  # pyright: ignore[reportMissingImports]

PARALLEL_PICK_APPROACH_STAGE_COUNT = 2
PARALLEL_PICK_GRASP_STAGE_COUNT = 1
PARALLEL_PICK_RETREAT_STAGE_COUNT = 2
PARALLEL_PICK_TOTAL_STAGES = (
    PARALLEL_PICK_APPROACH_STAGE_COUNT
    + PARALLEL_PICK_GRASP_STAGE_COUNT
    + PARALLEL_PICK_RETREAT_STAGE_COUNT
)


@dataclass(frozen=True)
class ParallelPickArmConfig:
    object_prim_path: str = ""
    object_xyz_random_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_pose_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    approach_clearance: float = 0.2
    grasp_clearance: float = 0.01
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    retreat_direction_extra: float = 0.0
    retreat_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    retreat_xyz: tuple[float, float, float] | None = None
    ee_base_orientation: tuple[float, float, float, float] = (-0.7, 0.7, 0.0, 0.0)
    grasp_direction: str = "top"
    grasp_direction_vector: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class BimanualParallelPickTaskConfig:
    left_pick: ParallelPickArmConfig
    right_pick: ParallelPickArmConfig


def parallel_pick_cfg_from_params(params: Mapping[str, Any]) -> BimanualParallelPickTaskConfig:
    left_raw = params.get("left_pick")
    right_raw = params.get("right_pick")
    if not isinstance(left_raw, Mapping) or not isinstance(right_raw, Mapping):
        raise ValueError(
            "dual_arm.parallel_pick* requires params.left_pick and params.right_pick mappings"
        )
    left = ParallelPickArmConfig(**kwargs_for_dataclass(ParallelPickArmConfig, dict(left_raw)))
    right = ParallelPickArmConfig(**kwargs_for_dataclass(ParallelPickArmConfig, dict(right_raw)))
    if not left.object_prim_path or not right.object_prim_path:
        raise ValueError("left_pick.object_prim_path and right_pick.object_prim_path are required")
    return BimanualParallelPickTaskConfig(left_pick=left, right_pick=right)


def slice_parallel_pick_stages_for_queue(
    full: list[StageTarget],
) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    if len(full) != PARALLEL_PICK_TOTAL_STAGES:
        raise ValueError(f"expected {PARALLEL_PICK_TOTAL_STAGES} parallel-pick stages, got {len(full)}")
    i = PARALLEL_PICK_APPROACH_STAGE_COUNT
    j = i + PARALLEL_PICK_GRASP_STAGE_COUNT
    return full[0:i], full[i:j], full[j:]


def build_bimanual_parallel_pick_record_sequence(
    *,
    task_cfg: BimanualParallelPickTaskConfig,
    left_target_pose: Pose,
    right_target_pose: Pose,
    gripper_open: float,
    gripper_closed: float,
) -> list[StageTarget]:
    left = task_cfg.left_pick
    right = task_cfg.right_pick
    left_seq = build_single_arm_pick_sequence(
        target_pose=left_target_pose,
        ee_base_orientation=left.ee_base_orientation,
        prepare_offset=(0.0, 0.0, left.approach_clearance),
        pick_clearance=left.grasp_clearance,
        grasp_direction=left.grasp_direction,
        grasp_direction_vector=left.grasp_direction_vector,
        object_position_offset=left.grasp_offset,
        retreat_direction_extra=left.retreat_direction_extra,
        retreat_offset=left.retreat_offset,
        retreat_xyz=left.retreat_xyz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickL",
    )
    right_seq = build_single_arm_pick_sequence(
        target_pose=right_target_pose,
        ee_base_orientation=right.ee_base_orientation,
        prepare_offset=(0.0, 0.0, right.approach_clearance),
        pick_clearance=right.grasp_clearance,
        grasp_direction=right.grasp_direction,
        grasp_direction_vector=right.grasp_direction_vector,
        object_position_offset=right.grasp_offset,
        retreat_direction_extra=right.retreat_direction_extra,
        retreat_offset=right.retreat_offset,
        retreat_xyz=right.retreat_xyz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickR",
    )
    return compose_bimanual_synchronized_sequence(left_seq, right_seq)
