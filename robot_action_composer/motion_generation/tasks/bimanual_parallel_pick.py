#!/usr/bin/env python3
"""Bimanual parallel pick config and synchronized Cartesian sequence builders."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import Pose

from ros2_robot_interface.utils.quat_pose import rotate_vector_by_quat  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_single_arm_pick_sequence,
    compose_bimanual_synchronized_sequence,
)
from robot_action_composer.task_runtime.merge.utils import kwargs_for_dataclass  # pyright: ignore[reportMissingImports]

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
    """单侧抓取几何，与 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSlicePick` 字段语义一致。"""

    object_prim_path: str = ""
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    prepare_offset: tuple[float, float, float] | None = None
    pick_clearance: float = 0.01
    ee_lift_offset: tuple[float, float, float] | None = None
    ee_retreat_offset: tuple[float, float, float] | None = None
    ee_base_orientation: tuple[float, float, float, float] = (-0.7, 0.7, 0.0, 0.0)
    ee_pick_axis: str = "+z"
    ee_pick_direction_vector: tuple[float, float, float] | None = None
    motion_frame_id: str | None = None
    tf_lookup_timeout: float | None = None
    arm_movel_duration: float | None = None


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
    left = ParallelPickArmConfig(**kwargs_for_dataclass(ParallelPickArmConfig, left_raw))
    right = ParallelPickArmConfig(**kwargs_for_dataclass(ParallelPickArmConfig, right_raw))
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


def _pick_retreat_world_vectors(
    arm: ParallelPickArmConfig,
) -> tuple[tuple[float, float, float], tuple[float, float, float] | None]:
    """与 ``single_arm.skill_pick`` 一致：工具系抬升/后撤先按 ``ee_base_orientation`` 旋到世界系再交给序列构建。"""
    q = arm.ee_base_orientation
    retreat_offset = (
        rotate_vector_by_quat(arm.ee_lift_offset, q) if arm.ee_lift_offset is not None else (0.0, 0.0, 0.0)
    )
    retreat_xyz = (
        rotate_vector_by_quat(arm.ee_retreat_offset, q) if arm.ee_retreat_offset is not None else None
    )
    return retreat_offset, retreat_xyz


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
    lo, lz = _pick_retreat_world_vectors(left)
    ro, rz = _pick_retreat_world_vectors(right)
    left_seq = build_single_arm_pick_sequence(
        target_pose=left_target_pose,
        ee_base_orientation=left.ee_base_orientation,
        prepare_offset=left.prepare_offset,
        pick_clearance=left.pick_clearance,
        ee_pick_axis=left.ee_pick_axis,
        ee_pick_direction_vector=left.ee_pick_direction_vector,
        object_position_offset=(0.0, 0.0, 0.0),
        retreat_offset=lo,
        retreat_xyz=lz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickL",
    )
    right_seq = build_single_arm_pick_sequence(
        target_pose=right_target_pose,
        ee_base_orientation=right.ee_base_orientation,
        prepare_offset=right.prepare_offset,
        pick_clearance=right.pick_clearance,
        ee_pick_axis=right.ee_pick_axis,
        ee_pick_direction_vector=right.ee_pick_direction_vector,
        object_position_offset=(0.0, 0.0, 0.0),
        retreat_offset=ro,
        retreat_xyz=rz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickR",
    )
    return compose_bimanual_synchronized_sequence(left_seq, right_seq)
