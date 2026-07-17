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
from robot_action_composer.motion_generation.tasks.object_orientation import (  # pyright: ignore[reportMissingImports]
    compose_aligned_ee_orientation,
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
    # 标定语义：ee_base / 工具系偏移按标称机物相对朝向标定；object 时按相对偏差左乘。
    ee_orientation_frame: str | None = None
    object_orientation_mode: str | None = None
    aligned_object_yaw: float | str | None = None


@dataclass(frozen=True)
class BimanualParallelPickTaskConfig:
    left_pick: ParallelPickArmConfig
    right_pick: ParallelPickArmConfig
    # 两侧共用的物体朝向策略（可被 left_pick/right_pick 同名字段覆盖）
    ee_orientation_frame: str = "motion"
    object_orientation_mode: str = "yaw"
    # 标称物体 yaw：数值，或 "auto"（吸附到最近的 k·π/2：0 / ±1.57 / ±3.14）
    aligned_object_yaw: float | str = "auto"


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
    top_frame = str(params.get("ee_orientation_frame", "motion") or "motion")
    top_mode = str(params.get("object_orientation_mode", "yaw") or "yaw")
    raw_yaw = params.get("aligned_object_yaw", "auto")
    if raw_yaw is None or (isinstance(raw_yaw, str) and not str(raw_yaw).strip()):
        top_yaw: float | str = "auto"
    elif isinstance(raw_yaw, str):
        top_yaw = raw_yaw.strip()
    else:
        top_yaw = float(raw_yaw)
    return BimanualParallelPickTaskConfig(
        left_pick=left,
        right_pick=right,
        ee_orientation_frame=top_frame,
        object_orientation_mode=top_mode,
        aligned_object_yaw=top_yaw,
    )


def slice_parallel_pick_stages_for_queue(
    full: list[StageTarget],
) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    if len(full) != PARALLEL_PICK_TOTAL_STAGES:
        raise ValueError(f"expected {PARALLEL_PICK_TOTAL_STAGES} parallel-pick stages, got {len(full)}")
    i = PARALLEL_PICK_APPROACH_STAGE_COUNT
    j = i + PARALLEL_PICK_GRASP_STAGE_COUNT
    return full[0:i], full[i:j], full[j:]


def _arm_orientation_policy(
    task_cfg: BimanualParallelPickTaskConfig,
    arm: ParallelPickArmConfig,
) -> tuple[str, str, float | str]:
    frame = arm.ee_orientation_frame if arm.ee_orientation_frame is not None else task_cfg.ee_orientation_frame
    mode = (
        arm.object_orientation_mode
        if arm.object_orientation_mode is not None
        else task_cfg.object_orientation_mode
    )
    yaw0: float | str = (
        arm.aligned_object_yaw
        if arm.aligned_object_yaw is not None
        else task_cfg.aligned_object_yaw
    )
    return str(frame or "motion"), str(mode or "yaw"), yaw0


def _pick_retreat_world_vectors(
    arm: ParallelPickArmConfig,
    ee_orientation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float] | None, tuple[float, float, float] | None]:
    """工具系抬升/后撤按**有效**末端姿态旋到运动系（与 single_arm.pick 一致）。"""
    retreat_offset = (
        rotate_vector_by_quat(arm.ee_lift_offset, ee_orientation) if arm.ee_lift_offset is not None else None
    )
    retreat_xyz = (
        rotate_vector_by_quat(arm.ee_retreat_offset, ee_orientation)
        if arm.ee_retreat_offset is not None
        else None
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
    l_frame, l_mode, l_yaw0 = _arm_orientation_policy(task_cfg, left)
    r_frame, r_mode, r_yaw0 = _arm_orientation_policy(task_cfg, right)
    left_ee = compose_aligned_ee_orientation(
        left.ee_base_orientation,
        left_target_pose,
        ee_orientation_frame=l_frame,
        object_orientation_mode=l_mode,
        aligned_object_yaw=l_yaw0,
    )
    right_ee = compose_aligned_ee_orientation(
        right.ee_base_orientation,
        right_target_pose,
        ee_orientation_frame=r_frame,
        object_orientation_mode=r_mode,
        aligned_object_yaw=r_yaw0,
    )
    lo, lz = _pick_retreat_world_vectors(left, left_ee)
    ro, rz = _pick_retreat_world_vectors(right, right_ee)
    left_seq = build_single_arm_pick_sequence(
        target_pose=left_target_pose,
        ee_base_orientation=left_ee,
        prepare_offset=left.prepare_offset,
        pick_clearance=left.pick_clearance,
        object_position_offset=(0.0, 0.0, 0.0),
        retreat_offset=lo,
        retreat_xyz=lz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickL",
    )
    right_seq = build_single_arm_pick_sequence(
        target_pose=right_target_pose,
        ee_base_orientation=right_ee,
        prepare_offset=right.prepare_offset,
        pick_clearance=right.pick_clearance,
        object_position_offset=(0.0, 0.0, 0.0),
        retreat_offset=ro,
        retreat_xyz=rz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix="ParallelPickR",
    )
    return compose_bimanual_synchronized_sequence(left_seq, right_seq)
