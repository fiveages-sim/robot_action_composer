#!/usr/bin/env python3
"""Drawer geometry config and Cartesian stage helpers for ``single_arm.drawer.*`` queue skills.

Pick/place motion fields come from :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice`;
this module only holds drawer-prim geometry and drawer-specific grasp clearance.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmStage,
    StageTarget,
    assign_to_arm,
    build_single_arm_pick_sequence,
    build_single_arm_place_sequence,
    build_single_arm_return_home_sequence,
)
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
    QueueSlicePick,
    QueueSlicePlace,
)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert Euler angles to quaternion (ZYX: yaw, pitch, roll)."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


@dataclass(frozen=True)
class DrawerGeometryConfig:
    """Drawer cabinet / handle prim paths and extents (merged flat → ``MergedQueueConfig.drawer``)."""

    source_object_path_drawer: str = ""
    source_object_path_drawer_all: str = ""
    handle_extent_max: tuple[float, float, float] = (0.0, 0.0, 0.0)
    handle_extent_min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    drawer_scale: float = 1.0
    object_xyz_random_offset_drawer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    grasp_clearance_drawer: float = 0.01


def format_drawer_task_cfg_summary(scene: str, drawer: DrawerGeometryConfig, single: QueueSingleArmSlice) -> str:
    p, c = single.pick, single.common
    return (
        f"[Drawer] scene={scene} object={p.source_object_entity_path} "
        f"drawer_prim={drawer.source_object_path_drawer} | "
        f"arm={c.arm}, grasp_dir={p.grasp_direction}"
    )


def _apply_target_pose_offset(pose: Any, offset: tuple[float, float, float]) -> Any:
    ox, oy, oz = offset
    pose.position.x += ox
    pose.position.y += oy
    pose.position.z += oz
    return pose


def build_single_arm_pull_drawer_sequence(
    *,
    target_pose: Any,
    pick: QueueSlicePick,
    drawer: DrawerGeometryConfig,
    arm_side: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    grasp_orientation: tuple[float, float, float, float],
    grasp_direction_vector: tuple[float, float, float],
) -> list[StageTarget]:
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            approach_clearance=pick.approach_clearance,
            grasp_clearance=drawer.grasp_clearance_drawer,
            grasp_orientation=grasp_orientation,
            grasp_direction_vector=grasp_direction_vector,
            grasp_offset=pick.grasp_offset,
            retreat_direction_extra=pick.retreat_direction_extra,
            retreat_offset=pick.retreat_offset,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix="PickPlaceFlow",
        )
    )
    return assign_to_arm(arm_seq, arm_side)


def build_single_arm_close_drawer_sequence(
    *,
    target_pose: Any,
    pick: QueueSlicePick,
    drawer: DrawerGeometryConfig,
    arm_side: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    grasp_orientation: tuple[float, float, float, float],
    grasp_direction_vector: tuple[float, float, float],
) -> list[StageTarget]:
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            approach_clearance=0,
            grasp_clearance=drawer.grasp_clearance_drawer,
            grasp_orientation=grasp_orientation,
            grasp_direction_vector=grasp_direction_vector,
            grasp_offset=pick.grasp_offset,
            retreat_direction_extra=0,
            retreat_offset=pick.retreat_offset,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix="PickPlaceFlow",
        )
    )
    return assign_to_arm(arm_seq, arm_side)


def build_single_arm_back_home_sequence(
    *,
    place_position: Any,
    home_pose: Any,
    place: QueueSlicePlace,
    arm_side: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    grasp_orientation: tuple[float, float, float, float],
) -> list[StageTarget]:
    arm_seq: list[ArmStage] = list(
        build_single_arm_place_sequence(
            place_position=place_position,
            place_orientation=grasp_orientation,
            place_direction=place.place_direction,
            place_direction_vector=place.place_direction_vector,
            place_approach_clearance=place.place_approach_clearance,
            place_insert_clearance=place.place_insert_clearance,
            post_release_retract_offset=(0, 0, 0),
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix="PickPlaceFlow",
            start_index=5,
        )
    )
    return_stage_name = "PickPlaceFlow-5-ReturnHomeHold"
    arm_seq.extend(
        build_single_arm_return_home_sequence(
            home_pose=home_pose,
            gripper=gripper_open,
            stage_name=return_stage_name,
        )
    )
    return assign_to_arm(arm_seq, arm_side)
