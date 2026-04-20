#!/usr/bin/env python3
"""Drawer geometry config and Cartesian stage helpers for ``single_arm.drawer.*`` queue skills.

Pick/place motion fields come from :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice`;
this module only holds drawer-prim geometry and drawer-specific grasp clearance.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmStage,
    StageTarget,
    assign_to_arm,
    build_single_arm_pick_sequence,
)
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
    QueueSlicePick,
)


@dataclass(frozen=True)
class DrawerGeometryConfig:
    """Drawer cabinet / handle prim paths and extents (structured overlays → ``MergedQueueConfig.drawer``)."""

    source_object_path_drawer: str = ""
    source_object_path_drawer_all: str = ""
    handle_extent_max: tuple[float, float, float] = (0.0, 0.0, 0.0)
    handle_extent_min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    drawer_scale: float = 1.0
    object_xyz_random_offset_drawer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    grasp_clearance_drawer: float = 0.01
    #: 拉开抽屉时沿 grasp 方向末段行程（米），语义同 pick 的 ``retreat_direction_extra``；在 ``skill_defaults.single_arm.drawer`` 中配置。
    pull_distance: float = 0.18


def format_drawer_task_cfg_summary(scene: str, drawer: DrawerGeometryConfig, single: QueueSingleArmSlice) -> str:
    p, c = single.pick, single.common
    return (
        f"[Drawer] scene={scene} object={p.object_prim_path} "
        f"drawer_prim={drawer.source_object_path_drawer} | "
        f"arm={c.arm}, grasp_dir={p.grasp_direction}, pull_distance={drawer.pull_distance}"
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
    ee_base_orientation: tuple[float, float, float, float],
    grasp_direction_vector: tuple[float, float, float],
    pull_distance: float,
) -> list[StageTarget]:
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            ee_base_orientation=ee_base_orientation,
            prepare_offset=pick.prepare_offset,
            pick_clearance=drawer.grasp_clearance_drawer,
            grasp_direction_vector=grasp_direction_vector,
            object_position_offset=pick.object_position_offset,
            retreat_direction_extra=pull_distance,
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
    ee_base_orientation: tuple[float, float, float, float],
    grasp_direction_vector: tuple[float, float, float],
) -> list[StageTarget]:
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            ee_base_orientation=ee_base_orientation,
            prepare_offset=(0.0, 0.0, 0.0),
            pick_clearance=drawer.grasp_clearance_drawer,
            grasp_direction_vector=grasp_direction_vector,
            object_position_offset=pick.object_position_offset,
            retreat_direction_extra=0,
            retreat_offset=pick.retreat_offset,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix="PickPlaceFlow",
        )
    )
    return assign_to_arm(arm_seq, arm_side)
