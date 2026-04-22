#!/usr/bin/env python3
"""Place-from-entity resolution for queue tasks（:class:`QueueSlicePlace`）。

单臂切片以 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice` 为准；
本模块仅保留与 Isaac 实体服务相关的放置位姿解析。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from ros2_robot_interface.utils.quat_pose import rotate_vector_by_quat  # pyright: ignore[reportMissingImports]

from robot_action_composer.isaac_sim import get_object_pose_from_service  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSlicePlace,
)


def _apply_target_pose_offset(pose: Any, offset: tuple[float, float, float]) -> Any:
    ox, oy, oz = offset
    pose.position.x += ox
    pose.position.y += oy
    pose.position.z += oz
    return pose


def apply_object_local_offset_to_pose(pose: Any, offset: tuple[float, float, float]) -> Any:
    """Apply ``offset`` in object local frame to ``pose`` position."""
    q = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    dx, dy, dz = rotate_vector_by_quat(offset, q)
    pose.position.x += dx
    pose.position.y += dy
    pose.position.z += dz
    return pose


def resolve_place_skill_from_entity(
    place: QueueSlicePlace,
    *,
    base_world_pos: Any,
    base_world_quat: Any,
) -> QueueSlicePlace:
    """When ``object_prim_path`` is set, fill ``place_position`` from the Isaac
    entity service (plus ``object_position_offset``).

    Place orientation is resolved by caller (``single_arm.place``) via
    ``ee_base_orientation`` / pick fallback.
    """
    if not place.object_prim_path:
        return place
    place_pose = get_object_pose_from_service(
        base_world_pos,
        base_world_quat,
        place.object_prim_path,
        include_orientation=False,
    )
    place_pose = _apply_target_pose_offset(place_pose, place.object_position_offset)
    resolved_place_position = (
        place_pose.position.x,
        place_pose.position.y,
        place_pose.position.z,
    )
    return replace(
        place,
        place_position=resolved_place_position,
    )
