#!/usr/bin/env python3
"""Place-from-entity resolution for queue tasks（:class:`QueueSlicePlace`）。

单臂切片以 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice` 为准；
本模块仅保留与 Isaac 实体服务相关的放置位姿解析。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

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


def resolve_place_skill_from_entity(
    place: QueueSlicePlace,
    *,
    base_world_pos: Any,
    base_world_quat: Any,
    current_obs: dict[str, Any] | None = None,
    ee_prefix_for_orientation_fallback: str = "left_ee",
) -> QueueSlicePlace:
    """When ``run_place_before_return`` and ``place_object_entity_path`` are set, fill
    ``place_position`` from the Isaac entity service (plus ``place_pose_offset``).

    If ``place_orientation`` is still ``None``, copy the current EE quaternion from
    ``current_obs`` (same idea as aligning pick target orientation to the arm).
    """
    if not place.run_place_before_return or not place.place_object_entity_path:
        return place
    place_pose = get_object_pose_from_service(
        base_world_pos,
        base_world_quat,
        place.place_object_entity_path,
        include_orientation=False,
    )
    place_pose = _apply_target_pose_offset(place_pose, place.place_pose_offset)
    resolved_place_position = (
        place_pose.position.x,
        place_pose.position.y,
        place_pose.position.z,
    )
    place_orientation = place.place_orientation
    if place_orientation is None and current_obs is not None:
        px = f"{ee_prefix_for_orientation_fallback}.quat.x"
        py = f"{ee_prefix_for_orientation_fallback}.quat.y"
        pz = f"{ee_prefix_for_orientation_fallback}.quat.z"
        pw = f"{ee_prefix_for_orientation_fallback}.quat.w"
        if px in current_obs and py in current_obs and pz in current_obs and pw in current_obs:
            place_orientation = (
                float(current_obs[px]),
                float(current_obs[py]),
                float(current_obs[pz]),
                float(current_obs[pw]),
            )
    return replace(
        place,
        place_position=resolved_place_position,
        place_orientation=place_orientation,
    )
