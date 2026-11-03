#!/usr/bin/env python3
"""Place-from-entity resolution for queue tasks（:class:`QueueSlicePlace`）。

单臂切片以 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSingleArmSlice` 为准；
本模块仅保留与 Isaac 实体服务相关的放置位姿解析。
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from ros2_robot_interface.utils.quat_pose import rotate_vector_by_quat  # pyright: ignore[reportMissingImports]

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_object_pose_from_service,
)
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSlicePlace,
)
from robot_action_composer.task_runtime.context import QueueRuntimeContext  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.object_binding import (  # pyright: ignore[reportMissingImports]
    resolve_pick_object_binding,
)
from robot_action_composer.task_runtime.object_resolution_replay import resolve_object_pose_for_task


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


def _place_binding_params(place: QueueSlicePlace) -> dict[str, Any]:
    """Build ``resolve_pick_object_binding`` input for place (no ``active_object`` fallback)."""
    out: dict[str, Any] = {}
    if place.object_prim_path:
        out["object_prim_path"] = place.object_prim_path
    if place.object_key:
        out["object_key"] = place.object_key
    if place.grasp_id:
        out["grasp_id"] = place.grasp_id
    if place.grasp_prim_path:
        out["grasp_prim_path"] = place.grasp_prim_path

    off = place.object_position_offset
    off_t = (
        (float(off[0]), float(off[1]), float(off[2]))
        if isinstance(off, (list, tuple)) and len(off) == 3
        else (0.0, 0.0, 0.0)
    )
    # 有 grasp 时：仅非零 offset 视为显式覆盖；否则走 USD grasp 自动解析
    if place.grasp_id or place.grasp_prim_path:
        if any(abs(v) > 1e-12 for v in off_t):
            out["object_position_offset"] = off_t
    else:
        out["object_position_offset"] = off_t
    return out


def _entity_pose_in_base(
    prim_path: str,
    *,
    base_world_pos: Any,
    base_world_quat: Any,
    ctx: QueueRuntimeContext | None,
):
    if ctx is not None and ctx.object_resolution is not None:
        return resolve_object_pose_for_task(
            ctx,
            object_prim_path=prim_path,
            include_orientation=True,
            arm_side="none",
            object_role="place_target",
            entity_state_timeout=SERVICE_CALL_TIMEOUT,
            retries=SERVICE_CALL_RETRIES,
            retry_delay=SERVICE_RETRY_DELAY,
        )
    return get_object_pose_from_service(
        base_world_pos,
        base_world_quat,
        prim_path,
        include_orientation=True,
    )


def resolve_place_skill_from_entity(
    place: QueueSlicePlace,
    *,
    base_world_pos: Any,
    base_world_quat: Any,
    ctx: QueueRuntimeContext | None = None,
) -> QueueSlicePlace:
    """Resolve place target from ``object_prim_path`` / ``object_key`` (+ optional grasp).

    Same geometry as pick: live **rigid parent** pose from ``/get_entity_state``, plus
    USD-local grasp offset from ``/get_prim_attribute`` (and optional
    ``object_position_offset``). Does **not** call ``/get_entity_state`` on child
    hole/grasp Xforms (stale after parent motion).

    Place orientation: when unset, ``single_arm.place`` uses current EE orientation
    (see ``skill_place``); not inherited from pick.

    ``active_object`` is **not** used (pick target must not leak into place).
    """
    objects = getattr(ctx, "objects", None) if ctx is not None else None
    binding = _place_binding_params(place)
    if not (
        binding.get("object_key")
        or binding.get("object_prim_path")
        or binding.get("grasp_id")
        or binding.get("grasp_prim_path")
    ):
        return place

    path, offset = resolve_pick_object_binding(
        binding,
        objects=objects,
        active_object=None,
        cache=getattr(ctx, "grasp_offset_cache", None) if ctx is not None else None,
    )
    place = replace(place, object_prim_path=path, object_position_offset=offset)
    if not place.object_prim_path:
        return place

    place_pose = _entity_pose_in_base(
        place.object_prim_path,
        base_world_pos=base_world_pos,
        base_world_quat=base_world_quat,
        ctx=ctx,
    )
    place_pose = apply_object_local_offset_to_pose(place_pose, place.object_position_offset)
    resolved_place_position = (
        float(place_pose.position.x),
        float(place_pose.position.y),
        float(place_pose.position.z),
    )
    print(
        f"[Place] target {place.object_prim_path} offset={place.object_position_offset} -> "
        f"({resolved_place_position[0]:.3f},{resolved_place_position[1]:.3f},"
        f"{resolved_place_position[2]:.3f})"
    )
    return replace(place, place_position=resolved_place_position)
