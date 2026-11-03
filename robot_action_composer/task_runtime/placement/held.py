"""Resolve the live pose of an object currently held by the gripper."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_object_pose_from_service,
)
from robot_action_composer.motion_generation.tasks.pick_place import (  # pyright: ignore[reportMissingImports]
    apply_object_local_offset_to_pose,
)
from robot_action_composer.task_runtime.object_binding import (  # pyright: ignore[reportMissingImports]
    normalize_prim_path,
    resolve_pick_object_binding,
)


def _clone_pose(pose: Pose) -> Pose:
    out = Pose()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    out.orientation.x = float(pose.orientation.x)
    out.orientation.y = float(pose.orientation.y)
    out.orientation.z = float(pose.orientation.z)
    out.orientation.w = float(pose.orientation.w)
    return out


def resolve_held_object_pose(
    ctx: Any,
    params: Mapping[str, Any],
) -> tuple[Pose, str] | None:
    """手中对齐点位姿（``ctx`` 物体解算系）。

    - ``held_object_key`` (+ 可选 ``held_grasp_id``)：刚体实时位姿 + USD 局部 grasp
      （如 ``bottom: tube_bottom``），勿对子 Xform 直接 ``get_entity_state``。
    - ``held_object_prim_path``：仍按刚体 prim 读（兼容旧配置）。

    始终读实时服务，**不走** object-resolution replay：手中物体是动态量，重放冻结
    位姿既无意义，也会让对齐前后的两次采样不一致。
    """
    raw = params.get("held_object_prim_path")
    key = str(params.get("held_object_key") or "").strip()
    grasp_id = str(params.get("held_grasp_id") or "").strip()
    if isinstance(raw, str) and raw.strip():
        path = normalize_prim_path(raw)
        offset = (0.0, 0.0, 0.0)
        label = path
    elif key:
        binding: dict[str, Any] = {"object_key": key}
        if grasp_id:
            binding["grasp_id"] = grasp_id
        path, offset = resolve_pick_object_binding(
            binding,
            objects=getattr(ctx, "objects", None),
            active_object=None,
            cache=getattr(ctx, "grasp_offset_cache", None),
        )
        label = f"{path}" + (f"/{grasp_id}" if grasp_id else "")
    else:
        return None

    held_pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path,
        include_orientation=True,
        entity_state_timeout=SERVICE_CALL_TIMEOUT,
        retries=SERVICE_CALL_RETRIES,
        retry_delay=SERVICE_RETRY_DELAY,
    )
    held_pose = apply_object_local_offset_to_pose(_clone_pose(held_pose), offset)
    return held_pose, label
