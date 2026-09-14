"""Resolve ``T_base_cam`` via TF (preferred) or Isaac ``/get_entity_state``."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from .se3 import T_from_transform_stamped, T_from_xyz_quat_xyzw, T_inv, T_mul


@dataclass
class CamFrameResolution:
    method: str  # tf | entity_state
    base_frame: str
    camera_frame: str
    T_base_cam: np.ndarray


def lookup_T_base_cam_tf(
    interface: Any,
    *,
    base_frame: str,
    camera_frames: Sequence[str],
    timeout: float = 2.0,
) -> CamFrameResolution:
    """Try TF frames in order; ``lookup_transform(base, cam)`` = cam in base."""
    errors: list[str] = []
    for cam in camera_frames:
        ts = interface.lookup_transform(base_frame, cam, timeout=timeout)
        if ts is None:
            errors.append(cam)
            continue
        return CamFrameResolution(
            method="tf",
            base_frame=base_frame,
            camera_frame=cam,
            T_base_cam=T_from_transform_stamped(ts),
        )
    raise RuntimeError(
        f"TF lookup failed for camera frames {list(camera_frames)} "
        f"relative to '{base_frame}' (tried: {errors}). "
        "Confirm frames with: ros2 run tf2_ros tf2_echo base_link head_camera"
    )


def lookup_T_base_cam_entity(
    *,
    base_prim: str,
    camera_prim: str,
) -> CamFrameResolution:
    """World poses via ``/get_entity_state``, then ``T_base_cam = inv(T_w_base) @ T_w_cam``."""
    from robot_action_composer.isaac_sim import get_entity_pose_world_service

    b_xyz, b_q = get_entity_pose_world_service(base_prim)
    c_xyz, c_q = get_entity_pose_world_service(camera_prim)
    T_w_base = T_from_xyz_quat_xyzw(b_xyz, b_q)
    T_w_cam = T_from_xyz_quat_xyzw(c_xyz, c_q)
    T_base_cam = T_mul(T_inv(T_w_base), T_w_cam)
    return CamFrameResolution(
        method="entity_state",
        base_frame=base_prim.rsplit("/", 1)[-1],
        camera_frame=camera_prim.rsplit("/", 1)[-1],
        T_base_cam=T_base_cam,
    )


def resolve_T_base_cam(
    interface: Optional[Any],
    *,
    base_frame: str,
    camera_frames: Sequence[str],
    camera_prim: str,
    base_prim: str,
    tf_timeout: float = 2.0,
    prefer: str = "tf",
) -> CamFrameResolution:
    """Resolve camera extrinsics. ``prefer``: tf | entity_state | auto."""
    if prefer in ("tf", "auto") and interface is not None:
        try:
            return lookup_T_base_cam_tf(
                interface,
                base_frame=base_frame,
                camera_frames=camera_frames,
                timeout=tf_timeout,
            )
        except Exception as exc:
            if prefer == "tf":
                raise
            last_tf_err = exc
    else:
        last_tf_err = None

    try:
        return lookup_T_base_cam_entity(base_prim=base_prim, camera_prim=camera_prim)
    except Exception as entity_err:
        raise RuntimeError(
            f"Could not resolve T_base_cam via TF or entity_state. "
            f"tf_error={last_tf_err!r} entity_error={entity_err!r}"
        ) from entity_err
