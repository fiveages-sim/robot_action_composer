"""Wrist micro-press / lift helpers (OCS2 position interface)."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Optional

from ..se3 import T_from_pose, T_to_xyz_quat_xyzw, pose_from_T


def press_wrist_along_tip_z(
    interface: Any,
    plan: Any,
    *,
    press_m: float,
    arrival_timeout: float = 4.0,
    axis: str = "tip_z",
) -> dict[str, Any]:
    """Translate EE and send.

    Axes: ``tip_z`` / ``tip_neg_z`` / ``world_neg_z`` / ``world_pos_z``.
    """
    if plan is None:
        raise RuntimeError("wrist move needs last wrist plan (run Plan/move first)")
    if abs(float(press_m)) < 1e-6:
        return {"status": "noop", "press_m": 0.0}

    arm = getattr(plan, "arm", "right")
    handler = (
        getattr(interface, "right_arm_handler", None)
        if arm == "right"
        else getattr(interface, "left_arm_handler", None)
    )
    if handler is None:
        raise RuntimeError(f"no arm handler for {arm}")

    pose = deepcopy(plan.wrist_pose)
    T = T_from_pose(pose)
    delta = abs(float(press_m))
    axis_l = (axis or "tip_neg_z").strip().lower()
    if axis_l == "world_neg_z":
        T[2, 3] -= delta
    elif axis_l in ("world_pos_z", "world_z", "+z"):
        T[2, 3] += delta
    else:
        z_axis = T[:3, 2]
        sign = -1.0 if axis_l in ("tip_neg_z", "neg_z", "-tip_z") else 1.0
        T[:3, 3] = T[:3, 3] + z_axis * (sign * delta)

    new_pose = pose_from_T(T)
    frame_id = getattr(plan, "motion_frame_id", "base_link")
    handler.send_target_stamped(frame_id, new_pose)
    part = "right_arm" if arm == "right" else "left_arm"
    arrival = interface.wait_until_arrive(part=part, timeout=arrival_timeout)
    xyz, _ = T_to_xyz_quat_xyzw(T)
    try:
        plan.wrist_pose = new_pose
    except Exception:
        pass
    return {
        "status": "sent",
        "press_m": float(press_m),
        "axis": axis_l,
        "target_xyz": list(xyz),
        "arrival": arrival,
    }


def press_wrist_world_z(
    interface: Any,
    plan: Any,
    *,
    lift_m: float,
    arrival_timeout: float = 4.0,
) -> dict[str, Any]:
    """Lift EE by ``lift_m`` in world +Z (table clearance before close)."""
    if abs(float(lift_m)) < 1e-6:
        return {"status": "noop", "lift_m": 0.0}
    return press_wrist_along_tip_z(
        interface,
        plan,
        press_m=abs(float(lift_m)),
        arrival_timeout=arrival_timeout,
        axis="world_pos_z",
    )


def lookup_frame_xyz(
    interface: Any,
    frame: str,
    base_frame: str,
    *,
    timeout: float = 0.5,
) -> Optional[list[float]]:
    """XYZ of ``frame`` expressed in ``base_frame`` (for Δpose logging)."""
    if not frame:
        return None
    try:
        ts = interface.lookup_transform(base_frame, frame, timeout=timeout)
        if ts is None:
            return None
        t = ts.transform.translation
        return [float(t.x), float(t.y), float(t.z)]
    except Exception:
        return None
