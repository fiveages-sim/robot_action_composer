"""Pure geometry helpers for place / insert planning (no ROS services)."""

from __future__ import annotations

import math
from typing import Any

from geometry_msgs.msg import Pose
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_conjugate,
    quat_normalize,
    rotate_vector_by_quat,
)

Vec3 = tuple[float, float, float]

_AXIS_VECTORS: dict[str, Vec3] = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}


def axis_vector_from_label(label: str) -> Vec3:
    """Parse ``+x/-x/+y/-y/+z/-z`` into a unit vector."""
    key = str(label or "").strip().lower()
    vec = _AXIS_VECTORS.get(key)
    if vec is None:
        raise ValueError(
            f"axis label must be one of {sorted(_AXIS_VECTORS)}, got {label!r}"
        )
    return vec


def xyz(pose: Pose) -> Vec3:
    return (float(pose.position.x), float(pose.position.y), float(pose.position.z))


def vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v: Vec3) -> Vec3:
    n = vec_norm(v)
    if n < 1e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def set_pose_quat_xyzw(
    pose: Pose, q: tuple[float, float, float, float]
) -> None:
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]


def pose_quat_xyzw(pose: Pose) -> tuple[float, float, float, float]:
    return (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def object_axis_in_world(pose: Pose, axis_label: str = "+z") -> Vec3:
    """Rotate an object-local axis (default local +Z) into the pose's reference frame."""
    return normalize(rotate_vector_by_quat(axis_vector_from_label(axis_label), pose_quat_xyzw(pose)))


def tilt_deg(a: Vec3, b: Vec3) -> float:
    """Angle between two axes, treating opposite directions as aligned (0..90°)."""
    na = normalize(a)
    nb = normalize(b)
    c = max(-1.0, min(1.0, abs(vec_dot(na, nb))))
    return math.degrees(math.acos(c))


def angle_deg(a: Vec3, b: Vec3) -> float:
    """Signed-less angle between two **directed** vectors (0..180°)."""
    na = normalize(a)
    nb = normalize(b)
    c = max(-1.0, min(1.0, vec_dot(na, nb)))
    return math.degrees(math.acos(c))


def quat_between_vectors(
    a: Vec3, b: Vec3
) -> tuple[float, float, float, float]:
    """Minimal rotation (xyzw) that rotates direction ``a`` onto ``b``."""
    ua = normalize(a)
    ub = normalize(b)
    d = vec_dot(ua, ub)
    if d > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -1.0 + 1e-9:
        # 180°: any axis orthogonal to ``ua`` works; prefer X unless parallel.
        axis = vec_cross(ua, (1.0, 0.0, 0.0))
        if vec_norm(axis) < 1e-6:
            axis = vec_cross(ua, (0.0, 1.0, 0.0))
        ax = normalize(axis)
        return (ax[0], ax[1], ax[2], 0.0)
    # Keep the raw cross product: its norm is sin(theta), so (cross, 1+dot)
    # normalizes to (axis*sin(theta/2), cos(theta/2)).
    cross = vec_cross(ua, ub)
    return quat_normalize((cross[0], cross[1], cross[2], 1.0 + d))


def world_vec_in_pose_frame(world_vec: Vec3, pose: Pose) -> Vec3:
    """Express a world/reference-frame vector in the pose's tool frame (R^T v)."""
    return rotate_vector_by_quat(
        world_vec, quat_conjugate(quat_normalize(pose_quat_xyzw(pose)))
    )


def project_onto_axis(
    vec: Vec3,
    axis: Vec3,
) -> tuple[Vec3, float, Vec3]:
    """Project ``vec`` onto ``axis``. Returns ``(projected, height, unit_axis)``."""
    n = vec_norm(axis)
    if n < 1e-9:
        return (0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 1.0)
    axis_u = vec_scale(axis, 1.0 / n)
    height = vec_dot(vec, axis_u)
    if height < 0.0:
        axis_u = vec_scale(axis_u, -1.0)
        height = -height
    return vec_scale(axis_u, height), height, axis_u


def pose_xyz_in_frame(
    iface: Any,
    pose: Pose,
    from_frame: str,
    to_frame: str,
    timeout: float,
) -> Vec3:
    """Transform ``pose`` between reference frames and return its xyz."""
    if from_frame == to_frame:
        return xyz(pose)
    if not hasattr(iface, "transform_pose"):
        raise TypeError("placement requires ROS2RobotInterface.transform_pose")
    out = iface.transform_pose(pose, from_frame, to_frame, timeout=timeout)
    if out is None:
        raise RuntimeError(
            f"placement: TF {from_frame!r} -> {to_frame!r} failed"
        )
    return xyz(out)
