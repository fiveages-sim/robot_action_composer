"""SE(3) helpers for HUG camera-frame poses ↔ ROS ``geometry_msgs/Pose``."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from geometry_msgs.msg import Pose


def T_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def R_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ROS-style R = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles in degrees."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def T_from_rpy_deg(
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    xyz: Sequence[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    T = T_identity()
    T[:3, :3] = R_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
    T[0, 3], T[1, 3], T[2, 3] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return T


def T_from_xyz_quat_xyzw(
    xyz: Sequence[float],
    quat_xyzw: Sequence[float],
) -> np.ndarray:
    """Build 4×4 from translation + quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    T = T_identity()
    T[:3, :3] = R
    T[0, 3], T[1, 3], T[2, 3] = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return T


def T_to_xyz_quat_xyzw(T: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Extract translation and quaternion (x, y, z, w) from 4×4."""
    R = np.asarray(T[:3, :3], dtype=np.float64)
    t = (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))
    # Shepperd's method
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0:
        s = (tr + 1.0) ** 0.5 * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = (float(x), float(y), float(z), float(w))
    n = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5
    q = (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    return t, q


def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = T_identity()
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def T_mul(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    return np.asarray(A, dtype=np.float64) @ np.asarray(B, dtype=np.float64)


def pose_from_T(T: np.ndarray) -> Pose:
    xyz, q = T_to_xyz_quat_xyzw(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def T_from_pose(pose: Pose) -> np.ndarray:
    return T_from_xyz_quat_xyzw(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def T_from_transform_stamped(ts) -> np.ndarray:
    """``geometry_msgs/TransformStamped`` → 4×4 (source in target frame)."""
    tr = ts.transform.translation
    rq = ts.transform.rotation
    return T_from_xyz_quat_xyzw(
        (tr.x, tr.y, tr.z),
        (rq.x, rq.y, rq.z, rq.w),
    )
