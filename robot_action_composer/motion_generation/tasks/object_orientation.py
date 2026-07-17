"""物体姿态滤波与「对齐标定」末端姿态合成。

抓取配置里的 ``ee_base_orientation`` / 工具系偏移，按「某一标称机物相对朝向」标定
（常见：机物 X 对齐，或导航到位后固定相对偏航，例如 ``0 / ±π/2 / ±π`` 一档）。

运行时只补偿**相对该标称的偏差**（推荐仅 yaw）：

``yaw_corr = wrap(yaw_object − aligned_object_yaw)``
``q_ee = normalize(R_z(yaw_corr) ⊗ q_ee_base)``

``aligned_object_yaw``：

- 数值（弧度）：显式标称偏航
- ``auto`` / ``snap``：把当前物体在运动系下的 yaw **吸附到最近的** ``k·π/2``
  （0、±1.5708、±3.1416…），再算偏差——适合导航朝向落在固定档位的场景
"""

from __future__ import annotations

import math
from typing import Any

from geometry_msgs.msg import Pose

from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    euler_rpy_to_quat_xyzw,
    quat_multiply,
    quat_normalize,
)

_AUTO_ALIGNED_YAW_TOKENS = frozenset(
    {"auto", "snap", "snap_pi_2", "nearest", "nearest_pi_2"}
)


def object_orientation_xyzw(pose: Any) -> tuple[float, float, float, float]:
    return quat_normalize(
        (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def quat_xyzw_to_rpy(q_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = quat_normalize(q_xyzw)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi * 0.5, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle_pi(angle: float) -> float:
    """将角包到 ``(-π, π]``。"""
    a = float(angle)
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def snap_yaw_to_nearest_pi_half(yaw: float) -> float:
    """将 yaw 吸附到最近的 ``k·π/2``（0、±π/2、±π），结果落在 ``(-π, π]``。"""
    step = 0.5 * math.pi
    n = int(round(float(yaw) / step))
    return wrap_angle_pi(n * step)


def resolve_aligned_object_yaw(
    aligned_object_yaw: float | str | None,
    object_pose: Any,
) -> float:
    """解析标称物体 yaw：数值直用；``auto`` 则按物体当前 yaw 吸附到最近 π/2 档。"""
    if aligned_object_yaw is None:
        return 0.0
    if isinstance(aligned_object_yaw, str):
        key = aligned_object_yaw.strip().lower()
        if key in _AUTO_ALIGNED_YAW_TOKENS:
            _, _, yaw = quat_xyzw_to_rpy(object_orientation_xyzw(object_pose))
            return snap_yaw_to_nearest_pi_half(yaw)
        if key == "":
            return 0.0
        return float(key)
    return float(aligned_object_yaw)


def filter_object_orientation_xyzw(
    q_obj: tuple[float, float, float, float],
    mode: str,
    *,
    aligned_object_yaw: float = 0.0,
) -> tuple[float, float, float, float]:
    """按 ``object_orientation_mode`` 过滤物体姿态（运动系下），并扣除标称偏航。

    - ``full`` / ``object``：完整物体姿态，再左乘 ``R_z(-aligned_object_yaw)`` 得到相对标称的姿态
    - ``yaw`` / ``yaw_only``：仅 ``wrap(yaw − aligned_object_yaw)``（导航后机物竖直、只差朝向时的推荐模式）
    - ``tilt`` / ``no_yaw``：roll+pitch，yaw=0（不扣标称偏航）
    - ``pitch``：仅 pitch
    - ``none`` / ``identity`` / ``motion``：单位姿态（不跟物体）
    """
    mode_norm = str(mode or "full").strip().lower()
    yaw0 = float(aligned_object_yaw)

    if mode_norm in ("none", "identity", "motion"):
        return (0.0, 0.0, 0.0, 1.0)

    roll, pitch, yaw = quat_xyzw_to_rpy(q_obj)
    yaw_corr = wrap_angle_pi(yaw - yaw0)

    if mode_norm in ("", "full", "object"):
        # 相对标称：R_z(-yaw0) 左乘到物体姿态 → 标称处近似单位，偏差保留
        q_align_inv = euler_rpy_to_quat_xyzw(0.0, 0.0, -yaw0)
        return quat_normalize(quat_multiply(q_align_inv, quat_normalize(q_obj)))
    if mode_norm in ("yaw", "yaw_only", "heading"):
        return euler_rpy_to_quat_xyzw(0.0, 0.0, yaw_corr)
    if mode_norm in ("tilt", "no_yaw", "roll_pitch", "roll_pitch_only"):
        return euler_rpy_to_quat_xyzw(roll, pitch, 0.0)
    if mode_norm in ("pitch", "pitch_only"):
        return euler_rpy_to_quat_xyzw(0.0, pitch, 0.0)
    raise ValueError(
        f"unsupported object_orientation_mode {mode!r}; "
        "expected 'full', 'yaw', 'tilt', 'pitch', or 'none'"
    )


def filter_object_pose_orientation(
    pose: Pose,
    mode: str,
    *,
    aligned_object_yaw: float | str = 0.0,
) -> Pose:
    """返回仅姿态被过滤、位置不变的副本。"""
    yaw0 = resolve_aligned_object_yaw(aligned_object_yaw, pose)
    q = filter_object_orientation_xyzw(
        object_orientation_xyzw(pose),
        mode,
        aligned_object_yaw=yaw0,
    )
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
    return out


def compose_aligned_ee_orientation(
    ee_base_orientation: tuple[float, float, float, float],
    object_pose: Any,
    *,
    ee_orientation_frame: str = "motion",
    object_orientation_mode: str = "yaw",
    aligned_object_yaw: float | str = 0.0,
) -> tuple[float, float, float, float]:
    """将「标称机物相对朝向」下标定的 ``ee_base_orientation`` 变换到当前物体朝向。

    - ``ee_orientation_frame=motion``：直接返回标定姿态（旧行为）。
    - ``ee_orientation_frame=object``：``q_corr(object, aligned_object_yaw) ⊗ ee_base``。

    ``aligned_object_yaw``：数值，或 ``auto``（吸附到最近的 ``k·π/2``）。
    """
    frame = str(ee_orientation_frame or "motion").strip().lower()
    if frame in ("", "motion", "base", "world"):
        return quat_normalize(ee_base_orientation)
    if frame not in ("object", "object_body", "body", "local"):
        raise ValueError(
            f"unsupported ee_orientation_frame {ee_orientation_frame!r}; "
            "expected 'motion' or 'object'"
        )
    yaw0 = resolve_aligned_object_yaw(aligned_object_yaw, object_pose)
    q_corr = filter_object_orientation_xyzw(
        object_orientation_xyzw(object_pose),
        object_orientation_mode,
        aligned_object_yaw=yaw0,
    )
    return quat_normalize(quat_multiply(q_corr, ee_base_orientation))
