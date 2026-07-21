"""物体姿态滤波与「对齐标定」末端姿态合成。

抓取配置里的 ``ee_base_orientation`` / 工具系偏移，按「某一标称机物相对朝向」标定
（常见：机物 X 对齐，或导航到位后固定相对偏航，例如 ``0 / ±π/2 / ±π`` 一档）。

运行时只补偿**相对该标称的偏差**：

``yaw`` 模式（默认）::

    yaw_corr = wrap(yaw_object − aligned_object_yaw)
    q_ee = normalize(R_z(yaw_corr) ⊗ q_ee_base)

``yaw_roll`` 模式（yaw + roll 同时吸附）::

    yaw_corr  = wrap(yaw_object  − aligned_object_yaw)
    roll_corr = wrap(roll_object − aligned_object_roll)
    q_corr    = euler_rpy(roll_corr, 0, yaw_corr)
    q_ee      = normalize(q_corr ⊗ q_ee_base)

在 pitch≈0 / 水平抓、且标定档落在物轴网格上时，``yaw_roll`` 几何上等价于把夹爪
运动轴（由 ``ee_base`` 标定，通常工具系 +Z）贴到物体最近的 ±X/±Y/±Z。

``aligned_object_yaw`` / ``aligned_object_roll``：

- 数值（弧度）：显式标称角
- ``auto`` / ``snap``：把当前物体在运动系下的对应角 **吸附到最近的** ``k·π/2``
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

_AUTO_ALIGNED_ANGLE_TOKENS = frozenset(
    {"auto", "snap", "snap_pi_2", "nearest", "nearest_pi_2"}
)
# Back-compat alias
_AUTO_ALIGNED_YAW_TOKENS = _AUTO_ALIGNED_ANGLE_TOKENS

_YAW_ROLL_MODES = frozenset({"yaw_roll", "yaw_and_roll", "heading_roll"})


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


def snap_angle_to_nearest_pi_half(angle: float) -> float:
    """将角吸附到最近的 ``k·π/2``（0、±π/2、±π），结果落在 ``(-π, π]``。"""
    step = 0.5 * math.pi
    n = int(round(float(angle) / step))
    return wrap_angle_pi(n * step)


def snap_yaw_to_nearest_pi_half(yaw: float) -> float:
    """将 yaw 吸附到最近的 ``k·π/2``（0、±π/2、±π），结果落在 ``(-π, π]``。"""
    return snap_angle_to_nearest_pi_half(yaw)


def _resolve_aligned_angle(
    aligned: float | str | None,
    object_pose: Any,
    *,
    component: str,
) -> float:
    """解析标称角：数值直用；``auto`` 则按物体当前 roll/yaw 吸附到最近 π/2 档。"""
    if aligned is None:
        return 0.0
    if isinstance(aligned, str):
        key = aligned.strip().lower()
        if key in _AUTO_ALIGNED_ANGLE_TOKENS:
            roll, _, yaw = quat_xyzw_to_rpy(object_orientation_xyzw(object_pose))
            value = roll if component == "roll" else yaw
            return snap_angle_to_nearest_pi_half(value)
        if key == "":
            return 0.0
        return float(key)
    return float(aligned)


def resolve_aligned_object_yaw(
    aligned_object_yaw: float | str | None,
    object_pose: Any,
) -> float:
    """解析标称物体 yaw：数值直用；``auto`` 则按物体当前 yaw 吸附到最近 π/2 档。"""
    return _resolve_aligned_angle(aligned_object_yaw, object_pose, component="yaw")


def resolve_aligned_object_roll(
    aligned_object_roll: float | str | None,
    object_pose: Any,
) -> float:
    """解析标称物体 roll：数值直用；``auto`` 则按物体当前 roll 吸附到最近 π/2 档。"""
    return _resolve_aligned_angle(aligned_object_roll, object_pose, component="roll")


def _mode_uses_roll(mode: str) -> bool:
    return str(mode or "").strip().lower() in _YAW_ROLL_MODES


def filter_object_orientation_xyzw(
    q_obj: tuple[float, float, float, float],
    mode: str,
    *,
    aligned_object_yaw: float = 0.0,
    aligned_object_roll: float = 0.0,
) -> tuple[float, float, float, float]:
    """按 ``object_orientation_mode`` 过滤物体姿态（运动系下），并扣除标称角。

    - ``full`` / ``object``：完整物体姿态，再左乘 ``R_z(-aligned_object_yaw)`` 得到相对标称的姿态
    - ``yaw`` / ``yaw_only``：仅 ``wrap(yaw − aligned_object_yaw)``（导航后机物竖直、只差朝向时的推荐模式）
    - ``yaw_roll`` / ``yaw_and_roll`` / ``heading_roll``：``euler(roll_corr, 0, yaw_corr)``，
      pitch 置 0；相对标称同时补偿 yaw 与 roll
    - ``tilt`` / ``no_yaw``：roll+pitch，yaw=0（不扣标称偏航）
    - ``pitch``：仅 pitch
    - ``none`` / ``identity`` / ``motion``：单位姿态（不跟物体）
    """
    mode_norm = str(mode or "full").strip().lower()
    yaw0 = float(aligned_object_yaw)
    roll0 = float(aligned_object_roll)

    if mode_norm in ("none", "identity", "motion"):
        return (0.0, 0.0, 0.0, 1.0)

    roll, pitch, yaw = quat_xyzw_to_rpy(q_obj)
    yaw_corr = wrap_angle_pi(yaw - yaw0)
    roll_corr = wrap_angle_pi(roll - roll0)

    if mode_norm in ("", "full", "object"):
        # 相对标称：R_z(-yaw0) 左乘到物体姿态 → 标称处近似单位，偏差保留
        q_align_inv = euler_rpy_to_quat_xyzw(0.0, 0.0, -yaw0)
        return quat_normalize(quat_multiply(q_align_inv, quat_normalize(q_obj)))
    if mode_norm in ("yaw", "yaw_only", "heading"):
        return euler_rpy_to_quat_xyzw(0.0, 0.0, yaw_corr)
    if mode_norm in _YAW_ROLL_MODES:
        return euler_rpy_to_quat_xyzw(roll_corr, 0.0, yaw_corr)
    if mode_norm in ("tilt", "no_yaw", "roll_pitch", "roll_pitch_only"):
        return euler_rpy_to_quat_xyzw(roll, pitch, 0.0)
    if mode_norm in ("pitch", "pitch_only"):
        return euler_rpy_to_quat_xyzw(0.0, pitch, 0.0)
    raise ValueError(
        f"unsupported object_orientation_mode {mode!r}; "
        "expected 'full', 'yaw', 'yaw_roll', 'tilt', 'pitch', or 'none'"
    )


def filter_object_pose_orientation(
    pose: Pose,
    mode: str,
    *,
    aligned_object_yaw: float | str = 0.0,
    aligned_object_roll: float | str = 0.0,
) -> Pose:
    """返回仅姿态被过滤、位置不变的副本。"""
    yaw0 = resolve_aligned_object_yaw(aligned_object_yaw, pose)
    roll0 = 0.0
    if _mode_uses_roll(mode):
        roll0 = resolve_aligned_object_roll(aligned_object_roll, pose)
    q = filter_object_orientation_xyzw(
        object_orientation_xyzw(pose),
        mode,
        aligned_object_yaw=yaw0,
        aligned_object_roll=roll0,
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
    aligned_object_roll: float | str = "auto",
) -> tuple[float, float, float, float]:
    """将「标称机物相对朝向」下标定的 ``ee_base_orientation`` 变换到当前物体朝向。

    - ``ee_orientation_frame=motion``：直接返回标定姿态（旧行为）。
    - ``ee_orientation_frame=object``：``q_corr(object, aligned_*) ⊗ ee_base``。

    ``aligned_object_yaw`` / ``aligned_object_roll``：数值，或 ``auto``（吸附到最近的 ``k·π/2``）。
    ``aligned_object_roll`` 仅在 ``object_orientation_mode`` 为 ``yaw_roll`` 时生效。
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
    roll0 = 0.0
    if _mode_uses_roll(object_orientation_mode):
        roll0 = resolve_aligned_object_roll(aligned_object_roll, object_pose)
    q_corr = filter_object_orientation_xyzw(
        object_orientation_xyzw(object_pose),
        object_orientation_mode,
        aligned_object_yaw=yaw0,
        aligned_object_roll=roll0,
    )
    return quat_normalize(quat_multiply(q_corr, ee_base_orientation))
