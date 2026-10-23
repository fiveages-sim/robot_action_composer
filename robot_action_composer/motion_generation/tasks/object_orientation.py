"""物体姿态滤波与「对齐标定」末端姿态合成。

抓取配置里的 ``ee_base_orientation`` / 工具系偏移，按「某一标称机物相对朝向」标定
（常见：机物 X 对齐，或导航到位后固定相对偏航，例如 ``0 / ±π/2 / ±π`` 一档）。

运行时只补偿**相对该标称的偏差**：

``yaw`` 模式（默认）::

    yaw_corr = wrap(yaw_object − aligned_object_yaw)
    q_ee = normalize(R_z(yaw_corr) ⊗ q_ee_base)

``yaw_roll`` 模式（相对标称 SO(3) 残差）::

    q_nom  = euler_rpy(aligned_roll, aligned_pitch, aligned_yaw)   # auto → 各角吸附到 k·π/2
    q_corr = q_obj ⊗ conjugate(q_nom)   # = R_obj · R_nom^{-1}，勿再拆成独立 RPY 残差重装
    q_ee   = normalize(q_corr ⊗ q_ee_base)

说明：把 ``(roll−roll0, pitch−pitch0, yaw−yaw0)`` 再 ``euler(...)`` 拼回去，在 yaw≈π/2 时会把
物体 body-pitch 拧到错误的运动系轴（看起来像「pitch 映射成了 roll」）。相对四元数才能让
夹爪运动轴继续贴住标定时对齐的那根物体 ±轴。

``aligned_object_yaw`` / ``aligned_object_roll`` / ``aligned_object_pitch``：

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
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

_AUTO_ALIGNED_ANGLE_TOKENS = frozenset(
    {"auto", "snap", "snap_pi_2", "nearest", "nearest_pi_2"}
)
# Back-compat alias
_AUTO_ALIGNED_YAW_TOKENS = _AUTO_ALIGNED_ANGLE_TOKENS

_YAW_ROLL_MODES = frozenset({"yaw_roll", "yaw_and_roll", "heading_roll"})

_OBJECT_AXIS_LOCAL: tuple[tuple[str, tuple[float, float, float]], ...] = (
    ("+X", (1.0, 0.0, 0.0)),
    ("-X", (-1.0, 0.0, 0.0)),
    ("+Y", (0.0, 1.0, 0.0)),
    ("-Y", (0.0, -1.0, 0.0)),
    ("+Z", (0.0, 0.0, 1.0)),
    ("-Z", (0.0, 0.0, -1.0)),
)

_PICK_AXIS_LOCAL: dict[str, tuple[float, float, float]] = {
    "+x": (1.0, 0.0, 0.0),
    "x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}


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
    """解析标称角：数值直用；``auto`` 则按物体当前 roll/pitch/yaw 吸附到最近 π/2 档。"""
    if aligned is None:
        return 0.0
    if isinstance(aligned, str):
        key = aligned.strip().lower()
        if key in _AUTO_ALIGNED_ANGLE_TOKENS:
            roll, pitch, yaw = quat_xyzw_to_rpy(object_orientation_xyzw(object_pose))
            if component == "roll":
                value = roll
            elif component == "pitch":
                value = pitch
            else:
                value = yaw
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


def resolve_aligned_object_pitch(
    aligned_object_pitch: float | str | None,
    object_pose: Any,
) -> float:
    """解析标称物体 pitch：数值直用；``auto`` 则按物体当前 pitch 吸附到最近 π/2 档。"""
    return _resolve_aligned_angle(aligned_object_pitch, object_pose, component="pitch")


def _mode_uses_roll(mode: str) -> bool:
    return str(mode or "").strip().lower() in _YAW_ROLL_MODES


def _pick_axis_local(pick_axis: str = "+z") -> tuple[float, float, float]:
    key = str(pick_axis or "+z").strip().lower()
    if key not in _PICK_AXIS_LOCAL:
        raise ValueError(
            f"unsupported pick_axis {pick_axis!r}; expected one of {sorted(_PICK_AXIS_LOCAL)}"
        )
    return _PICK_AXIS_LOCAL[key]


def ee_pick_axis_in_motion(
    ee_base_orientation: tuple[float, float, float, float],
    *,
    pick_axis: str = "+z",
) -> tuple[float, float, float]:
    """``ee_base`` 下工具系 ``pick_axis`` 在运动系中的单位方向。"""
    return rotate_vector_by_quat(_pick_axis_local(pick_axis), quat_normalize(ee_base_orientation))


def nearest_object_axis_to_ee_pick(
    ee_base_orientation: tuple[float, float, float, float],
    object_pose: Any,
    *,
    pick_axis: str = "+z",
) -> tuple[str, float, tuple[float, float, float]]:
    """夹爪运动轴（默认工具 +Z）在 ``ee_base`` 下，与物体哪根 ±轴最对齐。

    返回 ``(axis_label, cos_similarity, pick_dir_motion)``。
    例如 rotate blade 的 ``ee_base=(0,1,0,0)``：工具 +Z → 运动系 −Z，
    物体 identity 时最近轴为 ``-Z``（cos=1）。
    """
    pick_dir = ee_pick_axis_in_motion(ee_base_orientation, pick_axis=pick_axis)
    q_obj = object_orientation_xyzw(object_pose)
    best_label = "+Z"
    best_dot = -2.0
    for label, local in _OBJECT_AXIS_LOCAL:
        axis_motion = rotate_vector_by_quat(local, q_obj)
        dot = (
            pick_dir[0] * axis_motion[0]
            + pick_dir[1] * axis_motion[1]
            + pick_dir[2] * axis_motion[2]
        )
        if dot > best_dot:
            best_dot = float(dot)
            best_label = label
    return best_label, best_dot, pick_dir


def filter_object_orientation_xyzw(
    q_obj: tuple[float, float, float, float],
    mode: str,
    *,
    aligned_object_yaw: float = 0.0,
    aligned_object_roll: float = 0.0,
    aligned_object_pitch: float = 0.0,
) -> tuple[float, float, float, float]:
    """按 ``object_orientation_mode`` 过滤物体姿态（运动系下），并扣除标称角。

    - ``full`` / ``object``：完整物体姿态，再左乘 ``R_z(-aligned_object_yaw)`` 得到相对标称的姿态
    - ``yaw`` / ``yaw_only``：仅 ``wrap(yaw − aligned_object_yaw)``（导航后机物竖直、只差朝向时的推荐模式）
    - ``yaw_roll`` / ``yaw_and_roll`` / ``heading_roll``：``q_obj ⊗ conjugate(q_nom)``，
      相对标称同时补偿完整 SO(3) 残差（勿用独立 RPY 残差重装）
    - ``tilt`` / ``no_yaw``：roll+pitch，yaw=0（不扣标称偏航）
    - ``pitch``：仅 pitch
    - ``none`` / ``identity`` / ``motion``：单位姿态（不跟物体）
    """
    mode_norm = str(mode or "full").strip().lower()
    yaw0 = float(aligned_object_yaw)
    roll0 = float(aligned_object_roll)
    pitch0 = float(aligned_object_pitch)

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
    if mode_norm in _YAW_ROLL_MODES:
        # R_corr = R_obj · R_nom^{-1}；yaw≈π/2 时独立 RPY 残差重装会把 body-pitch 拧错轴
        q_nom = euler_rpy_to_quat_xyzw(roll0, pitch0, yaw0)
        return quat_normalize(quat_multiply(quat_normalize(q_obj), quat_conjugate(q_nom)))
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
    aligned_object_pitch: float | str = "auto",
) -> Pose:
    """返回仅姿态被过滤、位置不变的副本。"""
    yaw0 = resolve_aligned_object_yaw(aligned_object_yaw, pose)
    roll0 = 0.0
    pitch0 = 0.0
    if _mode_uses_roll(mode):
        roll0 = resolve_aligned_object_roll(aligned_object_roll, pose)
        pitch0 = resolve_aligned_object_pitch(aligned_object_pitch, pose)
    q = filter_object_orientation_xyzw(
        object_orientation_xyzw(pose),
        mode,
        aligned_object_yaw=yaw0,
        aligned_object_roll=roll0,
        aligned_object_pitch=pitch0,
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
    aligned_object_yaw: float | str = "auto",
    aligned_object_roll: float | str = "auto",
    aligned_object_pitch: float | str = "auto",
    object_prim_path: str = "",
    pick_axis: str = "+z",
) -> tuple[float, float, float, float]:
    """将「标称机物相对朝向」下标定的 ``ee_base_orientation`` 变换到当前物体朝向。

    - ``ee_orientation_frame=motion``：直接返回标定姿态（旧行为）。
    - ``ee_orientation_frame=object``：``q_corr(object, aligned_*) ⊗ ee_base``。

    ``object_pose`` 必须是 ``object_prim_path`` 刚体在运动系下的位姿（``/get_entity_state``），
    **不要**传入已叠加 grasp ``object_position_offset`` 后的抓取点位姿（位置可不同，但朝向
    来源必须是刚体 prim）。

    ``aligned_object_yaw`` / ``aligned_object_roll`` / ``aligned_object_pitch``：
    数值，或 ``auto``（吸附到最近的 ``k·π/2``）。
    pitch/roll 仅在 ``object_orientation_mode`` 为 ``yaw_roll`` 时生效。
    """
    frame = str(ee_orientation_frame or "motion").strip().lower()
    ee_base = quat_normalize(ee_base_orientation)
    if frame in ("", "motion", "base", "world"):
        return ee_base
    if frame not in ("object", "object_body", "body", "local"):
        raise ValueError(
            f"unsupported ee_orientation_frame {ee_orientation_frame!r}; "
            "expected 'motion' or 'object'"
        )
    yaw0 = resolve_aligned_object_yaw(aligned_object_yaw, object_pose)
    roll0 = 0.0
    pitch0 = 0.0
    if _mode_uses_roll(object_orientation_mode):
        roll0 = resolve_aligned_object_roll(aligned_object_roll, object_pose)
        pitch0 = resolve_aligned_object_pitch(aligned_object_pitch, object_pose)
    q_obj = object_orientation_xyzw(object_pose)
    q_corr = filter_object_orientation_xyzw(
        q_obj,
        object_orientation_mode,
        aligned_object_yaw=yaw0,
        aligned_object_roll=roll0,
        aligned_object_pitch=pitch0,
    )
    if _mode_uses_roll(object_orientation_mode) or str(object_orientation_mode).strip().lower() in (
        "yaw",
        "yaw_only",
        "heading",
        "full",
        "object",
    ):
        roll, pitch, yaw = quat_xyzw_to_rpy(q_obj)
        cr, cp, cy = quat_xyzw_to_rpy(q_corr)
        axis_label, axis_cos, pick_dir = nearest_object_axis_to_ee_pick(
            ee_base, object_pose, pick_axis=pick_axis
        )
        prim_label = str(object_prim_path or "").strip() or "<object_pose>"
        print(
            f"[ObjectOrient] mode={object_orientation_mode} prim={prim_label} "
            f"obj_rpy=({roll:.3f},{pitch:.3f},{yaw:.3f}) "
            f"aligned=({roll0:.3f},{pitch0:.3f},{yaw0:.3f}) "
            f"corr_rpy=({cr:.3f},{cp:.3f},{cy:.3f}) "
            f"ee_pick({pick_axis})->obj_axis={axis_label}(cos={axis_cos:.3f}) "
            f"pick_motion=({pick_dir[0]:.3f},{pick_dir[1]:.3f},{pick_dir[2]:.3f})"
        )
    return quat_normalize(quat_multiply(q_corr, ee_base))
