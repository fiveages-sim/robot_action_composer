"""任务运行时：末端位姿读取与 ``motion_frame_id`` 变换（单臂 / 双臂 skill 共用）。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Pose


def clone_pose(p: Pose) -> Pose:
    q = Pose()
    q.position.x = p.position.x
    q.position.y = p.position.y
    q.position.z = p.position.z
    q.orientation.x = p.orientation.x
    q.orientation.y = p.orientation.y
    q.orientation.z = p.orientation.z
    q.orientation.w = p.orientation.w
    return q


def pose_quat_xyzw(p: Pose) -> tuple[float, float, float, float]:
    return (
        float(p.orientation.x),
        float(p.orientation.y),
        float(p.orientation.z),
        float(p.orientation.w),
    )


def set_pose_quat_xyzw(p: Pose, q: tuple[float, float, float, float]) -> None:
    x, y, z, w = q
    p.orientation.x = x
    p.orientation.y = y
    p.orientation.z = z
    p.orientation.w = w


def param_vec3(
    params: Mapping[str, Any],
    key: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    v = params.get(key)
    if v is None:
        return default
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"{key} must be a length-3 list [x, y, z]")
    return (float(v[0]), float(v[1]), float(v[2]))


def motion_frame_id_from_params(
    params: Mapping[str, Any],
    *,
    pose_frame: str,
) -> str:
    """从 skill params 解析 ``motion_frame_id``（兼容 ``relative_frame_id``）。"""
    raw = params.get("motion_frame_id", params.get("relative_frame_id"))
    if raw is None or (isinstance(raw, str) and not str(raw).strip()):
        return pose_frame
    return str(raw).strip()


def tf_lookup_timeout_from_params(params: Mapping[str, Any], *, default: float = 2.0) -> float:
    try:
        return float(params.get("tf_lookup_timeout", default))
    except (TypeError, ValueError):
        return default


def handler_frame_id_or_none(handler: Any) -> str | None:
    """单臂 handler 的 ``frame_id``；无则 ``None``。"""
    gf = getattr(handler, "get_frame_id", None)
    if not callable(gf):
        return None
    raw = gf()
    if raw is None:
        return None
    s = str(raw).strip()
    return s or None


def handler_pose_source_frame(handler: Any, fallback_frame_id: str) -> str:
    """单臂：推断 ``handler.get_pose()`` 数值所在坐标系。"""
    fid = handler_frame_id_or_none(handler)
    if fid:
        return fid
    return str(fallback_frame_id).strip() or "base_link"


def bimanual_pose_source_frame(
    left_handler: Any,
    right_handler: Any,
    ctx_frame_id: str,
    *,
    label: str,
) -> str:
    """双臂：推断左右 ``get_pose()`` 共用的源坐标系。"""
    lf = handler_frame_id_or_none(left_handler) if left_handler else None
    rf = handler_frame_id_or_none(right_handler) if right_handler else None
    fallback = str(ctx_frame_id).strip() or "base_link"
    if lf and rf and lf != rf:
        raise RuntimeError(f"{label}: left EE frame_id {lf!r} != right EE frame_id {rf!r}")
    if lf:
        return lf
    if rf:
        return rf
    return fallback


def pose_in_motion_frame(
    iface: Any,
    pose_raw: Pose,
    pose_frame: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[Pose, str]:
    """将单个末端位姿变换到 ``motion_frame_id``（相同时拷贝）。"""
    motion_frame = motion_frame_id_from_params(params, pose_frame=pose_frame)
    tft = tf_lookup_timeout_from_params(params)
    if motion_frame == pose_frame:
        return clone_pose(pose_raw), motion_frame
    if not hasattr(iface, "transform_pose"):
        raise TypeError(f"{label}: motion_frame_id requires ROS2RobotInterface.transform_pose (TF buffer)")
    out = iface.transform_pose(pose_raw, pose_frame, motion_frame, timeout=tft)
    if out is None:
        raise RuntimeError(
            f"{label}: TF {pose_frame!r} -> {motion_frame!r} failed (timeout={tft}s)."
        )
    return out, motion_frame


def bimanual_poses_in_motion_frame(
    iface: Any,
    left_pose_raw: Pose,
    right_pose_raw: Pose,
    pose_frame: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[Pose, Pose, str]:
    """将左右末端位姿变换到 ``motion_frame_id``。"""
    motion_frame = motion_frame_id_from_params(params, pose_frame=pose_frame)
    tft = tf_lookup_timeout_from_params(params)
    if motion_frame == pose_frame:
        return clone_pose(left_pose_raw), clone_pose(right_pose_raw), motion_frame
    if not hasattr(iface, "transform_pose"):
        raise TypeError(f"{label}: motion_frame_id requires ROS2RobotInterface.transform_pose (TF buffer)")
    l0 = iface.transform_pose(left_pose_raw, pose_frame, motion_frame, timeout=tft)
    r0 = iface.transform_pose(right_pose_raw, pose_frame, motion_frame, timeout=tft)
    if l0 is None or r0 is None:
        raise RuntimeError(
            f"{label}: TF {pose_frame!r} -> {motion_frame!r} failed (timeout={tft}s). "
            "Check frames and that the interface is connected with TF listener."
        )
    return l0, r0, motion_frame


def read_current_ee_pose_in_motion_frame(
    iface: Any,
    handler: Any,
    fallback_frame_id: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[Pose, str]:
    """读取当前末端位姿并变换到 ``motion_frame_id``。"""
    pose_raw = handler.get_pose()
    if pose_raw is None:
        raise RuntimeError(f"{label}: could not read current EE pose")
    pose_frame = handler_pose_source_frame(handler, fallback_frame_id)
    return pose_in_motion_frame(iface, pose_raw, pose_frame, params, label=label)


def current_ee_orientation_xyzw(
    iface: Any,
    handler: Any,
    fallback_frame_id: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[tuple[float, float, float, float], str]:
    """未配置放置姿态时：返回当前末端四元数（``motion_frame_id`` 下）。"""
    pose_mf, motion_frame = read_current_ee_pose_in_motion_frame(
        iface, handler, fallback_frame_id, params, label=label
    )
    return pose_quat_xyzw(pose_mf), motion_frame
