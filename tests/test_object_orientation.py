"""Unit tests for object orientation alignment (no ROS runtime required)."""

from __future__ import annotations

import importlib.util
import math
import sys
import types
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[1]
_OO_PATH = _ROOT / "robot_action_composer" / "motion_generation" / "tasks" / "object_orientation.py"
_QUAT_PATH = (
    _ROOT.parent / "ros2_robot_interface" / "ros2_robot_interface" / "utils" / "quat_pose.py"
)


def _install_stubs() -> None:
    """Provide geometry_msgs + ros2_robot_interface.utils.quat_pose without a ROS env."""
    if "geometry_msgs.msg" not in sys.modules:
        geo = types.ModuleType("geometry_msgs")
        geo_msg = types.ModuleType("geometry_msgs.msg")

        class _XYZ:
            def __init__(self) -> None:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        class _XYZW:
            def __init__(self) -> None:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.w = 1.0

        class Pose:
            def __init__(self) -> None:
                self.position = _XYZ()
                self.orientation = _XYZW()

        geo_msg.Pose = Pose
        geo.msg = geo_msg
        sys.modules["geometry_msgs"] = geo
        sys.modules["geometry_msgs.msg"] = geo_msg

    if "ros2_robot_interface.utils.quat_pose" not in sys.modules:
        # Load real quat_pose after geometry_msgs stub is in place.
        pkg = types.ModuleType("ros2_robot_interface")
        utils = types.ModuleType("ros2_robot_interface.utils")
        sys.modules["ros2_robot_interface"] = pkg
        sys.modules["ros2_robot_interface.utils"] = utils
        spec = importlib.util.spec_from_file_location(
            "ros2_robot_interface.utils.quat_pose", _QUAT_PATH
        )
        assert spec is not None and spec.loader is not None
        mod = importlib.util.module_from_spec(spec)
        sys.modules["ros2_robot_interface.utils.quat_pose"] = mod
        spec.loader.exec_module(mod)
        utils.quat_pose = mod


def _load_object_orientation():
    name = "rac_object_orientation_under_test"
    if name in sys.modules:
        return sys.modules[name]
    _install_stubs()
    spec = importlib.util.spec_from_file_location(name, _OO_PATH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


oo = _load_object_orientation()


def _pose_from_rpy(roll: float, pitch: float, yaw: float):
    from geometry_msgs.msg import Pose

    from ros2_robot_interface.utils.quat_pose import euler_rpy_to_quat_xyzw

    q = euler_rpy_to_quat_xyzw(roll, pitch, yaw)
    p = Pose()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def test_snap_angle_to_nearest_pi_half() -> None:
    assert oo.snap_angle_to_nearest_pi_half(0.1) == pytest.approx(0.0)
    assert oo.snap_angle_to_nearest_pi_half(0.8) == pytest.approx(0.5 * math.pi)
    assert oo.snap_yaw_to_nearest_pi_half(-0.1) == pytest.approx(0.0)


def test_resolve_aligned_object_roll_auto() -> None:
    pose = _pose_from_rpy(0.3, 0.0, 0.0)
    assert oo.resolve_aligned_object_roll("auto", pose) == pytest.approx(0.0)
    pose2 = _pose_from_rpy(0.9, 0.0, 0.0)
    assert oo.resolve_aligned_object_roll("auto", pose2) == pytest.approx(0.5 * math.pi)


def test_yaw_mode_ignores_roll() -> None:
    # Object has roll + yaw; yaw mode should only encode yaw_corr.
    aligned_yaw = 0.0
    pose = _pose_from_rpy(0.4, 0.0, 0.2)
    q_corr = oo.filter_object_orientation_xyzw(
        oo.object_orientation_xyzw(pose),
        "yaw",
        aligned_object_yaw=aligned_yaw,
        aligned_object_roll=0.0,
    )
    roll, pitch, yaw = oo.quat_xyzw_to_rpy(q_corr)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(0.2, abs=1e-6)


def test_yaw_roll_mode_near_identity_matches_rpy() -> None:
    """标称为单位时 q_corr = q_obj，RPY 与物体一致。"""
    pose = _pose_from_rpy(0.4, 0.1, 0.2)
    q_corr = oo.filter_object_orientation_xyzw(
        oo.object_orientation_xyzw(pose),
        "yaw_roll",
        aligned_object_yaw=0.0,
        aligned_object_roll=0.0,
        aligned_object_pitch=0.0,
    )
    roll, pitch, yaw = oo.quat_xyzw_to_rpy(q_corr)
    assert roll == pytest.approx(0.4, abs=1e-6)
    assert pitch == pytest.approx(0.1, abs=1e-6)
    assert yaw == pytest.approx(0.2, abs=1e-6)


def test_yaw_roll_relative_quat_not_euler_residuals() -> None:
    """yaw≈π/2 时物体 pitch tip → 相对残差应落在 roll（勿用独立 RPY 残差重装成 pitch）。"""
    import sys

    name = "rac_object_orientation_under_test"
    if name in sys.modules:
        del sys.modules[name]
    mod = _load_object_orientation()

    pose = _pose_from_rpy(0.0, 0.3, 0.5 * math.pi)
    ee_base = (0.0, 0.0, 0.0, 1.0)
    out = mod.compose_aligned_ee_orientation(
        ee_base,
        pose,
        ee_orientation_frame="object",
        object_orientation_mode="yaw_roll",
        aligned_object_yaw="auto",
        aligned_object_roll="auto",
        aligned_object_pitch="auto",
    )
    roll, pitch, yaw = mod.quat_xyzw_to_rpy(out)
    assert abs(roll) == pytest.approx(0.3, abs=1e-5)
    assert abs(pitch) < 1e-5
    assert abs(yaw) < 1e-4

    # 合成后夹爪轴仍精确贴住某一物体主轴（identity ee_base → 物体 +Z）
    label, cos_sim, _ = mod.nearest_object_axis_to_ee_pick(out, pose)
    assert label == "+Z"
    assert cos_sim == pytest.approx(1.0, abs=1e-6)


def test_rotate_blade_ee_base_nearest_object_axis_is_minus_z() -> None:
    """rotate blade ``ee_base=(0,1,0,0)``：工具 +Z → 运动 −Z，与物体 −Z 最匹配。"""
    import sys

    name = "rac_object_orientation_under_test"
    if name in sys.modules:
        del sys.modules[name]
    mod = _load_object_orientation()

    ee_base = (0.0, 1.0, 0.0, 0.0)
    pose = _pose_from_rpy(0.0, 0.0, 0.5 * math.pi)
    label, cos_sim, pick_dir = mod.nearest_object_axis_to_ee_pick(ee_base, pose)
    assert label == "-Z"
    assert cos_sim == pytest.approx(1.0, abs=1e-6)
    assert pick_dir == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)


def test_yaw_roll_keeps_pick_on_object_axis_with_ee_base() -> None:
    """带 ee_base 时相对四元数仍让 pick 跟住标定时对齐的物体主轴。"""
    import sys

    name = "rac_object_orientation_under_test"
    if name in sys.modules:
        del sys.modules[name]
    mod = _load_object_orientation()

    ee_base = (0.0, 1.0, 0.0, 0.0)
    pose = _pose_from_rpy(0.0, 0.3, 0.5 * math.pi)
    out = mod.compose_aligned_ee_orientation(
        ee_base,
        pose,
        ee_orientation_frame="object",
        object_orientation_mode="yaw_roll",
        aligned_object_yaw="auto",
        aligned_object_roll="auto",
        aligned_object_pitch="auto",
    )
    label, cos_sim, _ = mod.nearest_object_axis_to_ee_pick(out, pose)
    assert label == "-Z"
    assert cos_sim == pytest.approx(1.0, abs=1e-6)


def test_compose_motion_frame_returns_ee_base() -> None:
    ee_base = (0.0, 1.0, 0.0, 0.0)
    pose = _pose_from_rpy(0.5, 0.0, 0.7)
    out = oo.compose_aligned_ee_orientation(
        ee_base,
        pose,
        ee_orientation_frame="motion",
        object_orientation_mode="yaw_roll",
        aligned_object_yaw="auto",
        aligned_object_roll="auto",
    )
    assert out == pytest.approx(ee_base, abs=1e-9)


def test_compose_yaw_roll_auto_snaps_then_corrects() -> None:
    # Object near π/2 roll and small yaw → aligned roll=π/2；
    # 相对 SO(3) 后夹爪轴仍贴住标称时对齐的那根物轴（此处为 +Y）。
    import sys

    name = "rac_object_orientation_under_test"
    if name in sys.modules:
        del sys.modules[name]
    mod = _load_object_orientation()

    pose = _pose_from_rpy(0.5 * math.pi + 0.15, 0.0, 0.12)
    ee_base = (0.0, 0.0, 0.0, 1.0)
    out = mod.compose_aligned_ee_orientation(
        ee_base,
        pose,
        ee_orientation_frame="object",
        object_orientation_mode="yaw_roll",
        aligned_object_yaw="auto",
        aligned_object_roll="auto",
        aligned_object_pitch="auto",
    )
    label, cos_sim, _ = mod.nearest_object_axis_to_ee_pick(out, pose)
    assert label == "+Y"
    assert cos_sim == pytest.approx(1.0, abs=1e-5)
