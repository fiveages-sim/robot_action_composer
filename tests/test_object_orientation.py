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


def test_yaw_roll_mode_includes_both() -> None:
    aligned_yaw = 0.0
    aligned_roll = 0.0
    pose = _pose_from_rpy(0.4, 0.1, 0.2)
    q_corr = oo.filter_object_orientation_xyzw(
        oo.object_orientation_xyzw(pose),
        "yaw_roll",
        aligned_object_yaw=aligned_yaw,
        aligned_object_roll=aligned_roll,
    )
    roll, pitch, yaw = oo.quat_xyzw_to_rpy(q_corr)
    assert roll == pytest.approx(0.4, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(0.2, abs=1e-6)


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
    # Object near π/2 roll and small yaw → aligned roll=π/2, yaw=0; corr ≈ (roll−π/2, 0, yaw)
    pose = _pose_from_rpy(0.5 * math.pi + 0.15, 0.0, 0.12)
    ee_base = (0.0, 0.0, 0.0, 1.0)
    out = oo.compose_aligned_ee_orientation(
        ee_base,
        pose,
        ee_orientation_frame="object",
        object_orientation_mode="yaw_roll",
        aligned_object_yaw="auto",
        aligned_object_roll="auto",
    )
    roll, pitch, yaw = oo.quat_xyzw_to_rpy(out)
    assert pitch == pytest.approx(0.0, abs=1e-6)
    assert roll == pytest.approx(0.15, abs=1e-5)
    assert yaw == pytest.approx(0.12, abs=1e-5)
