"""Unit tests for pure insert planning (no ROS calls)."""

from __future__ import annotations

import math

import pytest
from geometry_msgs.msg import Pose

from robot_action_composer.task_runtime.placement import (
    align_correction,
    align_ee_to_object_axis,
    compute_insert_geometry,
    object_axis_in_world,
    retreat_point,
    tilt_deg,
    vec_norm,
    vec_sub,
)


def _pose_with_quat(qx: float, qy: float, qz: float, qw: float) -> Pose:
    p = Pose()
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def _approx(a: tuple[float, ...], b: tuple[float, ...], tol: float = 1e-9) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b, strict=True))


def test_insert_axis_comes_from_target_object_orientation() -> None:
    # +90° about X maps object-local +Z to (0, -1, 0)
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    pose = _pose_with_quat(s, 0.0, 0.0, c)

    up = object_axis_in_world(pose, "+z")
    down = object_axis_in_world(pose, "-z")

    assert _approx(up, (0.0, -1.0, 0.0), 1e-9)
    assert _approx(down, (0.0, 1.0, 0.0), 1e-9)
    assert tilt_deg(up, down) == pytest.approx(0.0)


def test_insert_geometry_uses_object_axis_for_approach_and_depth() -> None:
    geom = compute_insert_geometry(
        rim=(1.0, 2.0, 3.0),
        insert_dir=(0.0, 0.0, -1.0),
        insert_depth=0.06,
        approach_distance=0.08,
    )

    assert _approx(geom.approach, (1.0, 2.0, 3.08))
    assert _approx(geom.rim, (1.0, 2.0, 3.0))
    assert _approx(geom.deep, (1.0, 2.0, 2.94))
    assert _approx(geom.insert_dir, (0.0, 0.0, -1.0))


def test_insert_geometry_rejects_bad_depth() -> None:
    with pytest.raises(ValueError):
        compute_insert_geometry(
            rim=(0.0, 0.0, 0.0),
            insert_dir=(0.0, 0.0, -1.0),
            insert_depth=0.0,
            approach_distance=0.05,
        )
    with pytest.raises(ValueError):
        compute_insert_geometry(
            rim=(0.0, 0.0, 0.0),
            insert_dir=(0.0, 0.0, -1.0),
            insert_depth=0.31,
            approach_distance=0.05,
        )


def test_align_ee_to_object_axis_levels_held_axis_and_keeps_bottom_on_rim() -> None:
    s = math.sin(math.pi / 24)  # half-angle → 15° rotation about X
    c = math.cos(math.pi / 24)

    ee = Pose()
    ee.position.x, ee.position.y, ee.position.z = 0.0, 0.0, 0.0
    ee.orientation.w = 1.0

    held = Pose()
    held.position.x, held.position.y, held.position.z = 0.0, 0.0, -0.1
    held.orientation.x = s
    held.orientation.w = c

    aligned = align_ee_to_object_axis(
        ee_pose=ee,
        held_pose=held,
        rim_bottom=(1.0, 2.0, 3.0),
        desired_object_axis=(0.0, 0.0, 1.0),
    )

    assert aligned.correction_deg == pytest.approx(15.0, abs=1e-6)
    assert _approx(aligned.object_axis_after, (0.0, 0.0, 1.0), 1e-6)
    # the object bottom is 0.1 m from the EE, so the EE must stay 0.1 m from the rim
    assert vec_norm(vec_sub(aligned.rim, (1.0, 2.0, 3.0))) == pytest.approx(0.1, abs=1e-6)


def test_retreat_point_moves_opposite_the_insert_axis() -> None:
    deep = (1.0, 2.0, 2.94)
    retreat = retreat_point(deep=deep, insert_dir=(0.0, 0.0, -1.0), distance=0.08)
    assert _approx(retreat, (1.0, 2.0, 3.02))
    # zero distance stays at the seat
    assert retreat_point(deep=deep, insert_dir=(0.0, 0.0, -1.0), distance=0.0) == deep


def test_align_correction_returns_delta_and_guards_limit() -> None:
    corr = align_correction(
        target_bottom_rim=(1.0, 0.0, 0.0),
        bottom_now=(0.99, 0.0, 0.0),
        max_corr=0.03,
    )
    assert _approx(corr, (0.01, 0.0, 0.0), 1e-6)

    with pytest.raises(RuntimeError):
        align_correction(
            target_bottom_rim=(1.0, 0.0, 0.0),
            bottom_now=(0.9, 0.0, 0.0),
            max_corr=0.03,
        )


def test_tilt_deg_treats_opposite_axes_as_aligned() -> None:
    assert tilt_deg((0.0, 0.0, 1.0), (0.0, 0.0, -1.0)) == pytest.approx(0.0)
    assert tilt_deg((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)) == pytest.approx(90.0)
