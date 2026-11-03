"""Pure insert planning: axis alignment, rim / approach / deep poses, correction."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Pose
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
    rotate_vector_by_quat_inverse,
)

from .geometry import (
    Vec3,
    angle_deg,
    axis_vector_from_label,
    normalize,
    pose_quat_xyzw,
    quat_between_vectors,
    tilt_deg,
    vec_add,
    vec_norm,
    vec_scale,
    vec_sub,
    xyz,
)

Quat4 = tuple[float, float, float, float]


@dataclass(frozen=True)
class AlignedInsertPose:
    """EE pose that puts the held object bottom on the rim with aligned axis."""

    rim: Vec3
    orientation: Quat4
    correction_deg: float
    object_axis_after: Vec3


def align_ee_to_object_axis(
    *,
    ee_pose: Pose,
    held_pose: Pose,
    rim_bottom: Vec3,
    desired_object_axis: Vec3,
    held_axis_label: str = "+z",
    base_ee_orientation: Quat4 | None = None,
) -> AlignedInsertPose:
    """Re-orient the EE so the held object's local axis matches the insert axis.

    The rigid grip transform (object in EE frame) is measured from the current
    poses, then the minimal rotation aligning ``held_axis`` to
    ``desired_object_axis`` is applied while preserving yaw about that axis.
    The EE position is recomputed so the object bottom stays on ``rim_bottom``.
    """
    q_ee_now = quat_normalize(pose_quat_xyzw(ee_pose))
    q_obj = quat_normalize(pose_quat_xyzw(held_pose))
    q_ee_obj = quat_normalize(quat_multiply(quat_conjugate(q_ee_now), q_obj))
    p_ee_obj = rotate_vector_by_quat_inverse(vec_sub(xyz(held_pose), xyz(ee_pose)), q_ee_now)

    q_base = quat_normalize(
        base_ee_orientation if base_ee_orientation is not None else q_ee_now
    )
    q_obj_base = quat_normalize(quat_multiply(q_base, q_ee_obj))
    axis_now = rotate_vector_by_quat(axis_vector_from_label(held_axis_label), q_obj_base)
    desired = normalize(desired_object_axis)

    correction = quat_between_vectors(axis_now, desired)
    q_obj_new = quat_normalize(quat_multiply(correction, q_obj_base))
    q_ee_new = quat_normalize(quat_multiply(q_obj_new, quat_conjugate(q_ee_obj)))
    p_ee_new = vec_sub(rim_bottom, rotate_vector_by_quat(p_ee_obj, q_ee_new))
    axis_after = rotate_vector_by_quat(axis_vector_from_label(held_axis_label), q_obj_new)
    return AlignedInsertPose(
        rim=p_ee_new,
        orientation=q_ee_new,
        correction_deg=angle_deg(axis_now, desired),
        object_axis_after=normalize(axis_after),
    )


@dataclass(frozen=True)
class InsertGeometry:
    """EE poses (execution frame) for a precise insertion."""

    rim: Vec3
    approach: Vec3
    deep: Vec3
    insert_dir: Vec3
    tilt_deg: float


def compute_insert_geometry(
    *,
    rim: Vec3,
    insert_dir: Vec3,
    insert_depth: float,
    approach_distance: float,
    tube_axis_w: Vec3 | None = None,
) -> InsertGeometry:
    """Build the three EE waypoints for ``approach → rim → deep``.

    ``insert_dir`` is the **insertion direction** (into the hole), expressed in
    the execution frame. ``approach`` is backed off opposite to it.
    """
    depth = float(insert_depth)
    if depth <= 1e-6 or depth > 0.3:
        raise ValueError(f"insert_depth must be in (0, 0.3] m, got {depth}")
    hover = float(approach_distance)
    if hover < 0.0 or hover > 0.5:
        raise ValueError(f"approach_distance must be in [0, 0.5] m, got {hover}")
    direction = normalize(insert_dir)
    approach = vec_sub(rim, vec_scale(direction, hover))
    deep = vec_add(rim, vec_scale(direction, depth))
    tilt = tilt_deg(tube_axis_w, direction) if tube_axis_w is not None else 0.0
    return InsertGeometry(
        rim=rim,
        approach=approach,
        deep=deep,
        insert_dir=direction,
        tilt_deg=tilt,
    )


def retreat_point(*, deep: Vec3, insert_dir: Vec3, distance: float) -> Vec3:
    """Point after backing out of the hole along the insert axis."""
    dist = float(distance)
    if dist < 0.0:
        raise ValueError(f"retreat_distance must be >= 0, got {dist}")
    return vec_sub(deep, vec_scale(normalize(insert_dir), dist))


def align_correction(
    *,
    target_bottom_rim: Vec3,
    bottom_now: Vec3,
    max_corr: float,
) -> Vec3:
    """Translation that moves the measured object bottom onto the rim point."""
    corr = vec_sub(target_bottom_rim, bottom_now)
    norm = vec_norm(corr)
    limit = float(max_corr)
    if limit > 0.0 and norm > limit:
        raise RuntimeError(
            "single_arm.insert: alignment correction "
            f"{norm:.4f} m exceeds max_align_corr={limit:.4f} m "
            f"(bottom_now={bottom_now}, target_rim={target_bottom_rim}); "
            "refusing to insert blind."
        )
    return corr
