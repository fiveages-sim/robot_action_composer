"""Place / insert planning helpers shared by queue skills.

The package is intentionally ROS-light: pure geometry lives in
:mod:`~robot_action_composer.task_runtime.placement.geometry` and
:mod:`~robot_action_composer.task_runtime.placement.insert`; ROS/TF access is
isolated in :mod:`.frames` / :mod:`.held`.
"""

from __future__ import annotations

from .frames import (  # noqa: F401
    feedback_pose_as_eef,
    pick_tf_timeout,
    resolve_ee_frames,
    stages_in_command_ee_frame,
)
from .geometry import (  # noqa: F401
    axis_vector_from_label,
    object_axis_in_world,
    pose_xyz_in_frame,
    tilt_deg,
    vec_add,
    vec_norm,
    vec_scale,
    vec_sub,
    xyz,
)
from .held import resolve_held_object_pose  # noqa: F401
from .insert import (  # noqa: F401
    AlignedInsertPose,
    InsertGeometry,
    align_correction,
    align_ee_to_object_axis,
    compute_insert_geometry,
    retreat_point,
)

__all__ = [
    "AlignedInsertPose",
    "InsertGeometry",
    "align_correction",
    "align_ee_to_object_axis",
    "axis_vector_from_label",
    "compute_insert_geometry",
    "feedback_pose_as_eef",
    "object_axis_in_world",
    "pick_tf_timeout",
    "pose_xyz_in_frame",
    "resolve_ee_frames",
    "resolve_held_object_pose",
    "retreat_point",
    "stages_in_command_ee_frame",
    "tilt_deg",
    "vec_add",
    "vec_norm",
    "vec_scale",
    "vec_sub",
    "xyz",
]
