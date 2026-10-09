"""Wuji Hand2 execution profile (primary hand for B0–B5 tables)."""

from __future__ import annotations

from ...retarget.squeeze import apply_squeeze, open_hand_q
from ...retarget.wuji_hand2 import WUJI_HAND2_JOINT_SUFFIXES
from .base import HandExecutionProfile

# Joint groups match SDK index 0..19 (see wuji_hand2_protocol.hpp).
_FINGER_JOINTS = {
    "thumb": (0, 1, 2, 3),
    "index": (4, 5, 6, 7),
    "middle": (8, 9, 10, 11),
    "ring": (12, 13, 14, 15),
    "pinky": (16, 17, 18, 19),
}

# Keep curling these after contact (MCP frozen) — wraps lime-sized objects.
_TIP_JOINTS = {
    "thumb": (2, 3),  # mcp + ip
    "index": (6, 7),  # pip + dip
    "middle": (10, 11),
    "ring": (14, 15),
    "pinky": (18, 19),
}

# PhysX ContactSensor links published by FaSim Sensor=contact_force.
# Distal is the tip; OR proximal/middle so early shaft contact also gates.
_CONTACT_LINKS = {
    "thumb": ("thumb_distal", "thumb_middle", "thumb_proximal"),
    "index": (
        "index_finger_distal",
        "index_finger_middle",
        "index_finger_proximal",
    ),
    "middle": (
        "middle_finger_distal",
        "middle_finger_middle",
        "middle_finger_proximal",
    ),
    "ring": ("ring_distal", "ring_middle", "ring_proximal"),
    "pinky": ("pinky_distal", "pinky_middle", "pinky_proximal"),
}

WUJI_HAND2_EXECUTION_PROFILE = HandExecutionProfile(
    name="wuji_hand2",
    n_dof=20,
    finger_joint_indices=_FINGER_JOINTS,
    contact_links=_CONTACT_LINKS,
    topic_prefix_by_arm={"right": "wuji_rh", "left": "wuji_lh"},
    joint_suffixes=WUJI_HAND2_JOINT_SUFFIXES,
    tip_joint_indices=_TIP_JOINTS,
    apply_squeeze_fn=apply_squeeze,
    open_hand_fn=open_hand_q,
)
