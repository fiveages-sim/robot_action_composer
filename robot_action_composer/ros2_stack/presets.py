"""Built-in motion presets and navigation profiles for ros2_stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class MotionPresetSpec:
    package: str
    launch_file: str
    ready_node_substrings: tuple[str, ...]


MOTION_PRESETS: Mapping[str, MotionPresetSpec] = {
    "ocs2-fullbody": MotionPresetSpec(
        package="ocs2_arm_controller",
        launch_file="full_body.launch.py",
        ready_node_substrings=("ocs2_wbc_controller",),
    ),
    "ocs2-split-body": MotionPresetSpec(
        package="ocs2_arm_controller",
        launch_file="split_body.launch.py",
        ready_node_substrings=("ocs2_arm_controller",),
    ),
    "ocs2-demo": MotionPresetSpec(
        package="ocs2_arm_controller",
        launch_file="demo.launch.py",
        ready_node_substrings=("ocs2_arm_controller",),
    ),
}

NAV_PROFILES: frozenset[str] = frozenset({"default", "map_only"})

DEFAULT_NAV_PACKAGE = "robot_common_launch"
DEFAULT_NAV_LAUNCH_FILE = "navigation_isaac_gt.launch.py"
DEFAULT_NAV_READY_SUBSTRINGS: tuple[str, ...] = ("controller_server",)

_FORBIDDEN_STACK_KEYS: frozenset[str] = frozenset(
    {
        "task_key",
        "task_queue",
        "skill_defaults",
        "scene_presets",
        "runtime_defaults",
        "label",
        "default_scene",
        "use_stamped",
        "record",
    }
)


def list_motion_preset_keys() -> list[str]:
    return list(MOTION_PRESETS.keys())


def resolve_motion_preset(name: str) -> MotionPresetSpec:
    key = str(name).strip()
    if key not in MOTION_PRESETS:
        raise ValueError(
            f"Unknown motion.preset {key!r}; expected one of {sorted(MOTION_PRESETS)}"
        )
    return MOTION_PRESETS[key]


def validate_nav_profile(name: str) -> str:
    key = str(name).strip()
    if key not in NAV_PROFILES:
        raise ValueError(
            f"Unknown navigation.profile {key!r}; expected one of {sorted(NAV_PROFILES)}"
        )
    return key


__all__ = [
    "DEFAULT_NAV_LAUNCH_FILE",
    "DEFAULT_NAV_PACKAGE",
    "DEFAULT_NAV_READY_SUBSTRINGS",
    "MOTION_PRESETS",
    "MotionPresetSpec",
    "NAV_PROFILES",
    "list_motion_preset_keys",
    "resolve_motion_preset",
    "validate_nav_profile",
    "_FORBIDDEN_STACK_KEYS",
]
