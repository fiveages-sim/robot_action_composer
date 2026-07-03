"""Load per-robot ``robot.yaml`` into :class:`MotionRobotConfig`."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_action_composer.config.robot_profiles import MotionRobotConfig, default_motion_ros2_interface

_MOTION_FIELD_NAMES: frozenset[str] = frozenset(
    {
        "gripper_control_mode",
        "base_link_entity_path",
        "fsm_switch_delay",
        "post_reset_wait",
        "arrival_timeout",
        "arrival_poll",
        "gripper_action_wait",
    }
)
_ROS2_PRESETS: frozenset[str] = frozenset({"auto", "single_arm", "ocs2_single_arm"})


def _load_yaml_dict(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:
        raise ImportError(
            "Loading robot.yaml requires PyYAML. Install with: pip install pyyaml"
        ) from err
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        raise ValueError(f"robot.yaml is empty: {path}")
    if not isinstance(data, dict):
        raise TypeError(f"robot.yaml root must be a mapping, got {type(data).__name__}: {path}")
    return data


def _build_ros2_interface(section: dict[str, Any] | None) -> Any:
    from ros2_robot_interface import ROS2RobotInterfaceConfig

    raw = dict(section or {})
    preset = str(raw.pop("preset", "auto"))
    if preset not in _ROS2_PRESETS:
        raise ValueError(
            f"Unknown ros2_interface.preset {preset!r}; expected one of {sorted(_ROS2_PRESETS)}"
        )

    if preset == "auto":
        thresholds: dict[str, float] = {}
        for key in ("pose_position_threshold", "pose_orientation_threshold"):
            if key in raw:
                thresholds[key] = float(raw.pop(key))
        if raw:
            raise ValueError(
                f"ros2_interface preset=auto does not accept keys: {sorted(raw)}"
            )
        return default_motion_ros2_interface(**thresholds)

    if preset == "single_arm":
        return ROS2RobotInterfaceConfig.default_single_arm(**raw)

    return ROS2RobotInterfaceConfig.default_single_arm_ocs2_arm_controller(**raw)


def motion_profile_from_robot_yaml(path: Path) -> tuple[MotionRobotConfig, str, str]:
    """Parse ``robot.yaml`` → ``(MotionRobotConfig, robot_key, robot_label)``."""
    data = _load_yaml_dict(path)
    robot_key = data.get("key")
    robot_label = data.get("label")
    if not robot_key or not robot_label:
        raise ValueError(f"{path} must define non-empty 'key' and 'label'")

    motion_section = data.get("motion")
    if motion_section is not None and not isinstance(motion_section, dict):
        raise TypeError(f"{path}: 'motion' must be a mapping if present")

    ros2_section = data.get("ros2_interface")
    if ros2_section is not None and not isinstance(ros2_section, dict):
        raise TypeError(f"{path}: 'ros2_interface' must be a mapping if present")

    motion_kwargs: dict[str, Any] = {}
    for source in (data, motion_section or {}):
        for name in _MOTION_FIELD_NAMES:
            if name in source:
                motion_kwargs[name] = source[name]

    unknown_root = [
        k
        for k in data
        if k not in {"key", "label", "motion", "ros2_interface", *_MOTION_FIELD_NAMES}
    ]
    if unknown_root:
        raise ValueError(f"{path}: unknown root keys: {sorted(unknown_root)}")

    if motion_section:
        unknown_motion = [k for k in motion_section if k not in _MOTION_FIELD_NAMES]
        if unknown_motion:
            raise ValueError(f"{path}: unknown motion keys: {sorted(unknown_motion)}")

    motion_cfg = MotionRobotConfig(
        ros2_interface=_build_ros2_interface(ros2_section),
        **motion_kwargs,
    )
    return motion_cfg, str(robot_key), str(robot_label)
