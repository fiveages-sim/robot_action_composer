"""Shared configuration types for robot profiles."""

from robot_action_composer.config.robot_profiles import (
    CameraTopicConfig,
    LeRobotRobotConfig,
    MotionRobotConfig,
    default_motion_ros2_interface,
)

__all__ = [
    "CameraTopicConfig",
    "LeRobotRobotConfig",
    "MotionRobotConfig",
    "default_motion_ros2_interface",
]
