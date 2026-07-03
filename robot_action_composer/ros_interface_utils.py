"""Construct :class:`ROS2RobotInterface` from robot configs (no LeRobot dependency)."""

from __future__ import annotations

from typing import Any

from geometry_msgs.msg import Pose
from ros2_robot_interface import (  # pyright: ignore[reportMissingImports]
    ROS2RobotInterface,
    ROS2RobotInterfaceConfig,
)


def build_ros2_interface_from_robot_cfg(robot_cfg: Any) -> ROS2RobotInterface:
    """Build interface from ``robot_cfg.ros2_interface`` (a :class:`ROS2RobotInterfaceConfig`)."""
    cfg = getattr(robot_cfg, "ros2_interface", None)
    if cfg is None:
        cfg = ROS2RobotInterfaceConfig()
    return ROS2RobotInterface(cfg)


def arm_handler_pose_or_raise(handler: Any, *, label: str = "arm") -> Pose:
    """Current EE pose from an arm handler; raises if handler or pose is missing."""
    if handler is None:
        raise RuntimeError(f"No arm handler for {label}")
    pose = handler.get_pose()
    if pose is None:
        raise RuntimeError(
            f"No end-effector pose received yet for {label}; wait for pose topic subscription."
        )
    return pose
