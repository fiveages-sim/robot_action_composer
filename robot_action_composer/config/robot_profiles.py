"""Robot profile types: motion (action composer) vs lerobot (record / inference)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ros2_robot_interface import ROS2RobotInterfaceConfig


def default_motion_ros2_interface(
    *,
    pose_position_threshold: float = 0.02,
    pose_orientation_threshold: float = 0.05,
) -> ROS2RobotInterfaceConfig:
    """Minimal ROS2 interface config; ``connect()`` auto-detects topics and bimanual layout."""
    from ros2_robot_interface import ROS2RobotInterfaceConfig

    return ROS2RobotInterfaceConfig(
        pose_position_threshold=pose_position_threshold,
        pose_orientation_threshold=pose_orientation_threshold,
    )


@dataclass(frozen=True)
class MotionRobotConfig:
    """Fields required by task queue / motion generation (no LeRobot dependency)."""

    ros2_interface: ROS2RobotInterfaceConfig
    gripper_control_mode: str = "target_command"
    base_link_entity_path: str = ""
    fsm_switch_delay: float = 0.1
    post_reset_wait: float = 1.0
    arrival_timeout: float = 3.0
    arrival_poll: float = 0.05
    gripper_action_wait: float = 0.3


@dataclass(frozen=True)
class CameraTopicConfig:
    """Neutral camera topic description (does not import lerobot)."""

    topic_name: str
    node_name: str
    depth_topic_name: str | None = None


@dataclass(frozen=True)
class LeRobotRobotConfig:
    """Camera and robot id for dataset recording / online inference."""

    robot_id: str
    cameras: dict[str, CameraTopicConfig]
    depth_camera_name: str = ""
    depth_info_topic: str = ""
