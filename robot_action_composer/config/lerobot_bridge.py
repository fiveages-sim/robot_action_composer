"""Convert motion + lerobot profiles into LeRobot ROS2 plugin types (lazy imports)."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from robot_action_composer.config.robot_profiles import CameraTopicConfig, LeRobotRobotConfig, MotionRobotConfig


def camera_topic_to_ros2(cam: CameraTopicConfig, *, fps: int | None = None) -> Any:
    from lerobot_camera_ros2 import ROS2CameraConfig  # pyright: ignore[reportMissingImports]

    kwargs: dict[str, Any] = {
        "topic_name": cam.topic_name,
        "node_name": cam.node_name,
    }
    if cam.depth_topic_name:
        kwargs["depth_topic_name"] = cam.depth_topic_name
    if fps is not None:
        kwargs["fps"] = fps
    return ROS2CameraConfig(**kwargs)


def build_ros2_robot_config(
    *,
    motion_cfg: MotionRobotConfig,
    lerobot_cfg: LeRobotRobotConfig,
    fps: int = 30,
    robot_id_suffix: str = "",
) -> Any:
    from lerobot_robot_ros2 import ROS2RobotConfig  # pyright: ignore[reportMissingImports]

    camera_config = {
        name: camera_topic_to_ros2(cam, fps=fps) for name, cam in lerobot_cfg.cameras.items()
    }
    robot_id = lerobot_cfg.robot_id
    if robot_id_suffix:
        robot_id = f"{robot_id}{robot_id_suffix}"
    return ROS2RobotConfig(
        id=robot_id,
        cameras=camera_config,
        ros2_interface=motion_cfg.ros2_interface,
        gripper_control_mode=motion_cfg.gripper_control_mode,
    )


def lerobot_profile_from_legacy(robot_cfg: Any) -> LeRobotRobotConfig | None:
    """Build :class:`LeRobotRobotConfig` from a legacy combined ``robot_config`` module."""
    cameras_raw = getattr(robot_cfg, "cameras", None) or {}
    if not cameras_raw:
        return None
    cameras: dict[str, CameraTopicConfig] = {}
    for name, cam in cameras_raw.items():
        cameras[name] = CameraTopicConfig(
            topic_name=str(getattr(cam, "topic_name", "")),
            node_name=str(getattr(cam, "node_name", "")),
            depth_topic_name=getattr(cam, "depth_topic_name", None),
        )
    robot_id = getattr(robot_cfg, "robot_id", None) or getattr(robot_cfg, "ROBOT_KEY", "robot")
    return LeRobotRobotConfig(
        robot_id=str(robot_id),
        cameras=cameras,
        depth_camera_name=str(getattr(robot_cfg, "depth_camera_name", "") or ""),
        depth_info_topic=str(getattr(robot_cfg, "depth_info_topic", "") or ""),
    )


def motion_profile_from_legacy(robot_cfg: Any) -> MotionRobotConfig:
    """Extract motion fields from a legacy combined config object."""
    return MotionRobotConfig(
        ros2_interface=robot_cfg.ros2_interface,
        gripper_control_mode=str(getattr(robot_cfg, "gripper_control_mode", "target_command")),
        base_link_entity_path=str(getattr(robot_cfg, "base_link_entity_path", "") or ""),
        fsm_switch_delay=float(getattr(robot_cfg, "fsm_switch_delay", 0.1)),
        post_reset_wait=float(getattr(robot_cfg, "post_reset_wait", 1.0)),
        arrival_timeout=float(getattr(robot_cfg, "arrival_timeout", 3.0)),
        arrival_poll=float(getattr(robot_cfg, "arrival_poll", 0.05)),
        gripper_action_wait=float(getattr(robot_cfg, "gripper_action_wait", 0.3)),
    )


def apply_fps_to_cameras(camera_config: dict[str, Any], fps: int) -> dict[str, Any]:
    return {name: replace(cfg, fps=fps) for name, cfg in camera_config.items()}
