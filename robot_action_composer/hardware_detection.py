"""Detect whether the current ROS robot is simulation or real hardware."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Literal

MotionEnvironment = Literal["sim", "real"]

SIMULATION_ROS2_CONTROL_PLUGINS = {
    "topic_based_ros2_control/TopicBasedSystem",
    "gz_ros2_control/GazeboSimSystem",
    # "mock_components/GenericSystem",
}


def extract_ros2_control_plugins(robot_description: str) -> list[str]:
    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as exc:
        raise ValueError(f"Failed to parse robot_description XML: {exc}") from exc

    plugins: list[str] = []
    for ros2_control in root.findall(".//ros2_control"):
        plugin_el = ros2_control.find("./hardware/plugin")
        if plugin_el is None or plugin_el.text is None:
            continue
        plugin = plugin_el.text.strip()
        if plugin:
            plugins.append(plugin)
    return plugins


def classify_motion_environment_from_plugins(plugins: list[str]) -> MotionEnvironment:
    if any(plugin in SIMULATION_ROS2_CONTROL_PLUGINS for plugin in plugins):
        return "sim"
    return "real"


def read_robot_description_from_ros(*, timeout_sec: float = 5.0) -> str:
    import time

    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node("motion_generation_hardware_detector")
    try:
        deadline = time.monotonic() + timeout_sec

        client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.1):
                future = client.call_async(GetParameters.Request(names=["robot_description"]))
                while time.monotonic() < deadline and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.05)
                if future.done():
                    result = future.result()
                    if result is not None and result.values:
                        description = result.values[0].string_value
                        if description.strip():
                            return description
                break

        received: dict[str, str] = {}

        def callback(msg: String) -> None:
            if msg.data.strip():
                received["robot_description"] = msg.data

        subscription = node.create_subscription(String, "/robot_description", callback, 1)
        try:
            while time.monotonic() < deadline and "robot_description" not in received:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_subscription(subscription)

        if "robot_description" in received:
            return received["robot_description"]

        raise RuntimeError(
            "Could not read robot_description from /robot_state_publisher parameter "
            "or /robot_description topic within timeout"
        )
    finally:
        node.destroy_node()


def detect_motion_environment_from_ros(*, timeout_sec: float = 5.0) -> MotionEnvironment:
    robot_description = read_robot_description_from_ros(timeout_sec=timeout_sec)
    plugins = extract_ros2_control_plugins(robot_description)
    if not plugins:
        raise RuntimeError("robot_description contains no ros2_control hardware plugins")
    environment = classify_motion_environment_from_plugins(plugins)
    print(f"[Hardware] ros2_control plugins={plugins} -> motion_environment={environment}")
    return environment
