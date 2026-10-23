"""Grasp-scoped ROS2RobotInterface helpers.

Grasp-generation runs as a **separate process** from ``motion-generation``.
It must not share the motion CLI's interface instance or task-queue state.

Each grasp session uses a dedicated ROS node name so publishers/subscribers
do not collide with a concurrent ``ros2_robot_interface`` used by motion
tools. Prefer one active writer to arm targets at a time (do not run a
motion task queue and grasp ``execute_move`` simultaneously).
"""

from __future__ import annotations

from typing import Any, Optional


def build_grasp_interface(
    *,
    robot_key: str = "fiveages_w2",
    node_name: Optional[str] = None,
) -> Any:
    """Create a ``ROS2RobotInterface`` owned by grasp-generation only."""
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

    key = "".join(c if c.isalnum() or c == "_" else "_" for c in str(robot_key))
    name = node_name or f"grasp_generation_{key}"
    cfg = ROS2RobotInterfaceConfig(node_name=name)
    return ROS2RobotInterface(cfg)


def connect_grasp_interface(
    *,
    robot_key: str = "fiveages_w2",
    node_name: Optional[str] = None,
    settle_s: float = 0.5,
) -> Any:
    """Connect and return a grasp-scoped interface."""
    import time

    iface = build_grasp_interface(robot_key=robot_key, node_name=node_name)
    print(f"[grasp] connecting ROS2RobotInterface node={iface.config.node_name!r} ...")
    iface.connect()
    if settle_s > 0:
        time.sleep(settle_s)
    return iface


def disconnect_grasp_interface(interface: Any) -> None:
    """Best-effort disconnect; does not touch motion-generation processes."""
    if interface is None:
        return
    try:
        interface.disconnect()
    except Exception:
        pass
