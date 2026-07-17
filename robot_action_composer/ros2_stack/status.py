"""Detect whether ros2_stack components are ready via ROS graph nodes."""

from __future__ import annotations

from typing import Sequence

from robot_action_composer.ros2_stack.config import ReadyWhen, ResolvedComponent


def list_ros_node_names() -> list[str]:
    """Return full node names currently visible on the ROS graph."""
    try:
        from ros2_robot_interface.utils.discovery import list_nodes
    except ImportError as err:
        raise ImportError(
            "ros2_stack status requires ros2_robot_interface (and a sourced ROS 2 env)"
        ) from err
    nodes = list_nodes()
    names: list[str] = []
    for n in nodes:
        full = n.get("full_name") or ""
        name = n.get("name") or ""
        if full:
            names.append(str(full))
        if name:
            names.append(str(name))
    return names


def ready_when_satisfied(ready: ReadyWhen, node_names: Sequence[str]) -> bool:
    if not ready.node_name_contains:
        return False
    haystacks = [str(n) for n in node_names]
    hits = [any(sub in n for n in haystacks) for sub in ready.node_name_contains]
    if ready.match == "all":
        return all(hits)
    return any(hits)


def component_is_ready(component: ResolvedComponent, node_names: Sequence[str] | None = None) -> bool:
    names = list(node_names) if node_names is not None else list_ros_node_names()
    return ready_when_satisfied(component.ready_when, names)


__all__ = [
    "component_is_ready",
    "list_ros_node_names",
    "ready_when_satisfied",
]
