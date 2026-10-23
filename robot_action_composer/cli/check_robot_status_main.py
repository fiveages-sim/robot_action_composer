#!/usr/bin/env python3
"""Print current robot joint positions and end-effector poses for task YAML authoring.

Examples::

    check-robot-status
    check-robot-status --robot fiveages_w2 --workspace examples/IsaacSim
    check-robot-status --wait 3.0
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any


def _fmt_list(values: list[float], *, decimals: int = 4) -> str:
    inner = ", ".join(f"{float(v):.{decimals}f}" for v in values)
    return f"[{inner}]"


def _pose_xyz(pose: Any) -> tuple[float, float, float]:
    pos = pose.position
    return float(pos.x), float(pos.y), float(pos.z)


def _pose_quat(pose: Any) -> tuple[float, float, float, float]:
    o = pose.orientation
    return float(o.x), float(o.y), float(o.z), float(o.w)


def _print_movej_snippet(categorized: dict[str, Any]) -> None:
    print("# joint.movej_to_config snippet (copy-paste into skill_defaults / params)")
    mapping = (
        ("body", "body_positions"),
        ("head", "head_positions"),
        ("left_arm", "left_arm_positions"),
        ("right_arm", "right_arm_positions"),
        ("arm", "arm_positions"),
    )
    printed = False
    for key, yaml_key in mapping:
        part = categorized.get(key) or {}
        positions = part.get("positions")
        if positions:
            print(f"{yaml_key}: {_fmt_list(list(positions))}")
            printed = True
    if not printed:
        print("# (no categorized joint positions available)")


def _print_gripper_snippet(categorized: dict[str, Any]) -> None:
    for key, yaml_key in (
        ("left_gripper", "left_gripper"),
        ("right_gripper", "right_gripper"),
        ("gripper", "gripper"),
    ):
        part = categorized.get(key) or {}
        positions = part.get("positions")
        if positions:
            print(f"{yaml_key}: {float(positions[0]):.4f}")


def _print_ee_snippet(
    *,
    label: str,
    pose: Any,
    frame_id: str | None,
) -> None:
    if pose is None:
        print(f"# {label}: (no pose received yet)")
        return
    xyz = _pose_xyz(pose)
    quat = _pose_quat(pose)
    frame = frame_id or "unknown"
    print(f"# {label} (frame: {frame})")
    print(f"position: {_fmt_list(list(xyz), decimals=6)}")
    print(f"ee_base_orientation: {_fmt_list(list(quat), decimals=6)}")


def _print_joint_details(categorized: dict[str, Any]) -> None:
    print("# joint names (debug)")
    for key in ("body", "head", "left_arm", "right_arm", "arm", "left_gripper", "right_gripper", "gripper"):
        part = categorized.get(key) or {}
        names = part.get("names")
        positions = part.get("positions")
        if not names:
            continue
        pos_str = _fmt_list(list(positions)) if positions else "[]"
        print(f"#   {key}: {', '.join(names)} = {pos_str}")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="check-robot-status",
        description="Query current robot joint positions and end-effector poses",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        default=None,
        help="Workspace root containing robots/ (default: current working directory)",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Robot key from robot.yaml (e.g. fiveages_w2); default: auto-detect ROS topics",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=2.0,
        help="Seconds to wait for joint state / EE pose after connect (default: 2)",
    )
    parser.add_argument(
        "--show-joint-names",
        action="store_true",
        help="Also print per-part joint names with positions",
    )
    return parser.parse_args(argv)


def _build_interface(*, workspace_dir: Any, robot_key: str | None) -> tuple[Any, str | None]:
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig  # pyright: ignore[reportMissingImports]

    from robot_action_composer.cli.motion_main import resolve_workspace_dir  # pyright: ignore[reportMissingImports]
    from robot_action_composer.discovery.registry_loader import load_motion_entries  # pyright: ignore[reportMissingImports]
    from robot_action_composer.ros_interface_utils import build_ros2_interface_from_robot_cfg  # pyright: ignore[reportMissingImports]

    if robot_key is None:
        return ROS2RobotInterface(ROS2RobotInterfaceConfig()), None

    workspace = resolve_workspace_dir(workspace_dir)
    registry = load_motion_entries(workspace)
    if robot_key not in registry:
        known = ", ".join(sorted(registry)) or "(none)"
        raise ValueError(f"Unknown robot key {robot_key!r}; known: {known}")
    robot_cfg = registry[robot_key]["robot_cfg"]
    return build_ros2_interface_from_robot_cfg(robot_cfg), robot_key


def main(argv: list[str] | None = None) -> None:
    raise SystemExit(_run(argv))


def _run(argv: list[str] | None = None) -> int:
    from pathlib import Path

    args = _parse_args(argv)
    interface = None
    try:
        interface, robot_key = _build_interface(
            workspace_dir=Path(args.workspace) if args.workspace else None,
            robot_key=args.robot,
        )
        interface.connect()
        wait_s = max(0.0, float(args.wait))
        if wait_s > 0.0:
            time.sleep(wait_s)

        is_dual = interface.config.right_end_effector_pose_topic is not None
        mode = "dual-arm" if is_dual else "single-arm"

        print("=== Robot status ===")
        if robot_key:
            print(f"robot: {robot_key}")
        print(f"mode:  {mode}")
        print()

        categorized = interface.get_joint_state(categorized=True) or {}
        body_target = interface.get_body_current_target()

        print("--- joints (movej) ---")
        if body_target and not (categorized.get("body") or {}).get("positions"):
            print(f"body_positions: {_fmt_list(list(body_target))}")
        _print_movej_snippet(categorized)
        print()

        print("--- grippers ---")
        _print_gripper_snippet(categorized)
        if not any(
            (categorized.get(k) or {}).get("positions")
            for k in ("left_gripper", "right_gripper", "gripper")
        ):
            print("# (no gripper joint state)")
        print()

        print("--- end-effector (Cartesian) ---")
        left_handler = interface.left_arm_handler
        right_handler = interface.right_arm_handler
        left_frame = left_handler.get_frame_id() if left_handler is not None else None
        right_frame = right_handler.get_frame_id() if right_handler is not None else None
        left_pose = left_handler.get_pose() if left_handler is not None else None
        right_pose = right_handler.get_pose() if right_handler is not None else None

        if is_dual:
            _print_ee_snippet(label="left", pose=left_pose, frame_id=left_frame)
            print()
            _print_ee_snippet(label="right", pose=right_pose, frame_id=right_frame)
        else:
            _print_ee_snippet(label="arm", pose=left_pose, frame_id=left_frame)
        print()

        if args.show_joint_names:
            print("--- details ---")
            _print_joint_details(categorized)
            print()

        if not categorized and left_pose is None and (not is_dual or right_pose is None):
            print(
                "warning: no joint state or EE pose received; "
                "ensure ROS 2 is running and controllers are publishing",
                file=sys.stderr,
            )
            return 1
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if interface is not None:
            try:
                interface.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
