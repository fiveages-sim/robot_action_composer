#!/usr/bin/env python3
"""Quick Isaac Sim entity pose lookup via ``/get_entity_state``.

Examples::

    check-isaac-pose /World/robot/FiveAges_W2/LinkHou_S2/base_footprint/base_link
    check-isaac-pose /World/scene/boxes/white_box_05 --relative-to \\
        /World/robot/FiveAges_W2/LinkHou_S2/base_footprint/base_link
"""

from __future__ import annotations

import argparse
import math
import sys


def _quat_xyzw_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Convert quaternion xyzw to roll/pitch/yaw (rad), intrinsic XYZ."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _fmt_xyz(x: float, y: float, z: float) -> str:
    return f"[{x:.6f}, {y:.6f}, {z:.6f}]"


def _fmt_quat(qx: float, qy: float, qz: float, qw: float) -> str:
    return f"[{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]"


def _fmt_rpy(roll: float, pitch: float, yaw: float) -> str:
    return (
        f"[{roll:.6f}, {pitch:.6f}, {yaw:.6f}] rad  "
        f"([{math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}] deg)"
    )


def _print_pose(
    *,
    prim_path: str,
    frame_label: str,
    xyz: tuple[float, float, float],
    quat_xyzw: tuple[float, float, float, float],
) -> None:
    qx, qy, qz, qw = quat_xyzw
    rpy = _quat_xyzw_to_rpy(qx, qy, qz, qw)
    print(f"prim:  {prim_path}")
    print(f"frame: {frame_label}")
    print(f"  position xyz:        {_fmt_xyz(*xyz)}")
    print(f"  orientation xyzw:    {_fmt_quat(qx, qy, qz, qw)}")
    print(f"  orientation rpy:     {_fmt_rpy(*rpy)}")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="check-isaac-pose",
        description="Query Isaac Sim entity pose via /get_entity_state",
    )
    parser.add_argument(
        "prim_path",
        nargs="+",
        help="Isaac prim path(s), e.g. /World/robot/.../base_link",
    )
    parser.add_argument(
        "--relative-to",
        default=None,
        metavar="BASE_PRIM",
        help="Print pose in this entity's frame instead of world",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Service call timeout in seconds (default: 8)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    raise SystemExit(_run(argv))


def _run(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        get_entity_pose_world_service,
        get_object_pose_from_service,
    )

    timeout = float(args.timeout)
    base_world: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None = None
    if args.relative_to:
        base_world = get_entity_pose_world_service(str(args.relative_to), timeout=timeout)

    paths = [str(p) for p in args.prim_path]
    for i, path in enumerate(paths):
        if i:
            print()
        try:
            if base_world is None:
                xyz, quat = get_entity_pose_world_service(path, timeout=timeout)
                _print_pose(prim_path=path, frame_label="world", xyz=xyz, quat_xyzw=quat)
            else:
                pose = get_object_pose_from_service(
                    base_world[0],
                    base_world[1],
                    path,
                    include_orientation=True,
                    entity_state_timeout=timeout,
                )
                _print_pose(
                    prim_path=path,
                    frame_label=str(args.relative_to),
                    xyz=(pose.position.x, pose.position.y, pose.position.z),
                    quat_xyzw=(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
        except Exception as exc:
            print(f"prim:  {path}", file=sys.stderr)
            print(f"error: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    main()
