#!/usr/bin/env python3
"""CLI: grasp-generation — HUG grasp assist beside ros2-stack (does not replace it).

Typical smoke::

    # terminal A (unchanged orchestration)
    cd examples/IsaacSim
    ros2-stack launch --robot fiveages_w2

    # terminal B (composer env with ROS + viser)
    grasp-generation serve --robot fiveages_w2 --workspace .
    # browser → Capture → set UV on object → Propose → Plan/move

Does **not** modify motion-generation task queues.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from robot_action_composer.cli.motion_main import resolve_workspace_dir
from robot_action_composer.grasp_generation.config import load_grasp_config


def _add_common(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--workspace",
        type=str,
        default=None,
        help="Workspace with robots/ (default: cwd; use examples/IsaacSim)",
    )
    p.add_argument(
        "--robot",
        type=str,
        default="fiveages_w2",
        help="Robot key (loads robots/.../grasp_generation.yaml)",
    )
    p.add_argument(
        "--config",
        type=str,
        default=None,
        help="Explicit grasp_generation.yaml path",
    )
    p.add_argument("--port", type=int, default=None, help="Viser port override")
    p.add_argument("--arm", type=str, choices=("left", "right"), default=None)
    p.add_argument(
        "--position-only",
        dest="position_only",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Follow wrist translation only (default from yaml)",
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="grasp-generation",
        description="HUG → wrist follow (uses ros2-stack motion; separate from motion-generation)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_serve = sub.add_parser("serve", help="Viser UI: capture → HUG → wrist plan/move")
    _add_common(p_serve)
    p_serve.add_argument(
        "--no-interface",
        action="store_true",
        help="Skip ROS2RobotInterface (capture/HUG only)",
    )

    p_follow = sub.add_parser(
        "follow-wrist",
        help="One-shot: proposal.json → T_base_wrist → dry-run or execute",
    )
    _add_common(p_follow)
    p_follow.add_argument(
        "--proposal",
        type=str,
        required=True,
        help="Path to proposal.json from HUG",
    )
    p_follow.add_argument(
        "--execute",
        action="store_true",
        help="Actually send_target_stamped (default: dry-run)",
    )

    p_show = sub.add_parser("show-config", help="Print merged grasp_generation config")
    _add_common(p_show)

    p_diag = sub.add_parser(
        "diagnose-frames",
        help="TF/EE dump: camera + right_tcp/eef vs HUG wrist_plan (Phase 2A.1)",
    )
    _add_common(p_diag)
    p_diag.add_argument(
        "--plan",
        type=str,
        default=None,
        help="wrist_plan.json (default: capture_dir/wrist_plan.json if present)",
    )

    p_orient = sub.add_parser(
        "diagnose-orientation",
        help="MANO wrist axes vs robot tcp→eef (debug palm-flip axis mismatch)",
    )
    _add_common(p_orient)
    p_orient.add_argument(
        "--proposal",
        type=str,
        default=None,
        help="proposal.json (default: capture_dir/proposal.json)",
    )

    p_retarget = sub.add_parser(
        "retarget-hand",
        help="HUG proposal landmarks → Wuji Hand2 q[20] (left/right; dry-run default)",
    )
    _add_common(p_retarget)
    p_retarget.add_argument(
        "--proposal",
        type=str,
        default=None,
        help="proposal.json (default: capture_dir/proposal.json)",
    )
    p_retarget.add_argument(
        "--execute",
        action="store_true",
        help="send_{arm}_hand_joint_positions (default: dry-run print + hand_retarget.json)",
    )
    p_retarget.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not call RetargetSession.reset() before step",
    )
    p_retarget.add_argument(
        "--squeeze",
        type=float,
        default=None,
        help="Flex squeeze amount 0..1 (default: yaml squeeze_amount)",
    )

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    workspace = resolve_workspace_dir(
        Path(args.workspace) if args.workspace else None
    )
    overrides = {
        "viser_port": args.port,
        "arm": args.arm,
        "position_only": args.position_only,
    }
    cfg = load_grasp_config(
        workspace=workspace,
        robot_key=args.robot,
        config_path=Path(args.config) if args.config else None,
        overrides=overrides,
    )

    if args.command == "show-config":
        from dataclasses import asdict

        print(json.dumps(asdict(cfg), indent=2))
        print("hug_python:", cfg.resolved_hug_python(workspace))
        print("capture_dir:", cfg.resolved_capture_dir(workspace))
        return

    if args.command == "serve":
        from robot_action_composer.grasp_generation.service import run_serve

        print("[grasp-generation] workspace=", workspace)
        print("[grasp-generation] robot=", cfg.robot_key, "port=", cfg.viser_port)
        print(
            "[grasp-generation] Expect ros2-stack motion already running "
            "(this CLI does not launch OCS2)."
        )
        run_serve(
            cfg=cfg,
            workspace=workspace,
            connect_interface=not args.no_interface,
        )
        return

    if args.command == "follow-wrist":
        from robot_action_composer.grasp_generation.service import run_follow_wrist_cli

        code = run_follow_wrist_cli(
            cfg=cfg,
            workspace=workspace,
            proposal=Path(args.proposal),
            execute=bool(args.execute),
        )
        raise SystemExit(code)

    if args.command == "diagnose-frames":
        from robot_action_composer.grasp_generation.diagnose_frames import (
            run_diagnose_frames,
        )

        code = run_diagnose_frames(
            cfg=cfg,
            workspace=workspace,
            plan_path=Path(args.plan) if args.plan else None,
        )
        raise SystemExit(code)

    if args.command == "diagnose-orientation":
        from robot_action_composer.grasp_generation.diagnose_frames import (
            run_diagnose_orientation,
        )

        code = run_diagnose_orientation(
            cfg=cfg,
            workspace=workspace,
            proposal_path=Path(args.proposal) if args.proposal else None,
        )
        raise SystemExit(code)

    if args.command == "retarget-hand":
        from robot_action_composer.grasp_generation.retarget_hand import (
            run_retarget_hand,
        )

        code = run_retarget_hand(
            cfg=cfg,
            workspace=workspace,
            proposal_path=Path(args.proposal) if args.proposal else None,
            execute=bool(args.execute),
            reset=not bool(args.no_reset),
            arm=args.arm,
            squeeze_amount=args.squeeze,
        )
        raise SystemExit(code)

    raise SystemExit(f"unknown command {args.command}")


if __name__ == "__main__":
    main()
