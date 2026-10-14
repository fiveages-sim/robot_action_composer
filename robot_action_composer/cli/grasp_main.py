#!/usr/bin/env python3
"""CLI: grasp-generation — modular grasp assist beside ros2-stack / motion-generation.

Aligned with ``motion-generation`` UX:

* Interactive (default)::

      cd examples/IsaacSim
      grasp-generation                 # pick robot (reads robots/*/grasp_generation.yaml)
      grasp-generation --robot fiveages_w2

* Scripted subcommands (CI / smoke)::

      grasp-generation serve --robot fiveages_w2 --workspace .
      grasp-generation show-config --robot fiveages_w2

Does **not** replace motion-generation task queues. Expect ``ros2-stack`` already up.
Uses a dedicated ROS node (``grasp_generation_<robot>``) so arm targets are not
shared with a concurrent motion CLI interface instance.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

from robot_action_composer.cli.interactive import select_from_list, select_yes_no
from robot_action_composer.cli.motion_main import resolve_workspace_dir
from robot_action_composer.grasp_generation.config import (
    discover_grasp_robots,
    load_grasp_config,
)

_GRASP_LAST_FILENAME = ".grasp_last.json"


def _grasp_last_path(workspace: Path) -> Path:
    return workspace / _GRASP_LAST_FILENAME


def _load_grasp_last(workspace: Path) -> dict[str, Any]:
    path = _grasp_last_path(workspace)
    if not path.is_file():
        return {}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return {}
    return dict(raw) if isinstance(raw, dict) else {}


def _save_grasp_last(workspace: Path, **updates: Any) -> None:
    data = _load_grasp_last(workspace)
    data.update({k: v for k, v in updates.items() if v is not None})
    try:
        path = _grasp_last_path(workspace)
        path.write_text(
            json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    except OSError:
        pass


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
        default=None,
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
        description=(
            "Grasp generation (HUG + dex hand close). "
            "Interactive by default; use subcommands for scripts. "
            "Separate from motion-generation."
        ),
    )
    _add_common(parser)
    sub = parser.add_subparsers(dest="command", required=False)

    p_serve = sub.add_parser("serve", help="Viser UI: capture → HUG → wrist → close")
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
    p_follow.add_argument("--proposal", type=str, required=True)
    p_follow.add_argument("--execute", action="store_true")

    p_show = sub.add_parser("show-config", help="Print merged grasp_generation config")
    _add_common(p_show)

    p_diag = sub.add_parser("diagnose-frames", help="TF/EE dump vs wrist plan")
    _add_common(p_diag)
    p_diag.add_argument("--plan", type=str, default=None)

    p_orient = sub.add_parser(
        "diagnose-orientation", help="MANO wrist axes vs robot tip"
    )
    _add_common(p_orient)
    p_orient.add_argument("--proposal", type=str, default=None)

    p_retarget = sub.add_parser(
        "retarget-hand", help="Proposal → hand q (dry-run default)"
    )
    _add_common(p_retarget)
    p_retarget.add_argument("--proposal", type=str, default=None)
    p_retarget.add_argument("--execute", action="store_true")
    p_retarget.add_argument("--no-reset", action="store_true")
    p_retarget.add_argument("--squeeze", type=float, default=None)

    p_close = sub.add_parser("close-hand", help="Run B0–B5 close executor")
    _add_common(p_close)
    p_close.add_argument("--mode", type=str, default=None)
    p_close.add_argument("--proposal", type=str, default=None)
    p_close.add_argument("--squeeze", type=float, default=None)
    p_close.add_argument("--trial-id", type=str, default="")

    p_auto = sub.add_parser(
        "auto",
        help="Privileged auto-grasp: object → capture → HUG → move → close",
    )
    _add_common(p_auto)
    p_auto.add_argument(
        "--object",
        type=str,
        required=True,
        help="Key from grasp_generation.yaml grasp_objects (e.g. apple)",
    )
    p_auto.add_argument(
        "--dry-run",
        action="store_true",
        help="Capture + propose + dry plan only (no wrist/close)",
    )
    p_auto.add_argument(
        "--execute",
        action="store_true",
        help="Actually move wrist and run close (default: dry wrist unless set)",
    )
    p_auto.add_argument(
        "--no-close",
        action="store_true",
        help="Skip B4 close even when --execute",
    )

    p_dbg = sub.add_parser(
        "dump-hug-debug",
        help="Save RGB + purple UV + green MANO overlay from a capture dir",
    )
    _add_common(p_dbg)
    p_dbg.add_argument(
        "--from-dir",
        type=str,
        default=None,
        help="Capture folder (default: cfg capture_dir)",
    )
    p_dbg.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Output folder (default: <from-dir>/hug_debug)",
    )
    p_dbg.add_argument("--proposal", type=str, default=None)

    return parser.parse_args(argv)


def _resolve_robot_key(
    *,
    workspace: Path,
    cli_robot: Optional[str],
    interactive: bool,
) -> str:
    robots = discover_grasp_robots(workspace)
    last = _load_grasp_last(workspace)
    default = (
        cli_robot
        or (last.get("robot_key") if isinstance(last.get("robot_key"), str) else None)
        or "fiveages_w2"
    )
    if cli_robot:
        if robots and cli_robot not in robots:
            print(
                f"[grasp-generation] warn: no grasp_generation.yaml for {cli_robot!r}; "
                f"known: {sorted(robots)}",
                file=sys.stderr,
            )
        return cli_robot
    if not interactive:
        return default if default in robots or not robots else next(iter(robots))
    if not robots:
        raise SystemExit(
            "[grasp-generation] No robots/*/grasp_generation.yaml under "
            f"{workspace / 'robots'}. Add a config (see FiveAges_W2 template)."
        )
    registry = {
        k: {
            "label": v["label"],
            "robot_dir_relpath": str(
                Path(v["robot_dir"]).relative_to(workspace / "robots")
            ),
        }
        for k, v in robots.items()
    }
    from robot_action_composer.cli.interactive import select_robot_from_registry

    if default not in registry:
        default = next(iter(registry))
    return select_robot_from_registry(
        title="Select robot (grasp_generation.yaml)",
        registry=registry,
        default_key=default,
        show_label=True,
    )


def _build_cfg(
    *,
    workspace: Path,
    robot_key: str,
    args: argparse.Namespace,
) -> Any:
    overrides = {
        "viser_port": getattr(args, "port", None),
        "arm": getattr(args, "arm", None),
        "position_only": getattr(args, "position_only", None),
    }
    config_path = Path(args.config) if getattr(args, "config", None) else None
    return load_grasp_config(
        workspace=workspace,
        robot_key=robot_key,
        config_path=config_path,
        overrides=overrides,
    )


def _run_serve(cfg: Any, workspace: Path, *, connect_interface: bool) -> None:
    from robot_action_composer.grasp_generation.service import run_serve

    print("[grasp-generation] workspace=", workspace)
    print("[grasp-generation] robot=", cfg.robot_key, "port=", cfg.viser_port)
    print(
        "[grasp-generation] close_executor=",
        cfg.close_executor,
        "hand_profile=",
        cfg.hand_profile,
        "pre_close_lift_m=",
        cfg.pre_close_lift_m,
    )
    print(
        "[grasp-generation] Expect ros2-stack already running. "
        "Do not run motion-generation task execute concurrently with grasp execute_move."
    )
    run_serve(cfg=cfg, workspace=workspace, connect_interface=connect_interface)


def _interactive_main(args: argparse.Namespace, workspace: Path) -> None:
    print("\n" + "=" * 70)
    print("grasp-generation (modular; separate from motion-generation)")
    print("=" * 70)
    print(f"workspace: {workspace}")
    robots = discover_grasp_robots(workspace)
    if robots:
        print("grasp-capable robots:", ", ".join(sorted(robots)))
    else:
        print("grasp-capable robots: (none — add robots/<Robot>/grasp_generation.yaml)")

    robot_key = _resolve_robot_key(
        workspace=workspace, cli_robot=args.robot, interactive=True
    )
    _save_grasp_last(workspace, robot_key=robot_key)
    cfg = _build_cfg(workspace=workspace, robot_key=robot_key, args=args)

    actions = [
        "serve (Viser UI)",
        "show-config",
        "quit",
    ]
    action = select_from_list(
        title="Select action",
        options=actions,
        default_value=actions[0],
    )
    if action.startswith("quit"):
        return
    if action.startswith("show-config"):
        from dataclasses import asdict

        print(json.dumps(asdict(cfg), indent=2))
        print("hug_python:", cfg.resolved_hug_python(workspace))
        print("capture_dir:", cfg.resolved_capture_dir(workspace))
        yml = discover_grasp_robots(workspace).get(robot_key, {}).get("config_path")
        if yml:
            print("config_path:", yml)
        return

    connect = select_yes_no(
        title="Connect ROS2RobotInterface? (needed for Plan / Retarget / Close)",
        default_yes=True,
    )
    _run_serve(cfg, workspace, connect_interface=connect)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    workspace = resolve_workspace_dir(
        Path(args.workspace) if args.workspace else None
    )

    # No subcommand → interactive (motion-generation style)
    if not args.command:
        _interactive_main(args, workspace)
        return

    robot_key = _resolve_robot_key(
        workspace=workspace,
        cli_robot=args.robot or "fiveages_w2",
        interactive=False,
    )
    _save_grasp_last(workspace, robot_key=robot_key)
    cfg = _build_cfg(workspace=workspace, robot_key=robot_key, args=args)

    if args.command == "show-config":
        from dataclasses import asdict

        print(json.dumps(asdict(cfg), indent=2))
        print("hug_python:", cfg.resolved_hug_python(workspace))
        print("capture_dir:", cfg.resolved_capture_dir(workspace))
        return

    if args.command == "serve":
        _run_serve(cfg, workspace, connect_interface=not args.no_interface)
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

    if args.command == "close-hand":
        from robot_action_composer.grasp_generation.execution import (
            ensure_q_retarget,
            run_close,
        )
        from robot_action_composer.grasp_generation.interface_util import (
            connect_grasp_interface,
            disconnect_grasp_interface,
        )

        arm = (args.arm or cfg.arm or "right").lower()
        prop = (
            Path(args.proposal)
            if args.proposal
            else cfg.resolved_capture_dir(workspace) / "proposal.json"
        )
        q_ret = ensure_q_retarget(
            cfg=cfg,
            workspace=workspace,
            proposal_path=prop,
            arm=arm,
            cached=None,
        )
        iface = connect_grasp_interface(robot_key=cfg.robot_key)
        try:
            result = run_close(
                cfg=cfg,
                interface=iface,
                q_retarget=q_ret,
                arm=arm,
                squeeze_amount=args.squeeze,
                executor_id=args.mode,
                workspace=workspace,
                trial_id=str(args.trial_id or ""),
                log=True,
            )
        finally:
            disconnect_grasp_interface(iface)
        print(json.dumps(result.summary_row(), indent=2))
        raise SystemExit(0 if result.ok else 1)

    if args.command == "auto":
        from robot_action_composer.grasp_generation.auto_grasp import run_auto_grasp_cli

        code = run_auto_grasp_cli(
            cfg=cfg,
            workspace=workspace,
            object_key=str(args.object),
            dry_run=bool(args.dry_run),
            execute=bool(args.execute) and not bool(args.dry_run),
            do_close=not bool(args.no_close),
        )
        raise SystemExit(code)

    if args.command == "dump-hug-debug":
        from robot_action_composer.grasp_generation.dump_hug_debug import (
            dump_hug_debug_from_capture,
        )

        from_dir = (
            Path(args.from_dir)
            if args.from_dir
            else cfg.resolved_capture_dir(workspace)
        )
        meta = dump_hug_debug_from_capture(
            from_dir,
            out_dir=Path(args.out_dir) if args.out_dir else None,
            proposal_path=Path(args.proposal) if args.proposal else None,
        )
        print(json.dumps(meta, indent=2))
        raise SystemExit(0)

    raise SystemExit(f"unknown command {args.command}")


if __name__ == "__main__":
    main()
