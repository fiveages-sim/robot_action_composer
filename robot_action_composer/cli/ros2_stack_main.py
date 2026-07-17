#!/usr/bin/env python3
"""CLI: ros2-stack status | start | stop — scenario ROS2 motion/navigation launches."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

from robot_action_composer.cli.i18n import Lang, normalize_lang, resolve_language, set_language, t
from robot_action_composer.cli.interactive import (
    select_option,
    select_robot_from_registry,
    select_task_group_drill_down,
    select_yes_no,
)
from robot_action_composer.cli.motion_main import resolve_workspace_dir
from robot_action_composer.discovery.registry_loader import load_motion_entries, resolve_robot_profile_dir
from robot_action_composer.ros2_stack.config import (
    ResolvedRos2Stack,
    apply_set_overrides,
    component_is_needed,
    load_merged_ros2_stack,
    resolve_ros2_stack,
    with_motion_preset,
    with_nav_profile,
)
from robot_action_composer.ros2_stack.ensure import prompt_stack_overrides_interactive
from robot_action_composer.ros2_stack.launcher import (
    ensure_components,
    list_component_logs,
    list_robots_with_live_managed_pids,
    read_managed_pids,
    stack_state_dir,
    stop_component,
)
from robot_action_composer.ros2_stack.presets import NAV_PROFILES, list_motion_preset_keys
from robot_action_composer.ros2_stack.status import component_is_ready, list_ros_node_names

_STACK_LAST_NAME = ".ros2_stack_last.json"


def _load_cached_lang(workspace_dir: Path) -> Lang | None:
    """Reuse language preference from motion-generation (``.motion_last.json``)."""
    path = workspace_dir / ".motion_last.json"
    if not path.is_file():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    if not isinstance(raw, dict):
        return None
    lang = raw.get("lang")
    if not isinstance(lang, str):
        return None
    try:
        return normalize_lang(lang)
    except ValueError:
        return None


def _stack_last_path(workspace_dir: Path) -> Path:
    return workspace_dir / _STACK_LAST_NAME


def _load_stack_last(workspace_dir: Path) -> dict[str, Any] | None:
    path = _stack_last_path(workspace_dir)
    if not path.is_file():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    return raw if isinstance(raw, dict) else None


def _save_stack_last(workspace_dir: Path, data: dict[str, Any]) -> None:
    path = _stack_last_path(workspace_dir)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError:
        pass


def _stack_last_applies(last: dict[str, Any], *, registry: dict[str, dict[str, Any]]) -> bool:
    robot = last.get("robot_key")
    if not isinstance(robot, str) or robot not in registry:
        return False
    skip_group = bool(last.get("skip_group"))
    group = last.get("group_id")
    if skip_group or group in (None, ""):
        return True
    if not isinstance(group, str):
        return False
    task_groups: dict[str, list[str]] = registry[robot].get("task_groups") or {}
    return bool(task_groups.get(group))


def _format_stack_last_brief(
    last: dict[str, Any],
    *,
    registry: dict[str, dict[str, Any]],
) -> list[str]:
    robot = str(last.get("robot_key", "?"))
    skip_group = bool(last.get("skip_group"))
    group = "" if skip_group else str(last.get("group_id") or "")
    group_label = group or t("ros2_stack.group_none")
    motion = str(last.get("motion_preset") or "?")
    include_nav = bool(last.get("include_nav"))
    if include_nav:
        profile = str(last.get("nav_profile") or "?")
        nav_map = str(last.get("nav_map") or "").strip()
        detail = profile if not nav_map else f"{profile}, map={nav_map}"
        nav_s = t("ros2_stack.brief.nav_yes", detail=detail)
    else:
        nav_s = t("ros2_stack.brief.nav_no")
    _ = registry  # reserved for future label lookup
    return [
        t("ros2_stack.brief.robot", value=robot),
        t("ros2_stack.brief.group", value=group_label),
        t("ros2_stack.brief.motion", value=motion),
        t("ros2_stack.brief.nav", value=nav_s),
    ]


def _prompt_start_entry_kind(
    *,
    last: dict[str, Any] | None,
    registry: dict[str, dict[str, Any]],
) -> str:
    """Return ``last`` or ``new``."""
    if last is None or not _stack_last_applies(last, registry=registry):
        return "new"
    print(t("ros2_stack.how_to_start"))
    print(t("ros2_stack.last_selection_header"))
    for line in _format_stack_last_brief(last, registry=registry):
        print(line)
    print(t("ros2_stack.new_selection"))
    print()
    raw = input(t("ros2_stack.select_start_mode")).strip().lower()
    if raw in ("", "1"):
        return "last"
    return "new"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start/status/stop scenario ROS2 motion & navigation stacks",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        default=None,
        help="Workspace root containing robots/ (default: cwd)",
    )
    parser.add_argument(
        "--lang",
        type=str,
        default=None,
        choices=("zh", "en"),
        help="UI language: zh or en (same as motion-generation)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    preset_choices = tuple(list_motion_preset_keys())
    profile_choices = tuple(sorted(NAV_PROFILES))

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--robot", type=str, default=None, help="Robot key")
        p.add_argument(
            "--group",
            type=str,
            default=None,
            help='Task folder group id (e.g. "projets/siemens/Wind turbo blade")',
        )
        p.add_argument(
            "--set",
            dest="sets",
            action="append",
            default=[],
            help="Override path=value (repeatable), e.g. motion.preset=ocs2-demo",
        )
        p.add_argument(
            "--motion-preset",
            type=str,
            default=None,
            choices=preset_choices,
            help="Motion launch preset",
        )
        p.add_argument(
            "--nav-profile",
            type=str,
            default=None,
            choices=profile_choices,
            help="Navigation profile",
        )
        p.add_argument(
            "--skip-group",
            action="store_true",
            help="Do not load leaf .meta (robot.yaml ros2_stack only)",
        )

    p_status = sub.add_parser("status", help="Show readiness of configured components")
    add_common(p_status)

    p_start = sub.add_parser("start", help="Start missing motion/navigation launches")
    add_common(p_start)
    p_start.add_argument("--new-terminal", action="store_true", help="Try GUI terminal for launches")
    p_start.add_argument("--timeout", type=float, default=60.0, help="Ready wait timeout (seconds)")
    p_start.add_argument(
        "--no-interactive-override",
        action="store_true",
        help="Do not prompt to change preset/profile",
    )
    p_start.add_argument(
        "--force-nav",
        action="store_true",
        help="Start navigation even if required=auto and no task selected",
    )

    p_stop = sub.add_parser("stop", help="Stop launches previously started by ros2-stack")
    p_stop.add_argument("--robot", type=str, default=None, help="Robot key")
    p_stop.add_argument(
        "--component",
        choices=("motion", "navigation", "all"),
        default="all",
        help="Which managed component to stop",
    )

    p_logs = sub.add_parser("logs", help="Show or follow ros2-stack log files")
    p_logs.add_argument("--robot", type=str, default=None, help="Robot key")
    p_logs.add_argument(
        "--component",
        choices=("motion", "navigation", "all"),
        default="all",
        help="Which log file(s) to show",
    )
    p_logs.add_argument(
        "-f",
        "--follow",
        action="store_true",
        help="Follow log output (like tail -f); Ctrl+C stops following only",
    )
    p_logs.add_argument(
        "-n",
        "--lines",
        type=int,
        default=80,
        help="Lines to show when not following (default: 80)",
    )

    p_comp = sub.add_parser("completion", help="Print shell tab-completion script")
    p_comp.add_argument(
        "shell",
        nargs="?",
        default="bash",
        choices=("bash",),
        help="Shell type (currently: bash)",
    )

    # Hidden helper for bash completion (robots / groups)
    p_hidden = sub.add_parser("_complete", help=argparse.SUPPRESS)
    p_hidden.add_argument("what", choices=("robots", "groups", "presets", "profiles"))
    p_hidden.add_argument("--workspace", type=str, default=None)
    p_hidden.add_argument("--robot", type=str, default=None)

    return parser


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def _pick_robot(
    registry: dict[str, dict[str, Any]],
    robot_key: str | None,
    *,
    default_robot: str | None = None,
) -> str:
    if robot_key is not None:
        if robot_key not in registry:
            raise SystemExit(t("ros2_stack.unknown_robot", robot=robot_key))
        return robot_key
    keys = list(registry.keys())
    if default_robot and default_robot in registry:
        default = default_robot
    else:
        default = "fiveages_w2" if "fiveages_w2" in registry else keys[0]
    return select_robot_from_registry(
        title=t("ros2_stack.select_robot"),
        registry=registry,
        default_key=default,
    )


def _pick_group(
    robot_entry: dict[str, Any],
    *,
    group: str | None,
    skip_group: bool,
    default_group: str = "",
) -> str:
    if skip_group:
        return ""
    if group is not None:
        return group
    task_groups: dict[str, list[str]] = robot_entry.get("task_groups") or {}
    nonempty = {g: ks for g, ks in task_groups.items() if ks}
    if not nonempty:
        return ""
    use_meta = select_option(
        title=t("ros2_stack.load_meta"),
        options={
            "yes": {"label": t("ros2_stack.load_meta_yes")},
            "no": {"label": t("ros2_stack.load_meta_no")},
        },
        default_key="yes",
    )
    if use_meta == "no":
        return ""
    preferred = default_group if default_group in nonempty else next(iter(nonempty))
    return select_task_group_drill_down(
        task_groups=nonempty,
        title=t("ros2_stack.select_task_folder"),
        default_group=preferred,
    )


def _apply_cli_overrides(
    resolved: ResolvedRos2Stack,
    *,
    sets: list[str],
    motion_preset: str | None,
    nav_profile: str | None,
    robot_key: str,
    merged: dict[str, Any] | None = None,
) -> ResolvedRos2Stack:
    out = resolved
    cur_merged = merged
    if sets:
        if cur_merged is None:
            raise SystemExit("internal: sets require merged config")
        cur_merged = apply_set_overrides(cur_merged, sets)
        out = resolve_ros2_stack(cur_merged, robot_key=robot_key)
    if motion_preset and out.motion is not None:
        out = with_motion_preset(out, motion_preset)
    if nav_profile and out.navigation is not None:
        out = with_nav_profile(out, nav_profile)
    return out


def _with_nav_map(resolved: ResolvedRos2Stack, nav_map: str | None) -> ResolvedRos2Stack:
    if not nav_map or resolved.navigation is None:
        return resolved
    return replace(
        resolved,
        navigation=replace(
            resolved.navigation,
            args={**resolved.navigation.args, "map": nav_map},
        ),
    )


def _resolve_stack(
    *,
    workspace_dir: Path,
    registry: dict[str, dict[str, Any]],
    robot_key: str,
    group_id: str,
    sets: list[str] | None = None,
    motion_preset: str | None = None,
    nav_profile: str | None = None,
    nav_map: str | None = None,
) -> tuple[str, dict[str, Any], Path, str, ResolvedRos2Stack]:
    robot_entry = registry[robot_key]
    robot_dir = resolve_robot_profile_dir(workspace_dir, robot_entry)
    merged, resolved, warnings = load_merged_ros2_stack(
        robot_dir=robot_dir,
        group_id=group_id,
        robot_key=robot_key,
    )
    for w in warnings:
        print(t("ros2_stack.warning", msg=w))
    if resolved is None:
        raise SystemExit(
            t("ros2_stack.no_config", robot=robot_key, group=group_id)
        )
    resolved = _apply_cli_overrides(
        resolved,
        sets=list(sets or []),
        motion_preset=motion_preset,
        nav_profile=nav_profile,
        robot_key=robot_key,
        merged=merged,
    )
    resolved = _with_nav_map(resolved, nav_map)
    return robot_key, robot_entry, robot_dir, group_id, resolved


def _last_defaults(
    last: dict[str, Any] | None,
    *,
    registry: dict[str, dict[str, Any]],
) -> tuple[str | None, str]:
    if last is None or not _stack_last_applies(last, registry=registry):
        return None, ""
    robot = str(last.get("robot_key") or "") or None
    if bool(last.get("skip_group")):
        return robot, ""
    return robot, str(last.get("group_id") or "")


def _resolve_for_cli(
    *,
    workspace_dir: Path,
    registry: dict[str, dict[str, Any]],
    args: argparse.Namespace,
    default_group: str = "",
    default_robot: str | None = None,
) -> tuple[str, dict[str, Any], Path, str, ResolvedRos2Stack]:
    robot_key = _pick_robot(registry, args.robot, default_robot=default_robot)
    robot_entry = registry[robot_key]
    group_id = _pick_group(
        robot_entry,
        group=getattr(args, "group", None),
        skip_group=bool(getattr(args, "skip_group", False)),
        default_group=default_group,
    )
    return _resolve_stack(
        workspace_dir=workspace_dir,
        registry=registry,
        robot_key=robot_key,
        group_id=group_id,
        sets=list(getattr(args, "sets", None) or []),
        motion_preset=getattr(args, "motion_preset", None),
        nav_profile=getattr(args, "nav_profile", None),
    )


def _decide_include_nav(
    resolved: ResolvedRos2Stack,
    *,
    force_nav: bool,
    interactive: bool,
    no_interactive_override: bool,
    default_include: bool | None,
) -> bool:
    if resolved.navigation is None:
        return False
    req = resolved.navigation.required
    if req is True or req == "true" or force_nav:
        return True
    if req == "auto":
        if interactive and not no_interactive_override:
            default_yes = True if default_include is None else bool(default_include)
            return select_yes_no(title=t("ros2_stack.start_nav_also"), default_yes=default_yes)
        return True if default_include is None else bool(default_include)
    return False


def _build_start_components(
    resolved: ResolvedRos2Stack,
    *,
    include_nav: bool,
) -> list:
    comps = []
    if resolved.motion is not None and component_is_needed(
        resolved.motion, task_needs_nav=include_nav
    ):
        comps.append(resolved.motion)
    if include_nav and resolved.navigation is not None:
        comps.append(resolved.navigation)
    return comps


def _snapshot_stack_last(
    *,
    robot_key: str,
    group_id: str,
    resolved: ResolvedRos2Stack,
    include_nav: bool,
) -> dict[str, Any]:
    nav_map = None
    if resolved.navigation is not None:
        raw_map = (resolved.navigation.args or {}).get("map")
        if raw_map is not None and str(raw_map).strip():
            nav_map = str(raw_map)
    return {
        "robot_key": robot_key,
        "group_id": group_id,
        "skip_group": not bool(group_id),
        "include_nav": bool(include_nav),
        "motion_preset": resolved.motion.preset if resolved.motion else None,
        "nav_profile": resolved.navigation.profile if resolved.navigation else None,
        "nav_map": nav_map,
    }


def cmd_status(workspace_dir: Path, args: argparse.Namespace) -> int:
    registry = load_motion_entries(workspace_dir)
    if not registry:
        raise SystemExit(t("ros2_stack.no_robots", path=workspace_dir / "robots"))
    last = _load_stack_last(workspace_dir)
    default_robot, default_group = _last_defaults(last, registry=registry)
    robot_key, _entry, _dir, group_id, resolved = _resolve_for_cli(
        workspace_dir=workspace_dir,
        registry=registry,
        args=args,
        default_group=default_group,
        default_robot=default_robot,
    )
    group_label = group_id or t("ros2_stack.group_none")
    print(t("ros2_stack.robot_group", robot=robot_key, group=group_label))
    try:
        names = list_ros_node_names()
    except Exception as exc:
        print(t("ros2_stack.list_nodes_failed", exc=exc))
        names = []
    for comp in (resolved.motion, resolved.navigation):
        if comp is None:
            continue
        ready = component_is_ready(comp, names)
        status = t("ros2_stack.ready") if ready else t("ros2_stack.not_ready")
        print(t("ros2_stack.component_line", name=comp.name, status=status))
        print(f"    preset/profile: {comp.preset or comp.profile}")
        print(t("ros2_stack.command", value=comp.launch_command_str()))
        print(f"    ready_when: {comp.ready_when.node_name_contains} (match={comp.ready_when.match})")
    pids = read_managed_pids(workspace_dir, robot_key)
    if pids:
        print(f"  managed pids: {pids}")
    return 0


def cmd_start(workspace_dir: Path, args: argparse.Namespace) -> int:
    registry = load_motion_entries(workspace_dir)
    if not registry:
        raise SystemExit(t("ros2_stack.no_robots", path=workspace_dir / "robots"))

    last = _load_stack_last(workspace_dir)
    interactive = sys.stdin.isatty()
    use_last = False
    # Only offer last-selection when robot/group not given on CLI.
    if (
        interactive
        and args.robot is None
        and args.group is None
        and not args.skip_group
    ):
        use_last = _prompt_start_entry_kind(last=last, registry=registry) == "last"

    if use_last:
        assert last is not None
        print(t("ros2_stack.using_last"))
        for line in _format_stack_last_brief(last, registry=registry):
            print(line)
        robot_key = str(last["robot_key"])
        group_id = "" if bool(last.get("skip_group")) else str(last.get("group_id") or "")
        motion_preset = args.motion_preset or (
            str(last["motion_preset"]) if last.get("motion_preset") else None
        )
        nav_profile = args.nav_profile or (
            str(last["nav_profile"]) if last.get("nav_profile") else None
        )
        nav_map = str(last["nav_map"]) if last.get("nav_map") else None
        robot_key, _entry, _dir, group_id, resolved = _resolve_stack(
            workspace_dir=workspace_dir,
            registry=registry,
            robot_key=robot_key,
            group_id=group_id,
            sets=list(args.sets or []),
            motion_preset=motion_preset,
            nav_profile=nav_profile,
            nav_map=nav_map,
        )
        include_nav = _decide_include_nav(
            resolved,
            force_nav=bool(args.force_nav),
            interactive=False,
            no_interactive_override=True,
            default_include=bool(last.get("include_nav")),
        )
        # Already chose "last selection" in the menu — start immediately.
    else:
        default_robot, default_group = _last_defaults(last, registry=registry)
        robot_key, _entry, _dir, group_id, resolved = _resolve_for_cli(
            workspace_dir=workspace_dir,
            registry=registry,
            args=args,
            default_group=default_group,
            default_robot=default_robot,
        )
        default_include = None
        if last and _stack_last_applies(last, registry=registry):
            default_include = bool(last.get("include_nav"))
        include_nav = _decide_include_nav(
            resolved,
            force_nav=bool(args.force_nav),
            interactive=interactive,
            no_interactive_override=bool(args.no_interactive_override),
            default_include=default_include,
        )
        if interactive and not args.no_interactive_override:
            resolved = prompt_stack_overrides_interactive(
                resolved,
                need_nav=include_nav,
                lock_motion_preset=args.motion_preset is not None,
                lock_nav_profile=args.nav_profile is not None,
                confirm_first=True,
            )

    group_label = group_id or t("ros2_stack.group_none")
    print(t("ros2_stack.robot_group", robot=robot_key, group=group_label))

    comps = _build_start_components(resolved, include_nav=include_nav)
    if not comps:
        print(t("ros2_stack.nothing_to_start"))
        return 0

    print(t("ros2_stack.will_ensure"))
    for c in comps:
        print(f"  - {c.name}: {c.launch_command_str()}")

    ensure_components(
        comps,
        workspace_dir=workspace_dir,
        robot_key=robot_key,
        new_terminal=bool(args.new_terminal),
        ready_timeout_sec=float(args.timeout),
    )
    _save_stack_last(
        workspace_dir,
        _snapshot_stack_last(
            robot_key=robot_key,
            group_id=group_id,
            resolved=resolved,
            include_nav=include_nav,
        ),
    )
    return 0


def _resolve_current_robot(
    workspace_dir: Path,
    *,
    registry: dict[str, dict[str, Any]],
    robot_arg: str | None,
) -> str | None:
    """Pick the current stack robot: CLI > live managed PID > last start. No menu."""
    if robot_arg is not None:
        return _pick_robot(registry, robot_arg)

    running = list_robots_with_live_managed_pids(workspace_dir)
    last = _load_stack_last(workspace_dir)
    last_robot, _ = _last_defaults(last, registry=registry)

    if len(running) == 1:
        return running[0]
    if len(running) > 1:
        if last_robot and last_robot in running:
            return last_robot
        return running[0]
    if last_robot:
        return last_robot
    return None


def _robots_to_stop(
    workspace_dir: Path,
    *,
    registry: dict[str, dict[str, Any]],
    robot_arg: str | None,
) -> list[str]:
    """Resolve which robot stack(s) to stop without forcing a menu when possible."""
    if robot_arg is not None:
        return [_pick_robot(registry, robot_arg)]

    running = list_robots_with_live_managed_pids(workspace_dir)
    last = _load_stack_last(workspace_dir)
    last_robot, _ = _last_defaults(last, registry=registry)

    if len(running) == 1:
        return running
    if len(running) > 1:
        # Prefer last started among live stacks; otherwise stop all live ones.
        if last_robot and last_robot in running:
            return [last_robot]
        return running
    if last_robot:
        return [last_robot]
    return []


def _stop_robot_components(
    workspace_dir: Path,
    *,
    robot_key: str,
    components: list[str],
) -> bool:
    print(t("ros2_stack.stop_auto_robot", robot=robot_key))
    any_stopped = False
    for name in components:
        ok = stop_component(workspace_dir=workspace_dir, robot_key=robot_key, component=name)
        result = t("ros2_stack.stop_signalled") if ok else t("ros2_stack.stop_no_pid")
        print(t("ros2_stack.stop_result", name=name, result=result))
        any_stopped = any_stopped or ok
    return any_stopped


def cmd_stop(workspace_dir: Path, args: argparse.Namespace) -> int:
    registry = load_motion_entries(workspace_dir)
    if not registry:
        raise SystemExit(t("ros2_stack.no_robots", path=workspace_dir / "robots"))
    robots = _robots_to_stop(workspace_dir, registry=registry, robot_arg=args.robot)
    if not robots:
        print(t("ros2_stack.stop_nothing"))
        return 1
    components = (
        ["motion", "navigation"] if args.component == "all" else [args.component]
    )
    any_stopped = False
    for robot_key in robots:
        any_stopped = (
            _stop_robot_components(
                workspace_dir, robot_key=robot_key, components=components
            )
            or any_stopped
        )
    return 0 if any_stopped else 1


def cmd_logs(workspace_dir: Path, args: argparse.Namespace) -> int:
    registry = load_motion_entries(workspace_dir)
    if not registry:
        raise SystemExit(t("ros2_stack.no_robots", path=workspace_dir / "robots"))
    robot_key = _resolve_current_robot(
        workspace_dir, registry=registry, robot_arg=args.robot
    )
    if robot_key is None:
        print(t("ros2_stack.logs_nothing"))
        return 1
    if args.robot is None:
        print(t("ros2_stack.logs_auto_robot", robot=robot_key))
    logs = list_component_logs(workspace_dir, robot_key)
    if args.component != "all":
        logs = {k: v for k, v in logs.items() if k == args.component}
    if not logs:
        print(t("ros2_stack.logs_empty", robot=robot_key))
        return 1

    state = stack_state_dir(workspace_dir, robot_key)
    print(t("ros2_stack.logs_header", path=state))
    for name, path in logs.items():
        print(f"  - {name}: {path}")

    paths = [str(p) for p in logs.values()]
    if args.follow:
        print(t("ros2_stack.logs_follow_hint"))
        # Use system tail so Ctrl+C only stops following
        cmd = ["tail", "-n", str(max(1, args.lines)), "-f", *paths]
        try:
            return int(subprocess.call(cmd))
        except FileNotFoundError:
            for p in paths:
                print(f"===== {p} =====")
                print(Path(p).read_text(encoding="utf-8", errors="replace")[-8000:])
            print("(tail not found; showing end of files once)")
            return 0

    # One-shot: last N lines each
    for name, path in logs.items():
        print(f"\n===== {name} ({path}) =====")
        try:
            text = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError as exc:
            print(f"(unreadable: {exc})")
            continue
        for line in text[-max(1, args.lines) :]:
            print(line)
    print(t("ros2_stack.logs_tail_hint", robot=robot_key))
    print(t("ros2_stack.stop_hint", robot=robot_key))
    return 0


def cmd_completion(args: argparse.Namespace) -> int:
    from robot_action_composer.cli.ros2_stack_completion import bash_completion_script

    shell = getattr(args, "shell", "bash") or "bash"
    if shell != "bash":
        raise SystemExit(f"Unsupported shell for completion: {shell!r}")
    script = bash_completion_script()
    sys.stdout.write(script if script.endswith("\n") else script + "\n")
    return 0


def cmd_complete_helper(args: argparse.Namespace) -> int:
    """Emit newline-separated completion candidates for bash (no i18n banner)."""
    from robot_action_composer.cli.ros2_stack_completion import complete_groups, complete_robots

    what = args.what
    if what == "robots":
        items = complete_robots(workspace=args.workspace)
    elif what == "groups":
        items = complete_groups(workspace=args.workspace, robot=args.robot)
    elif what == "presets":
        items = list_motion_preset_keys()
    elif what == "profiles":
        items = sorted(NAV_PROFILES)
    else:
        items = []
    sys.stdout.write("\n".join(items))
    if items:
        sys.stdout.write("\n")
    return 0


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    # Completion helpers must stay quiet (no title / i18n side effects)
    if args.command == "_complete":
        raise SystemExit(cmd_complete_helper(args))
    if args.command == "completion":
        raise SystemExit(cmd_completion(args))

    workspace_dir = resolve_workspace_dir(Path(args.workspace) if args.workspace else None)
    set_language(
        resolve_language(cli_lang=args.lang, cached_lang=_load_cached_lang(workspace_dir))
    )
    print(t("ros2_stack.title"))
    if args.command == "status":
        raise SystemExit(cmd_status(workspace_dir, args))
    if args.command == "start":
        raise SystemExit(cmd_start(workspace_dir, args))
    if args.command == "stop":
        raise SystemExit(cmd_stop(workspace_dir, args))
    if args.command == "logs":
        raise SystemExit(cmd_logs(workspace_dir, args))
    raise SystemExit(f"Unknown command {args.command!r}")


if __name__ == "__main__":
    main()
