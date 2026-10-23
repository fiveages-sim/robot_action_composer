"""High-level ensure helpers for motion-generation and ros2-stack CLI."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence

from robot_action_composer.cli.i18n import t
from robot_action_composer.cli.interactive import (
    select_motion_preset,
    select_nav_profile,
    select_yes_no,
)
from robot_action_composer.ros2_stack.config import (
    ResolvedRos2Stack,
    apply_set_overrides,
    component_is_needed,
    find_task_group_id,
    load_merged_ros2_stack,
    resolve_ros2_stack,
    task_queue_needs_navigation,
    with_motion_preset,
    with_nav_profile,
)
from robot_action_composer.ros2_stack.launcher import ensure_components
from robot_action_composer.ros2_stack.status import component_is_ready, list_ros_node_names


def resolve_stack_for_task(
    *,
    robot_dir: Path,
    robot_key: str,
    task_groups: Mapping[str, Sequence[str]],
    task_key: str | None,
    group_id: str | None = None,
    set_overrides: Sequence[str] = (),
    motion_preset: str | None = None,
    nav_profile: str | None = None,
) -> tuple[ResolvedRos2Stack | None, list[str]]:
    gid = group_id if group_id is not None else (
        find_task_group_id(task_groups, task_key) if task_key else ""
    )
    merged, resolved, warnings = load_merged_ros2_stack(
        robot_dir=robot_dir,
        group_id=gid,
        robot_key=robot_key,
    )
    if resolved is None:
        return None, warnings
    if set_overrides:
        merged = apply_set_overrides(merged, set_overrides)
        resolved = resolve_ros2_stack(merged, robot_key=robot_key)
    if motion_preset and resolved.motion is not None:
        resolved = with_motion_preset(resolved, motion_preset)
    if nav_profile and resolved.navigation is not None:
        resolved = with_nav_profile(resolved, nav_profile)
    return resolved, warnings


def needed_components(
    resolved: ResolvedRos2Stack,
    *,
    task_cfg: Mapping[str, Any] | None,
) -> list:
    task_needs_nav = task_queue_needs_navigation(
        (task_cfg or {}).get("task_queue") if task_cfg else None
    )
    out = []
    if resolved.motion is not None and component_is_needed(
        resolved.motion, task_needs_nav=task_needs_nav
    ):
        out.append(resolved.motion)
    if resolved.navigation is not None and component_is_needed(
        resolved.navigation, task_needs_nav=task_needs_nav
    ):
        out.append(resolved.navigation)
    return out


def prompt_stack_overrides_interactive(
    resolved: ResolvedRos2Stack,
    *,
    need_nav: bool,
    lock_motion_preset: bool = False,
    lock_nav_profile: bool = False,
    confirm_first: bool = True,
) -> ResolvedRos2Stack:
    """Optionally change motion preset / nav profile (shared menus + i18n).

    When ``confirm_first`` is True (default), ask once whether to change options;
    Enter / No keeps the merged config defaults without re-selecting presets.
    """
    out = resolved
    if confirm_first:
        print(t("ros2_stack.resolved_header"))
        if out.motion is not None:
            preset = out.motion.preset or "?"
            print(t("ros2_stack.configured_default", value=preset))
            print(t("ros2_stack.command", value=out.motion.launch_command_str()))
        if need_nav and out.navigation is not None:
            profile = out.navigation.profile or "?"
            print(t("ros2_stack.configured_default", value=profile))
            print(t("ros2_stack.command", value=out.navigation.launch_command_str()))
        if not select_yes_no(title=t("ros2_stack.change_options"), default_yes=False):
            return out

    if out.motion is not None and not lock_motion_preset:
        default = out.motion.preset or "ocs2-fullbody"
        print(t("ros2_stack.motion_header"))
        print(t("ros2_stack.configured_default", value=default))
        print(t("ros2_stack.command", value=out.motion.launch_command_str()))
        chosen = select_motion_preset(default_key=default)
        if chosen != out.motion.preset:
            out = with_motion_preset(out, chosen)

    if need_nav and out.navigation is not None and not lock_nav_profile:
        default = out.navigation.profile or "default"
        print(t("ros2_stack.nav_header"))
        print(t("ros2_stack.configured_default", value=default))
        print(t("ros2_stack.command", value=out.navigation.launch_command_str()))
        chosen = select_nav_profile(default_key=default)
        if chosen != out.navigation.profile:
            out = with_nav_profile(out, chosen)
        map_default = str((out.navigation.args or {}).get("map") or "")
        raw = input(t("ros2_stack.nav_map_prompt", default=map_default)).strip()
        if raw and out.navigation is not None:
            out = replace(
                out,
                navigation=replace(
                    out.navigation,
                    args={**out.navigation.args, "map": raw},
                ),
            )
    return out


def ensure_ros2_stack_for_motion(
    *,
    workspace_dir: Path,
    robot_dir: Path,
    robot_key: str,
    task_groups: Mapping[str, Sequence[str]],
    task_key: str,
    task_cfg: Mapping[str, Any],
    interactive: bool = True,
    ensure: bool | None = None,
    motion_preset: str | None = None,
    nav_profile: str | None = None,
    new_terminal: bool = False,
    ready_timeout_sec: float = 60.0,
) -> ResolvedRos2Stack | None:
    """Optionally ensure motion/nav launches before motion-generation connect.

    ``ensure``: None = ask in interactive mode (default yes if config exists); True/False force.
    """
    resolved, warnings = resolve_stack_for_task(
        robot_dir=robot_dir,
        robot_key=robot_key,
        task_groups=task_groups,
        task_key=task_key,
        motion_preset=motion_preset,
        nav_profile=nav_profile,
    )
    for w in warnings:
        print(t("ros2_stack.warning", msg=w))
    if resolved is None:
        return None

    comps = needed_components(resolved, task_cfg=task_cfg)
    if not comps:
        print(t("ros2_stack.no_components"))
        return resolved

    print(t("ros2_stack.resolved_header"))
    for c in comps:
        print(f"  - {c.name}: {c.launch_command_str()}")

    do_ensure = ensure
    if do_ensure is None:
        if interactive:
            do_ensure = select_yes_no(
                title=t("ros2_stack.ensure_prompt"),
                default_yes=True,
            )
        else:
            print(t("ros2_stack.ensure_hint_cli"))
            return resolved

    if not do_ensure:
        print(t("ros2_stack.ensure_skipped"))
        return resolved

    need_nav = any(c.name == "navigation" for c in comps)
    if interactive:
        resolved = prompt_stack_overrides_interactive(
            resolved,
            need_nav=need_nav,
            lock_motion_preset=motion_preset is not None,
            lock_nav_profile=nav_profile is not None,
        )
        comps = needed_components(resolved, task_cfg=task_cfg)

    try:
        names = list_ros_node_names()
        if all(component_is_ready(c, names) for c in comps):
            print(t("ros2_stack.all_ready"))
            return resolved
    except Exception:
        pass

    ensure_components(
        comps,
        workspace_dir=workspace_dir,
        robot_key=robot_key,
        new_terminal=new_terminal,
        ready_timeout_sec=ready_timeout_sec,
    )
    return resolved


__all__ = [
    "ensure_ros2_stack_for_motion",
    "needed_components",
    "prompt_stack_overrides_interactive",
    "resolve_stack_for_task",
]
