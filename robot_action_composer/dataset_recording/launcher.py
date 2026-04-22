#!/usr/bin/env python3
"""Common interactive helpers for IsaacSim dataset record launchers."""

from __future__ import annotations

from typing import Any


def prompt_positive_int(
    message: str,
    *,
    default: int = 1,
    min_value: int = 1,
) -> int:
    """Parse a positive integer from stdin; invalid input falls back to ``default``.

    Used for "how many episodes" / "how many demo runs" style prompts so record and
    motion-generation launchers share one implementation.
    """
    if min_value < 1:
        raise ValueError("min_value must be >= 1")
    default = max(min_value, int(default))
    try:
        raw = input(message).strip()
        if raw == "":
            return default
        value = int(raw)
        return max(min_value, value)
    except Exception:
        print(f"[info] invalid input, using default {default}")
        return default


def select_option(*, title: str, options: dict[str, dict[str, Any]], default_key: str) -> str:
    keys = list(options.keys())
    print(f"\n{title}")
    for idx, key in enumerate(keys, start=1):
        label = options[key].get("label", key)
        suffix = " (default)" if key == default_key else ""
        bracket = key if key else "top-level"
        print(f"  {idx}. {label} [{bracket}]{suffix}")
    raw = input("Select option (press Enter for default): ").strip()
    if raw == "":
        return default_key
    if raw.isdigit():
        idx = int(raw) - 1
        if 0 <= idx < len(keys):
            return keys[idx]
    if raw in options:
        return raw
    print(f"[info] Invalid option '{raw}', using default '{default_key}'.")
    return default_key


def task_group_menu_label(group_id: str) -> str:
    """Human label for a ``task_groups`` key (``\"\"`` = YAML directly under ``task_configs/``)."""
    return "Top level" if group_id == "" else group_id


def select_task_with_optional_group(
    *,
    title_group: str,
    title_task: str,
    tasks: dict[str, dict[str, Any]],
    task_groups: dict[str, list[str]],
    default_task_key: str,
) -> str:
    """If tasks span multiple first-level folders, prompt for folder then task; else one menu."""
    if not tasks:
        raise ValueError("select_task_with_optional_group: empty tasks")

    filtered: dict[str, list[str]] = {}
    for gid in sorted(task_groups.keys(), key=lambda g: (g != "", g)):
        present = [k for k in task_groups[gid] if k in tasks]
        if present:
            filtered[gid] = present

    assigned: set[str] = set()
    for keys in filtered.values():
        assigned.update(keys)
    missing = [k for k in tasks if k not in assigned]
    if missing:
        bucket = filtered.setdefault("", [])
        bucket.extend(missing)
        filtered[""] = sorted(set(bucket))

    if not filtered:
        raise ValueError("select_task_with_optional_group: no task keys overlap task_groups and tasks")

    if len(filtered) <= 1:
        options = {k: {"label": str(tasks[k].get("label", k))} for k in tasks}
        default = default_task_key if default_task_key in options else next(iter(options))
        return select_option(title=title_task, options=options, default_key=default)

    default_group = next(iter(sorted(filtered.keys(), key=lambda g: (g != "", g))))
    for gid, keys in filtered.items():
        if default_task_key in keys:
            default_group = gid
            break

    group_options = {gid: {"label": task_group_menu_label(gid)} for gid in sorted(filtered.keys(), key=lambda g: (g != "", g))}
    chosen_group = select_option(title=title_group, options=group_options, default_key=default_group)

    subset = filtered[chosen_group]
    sub = {k: tasks[k] for k in subset}
    options = {k: {"label": str(sub[k].get("label", k))} for k in subset}
    sub_default = default_task_key if default_task_key in subset else subset[0]
    return select_option(title=title_task, options=options, default_key=sub_default)


def collect_runtime_options(
    *,
    pointcloud_supported: bool,
    default_enable_keypoint_pcd: bool = False,
) -> tuple[int, bool, bool]:
    loops = prompt_positive_int("How many episodes to record? ", default=1, min_value=1)

    if pointcloud_supported:
        default_pcd = "Y" if default_enable_keypoint_pcd else "N"
        use_pcd_input = input(f"Capture depth+pointcloud at keypoints? [y/{default_pcd}]: ").strip().lower()
        if use_pcd_input == "":
            enable_keypoint_pcd = default_enable_keypoint_pcd
        else:
            enable_keypoint_pcd = use_pcd_input in {"y", "yes"}
    else:
        enable_keypoint_pcd = False
        print("[info] Pointcloud option hidden: no depth camera configured for this robot.")

    manual_check_input = input("Manually review each episode after return-to-home? [y/N]: ").strip().lower()
    enable_manual_episode_check = manual_check_input in {"y", "yes"}
    return loops, enable_keypoint_pcd, enable_manual_episode_check
