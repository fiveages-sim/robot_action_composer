#!/usr/bin/env python3
"""Shared interactive CLI helpers for motion and recording entry points."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_action_composer.cli.i18n import t


def prompt_positive_int(
    message: str,
    *,
    default: int = 1,
    min_value: int = 1,
) -> int:
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
        print(t("prompt.invalid_int", default=default))
        return default


def select_option(
    *,
    title: str,
    options: dict[str, dict[str, Any]],
    default_key: str,
    allow_back: bool = False,
) -> str:
    keys = list(options.keys())
    print(f"\n{title}")
    if allow_back:
        print(t("option.back"))
    for idx, key in enumerate(keys, start=1):
        label = options[key].get("label", key)
        suffix = t("option.default_suffix") if key == default_key else ""
        bracket = key if key else "top-level"
        print(f"  {idx}. {label} [{bracket}]{suffix}")
    prompt = t("option.select_with_back") if allow_back else t("option.select")
    raw = input(prompt).strip()
    if raw == "":
        return default_key
    if allow_back:
        rl = raw.lower()
        if raw == "0" or rl in ("b", "back"):
            return "__back__"
    if raw.isdigit():
        n = int(raw)
        if allow_back and n == 0:
            return "__back__"
        idx = n - 1
        if 0 <= idx < len(keys):
            return keys[idx]
    if raw in options:
        return raw
    print(t("option.invalid", raw=raw, default=default_key))
    return default_key


def task_group_menu_label(group_id: str) -> str:
    return t("group.top_level") if group_id == "" else group_id


def robot_vendor_id(entry: dict[str, Any], robot_key: str) -> str:
    """Vendor folder under ``robots/``; empty string when the profile is directly under ``robots/``."""
    relpath = str(entry.get("robot_dir_relpath") or entry.get("robot_dir_name") or robot_key)
    parts = Path(relpath.replace("\\", "/")).parts
    return parts[0] if len(parts) > 1 else ""


def group_registry_robots_by_vendor(
    registry: dict[str, dict[str, Any]],
) -> list[tuple[str, list[str]]]:
    buckets: dict[str, list[str]] = {}
    for robot_key in registry:
        vendor = robot_vendor_id(registry[robot_key], robot_key)
        buckets.setdefault(vendor, []).append(robot_key)
    for keys in buckets.values():
        keys.sort()
    return sorted(buckets.items(), key=lambda item: (item[0] != "", item[0]))


def robot_vendor_menu_label(vendor_id: str) -> str:
    return t("group.robots_root") if vendor_id == "" else vendor_id


def select_robot_from_registry(
    *,
    title: str,
    registry: dict[str, dict[str, Any]],
    default_key: str,
    show_label: bool = False,
) -> str:
    """Interactive robot picker; groups by vendor folder when ``robot_dir_relpath`` is nested."""
    if not registry:
        raise ValueError("select_robot_from_registry: empty registry")

    groups = group_registry_robots_by_vendor(registry)
    flat_keys = [key for _, keys in groups for key in keys]
    if default_key not in flat_keys:
        default_key = flat_keys[0]

    show_groups = len(groups) > 1 or (len(groups) == 1 and groups[0][0] != "")
    numbered: list[str] = []

    print(f"\n{title}")
    for vendor_id, keys in groups:
        if show_groups:
            print(f"\n  [{robot_vendor_menu_label(vendor_id)}]")
        for key in keys:
            numbered.append(key)
            idx = len(numbered)
            suffix = t("option.default_suffix") if key == default_key else ""
            prefix = "    " if show_groups else "  "
            if show_label:
                label = str(registry[key].get("label", key))
                print(f"{prefix}{idx}. {label} [{key}]{suffix}")
            else:
                print(f"{prefix}{idx}. {key}{suffix}")

    prompt = t("option.select")
    raw = input(prompt).strip()
    if raw == "":
        return default_key
    if raw.isdigit():
        index = int(raw) - 1
        if 0 <= index < len(numbered):
            return numbered[index]
    if raw in registry:
        return raw
    print(t("option.invalid", raw=raw, default=default_key))
    return default_key


def select_task_with_optional_group(
    *,
    title_group: str,
    title_task: str,
    tasks: dict[str, dict[str, Any]],
    task_groups: dict[str, list[str]],
    default_task_key: str,
) -> str:
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
        return select_option(title=title_task, options=options, default_key=default, allow_back=False)

    default_group = next(iter(sorted(filtered.keys(), key=lambda g: (g != "", g))))
    for gid, keys in filtered.items():
        if default_task_key in keys:
            default_group = gid
            break

    while True:
        group_options = {
            gid: {"label": task_group_menu_label(gid)}
            for gid in sorted(filtered.keys(), key=lambda g: (g != "", g))
        }
        chosen_group = select_option(
            title=title_group,
            options=group_options,
            default_key=default_group,
            allow_back=False,
        )

        subset = filtered[chosen_group]
        sub = {k: tasks[k] for k in subset}
        options = {k: {"label": str(sub[k].get("label", k))} for k in subset}
        sub_default = default_task_key if default_task_key in subset else subset[0]
        while True:
            task_key = select_option(
                title=title_task,
                options=options,
                default_key=sub_default,
                allow_back=True,
            )
            if task_key == "__back__":
                break
            return task_key
