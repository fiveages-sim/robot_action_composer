#!/usr/bin/env python3
"""Shared interactive CLI helpers for motion, ros2-stack, and recording entry points."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

from robot_action_composer.cli.i18n import t


def select_yes_no(*, title: str, default_yes: bool = True) -> bool:
    """Yes/no menu using shared i18n labels (same UX as motion-generation)."""
    yes = t("option.yes")
    no = t("option.no")
    options = {yes: {"label": yes}, no: {"label": no}}
    default_key = yes if default_yes else no
    picked = select_option(title=title, options=options, default_key=default_key)
    return picked == yes


def select_from_list(
    *,
    title: str,
    options: Sequence[str],
    default_value: str,
    allow_back: bool = False,
) -> str:
    """List-based menu (values displayed as labels). ``allow_back`` → ``\"__back__\"``."""
    values = list(options)
    if not values:
        raise ValueError("select_from_list: empty options")
    if default_value not in values:
        default_value = values[0]
    print(f"\n{title}")
    if allow_back:
        print(t("option.back"))
    for idx, name in enumerate(values, start=1):
        suffix = t("option.default_suffix") if name == default_value else ""
        print(f"  {idx}. {name}{suffix}")
    prompt = t("option.select_with_back") if allow_back else t("option.select")
    raw = input(prompt).strip()
    if raw == "":
        return default_value
    if allow_back:
        rl = raw.lower()
        if raw == "0" or rl in ("b", "back"):
            return "__back__"
    if raw.isdigit():
        n = int(raw)
        if allow_back and n == 0:
            return "__back__"
        index = n - 1
        if 0 <= index < len(values):
            return values[index]
    if raw in values:
        return raw
    print(t("option.invalid", raw=raw, default=default_value))
    return default_value


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


def _group_path_parts(group_id: str) -> tuple[str, ...]:
    if not group_id:
        return ()
    return tuple(p for p in group_id.replace("\\", "/").split("/") if p)


def _join_group_path(*parts: str) -> str:
    return "/".join(p for p in parts if p)


def _child_segments_under_prefix(leaf_groups: list[str], prefix: str) -> list[str]:
    """Next path segments under ``prefix`` among leaf group ids."""
    pref = _group_path_parts(prefix)
    children: set[str] = set()
    for gid in leaf_groups:
        parts = _group_path_parts(gid)
        if len(parts) <= len(pref):
            continue
        if parts[: len(pref)] != pref:
            continue
        children.add(parts[len(pref)])
    return sorted(children)


def _default_child_segment(children: list[str], *, preferred_leaf: str, prefix: str) -> str:
    if not children:
        raise ValueError("_default_child_segment: empty children")
    pref_parts = _group_path_parts(preferred_leaf)
    cur = _group_path_parts(prefix)
    if len(pref_parts) > len(cur) and pref_parts[: len(cur)] == cur:
        nxt = pref_parts[len(cur)]
        if nxt in children:
            return nxt
    if prefix == "" and "Factory Poc" in children:
        return "Factory Poc"
    return children[0]


def select_task_group_drill_down(
    *,
    task_groups: dict[str, list[str]],
    title: str,
    default_group: str = "",
) -> str:
    """Drill down folder segments until a leaf ``task_groups`` key is chosen."""
    leaf_groups = sorted(
        (gid for gid, keys in task_groups.items() if keys),
        key=lambda g: (g != "", g),
    )
    if not leaf_groups:
        raise ValueError("select_task_group_drill_down: no non-empty task_groups")
    if len(leaf_groups) == 1:
        return leaf_groups[0]

    preferred = default_group if default_group in task_groups and task_groups[default_group] else ""
    if not preferred:
        for gid in leaf_groups:
            if gid == "Factory Poc" or gid.startswith("Factory Poc/"):
                preferred = gid
                break
        if not preferred:
            preferred = leaf_groups[0]

    # Root leaf "" coexists only when it is the sole leaf (leaf-only rule); handled above.
    prefix = ""
    allow_auto = True
    while True:
        if prefix in task_groups and task_groups[prefix]:
            return prefix

        children = _child_segments_under_prefix(leaf_groups, prefix)
        if not children:
            raise ValueError(
                f"select_task_group_drill_down: no children under {prefix!r} "
                f"and {prefix!r} is not a leaf group"
            )

        while allow_auto and len(children) == 1:
            prefix = _join_group_path(prefix, children[0]) if prefix else children[0]
            if prefix in task_groups and task_groups[prefix]:
                return prefix
            children = _child_segments_under_prefix(leaf_groups, prefix)
            if not children:
                raise ValueError(
                    f"select_task_group_drill_down: dead-end under {prefix!r}"
                )

        crumb = task_group_menu_label(prefix) if prefix else t("group.top_level")
        level_title = t("group.drill_title", title=title, path=crumb)
        default_seg = _default_child_segment(children, preferred_leaf=preferred, prefix=prefix)
        options = {seg: {"label": seg} for seg in children}
        allow_back = bool(prefix)
        chosen = select_option(
            title=level_title,
            options=options,
            default_key=default_seg,
            allow_back=allow_back,
        )
        if chosen == "__back__":
            parts = _group_path_parts(prefix)
            prefix = _join_group_path(*parts[:-1]) if len(parts) > 1 else ""
            allow_auto = False
            continue
        allow_auto = True
        prefix = _join_group_path(prefix, chosen) if prefix else chosen


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
        chosen_group = select_task_group_drill_down(
            task_groups=filtered,
            title=title_group,
            default_group=default_group,
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


def motion_preset_option_label(preset_key: str) -> str:
    i18n_key = f"ros2_stack.preset.{preset_key}"
    try:
        return t(i18n_key)
    except KeyError:
        return preset_key


def nav_profile_option_label(profile_key: str) -> str:
    i18n_key = f"ros2_stack.profile.{profile_key}"
    try:
        return t(i18n_key)
    except KeyError:
        return profile_key


def select_motion_preset(*, default_key: str, preset_keys: Sequence[str] | None = None) -> str:
    """Interactive motion preset picker (shared by motion-generation ensure and ros2-stack)."""
    from robot_action_composer.ros2_stack.presets import list_motion_preset_keys

    keys = list(preset_keys) if preset_keys is not None else list_motion_preset_keys()
    options = {k: {"label": motion_preset_option_label(k)} for k in keys}
    default = default_key if default_key in options else keys[0]
    return select_option(
        title=t("ros2_stack.select_motion_preset"),
        options=options,
        default_key=default,
    )


def select_nav_profile(*, default_key: str = "default") -> str:
    """Interactive navigation profile picker."""
    keys = ["default", "map_only"]
    options = {k: {"label": nav_profile_option_label(k)} for k in keys}
    default = default_key if default_key in options else "default"
    return select_option(
        title=t("ros2_stack.select_nav_profile"),
        options=options,
        default_key=default,
    )
