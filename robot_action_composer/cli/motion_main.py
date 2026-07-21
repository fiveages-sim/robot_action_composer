#!/usr/bin/env python3
"""Motion-generation CLI (robot_action_composer)."""

from __future__ import annotations

import argparse
from dataclasses import fields
from datetime import datetime
import json
import sys
from pathlib import Path
from typing import Any, Literal, Mapping, Sequence, TypedDict

from robot_action_composer.cli.i18n import Lang, get_language, normalize_lang, resolve_language, set_language, t
from robot_action_composer.cli.interactive import (
    select_from_list,
    select_robot_from_registry,
    select_yes_no,
)
from robot_action_composer.discovery.registry_loader import load_motion_entries, resolve_robot_profile_dir


def _section_header(title: str) -> str:
    return f"\n{'=' * 70}\n{title}\n{'=' * 70}"


def _select_yes_no(*, title: str, default_yes: bool = True) -> bool:
    """Compat wrapper — prefer :func:`select_yes_no` from interactive."""
    return select_yes_no(title=title, default_yes=default_yes)


def _select_option(
    *,
    title: str,
    options: list[str],
    default_value: str,
    allow_back: bool = False,
) -> str:
    """Compat wrapper — prefer :func:`select_from_list` from interactive."""
    return select_from_list(
        title=title,
        options=options,
        default_value=default_value,
        allow_back=allow_back,
    )

class _ChainSegmentDict(TypedDict):
    task_key: str
    scene: str


class _MotionLastDict(TypedDict, total=False):
    """One run-configuration snapshot (history entry or legacy flat file)."""

    robot_key: str
    """Single-task mode (default when ``mode`` is absent or ``single``)."""
    task_key: str
    scene: str
    """``chain``: multi-segment run; ``single`` or omitted: one task + scene."""
    mode: Literal["single", "chain"]
    chain_segments: list[_ChainSegmentDict]
    num_runs: int
    reset_env: bool


_MOTION_LAST_FILENAME = ".motion_last.json"
_MOTION_HISTORY_MAX = 3
_MOTION_PREF_KEYS = frozenset({"lang", "record_object_resolution_json", "ensure_ros2_stack"})


def _motion_last_file(workspace_dir: Path) -> Path:
    return workspace_dir / _MOTION_LAST_FILENAME


def _read_motion_last_raw(workspace_dir: Path) -> dict[str, Any]:
    path = _motion_last_file(workspace_dir)
    if not path.is_file():
        return {}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return {}
    if not isinstance(raw, dict):
        return {}
    return dict(raw)


def _write_motion_last_raw(workspace_dir: Path, data: Mapping[str, Any]) -> None:
    path = _motion_last_file(workspace_dir)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(dict(data), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError:
        pass


def _is_legacy_flat_motion_last(raw: Mapping[str, Any]) -> bool:
    if "history_by_robot" in raw:
        return False
    return isinstance(raw.get("robot_key"), str) and (
        "task_key" in raw or "chain_segments" in raw or raw.get("mode") == "chain"
    )


def _run_config_from_mapping(raw: Mapping[str, Any]) -> _MotionLastDict | None:
    rk = raw.get("robot_key")
    if not isinstance(rk, str) or not rk:
        return None
    entry: _MotionLastDict = {"robot_key": rk}
    mode = raw.get("mode", "single")
    if mode == "chain":
        entry["mode"] = "chain"
        segs = raw.get("chain_segments")
        if isinstance(segs, list):
            cleaned: list[_ChainSegmentDict] = []
            for item in segs:
                if not isinstance(item, dict):
                    continue
                tk = item.get("task_key")
                sc = item.get("scene")
                if isinstance(tk, str) and isinstance(sc, str):
                    cleaned.append({"task_key": tk, "scene": sc})
            entry["chain_segments"] = cleaned
    else:
        entry["mode"] = "single"
        tk = raw.get("task_key")
        sc = raw.get("scene")
        if isinstance(tk, str):
            entry["task_key"] = tk
        if isinstance(sc, str):
            entry["scene"] = sc
    n = raw.get("num_runs", 1)
    entry["num_runs"] = int(n) if isinstance(n, int) else 1
    entry["reset_env"] = bool(raw.get("reset_env", True))
    return entry


def _normalize_motion_last_file(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Return prefs + ``history_by_robot`` / ``last_robot_key`` (migrate legacy flat files)."""
    out: dict[str, Any] = {k: raw[k] for k in _MOTION_PREF_KEYS if k in raw}
    history: dict[str, list[_MotionLastDict]] = {}
    last_robot: str | None = None

    if _is_legacy_flat_motion_last(raw):
        entry = _run_config_from_mapping(raw)
        if entry is not None:
            rk = str(entry["robot_key"])
            history[rk] = [entry]
            last_robot = rk
    else:
        raw_hist = raw.get("history_by_robot")
        if isinstance(raw_hist, dict):
            for rk, items in raw_hist.items():
                if not isinstance(rk, str) or not isinstance(items, list):
                    continue
                cleaned: list[_MotionLastDict] = []
                for item in items:
                    if not isinstance(item, dict):
                        continue
                    entry = _run_config_from_mapping({**item, "robot_key": item.get("robot_key", rk)})
                    if entry is not None:
                        cleaned.append(entry)
                if cleaned:
                    history[rk] = cleaned[:_MOTION_HISTORY_MAX]
        lr = raw.get("last_robot_key")
        if isinstance(lr, str) and lr:
            last_robot = lr
        elif history:
            # Prefer the robot whose first entry is "newest" when last_robot missing:
            # keep insertion order of history_by_robot keys as fallback.
            last_robot = next(iter(history))

    out["history_by_robot"] = history
    if last_robot:
        out["last_robot_key"] = last_robot
    return out


def _load_motion_last(workspace_dir: Path) -> dict[str, Any]:
    """Load normalized ``.motion_last.json`` (prefs + history). Empty dict if missing."""
    return _normalize_motion_last_file(_read_motion_last_raw(workspace_dir))


def _merge_motion_prefs(workspace_dir: Path, **updates: Any) -> None:
    data = _load_motion_last(workspace_dir)
    for key, value in updates.items():
        if key in _MOTION_PREF_KEYS:
            data[key] = value
    _write_motion_last_raw(workspace_dir, data)


def _load_cached_lang(workspace_dir: Path) -> Lang | None:
    last = _load_motion_last(workspace_dir)
    raw = last.get("lang")
    if not isinstance(raw, str):
        return None
    try:
        return normalize_lang(raw)
    except ValueError:
        return None


def _load_record_object_resolution_pref(workspace_dir: Path) -> bool:
    last = _load_motion_last(workspace_dir)
    return last.get("record_object_resolution_json") is True


def _load_ensure_ros2_stack_pref(workspace_dir: Path) -> bool:
    last = _load_motion_last(workspace_dir)
    return last.get("ensure_ros2_stack") is True


def _save_motion_lang(workspace_dir: Path, lang: Lang) -> None:
    _merge_motion_prefs(workspace_dir, lang=lang)


def _save_record_object_resolution_pref(workspace_dir: Path, enabled: bool) -> None:
    _merge_motion_prefs(workspace_dir, record_object_resolution_json=bool(enabled))


def _save_ensure_ros2_stack_pref(workspace_dir: Path, enabled: bool) -> None:
    _merge_motion_prefs(workspace_dir, ensure_ros2_stack=bool(enabled))


def _run_config_identity(entry: Mapping[str, Any]) -> tuple[Any, ...]:
    """Equality key for deduping history entries (full run config)."""
    mode = entry.get("mode", "single")
    n = int(entry.get("num_runs", 1) or 1)
    reset = bool(entry.get("reset_env", True))
    if mode == "chain":
        segs = entry.get("chain_segments") or []
        norm = tuple(
            (str(s.get("task_key")), str(s.get("scene")))
            for s in segs
            if isinstance(s, dict)
        )
        return ("chain", norm, n, reset)
    return ("single", str(entry.get("task_key")), str(entry.get("scene")), n, reset)


def _list_valid_history_for_robot(
    *,
    workspace_dir: Path,
    robot_key: str,
    registry: dict[str, dict[str, Any]],
) -> list[_MotionLastDict]:
    data = _load_motion_last(workspace_dir)
    history = data.get("history_by_robot") or {}
    if not isinstance(history, dict):
        return []
    items = history.get(robot_key) or []
    if not isinstance(items, list):
        return []
    out: list[_MotionLastDict] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        entry = _run_config_from_mapping(item)
        if entry is None:
            continue
        if _motion_last_applies(entry, registry=registry):
            out.append(entry)
        if len(out) >= _MOTION_HISTORY_MAX:
            break
    return out


def _recent_robot_history(
    *,
    workspace_dir: Path,
    registry: dict[str, dict[str, Any]],
) -> list[_MotionLastDict]:
    """Valid history entries for ``last_robot_key`` (MRU, max 3)."""
    data = _load_motion_last(workspace_dir)
    rk = data.get("last_robot_key")
    if not isinstance(rk, str) or rk not in registry:
        return []
    return _list_valid_history_for_robot(
        workspace_dir=workspace_dir,
        robot_key=rk,
        registry=registry,
    )


def _language_display_name(lang: Lang) -> str:
    return t("motion.language_name_zh") if lang == "zh" else t("motion.language_name_en")


def _prompt_language(*, workspace_dir: Path) -> None:
    zh_label = t("motion.lang_option_zh")
    en_label = t("motion.lang_option_en")
    current = get_language()
    default = zh_label if current == "zh" else en_label
    picked = _select_option(
        title=t("motion.select_language"),
        options=[zh_label, en_label],
        default_value=default,
    )
    new_lang: Lang = "zh" if picked == zh_label else "en"
    set_language(new_lang)
    _save_motion_lang(workspace_dir, new_lang)
    print(t("motion.language_saved", lang=_language_display_name(new_lang)))


def _prompt_record_object_resolution_pref(*, workspace_dir: Path) -> None:
    current = _load_record_object_resolution_pref(workspace_dir)
    enabled = _select_yes_no(title=t("motion.record_object_resolution"), default_yes=current)
    _save_record_object_resolution_pref(workspace_dir, enabled)
    value = t("motion.reset.yes") if enabled else t("motion.reset.no")
    print(t("motion.record_object_resolution_saved", value=value))


def _prompt_ensure_ros2_stack_pref(*, workspace_dir: Path) -> None:
    current = _load_ensure_ros2_stack_pref(workspace_dir)
    enabled = _select_yes_no(title=t("motion.ensure_ros2_stack"), default_yes=current)
    _save_ensure_ros2_stack_pref(workspace_dir, enabled)
    value = t("motion.reset.yes") if enabled else t("motion.reset.no")
    print(t("motion.ensure_ros2_stack_saved", value=value))


def _print_menu_divider() -> None:
    print(t("motion.menu_divider"))


def _task_brief_value(*, robot_entry: dict[str, Any] | None, task_key: str) -> str:
    if robot_entry is None:
        return task_key
    task_entry = (robot_entry.get("tasks") or {}).get(task_key)
    if not isinstance(task_entry, dict):
        return task_key
    label = str(task_entry.get("label") or "").strip()
    if label and label != task_key:
        return t("motion.brief.task_with_label", task_key=task_key, label=label)
    return task_key


def _format_motion_last_brief(
    last: _MotionLastDict,
    *,
    registry: dict[str, dict[str, Any]],
) -> list[str]:
    lines: list[str] = []
    rk = last.get("robot_key", "?")
    robot_entry = registry.get(rk) if isinstance(rk, str) else None
    lines.append(t("motion.brief.robot", value=rk))
    n = last.get("num_runs", 1)
    re_ = last.get("reset_env", True)
    reset_s = t("motion.reset.yes") if re_ else t("motion.reset.no")
    mode = last.get("mode", "single")
    if mode == "chain":
        lines.append(t("motion.brief.mode", value=t("motion.brief.mode_chain")))
        segs = last.get("chain_segments") or []
        seg_idx = 0
        for s in segs:
            if not isinstance(s, dict):
                continue
            seg_idx += 1
            tk = str(s.get("task_key", "?"))
            sc = s.get("scene", "?")
            sc_display = t("motion.all_scenes") if sc == "__all__" else str(sc)
            task_display = _task_brief_value(robot_entry=robot_entry, task_key=tk)
            lines.append(
                t("motion.brief.segment_item", index=seg_idx, task=task_display, scene=sc_display)
            )
    else:
        tk = str(last.get("task_key", "?"))
        sc = last.get("scene", "?")
        scene_s = t("motion.all_scenes") if sc == "__all__" else str(sc)
        lines.append(t("motion.brief.task", value=_task_brief_value(robot_entry=robot_entry, task_key=tk)))
        lines.append(t("motion.brief.scene", value=scene_s))
    lines.append(t("motion.brief.runs", value=n))
    lines.append(t("motion.brief.reset_env", value=reset_s))
    return lines


def _print_recent_history_menu_options(
    history: Sequence[_MotionLastDict],
    *,
    registry: dict[str, dict[str, Any]],
) -> None:
    for idx, entry in enumerate(history, start=1):
        print(t("motion.recent_selection_header", index=idx))
        for line in _format_motion_last_brief(entry, registry=registry):
            print(line)


def _print_using_last_summary(
    last: _MotionLastDict,
    *,
    registry: dict[str, dict[str, Any]],
) -> None:
    print(t("motion.using_last"))
    for line in _format_motion_last_brief(last, registry=registry):
        print(line)


def _print_how_to_run_menu(
    *,
    history: Sequence[_MotionLastDict],
    registry: dict[str, dict[str, Any]],
) -> tuple[int, int, int]:
    """Print history + new/settings options. Returns (single_idx, chain_idx, settings_idx)."""
    k = len(history)
    print(t("motion.how_to_run"))
    _print_recent_history_menu_options(history, registry=registry)
    single_idx = k + 1
    chain_idx = k + 2
    settings_idx = k + 3
    print(t("motion.interactive_single_n", index=single_idx))
    print(t("motion.interactive_chain_n", index=chain_idx))
    print()
    _print_menu_divider()
    print(t("motion.preferences_n", index=settings_idx))
    return single_idx, chain_idx, settings_idx


def _print_configure_menu() -> None:
    print(t("motion.configure"))
    print(t("motion.single_task"))
    print(t("motion.multi_chain"))
    print()
    _print_menu_divider()
    print(t("motion.language_settings"))


def _prompt_settings(*, workspace_dir: Path) -> None:
    while True:
        record_current = t("motion.reset.yes") if _load_record_object_resolution_pref(workspace_dir) else t(
            "motion.reset.no"
        )
        ensure_current = t("motion.reset.yes") if _load_ensure_ros2_stack_pref(workspace_dir) else t(
            "motion.reset.no"
        )
        print(t("motion.settings_menu"))
        print(t("motion.settings_language"))
        print(t("motion.settings_object_resolution", current=record_current))
        print(t("motion.settings_ensure_ros2_stack", current=ensure_current))
        print(t("motion.settings_back"))
        raw = input(t("motion.settings_select")).strip().lower()
        if raw in ("", "0", "b", "back"):
            return
        if raw == "1":
            _prompt_language(workspace_dir=workspace_dir)
            continue
        if raw == "2":
            _prompt_record_object_resolution_pref(workspace_dir=workspace_dir)
            continue
        if raw == "3":
            _prompt_ensure_ros2_stack_pref(workspace_dir=workspace_dir)
            continue


def _prompt_interactive_entry_kind(
    *,
    workspace_dir: Path,
    registry: dict[str, dict[str, Any]],
) -> tuple[
    Literal["last_single", "last_chain", "interactive_single", "interactive_chain"],
    _MotionLastDict | None,
]:
    while True:
        history = _recent_robot_history(workspace_dir=workspace_dir, registry=registry)
        if history:
            single_idx, chain_idx, settings_idx = _print_how_to_run_menu(
                history=history,
                registry=registry,
            )
            max_n = settings_idx
            raw = input(t("motion.select_mode_n", max_n=max_n)).strip().lower()
            if raw == "":
                raw = "1"
            if raw.isdigit():
                choice = int(raw)
                if 1 <= choice <= len(history):
                    selected = history[choice - 1]
                    lm = selected.get("mode", "single")
                    kind: Literal["last_single", "last_chain"] = (
                        "last_chain" if lm == "chain" else "last_single"
                    )
                    return kind, selected
                if choice == single_idx:
                    return "interactive_single", None
                if choice == chain_idx:
                    return "interactive_chain", None
                if choice == settings_idx:
                    _prompt_settings(workspace_dir=workspace_dir)
                    continue
            return "interactive_single", None

        _print_configure_menu()
        raw = input(t("motion.select_config")).strip().lower()
        if raw in ("", "1"):
            return "interactive_single", None
        if raw == "2":
            return "interactive_chain", None
        if raw == "3":
            _prompt_settings(workspace_dir=workspace_dir)
            continue
        return "interactive_single", None


def _save_motion_last(
    *,
    workspace_dir: Path,
    robot_key: str,
    num_runs: int,
    reset_env: bool,
    task_key: str | None = None,
    scene: str | None = None,
    mode: Literal["single", "chain"] = "single",
    chain_segments: list[_ChainSegmentDict] | None = None,
) -> None:
    data = _load_motion_last(workspace_dir)
    # Refresh prefs from current session
    data["lang"] = get_language()
    data["record_object_resolution_json"] = bool(data.get("record_object_resolution_json", False))
    data["ensure_ros2_stack"] = bool(data.get("ensure_ros2_stack", False))

    entry: _MotionLastDict = {
        "robot_key": robot_key,
        "num_runs": int(num_runs),
        "reset_env": bool(reset_env),
        "mode": mode,
    }
    if mode == "chain" and chain_segments:
        entry["chain_segments"] = list(chain_segments)
    else:
        if task_key is not None:
            entry["task_key"] = task_key
        if scene is not None:
            entry["scene"] = scene

    history_raw = data.get("history_by_robot")
    history: dict[str, list[_MotionLastDict]] = {}
    if isinstance(history_raw, dict):
        for rk, items in history_raw.items():
            if isinstance(rk, str) and isinstance(items, list):
                history[rk] = [x for x in items if isinstance(x, dict)]  # type: ignore[misc]

    existing = list(history.get(robot_key) or [])
    new_id = _run_config_identity(entry)
    existing = [e for e in existing if _run_config_identity(e) != new_id]
    history[robot_key] = [entry, *existing][:_MOTION_HISTORY_MAX]
    data["history_by_robot"] = history
    data["last_robot_key"] = robot_key
    _write_motion_last_raw(workspace_dir, data)


def _segment_valid_for_robot(
    robot_entry: dict[str, Any],
    *,
    task_key: str,
    scene: str,
) -> bool:
    tasks_map = robot_entry.get("tasks") or {}
    if task_key not in tasks_map:
        return False
    scene_presets = tasks_map[task_key].get("scene_presets") or {}
    if scene == "__all__":
        return bool(scene_presets)
    return scene in scene_presets


def _motion_last_applies(
    last: _MotionLastDict,
    *,
    registry: dict[str, dict[str, Any]],
) -> bool:
    rk = last.get("robot_key")
    if not isinstance(rk, str):
        return False
    robot_entry = registry.get(rk)
    if not robot_entry:
        return False

    mode = last.get("mode", "single")
    if mode == "chain":
        segs = last.get("chain_segments")
        if not isinstance(segs, list) or len(segs) < 1:
            return False
        for item in segs:
            if not isinstance(item, dict):
                return False
            tk = item.get("task_key")
            sc = item.get("scene")
            if not isinstance(tk, str) or not isinstance(sc, str):
                return False
            if not _segment_valid_for_robot(robot_entry, task_key=tk, scene=sc):
                return False
    else:
        tk = last.get("task_key")
        sc = last.get("scene")
        if not isinstance(tk, str) or not isinstance(sc, str):
            return False
        if not _segment_valid_for_robot(robot_entry, task_key=tk, scene=sc):
            return False

    n = last.get("num_runs", 1)
    if not isinstance(n, int) or n < 1:
        return False
    return True


def _robot_records_dir(*, workspace_dir: Path, robot_entry: dict[str, Any]) -> Path:
    return resolve_robot_profile_dir(workspace_dir, robot_entry) / "records"


def _default_object_resolution_json_output(
    *,
    workspace_dir: Path,
    robot_entry: dict[str, Any],
    robot_key: str,
    task_key: str,
    scene: str,
    run_idx: int,
) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_scene = scene.replace("/", "_")
    filename = f"{robot_key}_{task_key}_{safe_scene}_run{run_idx + 1:02d}_{ts}.object_resolution.json"
    return _robot_records_dir(workspace_dir=workspace_dir, robot_entry=robot_entry) / filename


def _resolve_record_object_resolution_json(
    *,
    workspace_dir: Path,
    object_resolution_json: Path | None,
) -> bool:
    if object_resolution_json is not None:
        return True
    return _load_record_object_resolution_pref(workspace_dir)


def _discover_object_resolution_jsons(*, workspace_dir: Path, robot_entry: dict[str, Any]) -> list[Path]:
    records_dir = _robot_records_dir(workspace_dir=workspace_dir, robot_entry=robot_entry)
    if not records_dir.is_dir():
        return []
    files: list[Path] = []
    for pattern in ("*.object_resolution.json",):
        files.extend(records_dir.glob(pattern))
    return sorted({p.resolve() for p in files}, key=lambda p: p.stat().st_mtime, reverse=True)


def _filter_object_resolution_jsons(
    jsons: list[Path],
    *,
    robot_key: str,
    task_key: str,
    scene: str,
) -> list[Path]:
    robot_token = str(robot_key).strip()
    task_token = str(task_key).strip()
    scene_token = str(scene).strip()
    if not robot_token or not task_token:
        return jsons

    def _matches(path: Path) -> bool:
        stem = path.stem
        if robot_token not in stem or task_token not in stem:
            return False
        if scene_token == "__all__":
            return "__all__" in stem
        return scene_token in stem or "__all__" in stem

    filtered = [path for path in jsons if _matches(path)]
    return filtered if filtered else jsons


def _read_object_resolution_json_manual() -> Path:
    raw = input(t("motion.object_resolution_manual")).strip()
    if not raw:
        raise ValueError("object resolution JSON path is required for real robot replay")
    json_path = Path(raw).expanduser()
    if not json_path.is_file():
        raise FileNotFoundError(f"object resolution JSON not found: {json_path}")
    return json_path


def _prompt_object_resolution_json_path(
    *,
    workspace_dir: Path,
    robot_entry: dict[str, Any],
    robot_key: str,
    task_key: str,
    scene: str,
    override_path: Path | None = None,
) -> Path:
    if override_path is not None:
        if not override_path.is_file():
            raise FileNotFoundError(f"object resolution JSON not found: {override_path}")
        return override_path

    records_dir = _robot_records_dir(workspace_dir=workspace_dir, robot_entry=robot_entry)
    all_jsons = _discover_object_resolution_jsons(workspace_dir=workspace_dir, robot_entry=robot_entry)
    jsons = _filter_object_resolution_jsons(
        all_jsons,
        robot_key=robot_key,
        task_key=task_key,
        scene=scene,
    )
    if not jsons:
        if not records_dir.is_dir():
            print(t("motion.records_dir_missing", path=records_dir))
        else:
            print(t("motion.no_object_resolution_json", path=records_dir))
        return _read_object_resolution_json_manual()

    default_path = jsons[0]
    print(t("motion.available_object_resolution_json"))
    for idx, path in enumerate(jsons, start=1):
        suffix = t("motion.json_default_newest") if path == default_path else ""
        try:
            display = path.relative_to(workspace_dir.resolve())
        except ValueError:
            display = path
        print(f"  {idx}. {path.name}  [{display}]{suffix}")

    raw = input(t("motion.select_json")).strip()
    if raw == "":
        selected = default_path
    elif raw.isdigit():
        index = int(raw) - 1
        if 0 <= index < len(jsons):
            selected = jsons[index]
        else:
            selected = default_path
    else:
        selected = default_path

    if not selected.is_file():
        raise FileNotFoundError(f"object resolution JSON not found: {selected}")
    return selected


def _build_runtime_for_scene(
    *,
    task_entry: dict[str, Any],
    scene: str,
    allowed: frozenset[str],
) -> tuple[Any, list[Any]]:
    from robot_action_composer.task_config_io import queue_root_overrides, validate_runtime_defaults_keys
    from robot_action_composer.task_runtime.merge import (  # pyright: ignore[reportMissingImports]
        merge_task_queue_skill_params,
    )
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        build_merged_queue_config,
    )
    from robot_action_composer.task_runtime.object_binding import (  # pyright: ignore[reportMissingImports]
        build_objects_for_task_entry,
    )

    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    scene_sd = scene_presets.get(scene, {})

    base_root = validate_runtime_defaults_keys(
        task_entry["runtime_defaults"],
        context="task.runtime_defaults",
    )
    scene_root = validate_runtime_defaults_keys(
        scene_sd,
        context=f"scene_presets.{scene}",
    )
    unknown_scene = [k for k in scene_root if k not in allowed]
    if unknown_scene:
        raise ValueError(f"Unknown scene preset keys for queue task: {unknown_scene}")

    skill_defaults = dict(task_entry.get("skill_defaults") or {})
    scene_skill_params = dict(scene_sd.get("skill_params") or {})
    objects_reg, active_object = build_objects_for_task_entry(task_entry, scene=scene)
    runtime = build_merged_queue_config(
        runtime_defaults=base_root,
        skill_defaults=skill_defaults,
        scene_preset=scene_sd,
        objects=objects_reg or None,
        active_object=active_object,
    )

    task_queue = task_entry.get("task_queue")
    if not task_queue:
        raise ValueError("Motion generation: task queue is empty.")
    merged_queue = merge_task_queue_skill_params(list(task_queue), skill_defaults, scene_skill_params)
    return runtime, merged_queue


def _merged_queue_allowed_keys() -> frozenset[str]:
    from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.bimanual_place import BimanualPlaceTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.task_runtime.config import QUEUE_SINGLE_ARM_KEYS  # pyright: ignore[reportMissingImports]

    names: set[str] = set(QUEUE_SINGLE_ARM_KEYS)
    names.add("base_link_entity_path")
    names.add("place_offset")  # 简化 dual_arm.place（YAML-only，非 dataclass 字段）
    names.update(
        {
            "motion_frame_id",
            "relative_frame_id",
            "tf_lookup_timeout",
            "translation_xyz",
            "spread_half",
            "spread_y_half",
            "stage_prefix",
        }
    )
    for cls in (BimanualCarryTaskConfig, BimanualPlaceTaskConfig, HandoverSyncConfig, DrawerGeometryConfig):
        names |= {f.name for f in fields(cls)}
    return frozenset(names)


def _group_display_name(group_key: str) -> str:
    return t("group.top_level") if group_key == "" else group_key


def _group_task_keys(robot_entry: dict[str, Any], group_key: str) -> list[str]:
    task_groups: dict[str, list[str]] = robot_entry.get("task_groups") or {}
    return list(task_groups.get(group_key, []))


def _prompt_chain_group(*, robot_entry: dict[str, Any]) -> str:
    """Pick one leaf task folder (``task_groups`` key) for the entire chain."""
    from robot_action_composer.cli.interactive import (  # pyright: ignore[reportMissingImports]
        select_task_group_drill_down,
    )

    task_groups: dict[str, list[str]] = robot_entry.get("task_groups") or {}
    group_keys = sorted((gk for gk, keys in task_groups.items() if keys), key=lambda gk: (gk == "", gk))
    if not group_keys:
        raise ValueError("Robot has no task_groups with tasks")
    if len(group_keys) == 1:
        only = group_keys[0]
        print(t("motion.chain_locked_folder", folder=_group_display_name(only)))
        return only

    preferred = ""
    for gk in group_keys:
        if gk == "Factory Poc" or gk.startswith("Factory Poc/"):
            preferred = gk
            break
    if not preferred:
        preferred = group_keys[0]

    chosen = select_task_group_drill_down(
        task_groups=task_groups,
        title=t("motion.select_chain_folder"),
        default_group=preferred,
    )
    print(t("motion.chain_locked_folder", folder=_group_display_name(chosen)))
    return chosen


def _scenes_for_chain_segment(
    *,
    task_key: str,
    task_entry: dict[str, Any],
    used_pairs: set[tuple[str, str]],
) -> list[str]:
    """Pick one scene or ``__all__`` (expands to unused task+scene pairs)."""
    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    scene_names = list(scene_presets.keys())
    if not scene_names:
        raise ValueError(f"Task {task_key!r} has no scene_presets")

    preferred = str(task_entry.get("default_scene") or scene_names[0])
    default_scene = preferred if preferred in scene_names else scene_names[0]
    unused_for_all = [sn for sn in scene_names if (task_key, sn) not in used_pairs]

    scene_options: list[str] = list(scene_names)
    if len(unused_for_all) > 1:
        scene_options.append("__all__")

    scene = _select_option(
        title=t("motion.select_scene_for_task", task=task_key),
        options=scene_options,
        default_value=default_scene,
        allow_back=True,
    )
    if scene == "__back__":
        return []
    if scene == "__all__":
        if not unused_for_all:
            print(t("motion.all_scenes_in_chain", task=task_key))
            return []
        print(
            t(
                "motion.expand_all_scenes",
                count=len(unused_for_all),
                scenes=", ".join(unused_for_all),
            )
        )
        return unused_for_all
    return [scene]


def _prompt_chain_segments(*, robot_entry: dict[str, Any]) -> list[_ChainSegmentDict]:
    """Interactive: one folder for the chain; per segment pick task + scene (or ``__all__``)."""
    from robot_action_composer.cli.interactive import (  # pyright: ignore[reportMissingImports]
        select_task_with_optional_group,
    )

    chain_group = _prompt_chain_group(robot_entry=robot_entry)
    tasks_map: dict[str, Any] = robot_entry["tasks"]
    group_keys = _group_task_keys(robot_entry, chain_group)
    if not group_keys:
        raise ValueError(f"Folder {_group_display_name(chain_group)!r} has no tasks")

    folder = _group_display_name(chain_group)
    filtered_tasks = {tk: tasks_map[tk] for tk in group_keys if tk in tasks_map}
    filtered_groups = {chain_group: list(filtered_tasks.keys())}
    task_options = {key: {"label": str(meta.get("label", key))} for key, meta in filtered_tasks.items()}
    default_task_key = (
        "siemens_box_carry"
        if "siemens_box_carry" in task_options
        else ("black_box_pick" if "black_box_pick" in task_options else next(iter(task_options)))
    )

    segments: list[_ChainSegmentDict] = []
    used_pairs: set[tuple[str, str]] = set()
    while True:
        while True:
            task_key = select_task_with_optional_group(
                title_group=t("motion.select_task_locked_folder", folder=folder),
                title_task=t("motion.select_task_segment"),
                tasks=task_options,
                task_groups=filtered_groups,
                default_task_key=default_task_key,
            )
            if task_key not in filtered_tasks:
                print(t("motion.task_not_in_folder", task=task_key, folder=folder))
                continue
            scene_names = _scenes_for_chain_segment(
                task_key=task_key,
                task_entry=filtered_tasks[task_key],
                used_pairs=used_pairs,
            )
            if not scene_names:
                continue
            break
        for sn in scene_names:
            segments.append({"task_key": task_key, "scene": sn})
            used_pairs.add((task_key, sn))
        default_task_key = task_key
        if len(segments) >= 30:
            print(t("motion.chain_max_segments"))
            break
        more = input(t("motion.add_chain_segment")).strip().lower()
        if more not in {"y", "yes"}:
            break
    if not segments:
        raise ValueError("Chain has no segments")
    return segments


def run_motion_generation(
    *,
    workspace_dir: Path,
    robot_key: str | None = None,
    task_key: str | None = None,
    scene: str | None = None,
    object_resolution_json: Path | None = None,
    no_reset: bool = False,
    lang: str | None = None,
    ensure_ros2_stack: bool | None = None,
    motion_preset: str | None = None,
    nav_profile: str | None = None,
    new_terminal: bool = False,
) -> None:
    set_language(resolve_language(cli_lang=lang, cached_lang=_load_cached_lang(workspace_dir)))
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        format_merged_queue_summary,
    )
    from robot_action_composer.cli.interactive import (  # pyright: ignore[reportMissingImports]
        prompt_positive_int,
        select_task_with_optional_group,
    )
    from robot_action_composer.ros_interface_utils import build_ros2_interface_from_robot_cfg  # pyright: ignore[reportMissingImports]
    from robot_action_composer.isaac_sim import SimTimeHelper  # pyright: ignore[reportMissingImports]

    registry = load_motion_entries(workspace_dir)
    robots_root = workspace_dir / "robots"
    if not registry:
        raise RuntimeError(
            f"No motion-generation capable robot configs found under {robots_root}"
        )

    cli_robot_key = robot_key
    cli_task_key = task_key
    cli_scene = scene

    if cli_robot_key is not None and cli_task_key is not None:
        if cli_robot_key not in registry:
            raise ValueError(f"Unknown robot key: {cli_robot_key!r}")
        robot_entry = registry[cli_robot_key]
        if cli_task_key not in robot_entry["tasks"]:
            raise ValueError(f"Unknown task key {cli_task_key!r} for robot {cli_robot_key!r}")
        task_entry = robot_entry["tasks"][cli_task_key]
        scene_presets_cli = task_entry["scene_presets"]
        if cli_scene is None:
            scene = str(task_entry.get("default_scene") or next(iter(scene_presets_cli)))
        else:
            scene = cli_scene
        if scene != "__all__" and scene not in scene_presets_cli:
            raise ValueError(f"Unknown scene {scene!r} for task {cli_task_key!r}")
        robot_key = cli_robot_key
        task_key = cli_task_key
        num_runs = 1
        reset_env = not no_reset
        entry_kind: Literal["last_single", "last_chain", "interactive_single", "interactive_chain"] = (
            "interactive_single"
        )
        chain_segments: list[_ChainSegmentDict] = []
        print(t("motion.title"))
        print("=" * 70)
        print(
            t(
                "motion.cli_selection",
                robot=robot_key,
                task=task_key,
                scene=scene,
                reset=t("motion.reset.yes") if reset_env else t("motion.reset.no"),
            )
        )
    else:
        print(t("motion.title"))
        print(t("motion.current_language", lang=_language_display_name(get_language())))
        record_on = _load_record_object_resolution_pref(workspace_dir)
        print(
            t(
                "motion.current_record_object_resolution",
                value=t("motion.reset.yes") if record_on else t("motion.reset.no"),
            )
        )
        ensure_on = _load_ensure_ros2_stack_pref(workspace_dir)
        print(
            t(
                "motion.current_ensure_ros2_stack",
                value=t("motion.reset.yes") if ensure_on else t("motion.reset.no"),
            )
        )
        print("=" * 70)

        entry_kind, selected_last = _prompt_interactive_entry_kind(
            workspace_dir=workspace_dir,
            registry=registry,
        )

        robot_key_sel: str
        num_runs: int
        reset_env: bool
        robot_entry: dict[str, Any]
        task_key = ""
        scene = ""
        task_entry: dict[str, Any] | None = None
        chain_segments = []

        if entry_kind == "last_single" and selected_last:
            last = selected_last
            robot_key_sel = str(last["robot_key"])
            task_key = str(last["task_key"])
            scene = str(last["scene"])
            num_runs = int(last.get("num_runs", 1))
            reset_env = bool(last.get("reset_env", True))
            robot_entry = registry[robot_key_sel]
            task_entry = robot_entry["tasks"][task_key]
            robot_key = robot_key_sel
            _print_using_last_summary(last, registry=registry)
        elif entry_kind == "last_chain" and selected_last:
            last = selected_last
            robot_key_sel = str(last["robot_key"])
            num_runs = int(last.get("num_runs", 1))
            reset_env = bool(last.get("reset_env", True))
            robot_entry = registry[robot_key_sel]
            robot_key = robot_key_sel
            raw_segs = last.get("chain_segments")
            if not isinstance(raw_segs, list):
                raise RuntimeError("Last chain selection is invalid (missing chain_segments)")
            chain_segments = [
                {"task_key": str(s["task_key"]), "scene": str(s["scene"])}
                for s in raw_segs
                if isinstance(s, dict) and "task_key" in s and "scene" in s
            ]
            if not chain_segments:
                raise RuntimeError("Last chain selection is empty")
            _print_using_last_summary(last, registry=registry)
        else:
            robot_keys = list(registry.keys())
            default_robot = "dobot_cr5" if "dobot_cr5" in registry else robot_keys[0]
            robot_key_sel = select_robot_from_registry(
                title=t("motion.select_robot"),
                registry=registry,
                default_key=default_robot,
            )
            robot_entry = registry[robot_key_sel]
            robot_key = robot_key_sel

            if entry_kind == "interactive_chain":
                chain_segments = _prompt_chain_segments(robot_entry=robot_entry)
                num_runs = prompt_positive_int(
                    t("motion.chain_runs"),
                    default=1,
                    min_value=1,
                )
                if num_runs > 1:
                    reset_env = True
                    print(t("motion.chain_multi_reset"))
                else:
                    reset_env = _select_yes_no(
                        title=t("motion.reset_first_segment"),
                        default_yes=True,
                    )
            else:
                tasks_map = robot_entry["tasks"]
                task_options = {
                    key: {"label": str(meta.get("label", key))}
                    for key, meta in tasks_map.items()
                }
                default_task_key = "pick_place" if "pick_place" in task_options else next(iter(task_options))
                while True:
                    task_key = select_task_with_optional_group(
                        title_group=t("motion.select_task_folder"),
                        title_task=t("motion.select_task"),
                        tasks=task_options,
                        task_groups=robot_entry.get("task_groups", {}),
                        default_task_key=default_task_key,
                    )
                    task_entry = robot_entry["tasks"][task_key]

                    scene_presets = task_entry["scene_presets"]
                    scene_names = list(scene_presets.keys())
                    default_scene = task_entry["default_scene"]
                    if default_scene not in scene_names and scene_names:
                        default_scene = scene_names[0]
                    scene = _select_option(
                        title=t("motion.select_scene_config"),
                        options=(scene_names + ["__all__"]),
                        default_value=default_scene,
                        allow_back=True,
                    )
                    if scene == "__back__":
                        continue
                    break

                num_runs = prompt_positive_int(
                    t("motion.motion_runs"),
                    default=1,
                    min_value=1,
                )

                if num_runs > 1:
                    reset_env = True
                    print(t("motion.multi_run_reset"))
                else:
                    reset_env = _select_yes_no(
                        title=t("motion.reset_randomize"),
                        default_yes=True,
                    )

    if entry_kind in ("last_chain", "interactive_chain"):
        _save_motion_last(
            workspace_dir=workspace_dir,
            robot_key=robot_key,
            num_runs=num_runs,
            reset_env=reset_env,
            mode="chain",
            chain_segments=chain_segments,
        )
    else:
        assert task_entry is not None
        _save_motion_last(
            workspace_dir=workspace_dir,
            robot_key=robot_key,
            task_key=task_key,
            scene=scene,
            num_runs=num_runs,
            reset_env=reset_env,
            mode="single",
        )

    allowed = _merged_queue_allowed_keys()
    import robot_action_composer.task_runtime.skills  # noqa: F401 - register built-in skills

    from robot_action_composer.hardware_detection import detect_motion_environment_from_ros
    from robot_action_composer.task_runtime.object_resolution_replay import build_object_resolution_session
    from robot_action_composer.task_runtime.runner import (  # pyright: ignore[reportMissingImports]
        run_task_queue,
        run_task_queue_on_connected_interface,
        run_task_queue_interactive_on_connected_interface,
    )
    from robot_action_composer.record.time_helpers import WallTimeHelper
    from robot_action_composer.ros2_stack.ensure import ensure_ros2_stack_for_motion

    # Ensure ROS2 motion/nav stack (if configured) before connecting
    ensure_task_key = task_key
    ensure_task_cfg: dict[str, Any] | None = task_entry
    if entry_kind in ("last_chain", "interactive_chain") and chain_segments:
        ensure_task_key = chain_segments[0]["task_key"]
        ensure_task_cfg = robot_entry["tasks"][ensure_task_key]
    if ensure_task_key and ensure_task_cfg is not None:
        # CLI --ensure / --no-ensure wins; otherwise preference (default: off).
        ensure_flag = (
            ensure_ros2_stack
            if ensure_ros2_stack is not None
            else _load_ensure_ros2_stack_pref(workspace_dir)
        )
        if ensure_flag:
            interactive_ensure = cli_robot_key is None or cli_task_key is None
            try:
                ensure_ros2_stack_for_motion(
                    workspace_dir=workspace_dir,
                    robot_dir=resolve_robot_profile_dir(workspace_dir, robot_entry),
                    robot_key=robot_key,
                    task_groups=robot_entry.get("task_groups") or {},
                    task_key=ensure_task_key,
                    task_cfg=ensure_task_cfg,
                    interactive=interactive_ensure,
                    ensure=True,
                    motion_preset=motion_preset,
                    nav_profile=nav_profile,
                    new_terminal=new_terminal,
                )
            except Exception as exc:
                print(f"[ros2-stack] ensure failed: {exc}")
                raise

    motion_environment = detect_motion_environment_from_ros()
    if motion_environment == "real":
        if entry_kind in ("last_chain", "interactive_chain"):
            raise ValueError("real object-resolution replay supports single task mode only")
        assert task_entry is not None
        scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
        scenes_to_run = list(scene_presets.keys()) if scene == "__all__" else [scene]
        json_path = _prompt_object_resolution_json_path(
            workspace_dir=workspace_dir,
            robot_entry=robot_entry,
            robot_key=robot_key,
            task_key=task_key,
            scene=scene,
            override_path=object_resolution_json,
        )
        object_resolution = build_object_resolution_session(
            mode="replay",
            replay_json_path=str(json_path),
        )
        print(t("motion.real_replay", path=json_path))
        use_stamped = task_entry.get("use_stamped", True)
        interface = build_ros2_interface_from_robot_cfg(robot_entry["robot_cfg"])
        sim_time = WallTimeHelper()
        connected = False
        try:
            interface.connect()
            connected = True
            label = t("motion.real_replay_label")
            if scene == "__all__":
                label += " (__all__)"
            print(t("motion.robot_connected", label=label))
            for scene_idx, scene_name in enumerate(scenes_to_run):
                runtime, merged_queue = _build_runtime_for_scene(
                    task_entry=task_entry,
                    scene=scene_name,
                    allowed=allowed,
                )
                print(format_merged_queue_summary(scene_name, runtime))
                print(
                    _section_header(
                        t(
                            "motion.real_replay_header",
                            index=scene_idx + 1,
                            total=len(scenes_to_run),
                            scene=scene_name,
                        )
                    )
                )
                if scene_idx > 0:
                    print(t("motion.real_all_no_reset"))
                run_task_queue_interactive_on_connected_interface(
                    interface=interface,
                    sim_time=sim_time,
                    robot_cfg=robot_entry["robot_cfg"],
                    runtime=runtime,
                    blocks=merged_queue,
                    reset_env=False,
                    use_stamped=use_stamped,
                    use_isaac_base_pose=False,
                    not_first_scene_in_batch=(scene_idx > 0),
                    task_key=task_key,
                    scene=scene_name,
                    object_resolution=object_resolution,
                )
        finally:
            sim_time.shutdown()
            if connected:
                interface.disconnect()
                print(t("motion.robot_disconnected"))
        return

    record_object_resolution_json = _resolve_record_object_resolution_json(
        workspace_dir=workspace_dir,
        object_resolution_json=object_resolution_json,
    )

    if entry_kind in ("last_chain", "interactive_chain"):
        for run_idx in range(num_runs):
            interface = build_ros2_interface_from_robot_cfg(robot_entry["robot_cfg"])
            sim_time = SimTimeHelper()
            connected = False
            try:
                interface.connect()
                connected = True
                print(t("motion.chain_connected"))
                prev_chain_task_key: str | None = None
                grasp_offset_cache: dict[tuple[str, str], tuple[float, float, float]] = {}
                for seg_idx, seg in enumerate(chain_segments):
                    tk = seg["task_key"]
                    task_entry_seg = robot_entry["tasks"][tk]
                    use_stamped_seg = task_entry_seg.get("use_stamped", True)
                    scene_name = seg["scene"]
                    object_resolution = None
                    if record_object_resolution_json:
                        record_json_path = object_resolution_json or _default_object_resolution_json_output(
                            workspace_dir=workspace_dir,
                            robot_entry=robot_entry,
                            robot_key=robot_key,
                            task_key=tk,
                            scene=scene_name,
                            run_idx=run_idx,
                        )
                        object_resolution = build_object_resolution_session(
                            mode="live",
                            record_json_path=str(record_json_path),
                        )
                        print(t("motion.sim_recording", path=record_json_path))
                    consecutive_same_task_in_chain = (
                        prev_chain_task_key is not None and tk == prev_chain_task_key
                    )
                    runtime, merged_queue = _build_runtime_for_scene(
                        task_entry=task_entry_seg,
                        scene=scene_name,
                        allowed=allowed,
                    )
                    print(format_merged_queue_summary(scene_name, runtime))
                    should_reset_env = reset_env and (seg_idx == 0)
                    print(
                        _section_header(
                            t(
                                "motion.chain_run_title",
                                run=run_idx + 1,
                                total_runs=num_runs,
                                seg=seg_idx + 1,
                                total_seg=len(chain_segments),
                                task=tk,
                                scene=scene_name,
                            )
                        )
                    )
                    if seg_idx > 0 and reset_env:
                        print(t("motion.chain_skip_reset_detail"))
                    if consecutive_same_task_in_chain:
                        print(
                            t(
                                "motion.chain_skip_blocks_detail",
                                task=tk,
                                scene=scene_name,
                            )
                        )
                    run_task_queue_on_connected_interface(
                        interface=interface,
                        sim_time=sim_time,
                        robot_cfg=robot_entry["robot_cfg"],
                        runtime=runtime,
                        blocks=merged_queue,
                        reset_env=should_reset_env,
                        use_stamped=use_stamped_seg,
                        consecutive_same_task_in_chain=consecutive_same_task_in_chain,
                        task_key=tk,
                        scene=scene_name,
                        object_resolution=object_resolution,
                        grasp_offset_cache=grasp_offset_cache,
                    )
                    prev_chain_task_key = tk
            finally:
                sim_time.shutdown()
                if connected:
                    interface.disconnect()
                    print(t("motion.robot_disconnected"))
        return

    assert task_entry is not None
    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    use_stamped = task_entry.get("use_stamped", True)
    scenes_to_run = list(scene_presets.keys()) if scene == "__all__" else [scene]

    run_all_scenes = len(scenes_to_run) > 1
    for run_idx in range(num_runs):
        object_resolution = None
        if record_object_resolution_json:
            record_json_path = object_resolution_json or _default_object_resolution_json_output(
                workspace_dir=workspace_dir,
                robot_entry=robot_entry,
                robot_key=robot_key,
                task_key=task_key,
                scene=scenes_to_run[0] if not run_all_scenes else "__all__",
                run_idx=run_idx,
            )
            object_resolution = build_object_resolution_session(
                mode="live",
                record_json_path=str(record_json_path),
            )
            print(t("motion.sim_recording", path=record_json_path))

        if run_all_scenes:
            interface = build_ros2_interface_from_robot_cfg(robot_entry["robot_cfg"])
            sim_time = SimTimeHelper()
            connected = False
            try:
                interface.connect()
                connected = True
                print(t("motion.all_scenes_connected"))
                grasp_offset_cache: dict[tuple[str, str], tuple[float, float, float]] = {}
                for scene_idx, scene_name in enumerate(scenes_to_run):
                    runtime, merged_queue = _build_runtime_for_scene(task_entry=task_entry, scene=scene_name, allowed=allowed)
                    print(format_merged_queue_summary(scene_name, runtime))
                    should_reset_env = reset_env and (scene_idx == 0)
                    not_first_scene_in_batch = scene_idx > 0
                    print(
                        _section_header(
                            t(
                                "motion.all_scenes_header",
                                run=run_idx + 1,
                                total=num_runs,
                                index=scene_idx + 1,
                                total_scenes=len(scenes_to_run),
                                scene=scene_name,
                            )
                        )
                    )
                    if scene_idx > 0 and reset_env:
                        print(t("motion.all_scenes_skip_reset"))
                    if not_first_scene_in_batch:
                        print(t("motion.all_scenes_skip_blocks"))
                    run_task_queue_on_connected_interface(
                        interface=interface,
                        sim_time=sim_time,
                        robot_cfg=robot_entry["robot_cfg"],
                        runtime=runtime,
                        blocks=merged_queue,
                        reset_env=should_reset_env,
                        use_stamped=use_stamped,
                        not_first_scene_in_batch=not_first_scene_in_batch,
                        task_key=task_key,
                        scene=scene_name,
                        object_resolution=object_resolution,
                        grasp_offset_cache=grasp_offset_cache,
                    )
            finally:
                sim_time.shutdown()
                if connected:
                    interface.disconnect()
                    print(t("motion.robot_disconnected"))
        else:
            scene_name = scenes_to_run[0]
            runtime, merged_queue = _build_runtime_for_scene(task_entry=task_entry, scene=scene_name, allowed=allowed)
            print(format_merged_queue_summary(scene_name, runtime))
            print(
                _section_header(
                    t(
                        "motion.single_scene_header",
                        run=run_idx + 1,
                        total=num_runs,
                        scene=scene_name,
                    )
                )
            )
            run_task_queue(
                robot_cfg=robot_entry["robot_cfg"],
                runtime=runtime,
                blocks=merged_queue,
                reset_env=reset_env,
                use_stamped=use_stamped,
                task_key=task_key,
                scene=scene_name,
                object_resolution=object_resolution,
            )


def resolve_workspace_dir(path: Path | None) -> Path:
    if path is None:
        return Path.cwd()
    return path.expanduser().resolve()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Motion generation launcher")
    parser.add_argument(
        "--workspace",
        type=str,
        default=None,
        help="Workspace root containing robots/ (default: current working directory)",
    )
    parser.add_argument("--robot", type=str, default=None, help="Robot key (e.g. dobot_cr5)")
    parser.add_argument("--task-key", type=str, default=None, help="Task key (e.g. pick_place)")
    parser.add_argument(
        "--scene",
        type=str,
        default=None,
        help="Scene preset name (defaults to task default_scene)",
    )
    parser.add_argument(
        "--object-resolution-json",
        dest="object_resolution_json",
        type=str,
        default=None,
        help="Optional override path for object-resolution JSON (sim: write; real: read)",
    )
    parser.add_argument("--no-reset", action="store_true", help="Skip environment reset (CLI mode only)")
    parser.add_argument(
        "--lang",
        type=str,
        default=None,
        choices=("zh", "en"),
        help="UI language: zh (Chinese) or en (English); default from MOTION_GENERATION_LANG or LANG",
    )
    parser.set_defaults(ensure_ros2_stack=None)
    ensure_group = parser.add_mutually_exclusive_group()
    ensure_group.add_argument(
        "--ensure-ros2-stack",
        dest="ensure_ros2_stack",
        action="store_const",
        const=True,
        help="Ensure motion/navigation ROS2 launches before connecting",
    )
    ensure_group.add_argument(
        "--no-ensure-ros2-stack",
        dest="ensure_ros2_stack",
        action="store_const",
        const=False,
        help="Do not auto-start motion/navigation ROS2 launches",
    )
    parser.add_argument(
        "--motion-preset",
        type=str,
        default=None,
        choices=("ocs2-fullbody", "ocs2-split-body", "ocs2-demo"),
        help="Override motion.preset when ensuring ros2_stack",
    )
    parser.add_argument(
        "--nav-profile",
        type=str,
        default=None,
        choices=("default", "map_only"),
        help="Override navigation.profile when ensuring ros2_stack",
    )
    parser.add_argument(
        "--new-terminal",
        action="store_true",
        help="Try spawning ros2_stack launches in a GUI terminal",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    workspace_dir = resolve_workspace_dir(Path(args.workspace) if args.workspace else None)
    json_path = (
        Path(args.object_resolution_json).expanduser() if args.object_resolution_json else None
    )
    run_motion_generation(
        workspace_dir=workspace_dir,
        robot_key=args.robot,
        task_key=args.task_key,
        scene=args.scene,
        object_resolution_json=json_path,
        no_reset=args.no_reset,
        lang=args.lang,
        ensure_ros2_stack=args.ensure_ros2_stack,
        motion_preset=args.motion_preset,
        nav_profile=args.nav_profile,
        new_terminal=args.new_terminal,
    )


if __name__ == "__main__":
    main()
