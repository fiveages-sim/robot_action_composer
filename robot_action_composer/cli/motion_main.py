#!/usr/bin/env python3
"""IsaacSim motion-generation launcher (robot_action_composer)."""

from __future__ import annotations

from dataclasses import fields
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Literal, TypedDict


def _load_module(module_name: str, file_path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec: {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _discover_task_registry(isaac_dir: Path) -> dict[str, dict[str, Any]]:
    from robot_action_composer.task_config_io import discover_task_configs

    robots_root = isaac_dir / "robots"
    if not robots_root.is_dir():
        return {}

    registry: dict[str, dict[str, Any]] = {}
    for robot_dir in sorted(p for p in robots_root.iterdir() if p.is_dir() and not p.name.startswith("__")):
        robot_cfg_file = robot_dir / "robot_config.py"
        task_cfg_dir = robot_dir / "task_configs"
        if not robot_cfg_file.is_file() or not task_cfg_dir.is_dir():
            continue

        robot_mod = _load_module(f"{robot_dir.name}_robot_cfg", robot_cfg_file)
        robot_key = getattr(robot_mod, "ROBOT_KEY", robot_dir.name.lower())
        robot_label = getattr(robot_mod, "ROBOT_LABEL", robot_dir.name)
        robot_cfg = getattr(robot_mod, "ROBOT_CFG")

        discovery = discover_task_configs(task_cfg_dir, robot_dir_name=robot_dir.name)

        if discovery.tasks:
            registry[robot_key] = {
                "label": robot_label,
                "robot_cfg": robot_cfg,
                "tasks": discovery.tasks,
                "task_groups": discovery.task_groups,
            }

    return registry


def _select_option(
    *,
    title: str,
    options: list[str],
    default_value: str,
    allow_back: bool = False,
) -> str:
    """List-based menu. With ``allow_back=True``, ``0`` / ``b`` / ``back`` returns ``\"__back__\"``."""
    print(f"\n{title}")
    if allow_back:
        print("  0. « Back (previous menu)")
    for idx, name in enumerate(options, start=1):
        suffix = " (default)" if name == default_value else ""
        print(f"  {idx}. {name}{suffix}")
    prompt = (
        "Select option (0/b/back = previous, Enter = default): "
        if allow_back
        else "Select option (press Enter for default): "
    )
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
        if 0 <= index < len(options):
            return options[index]
    if raw in options:
        return raw
    print(f"[info] Invalid option '{raw}', using default '{default_value}'.")
    return default_value


class _ChainSegmentDict(TypedDict):
    task_key: str
    scene: str


class _MotionLastDict(TypedDict, total=False):
    isaac_dir: str
    robot_key: str
    """Single-task mode (default when ``mode`` is absent or ``single``)."""
    task_key: str
    scene: str
    """``chain``: multi-segment run; ``single`` or omitted: one task + scene."""
    mode: Literal["single", "chain"]
    chain_segments: list[_ChainSegmentDict]
    num_runs: int
    reset_env: bool


def _motion_last_file() -> Path:
    return Path.home() / ".cache" / "robot_action_composer" / "motion_last.json"


def _load_motion_last() -> _MotionLastDict | None:
    path = _motion_last_file()
    if not path.is_file():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    if not isinstance(raw, dict):
        return None
    return raw  # type: ignore[return-value]


def _save_motion_last(
    *,
    isaac_dir: Path,
    robot_key: str,
    num_runs: int,
    reset_env: bool,
    task_key: str | None = None,
    scene: str | None = None,
    mode: Literal["single", "chain"] = "single",
    chain_segments: list[_ChainSegmentDict] | None = None,
) -> None:
    path = _motion_last_file()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        data: dict[str, Any] = {
            "isaac_dir": str(isaac_dir.resolve()),
            "robot_key": robot_key,
            "num_runs": int(num_runs),
            "reset_env": bool(reset_env),
            "mode": mode,
        }
        if mode == "chain" and chain_segments:
            data["chain_segments"] = list(chain_segments)
        else:
            if task_key is not None:
                data["task_key"] = task_key
            if scene is not None:
                data["scene"] = scene
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError:
        pass


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
    isaac_dir: Path,
    registry: dict[str, dict[str, Any]],
) -> bool:
    if str(isaac_dir.resolve()) != last.get("isaac_dir"):
        return False
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


def _format_motion_last_line(last: _MotionLastDict) -> str:
    rk = last.get("robot_key", "?")
    n = last.get("num_runs", 1)
    re_ = last.get("reset_env", True)
    reset_s = "yes" if re_ else "no"
    mode = last.get("mode", "single")
    if mode == "chain":
        segs = last.get("chain_segments") or []
        parts: list[str] = []
        for s in segs:
            if isinstance(s, dict):
                tk = s.get("task_key", "?")
                sc = s.get("scene", "?")
                parts.append(f"{tk}@{sc}")
        chain_s = " + ".join(parts) if parts else "?"
        return f"robot={rk}  mode=chain  segments=[{chain_s}]  runs={n}  reset_env={reset_s}"
    tk = last.get("task_key", "?")
    sc = last.get("scene", "?")
    scene_s = "ALL_SCENES" if sc == "__all__" else str(sc)
    return f"robot={rk}  task={tk}  scene={scene_s}  runs={n}  reset_env={reset_s}"


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
    runtime = build_merged_queue_config(
        runtime_defaults=base_root,
        skill_defaults=skill_defaults,
        scene_preset=scene_sd,
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


def _prompt_chain_segments(*, robot_entry: dict[str, Any]) -> list[_ChainSegmentDict]:
    """Interactive: append segments until user stops (same robot)."""
    from robot_action_composer.dataset_recording.launcher import (  # pyright: ignore[reportMissingImports]
        select_task_with_optional_group,
    )

    tasks_map: dict[str, Any] = robot_entry["tasks"]
    task_options = {key: {"label": str(meta.get("label", key))} for key, meta in tasks_map.items()}
    default_task_key = "pick_place" if "pick_place" in task_options else next(iter(task_options))
    segments: list[_ChainSegmentDict] = []
    while True:
        while True:
            task_key = select_task_with_optional_group(
                title_group="Select task folder (segment)",
                title_task="Select task for this segment",
                tasks=task_options,
                task_groups=robot_entry.get("task_groups", {}),
                default_task_key=default_task_key,
            )
            task_entry_one = tasks_map[task_key]
            scene_presets: dict[str, dict[str, object]] = task_entry_one["scene_presets"]
            scene_names = list(scene_presets.keys())
            if not scene_names:
                raise ValueError(f"Task {task_key!r} has no scene_presets")

            used_pairs = {(s["task_key"], s["scene"]) for s in segments}
            available_scenes = [sn for sn in scene_names if (task_key, sn) not in used_pairs]
            if not available_scenes:
                print(
                    "[info] This task has no unused scene presets left in the chain "
                    "(each task+scene pair can appear at most once). Pick another task, or Back."
                )
                continue

            preferred = str(task_entry_one.get("default_scene") or available_scenes[0])
            default_scene = preferred if preferred in available_scenes else available_scenes[0]

            scene_options: list[str] = list(available_scenes)
            if len(available_scenes) > 1:
                scene_options.append("__all__")

            scene = _select_option(
                title="Select config (scene) for this segment (unused presets only)",
                options=scene_options,
                default_value=default_scene,
                allow_back=True,
            )
            if scene == "__back__":
                continue
            break
        if scene == "__all__":
            print(
                f"[info] Segment uses ALL_SCENES — expanding to {len(available_scenes)} "
                f"sub-segment(s): {', '.join(available_scenes)}"
            )
            for sn in available_scenes:
                segments.append({"task_key": task_key, "scene": sn})
        else:
            segments.append({"task_key": task_key, "scene": scene})
        if len(segments) >= 30:
            print("[info] Reached 30 segments; finishing chain.")
            break
        more = input("Add another segment to the chain? [y/N]: ").strip().lower()
        if more not in {"y", "yes"}:
            break
    if not segments:
        raise ValueError("Chain has no segments")
    return segments


def run_motion_generation(*, isaac_dir: Path) -> None:
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        format_merged_queue_summary,
    )
    from robot_action_composer.dataset_recording.launcher import (  # pyright: ignore[reportMissingImports]
        prompt_positive_int,
        select_task_with_optional_group,
    )
    from robot_action_composer.ros_interface_utils import build_ros2_interface_from_robot_cfg  # pyright: ignore[reportMissingImports]
    from robot_action_composer.isaac_sim import SimTimeHelper  # pyright: ignore[reportMissingImports]

    registry = _discover_task_registry(isaac_dir)
    if not registry:
        raise RuntimeError("No motion-generation capable robot configs found under examples/IsaacSim/robots")

    print("IsaacSim Run Motion Generation")
    print("=" * 70)

    last = _load_motion_last()
    entry_kind: Literal["last_single", "last_chain", "interactive_single", "interactive_chain"] = (
        "interactive_single"
    )
    if last and _motion_last_applies(last, isaac_dir=isaac_dir, registry=registry):
        line = _format_motion_last_line(last)
        print("\nHow to run?")
        print(f"  1. Last selection — {line}")
        print("  2. Interactive — single task")
        print("  3. Interactive — multi-segment chain")
        raw_mode = input("Select [1/2/3] (Enter = 1): ").strip().lower()
        if raw_mode in ("", "1"):
            lm = last.get("mode", "single")
            entry_kind = "last_chain" if lm == "chain" else "last_single"
        elif raw_mode == "3":
            entry_kind = "interactive_chain"
        else:
            entry_kind = "interactive_single"
    else:
        print("\nConfigure motion generation:")
        print("  1. Single task")
        print("  2. Multi-segment chain")
        raw_cfg = input("Select [1/2] (Enter = 1): ").strip().lower()
        entry_kind = "interactive_chain" if raw_cfg == "2" else "interactive_single"

    robot_key: str
    num_runs: int
    reset_env: bool
    robot_entry: dict[str, Any]
    task_key: str = ""
    scene: str = ""
    task_entry: dict[str, Any] | None = None
    chain_segments: list[_ChainSegmentDict] = []

    if entry_kind == "last_single" and last:
        robot_key = str(last["robot_key"])
        task_key = str(last["task_key"])
        scene = str(last["scene"])
        num_runs = int(last.get("num_runs", 1))
        reset_env = bool(last.get("reset_env", True))
        robot_entry = registry[robot_key]
        task_entry = robot_entry["tasks"][task_key]
        print(f"\n[info] Using last selection: {_format_motion_last_line(last)}")
    elif entry_kind == "last_chain" and last:
        robot_key = str(last["robot_key"])
        num_runs = int(last.get("num_runs", 1))
        reset_env = bool(last.get("reset_env", True))
        robot_entry = registry[robot_key]
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
        print(f"\n[info] Using last selection: {_format_motion_last_line(last)}")
    else:
        robot_keys = list(registry.keys())
        default_robot = "dobot_cr5" if "dobot_cr5" in registry else robot_keys[0]
        robot_key = _select_option(title="Select robot", options=robot_keys, default_value=default_robot)
        robot_entry = registry[robot_key]

        if entry_kind == "interactive_chain":
            chain_segments = _prompt_chain_segments(robot_entry=robot_entry)
            num_runs = prompt_positive_int(
                "How many full-chain motion runs? (Enter = 1): ",
                default=1,
                min_value=1,
            )
            if num_runs > 1:
                reset_env = True
                print(
                    "[info] Multiple chain runs: each run resets on the first segment only "
                    "(same pattern as multi-scene motion)."
                )
            else:
                reset_env = _select_option(
                    title="Reset environment on first segment only?",
                    options=["yes", "no"],
                    default_value="yes",
                ) == "yes"
        else:
            tasks_map = robot_entry["tasks"]
            task_options = {
                key: {"label": str(meta.get("label", key))}
                for key, meta in tasks_map.items()
            }
            default_task_key = "pick_place" if "pick_place" in task_options else next(iter(task_options))
            while True:
                task_key = select_task_with_optional_group(
                    title_group="Select task folder",
                    title_task="Select task",
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
                    title="Select config",
                    options=(scene_names + ["__all__"]),
                    default_value=default_scene,
                    allow_back=True,
                )
                if scene == "__back__":
                    continue
                break

            num_runs = prompt_positive_int(
                "How many motion runs? (Enter = 1): ",
                default=1,
                min_value=1,
            )

            if num_runs > 1:
                reset_env = True
                print(
                    "[info] Multiple motion runs: each run will reset the environment "
                    "& randomize the object (same as record episodes)."
                )
            else:
                reset_env = _select_option(
                    title="Reset environment & randomize object?",
                    options=["yes", "no"],
                    default_value="yes",
                ) == "yes"

    if entry_kind in ("last_chain", "interactive_chain"):
        _save_motion_last(
            isaac_dir=isaac_dir,
            robot_key=robot_key,
            num_runs=num_runs,
            reset_env=reset_env,
            mode="chain",
            chain_segments=chain_segments,
        )
    else:
        assert task_entry is not None
        _save_motion_last(
            isaac_dir=isaac_dir,
            robot_key=robot_key,
            task_key=task_key,
            scene=scene,
            num_runs=num_runs,
            reset_env=reset_env,
            mode="single",
        )

    allowed = _merged_queue_allowed_keys()
    import robot_action_composer.task_runtime.skills  # noqa: F401 - register built-in skills

    from robot_action_composer.task_runtime.runner import (  # pyright: ignore[reportMissingImports]
        run_task_queue,
        run_task_queue_on_connected_interface,
    )

    if entry_kind in ("last_chain", "interactive_chain"):
        for run_idx in range(num_runs):
            interface = build_ros2_interface_from_robot_cfg(robot_entry["robot_cfg"])
            sim_time = SimTimeHelper()
            connected = False
            try:
                interface.connect()
                connected = True
                print("[OK] Robot connected (multi-segment chain)")
                for seg_idx, seg in enumerate(chain_segments):
                    tk = seg["task_key"]
                    task_entry_seg = robot_entry["tasks"][tk]
                    use_stamped_seg = task_entry_seg.get("use_stamped", True)
                    scene_name = seg["scene"]
                    runtime, merged_queue = _build_runtime_for_scene(
                        task_entry=task_entry_seg,
                        scene=scene_name,
                        allowed=allowed,
                    )
                    print(format_merged_queue_summary(scene_name, runtime))
                    should_reset_env = reset_env and (seg_idx == 0)
                    print(
                        f"\n{'=' * 70}\nChain run {run_idx + 1}/{num_runs} — "
                        f"segment {seg_idx + 1}/{len(chain_segments)}  "
                        f"task={tk!r}  scene={scene_name}\n{'=' * 70}"
                    )
                    if seg_idx > 0 and reset_env:
                        print(
                            "[info] Chain: skipping env reset for segments after the first "
                            "(robot state continues across segments)."
                        )
                    run_task_queue_on_connected_interface(
                        interface=interface,
                        sim_time=sim_time,
                        robot_cfg=robot_entry["robot_cfg"],
                        runtime=runtime,
                        blocks=merged_queue,
                        reset_env=should_reset_env,
                        use_stamped=use_stamped_seg,
                    )
            finally:
                sim_time.shutdown()
                if connected:
                    interface.disconnect()
                    print("[OK] Robot disconnected")
        return

    assert task_entry is not None
    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    use_stamped = task_entry.get("use_stamped", True)
    scenes_to_run = list(scene_presets.keys()) if scene == "__all__" else [scene]

    run_all_scenes = len(scenes_to_run) > 1
    for run_idx in range(num_runs):
        if run_all_scenes:
            interface = build_ros2_interface_from_robot_cfg(robot_entry["robot_cfg"])
            sim_time = SimTimeHelper()
            connected = False
            try:
                interface.connect()
                connected = True
                print("[OK] Robot connected (shared session for ALL_SCENES)")
                for scene_idx, scene_name in enumerate(scenes_to_run):
                    runtime, merged_queue = _build_runtime_for_scene(task_entry=task_entry, scene=scene_name, allowed=allowed)
                    print(format_merged_queue_summary(scene_name, runtime))
                    should_reset_env = reset_env and (scene_idx == 0)
                    print(
                        f"\n{'=' * 70}\nMotion run {run_idx + 1}/{num_runs} "
                        f"(scene {scene_idx + 1}/{len(scenes_to_run)}: {scene_name})\n{'=' * 70}"
                    )
                    if scene_idx > 0 and reset_env:
                        print("[info] ALL_SCENES mode: skip env reset for sub-scenes after the first one.")
                    run_task_queue_on_connected_interface(
                        interface=interface,
                        sim_time=sim_time,
                        robot_cfg=robot_entry["robot_cfg"],
                        runtime=runtime,
                        blocks=merged_queue,
                        reset_env=should_reset_env,
                        use_stamped=use_stamped,
                    )
            finally:
                sim_time.shutdown()
                if connected:
                    interface.disconnect()
                    print("[OK] Robot disconnected")
        else:
            scene_name = scenes_to_run[0]
            runtime, merged_queue = _build_runtime_for_scene(task_entry=task_entry, scene=scene_name, allowed=allowed)
            print(format_merged_queue_summary(scene_name, runtime))
            print(
                f"\n{'=' * 70}\nMotion run {run_idx + 1}/{num_runs} "
                f"(scene 1/1: {scene_name})\n{'=' * 70}"
            )
            run_task_queue(
                robot_cfg=robot_entry["robot_cfg"],
                runtime=runtime,
                blocks=merged_queue,
                reset_env=reset_env,
                use_stamped=use_stamped,
            )
