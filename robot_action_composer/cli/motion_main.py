#!/usr/bin/env python3
"""IsaacSim motion-generation launcher (robot_action_composer)."""

from __future__ import annotations

from dataclasses import fields
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, TypedDict


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


def _select_option(*, title: str, options: list[str], default_value: str) -> str:
    print(f"\n{title}")
    for idx, name in enumerate(options, start=1):
        suffix = " (default)" if name == default_value else ""
        print(f"  {idx}. {name}{suffix}")
    raw = input("Select option (press Enter for default): ").strip()
    if raw == "":
        return default_value
    if raw.isdigit():
        index = int(raw) - 1
        if 0 <= index < len(options):
            return options[index]
    if raw in options:
        return raw
    print(f"[info] Invalid option '{raw}', using default '{default_value}'.")
    return default_value


class _MotionLastDict(TypedDict, total=False):
    isaac_dir: str
    robot_key: str
    task_key: str
    scene: str
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
    task_key: str,
    scene: str,
    num_runs: int,
    reset_env: bool,
) -> None:
    path = _motion_last_file()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        data: _MotionLastDict = {
            "isaac_dir": str(isaac_dir.resolve()),
            "robot_key": robot_key,
            "task_key": task_key,
            "scene": scene,
            "num_runs": int(num_runs),
            "reset_env": bool(reset_env),
        }
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError:
        pass


def _motion_last_applies(
    last: _MotionLastDict,
    *,
    isaac_dir: Path,
    registry: dict[str, dict[str, Any]],
) -> bool:
    if str(isaac_dir.resolve()) != last.get("isaac_dir"):
        return False
    rk = last.get("robot_key")
    tk = last.get("task_key")
    sc = last.get("scene")
    if not isinstance(rk, str) or not isinstance(tk, str) or not isinstance(sc, str):
        return False
    robot_entry = registry.get(rk)
    if not robot_entry:
        return False
    tasks_map = robot_entry.get("tasks") or {}
    if tk not in tasks_map:
        return False
    scene_presets = tasks_map[tk].get("scene_presets") or {}
    if sc != "__all__" and sc not in scene_presets:
        return False
    if sc == "__all__" and not scene_presets:
        return False
    n = last.get("num_runs", 1)
    if not isinstance(n, int) or n < 1:
        return False
    return True


def _format_motion_last_line(last: _MotionLastDict) -> str:
    rk = last.get("robot_key", "?")
    tk = last.get("task_key", "?")
    sc = last.get("scene", "?")
    n = last.get("num_runs", 1)
    re_ = last.get("reset_env", True)
    reset_s = "yes" if re_ else "no"
    scene_s = "ALL_SCENES" if sc == "__all__" else str(sc)
    return f"robot={rk}  task={tk}  scene={scene_s}  runs={n}  reset_env={reset_s}"


def _build_runtime_for_scene(
    *,
    task_entry: dict[str, Any],
    scene: str,
    allowed: frozenset[str],
) -> tuple[Any, list[Any]]:
    from robot_action_composer.task_config_io import flatten_queue_task_overrides
    from robot_action_composer.task_runtime.merge import (  # pyright: ignore[reportMissingImports]
        merge_task_queue_skill_params,
    )
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        build_merged_queue_from_flat,
        merge_pick_place_skill_overlay,
    )

    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    scene_sd = scene_presets.get(scene, {})

    base_flat = flatten_queue_task_overrides(task_entry["base_task_overrides"])
    scene_flat = flatten_queue_task_overrides(scene_sd)
    unknown_scene = [k for k in scene_flat if k not in allowed]
    if unknown_scene:
        raise ValueError(f"Unknown scene preset keys for queue task: {unknown_scene}")

    common_flat = {**base_flat, **scene_flat}
    skill_defaults = dict(task_entry.get("skill_defaults") or {})
    scene_skill_params = dict(scene_sd.get("skill_params") or {})

    merged_flat = merge_pick_place_skill_overlay(common_flat, skill_defaults)
    merged_flat = merge_pick_place_skill_overlay(
        merged_flat,
        {
            "single_arm.pick": dict(scene_skill_params.get("single_arm.pick") or {}),
            "single_arm.place": dict(scene_skill_params.get("single_arm.place") or {}),
        },
    )
    merged_flat = {**merged_flat, **dict(skill_defaults.get("dual_arm.handover") or {})}
    merged_flat = {**merged_flat, **dict(scene_skill_params.get("dual_arm.handover") or {})}
    merged_flat = {**merged_flat, **dict(skill_defaults.get("dual_arm.carry") or {})}
    merged_flat = {**merged_flat, **dict(scene_skill_params.get("dual_arm.carry") or {})}
    merged_flat = {**merged_flat, **dict(skill_defaults.get("single_arm.drawer") or {})}
    merged_flat = {**merged_flat, **dict(scene_skill_params.get("single_arm.drawer") or {})}

    unknown_merged = [k for k in merged_flat if k not in allowed]
    if unknown_merged:
        raise ValueError(f"Unknown task/scene keys for queue task: {unknown_merged}")

    place_flat = {
        **dict(skill_defaults.get("dual_arm.place") or {}),
        **dict(scene_skill_params.get("dual_arm.place") or {}),
    }
    runtime = build_merged_queue_from_flat(merged_flat, place_flat if place_flat else None)

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
    from robot_action_composer.task_runtime.config import QUEUE_SINGLE_ARM_FLAT_KEYS  # pyright: ignore[reportMissingImports]

    names: set[str] = set(QUEUE_SINGLE_ARM_FLAT_KEYS)
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
    use_last = False
    if last and _motion_last_applies(last, isaac_dir=isaac_dir, registry=registry):
        line = _format_motion_last_line(last)
        print("\nHow to run?")
        print(f"  1. Last selection — {line}")
        print("  2. Interactive (choose robot / task / scene / …)")
        raw_mode = input("Select [1/2] (Enter = 1): ").strip().lower()
        if raw_mode in ("", "1"):
            use_last = True

    if use_last and last:
        robot_key = str(last["robot_key"])
        task_key = str(last["task_key"])
        scene = str(last["scene"])
        num_runs = int(last.get("num_runs", 1))
        reset_env = bool(last.get("reset_env", True))
        robot_entry = registry[robot_key]
        task_entry = robot_entry["tasks"][task_key]
        print(f"\n[info] Using last selection: {_format_motion_last_line(last)}")
    else:
        robot_keys = list(registry.keys())
        default_robot = "dobot_cr5" if "dobot_cr5" in registry else robot_keys[0]
        robot_key = _select_option(title="Select robot", options=robot_keys, default_value=default_robot)
        robot_entry = registry[robot_key]

        tasks_map = robot_entry["tasks"]
        task_options = {
            key: {"label": str(meta.get("label", key))}
            for key, meta in tasks_map.items()
        }
        default_task_key = "pick_place" if "pick_place" in task_options else next(iter(task_options))
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
        scene = _select_option(
            title="Select config",
            options=(scene_names + ["__all__"]),
            default_value=default_scene,
        )

        num_runs = prompt_positive_int(
            "How many motion runs? (Enter = 1): ",
            default=1,
            min_value=1,
        )

        run_all_scenes = scene == "__all__"
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
        if run_all_scenes and not reset_env:
            print("[info] scene=ALL_SCENES forces reset_env=yes (each scene should start from clean env).")
            reset_env = True

    _save_motion_last(
        isaac_dir=isaac_dir,
        robot_key=robot_key,
        task_key=task_key,
        scene=scene,
        num_runs=num_runs,
        reset_env=reset_env,
    )

    scene_presets: dict[str, dict[str, object]] = task_entry["scene_presets"]
    use_stamped = task_entry.get("use_stamped", True)
    scenes_to_run = list(scene_presets.keys()) if scene == "__all__" else [scene]

    allowed = _merged_queue_allowed_keys()
    import robot_action_composer.task_runtime.skills  # noqa: F401 - register built-in skills

    from robot_action_composer.task_runtime.runner import (  # pyright: ignore[reportMissingImports]
        run_task_queue,
        run_task_queue_on_connected_interface,
    )

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
                        robot_id=task_entry["robot_id"],
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
                robot_id=task_entry["robot_id"],
                blocks=merged_queue,
                reset_env=reset_env,
                use_stamped=use_stamped,
            )
