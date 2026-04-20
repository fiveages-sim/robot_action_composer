#!/usr/bin/env python3
"""IsaacSim dataset recording entry (robot_action_composer)."""

from __future__ import annotations

from dataclasses import fields, replace
from pathlib import Path
from typing import Any


def _merged_queue_allowed_keys() -> frozenset[str]:
    from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.bimanual_place import BimanualPlaceTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.task_runtime.config import QUEUE_SINGLE_ARM_KEYS  # pyright: ignore[reportMissingImports]

    names: set[str] = set(QUEUE_SINGLE_ARM_KEYS)
    names.add("base_link_entity_path")
    names.add("place_offset")
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


def _apply_task_preset_runtime(base: Any, preset_raw: dict[str, object]) -> Any:
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        MergedQueueConfig,
        merge_scene_preset_into_merged_queue,
    )

    if not isinstance(base, MergedQueueConfig):
        raise TypeError(f"Expected MergedQueueConfig, got {type(base)}")
    return merge_scene_preset_into_merged_queue(base, preset_raw)


def _build_task_runtime(task_entry_cfg: Any) -> Any:
    from robot_action_composer.task_config_io import queue_root_overrides
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        build_merged_queue_config,
    )

    if not (isinstance(task_entry_cfg, dict) and "base_task_overrides" in task_entry_cfg):
        raise TypeError("task YAML must be a dict with base_task_overrides")
    skill_defaults = dict(task_entry_cfg.get("skill_defaults") or {})
    return build_merged_queue_config(
        base_task_overrides=queue_root_overrides(task_entry_cfg["base_task_overrides"]),
        skill_defaults=skill_defaults,
        scene_preset=None,
    )


def _pointcloud_supported(robot_cfg: Any) -> bool:
    cameras = getattr(robot_cfg, "cameras", {}) or {}
    has_depth_topic = any(getattr(cam, "depth_topic_name", None) for cam in cameras.values())
    has_depth_info = getattr(robot_cfg, "depth_info_topic", None) is not None
    return bool(has_depth_topic and has_depth_info)


def run_record_datasets(*, isaac_dir: Path) -> None:
    from robot_action_composer.dataset_recording.launcher import (
        collect_runtime_options,
        select_option,
        select_task_with_optional_group,
    )
    from robot_action_composer.discovery.registry_loader import load_robot_entries
    from robot_action_composer.task_config_io import queue_root_overrides
    from robot_action_composer.task_runtime.merge import (  # pyright: ignore[reportMissingImports]
        merge_task_queue_skill_params,
    )

    discovered = load_robot_entries(isaac_dir)
    registry: dict[str, dict[str, Any]] = {}
    for key, entry in discovered.items():
        record = entry["record"]
        if not record:
            continue
        tasks: dict[str, dict[str, Any]] = {}
        for task_key, task_entry in record["tasks"].items():
            base_cfg = task_entry["record_cfg"]
            profiles = {}
            for profile in task_entry.get("profiles", []):
                overrides = profile.get("overrides", {})
                cfg = replace(base_cfg, **overrides) if overrides else base_cfg
                profiles[profile["key"]] = {
                    "label": profile.get("label", profile["key"]),
                    "record_cfg": cfg,
                }
            tasks[task_key] = {
                "label": task_entry.get("label", task_key),
                "task_cfg": task_entry["task_cfg"],
                "use_stamped": task_entry.get("use_stamped", True),
                "runner": task_entry["runner"],
                "record_profiles": profiles,
            }
        registry[key] = {
            "label": entry["label"],
            "robot_cfg": entry["robot_cfg"],
            "tasks": tasks,
            "task_groups": entry.get("task_groups", {}),
        }
    if not registry:
        raise RuntimeError("No record-capable robot configs found under examples/IsaacSim/robots")

    print("IsaacSim Record Datasets")
    print("=" * 70)
    robot_keys = list(registry.keys())
    default_robot = "dobot_cr5" if "dobot_cr5" in registry else robot_keys[0]
    robot_key = select_option(title="Select robot", options=registry, default_key=default_robot)
    robot_entry = registry[robot_key]

    task_keys = list(robot_entry["tasks"].keys())
    default_task = "pick_place" if "pick_place" in robot_entry["tasks"] else task_keys[0]
    task_key = select_task_with_optional_group(
        title_group="Select task folder",
        title_task="Select task",
        tasks=robot_entry["tasks"],
        task_groups=robot_entry.get("task_groups", {}),
        default_task_key=default_task,
    )
    task_entry = robot_entry["tasks"][task_key]
    task_runtime = _build_task_runtime(task_entry["task_cfg"])
    scene_presets: dict[str, dict[str, object]] = task_entry["task_cfg"].get("scene_presets", {})
    default_scene = task_entry["task_cfg"].get("default_scene")
    allowed = _merged_queue_allowed_keys()
    if scene_presets:
        scene_keys = list(scene_presets.keys())
        if default_scene not in scene_presets:
            default_scene = scene_keys[0]
        scene_options = {scene: {"label": scene} for scene in scene_keys}
        scene_key = select_option(
            title="Select scene",
            options=scene_options,
            default_key=default_scene,
        )
        preset_raw: dict[str, object] = dict(scene_presets.get(scene_key, {}))
        unknown = [k for k in queue_root_overrides(preset_raw) if k not in allowed]
        if unknown:
            raise ValueError(f"Unknown scene preset keys: {unknown}")
        task_runtime = _apply_task_preset_runtime(task_runtime, preset_raw)
    else:
        scene_key = "default"

    profile_keys = list(task_entry["record_profiles"].keys())
    default_profile = "default" if "default" in task_entry["record_profiles"] else profile_keys[0]
    profile_key = select_option(
        title="Select record profile",
        options=task_entry["record_profiles"],
        default_key=default_profile,
    )
    profile_entry = task_entry["record_profiles"][profile_key]

    loops, enable_keypoint_pcd, enable_manual_episode_check = collect_runtime_options(
        pointcloud_supported=_pointcloud_supported(robot_entry["robot_cfg"]),
        default_enable_keypoint_pcd=False,
    )
    print(
        f"[Selection] robot={robot_key}, task={task_key}, scene={scene_key}, profile={profile_key}, "
        f"episodes={loops}, pointcloud={enable_keypoint_pcd}, manual_review={enable_manual_episode_check}"
    )
    flow = task_entry["task_cfg"]
    tq = flow.get("task_queue")
    if not tq:
        raise ValueError(f"Recording task {task_key!r} requires a non-empty 'task_queue' in the task YAML")
    skill_defaults = dict(flow.get("skill_defaults") or {})
    scene_sd = scene_presets.get(scene_key, {}) if scene_presets else {}
    scene_skill_params = dict(scene_sd.get("skill_params") or {})
    merged_task_queue = merge_task_queue_skill_params(list(tq), skill_defaults, scene_skill_params)

    task_entry["runner"](
        robot_cfg=robot_entry["robot_cfg"],
        task_cfg=task_runtime,
        record_cfg=profile_entry["record_cfg"],
        loops=loops,
        enable_keypoint_pcd=enable_keypoint_pcd,
        enable_manual_episode_check=enable_manual_episode_check,
        task_name=task_key,
        use_stamped=bool(task_entry.get("use_stamped", True)),
        merged_task_queue=merged_task_queue,
    )
