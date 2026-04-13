#!/usr/bin/env python3
"""IsaacSim dataset recording entry (robot_action_composer)."""

from __future__ import annotations

from dataclasses import fields, replace
from pathlib import Path
from typing import Any


def _merged_queue_allowed_keys() -> frozenset[str]:
    from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.task_runtime.config import QUEUE_SINGLE_ARM_FLAT_KEYS  # pyright: ignore[reportMissingImports]

    names: set[str] = set(QUEUE_SINGLE_ARM_FLAT_KEYS)
    names.add("base_link_entity_path")
    for cls in (BimanualCarryTaskConfig, HandoverSyncConfig, DrawerGeometryConfig):
        names |= {f.name for f in fields(cls)}
    return frozenset(names)


def _apply_task_preset_runtime(base: Any, scene_flat: dict[str, object]) -> Any:
    from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
    from robot_action_composer.task_runtime.config.merged import (  # pyright: ignore[reportMissingImports]
        MergedQueueConfig,
        build_merged_queue_from_flat,
    )

    if not isinstance(base, MergedQueueConfig):
        raise TypeError(f"Expected MergedQueueConfig, got {type(base)}")
    merged: dict[str, object] = dict(base.single_arm.to_flat())
    if base.carry is not None:
        for f in fields(BimanualCarryTaskConfig):
            merged[f.name] = getattr(base.carry, f.name)
    if base.handover is not None:
        for f in fields(HandoverSyncConfig):
            merged[f.name] = getattr(base.handover, f.name)
    if base.drawer is not None:
        for f in fields(DrawerGeometryConfig):
            merged[f.name] = getattr(base.drawer, f.name)
    if base.base_link_entity_path is not None:
        merged["base_link_entity_path"] = base.base_link_entity_path
    merged.update(scene_flat)
    return build_merged_queue_from_flat(merged)


def _build_task_runtime(task_entry_cfg: Any) -> Any:
    from robot_action_composer.task_config_io import flatten_queue_task_overrides
    from robot_action_composer.task_runtime.merge import (  # pyright: ignore[reportMissingImports]
        merge_flat_with_skill_carry,
        merge_flat_with_skill_drawer,
        merge_flat_with_skill_handover,
        merge_flat_with_skill_pick_place,
    )
    from robot_action_composer.task_runtime.config.merged import build_merged_queue_from_flat  # pyright: ignore[reportMissingImports]

    if not (isinstance(task_entry_cfg, dict) and "base_task_overrides" in task_entry_cfg):
        raise TypeError("task YAML must be a dict with base_task_overrides")
    flat = merge_flat_with_skill_pick_place(
        flatten_queue_task_overrides(task_entry_cfg["base_task_overrides"]),
        task_entry_cfg.get("skill_defaults"),
    )
    flat = merge_flat_with_skill_handover(flat, task_entry_cfg.get("skill_defaults"))
    flat = merge_flat_with_skill_carry(flat, task_entry_cfg.get("skill_defaults"))
    flat = merge_flat_with_skill_drawer(flat, task_entry_cfg.get("skill_defaults"))
    return build_merged_queue_from_flat(flat)


def _pointcloud_supported(robot_cfg: Any) -> bool:
    cameras = getattr(robot_cfg, "cameras", {}) or {}
    has_depth_topic = any(getattr(cam, "depth_topic_name", None) for cam in cameras.values())
    has_depth_info = getattr(robot_cfg, "depth_info_topic", None) is not None
    return bool(has_depth_topic and has_depth_info)


def run_record_datasets(*, isaac_dir: Path) -> None:
    from robot_action_composer.dataset_recording.launcher import (
        collect_runtime_options,
        select_option,
    )
    from robot_action_composer.discovery.registry_loader import load_robot_entries
    from robot_action_composer.task_config_io import flatten_queue_task_overrides
    from robot_action_composer.task_runtime.merge import (  # pyright: ignore[reportMissingImports]
        merge_scene_skill_overlays_into_flat,
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
    task_key = select_option(
        title="Select task",
        options=robot_entry["tasks"],
        default_key=default_task,
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
        preset_flat = merge_scene_skill_overlays_into_flat(
            flatten_queue_task_overrides(preset_raw),
            preset_raw,
        )
        unknown = [k for k in preset_flat if k not in allowed]
        if unknown:
            raise ValueError(f"Unknown scene preset keys: {unknown}")
        task_runtime = _apply_task_preset_runtime(task_runtime, preset_flat)
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
