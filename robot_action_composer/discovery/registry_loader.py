#!/usr/bin/env python3
"""Auto-discovery loader for IsaacSim robot/task/record configs."""

from __future__ import annotations

import importlib.util
import sys
import warnings
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping

from robot_action_composer.config.lerobot_bridge import lerobot_profile_from_legacy, motion_profile_from_legacy
from robot_action_composer.config.robot_profiles import LeRobotRobotConfig, MotionRobotConfig
from robot_action_composer.config.robot_yaml import motion_profile_from_robot_yaml


def _load_module(module_name: str, file_path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec: {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except ImportError as exc:
        msg = str(exc).lower()
        if "lerobot" in msg:
            raise ImportError(
                f"Failed to load {file_path}: {exc}\n"
                "This robot profile requires lerobot packages. "
                "Run: ./init.sh install-lerobot\n"
                "Or split cameras into lerobot_config.py (see robot_action_composer/docs/ROBOT_CONFIG.md)."
            ) from exc
        raise
    return module


def _ensure_lerobot_package_paths(workspace_dir: Path) -> None:
    """Best-effort sys.path for lerobot plugin packages when loading lerobot_config."""
    if workspace_dir.name == "IsaacSim" and len(workspace_dir.parents) > 1:
        repo_root = workspace_dir.parents[1]
    else:
        repo_root = workspace_dir.parent
    for pkg_root in (repo_root / "lerobot_camera_ros2", repo_root / "lerobot_robot_ros2"):
        if pkg_root.is_dir() and str(pkg_root) not in sys.path:
            sys.path.insert(0, str(pkg_root))


def _is_robot_profile_dir(robot_dir: Path) -> bool:
    """True when ``robot_dir`` has motion config + ``task_configs/``."""
    if not robot_dir.is_dir() or robot_dir.name.startswith("__"):
        return False
    if not (robot_dir / "task_configs").is_dir():
        return False
    return any(
        (robot_dir / name).is_file()
        for name in ("robot.yaml", "motion_config.py", "robot_config.py")
    )


def _iter_robot_profile_dirs(robots_root: Path) -> list[Path]:
    """Discover robot profile dirs: ``robots/<Robot>`` or ``robots/<Vendor>/<Robot>``."""
    found: list[Path] = []
    for child in sorted(p for p in robots_root.iterdir() if p.is_dir() and not p.name.startswith("__")):
        if _is_robot_profile_dir(child):
            found.append(child)
            continue
        for nested in sorted(p for p in child.iterdir() if p.is_dir() and not p.name.startswith("__")):
            if _is_robot_profile_dir(nested):
                found.append(nested)
    return found


def resolve_robot_profile_dir(workspace_dir: Path, entry: Mapping[str, Any]) -> Path:
    """Absolute path to a robot profile directory.

    Prefers the already-resolved ``robot_dir`` (USD project workspaces keep their
    profiles outside ``<workspace>/robots``), otherwise joins the legacy layout.
    """
    resolved = entry.get("robot_dir")
    if resolved:
        return Path(str(resolved))
    robots_root = Path(str(entry.get("robots_root") or (Path(workspace_dir) / "robots")))
    relpath = str(entry.get("robot_dir_relpath") or entry.get("robot_dir_name") or "").strip()
    if not relpath:
        raise ValueError("robot entry missing robot_dir_relpath / robot_dir_name")
    return robots_root / relpath


def _load_motion_profile_from_dir(
    robot_dir: Path,
    *,
    robots_root: Path | None = None,
) -> tuple[MotionRobotConfig, str, str]:
    yaml_file = robot_dir / "robot.yaml"
    motion_file = robot_dir / "motion_config.py"
    legacy_file = robot_dir / "robot_config.py"

    if yaml_file.is_file():
        return motion_profile_from_robot_yaml(yaml_file, robots_root=robots_root)

    if motion_file.is_file():
        warnings.warn(
            f"{motion_file} is deprecated; use robot.yaml (see robot_action_composer/docs/ROBOT_CONFIG.md).",
            DeprecationWarning,
            stacklevel=2,
        )
        mod = _load_module(f"{robot_dir.name}_motion_cfg", motion_file)
        motion_cfg = getattr(mod, "MOTION_CFG", None)
        if motion_cfg is None:
            raise AttributeError(f"{motion_file} must define MOTION_CFG")
        robot_key = getattr(mod, "ROBOT_KEY", robot_dir.name.lower())
        robot_label = getattr(mod, "ROBOT_LABEL", robot_dir.name)
        return motion_cfg, str(robot_key), str(robot_label)

    if legacy_file.is_file():
        mod = _load_module(f"{robot_dir.name}_robot_cfg", legacy_file)
        legacy_cfg = getattr(mod, "ROBOT_CFG", None) or getattr(mod, "MOTION_CFG", None)
        if legacy_cfg is None:
            raise AttributeError(f"{legacy_file} must define ROBOT_CFG or MOTION_CFG")
        if getattr(legacy_cfg, "cameras", None):
            warnings.warn(
                f"{legacy_file} defines cameras on the combined config; "
                "split into motion_config.py + lerobot_config.py for motion-only installs.",
                DeprecationWarning,
                stacklevel=2,
            )
        robot_key = getattr(mod, "ROBOT_KEY", robot_dir.name.lower())
        robot_label = getattr(mod, "ROBOT_LABEL", robot_dir.name)
        return motion_profile_from_legacy(legacy_cfg), str(robot_key), str(robot_label)

    raise FileNotFoundError(f"No robot.yaml, motion_config.py, or robot_config.py in {robot_dir}")


def load_lerobot_profile(robot_dir: Path, *, workspace_dir: Path | None = None) -> LeRobotRobotConfig | None:
    lerobot_file = robot_dir / "lerobot_config.py"
    if lerobot_file.is_file():
        if workspace_dir is not None:
            _ensure_lerobot_package_paths(workspace_dir)
        mod = _load_module(f"{robot_dir.name}_lerobot_cfg", lerobot_file)
        lerobot_cfg = getattr(mod, "LEROBOT_CFG", None)
        if lerobot_cfg is None:
            raise AttributeError(f"{lerobot_file} must define LEROBOT_CFG")
        return lerobot_cfg

    legacy_file = robot_dir / "robot_config.py"
    if legacy_file.is_file():
        mod = _load_module(f"{robot_dir.name}_legacy_robot_cfg", legacy_file)
        legacy_cfg = getattr(mod, "ROBOT_CFG", None)
        if legacy_cfg is None:
            return None
        profile = lerobot_profile_from_legacy(legacy_cfg)
        if profile is not None:
            warnings.warn(
                f"Using cameras from legacy {legacy_file}; prefer lerobot_config.py.",
                DeprecationWarning,
                stacklevel=2,
            )
        return profile

    return None


def load_motion_entries(
    workspace_dir: Path,
    *,
    robots_root: Path | str | None = None,
) -> dict[str, dict[str, Any]]:
    """Discover robot entries plus their task configs.

    ``<workspace>/robots`` keeps the legacy behaviour. A FaSim USD project folder
    (``<project>/env``) instead resolves its shared profile via ``robot.yaml``
    (``extends`` / ``robots_root``) and reads tasks from ``<project>/task_configs``.
    """
    from robot_action_composer.discovery.workspace_layout import load_workspace_layout

    layout = load_workspace_layout(workspace_dir, robots_root=robots_root)
    if layout.legacy:
        return _load_legacy_motion_entries(layout.robots_root)
    if layout.scene_project is None:
        return {}
    return _load_scene_project_motion_entries(layout)


def _load_legacy_motion_entries(robots_root: Path) -> dict[str, dict[str, Any]]:
    from robot_action_composer.task_config_io import discover_task_configs

    entries: dict[str, dict[str, Any]] = {}
    if not robots_root.is_dir():
        return entries

    for robot_dir in _iter_robot_profile_dirs(robots_root):
        robot_dir_relpath = robot_dir.relative_to(robots_root).as_posix()
        task_cfg_dir = robot_dir / "task_configs"

        motion_cfg, robot_key, robot_label = _load_motion_profile_from_dir(
            robot_dir, robots_root=robots_root
        )
        discovery = discover_task_configs(task_cfg_dir, robot_dir_name=robot_dir_relpath)
        if not discovery.tasks:
            continue

        if robot_key in entries:
            prev = entries[robot_key].get("robot_dir_relpath", "?")
            raise ValueError(
                f"Duplicate robot key {robot_key!r} in {robot_dir_relpath!r} and {prev!r}"
            )

        entries[robot_key] = {
            "label": robot_label,
            "robot_dir_name": robot_dir.name,
            "robot_dir_relpath": robot_dir_relpath,
            "robot_dir": str(robot_dir),
            "robots_root": str(robots_root),
            "scene_project": None,
            "project_robot_yaml": None,
            "motion_cfg": motion_cfg,
            "robot_cfg": motion_cfg,
            "lerobot_cfg": None,
            "tasks": discovery.tasks,
            "task_groups": discovery.task_groups,
            "task_group_dirs": discovery.task_group_dirs,
        }
    return entries


def _load_scene_project_motion_entries(layout: Any) -> dict[str, dict[str, Any]]:
    """Load a FaSim USD project: shared robot profile + project task YAML."""
    from robot_action_composer.config.robot_yaml import (
        motion_profile_from_robot_yaml,
        resolve_profile_dir_for_key,
    )
    from robot_action_composer.discovery.workspace_layout import read_project_robot_yaml
    from robot_action_composer.task_config_io import TaskConfigDiscovery, discover_task_configs

    project = Path(layout.scene_project)
    yaml_path = layout.project_robot_yaml
    if yaml_path is None:
        raise ValueError(
            f"{project} is a USD project (has env/) but no robot.yaml.\n"
            "Add one with `extends: <robot_key>` and `robots_root: ...` "
            "(see robot_action_composer/docs/ROBOT_CONFIG.md)."
        )
    project_data = read_project_robot_yaml(project)

    motion_cfg, robot_key, robot_label = motion_profile_from_robot_yaml(
        Path(yaml_path), robots_root=layout.robots_root
    )
    base_dir = resolve_profile_dir_for_key(layout.robots_root, robot_key)

    if layout.task_configs_root is not None:
        discovery = discover_task_configs(
            layout.task_configs_root,
            robot_dir_name=project.name,
        )
    else:
        discovery = TaskConfigDiscovery(tasks={}, task_groups={}, task_group_dirs={})
    if not discovery.tasks:
        raise ValueError(
            f"USD project {project} has no task YAML; expected {layout.task_configs_root or project / 'task_configs'}"
        )

    return {
        robot_key: {
            "label": robot_label,
            "robot_dir_name": base_dir.name,
            "robot_dir_relpath": base_dir.relative_to(layout.robots_root).as_posix(),
            "robot_dir": str(base_dir),
            "robots_root": str(layout.robots_root),
            "scene_project": str(project),
            "project_robot_yaml": str(yaml_path),
            "default_env_scene": project_data.get("default_env_scene"),
            "motion_cfg": motion_cfg,
            "robot_cfg": motion_cfg,
            "lerobot_cfg": None,
            "tasks": discovery.tasks,
            "task_groups": discovery.task_groups,
            "task_group_dirs": discovery.task_group_dirs,
        }
    }


def load_robot_entries(
    workspace_dir: Path,
    *,
    robots_root: Path | str | None = None,
) -> dict[str, dict[str, Any]]:
    from robot_action_composer.dataset_recording.runner import DEFAULT_RECORD_CFG, run_recording

    motion_entries = load_motion_entries(workspace_dir, robots_root=robots_root)
    runner = run_recording
    base_record_cfg = DEFAULT_RECORD_CFG
    entries: dict[str, dict[str, Any]] = {}
    if not motion_entries:
        return entries

    for robot_key, motion_entry in motion_entries.items():
        robot_dir = resolve_robot_profile_dir(workspace_dir, motion_entry)
        lerobot_cfg = load_lerobot_profile(robot_dir, workspace_dir=workspace_dir)
        robot_label = motion_entry["label"]
        motion_cfg = motion_entry["motion_cfg"]
        flow_tasks: dict[str, dict[str, Any]] = motion_entry.get("tasks", {})
        record_entry = None
        record_tasks: dict[str, dict[str, Any]] = {}
        for task_key, flow_cfg in flow_tasks.items():
            record_cfg_section = flow_cfg.get("record", {})
            if not record_cfg_section:
                continue
            if lerobot_cfg is None:
                warnings.warn(
                    f"Task {task_key!r} has record section but robot {robot_key!r} has no lerobot_config.py; skipping.",
                    UserWarning,
                    stacklevel=2,
                )
                continue
            base_overrides = record_cfg_section.get("base_record_overrides", {})
            task_record_cfg = replace(base_record_cfg, **base_overrides) if base_overrides else base_record_cfg
            profiles = record_cfg_section.get(
                "profiles",
                [{"key": "default", "label": "Default", "overrides": {}}],
            )
            record_tasks[task_key] = {
                "label": flow_cfg.get("label", task_key),
                "task_cfg": flow_cfg,
                "use_stamped": flow_cfg.get("use_stamped", True),
                "runner": runner,
                "record_cfg": task_record_cfg,
                "profiles": profiles,
            }
        if record_tasks:
            record_entry = {"tasks": record_tasks}

        entries[robot_key] = {
            "label": robot_label,
            "motion_cfg": motion_cfg,
            "lerobot_cfg": lerobot_cfg,
            "robot_cfg": motion_cfg,
            "robot_dir_name": motion_entry.get("robot_dir_name", ""),
            "robot_dir_relpath": motion_entry.get("robot_dir_relpath", ""),
            "robot_dir": motion_entry.get("robot_dir", ""),
            "robots_root": motion_entry.get("robots_root", ""),
            "scene_project": motion_entry.get("scene_project"),
            "project_robot_yaml": motion_entry.get("project_robot_yaml"),
            "default_env_scene": motion_entry.get("default_env_scene"),
            "record": record_entry,
            "task_groups": motion_entry.get("task_groups", {}),
            "task_group_dirs": motion_entry.get("task_group_dirs", {}),
        }
    return entries
