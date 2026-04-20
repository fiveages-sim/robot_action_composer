"""Load IsaacSim per-robot task configs from ``.yaml`` / ``.yml`` only."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

_FORBIDDEN_ROOT_KEYS: frozenset[str] = frozenset({"pick", "place", "handover", "carry", "drawer"})
_STRIPPED_ROOT_KEYS: frozenset[str] = frozenset({"skill_params"})


def queue_root_overrides(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Extract queue root overrides (excluding nested skill sections)."""
    if not isinstance(raw, Mapping):
        raise TypeError(f"task overrides must be a mapping, got {type(raw).__name__}")
    ctx = "base_task_overrides or scene preset"
    for key in _FORBIDDEN_ROOT_KEYS:
        if key in raw:
            raise ValueError(
                f'{ctx}: root key "{key}" is not allowed. '
                "Use skill_defaults / skill_params (see motion CLI merge order)."
            )
    skip = _FORBIDDEN_ROOT_KEYS | _STRIPPED_ROOT_KEYS
    return {k: v for k, v in raw.items() if k not in skip}


def _normalize_numeric_lists(obj: Any) -> Any:
    """YAML expresses tuples as lists; dataclasses often expect ``tuple`` for poses/orientations.

    Rule: a non-empty list made only of int/float becomes ``tuple``. Nested dict/list structures
    are traversed; lists of dicts (e.g. ``task_queue``, ``profiles``) are preserved as lists.
    """
    if isinstance(obj, dict):
        return {k: _normalize_numeric_lists(v) for k, v in obj.items()}
    if isinstance(obj, list):
        if obj and all(isinstance(x, (int, float)) for x in obj):
            return tuple(obj)
        return [_normalize_numeric_lists(x) for x in obj]
    return obj


def load_task_dict_from_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:
        raise ImportError(
            "Loading task config from YAML requires PyYAML. Install with: pip install pyyaml"
        ) from err
    text = path.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    if data is None:
        raise ValueError(f"YAML task config is empty: {path}")
    if not isinstance(data, dict):
        raise TypeError(f"YAML task config must be a mapping at root, got {type(data).__name__}: {path}")
    return _normalize_numeric_lists(data)  # type: ignore[return-value]


def _should_skip_task_cfg_path(path: Path, *, root: Path) -> bool:
    """Skip hidden path segments and ``__pycache__`` under ``task_configs``."""
    try:
        rel = path.relative_to(root)
    except ValueError:
        return True
    return any(part == "__pycache__" or part.startswith(".") for part in rel.parts)


def _task_group_id(yaml_path: Path, task_cfg_dir: Path) -> str:
    """First-level folder under ``task_configs``; direct children use ``\"\"`` (root group)."""
    rel = yaml_path.relative_to(task_cfg_dir)
    if len(rel.parts) <= 1:
        return ""
    return rel.parts[0]


@dataclass(frozen=True)
class TaskConfigDiscovery:
    """YAML task registry plus one-level folder grouping for interactive menus."""

    tasks: dict[str, dict[str, Any]]
    """Flat ``task_key -> config`` (unique keys across the tree)."""

    task_groups: dict[str, list[str]]
    """Group id (``\"\"`` = files directly under ``task_configs/``) -> task_key list."""


def discover_task_configs(task_cfg_dir: Path, *, robot_dir_name: str) -> TaskConfigDiscovery:
    """Load all task YAML under ``task_cfg_dir`` and assign each task to a first-level group.

    Recursively loads ``*.yaml`` / ``*.yml``. Files directly in ``task_cfg_dir`` belong to
    group ``\"\"``; ``task_cfg_dir / <folder> / ...`` uses group id ``<folder>`` (deeper
    paths still count under that folder). In each directory, a basename may use either
    ``.yaml`` or ``.yml``, not both. Python task modules (``*.py``) are not loaded.
    Paths under hidden segments or ``__pycache__`` are ignored.
    """
    stems_by_parent: dict[Path, set[str]] = {}
    for pattern in ("*.yaml", "*.yml"):
        for p in task_cfg_dir.rglob(pattern):
            if not p.is_file() or _should_skip_task_cfg_path(p, root=task_cfg_dir):
                continue
            if not p.stem:
                continue
            stems_by_parent.setdefault(p.parent, set()).add(p.stem)

    tasks: dict[str, dict[str, Any]] = {}
    task_key_paths: dict[str, Path] = {}
    group_keys: dict[str, list[str]] = defaultdict(list)
    for parent in sorted(stems_by_parent.keys(), key=lambda d: str(d.relative_to(task_cfg_dir))):
        for stem in sorted(stems_by_parent[parent]):
            yaml_path = parent / f"{stem}.yaml"
            yml_path = parent / f"{stem}.yml"
            if yaml_path.is_file() and yml_path.is_file():
                raise ValueError(
                    f"Robot {robot_dir_name!r}: task {stem!r} in {parent} has both "
                    f"{yaml_path.name} and {yml_path.name} — keep only one."
                )
            path = yaml_path if yaml_path.is_file() else yml_path
            if not path.is_file():
                continue
            raw = load_task_dict_from_yaml(path)

            if not isinstance(raw, dict):
                continue
            task_key = raw.get("task_key")
            if not isinstance(task_key, str):
                raise ValueError(
                    f"Task config {path.relative_to(task_cfg_dir)!s} must define string task_key: {task_cfg_dir}"
                )
            tq = raw.get("task_queue")
            if not isinstance(tq, list) or len(tq) == 0:
                raise ValueError(
                    f"Robot {robot_dir_name!r} task {task_key!r}: requires a non-empty list 'task_queue'."
                )
            if task_key in tasks:
                prev = task_key_paths[task_key]
                raise ValueError(
                    f"Robot {robot_dir_name!r}: duplicate task_key {task_key!r} in "
                    f"{path} and {prev}"
                )
            task_key_paths[task_key] = path
            tasks[task_key] = dict(raw)
            group_keys[_task_group_id(path, task_cfg_dir)].append(task_key)

    task_groups = {gid: sorted(keys) for gid, keys in sorted(group_keys.items(), key=lambda x: (x[0] != "", x[0]))}
    return TaskConfigDiscovery(tasks=tasks, task_groups=task_groups)


__all__ = [
    "TaskConfigDiscovery",
    "discover_task_configs",
    "queue_root_overrides",
    "load_task_dict_from_yaml",
]
