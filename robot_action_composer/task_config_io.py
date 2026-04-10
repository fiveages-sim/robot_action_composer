"""Load IsaacSim per-robot task configs from ``.yaml`` / ``.yml`` only."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

_FORBIDDEN_ROOT_KEYS: frozenset[str] = frozenset({"pick", "place", "handover", "carry", "drawer"})
# Consumed separately by merge_scene_skill_overlays_into_flat / merge_task_queue_skill_params, not part of flat preset keys.
_STRIPPED_ROOT_KEYS: frozenset[str] = frozenset({"skill_params"})


def flatten_queue_task_overrides(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Flatten ``base_task_overrides`` or a scene-preset root to top-level scalars only.

    Nested task sections are **not** supported — use ``skill_defaults`` / ``skill_params``
    (e.g. ``single_arm.pick``, ``dual_arm.handover``, ``dual_arm.carry``).
    """
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


def discover_task_configs(task_cfg_dir: Path, *, robot_dir_name: str) -> dict[str, dict[str, Any]]:
    """Return ``task_key -> task config dict`` for one robot's ``task_configs`` directory.

    Each task is defined by ``<stem>.yaml`` or ``<stem>.yml`` (one file per stem). Python
    task modules (``*.py``) are not loaded.
    """
    stems: set[str] = set()
    for p in task_cfg_dir.glob("*.yaml"):
        stems.add(p.stem)
    for p in task_cfg_dir.glob("*.yml"):
        stems.add(p.stem)

    tasks: dict[str, dict[str, Any]] = {}
    for stem in sorted(stems):
        yaml_path = task_cfg_dir / f"{stem}.yaml"
        yml_path = task_cfg_dir / f"{stem}.yml"
        if yaml_path.is_file() and yml_path.is_file():
            raise ValueError(
                f"Robot {robot_dir_name!r}: task {stem!r} has both {yaml_path.name} and {yml_path.name} "
                f"— keep only one."
            )
        path = yaml_path if yaml_path.is_file() else yml_path
        if not path.is_file():
            continue
        raw = load_task_dict_from_yaml(path)

        if not isinstance(raw, dict):
            continue
        task_key = raw.get("task_key")
        if not isinstance(task_key, str):
            raise ValueError(f"Task config {stem!r} must define string task_key: {task_cfg_dir}")
        tq = raw.get("task_queue")
        if not isinstance(tq, list) or len(tq) == 0:
            raise ValueError(
                f"Robot {robot_dir_name!r} task {task_key!r}: requires a non-empty list 'task_queue'."
            )
        tasks[task_key] = dict(raw)
    return tasks


__all__ = [
    "discover_task_configs",
    "flatten_queue_task_overrides",
    "load_task_dict_from_yaml",
]
