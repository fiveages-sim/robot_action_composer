"""Load IsaacSim per-robot task configs from ``.yaml`` / ``.yml`` only."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Mapping

_FORBIDDEN_ROOT_KEYS: frozenset[str] = frozenset({"pick", "place", "handover", "carry", "drawer"})
# skill_params / objects / active_object live on task or scene root but are not runtime_defaults
_STRIPPED_ROOT_KEYS: frozenset[str] = frozenset({"skill_params", "objects", "active_object"})
RUNTIME_DEFAULTS_ALLOWED_KEYS: frozenset[str] = frozenset(
    {
        "base_link_entity_path",
        # Stamped / object-pose frame. ``world`` ⇒ no frozen-base subtraction (WBC-safe).
        # Otherwise often the TF frame matching the Isaac base prim pose (e.g. G1D
        # footprint, not TF base_link which sits +0.1114 m higher).
        "frame_id",
        "max_stage_duration",
        "pose_tol_pos",
        "pose_tol_ori",
        "gripper_open",
        "gripper_closed",
    }
)


def queue_root_overrides(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Extract queue root overrides (excluding nested skill sections)."""
    if not isinstance(raw, Mapping):
        raise TypeError(f"task overrides must be a mapping, got {type(raw).__name__}")
    ctx = "runtime_defaults or scene preset"
    for key in _FORBIDDEN_ROOT_KEYS:
        if key in raw:
            raise ValueError(
                f'{ctx}: root key "{key}" is not allowed. '
                "Use skill_defaults / skill_params (see motion CLI merge order)."
            )
    skip = _FORBIDDEN_ROOT_KEYS | _STRIPPED_ROOT_KEYS
    return {k: v for k, v in raw.items() if k not in skip}


def validate_runtime_defaults_keys(raw: Mapping[str, Any], *, context: str) -> dict[str, Any]:
    """Validate ``runtime_defaults``-like root keys and return stripped root overrides."""
    root = queue_root_overrides(raw)
    unknown = [k for k in root if k not in RUNTIME_DEFAULTS_ALLOWED_KEYS]
    if unknown:
        raise ValueError(
            f"{context}: unknown runtime_defaults keys: {unknown}. "
            f"Allowed keys: {sorted(RUNTIME_DEFAULTS_ALLOWED_KEYS)}"
        )
    return root


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
    """Parent directory relative to ``task_configs`` (posix); root files use ``\"\"``."""
    rel = yaml_path.relative_to(task_cfg_dir)
    if len(rel.parts) <= 1:
        return ""
    return "/".join(rel.parts[:-1])


def _group_path_parts(group_id: str) -> tuple[str, ...]:
    if not group_id:
        return ()
    return tuple(p for p in group_id.replace("\\", "/").split("/") if p)


def _is_strict_path_prefix(prefix: str, path: str) -> bool:
    """True when ``prefix`` is a strict directory prefix of ``path`` (posix segments)."""
    pref = _group_path_parts(prefix)
    full = _group_path_parts(path)
    if len(pref) >= len(full):
        return False
    return full[: len(pref)] == pref


def _validate_leaf_only_task_groups(
    task_groups: dict[str, list[str]],
    *,
    robot_dir_name: str,
) -> None:
    """Reject YAML in a directory that also has YAML under a subdirectory."""
    group_ids = [gid for gid, keys in task_groups.items() if keys]
    for a in group_ids:
        for b in group_ids:
            if a == b:
                continue
            if _is_strict_path_prefix(a, b):
                a_label = a if a else "(task_configs root)"
                raise ValueError(
                    f"Robot {robot_dir_name!r}: task YAML in {a_label!r} and under "
                    f"{b!r} — only leaf directories may contain task YAML "
                    f"(move files out of intermediate dirs)."
                )


@dataclass(frozen=True)
class TaskConfigDiscovery:
    """YAML task registry plus leaf-folder grouping for interactive menus."""

    tasks: dict[str, dict[str, Any]]
    """Flat ``task_key -> config`` (unique keys across the tree)."""

    task_groups: dict[str, list[str]]
    """Leaf group id (``\"\"`` = files directly under ``task_configs/``) -> task_key list."""

    task_group_dirs: dict[str, str] = field(default_factory=dict)
    """Leaf group id -> absolute directory holding that leaf's task YAML.

    Used to resolve ``.meta/ros2_stack.yaml`` / ``.meta/objects`` for config roots
    that are not ``<robot_dir>/task_configs`` (e.g. a FaSim USD project folder).
    """

    def merged(self, other: "TaskConfigDiscovery") -> "TaskConfigDiscovery":
        """Union two discoveries; duplicate ``task_key`` is an error naming both files."""
        duplicates = sorted(set(self.tasks) & set(other.tasks))
        if duplicates:
            details = ", ".join(
                f"{key!r} ({self.tasks[key].get('_config_path')} vs "
                f"{other.tasks[key].get('_config_path')})"
                for key in duplicates
            )
            raise ValueError(f"duplicate task_key across task config roots: {details}")

        groups = {gid: list(keys) for gid, keys in self.task_groups.items()}
        for gid, keys in other.task_groups.items():
            groups[gid] = sorted({*groups.get(gid, ()), *keys})
        group_dirs = {**self.task_group_dirs, **other.task_group_dirs}
        return TaskConfigDiscovery(
            tasks={**self.tasks, **other.tasks},
            task_groups={gid: groups[gid] for gid in sorted(groups, key=lambda g: (g != "", g))},
            task_group_dirs=group_dirs,
        )

def _prefix_group_id(prefix: str, group_id: str) -> str:
    """Namespace a leaf group id under ``prefix`` (posix join, empty stays empty)."""
    prefix = prefix.strip("/")
    if not prefix:
        return group_id
    return f"{prefix}/{group_id}" if group_id else prefix


def discover_task_configs(
    task_cfg_dir: Path,
    *,
    robot_dir_name: str,
    group_prefix: str = "",
) -> TaskConfigDiscovery:
    """Load all task YAML under ``task_cfg_dir`` and assign each to its parent-dir group.

    Recursively loads ``*.yaml`` / ``*.yml``. Group id is the parent directory relative
    to ``task_cfg_dir`` (posix path); files directly in ``task_cfg_dir`` use ``\"\"``.
    YAML may only live in leaf directories: a directory with YAML must not also have
    YAML in any subdirectory. In each directory, a basename may use either
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
    group_dirs: dict[str, str] = {}
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
            entry = dict(raw)
            entry["_config_path"] = str(path)
            entry["_config_dir"] = str(path.parent)
            tasks[task_key] = entry
            group_id = _prefix_group_id(group_prefix, _task_group_id(path, task_cfg_dir))
            group_keys[group_id].append(task_key)
            group_dirs[group_id] = str(path.parent)

    task_groups = {gid: sorted(keys) for gid, keys in sorted(group_keys.items(), key=lambda x: (x[0] != "", x[0]))}
    _validate_leaf_only_task_groups(task_groups, robot_dir_name=robot_dir_name)
    return TaskConfigDiscovery(
        tasks=tasks,
        task_groups=task_groups,
        task_group_dirs=group_dirs,
    )


ENV_SCENE_SUFFIXES: tuple[str, ...] = (".usd", ".usda", ".usdc")


def iter_env_scenes(project_root: Path) -> Iterator[Path]:
    """Yield runnable USD scenes under ``<project>/env`` (FaSim project layout)."""
    env_root = Path(project_root) / "env"
    if not env_root.is_dir():
        return
    for suffix in ENV_SCENE_SUFFIXES:
        yield from sorted(p for p in env_root.rglob(f"*{suffix}") if p.is_file())


def resolve_env_scene(
    task_cfg: Mapping[str, Any],
    *,
    project_root: Path | None,
    default_env_scene: str | None = None,
) -> Path | None:
    """Validate ``env_scene`` (relative to the USD project root) and return it.

    Returns ``None`` when the task has no ``env_scene`` or when the workspace is not a
    USD project (legacy ``<workspace>/robots`` layout), where the field is optional.
    ``default_env_scene`` (project ``robot.yaml``) is used when the task omits it.
    """
    raw = task_cfg.get("env_scene")
    if raw in (None, "") and default_env_scene:
        raw = default_env_scene
    if raw in (None, ""):
        return None
    scene_rel = str(raw).strip()
    scene_path = Path(scene_rel)
    if scene_path.is_absolute():
        raise ValueError(
            f"task env_scene must be relative to the USD project root, got absolute path: {scene_rel!r}"
        )
    if project_root is None:
        return None
    target = (Path(project_root) / scene_path).resolve()
    if target.is_file():
        return target
    candidates = [p.relative_to(Path(project_root)).as_posix() for p in iter_env_scenes(project_root)]
    hint = ", ".join(candidates) if candidates else f"(no .usd/.usda under {Path(project_root) / 'env'})"
    raise FileNotFoundError(
        f"task env_scene {scene_rel!r} not found under {project_root} (resolved: {target}). "
        f"Available scenes: {hint}"
    )


__all__ = [
    "ENV_SCENE_SUFFIXES",
    "RUNTIME_DEFAULTS_ALLOWED_KEYS",
    "TaskConfigDiscovery",
    "discover_task_configs",
    "iter_env_scenes",
    "resolve_env_scene",
    "validate_runtime_defaults_keys",
    "queue_root_overrides",
    "load_task_dict_from_yaml",
]
