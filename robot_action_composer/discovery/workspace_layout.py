"""Resolve a workspace into robot profiles plus task-config roots.

Two layouts are supported:

* **legacy**: ``<workspace>/robots/<Robot>/robot.yaml`` with per-robot
  ``task_configs/`` (``examples/IsaacSim``).
* **USD project** (FaSim ``fa-project-usd`` convention): ``<project>/env/``
  marks a runnable simulation project, ``<project>/robot.yaml`` overlays a
  shared robot profile (``extends: <robot_key>``), and task YAML lives in
  ``<project>/task_configs/``.

The project layout is auto-detected; ``<workspace>/robots`` always wins when both
exist so existing checkouts keep working unchanged.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

#: Environment variables tried (in order) when a USD project does not declare ``robots_root``.
ROBOTS_ROOT_ENV_VARS: tuple[str, ...] = (
    "ROBOT_ACTION_COMPOSER_ROBOTS_ROOT",
    "LEROBOT_ISAACSIM_ROBOTS",
)

#: Value printed in error messages to show the expected ``robots_root`` shape.
ROBOTS_ROOT_EXAMPLE = "${LEROBOT_ROOT}/examples/IsaacSim/robots"

_EXPAND_RE = re.compile(
    r"\$(?:\{(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?::-(?P<default>[^}]*))?\}|(?P<plain>[A-Za-z_][A-Za-z0-9_]*))"
)


class WorkspaceLayoutError(ValueError):
    """Raised when a workspace cannot be resolved into robots + task configs."""


def detect_lerobot_root() -> Path | None:
    """Best-effort lerobot checkout root (for ``${LEROBOT_ROOT}`` when it is unset).

    Works for the usual editable install (``<root>/submodules/robot_action_composer``)
    and for a package installed under ``<root>/.venv``; returns ``None`` otherwise.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "examples" / "IsaacSim" / "robots").is_dir():
            return parent
    return None


def expand_path(text: str, *, base: Path | None = None) -> Path:
    """Expand ``~``, ``$VAR`` / ``${VAR}`` / ``${VAR:-default}`` then resolve against ``base``.

    ``${LEROBOT_ROOT}`` falls back to the detected lerobot checkout when the variable
    is unset, so a project ``robot.yaml`` works without exporting anything.
    """

    def _sub(match: re.Match[str]) -> str:
        name = match.group("name") or match.group("plain")
        default = match.group("default")
        value = os.environ.get(name or "")
        if value:
            return value
        if default is not None:
            return default
        if name == "LEROBOT_ROOT":
            detected = detect_lerobot_root()
            if detected is not None:
                return str(detected)
        raise WorkspaceLayoutError(
            f"environment variable {name!r} is not set (used in path {text!r}). "
            f"Export it, or pass --robots-root {ROBOTS_ROOT_EXAMPLE.replace('${LEROBOT_ROOT}', '<lerobot-root>')}"
        )

    expanded = _EXPAND_RE.sub(_sub, text)
    path = Path(expanded).expanduser()
    if not path.is_absolute() and base is not None:
        path = Path(base) / path
    return path.resolve(strict=False)


def is_usd_project(path: Path) -> bool:
    """True when ``path`` looks like a FaSim ``fa-project-usd`` project (has ``env/``)."""
    return (Path(path) / "env").is_dir()


def _is_robot_profile_dir(path: Path) -> bool:
    if not path.is_dir() or path.name.startswith("__"):
        return False
    if not (path / "task_configs").is_dir():
        return False
    return any(
        (path / name).is_file() for name in ("robot.yaml", "motion_config.py", "robot_config.py")
    )


def has_robot_profiles(robots_root: Path) -> bool:
    """True when ``robots_root`` holds a legacy robot profile (``robots/<X>`` or ``robots/<V>/<X>``).

    FaSim USD projects also contain ``robots/`` (USD layers), so the legacy layout must
    be detected by an actual profile rather than by the directory name alone.
    """
    root = Path(robots_root)
    if not root.is_dir():
        return False
    for child in root.iterdir():
        if not child.is_dir() or child.name.startswith("__"):
            continue
        if _is_robot_profile_dir(child):
            return True
        for nested in child.iterdir():
            if _is_robot_profile_dir(nested):
                return True
    return False


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:  # pragma: no cover - PyYAML is a declared dependency
        raise ImportError("Loading robot.yaml requires PyYAML. Install with: pip install pyyaml") from err
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise TypeError(f"{path}: YAML root must be a mapping")
    return data


def read_project_robot_yaml(project_root: Path) -> dict[str, Any]:
    """Load ``<project>/robot.yaml`` (scene overlay) or return ``{}`` when absent."""
    path = Path(project_root) / "robot.yaml"
    if not path.is_file():
        return {}
    data = _load_yaml_mapping(path)
    data.setdefault("_config_path", str(path))
    return data


def _robots_root_from_mapping(data: Mapping[str, Any], *, base: Path) -> Path | None:
    raw = data.get("robots_root")
    if raw in (None, ""):
        return None
    return expand_path(str(raw), base=base)


def _robots_root_from_env() -> Path | None:
    for name in ROBOTS_ROOT_ENV_VARS:
        value = os.environ.get(name)
        if value:
            return expand_path(value)
    return None


@dataclass(frozen=True)
class WorkspaceLayout:
    """Resolved workspace roots."""

    workspace_dir: Path
    robots_root: Path
    legacy: bool
    scene_project: Path | None = None
    """USD project root (``<project>`` with ``env/``) when ``legacy`` is False."""

    project_robot_yaml: Path | None = None
    """``<project>/robot.yaml`` scene overlay (may be absent)."""

    task_configs_root: Path | None = None
    """``<project>/task_configs`` when the scene project provides one."""

    @property
    def project_root(self) -> Path | None:
        return self.scene_project


def resolve_robots_root(
    workspace_dir: Path,
    *,
    cli_robots_root: Path | str | None = None,
    project_data: Mapping[str, Any] | None = None,
) -> Path | None:
    """Resolve the robot profile root (CLI > project > legacy ``robots/`` > env)."""
    if cli_robots_root is not None:
        return expand_path(str(cli_robots_root), base=Path.cwd())
    if project_data:
        from_mapping = _robots_root_from_mapping(project_data, base=Path(workspace_dir))
        if from_mapping is not None:
            return from_mapping
    legacy = Path(workspace_dir) / "robots"
    if has_robot_profiles(legacy):
        return legacy.resolve()
    return _robots_root_from_env()


def load_workspace_layout(
    workspace_dir: Path,
    *,
    robots_root: Path | str | None = None,
) -> WorkspaceLayout:
    """Resolve ``workspace_dir`` into a :class:`WorkspaceLayout`.

    Raises :class:`WorkspaceLayoutError` with an actionable message when neither a
    legacy ``robots/`` tree nor a USD project (``env/``) can be found.
    """
    workspace = Path(workspace_dir).expanduser().resolve()
    legacy_robots = workspace / "robots"
    if has_robot_profiles(legacy_robots):
        resolved = resolve_robots_root(workspace, cli_robots_root=robots_root)
        assert resolved is not None
        return WorkspaceLayout(workspace_dir=workspace, robots_root=resolved, legacy=True)

    if not is_usd_project(workspace):
        if legacy_robots.is_dir():
            resolved = resolve_robots_root(workspace, cli_robots_root=robots_root)
            assert resolved is not None
            return WorkspaceLayout(workspace_dir=workspace, robots_root=resolved, legacy=True)
        raise WorkspaceLayoutError(
            f"{workspace} is neither a legacy workspace (needs robots/<Robot>/ with robot.yaml "
            f"and task_configs/) nor a USD project (needs env/). "
            "Run from examples/IsaacSim or from a fa-project-usd project folder."
        )

    project_data = read_project_robot_yaml(workspace)
    resolved = resolve_robots_root(workspace, cli_robots_root=robots_root, project_data=project_data)
    if resolved is None:
        raise WorkspaceLayoutError(
            f"{workspace} is a USD project but no robot profiles were found. "
            f"Add `robots_root: {ROBOTS_ROOT_EXAMPLE}` to {workspace / 'robot.yaml'}, "
            f"or pass --robots-root, or export {ROBOTS_ROOT_ENV_VARS[0]}"
        )
    if not resolved.is_dir():
        raise WorkspaceLayoutError(
            f"robots_root {resolved} does not exist (from {workspace / 'robot.yaml'} or --robots-root)"
        )

    return WorkspaceLayout(
        workspace_dir=workspace,
        robots_root=resolved,
        legacy=False,
        scene_project=workspace,
        project_robot_yaml=(workspace / "robot.yaml") if (workspace / "robot.yaml").is_file() else None,
        task_configs_root=(workspace / "task_configs")
        if (workspace / "task_configs").is_dir()
        else None,
    )


__all__ = [
    "ROBOTS_ROOT_ENV_VARS",
    "ROBOTS_ROOT_EXAMPLE",
    "WorkspaceLayout",
    "WorkspaceLayoutError",
    "detect_lerobot_root",
    "expand_path",
    "has_robot_profiles",
    "is_usd_project",
    "load_workspace_layout",
    "read_project_robot_yaml",
    "resolve_robots_root",
]
