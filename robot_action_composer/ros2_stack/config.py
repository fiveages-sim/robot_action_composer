"""Load, merge, and resolve ros2_stack configuration."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Literal, Mapping, Sequence

from robot_action_composer.ros2_stack.presets import (
    DEFAULT_NAV_LAUNCH_FILE,
    DEFAULT_NAV_PACKAGE,
    DEFAULT_NAV_READY_SUBSTRINGS,
    _FORBIDDEN_STACK_KEYS,
    resolve_motion_preset,
    validate_nav_profile,
)

RequiredMode = Literal["true", "false", "auto"] | bool


@dataclass
class ReadyWhen:
    node_name_contains: list[str] = field(default_factory=list)
    match: Literal["any", "all"] = "any"


@dataclass
class ResolvedComponent:
    name: Literal["motion", "navigation"]
    required: RequiredMode
    package: str
    launch_file: str
    args: dict[str, Any] = field(default_factory=dict)
    extra_args: list[str] = field(default_factory=list)
    ready_when: ReadyWhen = field(default_factory=ReadyWhen)
    preset: str | None = None
    profile: str | None = None

    def build_launch_argv(self) -> list[str]:
        argv = ["ros2", "launch", self.package, self.launch_file]
        for key, value in self.args.items():
            if value is None:
                continue
            argv.append(f"{key}:={value}")
        argv.extend(str(a) for a in self.extra_args)
        return argv

    def launch_command_str(self) -> str:
        return " ".join(self.build_launch_argv())


@dataclass
class ResolvedRos2Stack:
    motion: ResolvedComponent | None = None
    navigation: ResolvedComponent | None = None
    defaults_args: dict[str, Any] = field(default_factory=dict)

    def has_any(self) -> bool:
        return self.motion is not None or self.navigation is not None


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:
        raise ImportError(
            "Loading ros2_stack YAML requires PyYAML. Install with: pip install pyyaml"
        ) from err
    text = path.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    if data is None:
        raise ValueError(f"YAML is empty: {path}")
    if not isinstance(data, dict):
        raise TypeError(f"YAML root must be a mapping: {path}")
    return data


def _reject_task_keys(data: Mapping[str, Any], *, context: str) -> None:
    bad = sorted(k for k in data if k in _FORBIDDEN_STACK_KEYS)
    if bad:
        raise ValueError(
            f"{context}: forbidden task-orchestration keys {bad}; "
            "ros2_stack must not contain task_key/task_queue/etc."
        )


def load_ros2_stack_yaml(path: Path) -> dict[str, Any]:
    """Load a leaf ``.meta/ros2_stack.yaml`` (flat roots: defaults/motion/navigation)."""
    data = _load_yaml_mapping(path)
    _reject_task_keys(data, context=str(path))
    allowed = {"defaults", "motion", "navigation"}
    unknown = sorted(k for k in data if k not in allowed)
    if unknown:
        raise ValueError(f"{path}: unknown keys {unknown}; allowed {sorted(allowed)}")
    return dict(data)


def load_robot_ros2_stack_section(robot_yaml: Path) -> dict[str, Any]:
    """Load optional ``ros2_stack:`` section from ``robot.yaml``."""
    if not robot_yaml.is_file():
        return {}
    data = _load_yaml_mapping(robot_yaml)
    section = data.get("ros2_stack")
    if section is None:
        return {}
    if not isinstance(section, dict):
        raise TypeError(f"{robot_yaml}: ros2_stack must be a mapping")
    _reject_task_keys(section, context=f"{robot_yaml} ros2_stack")
    allowed = {"defaults", "motion", "navigation"}
    unknown = sorted(k for k in section if k not in allowed)
    if unknown:
        raise ValueError(
            f"{robot_yaml}: unknown ros2_stack keys {unknown}; allowed {sorted(allowed)}"
        )
    return dict(section)


def deep_merge_dicts(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    """Deep-merge mappings; overlay wins. Nested dicts merge; other values replace."""
    out: dict[str, Any] = dict(base)
    for key, value in overlay.items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = deep_merge_dicts(out[key], value)
        else:
            out[key] = value
    return out


def meta_ros2_stack_path(task_configs_dir: Path, group_id: str) -> Path:
    leaf = task_configs_dir if not group_id else task_configs_dir / Path(group_id)
    return leaf / ".meta" / "ros2_stack.yaml"


def find_task_group_id(task_groups: Mapping[str, Sequence[str]], task_key: str) -> str:
    for gid, keys in task_groups.items():
        if task_key in keys:
            return gid
    return ""


def warn_misplaced_ros2_stack(task_configs_dir: Path, group_id: str) -> str | None:
    """Return a warning if ros2_stack.yaml was placed at the leaf root instead of .meta/."""
    leaf = task_configs_dir if not group_id else task_configs_dir / Path(group_id)
    for name in ("ros2_stack.yaml", "ros2_stack.yml"):
        misplaced = leaf / name
        if misplaced.is_file():
            return (
                f"Found {misplaced.name} at task leaf root; move it to "
                f"{leaf / '.meta' / 'ros2_stack.yaml'}"
            )
    return None


def _normalize_required(raw: Any, *, default: RequiredMode) -> RequiredMode:
    if raw is None:
        return default
    if isinstance(raw, bool):
        return raw
    text = str(raw).strip().lower()
    if text in {"true", "yes", "1"}:
        return True
    if text in {"false", "no", "0"}:
        return False
    if text == "auto":
        return "auto"
    raise ValueError(f"required must be true/false/auto, got {raw!r}")


def _parse_ready_when(raw: Any, *, defaults: Sequence[str]) -> ReadyWhen:
    if raw is None:
        return ReadyWhen(node_name_contains=list(defaults), match="any")
    if not isinstance(raw, Mapping):
        raise TypeError("ready_when must be a mapping")
    match = str(raw.get("match", "any")).strip().lower()
    if match not in {"any", "all"}:
        raise ValueError(f"ready_when.match must be any|all, got {match!r}")
    contains = raw.get("node_name_contains", list(defaults))
    if not isinstance(contains, (list, tuple)):
        raise TypeError("ready_when.node_name_contains must be a list")
    return ReadyWhen(
        node_name_contains=[str(x) for x in contains],
        match=match,  # type: ignore[arg-type]
    )


def _component_args(
    defaults_args: Mapping[str, Any],
    component: Mapping[str, Any],
) -> dict[str, Any]:
    args = dict(defaults_args)
    raw_args = component.get("args") or {}
    if not isinstance(raw_args, Mapping):
        raise TypeError("args must be a mapping")
    if "nav2_profile" in raw_args:
        raise ValueError(
            "Do not set args.nav2_profile; use navigation.profile (default|map_only)"
        )
    args.update(dict(raw_args))
    return args


def resolve_ros2_stack(
    merged: Mapping[str, Any],
    *,
    robot_key: str | None = None,
) -> ResolvedRos2Stack:
    """Expand presets/profiles into resolvable launch components."""
    defaults = merged.get("defaults") or {}
    if defaults is not None and not isinstance(defaults, Mapping):
        raise TypeError("defaults must be a mapping")
    defaults_args = dict((defaults or {}).get("args") or {})
    if robot_key and "robot" not in defaults_args:
        defaults_args["robot"] = robot_key

    motion_out: ResolvedComponent | None = None
    motion_raw = merged.get("motion")
    if motion_raw is not None:
        if not isinstance(motion_raw, Mapping):
            raise TypeError("motion must be a mapping")
        if "package" in motion_raw or "launch_file" in motion_raw:
            raise ValueError(
                "motion must use preset (ocs2-fullbody|ocs2-split-body|ocs2-demo); "
                "do not set package/launch_file"
            )
        preset_name = motion_raw.get("preset")
        if not preset_name:
            raise ValueError("motion.preset is required when motion section is present")
        spec = resolve_motion_preset(str(preset_name))
        ready = _parse_ready_when(
            motion_raw.get("ready_when"),
            defaults=spec.ready_node_substrings,
        )
        extra = motion_raw.get("extra_args") or []
        if not isinstance(extra, (list, tuple)):
            raise TypeError("motion.extra_args must be a list")
        motion_out = ResolvedComponent(
            name="motion",
            required=_normalize_required(motion_raw.get("required"), default=True),
            package=spec.package,
            launch_file=spec.launch_file,
            args=_component_args(defaults_args, motion_raw),
            extra_args=[str(x) for x in extra],
            ready_when=ready,
            preset=str(preset_name).strip(),
        )

    nav_out: ResolvedComponent | None = None
    nav_raw = merged.get("navigation")
    if nav_raw is not None:
        if not isinstance(nav_raw, Mapping):
            raise TypeError("navigation must be a mapping")
        profile = validate_nav_profile(str(nav_raw.get("profile", "default")))
        package = str(nav_raw.get("package") or DEFAULT_NAV_PACKAGE)
        launch_file = str(nav_raw.get("launch_file") or DEFAULT_NAV_LAUNCH_FILE)
        ready = _parse_ready_when(
            nav_raw.get("ready_when"),
            defaults=DEFAULT_NAV_READY_SUBSTRINGS,
        )
        extra = nav_raw.get("extra_args") or []
        if not isinstance(extra, (list, tuple)):
            raise TypeError("navigation.extra_args must be a list")
        args = _component_args(defaults_args, nav_raw)
        args["nav2_profile"] = profile
        nav_out = ResolvedComponent(
            name="navigation",
            required=_normalize_required(nav_raw.get("required"), default="auto"),
            package=package,
            launch_file=launch_file,
            args=args,
            extra_args=[str(x) for x in extra],
            ready_when=ready,
            profile=profile,
        )

    return ResolvedRos2Stack(
        motion=motion_out,
        navigation=nav_out,
        defaults_args=defaults_args,
    )


def load_merged_ros2_stack(
    *,
    robot_dir: Path,
    group_id: str = "",
    robot_key: str | None = None,
) -> tuple[dict[str, Any], ResolvedRos2Stack | None, list[str]]:
    """Load robot.yaml ros2_stack + optional leaf .meta overlay; return raw merge + resolved.

    Returns ``(merged_raw, resolved_or_none, warnings)``. ``resolved`` is None when no stack config.
    """
    warnings: list[str] = []
    robot_yaml = robot_dir / "robot.yaml"
    base = load_robot_ros2_stack_section(robot_yaml)
    task_configs = robot_dir / "task_configs"
    meta_path = meta_ros2_stack_path(task_configs, group_id)
    overlay: dict[str, Any] = {}
    if meta_path.is_file():
        overlay = load_ros2_stack_yaml(meta_path)
    else:
        tip = warn_misplaced_ros2_stack(task_configs, group_id)
        if tip:
            warnings.append(tip)

    if not base and not overlay:
        return {}, None, warnings

    merged = deep_merge_dicts(base, overlay)
    resolved = resolve_ros2_stack(merged, robot_key=robot_key)
    if not resolved.has_any():
        return merged, None, warnings
    return merged, resolved, warnings


def apply_set_overrides(
    merged: dict[str, Any],
    sets: Sequence[str],
) -> dict[str, Any]:
    """Apply ``--set path.to.key=value`` overlays (e.g. ``motion.preset=ocs2-demo``)."""
    out = dict(merged)
    for item in sets:
        if "=" not in item:
            raise ValueError(f"--set expects path=value, got {item!r}")
        path, value = item.split("=", 1)
        parts = [p for p in path.strip().split(".") if p]
        if not parts:
            raise ValueError(f"empty --set path: {item!r}")
        # Convenience: navigation.map -> navigation.args.map
        if parts == ["navigation", "map"]:
            parts = ["navigation", "args", "map"]
        if parts == ["motion", "type"]:
            parts = ["motion", "args", "type"]
        cursor: dict[str, Any] = out
        for part in parts[:-1]:
            nxt = cursor.get(part)
            if not isinstance(nxt, dict):
                nxt = {}
                cursor[part] = nxt
            cursor = nxt
        cursor[parts[-1]] = _coerce_set_value(value)
    return out


def _coerce_set_value(raw: str) -> Any:
    text = raw.strip()
    lower = text.lower()
    if lower in {"true", "false"}:
        return lower == "true"
    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return text


def task_queue_needs_navigation(task_queue: Sequence[Any] | None) -> bool:
    """True when task_queue (possibly nested parallel) contains a ``nav.*`` skill."""
    if not task_queue:
        return False

    def walk(items: Sequence[Any]) -> bool:
        for item in items:
            if not isinstance(item, Mapping):
                continue
            if "parallel" in item:
                par = item.get("parallel")
                if isinstance(par, Sequence) and walk(par):
                    return True
                continue
            skill = item.get("skill")
            if isinstance(skill, str) and skill.startswith("nav."):
                return True
        return False

    return walk(task_queue)


def component_is_needed(
    component: ResolvedComponent,
    *,
    task_needs_nav: bool,
) -> bool:
    req = component.required
    if req is True or req == "true":
        return True
    if req is False or req == "false":
        return False
    # auto
    if component.name == "navigation":
        return task_needs_nav
    return True


def with_motion_preset(resolved: ResolvedRos2Stack, preset: str) -> ResolvedRos2Stack:
    """Return a copy with motion re-resolved for a different preset (keep args)."""
    if resolved.motion is None:
        raise ValueError("no motion component to override")
    merged = {
        "defaults": {"args": dict(resolved.defaults_args)},
        "motion": {
            "required": resolved.motion.required,
            "preset": preset,
            "args": {
                k: v
                for k, v in resolved.motion.args.items()
                if k not in resolved.defaults_args or resolved.defaults_args.get(k) != v
            },
            "extra_args": list(resolved.motion.extra_args),
        },
    }
    # Keep full motion args (including defaults) by passing them explicitly
    merged["motion"]["args"] = {
        k: v for k, v in resolved.motion.args.items() if k not in ("nav2_profile",)
    }
    # Strip defaults that will be re-applied
    for k in list(merged["motion"]["args"]):
        if k in resolved.defaults_args and merged["motion"]["args"][k] == resolved.defaults_args[k]:
            # keep them in args for clarity — resolve will merge defaults anyway
            pass
    if resolved.navigation is not None:
        nav_args = {
            k: v for k, v in resolved.navigation.args.items() if k != "nav2_profile"
        }
        merged["navigation"] = {
            "required": resolved.navigation.required,
            "package": resolved.navigation.package,
            "launch_file": resolved.navigation.launch_file,
            "profile": resolved.navigation.profile or "default",
            "args": nav_args,
            "extra_args": list(resolved.navigation.extra_args),
            "ready_when": {
                "match": resolved.navigation.ready_when.match,
                "node_name_contains": list(resolved.navigation.ready_when.node_name_contains),
            },
        }
    return resolve_ros2_stack(merged, robot_key=str(resolved.defaults_args.get("robot") or "") or None)


def with_nav_profile(resolved: ResolvedRos2Stack, profile: str) -> ResolvedRos2Stack:
    if resolved.navigation is None:
        raise ValueError("no navigation component to override")
    validate_nav_profile(profile)
    nav = replace(
        resolved.navigation,
        profile=profile,
        args={**resolved.navigation.args, "nav2_profile": profile},
    )
    return replace(resolved, navigation=nav)


__all__ = [
    "ReadyWhen",
    "ResolvedComponent",
    "ResolvedRos2Stack",
    "apply_set_overrides",
    "component_is_needed",
    "deep_merge_dicts",
    "find_task_group_id",
    "load_merged_ros2_stack",
    "load_robot_ros2_stack_section",
    "load_ros2_stack_yaml",
    "meta_ros2_stack_path",
    "resolve_ros2_stack",
    "task_queue_needs_navigation",
    "warn_misplaced_ros2_stack",
    "with_motion_preset",
    "with_nav_profile",
]
