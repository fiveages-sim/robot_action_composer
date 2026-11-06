"""Load per-robot ``robot.yaml`` into :class:`MotionRobotConfig`."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_action_composer.config.robot_profiles import (
    EeTipPrimPair,
    MotionRobotConfig,
    default_motion_ros2_interface,
)

_MOTION_FIELD_NAMES: frozenset[str] = frozenset(
    {
        "gripper_control_mode",
        "base_link_entity_path",
        "fsm_switch_delay",
        "post_reset_wait",
        "arrival_timeout",
        "arrival_poll",
        "gripper_action_wait",
        "ee_target_frame",
        "ee_command_frame",
        "ee_tip",
    }
)
_ROS2_PRESETS: frozenset[str] = frozenset({"auto", "single_arm", "ocs2_single_arm"})

#: Extra root keys understood by a scene overlay (USD project ``robot.yaml``).
_OVERLAY_ROOT_KEYS: frozenset[str] = frozenset({"extends", "robots_root", "default_env_scene"})


def _load_yaml_dict(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:
        raise ImportError(
            "Loading robot.yaml requires PyYAML. Install with: pip install pyyaml"
        ) from err
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        raise ValueError(f"robot.yaml is empty: {path}")
    if not isinstance(data, dict):
        raise TypeError(f"robot.yaml root must be a mapping, got {type(data).__name__}: {path}")
    return data


def _parse_ee_tip(raw: Any, *, path: Path) -> dict[str, EeTipPrimPair] | None:
    """Parse ``ee_tip: {left|right: {parent_prim, tip_prim, parent_frame?}}``."""
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise TypeError(f"{path}: 'ee_tip' must be a mapping if present")
    if not raw:
        return None
    out: dict[str, EeTipPrimPair] = {}
    for side, entry in raw.items():
        side_key = str(side).strip().lower()
        if side_key not in {"left", "right"}:
            raise ValueError(
                f"{path}: ee_tip keys must be 'left' and/or 'right', got {side!r}"
            )
        if not isinstance(entry, dict):
            raise TypeError(f"{path}: ee_tip.{side_key} must be a mapping")
        parent = str(entry.get("parent_prim") or "").strip()
        tip = str(entry.get("tip_prim") or "").strip()
        parent_frame = str(entry.get("parent_frame") or "").strip()
        unknown = [k for k in entry if k not in {"parent_prim", "tip_prim", "parent_frame"}]
        if unknown:
            raise ValueError(
                f"{path}: ee_tip.{side_key} unknown keys: {sorted(unknown)}"
            )
        if not parent or not tip:
            raise ValueError(
                f"{path}: ee_tip.{side_key} requires non-empty parent_prim and tip_prim"
            )
        out[side_key] = EeTipPrimPair(
            parent_prim=parent, tip_prim=tip, parent_frame=parent_frame
        )
    return out or None


def _build_ros2_interface(section: dict[str, Any] | None) -> Any:
    from ros2_robot_interface import ROS2RobotInterfaceConfig

    raw = dict(section or {})
    preset = str(raw.pop("preset", "auto"))
    if preset not in _ROS2_PRESETS:
        raise ValueError(
            f"Unknown ros2_interface.preset {preset!r}; expected one of {sorted(_ROS2_PRESETS)}"
        )

    if preset == "auto":
        thresholds: dict[str, float] = {}
        for key in ("pose_position_threshold", "pose_orientation_threshold"):
            if key in raw:
                thresholds[key] = float(raw.pop(key))
        if raw:
            raise ValueError(
                f"ros2_interface preset=auto does not accept keys: {sorted(raw)}"
            )
        return default_motion_ros2_interface(**thresholds)

    if preset == "single_arm":
        return ROS2RobotInterfaceConfig.default_single_arm(**raw)

    return ROS2RobotInterfaceConfig.default_single_arm_ocs2_arm_controller(**raw)


def resolve_profile_dir_for_key(robots_root: Path, robot_key: str) -> Path:
    """Locate the robot profile directory whose ``robot.yaml`` declares ``key: robot_key``."""
    root = Path(robots_root)
    if not root.is_dir():
        raise FileNotFoundError(f"robots_root does not exist: {root}")
    for candidate in sorted(root.rglob("robot.yaml")):
        try:
            data = _load_yaml_dict(candidate)
        except (TypeError, ValueError):
            continue
        if str(data.get("key") or "") == robot_key:
            return candidate.parent
    raise FileNotFoundError(
        f"no robot profile with key {robot_key!r} under {root}; "
        "check `extends:` in the scene robot.yaml or pass --robots-root"
    )


def _resolve_extends_path(
    path: Path,
    data: dict[str, Any],
    robots_root: Path | None,
) -> Path | None:
    """Resolve ``extends`` to the base ``robot.yaml`` (by robot key or relative path)."""
    extends = str(data.get("extends") or "").strip()
    if not extends:
        return None

    root = Path(robots_root) if robots_root is not None else None
    if root is None:
        declared = data.get("robots_root")
        if declared not in (None, ""):
            from robot_action_composer.discovery.workspace_layout import expand_path

            root = expand_path(str(declared), base=path.parent)
    if root is None:
        raise ValueError(
            f"{path}: 'extends: {extends}' needs a robots root; add `robots_root: ...` "
            "to this file, pass --robots-root, or export ROBOT_ACTION_COMPOSER_ROBOTS_ROOT"
        )

    if "/" in extends or extends.startswith("."):
        candidate = (root / extends).resolve()
        if candidate.is_file():
            return candidate
        if (candidate / "robot.yaml").is_file():
            return candidate / "robot.yaml"
        raise FileNotFoundError(f"{path}: extends path {extends!r} not found under {root}")

    return resolve_profile_dir_for_key(root, extends) / "robot.yaml"


def motion_profile_from_robot_yaml(
    path: Path,
    *,
    robots_root: Path | None = None,
    _chain: tuple[Path, ...] = (),
) -> tuple[MotionRobotConfig, str, str]:
    """Parse ``robot.yaml`` → ``(MotionRobotConfig, robot_key, robot_label)``.

    A USD project ``robot.yaml`` may declare ``extends: <robot_key>`` (plus
    ``robots_root``) to inherit a shared robot profile and only override the keys it
    needs; the merged result is what every consumer sees.
    """
    path = Path(path).resolve()
    if path in _chain:
        raise ValueError(
            "robot.yaml 'extends' cycle: " + " -> ".join(str(p) for p in (*_chain, path))
        )
    data = _load_yaml_dict(path)

    base_cfg: MotionRobotConfig | None = None
    base_key: str | None = None
    base_label: str | None = None
    base_path = _resolve_extends_path(path, data, robots_root)
    if base_path is not None:
        base_cfg, base_key, base_label = motion_profile_from_robot_yaml(
            base_path, robots_root=robots_root, _chain=(*_chain, path)
        )

    robot_key = data.get("key") or base_key
    robot_label = data.get("label") or base_label
    if not robot_key or not robot_label:
        raise ValueError(
            f"{path} must define non-empty 'key' and 'label' (or inherit them via 'extends')"
        )
    if base_key is not None and data.get("key") not in (None, "") and str(robot_key) != base_key:
        raise ValueError(
            f"{path}: 'key' {robot_key!r} differs from extends base key {base_key!r}; "
            "a scene overlay must keep the shared robot key"
        )

    motion_section = data.get("motion")
    if motion_section is not None and not isinstance(motion_section, dict):
        raise TypeError(f"{path}: 'motion' must be a mapping if present")

    ros2_section = data.get("ros2_interface")
    if ros2_section is not None and not isinstance(ros2_section, dict):
        raise TypeError(f"{path}: 'ros2_interface' must be a mapping if present")

    ros2_stack_section = data.get("ros2_stack")
    if ros2_stack_section is not None and not isinstance(ros2_stack_section, dict):
        raise TypeError(f"{path}: 'ros2_stack' must be a mapping if present")

    motion_kwargs: dict[str, Any] = {}
    for source in (data, motion_section or {}):
        for name in _MOTION_FIELD_NAMES:
            if name in source:
                motion_kwargs[name] = source[name]

    if "ee_tip" in motion_kwargs:
        motion_kwargs["ee_tip"] = _parse_ee_tip(motion_kwargs["ee_tip"], path=path)

    unknown_root = [
        k
        for k in data
        if k
        not in {
            "key",
            "label",
            "motion",
            "ros2_interface",
            "ros2_stack",
            *_MOTION_FIELD_NAMES,
            *_OVERLAY_ROOT_KEYS,
        }
    ]
    if unknown_root:
        raise ValueError(f"{path}: unknown root keys: {sorted(unknown_root)}")

    if motion_section:
        unknown_motion = [k for k in motion_section if k not in _MOTION_FIELD_NAMES]
        if unknown_motion:
            raise ValueError(f"{path}: unknown motion keys: {sorted(unknown_motion)}")

    if base_cfg is None:
        motion_cfg = MotionRobotConfig(
            ros2_interface=_build_ros2_interface(ros2_section),
            **motion_kwargs,
        )
    else:
        merged_kwargs = {name: getattr(base_cfg, name) for name in _MOTION_FIELD_NAMES}
        merged_kwargs.update(motion_kwargs)
        ros2_interface = (
            _build_ros2_interface(ros2_section) if ros2_section is not None else base_cfg.ros2_interface
        )
        motion_cfg = MotionRobotConfig(ros2_interface=ros2_interface, **merged_kwargs)

    return motion_cfg, str(robot_key), str(robot_label)
