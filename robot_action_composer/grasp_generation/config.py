"""Load ``grasp_generation.yaml`` (robot profile) with package defaults."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional

import yaml


def _package_default_yaml() -> Path:
    return Path(__file__).resolve().parent / "default_config.yaml"


@dataclass
class GraspGenerationConfig:
    """Runtime config for ``grasp-generation`` CLI / serve."""

    robot_key: str = "fiveages_w2"
    arm: str = "right"  # right | left
    motion_frame_id: str = "base_link"
    base_frame: str = "base_link"
    # TF candidates for head optical / camera link (tried in order)
    camera_frames: list[str] = field(
        default_factory=lambda: [
            "head_camera",
            "head_camera_optical_frame",
            "head_link2",
        ]
    )
    camera_prim: str = (
        "/World/FiveAges_W2/Head_V1/head_link2/head_camera/head_camera"
    )
    base_prim: str = "/World/FiveAges_W2/LinkHou_S2/base_footprint/base_link"
    rgb_topic: str = "/head_camera/rgb"
    depth_topic: str = "/head_camera/depth"
    info_topic: str = "/head_camera/camera_info"
    viser_port: int = 8080
    capture_dir: str = "data/w2_head_capture"
    # HUG runs in its own uv venv (Python 3.10); empty → auto-discover
    hug_python: str = ""
    hug_root: str = ""
    sampling_steps: int = 1
    position_only: bool = True  # 2A default: follow translation, keep EE orientation
    # HUG wrist should coincide with hug_tip_frame; commands are for control_tip_frame
    # (OCS2). Use "{arm}_tcp" / "{arm}_eef" or concrete names. Empty hug_tip = no remap.
    hug_tip_frame: str = "{arm}_tcp"
    control_tip_frame: str = "{arm}_eef"
    # Fixed MANO-wrist → hug_tip axis correction: T_base_tip = T_base_mano @ T_mano_tip
    # ROS rpy deg [roll, pitch, yaw]. Identity often wrong (MANO −X≈fingers, O7 +Z≈palm).
    mano_to_tip_rpy_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Phase 2B: which dex hand retarget backend (wrist path does not use this)
    hand_profile: str = "wuji_hand2"  # wuji_hand2 | linkerhand_o7 | none
    # Optional fixed rotation on wrist-relative keypoints before SDK step (deg).
    keypoint_rpy_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # HUG MANO is right-handed; keep Right RetargetSession even when commanding left arm.
    hug_mano_is_right: bool = True
    # Amplify open→retarget deltas (0=shape only, ~0.45–0.7 typical). Not limit slam.
    squeeze_amount: float = 0.5
    # Extra gain on thumb_cmc_abd when amplifying (deepen opposition along retarget sign).
    thumb_abd_boost: float = 1.25
    # Close-phase executor (B0–B5). See docs/CLOSE_EXECUTORS.md
    close_executor: str = "B4"
    close_steps: int = 40
    close_dt_s: float = 0.04
    contact_hold_frames: int = 2
    min_contact_fingers: int = 2
    # full | keep_tip — keep_tip wraps small objects after proximal contact
    freeze_mode: str = "keep_tip"
    ramp_steps: int = 25
    ramp_amount: float = 0.18
    tip_ramp_boost: float = 0.35
    tip_pinch_steps: int = 12
    tip_pinch_amount: float = 0.25
    wrist_press_m: float = 0.008
    wrist_press_axis: str = "tip_neg_z"  # tip_neg_z | tip_z | world_neg_z
    # Close: recompute retarget → open-loop MANO shape → settle → executor
    auto_retarget_on_close: bool = True
    shape_steps: int = 30
    shape_settle_s: float = 0.5
    # False = visible MANO (recommended). True = old gated shape (table can freeze).
    shape_use_contact_gate: bool = False
    # World +Z lift before shape/close (table clearance for small objects)
    pre_close_lift_m: float = 0.015
    # Ignore these partner substrings when gating (desk / ground)
    ignore_contact_partners: list[str] = field(
        default_factory=lambda: [
            "ground",
            "GroundPlane",
            "defaultGroundPlane",
            "default_environment",
            "table",
            "desk",
            "floor",
            "Plane",
        ]
    )
    # If object_frame set, warn when object moves more than this (m)
    object_push_warn_m: float = 0.015
    # Empty → profile default (wuji_rh / wuji_lh). Override for alternate mounts.
    contact_topic_prefix: str = ""
    use_contact_bool: bool = True
    force_mag_threshold_n: float = 0.0
    always_log_contact: bool = True
    # Optional TF frame for object Δpose logging (empty → skip)
    object_frame: str = ""
    # Empty → <capture_dir>/close_logs
    close_log_dir: str = ""
    arrival_timeout: float = 8.0
    tf_timeout: float = 2.0
    execute_default: bool = False

    def resolved_hug_tip_frame(self) -> str:
        return self.hug_tip_frame.replace("{arm}", self.arm).strip()

    def resolved_control_tip_frame(self) -> str:
        return self.control_tip_frame.replace("{arm}", self.arm).strip()

    def mano_to_tip_T(self):
        from .se3 import T_from_rpy_deg

        rpy = list(self.mano_to_tip_rpy_deg) + [0.0, 0.0, 0.0]
        return T_from_rpy_deg(rpy[0], rpy[1], rpy[2])

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "GraspGenerationConfig":
        raw = dict(data or {})
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        kwargs = {k: v for k, v in raw.items() if k in known}
        return cls(**kwargs)

    def resolved_hug_root(self, workspace: Path) -> Path:
        if self.hug_root:
            return Path(self.hug_root).expanduser().resolve()
        # workspace = examples/IsaacSim → repo root parents[1]
        for parent in [workspace, *workspace.parents]:
            cand = parent / "submodules" / "hug"
            if cand.is_dir():
                return cand.resolve()
        return (workspace / "submodules" / "hug").resolve()

    def resolved_hug_python(self, workspace: Path) -> Path:
        if self.hug_python:
            return Path(self.hug_python).expanduser().absolute()
        root = self.resolved_hug_root(workspace)
        # Do NOT Path.resolve() — that follows the venv symlink to bare cpython
        # and drops site-packages. Keep .../hug/.venv/bin/python as-is.
        return (root / ".venv" / "bin" / "python").absolute()

    def resolved_capture_dir(self, workspace: Path) -> Path:
        p = Path(self.capture_dir)
        if p.is_absolute():
            return p
        hug = self.resolved_hug_root(workspace)
        return (hug / p).resolve()


def _robot_key_from_dir(robot_dir: Path) -> str:
    key = robot_dir.name.lower()
    ry = robot_dir / "robot.yaml"
    if ry.is_file():
        try:
            meta = yaml.safe_load(ry.read_text(encoding="utf-8")) or {}
            if isinstance(meta.get("key"), str) and meta["key"]:
                key = str(meta["key"])
        except Exception:
            pass
    return key


def _robot_label_from_dir(robot_dir: Path) -> str:
    ry = robot_dir / "robot.yaml"
    if ry.is_file():
        try:
            meta = yaml.safe_load(ry.read_text(encoding="utf-8")) or {}
            if isinstance(meta.get("label"), str) and meta["label"]:
                return str(meta["label"])
        except Exception:
            pass
    return robot_dir.name


def discover_grasp_robots(workspace: Path) -> dict[str, dict[str, Any]]:
    """Robots that ship ``grasp_generation.yaml`` under ``workspace/robots/``.

    Independent of ``task_configs/`` / motion-generation discovery so a robot
    can expose grasp assist without a full skill queue.
    """
    robots_root = Path(workspace) / "robots"
    out: dict[str, dict[str, Any]] = {}
    if not robots_root.is_dir():
        return out
    for robot_dir in _iter_robot_dirs(robots_root):
        yml = robot_dir / "grasp_generation.yaml"
        if not yml.is_file():
            continue
        key = _robot_key_from_dir(robot_dir)
        out[key] = {
            "key": key,
            "label": _robot_label_from_dir(robot_dir),
            "robot_dir": robot_dir,
            "config_path": yml.resolve(),
        }
    return out


def find_grasp_config_path(
    workspace: Path,
    robot_key: str,
) -> Optional[Path]:
    """Locate ``grasp_generation.yaml`` for ``robot_key`` (or None)."""
    robots = discover_grasp_robots(workspace)
    entry = robots.get(robot_key)
    if entry is not None:
        return Path(entry["config_path"])
    # Fallback: name / path heuristics (same as load)
    candidates = [
        workspace / "robots" / robot_key / "grasp_generation.yaml",
    ]
    for p in (workspace / "robots").glob(f"*/{robot_key}/grasp_generation.yaml"):
        candidates.append(p)
    for robot_dir in _iter_robot_dirs(workspace / "robots"):
        yml = robot_dir / "grasp_generation.yaml"
        if not yml.is_file():
            continue
        if _robot_key_from_dir(robot_dir) == robot_key or robot_dir.name == robot_key:
            return yml.resolve()
    for c in candidates:
        if c.is_file():
            return c.resolve()
    return None


def load_grasp_config(
    *,
    workspace: Path,
    robot_key: Optional[str] = None,
    config_path: Optional[Path] = None,
    overrides: Optional[dict[str, Any]] = None,
) -> GraspGenerationConfig:
    """Merge package default ← robot ``grasp_generation.yaml`` ← overrides."""
    merged: dict[str, Any] = {}
    default_path = _package_default_yaml()
    if default_path.is_file():
        merged.update(yaml.safe_load(default_path.read_text(encoding="utf-8")) or {})

    path = Path(config_path) if config_path is not None else None
    if path is None and robot_key:
        path = find_grasp_config_path(workspace, robot_key)

    if path is not None and Path(path).is_file():
        merged.update(yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {})

    if robot_key:
        merged.setdefault("robot_key", robot_key)
    if overrides:
        merged.update({k: v for k, v in overrides.items() if v is not None})

    return GraspGenerationConfig.from_mapping(merged)


def _iter_robot_dirs(robots_root: Path):
    if not robots_root.is_dir():
        return
    for p in sorted(robots_root.iterdir()):
        if not p.is_dir() or p.name.startswith("__"):
            continue
        if (p / "robot.yaml").is_file() or (p / "task_configs").is_dir():
            yield p
        for c in sorted(p.iterdir()):
            if c.is_dir() and ((c / "robot.yaml").is_file() or (c / "task_configs").is_dir()):
                yield c
