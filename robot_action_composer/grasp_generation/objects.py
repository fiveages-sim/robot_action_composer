"""Hardcoded grasp-object catalog helpers (privileged auto-grasp)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence

from .config import GraspGenerationConfig


@dataclass(frozen=True)
class GraspObjectSpec:
    key: str
    pose_prim: str
    placement_prim: str
    home_translate: Optional[tuple[float, float, float]] = None
    home_orient_wxyz: Optional[tuple[float, float, float, float]] = None
    # HUG click point offset in pose_prim local frame (meters).
    # Origin is often at the mesh base — raise Z so the purple marker sits
    # on the graspable surface (same idea as motion-gen object_position_offset).
    click_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _as_vec3(raw: Any, *, label: str) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{label} must be length-3 list, got {raw!r}")
    return float(raw[0]), float(raw[1]), float(raw[2])


def _as_vec4(raw: Any, *, label: str) -> tuple[float, float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        raise ValueError(f"{label} must be length-4 list, got {raw!r}")
    return float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3])


def parse_grasp_object(key: str, entry: Mapping[str, Any]) -> GraspObjectSpec:
    pose = str(entry.get("pose_prim") or "").strip()
    placement = str(entry.get("placement_prim") or pose).strip()
    if not pose:
        raise ValueError(f"grasp_objects[{key!r}] missing pose_prim")
    if not placement:
        raise ValueError(f"grasp_objects[{key!r}] missing placement_prim")
    home_t = entry.get("home_translate")
    home_q = entry.get("home_orient_wxyz")
    # Accept click_offset or legacy alias click_offset_m.
    off_raw = entry.get("click_offset", entry.get("click_offset_m"))
    return GraspObjectSpec(
        key=str(key).strip(),
        pose_prim=pose,
        placement_prim=placement,
        home_translate=_as_vec3(home_t, label=f"grasp_objects[{key}].home_translate")
        if home_t is not None
        else None,
        home_orient_wxyz=_as_vec4(home_q, label=f"grasp_objects[{key}].home_orient_wxyz")
        if home_q is not None
        else None,
        click_offset=_as_vec3(off_raw, label=f"grasp_objects[{key}].click_offset")
        if off_raw is not None
        else (0.0, 0.0, 0.0),
    )


def list_grasp_object_keys(cfg: GraspGenerationConfig) -> list[str]:
    objs = cfg.grasp_objects or {}
    return [str(k) for k in objs.keys() if str(k).strip()]


def get_grasp_object(cfg: GraspGenerationConfig, key: str) -> GraspObjectSpec:
    objs = cfg.grasp_objects or {}
    if key not in objs:
        known = ", ".join(list_grasp_object_keys(cfg)) or "(none)"
        raise KeyError(f"Unknown grasp object {key!r}; known: {known}")
    entry = objs[key]
    if not isinstance(entry, Mapping):
        raise TypeError(f"grasp_objects[{key!r}] must be a mapping")
    return parse_grasp_object(key, entry)


def iter_grasp_objects(cfg: GraspGenerationConfig) -> list[GraspObjectSpec]:
    return [get_grasp_object(cfg, k) for k in list_grasp_object_keys(cfg)]


def default_object_key(cfg: GraspGenerationConfig) -> str:
    keys = list_grasp_object_keys(cfg)
    if not keys:
        return ""
    pref = str(cfg.default_object_key or "").strip()
    if pref and pref in keys:
        return pref
    return keys[0]


def scene_prep_vec2(
    raw: Any,
    default: Sequence[float],
    *,
    label: str,
) -> tuple[float, float]:
    if raw is None:
        return float(default[0]), float(default[1])
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ValueError(f"{label} must be length-2 list, got {raw!r}")
    return float(raw[0]), float(raw[1])
