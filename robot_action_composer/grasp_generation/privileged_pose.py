"""Newton-safe object pose for privileged auto-grasp.

Prefer ``/get_entity_state`` (PhysX). On failure (common under Newton kit launch),
compose world pose from ``/get_prim_attribute`` local xform chain — same Isaac
service pattern as motion-generation ``env.randomize_object_local_xyz``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from .se3 import T_from_xyz_quat_xyzw, T_inv, T_mul, T_to_xyz_quat_xyzw


@dataclass
class ObjectPoseResult:
    xyz_world: tuple[float, float, float]
    quat_xyzw_world: tuple[float, float, float, float]
    xyz_base: tuple[float, float, float]
    method: str  # entity_state | prim_attr_chain
    pose_prim: str


def _normalize_prim_path(path: str) -> str:
    s = str(path or "").strip().rstrip("/")
    if not s:
        return ""
    if not s.startswith("/"):
        s = "/" + s
    while "//" in s:
        s = s.replace("//", "/")
    return s


def _try_prim_scale(path: str) -> tuple[float, float, float]:
    """Product of ``xformOp:scale`` and ``xformOp:scale:unitsResolve`` if both exist."""
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        try_get_prim_attribute,
    )

    sx = sy = sz = 1.0
    for attr in ("xformOp:scale", "xformOp:scale:unitsResolve"):
        raw = try_get_prim_attribute(path, attr)
        if raw is None:
            continue
        vec = np.fromstring(str(raw).strip().strip("[]"), sep=",")
        if vec.shape[0] == 3:
            sx *= float(vec[0])
            sy *= float(vec[1])
            sz *= float(vec[2])
    return sx, sy, sz


def _T_local_from_prim(path: str) -> np.ndarray:
    """Local TRS at ``path`` (missing ops → identity). Scale included."""
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        try_get_prim_orient_local_xyzw,
        try_get_prim_translate_local,
    )

    t = try_get_prim_translate_local(path) or (0.0, 0.0, 0.0)
    q = try_get_prim_orient_local_xyzw(path) or (0.0, 0.0, 0.0, 1.0)
    sx, sy, sz = _try_prim_scale(path)
    T = T_from_xyz_quat_xyzw(t, q)
    # Apply non-uniform scale in local frame: T @ S
    S = np.eye(4, dtype=np.float64)
    S[0, 0], S[1, 1], S[2, 2] = sx, sy, sz
    return T @ S


def compose_world_T_from_prim_attrs(prim_path: str) -> np.ndarray:
    """Compose ``T_world_prim`` by walking ``/World`` → ``prim`` local xforms."""
    prim = _normalize_prim_path(prim_path)
    if not prim.startswith("/World"):
        # Still walk from root segments
        segments = [s for s in prim.strip("/").split("/") if s]
        acc = np.eye(4, dtype=np.float64)
        cur = ""
        for seg in segments:
            cur = f"{cur}/{seg}"
            acc = T_mul(acc, _T_local_from_prim(cur))
        return acc

    segments = [s for s in prim[len("/World") :].split("/") if s]
    acc = _T_local_from_prim("/World")
    cur = "/World"
    for seg in segments:
        cur = f"{cur}/{seg}"
        acc = T_mul(acc, _T_local_from_prim(cur))
    return acc


def _entity_pose_world(prim: str) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    from robot_action_composer.isaac_sim import get_entity_pose_world_service

    return get_entity_pose_world_service(prim)


def resolve_object_pose_world(
    pose_prim: str,
    *,
    prefer: str = "entity_state",
    retries: int = 3,
    retry_delay_s: float = 0.15,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float], str]:
    """Return ``(xyz, quat_xyzw, method)`` in world.

    Prefers live ``/get_entity_state`` (PhysX). Retries briefly after STOP/PLAY
    settle. Falls back to prim-attr chain; logs a warning if the two disagree.
    """
    import time

    prim = _normalize_prim_path(pose_prim)
    last_err: Exception | None = None
    entity_xyz: tuple[float, float, float] | None = None
    entity_q: tuple[float, float, float, float] | None = None

    if prefer in ("entity_state", "auto"):
        for attempt in range(max(1, int(retries))):
            try:
                entity_xyz, entity_q = _entity_pose_world(prim)
                break
            except Exception as exc:
                last_err = exc
                if attempt + 1 < max(1, int(retries)):
                    time.sleep(float(retry_delay_s))

    chain_xyz: tuple[float, float, float] | None = None
    chain_q: tuple[float, float, float, float] | None = None
    chain_err: Exception | None = None
    try:
        T = compose_world_T_from_prim_attrs(prim)
        chain_xyz, chain_q = T_to_xyz_quat_xyzw(T)
    except Exception as exc:
        chain_err = exc

    if entity_xyz is not None and entity_q is not None:
        if chain_xyz is not None:
            drift = float(
                np.linalg.norm(np.asarray(entity_xyz) - np.asarray(chain_xyz))
            )
            if drift > 0.05:
                print(
                    f"[pose] WARN {prim}: entity vs prim_attr drift={drift:.3f}m "
                    f"— using entity_state"
                )
        return entity_xyz, entity_q, "entity_state"

    if chain_xyz is not None and chain_q is not None:
        if last_err is not None:
            print(
                f"[pose] WARN {prim}: entity_state failed ({last_err}); "
                f"using prim_attr_chain"
            )
        return chain_xyz, chain_q, "prim_attr_chain"

    raise RuntimeError(
        f"Could not resolve world pose for {prim!r}. "
        f"entity_error={last_err!r} prim_attr_error={chain_err!r}"
    )


def resolve_object_pose_in_base(
    pose_prim: str,
    *,
    base_prim: str,
    interface: Optional[Any] = None,
    base_frame: str = "base_link",
    tf_timeout: float = 2.0,
) -> ObjectPoseResult:
    """Object pose in ``base_link`` (or ``base_prim`` frame).

    Base world pose: TF ``map/world → base`` is not required; we use
    ``/get_entity_state(base_prim)`` then fall back to prim-attr chain.
    """
    obj_xyz_w, obj_q_w, method = resolve_object_pose_world(pose_prim)
    base_xyz_w: tuple[float, float, float]
    base_q_w: tuple[float, float, float, float]
    try:
        base_xyz_w, base_q_w = _entity_pose_world(base_prim)
    except Exception:
        T_w_base = compose_world_T_from_prim_attrs(base_prim)
        base_xyz_w, base_q_w = T_to_xyz_quat_xyzw(T_w_base)

    # Optional TF refinement for base if available (same frame as motion).
    if interface is not None:
        try:
            # Prefer live TF origin of base in a fixed world if published — skip if absent.
            # We only use entity/prim for base to stay Newton-safe.
            _ = base_frame  # reserved; TF world↔base often unavailable
            _ = tf_timeout
        except Exception:
            pass

    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        get_object_pose_from_service,
    )

    # Reuse motion-generation helper: world obj − world base → base-relative Pose.
    # get_object_pose_from_service always calls entity_state for object; so only use
    # it when method was entity_state. Otherwise compose manually.
    if method == "entity_state":
        try:
            pose = get_object_pose_from_service(
                base_xyz_w,
                base_q_w,
                pose_prim,
                include_orientation=True,
            )
            return ObjectPoseResult(
                xyz_world=obj_xyz_w,
                quat_xyzw_world=obj_q_w,
                xyz_base=(
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ),
                method=method,
                pose_prim=_normalize_prim_path(pose_prim),
            )
        except Exception:
            pass

    T_w_base = T_from_xyz_quat_xyzw(base_xyz_w, base_q_w)
    T_w_obj = T_from_xyz_quat_xyzw(obj_xyz_w, obj_q_w)
    T_base_obj = T_mul(T_inv(T_w_base), T_w_obj)
    xyz_b, _q_b = T_to_xyz_quat_xyzw(T_base_obj)
    return ObjectPoseResult(
        xyz_world=obj_xyz_w,
        quat_xyzw_world=obj_q_w,
        xyz_base=xyz_b,
        method=method,
        pose_prim=_normalize_prim_path(pose_prim),
    )


def transform_base_point_to_cam(
    xyz_base: Sequence[float],
    T_base_cam: np.ndarray,
) -> np.ndarray:
    """``p_cam = inv(T_base_cam) @ p_base``."""
    T_cam_base = T_inv(np.asarray(T_base_cam, dtype=np.float64))
    p = np.array(
        [float(xyz_base[0]), float(xyz_base[1]), float(xyz_base[2]), 1.0],
        dtype=np.float64,
    )
    return (T_cam_base @ p)[:3]


def apply_local_offset_world(
    xyz_world: Sequence[float],
    quat_xyzw_world: Sequence[float],
    offset_local_m: Sequence[float],
) -> tuple[float, float, float]:
    """``p_world = R(q) @ offset_local + t`` (offset in pose_prim local meters)."""
    T = T_from_xyz_quat_xyzw(xyz_world, quat_xyzw_world)
    p = np.array(
        [
            float(offset_local_m[0]),
            float(offset_local_m[1]),
            float(offset_local_m[2]),
            1.0,
        ],
        dtype=np.float64,
    )
    out = T @ p
    return float(out[0]), float(out[1]), float(out[2])


def world_point_to_base(
    xyz_world: Sequence[float],
    *,
    base_prim: str,
) -> tuple[float, float, float]:
    """Map a world point into ``base_prim`` frame (entity_state / prim chain)."""
    try:
        base_xyz_w, base_q_w = _entity_pose_world(base_prim)
    except Exception:
        T_w_base = compose_world_T_from_prim_attrs(base_prim)
        base_xyz_w, base_q_w = T_to_xyz_quat_xyzw(T_w_base)
    T_w_base = T_from_xyz_quat_xyzw(base_xyz_w, base_q_w)
    T_base_w = T_inv(T_w_base)
    p = np.array(
        [float(xyz_world[0]), float(xyz_world[1]), float(xyz_world[2]), 1.0],
        dtype=np.float64,
    )
    out = T_base_w @ p
    return float(out[0]), float(out[1]), float(out[2])
