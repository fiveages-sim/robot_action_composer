"""Project privileged 3D points to head-camera UV for HUG propose.

Privileged pose is typically the **rigid-body center** (inside the object).
The depth image sees the **front surface**. Matching refine to ``z_cam``
(center) pulls UV toward edges / table / gaps — HUG then gets a bad click.

Correct behavior (same as a human click): keep the projected pixel (or nearest
on-object surface) and use **surface depth** at that pixel.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class UvProjection:
    u_norm: float
    v_norm: float
    u_px: float
    v_px: float
    depth_m: float
    z_cam_privileged: float
    refined: bool
    surface_delta_m: float = 0.0  # depth_m - z_cam (usually negative)


def project_cam_xyz_to_uv(
    xyz_cam: Sequence[float],
    K: np.ndarray,
    *,
    width: int,
    height: int,
) -> tuple[float, float, float, float]:
    """Return ``(u_norm, v_norm, u_px, v_px)`` from camera-frame XYZ."""
    x, y, z = float(xyz_cam[0]), float(xyz_cam[1]), float(xyz_cam[2])
    if z <= 1e-4:
        raise ValueError(f"Point behind/at camera (z={z:.4f}); cannot project")
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    u_px = fx * x / z + cx
    v_px = fy * y / z + cy
    if not (0.0 <= u_px < width and 0.0 <= v_px < height):
        raise ValueError(
            f"Projected UV ({u_px:.1f},{v_px:.1f}) outside image {width}x{height}. "
            "Object may be off-camera after scene_prep — reset scene / tighten near_xy_*."
        )
    u_norm = float(u_px / max(width - 1, 1))
    v_norm = float(v_px / max(height - 1, 1))
    return u_norm, v_norm, float(u_px), float(v_px)


def _valid_depth(d: float) -> bool:
    return bool(np.isfinite(d) and 0.05 < d < 8.0)


def refine_uv_to_surface(
    depth_m: np.ndarray,
    *,
    u_px: float,
    v_px: float,
    z_cam: float,
    search_radius_px: int = 18,
    # Surface should be in front of / near the center, not far behind it.
    max_in_front_m: float = 0.20,
    max_behind_center_m: float = 0.03,
) -> tuple[float, float, float, bool]:
    """Snap to front-surface depth near the projected center UV.

    Does **not** search for depth ≈ ``z_cam`` (that was the offset bug).
    Prefers the seed pixel if its depth looks like an object surface.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"depth must be HxW, got {depth.shape}")
    h, w = depth.shape
    ui0 = int(round(u_px))
    vi0 = int(round(v_px))
    if not (0 <= ui0 < w and 0 <= vi0 < h):
        raise ValueError(f"seed pixel ({ui0},{vi0}) out of bounds {w}x{h}")

    zc = float(z_cam)
    d_lo = zc - float(max_in_front_m)
    d_hi = zc + float(max_behind_center_m)

    def _in_surface_band(d: float) -> bool:
        return _valid_depth(d) and d_lo <= d <= d_hi

    # 1) Seed pixel — same as a human click on the silhouette center.
    d0 = float(depth[vi0, ui0])
    if _in_surface_band(d0):
        return float(ui0), float(vi0), d0, False

    # 2) Neighborhood: on-object surface band, prefer near seed & closer to camera.
    r = max(0, int(search_radius_px))
    best: Optional[tuple[float, float, int, int, float]] = None
    # score = (pixel_dist^2, depth)  — smaller depth = front surface
    for dv in range(-r, r + 1):
        for du in range(-r, r + 1):
            ui, vi = ui0 + du, vi0 + dv
            if not (0 <= ui < w and 0 <= vi < h):
                continue
            d = float(depth[vi, ui])
            if not _in_surface_band(d):
                continue
            pix2 = float(du * du + dv * dv)
            cand = (pix2, d, ui, vi, d)
            if best is None or cand[0] < best[0] or (
                cand[0] == best[0] and cand[1] < best[1]
            ):
                best = cand

    if best is not None:
        _pix2, _dkey, ui, vi, d = best
        return float(ui), float(vi), d, True

    # 3) Last resort: nearest any valid depth (may be table — caller should warn).
    best2: Optional[tuple[float, int, int, float]] = None
    for dv in range(-r, r + 1):
        for du in range(-r, r + 1):
            ui, vi = ui0 + du, vi0 + dv
            if not (0 <= ui < w and 0 <= vi < h):
                continue
            d = float(depth[vi, ui])
            if not _valid_depth(d):
                continue
            dist = float(du * du + dv * dv)
            if best2 is None or dist < best2[0]:
                best2 = (dist, ui, vi, d)
    if best2 is None:
        raise RuntimeError(
            f"No valid depth near projected pixel ({ui0},{vi0}) "
            f"within r={r}px (privileged z={zc:.3f}m)"
        )
    _dist, ui, vi, d = best2
    print(
        f"[uv] WARN surface band empty at ({ui0},{vi0}); "
        f"using nearest depth={d:.3f}m (z_center={zc:.3f}m, Δ={d - zc:+.3f}m)"
    )
    return float(ui), float(vi), d, True


# Back-compat alias (old name implied z_cam matching — behavior is now surface snap).
def refine_uv_with_depth(
    depth_m: np.ndarray,
    *,
    u_px: float,
    v_px: float,
    z_cam: float,
    search_radius_px: int = 18,
    max_depth_delta_m: float = 0.12,
) -> tuple[float, float, float, bool]:
    _ = max_depth_delta_m
    return refine_uv_to_surface(
        depth_m,
        u_px=u_px,
        v_px=v_px,
        z_cam=z_cam,
        search_radius_px=search_radius_px,
        max_in_front_m=0.20,
        max_behind_center_m=0.03,
    )


def project_privileged_to_uv(
    xyz_cam: Sequence[float],
    *,
    K: np.ndarray,
    depth_m: np.ndarray,
    search_radius_px: int = 18,
) -> UvProjection:
    """Full-image UV for HUG: project center, snap to **surface** depth."""
    depth = np.asarray(depth_m, dtype=np.float32)
    h, w = depth.shape[:2]
    z_cam = float(xyz_cam[2])
    _u_n, _v_n, u_px, v_px = project_cam_xyz_to_uv(xyz_cam, K, width=w, height=h)
    u2, v2, d, refined = refine_uv_to_surface(
        depth,
        u_px=u_px,
        v_px=v_px,
        z_cam=z_cam,
        search_radius_px=search_radius_px,
    )
    delta = float(d - z_cam)
    if delta > 0.05:
        print(
            f"[uv] WARN surface deeper than center (Δ={delta:+.3f}m) — "
            "click may be off-object / table"
        )
    return UvProjection(
        u_norm=float(u2 / max(w - 1, 1)),
        v_norm=float(v2 / max(h - 1, 1)),
        u_px=u2,
        v_px=v2,
        depth_m=d,
        z_cam_privileged=z_cam,
        refined=refined,
        surface_delta_m=delta,
    )
