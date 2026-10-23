"""Viser 3D helpers for grasp-generation serve (HUG-app aligned, no hug import)."""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

# MANO 21-joint skeleton (same as hug.utils.visualization_utils)
MANO_SKELETON_PAIRS = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 4],
    [0, 5],
    [5, 6],
    [6, 7],
    [7, 8],
    [9, 10],
    [10, 11],
    [11, 12],
    [13, 14],
    [14, 15],
    [15, 16],
    [0, 17],
    [17, 18],
    [18, 19],
    [19, 20],
    [5, 9],
    [9, 13],
    [13, 17],
]

HAND_SKELETON_COLOR = (50, 255, 100)
HAND_KEYPOINT_COLOR = (0, 200, 50)
CLICK_MARKER_COLOR = (255, 0, 255)


def backproject_rgbd(
    depth_m: np.ndarray,
    rgb: np.ndarray,
    K: np.ndarray,
    *,
    max_depth: float = 5.0,
    max_points: int = 180_000,
) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """Depth (meters) + RGB + K → (N,3) points and (N,3) uint8 colors in camera frame."""
    depth = np.asarray(depth_m, dtype=np.float32)
    rgb_a = np.asarray(rgb)
    if depth.ndim != 2:
        raise ValueError(f"depth must be HxW, got {depth.shape}")
    h, w = depth.shape
    if rgb_a.shape[0] != h or rgb_a.shape[1] != w:
        import cv2

        rgb_a = cv2.resize(rgb_a, (w, h), interpolation=cv2.INTER_LINEAR)

    from .rgbd_io import suppress_depth_spikes

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    z = suppress_depth_spikes(depth)
    valid = np.isfinite(z) & (z > 0.05) & (z < float(max_depth))
    if not np.any(valid):
        return None
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    cols = rgb_a.reshape(-1, 3)
    mask = valid.reshape(-1)
    pts, cols = pts[mask], cols[mask]
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_points, replace=False)
        pts, cols = pts[idx], cols[idx]
    return pts.astype(np.float32), cols.astype(np.uint8)


def skeleton_segments(landmarks: np.ndarray) -> np.ndarray:
    lm = np.asarray(landmarks, dtype=np.float32)
    return np.stack(
        [np.stack([lm[a], lm[b]], axis=0) for a, b in MANO_SKELETON_PAIRS],
        axis=0,
    )


def pixel_to_cam_xyz(
    u_px: float,
    v_px: float,
    depth_m: float,
    K: np.ndarray,
) -> np.ndarray:
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    z = float(depth_m)
    return np.array([(u_px - cx) * z / fx, (v_px - cy) * z / fy, z], dtype=np.float32)


class GraspScene3D:
    """Left-canvas scene: RGB-D cloud + MANO skeleton dry-run (camera frame)."""

    def __init__(self, server: Any) -> None:
        self.server = server
        self._pc = None
        self._hand_skel = None
        self._hand_kp = None
        self._wrist = None
        self._click = None
        self._cam_frame = None

        try:
            import viser.transforms as vtf  # noqa: F401

            self._vtf = vtf
        except Exception:
            self._vtf = None

        server.scene.set_up_direction("-y")

        @server.on_client_connect
        def _on_connect(client: Any) -> None:
            client.camera.up_direction = (0.0, -1.0, 0.0)
            client.camera.position = (0.0, 0.0, -0.15)
            client.camera.look_at = (0.0, 0.0, 1.0)

        self._cam_frame = server.scene.add_frame(
            "/scene/camera_frame",
            wxyz=(1.0, 0.0, 0.0, 0.0),
            position=(0.0, 0.0, 0.0),
            axes_length=0.08,
            axes_radius=0.003,
        )

    def clear_hand(self) -> None:
        for attr in ("_hand_skel", "_hand_kp", "_wrist", "_click"):
            h = getattr(self, attr)
            if h is not None:
                try:
                    h.remove()
                except Exception:
                    pass
                setattr(self, attr, None)

    def set_point_cloud(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        *,
        point_size: float = 0.004,
    ) -> None:
        if self._pc is not None:
            try:
                self._pc.remove()
            except Exception:
                pass
            self._pc = None
        self._pc = self.server.scene.add_point_cloud(
            "/depth_points",
            points=np.asarray(points, dtype=np.float32),
            colors=np.asarray(colors, dtype=np.uint8),
            point_size=float(point_size),
            point_shape="rounded",
        )

    def set_click_xyz(self, xyz: np.ndarray, *, radius: float = 0.012) -> None:
        if self._click is not None:
            try:
                self._click.remove()
            except Exception:
                pass
        self._click = self.server.scene.add_icosphere(
            "/predictions/click",
            radius=float(radius),
            color=CLICK_MARKER_COLOR,
            position=np.asarray(xyz, dtype=np.float32),
        )

    def set_hand_from_landmarks(
        self,
        landmarks_3d: np.ndarray,
        T_camera_wrist: Optional[np.ndarray] = None,
        *,
        show_wrist: bool = True,
    ) -> None:
        lm = np.asarray(landmarks_3d, dtype=np.float32).reshape(-1, 3)
        if lm.shape[0] < 21:
            raise ValueError(f"need 21 landmarks, got {lm.shape}")

        if self._hand_skel is not None:
            try:
                self._hand_skel.remove()
            except Exception:
                pass
        if self._hand_kp is not None:
            try:
                self._hand_kp.remove()
            except Exception:
                pass

        segs = skeleton_segments(lm)
        n = len(segs)
        self._hand_skel = self.server.scene.add_line_segments(
            "/predictions/hand/skeleton",
            points=segs,
            colors=np.tile(HAND_SKELETON_COLOR, (n, 2, 1)).astype(np.uint8),
            line_width=4.0,
        )
        self._hand_kp = self.server.scene.add_point_cloud(
            "/predictions/hand/keypoints",
            points=lm,
            colors=np.tile(HAND_KEYPOINT_COLOR, (lm.shape[0], 1)).astype(np.uint8),
            point_size=0.006,
            point_shape="rounded",
        )

        if show_wrist and T_camera_wrist is not None:
            T = np.asarray(T_camera_wrist, dtype=np.float64)
            if self._wrist is not None:
                try:
                    self._wrist.remove()
                except Exception:
                    pass
            wxyz = (1.0, 0.0, 0.0, 0.0)
            if self._vtf is not None:
                wxyz = tuple(self._vtf.SO3.from_matrix(T[:3, :3]).wxyz)
            self._wrist = self.server.scene.add_frame(
                "/predictions/hand/wrist",
                wxyz=wxyz,
                position=T[:3, 3],
                axes_length=0.1,
                axes_radius=0.004,
            )

    def set_point_size(self, point_size: float) -> None:
        if self._pc is not None:
            self._pc.point_size = float(point_size)
