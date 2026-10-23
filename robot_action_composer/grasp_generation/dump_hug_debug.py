"""Save HUG debug overlays: RGB + purple UV click + green MANO skeleton.

Standalone CLI and a helper used by ``run_auto_grasp`` after propose so you can
inspect misaligned grasps without opening Viser.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Optional, Sequence

import cv2
import numpy as np

from .rgbd_io import crop_frame_to_legacy_vfov, load_frame_from_dir
from .scene_vis import (
    CLICK_MARKER_COLOR,
    HAND_KEYPOINT_COLOR,
    HAND_SKELETON_COLOR,
    MANO_SKELETON_PAIRS,
)


def center_crop_square(rgb: np.ndarray) -> tuple[np.ndarray, int, int, int]:
    """Match HUG prepare: center square crop. Returns (crop, size, x_off, y_off)."""
    h, w = rgb.shape[:2]
    size = min(h, w)
    x_off = (w - size) // 2
    y_off = (h - size) // 2
    return rgb[y_off : y_off + size, x_off : x_off + size].copy(), size, x_off, y_off


def project_landmarks_uv224(
    landmarks_3d: np.ndarray,
    K224: np.ndarray,
) -> np.ndarray:
    """Camera-frame landmarks (N,3) → 224 pixel UV (N,2)."""
    lm = np.asarray(landmarks_3d, dtype=np.float64).reshape(-1, 3)
    K = np.asarray(K224, dtype=np.float64).reshape(3, 3)
    z = np.clip(lm[:, 2], 1e-4, None)
    u = K[0, 0] * lm[:, 0] / z + K[0, 2]
    v = K[1, 1] * lm[:, 1] / z + K[1, 2]
    return np.stack([u, v], axis=-1)


def draw_hug_debug_overlay(
    rgb_full: np.ndarray,
    *,
    click_uv_224: Sequence[float] | None,
    landmarks_uv_224: np.ndarray | None,
    out_size: int = 224,
) -> np.ndarray:
    """Center-crop → resize to ``out_size``; purple click + green MANO."""
    crop, _size, _xo, _yo = center_crop_square(rgb_full)
    canvas = cv2.resize(crop, (out_size, out_size), interpolation=cv2.INTER_LINEAR)

    def _to_px(u224: float, v224: float) -> tuple[int, int]:
        # click_uv / landmarks are in HUG 224 space; scale if out_size != 224.
        x = int(round(float(u224) * (out_size / 224.0)))
        y = int(round(float(v224) * (out_size / 224.0)))
        return x, y

    if landmarks_uv_224 is not None and len(landmarks_uv_224) > 0:
        uv = np.asarray(landmarks_uv_224, dtype=np.float64).reshape(-1, 2)
        for a, b in MANO_SKELETON_PAIRS:
            if a >= len(uv) or b >= len(uv):
                continue
            p0 = _to_px(uv[a, 0], uv[a, 1])
            p1 = _to_px(uv[b, 0], uv[b, 1])
            cv2.line(canvas, p0, p1, HAND_SKELETON_COLOR, 2, cv2.LINE_AA)
        for i in range(len(uv)):
            p = _to_px(uv[i, 0], uv[i, 1])
            cv2.circle(canvas, p, 3, HAND_KEYPOINT_COLOR, -1, cv2.LINE_AA)

    if click_uv_224 is not None and len(click_uv_224) >= 2:
        cx, cy = _to_px(float(click_uv_224[0]), float(click_uv_224[1]))
        cv2.drawMarker(
            canvas,
            (cx, cy),
            CLICK_MARKER_COLOR,
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        cv2.circle(canvas, (cx, cy), 8, CLICK_MARKER_COLOR, 2, cv2.LINE_AA)

    return canvas


def dump_hug_debug_from_capture(
    capture_dir: Path,
    *,
    out_dir: Optional[Path] = None,
    proposal_path: Optional[Path] = None,
) -> dict[str, Any]:
    """Write debug PNGs under ``out_dir`` (default ``capture_dir/hug_debug``).

    Expects ``w2_head_rgb.png`` (or any *rgb*) + ``proposal.json`` with
    ``click_uv_224``, ``landmarks_3d``, ``camera_K_224``.
    """
    capture_dir = Path(capture_dir)
    prop_path = Path(proposal_path) if proposal_path else capture_dir / "proposal.json"
    if not prop_path.is_file():
        raise FileNotFoundError(f"missing proposal: {prop_path}")

    frame = load_frame_from_dir(capture_dir)
    rgb_c, _, _, _, _ = crop_frame_to_legacy_vfov(frame.rgb, frame.depth_m, frame.K)
    frame = type(frame)(
        rgb=rgb_c,
        depth_m=frame.depth_m,
        K=frame.K,
        stamp_sec=frame.stamp_sec,
    )
    data = json.loads(prop_path.read_text(encoding="utf-8"))
    click = data.get("click_uv_224")
    K224 = np.asarray(data.get("camera_K_224"), dtype=np.float64)
    lm3 = data.get("landmarks_3d")
    lm2 = data.get("landmarks_2d")

    if lm2 is not None:
        uv = np.asarray(lm2, dtype=np.float64).reshape(-1, 2)
    elif lm3 is not None and K224.shape == (3, 3):
        uv = project_landmarks_uv224(lm3, K224)
    else:
        uv = None

    overlay = draw_hug_debug_overlay(
        frame.rgb,
        click_uv_224=click,
        landmarks_uv_224=uv,
    )

    dest = Path(out_dir) if out_dir is not None else capture_dir / "hug_debug"
    dest.mkdir(parents=True, exist_ok=True)

    rgb_path = dest / "rgb_224.png"
    overlay_path = dest / "overlay_uv_mano.png"
    crop, *_ = center_crop_square(frame.rgb)
    crop224 = cv2.resize(crop, (224, 224), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite(str(rgb_path), cv2.cvtColor(crop224, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(overlay_path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

    # Also keep a copy next to capture for quick glance during auto-grasp.
    quick = capture_dir / "hug_debug_overlay.png"
    cv2.imwrite(str(quick), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

    meta = {
        "capture_dir": str(capture_dir),
        "proposal": str(prop_path),
        "out_dir": str(dest),
        "rgb_224": str(rgb_path),
        "overlay": str(overlay_path),
        "quick_overlay": str(quick),
        "click_uv_224": list(click) if click is not None else None,
        "n_landmarks": int(uv.shape[0]) if uv is not None else 0,
    }
    (dest / "meta.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )
    print(f"[hug_debug] wrote {overlay_path}")
    return meta


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Dump HUG RGB + purple UV + green MANO overlay from a capture dir"
    )
    parser.add_argument(
        "--from-dir",
        type=Path,
        required=True,
        help="Capture folder with rgb + proposal.json",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output folder (default: <from-dir>/hug_debug)",
    )
    parser.add_argument("--proposal", type=Path, default=None)
    args = parser.parse_args(argv)
    meta = dump_hug_debug_from_capture(
        args.from_dir, out_dir=args.out_dir, proposal_path=args.proposal
    )
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
