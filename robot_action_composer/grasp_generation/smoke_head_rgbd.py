"""Smoke: W2 head RGB-D → HUG MANO grasp (camera frame).

Run inside the HUG uv venv (Python 3.10), with composer on PYTHONPATH:

    cd submodules/hug && source .venv/bin/activate
    export PYTHONPATH=../robot_action_composer:${PYTHONPATH}
    python -m robot_action_composer.grasp_generation.smoke_head_rgbd --help
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--from-ros",
        action="store_true",
        help="Capture one frame from /head_camera/{rgb,depth,camera_info}",
    )
    src.add_argument(
        "--from-dir",
        type=Path,
        help="Folder with *rgb* / *depth* / *intrinsics* files",
    )
    parser.add_argument("--u-norm", type=float, default=0.5, help="Click u on full image [0,1]")
    parser.add_argument("--v-norm", type=float, default=0.45, help="Click v on full image [0,1]")
    parser.add_argument("--u-px", type=float, default=None)
    parser.add_argument("--v-px", type=float, default=None)
    parser.add_argument("--save-dir", type=Path, default=None, help="Dump capture + proposal JSON")
    parser.add_argument("--sampling-steps", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)

    from robot_action_composer.grasp_generation.hug_adapter import HugAdapter
    from robot_action_composer.grasp_generation.rgbd_io import (
        capture_head_camera_ros,
        load_frame_from_dir,
        save_frame_to_dir,
    )

    adapter = HugAdapter(sampling_steps=args.sampling_steps)
    print("HUG readiness:", json.dumps(adapter.describe(), indent=2))
    if not adapter.available:
        print("HUG not available; abort.", file=sys.stderr)
        return 2

    if args.from_ros:
        # HUG venv = py3.10; ROS Jazzy rclpy = py3.12 → auto subprocess capture.
        save_dir = args.save_dir
        if save_dir is None:
            from tempfile import mkdtemp

            save_dir = Path(mkdtemp(prefix="w2_head_capture_"))
            print(f"(no --save-dir; using temp {save_dir})")
        print("Waiting for /head_camera RGB-D ...")
        frame = capture_head_camera_ros(
            timeout_sec=args.timeout,
            out_dir=save_dir,
        )
        args.save_dir = Path(save_dir)
    else:
        frame = load_frame_from_dir(args.from_dir)

    valid = frame.depth_m[frame.depth_m > 0]
    dmin = float(valid.min()) if valid.size else 0.0
    print(
        f"frame rgb={frame.rgb.shape} depth=[{dmin:.3f},{frame.depth_m.max():.3f}]m  "
        f"fx={frame.K[0,0]:.1f} fy={frame.K[1,1]:.1f}"
    )

    if args.save_dir is not None and not args.from_ros:
        save_frame_to_dir(frame, args.save_dir, stem="w2_head")
    elif args.from_ros and args.save_dir is not None:
        # Subprocess already wrote png/txt; refresh if in-process capture was used.
        rgb_png = Path(args.save_dir) / "w2_head_rgb.png"
        if not rgb_png.is_file():
            save_frame_to_dir(frame, args.save_dir, stem="w2_head")

    prop = adapter.propose(
        frame,
        u_norm=None if args.u_px is not None else args.u_norm,
        v_norm=None if args.v_px is not None else args.v_norm,
        u_px=args.u_px,
        v_px=args.v_px,
        work_dir=args.save_dir / "hug_work" if args.save_dir else None,
    )

    t = prop.T_camera_wrist[:3, 3]
    print("click_uv_224", prop.click_uv_224, "depth_m", f"{prop.depth_at_click_m:.3f}")
    print("T_camera_wrist translation (m):", t.tolist())
    print("landmarks_3d wrist:", prop.landmarks_3d[0].tolist())

    if args.save_dir is not None:
        out = {
            "T_camera_wrist": prop.T_camera_wrist.tolist(),
            "landmarks_3d": prop.landmarks_3d.tolist(),
            "click_uv_224": list(prop.click_uv_224),
            "depth_at_click_m": prop.depth_at_click_m,
            "camera_K_224": prop.camera_K_224.tolist(),
        }
        path = Path(args.save_dir) / "proposal.json"
        path.write_text(json.dumps(out, indent=2))
        print("wrote", path)

    if not np.isfinite(t).all() or float(np.linalg.norm(t)) < 1e-4:
        print("WARN: wrist translation looks invalid", file=sys.stderr)
        return 1
    print("OK: HUG propose succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
