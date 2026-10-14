"""HUG propose worker — must run under ``submodules/hug/.venv`` (Python 3.10).

Invoked by ``grasp-generation`` (system / composer Python) via subprocess:

    hug_python -m robot_action_composer.grasp_generation.hug_worker \\
        --from-dir CAPTURE_DIR --u-norm 0.5 --v-norm 0.42 --out proposal.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--from-dir", type=Path, required=True)
    parser.add_argument("--u-norm", type=float, default=0.5)
    parser.add_argument("--v-norm", type=float, default=0.45)
    parser.add_argument("--u-px", type=float, default=None)
    parser.add_argument("--v-px", type=float, default=None)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--sampling-steps", type=int, default=1)
    parser.add_argument("--work-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    try:
        from robot_action_composer.grasp_generation.hug_adapter import HugAdapter
        from robot_action_composer.grasp_generation.rgbd_io import load_frame_from_dir
    except ImportError as exc:
        print(
            "FATAL: run this module with HUG uv python and composer on PYTHONPATH.\n"
            f"  python={sys.executable}\n  err={exc}",
            file=sys.stderr,
        )
        return 2

    adapter = HugAdapter(sampling_steps=args.sampling_steps)
    if not adapter.available:
        print("HUG not available:", adapter.describe(), file=sys.stderr)
        return 2

    frame = load_frame_from_dir(args.from_dir)
    work = args.work_dir or (args.out.parent / "hug_work")
    prop = adapter.propose(
        frame,
        u_norm=None if args.u_px is not None else args.u_norm,
        v_norm=None if args.v_px is not None else args.v_norm,
        u_px=args.u_px,
        v_px=args.v_px,
        work_dir=work,
    )
    out = {
        "T_camera_wrist": prop.T_camera_wrist.tolist(),
        "landmarks_3d": prop.landmarks_3d.tolist(),
        "landmarks_2d": prop.landmarks_2d.tolist(),
        "click_uv_224": list(prop.click_uv_224),
        "depth_at_click_m": prop.depth_at_click_m,
        "camera_K_224": prop.camera_K_224.tolist(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"OK wrote {args.out}")
    print(
        "wrist_t",
        prop.T_camera_wrist[:3, 3].tolist(),
        "depth_m",
        prop.depth_at_click_m,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
