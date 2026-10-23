"""Subprocess helper: run HUG propose in hug .venv from composer/system Python."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Optional


def run_hug_propose(
    *,
    hug_python: Path,
    capture_dir: Path,
    out_json: Path,
    u_norm: float,
    v_norm: float,
    sampling_steps: int = 1,
    composer_root: Optional[Path] = None,
) -> Path:
    """Call ``hug_worker``; return path to proposal JSON."""
    hug_python = Path(hug_python)
    if not hug_python.is_file():
        from robot_action_composer.grasp_generation.preflight import (
            warn_hug_python_missing,
        )

        raise FileNotFoundError(warn_hug_python_missing(hug_python))
    from robot_action_composer.grasp_generation.preflight import (
        _hug_root_from_python,
        hug_asset_lines,
    )

    hug_root = _hug_root_from_python(hug_python)
    if hug_root is not None:
        for line in hug_asset_lines(hug_root):
            print(f"[grasp] {line}", file=sys.stderr)

    if composer_root is None:
        # .../robot_action_composer/robot_action_composer/grasp_generation/hug_subprocess.py
        composer_root = Path(__file__).resolve().parents[2]

    env = os.environ.copy()
    # Prefer composer on path for the worker; drop VIRTUAL_ENV so hug venv wins for site-packages
    env.pop("VIRTUAL_ENV", None)
    pp = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{composer_root}{os.pathsep}{pp}" if pp else str(composer_root)

    cmd = [
        str(hug_python),
        "-m",
        "robot_action_composer.grasp_generation.hug_worker",
        "--from-dir",
        str(capture_dir),
        "--u-norm",
        str(u_norm),
        "--v-norm",
        str(v_norm),
        "--out",
        str(out_json),
        "--sampling-steps",
        str(sampling_steps),
        "--work-dir",
        str(Path(out_json).parent / "hug_work"),
    ]
    print("[grasp] HUG subprocess:", " ".join(cmd))
    print(f"[grasp] hug_python={hug_python} (host={sys.executable})")
    proc = subprocess.run(cmd, check=False, env=env)
    if proc.returncode != 0:
        raise RuntimeError(f"HUG worker failed with exit {proc.returncode}")
    if not Path(out_json).is_file():
        raise FileNotFoundError(f"HUG worker did not write {out_json}")
    return Path(out_json)
