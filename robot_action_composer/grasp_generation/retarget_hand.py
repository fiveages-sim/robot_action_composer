"""CLI: proposal.json landmarks → Wuji Hand2 q[20] (left/right)."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from .config import GraspGenerationConfig
from .retarget.pipeline import run_retarget_pipeline


def run_retarget_hand(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    proposal_path: Optional[Path] = None,
    execute: bool = False,
    reset: bool = True,
    arm: Optional[str] = None,
    squeeze_amount: Optional[float] = None,
) -> int:
    """Retarget HUG landmarks to Hand2 joints. Default dry-run (print only)."""
    code, out = run_retarget_pipeline(
        cfg=cfg,
        workspace=workspace,
        proposal_path=proposal_path,
        arm=arm,
        execute=execute,
        reset=reset,
        squeeze_amount=squeeze_amount,
    )
    if code != 0:
        print(f"[retarget-hand] failed: {out.get('error', out)}")
        return code

    print(f"[retarget-hand] proposal={out['proposal']}")
    print(
        f"[retarget-hand] arm={out['arm']} model_side={out['model_side']} "
        f"squeeze={out['squeeze_amount']:.2f} absmax={out['q_absmax']:.4f} rad"
    )
    for name, val in out["q_named"].items():
        print(f"  {name:28s} {val:+.4f}")
    if out.get("squeeze_deltas"):
        print("[retarget-hand] squeeze Δflex:")
        for name, d in out["squeeze_deltas"].items():
            print(f"  {name:28s} {d:+.4f}")
    print(f"[retarget-hand] wrote {out['out_path']}")
    if not execute:
        print(
            f"[retarget-hand] dry-run (pass --execute to send_{out['arm']}_hand_joint_positions)"
        )
    else:
        print(f"[retarget-hand] sent → {out['arm']}_hand_controller")
    return 0
