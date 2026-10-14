"""Executor registry B0–B5."""

from __future__ import annotations

from ..types import ExecutorId
from .base import CloseExecutor
from .modes import ContactGatedExecutor, InstantRetargetExecutor, SlowOpenLoopExecutor

__all__ = [
    "CloseExecutor",
    "get_executor",
    "EXECUTOR_CHOICES",
    "EXECUTOR_LABELS",
    "EXECUTOR_HELP_MD",
]


def get_executor(executor_id: str | ExecutorId) -> CloseExecutor:
    eid = ExecutorId.parse(executor_id)
    if eid == ExecutorId.B0_INSTANT:
        return InstantRetargetExecutor()
    if eid == ExecutorId.B1_SLOW_RETARGET:
        return SlowOpenLoopExecutor(ExecutorId.B1_SLOW_RETARGET)
    if eid == ExecutorId.B2_SLOW_SQUEEZE:
        return SlowOpenLoopExecutor(ExecutorId.B2_SLOW_SQUEEZE)
    if eid == ExecutorId.B3_CONTACT_GATE:
        return ContactGatedExecutor(ExecutorId.B3_CONTACT_GATE)
    if eid == ExecutorId.B4_GATED_RAMP:
        return ContactGatedExecutor(ExecutorId.B4_GATED_RAMP)
    if eid == ExecutorId.B5_GATED_RAMP_WRIST:
        return ContactGatedExecutor(ExecutorId.B5_GATED_RAMP_WRIST)
    raise ValueError(f"no executor for {eid}")


EXECUTOR_CHOICES: tuple[str, ...] = tuple(m.value for m in ExecutorId)

EXECUTOR_LABELS: dict[str, str] = {
    "B0": "B0 shape only (open-loop MANO)",
    "B1": "B1 slow → retarget (ablation)",
    "B2": "B2 slow → squeeze (open-loop)",
    "B3": "B3 contact-gate keep_tip (TacDex-lite)",
    "B4": "B4 gate + ramp + tip_pinch (recommended)",
    "B5": "B5 + wrist press (experimental)",
}

# Shown in Viser Help panel (markdown). Keep concise; full docs stay local.
EXECUTOR_HELP_MD: str = """
### Close executors (B0–B5)

Pipeline after Plan: **lift (optional) → open-loop MANO shape → settle → executor**.

| Id | What it does | When to use |
|----|----------------|-------------|
| **B0** | Shape to HUG MANO only | Check retarget |
| **B1** | Slow open-loop to retarget | Timing ablation |
| **B2** | **Open-loop** slow squeeze to `squeeze(q)` | Baseline force |
| **B3** | Contact-gate `keep_tip` toward amplified goal | Less push-away |
| **B4** | B3 + ramp + tip_pinch（**不是** B2 开环挤压） | **Default** |
| **B5** | B4 + small wrist press | Hover / miss |

`squeeze_amount` = B3–B5 **goal** amplify；B2 才是全程开环挤压。  
`post_shape_squeeze_scale` **仅 B2**。`soft_stop_force_n` 防接触后假张开。  
Contact: `/wuji_rh|lh/in_contact/*`。配置：`robots/<Robot>/grasp_generation.yaml`。

**vs motion-generation:** 勿与 task-queue 同时抢同一接口做 Close。
""".strip()

