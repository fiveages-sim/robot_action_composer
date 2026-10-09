"""Shared types for close-phase executors (hand-agnostic API)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence


class ExecutorId(str, Enum):
    """Ablation modes B0–B5 (thesis open-loop / contact-gated close).

    See ``docs/CLOSE_EXECUTORS.md`` and Desktop 开题方向 §B0–B5.
    """

    B0_INSTANT = "B0"  # instant → q_retarget
    B1_SLOW_RETARGET = "B1"  # slow interp → q_retarget
    B2_SLOW_SQUEEZE = "B2"  # slow interp → squeeze(q_retarget)
    B3_CONTACT_GATE = "B3"  # TacDexGrasp-style: freeze finger on contact
    B4_GATED_RAMP = "B4"  # B3 + uniform micro-ramp after contact
    B5_GATED_RAMP_WRIST = "B5"  # B4 + wrist micro-press

    @classmethod
    def parse(cls, value: str | "ExecutorId") -> "ExecutorId":
        if isinstance(value, ExecutorId):
            return value
        raw = str(value).strip().upper()
        # accept "B2_SLOW_SQUEEZE" or "B2"
        if raw in cls.__members__:
            return cls[raw]
        for member in cls:
            if member.value == raw or raw.startswith(member.value + "_"):
                return member
        raise ValueError(
            f"unknown executor {value!r}; expected one of "
            f"{[m.value for m in cls]}"
        )


FINGER_IDS: tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")


@dataclass
class ContactSnapshot:
    """Per-link / per-finger contact at one control tick.

    Binary ``in_contact`` is the primary gate; ``force_mag`` is diagnostic only
    (PhysX magnitudes are not calibrated Newtons — do not track absolute force).
    """

    stamp_sec: float = 0.0
    any_contact: bool = False
    # link name → in contact (Bool topic)
    link_in_contact: dict[str, bool] = field(default_factory=dict)
    # link name → |F| if wrench subscribed
    link_force_mag: dict[str, float] = field(default_factory=dict)
    # finger id → OR of its contact links (filled by profile)
    finger_in_contact: dict[str, bool] = field(default_factory=dict)
    # link → partner USD paths (from /contact_partners); used to ignore table
    link_partners: dict[str, list[str]] = field(default_factory=dict)

    def contacted_fingers(self) -> list[str]:
        return [f for f, v in self.finger_in_contact.items() if v]


@dataclass
class CloseStepLog:
    """One control tick for table / JSONL export."""

    step: int
    phase: str  # approach | ramp | wrist | done
    t_sec: float
    alpha: float
    q: list[float]
    frozen_fingers: list[str]
    contact: dict[str, bool]
    any_contact: bool
    force_mag: dict[str, float] = field(default_factory=dict)
    object_xyz: Optional[list[float]] = None
    wrist_xyz: Optional[list[float]] = None


@dataclass
class CloseRunResult:
    """Outcome of one Close execution (for UI status + table rows)."""

    executor_id: str
    arm: str
    hand_profile: str
    ok: bool
    message: str
    q_start: list[float]
    q_goal: list[float]
    q_final: list[float]
    steps: list[CloseStepLog] = field(default_factory=list)
    n_contact_fingers_max: int = 0
    log_path: Optional[str] = None
    extras: dict[str, Any] = field(default_factory=dict)

    def summary_row(self) -> dict[str, Any]:
        """Flat dict suitable for CSV / ablation tables."""
        return {
            "executor_id": self.executor_id,
            "arm": self.arm,
            "hand_profile": self.hand_profile,
            "ok": self.ok,
            "n_steps": len(self.steps),
            "n_contact_fingers_max": self.n_contact_fingers_max,
            "log_path": self.log_path,
            "message": self.message,
        }


@dataclass
class CloseRequest:
    """Inputs for ``run_close`` (retarget already done or provided)."""

    arm: str
    q_retarget: Sequence[float]
    q_start: Optional[Sequence[float]] = None  # default: open or last sent
    squeeze_amount: float = 0.5
    thumb_abd_boost: float = 1.25
    executor_id: ExecutorId = ExecutorId.B2_SLOW_SQUEEZE
    # timing
    close_steps: int = 40
    close_dt_s: float = 0.04
    # contact gate
    contact_hold_frames: int = 2
    min_contact_fingers: int = 2
    # "full" = freeze whole finger; "keep_tip" = freeze MCP, keep PIP/DIP curling
    freeze_mode: str = "keep_tip"
    # B4/B5 ramp
    ramp_steps: int = 25
    ramp_amount: float = 0.18
    tip_ramp_boost: float = 0.35  # extra curl on tip joints during ramp
    tip_pinch_steps: int = 12  # post-ramp tip-only curl (small objects)
    tip_pinch_amount: float = 0.25
    # B5 wrist
    wrist_press_m: float = 0.008
    # optional object frame for Δpose (empty → skip)
    object_frame: str = ""
    base_frame: str = "base_link"
    # shape phase: open-loop to MANO by default (gate only on B3+ close)
    do_shape_phase: bool = True
    shape_steps: int = 30
    shape_settle_s: float = 0.5
    shape_use_contact_gate: bool = False
    # world +Z lift before shape/close (clearance vs table for small objects)
    pre_close_lift_m: float = 0.015
    # logging
    trial_id: str = ""
    meta: Mapping[str, Any] = field(default_factory=dict)
