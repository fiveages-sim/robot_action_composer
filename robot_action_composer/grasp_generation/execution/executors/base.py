"""Close executor protocol + shared helpers."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, Sequence

import numpy as np

from ..contact.base import ContactSensor
from ..profiles.base import HandExecutionProfile
from ..types import CloseRequest, CloseRunResult, CloseStepLog, ContactSnapshot


SendHandFn = Callable[[Sequence[float]], None]
WristPressFn = Callable[[float], None]  # press_m along tip approach
ObjectPoseFn = Callable[[], Optional[list[float]]]
WristPoseFn = Callable[[], Optional[list[float]]]


class CloseExecutor(ABC):
    """One ablation mode (B0–B5). Stateless aside from request fields."""

    id: str = "B?"

    @abstractmethod
    def run(
        self,
        req: CloseRequest,
        *,
        profile: HandExecutionProfile,
        contact: ContactSensor,
        send_hand: SendHandFn,
        wrist_press: Optional[WristPressFn] = None,
        object_xyz: Optional[ObjectPoseFn] = None,
        wrist_xyz: Optional[WristPoseFn] = None,
    ) -> CloseRunResult:
        ...


def lerp_q(
    q0: np.ndarray,
    q1: np.ndarray,
    alpha: float,
    *,
    mask: Optional[np.ndarray] = None,
    hold: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Linear blend; ``mask[i]=False`` keeps ``hold[i]`` (contact freeze)."""
    a = float(np.clip(alpha, 0.0, 1.0))
    out = q0 + a * (q1 - q0)
    if mask is not None and hold is not None:
        frozen = ~np.asarray(mask, dtype=bool)
        out = out.copy()
        out[frozen] = hold[frozen]
    return out.astype(np.float32)


def finger_mask(
    profile: HandExecutionProfile,
    frozen_fingers: set[str],
    *,
    freeze_mode: str = "full",
) -> np.ndarray:
    """True = joint may move; False = frozen.

    ``keep_tip``: freeze MCP / abduction of contacted fingers but keep tip
    joints (PIP/DIP) free so small objects can still be wrapped.
    """
    mask = np.ones(profile.n_dof, dtype=bool)
    mode = (freeze_mode or "full").strip().lower()
    tip_map = getattr(profile, "tip_joint_indices", None) or {}
    for finger, idxs in profile.finger_joint_indices.items():
        if finger not in frozen_fingers:
            continue
        if mode == "keep_tip":
            tips = set(tip_map.get(finger, ()))
            for i in idxs:
                if i not in tips:
                    mask[i] = False
        else:
            for i in idxs:
                mask[i] = False
    return mask


def update_freeze(
    frozen: set[str],
    snap: ContactSnapshot,
    *,
    hold_counts: dict[str, int],
    hold_frames: int,
) -> set[str]:
    """Latch finger freeze after ``hold_frames`` consecutive contacts."""
    for finger, touching in snap.finger_in_contact.items():
        if finger in frozen:
            continue
        if touching:
            hold_counts[finger] = hold_counts.get(finger, 0) + 1
            if hold_counts[finger] >= max(1, hold_frames):
                frozen.add(finger)
        else:
            hold_counts[finger] = 0
    return frozen


def run_gated_approach(
    *,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    profile: HandExecutionProfile,
    contact: ContactSensor,
    send_hand: SendHandFn,
    n_steps: int,
    dt: float,
    hold_frames: int,
    freeze_mode: str,
    phase: str,
    t0: float,
    step_offset: int = 0,
    object_xyz: Optional[ObjectPoseFn] = None,
    wrist_xyz: Optional[WristPoseFn] = None,
    early_exit_fingers: Optional[int] = None,
) -> tuple[np.ndarray, set[str], list[CloseStepLog]]:
    """Contact-gated interpolate ``q_start → q_goal`` (retarget or squeeze)."""
    n_steps = max(1, int(n_steps))
    frozen: set[str] = set()
    hold_counts: dict[str, int] = {}
    q_cmd = q_start.copy()
    q_hold = q_start.copy()
    steps: list[CloseStepLog] = []

    for i in range(n_steps):
        snap = contact.snapshot()
        frozen = update_freeze(
            frozen,
            snap,
            hold_counts=hold_counts,
            hold_frames=hold_frames,
        )
        alpha = float(i + 1) / float(n_steps)
        mask = finger_mask(profile, frozen, freeze_mode=freeze_mode)
        for finger in frozen:
            tips = set((getattr(profile, "tip_joint_indices", None) or {}).get(finger, ()))
            for idx in profile.finger_joint_indices.get(finger, ()):
                if freeze_mode == "keep_tip" and idx in tips:
                    continue
                q_hold[idx] = q_cmd[idx]
        q_cmd = lerp_q(q_start, q_goal, alpha, mask=mask, hold=q_hold)
        send_hand(q_cmd)
        steps.append(
            make_step_log(
                step=step_offset + i,
                phase=phase,
                t0=t0,
                alpha=alpha,
                q=q_cmd,
                frozen=set(frozen),
                snap=snap,
                object_xyz=object_xyz,
                wrist_xyz=wrist_xyz,
            )
        )
        sleep_dt(dt)
        if early_exit_fingers is not None and len(frozen) >= early_exit_fingers:
            break

    return q_cmd, frozen, steps


def run_open_loop_approach(
    *,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    contact: ContactSensor,
    send_hand: SendHandFn,
    n_steps: int,
    dt: float,
    phase: str,
    t0: float,
    step_offset: int = 0,
    object_xyz: Optional[ObjectPoseFn] = None,
    wrist_xyz: Optional[WristPoseFn] = None,
) -> tuple[np.ndarray, list[CloseStepLog]]:
    """Open-loop interpolate (no freeze) — used for MANO shape retarget."""
    n_steps = max(1, int(n_steps))
    q_cmd = q_start.copy()
    steps: list[CloseStepLog] = []
    for i in range(n_steps):
        alpha = float(i + 1) / float(n_steps)
        q_cmd = lerp_q(q_start, q_goal, alpha)
        send_hand(q_cmd)
        snap = contact.snapshot()
        steps.append(
            make_step_log(
                step=step_offset + i,
                phase=phase,
                t0=t0,
                alpha=alpha,
                q=q_cmd,
                frozen=set(),
                snap=snap,
                object_xyz=object_xyz,
                wrist_xyz=wrist_xyz,
            )
        )
        sleep_dt(dt)
    return q_cmd, steps


def sleep_dt(dt: float) -> None:
    if dt > 0.0:
        time.sleep(dt)


def base_result(
    req: CloseRequest,
    profile: HandExecutionProfile,
    *,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    q_final: np.ndarray,
    steps: list[CloseStepLog],
    ok: bool,
    message: str,
) -> CloseRunResult:
    n_max = 0
    for s in steps:
        n_max = max(n_max, sum(1 for v in s.contact.values() if v))
    return CloseRunResult(
        executor_id=req.executor_id.value
        if hasattr(req.executor_id, "value")
        else str(req.executor_id),
        arm=req.arm,
        hand_profile=profile.name,
        ok=ok,
        message=message,
        q_start=[float(x) for x in q_start.reshape(-1)],
        q_goal=[float(x) for x in q_goal.reshape(-1)],
        q_final=[float(x) for x in q_final.reshape(-1)],
        steps=steps,
        n_contact_fingers_max=n_max,
        extras=dict(req.meta) if req.meta else {},
    )


def make_step_log(
    *,
    step: int,
    phase: str,
    t0: float,
    alpha: float,
    q: np.ndarray,
    frozen: set[str],
    snap: ContactSnapshot,
    object_xyz: Optional[ObjectPoseFn],
    wrist_xyz: Optional[WristPoseFn],
) -> CloseStepLog:
    ox = object_xyz() if object_xyz else None
    wx = wrist_xyz() if wrist_xyz else None
    return CloseStepLog(
        step=step,
        phase=phase,
        t_sec=time.time() - t0,
        alpha=float(alpha),
        q=[float(x) for x in q.reshape(-1)],
        frozen_fingers=sorted(frozen),
        contact=dict(snap.finger_in_contact),
        any_contact=bool(snap.any_contact),
        force_mag={k: float(v) for k, v in snap.link_force_mag.items()},
        object_xyz=ox,
        wrist_xyz=wx,
    )
