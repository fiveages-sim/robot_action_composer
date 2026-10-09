"""B0–B2 open-loop and B3–B5 contact-gated close executors."""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from ..contact.base import ContactSensor
from ..profiles.base import HandExecutionProfile
from ..types import CloseRequest, CloseRunResult, CloseStepLog, ExecutorId
from .base import (
    CloseExecutor,
    ObjectPoseFn,
    SendHandFn,
    WristPoseFn,
    WristPressFn,
    base_result,
    make_step_log,
    run_gated_approach,
    sleep_dt,
    update_freeze,
)


def _as_q(q, n: int) -> np.ndarray:
    return np.asarray(q, dtype=np.float32).reshape(n)


def _resolve_start_goal(
    req: CloseRequest,
    profile: HandExecutionProfile,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (q_start, q_retarget, q_goal) for the selected mode."""
    n = profile.n_dof
    q_ret = _as_q(req.q_retarget, n)
    if req.q_start is not None:
        q_start = _as_q(req.q_start, n)
    else:
        q_start = profile.open_hand_q()

    eid = ExecutorId.parse(req.executor_id)
    if eid in (ExecutorId.B0_INSTANT, ExecutorId.B1_SLOW_RETARGET):
        q_goal = q_ret.copy()
    else:
        # B2–B5: squeeze-amplified goal (same target; differ by timing/gates)
        q_goal = profile.apply_squeeze(
            q_ret,
            amount=float(req.squeeze_amount),
            thumb_abd_boost=float(req.thumb_abd_boost),
        )
    return q_start, q_ret, q_goal


class InstantRetargetExecutor(CloseExecutor):
    """B0: single send of ``q_retarget`` (ablation baseline; prefer gated shape)."""

    id = "B0"

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
        q_start, _, q_goal = _resolve_start_goal(req, profile)
        t0 = time.time()
        send_hand(q_goal)
        snap = contact.snapshot()
        steps = [
            make_step_log(
                step=0,
                phase="instant",
                t0=t0,
                alpha=1.0,
                q=q_goal,
                frozen=set(),
                snap=snap,
                object_xyz=object_xyz,
                wrist_xyz=wrist_xyz,
            )
        ]
        return base_result(
            req,
            profile,
            q_start=q_start,
            q_goal=q_goal,
            q_final=q_goal,
            steps=steps,
            ok=True,
            message="B0 instant → q_retarget",
        )


class SlowOpenLoopExecutor(CloseExecutor):
    """B1 / B2: time-parameterized open-loop interpolation (no contact gate)."""

    def __init__(self, mode: ExecutorId) -> None:
        self._mode = ExecutorId.parse(mode)
        self.id = self._mode.value

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
        q_start, _, q_goal = _resolve_start_goal(req, profile)
        n_steps = max(1, int(req.close_steps))
        dt = float(req.close_dt_s)
        t0 = time.time()
        steps: list[CloseStepLog] = []
        q_cmd = q_start.copy()

        for i in range(n_steps):
            alpha = float(i + 1) / float(n_steps)
            q_cmd = (q_start + alpha * (q_goal - q_start)).astype(np.float32)
            send_hand(q_cmd)
            snap = contact.snapshot()
            steps.append(
                make_step_log(
                    step=i,
                    phase="approach",
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

        label = (
            "B1 slow → q_retarget"
            if self._mode == ExecutorId.B1_SLOW_RETARGET
            else "B2 slow → squeeze(q_retarget)"
        )
        return base_result(
            req,
            profile,
            q_start=q_start,
            q_goal=q_goal,
            q_final=q_cmd,
            steps=steps,
            ok=True,
            message=label,
        )


class ContactGatedExecutor(CloseExecutor):
    """B3–B5: TacDexGrasp-style freeze + optional CoorGrasp-lite ramp / tip pinch.

    ``freeze_mode=keep_tip`` (default): MCP freezes on contact, tip joints keep
    curling so lime-sized objects wrap instead of skittering off the fingertip.
    """

    def __init__(self, mode: ExecutorId) -> None:
        self._mode = ExecutorId.parse(mode)
        self.id = self._mode.value

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
        q_start, _, q_goal = _resolve_start_goal(req, profile)
        dt = float(req.close_dt_s)
        hold_frames = max(1, int(req.contact_hold_frames))
        min_fingers = max(1, int(req.min_contact_fingers))
        freeze_mode = str(getattr(req, "freeze_mode", "keep_tip") or "keep_tip")
        do_ramp = self._mode in (
            ExecutorId.B4_GATED_RAMP,
            ExecutorId.B5_GATED_RAMP_WRIST,
        )
        do_wrist = self._mode == ExecutorId.B5_GATED_RAMP_WRIST

        t0 = time.time()
        q_cmd, frozen, steps = run_gated_approach(
            q_start=q_start,
            q_goal=q_goal,
            profile=profile,
            contact=contact,
            send_hand=send_hand,
            n_steps=int(req.close_steps),
            dt=dt,
            hold_frames=hold_frames,
            freeze_mode=freeze_mode,
            phase="approach",
            t0=t0,
            object_xyz=object_xyz,
            wrist_xyz=wrist_xyz,
            early_exit_fingers=min_fingers if do_ramp else None,
        )

        tip_map = getattr(profile, "tip_joint_indices", None) or {}

        # --- B4/B5: uniform micro-ramp + tip boost on contacted fingers ---
        if do_ramp:
            q_open = profile.open_hand_q()
            ramp_gain = float(max(0.0, req.ramp_amount))
            tip_boost = float(max(0.0, getattr(req, "tip_ramp_boost", 0.0)))
            q_ramp = q_goal.copy()
            for finger in frozen:
                for idx in profile.finger_joint_indices.get(finger, ()):
                    delta = float(q_goal[idx] - q_open[idx])
                    gain = ramp_gain
                    if idx in tip_map.get(finger, ()):
                        gain = ramp_gain + tip_boost
                    q_ramp[idx] = float(q_goal[idx] + gain * delta)

            n_ramp = max(1, int(req.ramp_steps))
            q_ramp_start = q_cmd.copy()
            hold_counts: dict[str, int] = {f: hold_frames for f in frozen}
            for j in range(n_ramp):
                snap = contact.snapshot()
                frozen = update_freeze(
                    frozen,
                    snap,
                    hold_counts=hold_counts,
                    hold_frames=hold_frames,
                )
                alpha = float(j + 1) / float(n_ramp)
                q_blend = q_ramp_start.copy()
                for finger, idxs in profile.finger_joint_indices.items():
                    if finger in frozen:
                        for idx in idxs:
                            # tip joints always blend toward ramp; MCP held
                            tips = set(tip_map.get(finger, ()))
                            if freeze_mode == "keep_tip" and idx not in tips:
                                q_blend[idx] = q_ramp_start[idx]
                            else:
                                q_blend[idx] = (1.0 - alpha) * q_ramp_start[
                                    idx
                                ] + alpha * q_ramp[idx]
                    else:
                        for idx in idxs:
                            q_blend[idx] = (1.0 - alpha) * q_ramp_start[
                                idx
                            ] + alpha * q_goal[idx]
                q_cmd = q_blend.astype(np.float32)
                send_hand(q_cmd)
                steps.append(
                    make_step_log(
                        step=len(steps),
                        phase="ramp",
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

            # Tip-only pinch: extra DIP/IP curl (lime / grape scale)
            n_pinch = max(0, int(getattr(req, "tip_pinch_steps", 0)))
            pinch_amt = float(max(0.0, getattr(req, "tip_pinch_amount", 0.0)))
            if n_pinch > 0 and pinch_amt > 0.0 and frozen:
                q_pinch = q_cmd.copy()
                for finger in frozen:
                    for idx in tip_map.get(finger, ()):
                        delta = float(q_goal[idx] - q_open[idx])
                        if abs(delta) < 1e-4:
                            # no retarget curl — nudge toward squeeze goal anyway
                            delta = float(q_goal[idx] - q_cmd[idx]) or 0.15
                        q_pinch[idx] = float(q_cmd[idx] + pinch_amt * delta)
                q_pinch_start = q_cmd.copy()
                for j in range(n_pinch):
                    alpha = float(j + 1) / float(n_pinch)
                    q_cmd = (
                        (1.0 - alpha) * q_pinch_start + alpha * q_pinch
                    ).astype(np.float32)
                    send_hand(q_cmd)
                    snap = contact.snapshot()
                    steps.append(
                        make_step_log(
                            step=len(steps),
                            phase="tip_pinch",
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

        # --- B5: wrist micro-press ---
        wrist_msg = ""
        if do_wrist:
            if wrist_press is None:
                wrist_msg = " (wrist press skipped: no plan/callback)"
            else:
                try:
                    wrist_press(float(req.wrist_press_m))
                    snap = contact.snapshot()
                    steps.append(
                        make_step_log(
                            step=len(steps),
                            phase="wrist",
                            t0=t0,
                            alpha=1.0,
                            q=q_cmd,
                            frozen=set(frozen),
                            snap=snap,
                            object_xyz=object_xyz,
                            wrist_xyz=wrist_xyz,
                        )
                    )
                    wrist_msg = f" + wrist_press={req.wrist_press_m:.3f}m"
                except Exception as exc:
                    wrist_msg = f" (wrist press failed: {exc})"

        labels = {
            ExecutorId.B3_CONTACT_GATE: "B3 contact-gate (keep_tip)",
            ExecutorId.B4_GATED_RAMP: "B4 gate + ramp + tip_pinch",
            ExecutorId.B5_GATED_RAMP_WRIST: "B5 gate + ramp + wrist",
        }
        return base_result(
            req,
            profile,
            q_start=q_start,
            q_goal=q_goal,
            q_final=q_cmd,
            steps=steps,
            ok=True,
            message=labels.get(self._mode, self.id) + wrist_msg,
        )
