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
    MeasureHandFn,
    ObjectPoseFn,
    SendHandFn,
    WristPoseFn,
    WristPressFn,
    base_result,
    make_step_log,
    run_gated_approach,
    run_open_loop_approach,
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
        measure_hand: Optional[MeasureHandFn] = None,
    ) -> CloseRunResult:
        q_start, _, q_goal = _resolve_start_goal(req, profile)
        t0 = time.time()
        send_hand(q_goal)
        snap = contact.snapshot()
        meas = measure_hand() if measure_hand else None
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
                q_meas=meas,
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
        measure_hand: Optional[MeasureHandFn] = None,
    ) -> CloseRunResult:
        q_start, _, q_goal = _resolve_start_goal(req, profile)
        # B2 squeezes after shape; stop on real force-contact to avoid PhysX bounce-open.
        stop = self._mode == ExecutorId.B2_SLOW_SQUEEZE
        t0 = time.time()
        q_cmd, steps = run_open_loop_approach(
            q_start=q_start,
            q_goal=q_goal,
            contact=contact,
            send_hand=send_hand,
            n_steps=int(req.close_steps),
            dt=float(req.close_dt_s),
            phase="approach",
            t0=t0,
            object_xyz=object_xyz,
            wrist_xyz=wrist_xyz,
            measure_hand=measure_hand,
            stop_on_contact=stop,
            hold_frames=max(1, int(req.contact_hold_frames)),
        )

        label = (
            "B1 slow → q_retarget"
            if self._mode == ExecutorId.B1_SLOW_RETARGET
            else "B2 slow → squeeze(q_retarget)"
            + (" [stop_on_contact]" if stop else "")
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
        measure_hand: Optional[MeasureHandFn] = None,
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
        soft_stop_n = float(getattr(req, "soft_stop_force_n", 10.0) or 0.0)
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
            measure_hand=measure_hand,
            soft_stop_force_n=soft_stop_n,
        )
        soft_stopped = any(s.phase == "soft_stop" for s in steps)

        tip_map = getattr(profile, "tip_joint_indices", None) or {}

        def _peak_force(snap) -> float:
            mags = getattr(snap, "link_force_mag", None) or {}
            return float(max(mags.values())) if mags else 0.0

        # --- B4/B5: uniform micro-ramp + tip boost on contacted fingers ---
        if do_ramp and not soft_stopped:
            q_open = profile.open_hand_q()
            ramp_gain = float(max(0.0, req.ramp_amount))
            tip_boost = float(max(0.0, getattr(req, "tip_ramp_boost", 0.0)))

            def _fill_ramp_target(fingers: set[str], dest: np.ndarray) -> None:
                for finger in fingers:
                    for idx in profile.finger_joint_indices.get(finger, ()):
                        delta = float(q_goal[idx] - q_open[idx])
                        gain = ramp_gain
                        if idx in tip_map.get(finger, ()):
                            gain = ramp_gain + tip_boost
                        dest[idx] = float(q_goal[idx] + gain * delta)

            q_ramp = q_goal.copy()
            _fill_ramp_target(set(frozen), q_ramp)

            n_ramp = max(1, int(req.ramp_steps))
            q_ramp_start = q_cmd.copy()
            # Pose locked when each finger freezes. Using global q_ramp_start for a
            # finger that freezes mid-ramp snapped MCP/tips more open (接触后张开).
            q_at_freeze = q_ramp_start.copy()
            freeze_step: dict[str, int] = {f: 0 for f in frozen}
            hold_counts: dict[str, int] = {f: hold_frames for f in frozen}
            for j in range(n_ramp):
                snap = contact.snapshot()
                if soft_stop_n > 0.0 and _peak_force(snap) >= soft_stop_n:
                    # Already gripping hard — hold; further curl causes open/close pulse.
                    send_hand(q_cmd)
                    meas = measure_hand() if measure_hand else None
                    steps.append(
                        make_step_log(
                            step=len(steps),
                            phase="soft_stop",
                            t0=t0,
                            alpha=float(j + 1) / float(n_ramp),
                            q=q_cmd,
                            frozen=set(frozen),
                            snap=snap,
                            object_xyz=object_xyz,
                            wrist_xyz=wrist_xyz,
                            q_meas=meas,
                        )
                    )
                    soft_stopped = True
                    break
                prev_frozen = set(frozen)
                frozen = update_freeze(
                    frozen,
                    snap,
                    hold_counts=hold_counts,
                    hold_frames=hold_frames,
                )
                newly = frozen - prev_frozen
                if newly:
                    for finger in newly:
                        freeze_step[finger] = j
                        for idx in profile.finger_joint_indices.get(finger, ()):
                            q_at_freeze[idx] = float(q_cmd[idx])
                    _fill_ramp_target(newly, q_ramp)

                alpha_global = float(j + 1) / float(n_ramp)
                q_blend = q_ramp_start.copy()
                for finger, idxs in profile.finger_joint_indices.items():
                    if finger in frozen:
                        j0 = int(freeze_step.get(finger, 0))
                        denom = max(1, n_ramp - j0)
                        local_alpha = float(j - j0 + 1) / float(denom)
                        local_alpha = float(min(1.0, max(0.0, local_alpha)))
                        tips = set(tip_map.get(finger, ()))
                        for idx in idxs:
                            if freeze_mode == "keep_tip" and idx not in tips:
                                q_blend[idx] = q_at_freeze[idx]
                            else:
                                q_blend[idx] = (1.0 - local_alpha) * q_at_freeze[
                                    idx
                                ] + local_alpha * float(q_ramp[idx])
                    else:
                        for idx in idxs:
                            q_blend[idx] = (1.0 - alpha_global) * q_ramp_start[
                                idx
                            ] + alpha_global * q_goal[idx]
                q_cmd = q_blend.astype(np.float32)
                send_hand(q_cmd)
                meas = measure_hand() if measure_hand else None
                steps.append(
                    make_step_log(
                        step=len(steps),
                        phase="ramp",
                        t0=t0,
                        alpha=alpha_global,
                        q=q_cmd,
                        frozen=set(frozen),
                        snap=snap,
                        object_xyz=object_xyz,
                        wrist_xyz=wrist_xyz,
                        q_meas=meas,
                    )
                )
                sleep_dt(dt)

            # Tip-only pinch: extra DIP/IP curl (lime / grape scale).
            # Skip when soft-stop already fired — apple logs showed tip_pinch
            # pushing cmd while contact force fell 45 N → 0 (stable slip, not fly).
            n_pinch = max(0, int(getattr(req, "tip_pinch_steps", 0)))
            pinch_amt = float(max(0.0, getattr(req, "tip_pinch_amount", 0.0)))
            if (
                n_pinch > 0
                and pinch_amt > 0.0
                and frozen
                and not soft_stopped
            ):
                snap0 = contact.snapshot()
                if soft_stop_n > 0.0 and _peak_force(snap0) >= soft_stop_n:
                    soft_stopped = True
                else:
                    q_pinch = q_cmd.copy()
                    for finger in frozen:
                        for idx in tip_map.get(finger, ()):
                            delta = float(q_goal[idx] - q_open[idx])
                            if abs(delta) < 1e-4:
                                delta = float(q_goal[idx] - q_cmd[idx]) or 0.15
                            q_pinch[idx] = float(q_cmd[idx] + pinch_amt * delta)
                    q_pinch_start = q_cmd.copy()
                    for j in range(n_pinch):
                        snap = contact.snapshot()
                        if soft_stop_n > 0.0 and _peak_force(snap) >= soft_stop_n:
                            send_hand(q_cmd)
                            meas = measure_hand() if measure_hand else None
                            steps.append(
                                make_step_log(
                                    step=len(steps),
                                    phase="soft_stop",
                                    t0=t0,
                                    alpha=float(j + 1) / float(n_pinch),
                                    q=q_cmd,
                                    frozen=set(frozen),
                                    snap=snap,
                                    object_xyz=object_xyz,
                                    wrist_xyz=wrist_xyz,
                                    q_meas=meas,
                                )
                            )
                            soft_stopped = True
                            break
                        alpha = float(j + 1) / float(n_pinch)
                        q_cmd = (
                            (1.0 - alpha) * q_pinch_start + alpha * q_pinch
                        ).astype(np.float32)
                        send_hand(q_cmd)
                        meas = measure_hand() if measure_hand else None
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
                                q_meas=meas,
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
                    meas = measure_hand() if measure_hand else None
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
                            q_meas=meas,
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
        stop_msg = " [soft_stop]" if soft_stopped else ""
        return base_result(
            req,
            profile,
            q_start=q_start,
            q_goal=q_goal,
            q_final=q_cmd,
            steps=steps,
            ok=True,
            message=labels.get(self._mode, self.id) + wrist_msg + stop_msg,
        )
