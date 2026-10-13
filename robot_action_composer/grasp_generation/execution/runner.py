"""High-level ``run_close``: lift → open-loop MANO shape → settle → B0–B5."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from ..config import GraspGenerationConfig
from ..retarget.pipeline import send_hand_q
from .close_log import append_table_row, write_close_log
from .contact import make_contact_sensor
from .executors import get_executor
from .executors.base import run_gated_approach, run_open_loop_approach
from .profiles import get_hand_profile
from .types import CloseRequest, CloseRunResult, CloseStepLog, ExecutorId
from .wrist import lookup_frame_xyz, press_wrist_along_tip_z, press_wrist_world_z


def close_request_from_config(
    cfg: GraspGenerationConfig,
    *,
    arm: str,
    q_retarget: Sequence[float],
    q_start: Optional[Sequence[float]] = None,
    squeeze_amount: Optional[float] = None,
    executor_id: Optional[str] = None,
    trial_id: str = "",
    do_shape_phase: Optional[bool] = None,
) -> CloseRequest:
    """Build a ``CloseRequest`` from YAML / UI overrides."""
    eid = ExecutorId.parse(
        executor_id or getattr(cfg, "close_executor", "B4")
    )
    shape_default = bool(getattr(cfg, "auto_retarget_on_close", True))
    return CloseRequest(
        arm=arm,
        q_retarget=list(q_retarget),
        q_start=list(q_start) if q_start is not None else None,
        squeeze_amount=float(
            cfg.squeeze_amount if squeeze_amount is None else squeeze_amount
        ),
        thumb_abd_boost=float(getattr(cfg, "thumb_abd_boost", 1.25)),
        executor_id=eid,
        close_steps=int(getattr(cfg, "close_steps", 40)),
        close_dt_s=float(getattr(cfg, "close_dt_s", 0.04)),
        contact_hold_frames=int(getattr(cfg, "contact_hold_frames", 2)),
        min_contact_fingers=int(getattr(cfg, "min_contact_fingers", 2)),
        freeze_mode=str(getattr(cfg, "freeze_mode", "keep_tip") or "keep_tip"),
        ramp_steps=int(getattr(cfg, "ramp_steps", 25)),
        ramp_amount=float(getattr(cfg, "ramp_amount", 0.18)),
        tip_ramp_boost=float(getattr(cfg, "tip_ramp_boost", 0.35)),
        tip_pinch_steps=int(getattr(cfg, "tip_pinch_steps", 12)),
        tip_pinch_amount=float(getattr(cfg, "tip_pinch_amount", 0.25)),
        wrist_press_m=float(getattr(cfg, "wrist_press_m", 0.008)),
        object_frame=str(getattr(cfg, "object_frame", "") or ""),
        base_frame=str(getattr(cfg, "base_frame", "base_link")),
        do_shape_phase=shape_default if do_shape_phase is None else bool(do_shape_phase),
        shape_steps=int(getattr(cfg, "shape_steps", 30)),
        shape_settle_s=float(getattr(cfg, "shape_settle_s", 0.5)),
        shape_use_contact_gate=bool(
            getattr(cfg, "shape_use_contact_gate", False)
        ),
        pre_close_lift_m=float(getattr(cfg, "pre_close_lift_m", 0.015)),
        trial_id=trial_id,
    )


def run_close(
    *,
    cfg: GraspGenerationConfig,
    interface: Any,
    q_retarget: Sequence[float],
    arm: Optional[str] = None,
    q_start: Optional[Sequence[float]] = None,
    squeeze_amount: Optional[float] = None,
    executor_id: Optional[str] = None,
    last_plan: Any = None,
    workspace: Optional[Path] = None,
    trial_id: str = "",
    log: bool = True,
    do_shape_phase: Optional[bool] = None,
) -> CloseRunResult:
    """Optional wrist lift → open-loop MANO shape → settle → B0–B5 close.

    Shape is **open-loop** by default so the hand visibly matches HUG MANO.
    Contact gating applies only in B3+ close (table partners filtered out).
    """
    side = (arm or cfg.arm or "right").lower()
    if side not in ("left", "right"):
        raise ValueError(f"arm must be left|right, got {side!r}")

    profile = get_hand_profile(cfg.hand_profile)
    req = close_request_from_config(
        cfg,
        arm=side,
        q_retarget=q_retarget,
        q_start=q_start,
        squeeze_amount=squeeze_amount,
        executor_id=executor_id,
        trial_id=trial_id,
        do_shape_phase=do_shape_phase,
    )
    eid = ExecutorId.parse(req.executor_id)

    ignore = getattr(cfg, "ignore_contact_partners", None)
    contact = make_contact_sensor(
        profile,
        arm=side,
        topic_prefix=str(getattr(cfg, "contact_topic_prefix", "") or ""),
        enabled=True,
        force_mag_threshold_n=float(
            getattr(cfg, "force_mag_threshold_n", 0.0)
        ),
        use_bool=bool(getattr(cfg, "use_contact_bool", True)),
        ignore_partners=ignore,
    )

    def _send(q: Sequence[float]) -> None:
        send_hand_q(interface, side, q)  # type: ignore[arg-type]

    def _wrist_press(press_m: float) -> None:
        press_wrist_along_tip_z(
            interface,
            last_plan,
            press_m=press_m,
            arrival_timeout=float(getattr(cfg, "arrival_timeout", 8.0)),
            axis=str(getattr(cfg, "wrist_press_axis", "tip_neg_z")),
        )

    obj0 = None
    push_warn = float(getattr(cfg, "object_push_warn_m", 0.015))

    def _object_xyz():
        return lookup_frame_xyz(
            interface, req.object_frame, req.base_frame
        )

    def _wrist_xyz():
        tip = cfg.resolved_control_tip_frame()
        return lookup_frame_xyz(interface, tip, req.base_frame)

    n = profile.n_dof
    q_ret = np.asarray(req.q_retarget, dtype=np.float32).reshape(n)
    if req.q_start is not None:
        q_cur = np.asarray(req.q_start, dtype=np.float32).reshape(n)
    else:
        q_cur = profile.open_hand_q()

    prefix_steps: list[CloseStepLog] = []
    extras: dict[str, Any] = {}
    contact.start()
    try:
        if req.object_frame:
            obj0 = _object_xyz()
            extras["object_xyz0"] = obj0

        # --- Optional world +Z lift (clearance vs table for lime-scale) ---
        lift = float(req.pre_close_lift_m)
        if lift > 1e-6 and last_plan is not None:
            try:
                extras["pre_close_lift"] = press_wrist_world_z(
                    interface,
                    last_plan,
                    lift_m=lift,
                    arrival_timeout=float(
                        getattr(cfg, "arrival_timeout", 8.0)
                    ),
                )
            except Exception as exc:
                extras["pre_close_lift_error"] = str(exc)

        # --- Phase A: open-loop (default) → visible MANO shape ---
        if req.do_shape_phase:
            # Snap to commanded start first. Otherwise leftover curled joints
            # from a prior grasp / STOP→PLAY look like a "fake close", then the
            # first interp frames jump open before the real MANO approach.
            _send(q_cur)
            time.sleep(min(0.25, max(0.05, float(req.close_dt_s) * 2.0)))
            t0 = time.time()
            if req.shape_use_contact_gate:
                q_cur, _fr, shape_steps = run_gated_approach(
                    q_start=q_cur,
                    q_goal=q_ret,
                    profile=profile,
                    contact=contact,
                    send_hand=_send,
                    n_steps=int(req.shape_steps),
                    dt=float(req.close_dt_s),
                    hold_frames=int(req.contact_hold_frames),
                    freeze_mode=str(req.freeze_mode),
                    phase="shape",
                    t0=t0,
                    object_xyz=_object_xyz if req.object_frame else None,
                    wrist_xyz=_wrist_xyz,
                )
            else:
                q_cur, shape_steps = run_open_loop_approach(
                    q_start=q_cur,
                    q_goal=q_ret,
                    contact=contact,
                    send_hand=_send,
                    n_steps=int(req.shape_steps),
                    dt=float(req.close_dt_s),
                    phase="shape",
                    t0=t0,
                    object_xyz=_object_xyz if req.object_frame else None,
                    wrist_xyz=_wrist_xyz,
                )
            prefix_steps.extend(shape_steps)
            # Snap to exact retarget (open-loop last step ≈ goal; enforce)
            q_cur = q_ret.copy()
            _send(q_cur)
            settle = float(req.shape_settle_s)
            if settle > 0.0:
                time.sleep(settle)

            if obj0 is not None:
                obj1 = _object_xyz()
                extras["object_xyz_after_shape"] = obj1
                if obj0 and obj1:
                    d = float(
                        np.linalg.norm(
                            np.asarray(obj1) - np.asarray(obj0)
                        )
                    )
                    extras["object_push_m_after_shape"] = d
                    if d >= push_warn:
                        extras["object_push_warn"] = True

        # B0/B1 with shape: done after MANO
        if eid in (ExecutorId.B0_INSTANT, ExecutorId.B1_SLOW_RETARGET) and req.do_shape_phase:
            from .executors.base import base_result

            result = base_result(
                req,
                profile,
                q_start=np.asarray(req.q_start or profile.open_hand_q()),
                q_goal=q_ret,
                q_final=q_cur,
                steps=prefix_steps,
                ok=True,
                message=(
                    f"{eid.value} open-loop MANO shape "
                    f"(settle={req.shape_settle_s:.1f}s)"
                ),
            )
            result.extras.update(extras)
        else:
            req.q_start = [float(x) for x in q_cur.reshape(-1)]
            executor = get_executor(eid)
            result = executor.run(
                req,
                profile=profile,
                contact=contact,
                send_hand=_send,
                wrist_press=_wrist_press
                if eid == ExecutorId.B5_GATED_RAMP_WRIST
                else None,
                object_xyz=_object_xyz if req.object_frame else None,
                wrist_xyz=_wrist_xyz,
            )
            if prefix_steps:
                merged = list(prefix_steps)
                for s in result.steps:
                    s.step = len(merged)
                    merged.append(s)
                result.steps = merged
                result.message = f"shape → {result.message}"
            result.extras.update(extras)
            if obj0 is not None:
                obj_f = _object_xyz()
                result.extras["object_xyz_final"] = obj_f
                if obj0 and obj_f:
                    d = float(
                        np.linalg.norm(
                            np.asarray(obj_f) - np.asarray(obj0)
                        )
                    )
                    result.extras["object_push_m_total"] = d
    finally:
        contact.stop()

    if log and workspace is not None:
        log_dir = getattr(cfg, "close_log_dir", "") or ""
        if log_dir:
            out = Path(log_dir)
            if not out.is_absolute():
                out = Path(workspace) / out
        else:
            out = cfg.resolved_capture_dir(Path(workspace)) / "close_logs"
        write_close_log(result, out, trial_id=trial_id or "")
        table = out / "ablation_table.csv"
        append_table_row(table, result)

    return result


def ensure_q_retarget(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    proposal_path: Path,
    arm: str,
    cached: Optional[Sequence[float]] = None,
    force_recompute: bool = False,
) -> list[float]:
    """Compute (or reuse) retarget q from proposal."""
    if cached is not None and len(cached) > 0 and not force_recompute:
        return [float(x) for x in cached]
    from ..retarget.pipeline import run_retarget_pipeline

    code, out = run_retarget_pipeline(
        cfg=cfg,
        workspace=workspace,
        proposal_path=proposal_path,
        arm=arm,
        execute=False,
        reset=True,
        squeeze_amount=0.0,
        interface=None,
        owns_interface=False,
    )
    if code != 0:
        raise RuntimeError(out.get("error", str(out)))
    return list(out["q_retarget"])
