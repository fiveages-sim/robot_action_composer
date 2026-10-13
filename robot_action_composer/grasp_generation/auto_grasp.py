"""Privileged auto-grasp orchestration (Viser + CLI).

Does not replace the manual click path. Object select → UV / HUG / wrist /
close are sequenced here so serve UI and ``grasp-generation auto`` share one
code path.

Scene clear / exile / near-spawn were removed: objects stay on the authored
table layout; only privileged UV selection uses ``grasp_objects``.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from .bridge import (
    dump_plan,
    load_proposal_T_camera_wrist,
    plan_wrist_from_proposal,
    send_wrist_plan,
)
from .config import GraspGenerationConfig
from .frames import resolve_T_base_cam
from .hug_subprocess import run_hug_propose
from .objects import get_grasp_object
from .privileged_pose import resolve_object_pose_in_base, transform_base_point_to_cam
from .project_uv import UvProjection, project_privileged_to_uv
from .rgbd_io import RgbdFrame, capture_head_camera_ros, save_frame_to_dir


StatusFn = Callable[[str], None]


@dataclass
class AutoGraspResult:
    ok: bool
    object_key: str
    u_norm: float = 0.0
    v_norm: float = 0.0
    pose_method: str = ""
    proposal_path: Optional[Path] = None
    plan_path: Optional[Path] = None
    message: str = ""
    extras: dict[str, Any] = field(default_factory=dict)


def _status(log: Optional[StatusFn], msg: str) -> None:
    if log is not None:
        log(msg)
    else:
        print(f"[auto_grasp] {msg}")


def capture_frame(
    cfg: GraspGenerationConfig,
    workspace: Path,
) -> tuple[RgbdFrame, Path]:
    capture_dir = cfg.resolved_capture_dir(workspace)
    capture_dir.mkdir(parents=True, exist_ok=True)
    frame = capture_head_camera_ros(
        rgb_topic=cfg.rgb_topic,
        depth_topic=cfg.depth_topic,
        info_topic=cfg.info_topic,
        timeout_sec=30.0,
        out_dir=capture_dir,
    )
    save_frame_to_dir(frame, capture_dir, stem="w2_head")
    return frame, capture_dir


def privileged_uv_for_object(
    cfg: GraspGenerationConfig,
    *,
    object_key: str,
    frame: RgbdFrame,
    interface: Any,
) -> tuple[UvProjection, str]:
    from .privileged_pose import apply_local_offset_world, world_point_to_base

    spec = get_grasp_object(cfg, object_key)
    wanted = (spec.click_prim or "").strip()
    click_path = wanted or spec.pose_prim
    try:
        pose = resolve_object_pose_in_base(
            click_path,
            base_prim=cfg.base_prim,
            interface=interface,
            base_frame=cfg.base_frame,
            tf_timeout=cfg.tf_timeout,
        )
    except Exception as exc:
        if wanted and wanted != spec.pose_prim:
            print(
                f"[uv] WARN click_prim={wanted!r} failed ({exc}); "
                f"falling back to pose_prim + click_offset"
            )
            click_path = spec.pose_prim
            pose = resolve_object_pose_in_base(
                click_path,
                base_prim=cfg.base_prim,
                interface=interface,
                base_frame=cfg.base_frame,
                tf_timeout=cfg.tf_timeout,
            )
        else:
            raise
    off = spec.click_offset or (0.0, 0.0, 0.0)
    method = pose.method
    if click_path != spec.pose_prim:
        method = f"{pose.method}+click_prim"
        print(f"[uv] click_prim={click_path}")
    if any(abs(float(v)) > 1e-9 for v in off):
        click_w = apply_local_offset_world(
            pose.xyz_world, pose.quat_xyzw_world, off
        )
        xyz_base = world_point_to_base(click_w, base_prim=cfg.base_prim)
        frame_lbl = "click_prim" if click_path != spec.pose_prim else "pose_prim"
        print(
            f"[uv] click_offset={list(off)} m ({frame_lbl} local) → "
            f"base={list(xyz_base)}"
        )
    else:
        xyz_base = pose.xyz_base
    cam = resolve_T_base_cam(
        interface,
        base_frame=cfg.base_frame,
        camera_frames=cfg.camera_frames,
        camera_prim=cfg.camera_prim,
        base_prim=cfg.base_prim,
        tf_timeout=cfg.tf_timeout,
        prefer="tf",
    )
    xyz_cam = transform_base_point_to_cam(xyz_base, cam.T_base_cam)
    uv = project_privileged_to_uv(xyz_cam, K=frame.K, depth_m=frame.depth_m)
    return uv, method


def run_auto_grasp(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    object_key: str,
    interface: Any,
    execute: bool = True,
    dry_run: bool = False,
    do_close: bool = True,
    status: Optional[StatusFn] = None,
    frame: Optional[RgbdFrame] = None,
    capture_dir: Optional[Path] = None,
    cancel_event: Any = None,
) -> AutoGraspResult:
    """Privileged pipeline. ``dry_run`` stops after propose (+ dry plan).

    Objects are **not** moved or hidden — table layout stays as authored.
    When ``frame`` is provided (Viser already captured), skip ROS capture.
    ``cancel_event`` aborts between major steps for Go home.
    """
    key = str(object_key).strip()
    if not key:
        return AutoGraspResult(ok=False, object_key="", message="empty object_key")

    def _check_cancel() -> None:
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("cancelled (Go home priority)")

    try:
        get_grasp_object(cfg, key)
    except Exception as exc:
        return AutoGraspResult(ok=False, object_key=key, message=str(exc))

    uv = None
    pose_method = ""
    proposal_path: Optional[Path] = None
    plan_path: Optional[Path] = None
    plan = None
    info: dict[str, Any] = {}
    close_summary: dict[str, Any] = {}

    def _partial(*, ok: bool, message: str) -> AutoGraspResult:
        extras: dict[str, Any] = {
            "uv": uv,
            "frame": frame,
            "close": close_summary,
            "move": info,
        }
        return AutoGraspResult(
            ok=ok,
            object_key=key,
            u_norm=float(uv.u_norm) if uv is not None else 0.0,
            v_norm=float(uv.v_norm) if uv is not None else 0.0,
            pose_method=pose_method,
            proposal_path=proposal_path,
            plan_path=plan_path,
            message=message,
            extras=extras,
        )

    try:
        _check_cancel()
        if frame is None:
            _status(status, "Capturing head RGB-D ...")
            frame, capture_dir = capture_frame(cfg, workspace)
        elif capture_dir is None:
            capture_dir = cfg.resolved_capture_dir(workspace)

        assert capture_dir is not None
        _check_cancel()
        _status(status, f"Privileged UV for `{key}` ...")
        uv, pose_method = privileged_uv_for_object(
            cfg, object_key=key, frame=frame, interface=interface
        )
        _status(
            status,
            f"UV=({uv.u_norm:.3f},{uv.v_norm:.3f}) "
            f"surf_z={uv.depth_m:.3f}m center_z={uv.z_cam_privileged:.3f}m "
            f"Δ={getattr(uv, 'surface_delta_m', uv.depth_m - uv.z_cam_privileged):+.3f}m "
            f"pose={pose_method} refined={uv.refined}",
        )

        _check_cancel()
        hug_py = cfg.resolved_hug_python(workspace)
        composer_root = Path(__file__).resolve().parents[2]
        proposal_path = capture_dir / "proposal.json"
        _status(status, "HUG propose ...")
        run_hug_propose(
            hug_python=hug_py,
            capture_dir=capture_dir,
            out_json=proposal_path,
            u_norm=uv.u_norm,
            v_norm=uv.v_norm,
            sampling_steps=cfg.sampling_steps,
            composer_root=composer_root,
        )

        _check_cancel()
        T, meta = load_proposal_T_camera_wrist(proposal_path)
        plan = plan_wrist_from_proposal(
            T,
            interface=interface,
            cfg=cfg,
            click_uv_224=tuple(meta.get("click_uv_224") or ()),
            depth_at_click_m=meta.get("depth_at_click_m"),
        )
        plan_path = capture_dir / "wrist_plan.json"
        dump_plan(plan, plan_path)

        if dry_run:
            send_wrist_plan(
                interface, plan, execute=False, arrival_timeout=cfg.arrival_timeout
            )
            return _partial(
                ok=True, message="dry_run OK (propose + dry plan; no move/close)"
            )

        if execute:
            from .home import require_arm_handler

            require_arm_handler(interface, cfg.arm or "right")
            # Open fingers before approach (leftover curl looks like fake close-open).
            from .retarget.pipeline import run_open_hand

            arm_open = (cfg.arm or "right").lower()
            if arm_open in ("left", "right"):
                _status(status, f"Open {arm_open} hand before wrist approach ...")
                run_open_hand(arm=arm_open, interface=interface)  # type: ignore[arg-type]

        _check_cancel()
        _status(status, f"{'EXECUTE' if execute else 'DRY'} move wrist ...")
        info = send_wrist_plan(
            interface,
            plan,
            execute=bool(execute),
            arrival_timeout=cfg.arrival_timeout,
        )

        if do_close and execute:
            from .execution import ensure_q_retarget, run_close

            _check_cancel()
            arm = (cfg.arm or "right").lower()
            eid = str(getattr(cfg, "close_executor", "B4") or "B4").upper()
            _status(status, f"Close {eid} (retarget → shape → grasp) ...")
            q_ret = ensure_q_retarget(
                cfg=cfg,
                workspace=workspace,
                proposal_path=proposal_path,
                arm=arm,
                force_recompute=True,
            )
            _check_cancel()
            result = run_close(
                cfg=cfg,
                interface=interface,
                q_retarget=q_ret,
                arm=arm,
                squeeze_amount=float(cfg.squeeze_amount),
                executor_id=eid,
                last_plan=plan,
                workspace=workspace,
                trial_id=f"auto_{key}",
                log=True,
                do_shape_phase=bool(getattr(cfg, "auto_retarget_on_close", True)),
            )
            close_summary = (
                result.summary_row()
                if hasattr(result, "summary_row")
                else {"ok": result.ok, "message": result.message}
            )
            if not result.ok:
                return _partial(ok=False, message=f"close failed: {result.message}")

            if bool(getattr(cfg, "post_grasp_lift", True)):
                from .home import lift_ee_world_z

                settle = float(getattr(cfg, "post_grasp_settle_s", 1.5))
                if settle > 0:
                    _status(
                        status,
                        f"Waiting {settle:.1f}s for fingers to settle before lift ...",
                    )
                    remaining = settle
                    while remaining > 1e-6:
                        _check_cancel()
                        step = min(0.2, remaining)
                        time.sleep(step)
                        remaining -= step
                lift_m = float(getattr(cfg, "post_grasp_lift_m", 0.20))
                _check_cancel()
                _status(status, f"Post-grasp lift +Z {lift_m:.2f}m (verify grasp) ...")
                lift_info = lift_ee_world_z(
                    interface,
                    arm=arm,
                    lift_m=lift_m,
                    motion_frame_id=cfg.motion_frame_id,
                    arrival_timeout=float(cfg.arrival_timeout),
                    last_plan=plan,
                )
                close_summary["post_grasp_lift"] = lift_info
                close_summary["post_grasp_settle_s"] = settle

        return _partial(ok=True, message="auto grasp OK")
    except Exception as exc:
        return _partial(ok=False, message=str(exc))


def run_auto_grasp_cli(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    object_key: str,
    dry_run: bool,
    execute: bool,
    do_close: bool,
) -> int:
    from .interface_util import connect_grasp_interface, disconnect_grasp_interface

    interface = connect_grasp_interface(robot_key=str(cfg.robot_key), settle_s=0.3)
    try:
        result = run_auto_grasp(
            cfg=cfg,
            workspace=workspace,
            object_key=object_key,
            interface=interface,
            execute=execute and not dry_run,
            dry_run=dry_run,
            do_close=do_close and not dry_run,
        )
        payload = {
            "ok": result.ok,
            "object_key": result.object_key,
            "u_norm": result.u_norm,
            "v_norm": result.v_norm,
            "pose_method": result.pose_method,
            "proposal": str(result.proposal_path) if result.proposal_path else None,
            "message": result.message,
        }
        print(json.dumps(payload, indent=2))
        return 0 if result.ok else 1
    finally:
        disconnect_grasp_interface(interface)
