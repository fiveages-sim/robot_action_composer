"""Runtime TF / EE diagnostics for HUG wrist ↔ robot tip alignment (Phase 2A.1)."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from .config import GraspGenerationConfig
from .frames import lookup_T_base_cam_entity, lookup_T_base_cam_tf
from .se3 import T_from_pose, T_from_transform_stamped


def _xyz(T: np.ndarray) -> np.ndarray:
    return np.asarray(T[:3, 3], dtype=np.float64)


def _rot_angle_deg(A: np.ndarray, B: np.ndarray) -> float:
    R = A[:3, :3].T @ B[:3, :3]
    c = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _try_lookup(interface: Any, target: str, source: str, timeout: float) -> Optional[np.ndarray]:
    ts = interface.lookup_transform(target, source, timeout=timeout)
    if ts is None:
        return None
    return T_from_transform_stamped(ts)


def _fmt_xyz(v: Sequence[float]) -> str:
    return f"({float(v[0]):+.4f}, {float(v[1]):+.4f}, {float(v[2]):+.4f})"


def run_diagnose_frames(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    plan_path: Optional[Path] = None,
    tip_frames: Optional[Sequence[str]] = None,
) -> int:
    """Print camera / tip TF table and optional Δ vs wrist_plan (needs ros2-stack)."""
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

    arm = cfg.arm
    tips = list(
        tip_frames
        or (
            f"{arm}_link7",
            f"{arm}_tcp",
            f"{arm}_hand_hand_base",
            f"{arm}_hand_eef",
            f"{arm}_eef",
        )
    )
    base = cfg.base_frame
    timeout = float(cfg.tf_timeout)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    print("[diagnose] connecting ROS2RobotInterface ...")
    interface.connect()
    time.sleep(0.4)

    report: dict[str, Any] = {
        "base_frame": base,
        "arm": arm,
        "motion_frame_id": cfg.motion_frame_id,
        "camera_frames_cfg": list(cfg.camera_frames),
    }

    try:
        print("\n=== Camera TF vs base ===")
        cam_ok: dict[str, list[float]] = {}
        for cam in cfg.camera_frames:
            T = _try_lookup(interface, base, cam, timeout)
            if T is None:
                print(f"  FAIL  {base} <- {cam}")
                continue
            t = _xyz(T).tolist()
            cam_ok[cam] = t
            print(f"  OK    {base} <- {cam}  xyz={_fmt_xyz(t)}")
        report["camera_tf"] = cam_ok

        print("\n=== resolve_T_base_cam (same as plan) ===")
        try:
            res_tf = lookup_T_base_cam_tf(
                interface,
                base_frame=base,
                camera_frames=cfg.camera_frames,
                timeout=timeout,
            )
            print(
                f"  TF winner: {res_tf.camera_frame}  "
                f"xyz={_fmt_xyz(_xyz(res_tf.T_base_cam))}"
            )
            report["plan_camera_frame"] = res_tf.camera_frame
            report["T_base_cam_tf_xyz"] = _xyz(res_tf.T_base_cam).tolist()
        except Exception as exc:
            print(f"  TF resolve failed: {exc}")
            res_tf = None

        try:
            res_ent = lookup_T_base_cam_entity(
                base_prim=cfg.base_prim, camera_prim=cfg.camera_prim
            )
            print(
                f"  entity_state: {res_ent.camera_frame}  "
                f"xyz={_fmt_xyz(_xyz(res_ent.T_base_cam))}"
            )
            report["T_base_cam_entity_xyz"] = _xyz(res_ent.T_base_cam).tolist()
            if res_tf is not None:
                d = _xyz(res_ent.T_base_cam) - _xyz(res_tf.T_base_cam)
                ang = _rot_angle_deg(res_tf.T_base_cam, res_ent.T_base_cam)
                print(f"  Δ(entity−tf) xyz={_fmt_xyz(d)}  |Δ|={np.linalg.norm(d):.4f} m  rot={ang:.2f}°")
                report["delta_entity_minus_tf"] = {
                    "xyz": d.tolist(),
                    "norm_m": float(np.linalg.norm(d)),
                    "rot_deg": ang,
                }
        except Exception as exc:
            print(f"  entity_state failed (ok if Isaac service down): {exc}")

        print("\n=== Right-arm tip frames vs base ===")
        tip_xyz: dict[str, list[float]] = {}
        for tip in tips:
            T = _try_lookup(interface, base, tip, timeout)
            if T is None:
                print(f"  FAIL  {base} <- {tip}")
                continue
            t = _xyz(T).tolist()
            tip_xyz[tip] = t
            print(f"  OK    {base} <- {tip}  xyz={_fmt_xyz(t)}")
        report["tip_tf"] = tip_xyz

        if f"{arm}_tcp" in tip_xyz and f"{arm}_eef" in tip_xyz:
            d_te = np.asarray(tip_xyz[f"{arm}_eef"]) - np.asarray(tip_xyz[f"{arm}_tcp"])
            print(
                f"  note  {arm}_eef − {arm}_tcp = {_fmt_xyz(d_te)}  "
                f"|Δ|={np.linalg.norm(d_te):.4f} m  (expect ~palm offset if O7)"
            )
            report["eef_minus_tcp"] = {
                "xyz": d_te.tolist(),
                "norm_m": float(np.linalg.norm(d_te)),
            }

        print("\n=== Arm handler get_pose ===")
        handler = (
            interface.right_arm_handler if arm == "right" else interface.left_arm_handler
        )
        pose = handler.get_pose() if handler is not None else None
        frame_id = handler.get_frame_id() if handler is not None else None
        print(f"  get_frame_id() = {frame_id!r}")
        if pose is None:
            print("  get_pose() = None")
        else:
            T_pose = T_from_pose(pose)
            print(f"  get_pose() xyz={_fmt_xyz(_xyz(T_pose))}  (raw topic frame)")
            report["get_pose_raw_xyz"] = _xyz(T_pose).tolist()
            report["get_frame_id"] = frame_id
            if frame_id and frame_id != base:
                try:
                    transformed = interface.transform_pose(pose, frame_id, base, timeout=timeout)
                    if transformed is not None:
                        # transform_pose may return Pose or PoseStamped
                        p = getattr(transformed, "pose", transformed)
                        T_b = T_from_pose(p)
                        print(f"  → {base} xyz={_fmt_xyz(_xyz(T_b))}")
                        report["get_pose_in_base_xyz"] = _xyz(T_b).tolist()
                except Exception as exc:
                    print(f"  transform_pose failed: {exc}")

        plan_file = plan_path
        if plan_file is None:
            cand = cfg.resolved_capture_dir(workspace) / "wrist_plan.json"
            if cand.is_file():
                plan_file = cand
        if plan_file is not None and Path(plan_file).is_file():
            print(f"\n=== Compare plan: {plan_file} ===")
            data = json.loads(Path(plan_file).read_text(encoding="utf-8"))
            plan_xyz = np.asarray(data.get("wrist_xyz") or _xyz(np.asarray(data["T_base_wrist"])), dtype=np.float64)
            print(
                f"  plan wrist_xyz ({data.get('motion_frame_id')}) = {_fmt_xyz(plan_xyz)}  "
                f"cam={data.get('cam_method')}/{data.get('camera_frame')}  "
                f"position_only={data.get('position_only')}"
            )
            report["plan_wrist_xyz"] = plan_xyz.tolist()
            deltas = {}
            for tip, t in tip_xyz.items():
                d = plan_xyz - np.asarray(t, dtype=np.float64)
                n = float(np.linalg.norm(d))
                deltas[tip] = {"xyz": d.tolist(), "norm_m": n}
                print(f"  plan − {tip}: Δ={_fmt_xyz(d)}  |Δ|={n:.4f} m")
            if pose is not None and "get_pose_in_base_xyz" in report:
                d = plan_xyz - np.asarray(report["get_pose_in_base_xyz"])
                print(f"  plan − get_pose@{base}: Δ={_fmt_xyz(d)}  |Δ|={np.linalg.norm(d):.4f} m")
                deltas["get_pose_base"] = {"xyz": d.tolist(), "norm_m": float(np.linalg.norm(d))}
            elif pose is not None:
                d = plan_xyz - _xyz(T_from_pose(pose))
                print(f"  plan − get_pose(raw): Δ={_fmt_xyz(d)}  |Δ|={np.linalg.norm(d):.4f} m")
                deltas["get_pose_raw"] = {"xyz": d.tolist(), "norm_m": float(np.linalg.norm(d))}
            report["plan_minus_tips"] = deltas
            # Hint
            if deltas:
                best = min(deltas.items(), key=lambda kv: kv[1]["norm_m"])
                print(
                    "\n  note: plan−tip above = remaining distance to CURRENT tips "
                    "(not proof of which tip HUG means). After tip-map, compare "
                    "hug_wrist_xyz vs right_tcp and cmd_xyz vs right_eef."
                )
                print(
                    f"  closest current tip to plan target: **{best[0]}** "
                    f"({best[1]['norm_m']:.4f} m)."
                )
            if "hug_wrist_xyz" in data:
                hug = np.asarray(data["hug_wrist_xyz"], dtype=np.float64)
                print(f"\n  plan hug_wrist_xyz = {_fmt_xyz(hug)}")
                if f"{arm}_tcp" in tip_xyz:
                    d = hug - np.asarray(tip_xyz[f"{arm}_tcp"])
                    print(f"  hug − {arm}_tcp: Δ={_fmt_xyz(d)}  |Δ|={np.linalg.norm(d):.4f} m")
                print(
                    f"  tip map: {data.get('hug_tip_frame')} → {data.get('control_tip_frame')} "
                    f"offset={data.get('tip_offset_xyz')}"
                )
        else:
            print("\n=== No wrist_plan.json — run Plan once, then re-diagnose ===")

        out = cfg.resolved_capture_dir(workspace) / "frame_diagnose.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"\n[diagnose] wrote {out}")
        print("See docs/SIM_GRASP_EE_ALIGN.md for tip map + orientation axes.")
        return 0
    finally:
        try:
            interface.disconnect()
        except Exception:
            pass


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=np.float64)
    return v / n


def _axis_name(v: np.ndarray) -> str:
    a = np.abs(v)
    i = int(np.argmax(a))
    sign = "+" if v[i] >= 0 else "-"
    return f"{sign}{['X', 'Y', 'Z'][i]} (dom={a[i]:.2f})"


def run_diagnose_orientation(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    proposal_path: Optional[Path] = None,
) -> int:
    """Print MANO wrist-local finger/palm axes vs robot tcp→eef (axis mismatch)."""
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

    hug_tip = cfg.resolved_hug_tip_frame()
    ctrl_tip = cfg.resolved_control_tip_frame()
    base = cfg.base_frame
    timeout = float(cfg.tf_timeout)

    prop = proposal_path
    if prop is None:
        cand = cfg.resolved_capture_dir(workspace) / "proposal.json"
        if cand.is_file():
            prop = cand
    if prop is None or not Path(prop).is_file():
        print("Need proposal.json (Capture + HUG propose first).")
        return 1

    data = json.loads(Path(prop).read_text(encoding="utf-8"))
    T = np.asarray(data["T_camera_wrist"], dtype=np.float64)
    lm = np.asarray(data["landmarks_3d"], dtype=np.float64)
    R, t = T[:3, :3], T[:3, 3]
    loc = (R.T @ (lm - t).T).T

    finger = _unit(loc[12])  # middle tip in MANO wrist
    mid_mcp = loc[9]
    idx_mcp = loc[5]
    palm = _unit(np.cross(mid_mcp, idx_mcp))

    print(f"=== MANO wrist axes (from {prop}) ===")
    print(f"  wrist→mid_tip (finger): {_fmt_xyz(finger)}  ≈ {_axis_name(finger)}")
    print(f"  palm normal mid×idx:    {_fmt_xyz(palm)}  ≈ {_axis_name(palm)}")
    print("  (HUG/manotorch on this sample: finger≈−X, palm≈+Y)")
    print(f"  cfg mano_to_tip_rpy_deg = {list(cfg.mano_to_tip_rpy_deg)}")

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    print("\n[diagnose-orient] connecting ROS2RobotInterface ...")
    interface.connect()
    time.sleep(0.3)
    try:
        print(f"\n=== Robot tip frames ({hug_tip} → {ctrl_tip}) ===")
        T_tip_ctrl = _try_lookup(interface, hug_tip, ctrl_tip, timeout)
        if T_tip_ctrl is None:
            print(f"  FAIL TF {hug_tip} <- {ctrl_tip}")
            return 2
        palm_robot = _unit(_xyz(T_tip_ctrl))
        print(
            f"  {ctrl_tip} origin in {hug_tip}: {_fmt_xyz(_xyz(T_tip_ctrl))}  "
            f"dir≈ {_axis_name(palm_robot)}  ← treat as robot 'palm/out' axis"
        )
        T_base_tip = _try_lookup(interface, base, hug_tip, timeout)
        if T_base_tip is not None:
            print(f"  {hug_tip} axes in {base}:")
            for name, col in zip("XYZ", range(3)):
                ax = T_base_tip[:3, col]
                print(f"    +{name}: {_fmt_xyz(ax)}")

        print("\n=== How to read mismatch ===")
        print(
            "  Goal: palm flips about finger axis; fingertip direction stays stable."
        )
        print(
            f"  Identity rpy=[0,0,0] is wrong when MANO finger≈{_axis_name(finger)} "
            f"but robot out≈{_axis_name(palm_robot)}."
        )
        print(
            "  Try yaml mano_to_tip_rpy_deg: [0, -90, 0] then ±90° steps;\n"
            "  Formula: T_cmd = T_mano @ T_mano_tip(rpy) @ T_tcp_eef"
        )
        return 0
    finally:
        try:
            interface.disconnect()
        except Exception:
            pass
