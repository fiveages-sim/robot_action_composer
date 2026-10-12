"""HUG proposal ↔ base-frame wrist pose ↔ ``ROS2RobotInterface`` arm command."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
from geometry_msgs.msg import Pose

from .config import GraspGenerationConfig
from .frames import CamFrameResolution, resolve_T_base_cam
from .se3 import T_from_transform_stamped, T_mul, pose_from_T



@dataclass
class WristPlan:
    """Camera → base wrist plan ready for arm IK (Phase 2A / 2A.1)."""

    T_camera_wrist: np.ndarray
    T_base_cam: np.ndarray
    T_base_wrist: np.ndarray  # HUG wrist in base (desired hug_tip pose)
    T_base_cmd: np.ndarray  # command pose for control_tip (OCS2)
    wrist_pose: Pose  # stamped command (from T_base_cmd)
    motion_frame_id: str
    arm: str
    cam_resolution: CamFrameResolution
    position_only: bool
    hug_tip_frame: str = ""
    control_tip_frame: str = ""
    tip_offset_xyz: tuple[float, float, float] | None = None
    mano_to_tip_rpy_deg: tuple[float, float, float] | None = None
    click_uv_224: tuple[float, float] | None = None
    depth_at_click_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "T_camera_wrist": np.asarray(self.T_camera_wrist).tolist(),
            "T_base_cam": np.asarray(self.T_base_cam).tolist(),
            "T_base_wrist": np.asarray(self.T_base_wrist).tolist(),
            "T_base_cmd": np.asarray(self.T_base_cmd).tolist(),
            "motion_frame_id": self.motion_frame_id,
            "arm": self.arm,
            "position_only": self.position_only,
            "hug_tip_frame": self.hug_tip_frame,
            "control_tip_frame": self.control_tip_frame,
            "tip_offset_xyz": list(self.tip_offset_xyz) if self.tip_offset_xyz else None,
            "mano_to_tip_rpy_deg": list(self.mano_to_tip_rpy_deg)
            if self.mano_to_tip_rpy_deg
            else None,
            "cam_method": self.cam_resolution.method,
            "camera_frame": self.cam_resolution.camera_frame,
            "hug_wrist_xyz": [
                float(self.T_base_wrist[0, 3]),
                float(self.T_base_wrist[1, 3]),
                float(self.T_base_wrist[2, 3]),
            ],
            "wrist_xyz": [
                self.wrist_pose.position.x,
                self.wrist_pose.position.y,
                self.wrist_pose.position.z,
            ],
            "wrist_quat_xyzw": [
                self.wrist_pose.orientation.x,
                self.wrist_pose.orientation.y,
                self.wrist_pose.orientation.z,
                self.wrist_pose.orientation.w,
            ],
            "click_uv_224": list(self.click_uv_224) if self.click_uv_224 else None,
            "depth_at_click_m": self.depth_at_click_m,
        }


def load_proposal_T_camera_wrist(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    T = np.asarray(data["T_camera_wrist"], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T_camera_wrist shape {T.shape}, expected (4,4)")
    return T, data


def _lookup_T(interface: Any, target: str, source: str, timeout: float) -> np.ndarray:
    ts = interface.lookup_transform(target, source, timeout=timeout)
    if ts is None:
        raise RuntimeError(
            f"TF lookup failed: {target} <- {source}. "
            f"Confirm: ros2 run tf2_ros tf2_echo {target} {source}"
        )
    return T_from_transform_stamped(ts)


def map_hug_wrist_to_control_tip(
    T_base_hug_wrist: np.ndarray,
    *,
    interface: Any,
    base_frame: str,
    hug_tip_frame: str,
    control_tip_frame: str,
    position_only: bool,
    current_ee_pose: Optional[Pose],
    tf_timeout: float,
    T_mano_tip: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, Pose, tuple[float, float, float] | None]:
    """Place ``hug_tip`` at HUG wrist; return command SE3/Pose for ``control_tip``.

    Uses **live TF** ``base←hug_tip`` and ``base←control_tip`` so env-specific TCP
    orientation / hand mount (O7 on tcp, etc.) is baked into the offset.

    ``T_mano_tip``: fixed frame correction (tcp pose expressed in MANO wrist frame).
    ``T_base_tip = T_base_mano @ T_mano_tip`` when following orientation.
    """
    T_hug = np.asarray(T_base_hug_wrist, dtype=np.float64)
    hug_tip = (hug_tip_frame or "").strip()
    ctrl_tip = (control_tip_frame or "").strip()
    T_corr = np.eye(4, dtype=np.float64) if T_mano_tip is None else np.asarray(T_mano_tip, dtype=np.float64)

    if not hug_tip or hug_tip == ctrl_tip:
        T_aligned = T_mul(T_hug, T_corr)
        pose = pose_from_T(T_aligned)
        if position_only and current_ee_pose is not None:
            pose.orientation = current_ee_pose.orientation
        T_cmd = T_aligned.copy()
        if position_only and current_ee_pose is not None:
            from .se3 import T_from_pose

            T_cur = T_from_pose(current_ee_pose)
            T_cmd[:3, :3] = T_cur[:3, :3]
            T_cmd[:3, 3] = T_hug[:3, 3]
        return T_cmd, pose, None

    T_base_tip = _lookup_T(interface, base_frame, hug_tip, tf_timeout)
    T_base_ctrl = _lookup_T(interface, base_frame, ctrl_tip, tf_timeout)
    offset = (T_base_ctrl[:3, 3] - T_base_tip[:3, 3]).astype(np.float64)
    tip_offset_xyz = (float(offset[0]), float(offset[1]), float(offset[2]))

    if position_only:
        p_cmd = T_hug[:3, 3] + offset
        from .se3 import T_from_pose, T_identity

        if current_ee_pose is not None:
            T_cmd = T_from_pose(current_ee_pose)
        else:
            T_cmd = T_identity()
            T_cmd[:3, :3] = T_base_ctrl[:3, :3]
        T_cmd = T_cmd.copy()
        T_cmd[:3, 3] = p_cmd
        return T_cmd, pose_from_T(T_cmd), tip_offset_xyz

    # Full SE3: T_base_tcp = T_base_mano @ T_mano_tip ; then @ T_tcp_eef
    T_tip_ctrl = _lookup_T(interface, hug_tip, ctrl_tip, tf_timeout)
    T_cmd = T_mul(T_mul(T_hug, T_corr), T_tip_ctrl)
    return T_cmd, pose_from_T(T_cmd), tip_offset_xyz


def plan_wrist_from_proposal(
    T_camera_wrist: np.ndarray,
    *,
    interface: Any,
    cfg: GraspGenerationConfig,
    current_ee_pose: Optional[Pose] = None,
    click_uv_224: tuple[float, float] | None = None,
    depth_at_click_m: float | None = None,
) -> WristPlan:
    """``T_base_wrist = T_base_cam @ T_camera_wrist``, then tip remap to OCS2 EE."""
    cam = resolve_T_base_cam(
        interface,
        base_frame=cfg.base_frame,
        camera_frames=cfg.camera_frames,
        camera_prim=cfg.camera_prim,
        base_prim=cfg.base_prim,
        tf_timeout=cfg.tf_timeout,
        prefer="auto",
    )
    T_base_wrist = T_mul(cam.T_base_cam, np.asarray(T_camera_wrist, dtype=np.float64))

    if current_ee_pose is None and interface is not None:
        handler = _arm_handler(interface, cfg.arm)
        current_ee_pose = handler.get_pose() if handler is not None else None
        # get_pose is usually in base_footprint; stamp uses motion_frame_id.
        # Orientation-only reuse is OK across parallel frames for position_only.

    hug_tip = cfg.resolved_hug_tip_frame()
    ctrl_tip = cfg.resolved_control_tip_frame()
    T_mano_tip = cfg.mano_to_tip_T()
    rpy = list(cfg.mano_to_tip_rpy_deg) + [0.0, 0.0, 0.0]
    T_base_cmd, pose, tip_offset = map_hug_wrist_to_control_tip(
        T_base_wrist,
        interface=interface,
        base_frame=cfg.base_frame,
        hug_tip_frame=hug_tip,
        control_tip_frame=ctrl_tip,
        position_only=bool(cfg.position_only),
        current_ee_pose=current_ee_pose,
        tf_timeout=cfg.tf_timeout,
        T_mano_tip=T_mano_tip,
    )

    return WristPlan(
        T_camera_wrist=np.asarray(T_camera_wrist, dtype=np.float64),
        T_base_cam=cam.T_base_cam,
        T_base_wrist=T_base_wrist,
        T_base_cmd=T_base_cmd,
        wrist_pose=pose,
        motion_frame_id=cfg.motion_frame_id,
        arm=cfg.arm,
        cam_resolution=cam,
        position_only=bool(cfg.position_only),
        hug_tip_frame=hug_tip,
        control_tip_frame=ctrl_tip,
        tip_offset_xyz=tip_offset,
        mano_to_tip_rpy_deg=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        click_uv_224=click_uv_224,
        depth_at_click_m=depth_at_click_m,
    )


def send_wrist_plan(
    interface: Any,
    plan: WristPlan,
    *,
    execute: bool,
    arrival_timeout: float = 8.0,
) -> dict[str, Any]:
    """Dry-run or ``send_target_stamped`` + ``wait_until_arrive``.

    Dry-run does **not** require a live arm handler (plan-only).
    """
    info: dict[str, Any] = {
        "execute": execute,
        "arm": plan.arm,
        "frame_id": plan.motion_frame_id,
        "hug_tip_frame": plan.hug_tip_frame,
        "control_tip_frame": plan.control_tip_frame,
        "hug_wrist_xyz": [
            float(plan.T_base_wrist[0, 3]),
            float(plan.T_base_wrist[1, 3]),
            float(plan.T_base_wrist[2, 3]),
        ],
        "target_xyz": [
            plan.wrist_pose.position.x,
            plan.wrist_pose.position.y,
            plan.wrist_pose.position.z,
        ],
        "tip_offset_xyz": list(plan.tip_offset_xyz) if plan.tip_offset_xyz else None,
    }
    if not execute:
        info["status"] = "dry_run"
        return info

    from .home import require_arm_handler

    handler = require_arm_handler(interface, plan.arm)
    # Best-effort: ensure OCS2 before cartesian (after STOP/PLAY / FSM HOME).
    try:
        from ros2_robot_interface import FSM_OCS2

        interface.send_fsm_command(FSM_OCS2)
    except Exception as exc:
        print(f"[bridge] WARN could not send FSM_OCS2 before wrist: {exc}")
    handler.send_target_stamped(plan.motion_frame_id, plan.wrist_pose)
    part = "right_arm" if plan.arm == "right" else "left_arm"
    result = interface.wait_until_arrive(part=part, timeout=arrival_timeout)
    info["status"] = "sent"
    info["arrival"] = result
    return info


def _arm_handler(interface: Any, arm: str) -> Any:
    if arm == "right":
        return getattr(interface, "right_arm_handler", None)
    if arm == "left":
        return getattr(interface, "left_arm_handler", None)
    raise ValueError(f"arm must be left|right, got {arm!r}")


def dump_plan(plan: WristPlan, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(plan.to_dict(), indent=2) + "\n", encoding="utf-8")
