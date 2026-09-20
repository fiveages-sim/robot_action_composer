"""Shared retarget + send helpers (CLI and Viser serve)."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Literal, Optional, Sequence

import numpy as np

from ..config import GraspGenerationConfig
from .keypoints import load_landmarks_from_proposal, preprocess_keypoints
from .squeeze import apply_squeeze, open_hand_q, squeeze_delta_named
from .wuji_hand2 import RetargetResult, Side, WujiHand2Retarget

Arm = Literal["left", "right"]


def resolve_retarget_model_side(
    arm: Arm,
    *,
    hug_mano_is_right: bool = True,
) -> Side:
    """HUG proposals are right-MANO; keep Right session for quality on both arms."""
    if hug_mano_is_right:
        return "right"
    return arm


def retarget_proposal(
    proposal_path: Path,
    *,
    arm: Arm = "right",
    keypoint_rpy_deg: Sequence[float] | None = None,
    reset: bool = True,
    hug_mano_is_right: bool = True,
    retargeter: WujiHand2Retarget | None = None,
) -> tuple[RetargetResult, np.ndarray]:
    """Load proposal → retarget. Returns (result, preprocessed_kp)."""
    landmarks = load_landmarks_from_proposal(proposal_path)
    kp = preprocess_keypoints(landmarks, keypoint_rpy_deg=keypoint_rpy_deg)
    model_side = resolve_retarget_model_side(arm, hug_mano_is_right=hug_mano_is_right)
    eng = retargeter or WujiHand2Retarget(side=model_side)
    result = eng.step(kp, reset=reset)
    return (
        RetargetResult(q=result.q, side=arm, backend=result.backend),
        kp,
    )


def send_hand_q(interface: Any, arm: Arm, q: Sequence[float]) -> None:
    q_list = [float(x) for x in q]
    if arm == "right":
        interface.send_right_hand_joint_positions(q_list)
    elif arm == "left":
        interface.send_left_hand_joint_positions(q_list)
    else:
        raise ValueError(f"arm must be left|right, got {arm!r}")


def run_retarget_pipeline(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    proposal_path: Optional[Path] = None,
    arm: Optional[str] = None,
    execute: bool = False,
    reset: bool = True,
    squeeze_amount: Optional[float] = None,
    interface: Any = None,
    owns_interface: bool = False,
) -> tuple[int, dict[str, Any]]:
    """Retarget (+ optional amplify-squeeze) and optionally send."""
    if cfg.hand_profile not in ("wuji_hand2", "wuji-hand2"):
        return 2, {"error": f"hand_profile={cfg.hand_profile!r} not supported"}

    side: Arm = (arm or cfg.arm or "right").lower()  # type: ignore[assignment]
    if side not in ("left", "right"):
        return 2, {"error": f"bad arm {side!r}"}

    prop = proposal_path
    if prop is None:
        cand = cfg.resolved_capture_dir(workspace) / "proposal.json"
        if cand.is_file():
            prop = cand
    if prop is None or not Path(prop).is_file():
        return 1, {"error": "need proposal.json"}

    prop = Path(prop)
    hug_right = bool(getattr(cfg, "hug_mano_is_right", True))
    result, kp = retarget_proposal(
        prop,
        arm=side,
        keypoint_rpy_deg=cfg.keypoint_rpy_deg,
        reset=reset,
        hug_mano_is_right=hug_right,
    )
    q_retarget = result.as_list()
    sq = float(
        cfg.squeeze_amount if squeeze_amount is None else squeeze_amount
    )
    thumb_boost = float(getattr(cfg, "thumb_abd_boost", 1.25))

    if sq > 1e-9:
        q_final = apply_squeeze(
            result.q, amount=sq, thumb_abd_boost=thumb_boost
        )
    else:
        q_final = np.asarray(result.q)

    deltas = squeeze_delta_named(result.q, q_final) if sq > 1e-9 else []

    out: dict[str, Any] = {
        "proposal": str(prop),
        "hand_profile": cfg.hand_profile,
        "arm": side,
        "model_side": resolve_retarget_model_side(side, hug_mano_is_right=hug_right),
        "hug_mano_is_right": hug_right,
        "backend": result.backend,
        "keypoint_rpy_deg": list(cfg.keypoint_rpy_deg),
        "keypoints_wrist_span_m": (kp.max(0) - kp.min(0)).tolist(),
        "q_retarget": q_retarget,
        "squeeze_amount": sq,
        "squeeze_mode": "amplify_from_retarget",
        "thumb_abd_boost": thumb_boost,
        "q": [float(x) for x in np.asarray(q_final).reshape(-1)],
        "q_named": {
            n: float(v)
            for n, v in zip(result.joint_suffixes, np.asarray(q_final).reshape(-1))
        },
        "squeeze_deltas": {n: d for n, d in deltas},
        "q_absmax": float(np.max(np.abs(q_final))),
        "execute": bool(execute),
    }

    out_path = cfg.resolved_capture_dir(workspace) / "hand_retarget.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    out["out_path"] = str(out_path)

    if not execute:
        return 0, out

    created = False
    iface = interface
    if iface is None:
        from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

        iface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
        iface.connect()
        time.sleep(0.3)
        created = True
        owns_interface = True

    try:
        send_hand_q(iface, side, out["q"])
        out["sent"] = True
        time.sleep(0.35)
        return 0, out
    finally:
        if owns_interface and created:
            try:
                iface.disconnect()
            except Exception:
                pass


def run_open_hand(
    *,
    arm: Arm,
    interface: Any,
    q: Sequence[float] | None = None,
) -> list[float]:
    """Send open/rest finger pose (re-grasp). Wrist untouched."""
    pose = open_hand_q(None if q is None else np.asarray(q, dtype=np.float32))
    q_list = [float(x) for x in pose.reshape(-1)]
    send_hand_q(interface, arm, q_list)
    time.sleep(0.25)
    return q_list
