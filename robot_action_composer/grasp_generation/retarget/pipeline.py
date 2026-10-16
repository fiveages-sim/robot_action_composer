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
from .wuji_hand2 import (
    WUJI_HAND2_JOINT_SUFFIXES,
    RetargetResult,
    Side,
    WujiHand2Retarget,
)

Arm = Literal["left", "right"]

# ``right_hand_controller`` MOVEJ duration is 0.5s
# (fa_w2_ws config/robot.profiles/wuji_hand2.yaml). A newer
# ``target_joint_position`` while that move is still running starts
# stop-to-zero, which rewinds the fingers toward open. Shape / B4 / manual
# retarget all publish through ``send_hand_q``, so the gap lives here.
_HAND_MOVEJ_GAP_S = 0.55
_last_hand_send_mono: dict[str, float] = {}


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
    """Publish one hand target, at most once per MOVEJ duration.

    Intermediate shape/close samples are dropped so the controller can finish
    the interpolation it already started. The latest ``q`` still goes out on
    the next allowed tick (hold loops cover the final pose).
    """
    now = time.monotonic()
    last = _last_hand_send_mono.get(arm, 0.0)
    if now - last < _HAND_MOVEJ_GAP_S:
        return
    q_list = [float(x) for x in q]
    if arm == "right":
        interface.send_right_hand_joint_positions(q_list)
    elif arm == "left":
        interface.send_left_hand_joint_positions(q_list)
    else:
        raise ValueError(f"arm must be left|right, got {arm!r}")
    _last_hand_send_mono[arm] = now


def read_hand_q(
    interface: Any,
    arm: Arm,
    *,
    suffixes: Sequence[str] = WUJI_HAND2_JOINT_SUFFIXES,
) -> list[float] | None:
    """Read current dex-hand joint positions from ``/joint_states`` (suffix order).

    Returns ``None`` if joint state is missing or incomplete. Category keys follow
    ``ROS2RobotInterface`` dual-arm naming (``left_gripper`` / ``right_gripper``
    also cover ``*_hand_*`` joints).
    """
    side: Arm = arm  # type: ignore[assignment]
    if side not in ("left", "right"):
        raise ValueError(f"arm must be left|right, got {arm!r}")
    getter = getattr(interface, "get_joint_state", None)
    if not callable(getter):
        return None
    try:
        cats = getter(categorized=True)
    except Exception:
        return None
    if not isinstance(cats, dict):
        return None
    key = "right_gripper" if side == "right" else "left_gripper"
    block = cats.get(key) or cats.get("gripper")
    if not isinstance(block, dict):
        return None
    names = [str(n) for n in (block.get("names") or [])]
    positions = list(block.get("positions") or [])
    if not names or len(names) != len(positions):
        return None
    by_suffix: dict[str, float] = {}
    for name, pos in zip(names, positions):
        low = name.lower()
        for suf in suffixes:
            if low.endswith(suf) or low.endswith(f"hand_{suf}"):
                by_suffix[suf] = float(pos)
                break
    if len(by_suffix) < len(suffixes):
        return None
    return [by_suffix[s] for s in suffixes]


def _hand_gripper_handler(interface: Any, arm: Arm) -> Any | None:
    if arm == "right":
        return getattr(interface, "right_gripper_handler", None)
    if arm == "left":
        return getattr(interface, "left_gripper_handler", None)
    return None


def hold_hand_current(
    interface: Any,
    arm: Arm,
    *,
    settle_s: float = 0.05,
) -> list[float] | None:
    """Re-command measured finger pose so hand MOVEJ does not replay a stale target.

    **运控耦合：** ``/fsm_command`` 全局共享。臂切 ``FSM_OCS2``(3) / ``MOVEJ``(4) 时，
    ``left/right_hand_controller`` 也进 MOVEJ；非 MOVEJ 时 ``target_joint_position``
    / ``target_command`` 会被丢弃。期望流程是：OCS2 后先 ``ensure_hand_open``，
    再 hold 当前（全开）姿态跟腕，直到 close/shape 才发关节轨迹。

    Returns the held ``q`` or ``None`` if joint state was unavailable.
    """
    q = read_hand_q(interface, arm)
    if q is None:
        return None
    send_hand_q(interface, arm, q)
    if settle_s > 0:
        time.sleep(float(settle_s))
    return q


def ensure_hand_open(
    interface: Any,
    arm: Arm,
    *,
    settle_s: float = 0.5,
) -> bool:
    """Fully open one hand via 运控 ``target_command(1)`` (RViz Open Hand).

    Requires the hand controller already in **MOVEJ** (after FSM OCS2/MOVEJ).
    ``target_command`` open index must map to ``home_1`` (zeros) — set via local
    ``robot.profiles/wuji_hand2.yaml`` control.patch, not by editing wuji package.

    Falls back to joint-space zeros only if no gripper handler is available.
    """
    side: Arm = arm  # type: ignore[assignment]
    if side not in ("left", "right"):
        raise ValueError(f"arm must be left|right, got {arm!r}")

    handler = _hand_gripper_handler(interface, side)
    if handler is not None and hasattr(handler, "send_target_command"):
        try:
            handler.send_target_command(1)  # 1 = open → open_config (home_1)
            if settle_s > 0:
                time.sleep(float(settle_s))
            # Lock measured open pose so a later MOVEJ re-enter cannot replay mid.
            hold_hand_current(interface, side, settle_s=min(0.1, float(settle_s) * 0.25))
            return True
        except Exception as exc:
            print(f"[hand] WARN target_command open {side}: {exc}")

    try:
        run_open_hand(arm=side, interface=interface)
        return True
    except Exception as exc:
        print(f"[hand] WARN joint-space open {side}: {exc}")
        return False


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
