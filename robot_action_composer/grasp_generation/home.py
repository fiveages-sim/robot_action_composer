"""Grasp go-home: controller FSM_HOME, then optional dual-EE +Z lift.

Matches ros2_robot_interface demos: HOLD → HOME → settle → HOLD → OCS2,
then raises both wrists by ``grasp_home_lift_m`` (grasp workspace home ≠
raw controller home).
"""

from __future__ import annotations

import time
from copy import deepcopy
from typing import Any, Mapping, Optional

from .config import GraspGenerationConfig


class HomeCancelled(RuntimeError):
    """Raised when a higher-priority Go home preempts a long settle."""


def _home_cfg(cfg: GraspGenerationConfig) -> Mapping[str, Any]:
    raw = getattr(cfg, "home_joints", None) or {}
    return raw if isinstance(raw, Mapping) else {}


def _interruptible_sleep(
    seconds: float,
    cancel_event: Any = None,
    *,
    slice_s: float = 0.2,
) -> None:
    remaining = float(seconds)
    while remaining > 1e-6:
        if cancel_event is not None and cancel_event.is_set():
            raise HomeCancelled("go-home settle interrupted")
        step = min(slice_s, remaining)
        time.sleep(step)
        remaining -= step


def go_home(
    interface: Any,
    cfg: GraspGenerationConfig,
    *,
    arms: str = "both",
    cancel_event: Any = None,
    do_grasp_lift: bool = True,
) -> dict[str, Any]:
    """FSM HOME, then raise both EE by ``grasp_home_lift_m`` (grasp home).

    Sequence::

        HOLD → HOME → sleep(home_settle_s) → HOLD → OCS2
        → lift right/left EE world +Z by grasp_home_lift_m
    """
    from ros2_robot_interface import FSM_HOLD, FSM_HOME, FSM_OCS2

    _ = arms
    hj = _home_cfg(cfg)
    settle = float(
        getattr(cfg, "home_settle_s", 0.0)
        or hj.get("home_settle_s", 0.0)
        or hj.get("settle_s", 0.0)
        or 5.0
    )
    settle = max(1.0, settle)
    lift_m = float(getattr(cfg, "grasp_home_lift_m", 0.20) or 0.0)

    if not getattr(interface, "is_connected", False):
        raise RuntimeError("ROS2RobotInterface is not connected")

    print(f"[home] FSM HOME (settle={settle:.1f}s) — not MoveJ")
    interface.send_fsm_command(FSM_HOLD)
    interface.send_fsm_command(FSM_HOME)
    _interruptible_sleep(settle, cancel_event)
    interface.send_fsm_command(FSM_HOLD)
    interface.send_fsm_command(FSM_OCS2)
    print("[home] FSM HOME done → OCS2")

    # FSM/STOP→PLAY can leave Wuji fingers curled from USD defaults. Open both
    # so the next wrist approach is not a "虚握再张开" pulse.
    try:
        from .retarget.pipeline import run_open_hand

        for side in ("right", "left"):
            try:
                run_open_hand(arm=side, interface=interface)  # type: ignore[arg-type]
            except Exception as exc:
                print(f"[home] WARN open {side} hand: {exc}")
    except Exception as exc:
        print(f"[home] WARN open hands after HOME: {exc}")

    lift_info: dict[str, Any] = {}
    if do_grasp_lift and abs(lift_m) > 1e-6:
        print(f"[home] grasp home lift both EE +Z {lift_m:.3f}m")
        lift_info = lift_both_ee_world_z(
            interface,
            cfg,
            lift_m=lift_m,
            cancel_event=cancel_event,
        )

    return {
        "status": "grasp_home",
        "settle_s": settle,
        "fsm": "HOLD→HOME→HOLD→OCS2",
        "grasp_home_lift_m": lift_m if do_grasp_lift else 0.0,
        "lift": lift_info,
    }


def lift_ee_world_z(
    interface: Any,
    *,
    arm: str = "right",
    lift_m: float = 0.20,
    motion_frame_id: str = "base_link",
    arrival_timeout: float = 8.0,
    last_plan: Any = None,
) -> dict[str, Any]:
    """Lift current EE by ``lift_m`` along world/base +Z."""
    from .execution.wrist import press_wrist_world_z

    side = (arm or "right").lower()
    handler = (
        getattr(interface, "right_arm_handler", None)
        if side == "right"
        else getattr(interface, "left_arm_handler", None)
    )
    if handler is None:
        raise RuntimeError(
            f"no arm handler for {side} — check ros2-stack / EE pose topics; "
            "Hard reset may require restarting grasp-generation serve"
        )

    lift = float(lift_m)
    if abs(lift) < 1e-6:
        return {"status": "noop", "lift_m": 0.0}

    class _Plan:
        pass

    plan = _Plan()
    plan.arm = side
    plan.motion_frame_id = motion_frame_id or "base_link"

    pose = None
    try:
        pose = handler.get_pose()
    except Exception:
        pose = None
    if pose is None and last_plan is not None:
        pose = getattr(last_plan, "wrist_pose", None)
    if pose is None:
        raise RuntimeError("cannot lift: no live EE pose and no last_plan")

    plan.wrist_pose = deepcopy(pose)
    info = press_wrist_world_z(
        interface,
        plan,
        lift_m=lift,
        arrival_timeout=arrival_timeout,
    )
    info["arm"] = side
    info["lift_m"] = lift
    print(
        f"[home] lift {side} +Z {lift:.3f}m → {info.get('target_xyz')}"
    )
    return info


def lift_both_ee_world_z(
    interface: Any,
    cfg: GraspGenerationConfig,
    *,
    lift_m: float = 0.20,
    cancel_event: Any = None,
) -> dict[str, Any]:
    """Raise left and right EE by ``lift_m`` world +Z (best-effort per arm)."""
    if cancel_event is not None and cancel_event.is_set():
        raise HomeCancelled("grasp-home lift interrupted")
    frame = str(getattr(cfg, "motion_frame_id", None) or "base_link")
    timeout = float(getattr(cfg, "arrival_timeout", 8.0) or 8.0)
    out: dict[str, Any] = {}
    for side in ("right", "left"):
        if cancel_event is not None and cancel_event.is_set():
            raise HomeCancelled("grasp-home lift interrupted")
        try:
            out[side] = lift_ee_world_z(
                interface,
                arm=side,
                lift_m=lift_m,
                motion_frame_id=frame,
                arrival_timeout=timeout,
            )
        except Exception as exc:
            out[side] = {"status": "skip", "error": str(exc)}
            print(f"[home] WARN lift {side}: {exc}")
    return out


def require_arm_handler(interface: Any, arm: str) -> Any:
    """Return arm handler or raise a clear error."""
    side = (arm or "right").lower()
    handler = (
        getattr(interface, "right_arm_handler", None)
        if side == "right"
        else getattr(interface, "left_arm_handler", None)
    )
    if handler is None:
        raise RuntimeError(
            f"No arm handler for arm={side!r}. "
            "ros2-stack may not expose dual-arm EE topics, or the grasp "
            "interface connected before controllers were ready. "
            "Restart `grasp-generation serve` after ros2-stack is up "
            "(do not rely on Hard reset alone)."
        )
    return handler
