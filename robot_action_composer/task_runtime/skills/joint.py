"""关节空间 skill（``joint.*``）：MoveJ 到显式配置、或回到 ``robot.cache_joint_state`` 缓存。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from ros2_robot_interface import FSM_HOLD, FSM_MOVEJ, FSM_OCS2  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.movej_return import (  # pyright: ignore[reportMissingImports]
    movej_return_to_initial_state,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.registry import register_skill  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.types import ExecutionMeta  # pyright: ignore[reportMissingImports]


def _joint_meta(ctx: QueueRuntimeContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if getattr(ctx, "use_stamped", True) else SendMode.UNSTAMPED,
        frame_id=getattr(ctx, "frame_id", "arm_base"),
        warn_prefix="joint",
    )


def skill_movej_to_config(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """全身/单臂关节空间运动到指定配置（body / 左臂 / 右臂 均可选）。

    执行顺序：FSM HOLD → 发送关节目标 → 等待到位 → 可选恢复 OCS2。
    三组关节均为可选：只填右臂时仅驱动右臂，body 和左臂维持当前位置。

    params:
        body_positions (list[float], 可选): 躯干/腰部关节目标，典型 4 个。
            可**单独**填写（无手臂目标）：**WBC**（``ocs2_wbc_controller`` 统一 topic）下用当前双臂位置
            + 目标躯干走 ``send_dual_arm_joint_positions``；否则先 ``FSM_MOVEJ`` 再 ``send_body_joint_positions``。
        left_arm_positions (list[float], 可选): 左臂关节目标，典型 7 个。
        right_arm_positions (list[float], 可选): 右臂关节目标，典型 7 个。
        arrival_timeout (float): 手臂到位等待超时（秒），默认 30.0。
        joint_tolerance (float): 到位判定阈值（rad），默认 0.05。
        resume_ocs2 (bool): 到位后切回 OCS2，供后续笛卡尔 skill 使用，默认 False。
            当本步骤是关节运动序列中最后一步、之后紧接 pick/place 等笛卡尔 skill 时设为 True。
        skip_fsm_change (bool): 为 True 时跳过本 skill 内所有 FSM 切换（``HOLD`` 与 body-only 分支的 ``MOVEJ``）；默认 False。
        body_movej_duration (float, 可选): 执行本块前，动态设置 body joint controller 的 ``movej_duration``。
            controller 节点由 ``interface.body_controller`` 自动提供。
    """
    body_positions = [float(v) for v in (params.get("body_positions") or [])]
    left_positions = [float(v) for v in (params.get("left_arm_positions") or [])]
    right_positions = [float(v) for v in (params.get("right_arm_positions") or [])]
    arrival_timeout = float(params.get("arrival_timeout", 30.0))
    joint_tolerance = float(params.get("joint_tolerance", 0.05))
    resume_ocs2 = bool(params.get("resume_ocs2", False))
    skip_fsm_change = bool(params.get("skip_fsm_change", False))
    body_movej_duration = params.get("body_movej_duration", None)

    interface = ctx.interface
    sim_time = getattr(ctx, "sim_time", None)
    sleep_fn = sim_time.sleep if sim_time else None
    time_now_fn = sim_time.now_seconds if sim_time else None

    if body_movej_duration is not None:
        body_ctrl = str(getattr(interface, "body_controller", "")).strip()
        if not body_ctrl:
            raise ValueError(
                "joint.movej_to_config: cannot infer body controller node; "
                "ensure interface.body_controller is available (body_joint_controller_topic on ROS2RobotInterfaceConfig)"
            )
        duration_val = float(body_movej_duration)
        ok = interface.set_node_parameters(
            full_node_name=body_ctrl,
            parameters={"movej_duration": duration_val},
        )
        if ok:
            print(
                f"[MoveJ] Set {body_ctrl}.movej_duration = {duration_val}"
            )
        else:
            print(
                f"[MoveJ] WARN: failed to set {body_ctrl}.movej_duration = {duration_val}"
            )

    if not skip_fsm_change:
        try:
            interface.send_fsm_command(FSM_HOLD)
            if sleep_fn:
                sleep_fn(0.1)
        except Exception as exc:
            print(f"[MoveJ] WARN: FSM HOLD failed: {exc}")

    moved = False
    if left_positions and right_positions:
        try:
            interface.send_dual_arm_joint_positions(
                left_positions,
                right_positions,
                body_positions=body_positions or None,
            )
            print(f"[MoveJ] Dual-arm → L:{left_positions}  R:{right_positions}")
            moved = True
        except Exception as exc:
            print(f"[MoveJ] WARN: dual-arm failed, fallback: {exc}")

    if not moved and left_positions and interface.left_arm_handler is not None:
        interface.left_arm_handler.send_joint_positions(left_positions)
        moved = True
    if not moved and right_positions and interface.right_arm_handler is not None:
        interface.right_arm_handler.send_joint_positions(right_positions)
        moved = True

    if not moved and body_positions:
        body_sent = False
        try:
            ut = getattr(getattr(interface, "config", None), "unified_arm_joint_controller_topic", None) or ""
            is_wbc = "ocs2_wbc_controller" in ut
            categorized = interface.get_joint_state(categorized=True) or {}
            lp = categorized.get("left_arm", {}).get("positions")
            rp = categorized.get("right_arm", {}).get("positions")
            if is_wbc and lp and rp and len(lp) >= 7 and len(rp) >= 7:
                left_hold = [float(x) for x in lp[:7]]
                right_hold = [float(x) for x in rp[:7]]
                interface.send_dual_arm_joint_positions(
                    left_hold,
                    right_hold,
                    body_positions=body_positions,
                )
                print(f"[MoveJ] Body via WBC unified (hold arms) → body={body_positions}")
                body_sent = True
        except Exception as exc:
            print(f"[MoveJ] WARN: WBC body-via-dual failed: {exc}")

        if not body_sent:
            if not skip_fsm_change:
                try:
                    interface.send_fsm_command(FSM_MOVEJ)
                    if sleep_fn:
                        sleep_fn(0.1)
                except Exception as exc:
                    print(f"[MoveJ] WARN: FSM MOVEJ before split-body failed: {exc}")
            try:
                interface.send_body_joint_positions(body_positions)
                print(f"[MoveJ] Body split-topic → {body_positions}")
                body_sent = True
            except Exception as exc:
                print(f"[MoveJ] WARN: send_body_joint_positions failed: {exc}")

        if body_sent:
            moved = True

    if body_positions and moved and (left_positions or right_positions):
        try:
            interface.send_body_joint_positions(body_positions)
            print(f"[MoveJ] Body → {body_positions}")
        except Exception as exc:
            print(f"[MoveJ] WARN: send body joints failed: {exc}")

    if moved and (left_positions or right_positions or body_positions):
        result = interface.wait_until_joint_arrive(
            left_target_positions=left_positions or None,
            right_target_positions=right_positions or None,
            body_target_positions=body_positions or None,
            timeout=arrival_timeout,
            poll_period=0.05,
            joint_tolerance=joint_tolerance,
            time_now_fn=time_now_fn,
            sleep_fn=sleep_fn,
        )
        if result.get("arrived"):
            print("[MoveJ] Arrived at target config.")
        else:
            l_err = result.get("left_error_max_abs")
            r_err = result.get("right_error_max_abs")
            b_err = result.get("body_error_max_abs")
            print(f"[MoveJ] WARN: timeout. L_err={l_err}  R_err={r_err}  Body_err={b_err}")

    if resume_ocs2:
        delay = getattr(getattr(ctx, "robot_cfg", None), "fsm_switch_delay", 0.5)
        try:
            interface.send_fsm_command(FSM_HOLD)
            if sleep_fn:
                sleep_fn(delay)
            interface.send_fsm_command(FSM_OCS2)
            if sleep_fn:
                sleep_fn(delay)
            print("[MoveJ] OCS2 resumed (HOLD→OCS2) — ready for Cartesian control.")
        except Exception as exc:
            print(f"[MoveJ] WARN: FSM OCS2 resume failed: {exc}")

    return [], _joint_meta(ctx)


def _scratch_joint_list(raw: Any) -> list[float] | None:
    if raw is None:
        return None
    if isinstance(raw, (list, tuple)):
        if len(raw) == 0:
            return None
        return [float(x) for x in raw]
    return None


def skill_goto_cached_joints(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """关节空间运动到 ``robot.cache_joint_state`` 写入的缓存（与笛卡尔 ``goto_cache_pose`` 对称）。

    params:
        key (str, 可选): 与缓存时一致，默认 ``saved_joint_state``。
        resume_ocs2 (bool, 可选): 到位后 HOLD→延迟→OCS2，供后续 pick/place 等笛卡尔 skill；默认 False。
    """
    key = str(params.get("key", "saved_joint_state")).strip() or "saved_joint_state"
    resume_ocs2 = bool(params.get("resume_ocs2", False))

    raw = ctx.scratch_get(key)
    if not isinstance(raw, Mapping):
        raise ValueError(
            f"joint.goto_cached_joints: scratch[{key!r}] missing or not a mapping; "
            "run robot.cache_joint_state with the same key earlier in the task_queue"
        )

    left = _scratch_joint_list(raw.get("left"))
    right = _scratch_joint_list(raw.get("right"))
    body = _scratch_joint_list(raw.get("body"))

    if not left and not right and not body:
        raise ValueError(
            f"joint.goto_cached_joints: scratch[{key!r}] has no non-empty left/right/body joint lists"
        )

    rcfg = ctx.robot_cfg
    moved = movej_return_to_initial_state(
        interface=ctx.interface,
        left_initial_positions=left,
        right_initial_positions=right,
        body_initial_positions=body,
        arrival_timeout=float(rcfg.arrival_timeout),
        arrival_poll=float(rcfg.arrival_poll),
        sim_time=ctx.sim_time,
    )
    if not moved:
        print("[WARN] joint.goto_cached_joints: movej_return_to_initial_state did not command motion.")

    if resume_ocs2:
        interface = ctx.interface
        sim_time = getattr(ctx, "sim_time", None)
        sleep_fn = sim_time.sleep if sim_time else None
        delay = getattr(getattr(ctx, "robot_cfg", None), "fsm_switch_delay", 0.5)
        try:
            interface.send_fsm_command(FSM_HOLD)
            if sleep_fn:
                sleep_fn(delay)
            interface.send_fsm_command(FSM_OCS2)
            if sleep_fn:
                sleep_fn(delay)
            print("[MoveJ] goto_cached_joints: OCS2 resumed after cached joint target.")
        except Exception as exc:
            print(f"[MoveJ] WARN: OCS2 resume after goto_cached_joints failed: {exc}")

    return [], _joint_meta(ctx)


def _register_joint_skills() -> None:
    register_skill("joint.movej_to_config", skill_movej_to_config)
    register_skill("joint.goto_cached_joints", skill_goto_cached_joints)


_register_joint_skills()
