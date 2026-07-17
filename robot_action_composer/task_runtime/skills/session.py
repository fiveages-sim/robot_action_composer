"""会话暂存、末端/关节缓存与机器人状态快照 skill（无运动阶段，仅副作用）。"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from typing import Any

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.movej_return import (  # pyright: ignore[reportMissingImports]
    capture_initial_arm_joint_positions,
    capture_initial_body_joint_positions,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.types import ExecutionMeta


def _work_arm_handler(ctx: QueueRuntimeContext) -> tuple[Any, str]:
    """按 ``ctx.task_cfg.common.arm`` 返回 (ArmHandler|None, "left"|"right")。"""
    if not hasattr(ctx.task_cfg, "common"):
        raise TypeError("cache_ee_pose expects ctx.task_cfg with .common.arm")
    arm = str(ctx.task_cfg.common.arm).strip().lower()
    if arm == "right":
        return ctx.interface.right_arm_handler, "right"
    if arm == "left":
        return ctx.interface.left_arm_handler, "left"
    raise ValueError(f"arm must be 'left' or 'right', got {ctx.task_cfg.common.arm!r}")


def _session_meta(ctx: QueueRuntimeContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if ctx.use_stamped else SendMode.UNSTAMPED,
        frame_id=ctx.frame_id,
        warn_prefix="session",
    )


def _pose_to_plain(pose: Any) -> dict[str, Any] | None:
    if pose is None:
        return None
    pos = getattr(pose, "position", None)
    ori = getattr(pose, "orientation", None)
    if pos is None or ori is None:
        return None
    return {
        "position": {
            "x": float(pos.x),
            "y": float(pos.y),
            "z": float(pos.z),
        },
        "orientation": {
            "x": float(ori.x),
            "y": float(ori.y),
            "z": float(ori.z),
            "w": float(ori.w),
        },
    }


def skill_scratch_put(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[Any], ExecutionMeta]:
    """写入 :attr:`QueueRuntimeContext.scratch`。

    params:
        key (str): 键。
        value: 任意 YAML 可表达的值（标量 / 列表 / 映射）。
    """
    raw_key = params.get("key")
    if raw_key is None or not str(raw_key).strip():
        raise ValueError("session.scratch_put requires non-empty params.key")
    if "value" not in params:
        raise ValueError("session.scratch_put requires params.value")
    key = str(raw_key)
    ctx.scratch_put(key, params["value"])
    print(f"[Session] scratch_put key={key!r}")
    return [], _session_meta(ctx)


def skill_scratch_clear(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[Any], ExecutionMeta]:
    """清空暂存区或删除键名前缀匹配项。

    params:
        prefix (str, 可选): 若设置，仅删除 ``str(key).startswith(prefix)`` 的键；否则清空全部。
    """
    prefix = params.get("prefix")
    if prefix is None or str(prefix) == "":
        n = len(ctx.scratch)
        ctx.scratch.clear()
        print(f"[Session] scratch_clear all ({n} key(s) removed)")
        return [], _session_meta(ctx)
    pfx = str(prefix)
    to_del = [k for k in list(ctx.scratch.keys()) if str(k).startswith(pfx)]
    for k in to_del:
        del ctx.scratch[k]
    print(f"[Session] scratch_clear prefix={pfx!r} removed={len(to_del)}")
    return [], _session_meta(ctx)


def _cache_one_arm_block(
    handler: Any,
    *,
    arm_label: str,
    grip_f: float,
) -> dict[str, Any] | None:
    """单臂一块 ``{pose, gripper}``；无 handler 时返回 ``None``（仅 ``which: both`` 时允许一侧缺失）。"""
    if handler is None:
        return None
    pose = handler.get_pose()
    if pose is None:
        raise RuntimeError(f"robot.cache_ee_pose: {arm_label} get_pose() returned None (no EE state yet)")
    plain = _pose_to_plain(pose)
    if plain is None:
        raise RuntimeError(f"robot.cache_ee_pose: failed to serialize {arm_label} pose")
    return {"pose": plain, "gripper": grip_f}


def skill_cache_ee_pose(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[Any], ExecutionMeta]:
    """将末端位姿与夹爪目标缓存到 ``ctx.scratch``。

    - **单臂**（默认）：``which: work``（或省略）按 ``task_cfg.common.arm`` 选臂，写入扁平
      ``{"pose": ..., "gripper": ...}``，配合 ``single_arm.goto_cache_pose``。
    - **显式单臂**：``which: left`` / ``which: right`` 只缓存该侧（仍为扁平结构）。
    - **双臂**：``which: both`` 写入 ``{"left": {...}|null, "right": {...}|null}``，配合
      ``dual_arm.goto_cache_pose``（两侧均需有效 pose）；若只需动一侧可用 ``single_arm.goto_cache_pose``
      并设 ``side: left|right``。

    典型放在 **task_queue 靠前**（reset/OCS2 后首步或 pregrasp 之后）、pick 之前。

    params:
        key (str, 可选): 缓存键，默认 ``saved_ee_pose``。
        which (str, 可选): ``work`` | ``left`` | ``right`` | ``both``，默认 ``work``。
        gripper (float, 可选): 单臂或与 ``both`` 时两侧默认夹爪目标；默认 ``ctx.gripper_open``。
        left_gripper / right_gripper (float, 可选): 仅 ``which: both`` 时按侧覆盖 ``gripper``。
    """
    key = str(params.get("key", "saved_ee_pose")).strip() or "saved_ee_pose"
    which = str(params.get("which", "work")).strip().lower()
    if which in ("", "work", "pick", "primary"):
        which = "work"

    grip_default = float(ctx.gripper_open if params.get("gripper") is None else params["gripper"])

    if which == "both":
        lg = float(params.get("left_gripper", grip_default))
        rg = float(params.get("right_gripper", grip_default))
        left_b = _cache_one_arm_block(ctx.interface.left_arm_handler, arm_label="left", grip_f=lg)
        right_b = _cache_one_arm_block(ctx.interface.right_arm_handler, arm_label="right", grip_f=rg)
        if left_b is None and right_b is None:
            raise RuntimeError("robot.cache_ee_pose which=both: neither arm handler available")
        ctx.scratch_put(key, {"left": left_b, "right": right_b})
        print(
            f"[Session] robot.cache_ee_pose which=both key={key!r} "
            f"left={'ok' if left_b else 'null'} right={'ok' if right_b else 'null'}"
        )
        return [], _session_meta(ctx)

    if which == "left":
        handler, arm_label = ctx.interface.left_arm_handler, "left"
    elif which == "right":
        handler, arm_label = ctx.interface.right_arm_handler, "right"
    elif which == "work":
        handler, arm_label = _work_arm_handler(ctx)
    else:
        raise ValueError(
            "robot.cache_ee_pose: which must be work|left|right|both, "
            f"got {params.get('which')!r}"
        )

    block = _cache_one_arm_block(handler, arm_label=arm_label, grip_f=grip_default)
    if block is None:
        raise RuntimeError(f"robot.cache_ee_pose: no {arm_label} arm handler")
    ctx.scratch_put(key, block)
    print(f"[Session] robot.cache_ee_pose which={which!r} arm={arm_label!r} key={key!r} gripper={grip_default}")
    return [], _session_meta(ctx)


def skill_cache_joint_state(ctx: QueueRuntimeContext, params: Mapping[str, Any]) -> tuple[list[Any], ExecutionMeta]:
    """将当前手臂（及可选躯干）关节角缓存到 ``ctx.scratch``，供 ``joint.goto_cached_joints`` 恢复。

    与 ``robot.cache_ee_pose`` 对称：前者笛卡尔回程，本技能关节空间回程。

    params:
        key (str, 可选): 默认 ``saved_joint_state``。
        include_body (bool, 可选): 是否缓存 ``body``，默认 True。
        wait_timeout (float, 可选): 读取关节状态等待超时（秒），默认 2.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.05。

    写入结构::

        {"left": [...]|null, "right": [...]|null, "body": [...]|null}
    """
    key = str(params.get("key", "saved_joint_state")).strip() or "saved_joint_state"
    include_body = bool(params.get("include_body", True))
    wait_timeout = float(params.get("wait_timeout", 2.0))
    poll_period = float(params.get("poll_period", 0.05))

    iface = ctx.interface
    left, right = capture_initial_arm_joint_positions(
        iface, wait_timeout=wait_timeout, poll_period=poll_period
    )
    body = None
    if include_body:
        body = capture_initial_body_joint_positions(
            iface, wait_timeout=wait_timeout, poll_period=poll_period
        )

    if not left and not right and not body:
        raise RuntimeError(
            "robot.cache_joint_state: no arm or body joint positions captured "
            "(check get_joint_state / interface connection)"
        )

    ctx.scratch_put(
        key,
        {
            "left": left,
            "right": right,
            "body": body,
        },
    )
    lb = len(left) if left else 0
    rb = len(right) if right else 0
    bb = len(body) if body else 0
    print(f"[Session] robot.cache_joint_state key={key!r} left={lb} right={rb} body={bb}")
    return [], _session_meta(ctx)


def skill_robot_snapshot_state(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """从 ``ROS2RobotInterface`` 读取当前关节状态、可选双臂末端位姿等，写入 ``ctx.scratch[key]``。

    params:
        key (str, 可选): 写入 ``scratch`` 的键，默认 ``robot_state``。
        include_joint_state (bool, 可选): 默认 True；调用 ``get_joint_state``。
        joint_state_categorized (bool, 可选): 默认 False。
        include_left_pose / include_right_pose (bool, 可选): 默认 True；无 handler 时为 null。
        include_body_target (bool, 可选): 默认 True。
        include_fsm (bool, 可选): 默认 True；记录 ``fsm_command`` 数值。
    """
    key = str(params.get("key", "robot_state")).strip() or "robot_state"
    iface = ctx.interface
    include_js = bool(params.get("include_joint_state", True))
    js_cat = bool(params.get("joint_state_categorized", False))
    inc_l = bool(params.get("include_left_pose", True))
    inc_r = bool(params.get("include_right_pose", True))
    inc_body = bool(params.get("include_body_target", True))
    inc_fsm = bool(params.get("include_fsm", True))

    snap: dict[str, Any] = {}
    st = ctx.sim_time
    if st is not None and hasattr(st, "now_seconds"):
        snap["captured_at_sim_s"] = float(st.now_seconds())

    if include_js:
        js = iface.get_joint_state(categorized=js_cat)
        snap["joint_state"] = copy.deepcopy(js) if js is not None else None

    if inc_l:
        lh = iface.left_arm_handler
        snap["left_ee_pose"] = _pose_to_plain(lh.get_pose() if lh is not None else None)
    if inc_r:
        rh = iface.right_arm_handler
        snap["right_ee_pose"] = _pose_to_plain(rh.get_pose() if rh is not None else None)

    if inc_body:
        bt = iface.get_body_current_target()
        snap["body_current_target"] = list(bt) if bt is not None else None

    if inc_fsm and hasattr(iface, "get_fsm_command"):
        try:
            snap["fsm_command"] = int(iface.get_fsm_command())
        except Exception:  # noqa: BLE001
            snap["fsm_command"] = None

    ctx.scratch_put(key, snap)
    print(f"[Session] robot.snapshot_state → scratch[{key!r}] keys={list(snap.keys())!r}")
    return [], _session_meta(ctx)


def skill_send_mode_command(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """向 ``/mode_command`` 发布 WBC / 底盘模式命令（无运动阶段）。

    全身控制下开启双臂耦合示例：``command: ARMS_COUPLED``。
    追踪 + 双臂耦合可写 ``commands: [BODY_TRACKING, ARMS_COUPLED]``（按序发送）。
    本技能只发布 mode，不切换 FSM。默认先发完所有命令，再一次性等待
    ``/ocs2_wbc_controller/current_state`` 同时满足全部期望字段。

    params:
        command / mode (str | list): 单条或列表；也可用 ``commands``。
            如 ``ARMS_COUPLED``、``BODY_TRACKING``、``BODY_FREE``、``BASE_LOCK``。
        commands (list, 可选): 多条模式命令，按顺序发布（与 ``command`` 二选一或合并）。
        wait_for_state (bool, 可选): 是否等 current_state 确认，默认 True。
        wait_timeout (float, 可选): 全部命令确认超时（秒），默认 5.0。
        settle_time (float, 可选): 确认后（或跳过确认时）额外固定等待；
            接口内置约 0.1s，更大时补睡差额。
    """
    import time

    cmds: list[str] = []
    raw_list = params.get("commands")
    if raw_list is not None:
        if not isinstance(raw_list, (list, tuple)):
            raise ValueError("robot.send_mode_command: params.commands must be a list of strings")
        cmds.extend(str(x).strip().upper() for x in raw_list if x is not None and str(x).strip())
    raw = params.get("command", params.get("mode"))
    if raw is not None:
        if isinstance(raw, (list, tuple)):
            cmds.extend(str(x).strip().upper() for x in raw if x is not None and str(x).strip())
        elif str(raw).strip():
            cmds.append(str(raw).strip().upper())
    if not cmds:
        raise ValueError(
            "robot.send_mode_command requires non-empty params.command / params.mode "
            "or params.commands"
        )

    wait_for_state = bool(params.get("wait_for_state", True))
    wait_timeout = float(params.get("wait_timeout", 5.0))
    if wait_timeout < 0.0:
        raise ValueError("robot.send_mode_command: wait_timeout must be >= 0")

    settle_extra = 0.0
    settle_raw = params.get("settle_time")
    if settle_raw is not None:
        settle_target = float(settle_raw)
        if settle_target < 0.0:
            raise ValueError("robot.send_mode_command: settle_time must be >= 0")
        iface_settle = float(getattr(ctx.interface, "MODE_SWITCH_SETTLE_TIME_SEC", 0.1) or 0.0)
        settle_extra = max(0.0, settle_target - iface_settle)

    iface = ctx.interface
    if not hasattr(iface, "send_mode_command"):
        raise RuntimeError("robot.send_mode_command: interface has no send_mode_command")
    sim_time = getattr(ctx, "sim_time", None)
    sleep_fn = sim_time.sleep if sim_time else time.sleep
    time_now_fn = sim_time.now_seconds if sim_time else None

    for command in cmds:
        iface.send_mode_command(command)
        print(f"[Session] robot.send_mode_command command={command!r}")

    ok = True
    if wait_for_state and hasattr(iface, "wait_until_mode_commands_applied"):
        ok = bool(
            iface.wait_until_mode_commands_applied(
                cmds,
                timeout=wait_timeout,
                time_now_fn=time_now_fn,
                sleep_fn=sleep_fn,
            )
        )
        if not ok:
            print(
                f"[Session] WARN: modes {cmds!r} not confirmed on "
                f"/ocs2_wbc_controller/current_state within {wait_timeout:.1f}s"
            )
        else:
            print(f"[Session] robot.send_mode_command wait_ok=True modes={cmds!r}")
    elif wait_for_state and hasattr(iface, "wait_until_mode_command_applied"):
        # 旧接口兜底：仍按批末一次确认最后一条
        ok = bool(
            iface.wait_until_mode_command_applied(
                cmds[-1],
                timeout=wait_timeout,
                time_now_fn=time_now_fn,
                sleep_fn=sleep_fn,
            )
        )
        if not ok:
            print(
                f"[Session] WARN: mode {cmds[-1]!r} not confirmed on "
                f"/ocs2_wbc_controller/current_state within {wait_timeout:.1f}s"
            )

    if settle_extra > 0.0:
        sleep_fn(settle_extra)
        print(f"[Session] robot.send_mode_command settle_extra={settle_extra:.3f}s")
    return [], _session_meta(ctx)


register_skill("session.scratch_put", skill_scratch_put)
register_skill("session.scratch_clear", skill_scratch_clear)
register_skill("robot.cache_ee_pose", skill_cache_ee_pose)
register_skill("robot.cache_joint_state", skill_cache_joint_state)
register_skill("robot.snapshot_state", skill_robot_snapshot_state)
register_skill("robot.send_mode_command", skill_send_mode_command)
