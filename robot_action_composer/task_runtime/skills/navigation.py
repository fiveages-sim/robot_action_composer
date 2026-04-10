"""导航 skill：robot.send_nav_goal / robot.wait_nav_arrived / robot.navigate_to_pose。

这三个 skill 封装 ``ROS2RobotInterface`` 的 Nav2 四件套 API，
并在导航完成后自动刷新 ``ctx.base_world_pos / ctx.base_world_quat``，
确保后续 pick/place 的相对位置计算使用机器人到达新位置后的基座坐标。

顺序模式（当前 runner 支持）::

    task_queue:
      - skill: robot.navigate_to_pose
        params:
          x: 2.0
          y: 1.5
          yaw: 0.0
          frame_id: map
          timeout: 60.0
      - skill: single_arm.pregrasp
      - skill: single_arm.pick

并行模式（未来 runner 支持 parallel 语法后）::

    task_queue:
      - parallel:
          - skill: robot.send_nav_goal
            params: {x: 2.0, y: 1.5, yaw: 0.0}
          - skill: single_arm.pregrasp   # 导航期间准备手臂姿态
      - skill: robot.wait_nav_arrived
        params: {timeout: 60.0}
      - skill: single_arm.pick
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from ros2_robot_interface import FSM_HOLD, FSM_OCS2  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]

from robot_action_composer.isaac_sim import get_entity_pose_world_service  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.registry import register_skill  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.types import ExecutionMeta  # pyright: ignore[reportMissingImports]

_NavContext = QueueRuntimeContext


def _refresh_base_pose(ctx: _NavContext) -> None:
    """导航完成后，从 Isaac Sim 重新获取机器人基座世界坐标。

    更新 ctx.base_world_pos / ctx.base_world_quat，
    后续 pick/place skill 的相对位置计算将使用新坐标。
    """
    base_world_pos, base_world_quat = get_entity_pose_world_service(
        ctx.robot_cfg.base_link_entity_path
    )
    ctx.base_world_pos = base_world_pos
    ctx.base_world_quat = base_world_quat


def _default_meta(ctx: _NavContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if getattr(ctx, "use_stamped", True) else SendMode.UNSTAMPED,
        frame_id=getattr(ctx, "frame_id", "arm_base"),
        warn_prefix="nav",
    )


def skill_send_nav_goal(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """非阻塞发送导航目标（立即返回，不等待到达）。

    params:
        x (float): 目标 X 坐标（米）。
        y (float): 目标 Y 坐标（米）。
        yaw (float, 可选): 目标朝向（弧度），默认 0.0。
        frame_id (str, 可选): TF 帧，默认 ``"map"``。
    """
    x = float(params["x"])
    y = float(params["y"])
    yaw = float(params.get("yaw", 0.0))
    frame_id = str(params.get("frame_id", "map"))

    ctx.interface.send_nav_goal(x, y, yaw, frame_id=frame_id)
    print(f"[Nav] Goal sent → x={x:.3f} y={y:.3f} yaw={yaw:.3f} (non-blocking)")
    return [], _default_meta(ctx)


def skill_wait_nav_arrived(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """阻塞等待当前导航目标到达，完成后刷新基座世界坐标。

    params:
        timeout (float, 可选): 最大等待时间（秒），默认 60.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.1。
    """
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    success = ctx.interface.wait_nav_arrived(
        timeout=timeout,
        poll_period=poll_period,
        time_now_fn=time_now_fn,
        sleep_fn=sleep_fn,
    )
    if not success:
        raise RuntimeError(f"Navigation timed out or failed (timeout={timeout:.1f}s)")

    print("[Nav] Arrived — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def skill_navigate_to_pose(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """发送导航目标并阻塞等待到达，完成后刷新基座世界坐标。

    等价于 robot.send_nav_goal + robot.wait_nav_arrived 的顺序组合，
    适用于不需要并行的场景。

    params:
        x (float): 目标 X 坐标（米）。
        y (float): 目标 Y 坐标（米）。
        yaw (float, 可选): 目标朝向（弧度），默认 0.0。
        frame_id (str, 可选): TF 帧，默认 ``"map"``。
        timeout (float, 可选): 最大等待时间（秒），默认 60.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.1。
    """
    x = float(params["x"])
    y = float(params["y"])
    yaw = float(params.get("yaw", 0.0))
    frame_id = str(params.get("frame_id", "map"))
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    print(f"[Nav] Navigating to x={x:.3f} y={y:.3f} yaw={yaw:.3f} (timeout={timeout:.0f}s)")
    success = ctx.interface.navigate_to_pose(
        x, y, yaw,
        frame_id=frame_id,
        timeout=timeout,
        poll_period=poll_period,
        time_now_fn=time_now_fn,
        sleep_fn=sleep_fn,
    )
    if not success:
        raise RuntimeError(f"Navigation failed or timed out (x={x:.3f} y={y:.3f})")

    print("[Nav] Arrived — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def skill_navigate_to_object(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """根据 Isaac Sim 中物体的世界坐标自动计算导航目标，并阻塞等待到达。

    从 ``object_entity_path`` 获取物体世界 (x, y)，叠加 ``approach_offset_x/y``
    后作为 Nav2 目标，适合"先导航到物体附近再抓取"的场景。

    完成后自动刷新 ``ctx.base_world_pos/quat``。

    params:
        object_entity_path (str): Isaac Sim 物体的 Prim 路径（世界坐标查询用）。
            若未指定，则从 ``ctx.task_cfg.source_object_entity_path`` 读取。
        approach_offset_x (float, 可选): 导航目标相对物体世界 X 的偏移（米），默认 -0.20。
            负值表示机器人在物体 -X 方向（从 X 前方接近）。
        approach_offset_y (float, 可选): 导航目标相对物体世界 Y 的偏移（米），默认 0.0。
        yaw (float, 可选): 机器人到达时的朝向（弧度），默认 0.0。
        frame_id (str, 可选): TF 帧，默认 ``"map"``。
        timeout (float, 可选): 最大等待时间（秒），默认 60.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.1。
    """
    object_path = str(
        params.get("object_entity_path")
        or getattr(ctx.task_cfg, "source_object_entity_path", None)
        or ""
    )
    if not object_path:
        raise ValueError(
            "nav.navigate_to_object requires 'object_entity_path' param "
            "or ctx.task_cfg.source_object_entity_path"
        )

    offset_x = float(params.get("approach_offset_x", -0.20))
    offset_y = float(params.get("approach_offset_y", 0.0))
    yaw = float(params.get("yaw", 0.0))
    frame_id = str(params.get("frame_id", "map"))
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    # 从 Isaac Sim 获取物体世界坐标
    (obj_x, obj_y, obj_z), _ = get_entity_pose_world_service(object_path)
    nav_x = obj_x + offset_x
    nav_y = obj_y + offset_y
    print(
        f"[Nav] Object world pos: ({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
        f"  →  nav target: ({nav_x:.3f}, {nav_y:.3f}) yaw={yaw:.3f}"
    )

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    success = ctx.interface.navigate_to_pose(
        nav_x, nav_y, yaw,
        frame_id=frame_id,
        timeout=timeout,
        poll_period=poll_period,
        time_now_fn=time_now_fn,
        sleep_fn=sleep_fn,
    )
    if not success:
        raise RuntimeError(
            f"Navigation to object approach failed (target=({nav_x:.3f}, {nav_y:.3f}))"
        )

    print("[Nav] Arrived at object approach position — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def skill_movej_to_config(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """全身/单臂关节空间运动到指定配置（body / 左臂 / 右臂 均可选）。

    执行顺序：FSM HOLD → 发送关节目标 → 等待到位 → 可选恢复 OCS2。
    三组关节均为可选：只填右臂时仅驱动右臂，body 和左臂维持当前位置。

    params:
        body_positions (list[float], 可选): 躯干/腰部关节目标，典型 4 个。
        left_arm_positions (list[float], 可选): 左臂关节目标，典型 7 个。
        right_arm_positions (list[float], 可选): 右臂关节目标，典型 7 个。
        arrival_timeout (float): 手臂到位等待超时（秒），默认 30.0。
        joint_tolerance (float): 到位判定阈值（rad），默认 0.05。
        resume_ocs2 (bool): 到位后切回 OCS2，供后续笛卡尔 skill 使用，默认 False。
            当本步骤是关节运动序列中最后一步、之后紧接 pick/place 等笛卡尔 skill 时设为 True。
    """
    body_positions   = [float(v) for v in (params.get("body_positions")     or [])]
    left_positions   = [float(v) for v in (params.get("left_arm_positions")  or [])]
    right_positions  = [float(v) for v in (params.get("right_arm_positions") or [])]
    arrival_timeout  = float(params.get("arrival_timeout", 30.0))
    joint_tolerance  = float(params.get("joint_tolerance", 0.05))
    resume_ocs2      = bool(params.get("resume_ocs2", False))

    interface  = ctx.interface
    sim_time   = getattr(ctx, "sim_time", None)
    sleep_fn   = sim_time.sleep       if sim_time else None
    time_now_fn= sim_time.now_seconds if sim_time else None

    # ── 1. 切 HOLD ───────────────────────────────────────────────────────────
    try:
        interface.send_fsm_command(FSM_HOLD)
        if sleep_fn:
            sleep_fn(0.1)
    except Exception as exc:
        print(f"[MoveJ] WARN: FSM HOLD failed: {exc}")

    # ── 2. 发送手臂关节 ───────────────────────────────────────────────────────
    # send_dual_arm_joint_positions 内部会切到 MOVEJ(4)；
    # 对 WBC 控制器同时将 body_positions 打包进同一条消息；
    # 对 split-body 控制器忽略 body_positions（将在步骤 3 单独发送）。
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

    # ── 3. 发送躯干关节（FSM 此时已在 MOVEJ=4，body controller 可接受指令）────
    # split-body 模式：body controller 独立订阅，需在 MOVEJ 状态下单独发送。
    # WBC 模式：body 已随手臂消息一起发出，此处 publisher 为 None，调用静默跳过。
    if body_positions and moved:
        try:
            interface.send_body_joint_positions(body_positions)
            print(f"[MoveJ] Body → {body_positions}")
        except Exception as exc:
            print(f"[MoveJ] WARN: send body joints failed: {exc}")

    # ── 4. 等待全部关节到位（手臂 + 躯干）──────────────────────────────────────
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

    # ── 5. 可选：切回 OCS2 供后续笛卡尔 skill（pick/place）使用 ─────────────────
    # ros2_robot_interface 的笛卡尔接口不会自动切 FSM，必须手动切到 OCS2(3)。
    # 必须先过 HOLD 再切 OCS2（与 Runner 启动时序一致：HOLD→delay→OCS2→delay），
    # 否则 OCS2 控制器从 MOVEJ 直接切入时不接管。
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

    return [], _default_meta(ctx)


def _register_navigation_skills() -> None:
    register_skill("nav.send_nav_goal", skill_send_nav_goal)
    register_skill("nav.wait_nav_arrived", skill_wait_nav_arrived)
    register_skill("nav.navigate_to_pose", skill_navigate_to_pose)
    register_skill("nav.navigate_to_object", skill_navigate_to_object)
    register_skill("joint.movej_to_config", skill_movej_to_config)


_register_navigation_skills()
