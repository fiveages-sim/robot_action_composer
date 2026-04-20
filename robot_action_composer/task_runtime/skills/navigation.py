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
      - skill: single_arm.pick

并行模式（未来 runner 支持 parallel 语法后）::

    task_queue:
      - parallel:
          - skill: robot.send_nav_goal
            params: {x: 2.0, y: 1.5, yaw: 0.0}
      - skill: robot.wait_nav_arrived
        params: {timeout: 60.0}
      - skill: single_arm.pick
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from ros2_robot_interface.utils.quat_pose import quat_multiply, quat_conjugate  # pyright: ignore[reportMissingImports]

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
    base_world_pos, base_world_quat = get_entity_pose_world_service(ctx.base_link_entity_path)
    ctx.base_world_pos = base_world_pos
    ctx.base_world_quat = base_world_quat


def _default_meta(ctx: _NavContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if getattr(ctx, "use_stamped", True) else SendMode.UNSTAMPED,
        frame_id=getattr(ctx, "frame_id", "arm_base"),
        warn_prefix="nav",
    )


def _quat_rotate_vector_xyzw(
    q: tuple[float, float, float, float], v: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Rotate 3D vector ``v`` by unit quaternion ``q`` (x, y, z, w)."""
    x, y, z = v
    v_quat = (float(x), float(y), float(z), 0.0)
    t = quat_multiply(q, v_quat)
    out = quat_multiply(t, quat_conjugate(q))
    return (out[0], out[1], out[2])


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

    从 ``object_prim_path`` 获取物体世界 (x, y)，叠加 ``approach_offset_x/y``
    后作为 Nav2 目标，适合"先导航到物体附近再抓取"的场景。

    完成后自动刷新 ``ctx.base_world_pos/quat``。

    params:
        object_prim_path (str): Isaac Sim 物体的 Prim 路径（世界坐标查询用）。
            若未指定，则从 ``ctx.task_cfg.object_prim_path`` 读取。
        approach_offset_x (float, 可选): 导航目标相对物体世界 X 的偏移（米），默认 -0.20。
            负值表示机器人在物体 -X 方向（从 X 前方接近）。
        approach_offset_y (float, 可选): 导航目标相对物体世界 Y 的偏移（米），默认 0.0。
        yaw (float, 可选): 机器人到达时的朝向（弧度），默认 0.0。
        frame_id (str, 可选): TF 帧，默认 ``"map"``。
        timeout (float, 可选): 最大等待时间（秒），默认 60.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.1。
    """
    object_path = str(
        params.get("object_prim_path")
        or getattr(ctx.task_cfg, "object_prim_path", None)
        or ""
    )
    if not object_path:
        raise ValueError(
            "nav.navigate_to_object requires 'object_prim_path' param "
            "or ctx.task_cfg.object_prim_path"
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


def skill_navigate_backup(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """沿基座 **base_link +X 反方向** 在水平面内平移 ``distance_m``，朝向与当前一致。

    使用 Isaac 查询的基座世界位姿；后退目标为 ``pos - distance * normalize(forward_xy)``，
    ``forward`` 为将 body +X 旋到世界系后的水平投影。

    params:
        distance_m (float, 可选): 后退距离（米），默认 ``0.5``。
        frame_id (str, 可选): Nav2 目标帧，默认 ``map``。
        timeout (float, 可选): 默认 ``60.0``。
        poll_period (float, 可选): 默认 ``0.1``。
    """
    distance_m = float(params.get("distance_m", 0.5))
    frame_id = str(params.get("frame_id", "map"))
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    base_path = getattr(ctx, "base_link_entity_path", None) or ""
    if not base_path:
        raise ValueError("nav.navigate_backup requires a resolved base_link_entity_path on context")

    (bx, by, _bz), quat = get_entity_pose_world_service(base_path)
    fx, fy, _fz = _quat_rotate_vector_xyzw(quat, (1.0, 0.0, 0.0))
    horiz = math.hypot(fx, fy)
    if horiz < 1e-6:
        fx, fy = 1.0, 0.0
        horiz = 1.0
    fx, fy = fx / horiz, fy / horiz
    gx = bx - distance_m * fx
    gy = by - distance_m * fy
    yaw = math.atan2(fy, fx)

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    print(
        f"[Nav] Backup {distance_m:.2f} m from ({bx:.3f},{by:.3f}) "
        f"→ ({gx:.3f},{gy:.3f}) yaw={yaw:.3f}"
    )
    success = ctx.interface.navigate_to_pose(
        gx,
        gy,
        yaw,
        frame_id=frame_id,
        timeout=timeout,
        poll_period=poll_period,
        time_now_fn=time_now_fn,
        sleep_fn=sleep_fn,
    )
    if not success:
        raise RuntimeError(
            f"nav.navigate_backup failed (target=({gx:.3f}, {gy:.3f}), distance_m={distance_m})"
        )
    print("[Nav] Backup complete — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def _register_navigation_skills() -> None:
    register_skill("nav.send_nav_goal", skill_send_nav_goal)
    register_skill("nav.wait_nav_arrived", skill_wait_nav_arrived)
    register_skill("nav.navigate_to_pose", skill_navigate_to_pose)
    register_skill("nav.navigate_to_object", skill_navigate_to_object)
    register_skill("nav.navigate_backup", skill_navigate_backup)


_register_navigation_skills()
