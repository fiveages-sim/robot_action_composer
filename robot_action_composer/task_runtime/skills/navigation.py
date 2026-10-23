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
import time
from collections.abc import Mapping
from typing import Any

from ros2_robot_interface.utils.quat_pose import quat_multiply, quat_conjugate  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]

from robot_action_composer.isaac_sim import get_entity_pose_world_service  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.object_binding import resolve_object_prim_path  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.object_resolution_replay import resolve_nav_object_pose_for_task  # pyright: ignore[reportMissingImports]
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


def _yaw_from_quat_xyzw(q: tuple[float, float, float, float]) -> float:
    x, y, z, w = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _world_xy_yaw_to_base_footprint(
    *,
    target_x: float,
    target_y: float,
    target_yaw: float,
    base_xy: tuple[float, float],
    base_quat_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """World approach pose → pose in current ``base_footprint`` (dx, dy, dyaw).

    Isaac GT ``/odom`` is relative to Play-start; with ``map≈odom`` at identity,
    sending **world** XY as ``frame_id=map`` goals is wrong whenever the robot
    did not start at world origin (e.g. teleported to x=-2). Body-frame goals
    stay correct.
    """
    dx_w = float(target_x) - float(base_xy[0])
    dy_w = float(target_y) - float(base_xy[1])
    base_yaw = _yaw_from_quat_xyzw(base_quat_xyzw)
    c = math.cos(base_yaw)
    s = math.sin(base_yaw)
    dx_b = c * dx_w + s * dy_w
    dy_b = -s * dx_w + c * dy_w
    dyaw = math.atan2(
        math.sin(float(target_yaw) - base_yaw),
        math.cos(float(target_yaw) - base_yaw),
    )
    return dx_b, dy_b, dyaw


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
            可与 ``object_key`` 二选一；显式 prim 优先。
        object_key (str): 从任务 ``objects`` / ``.meta/objects`` 解析 ``object_prim_path``。
        approach_offset_x (float, 可选): 导航目标相对物体世界 X 的偏移（米），默认 -0.20。
            负值表示机器人在物体 -X 方向（从 X 前方接近）。
        approach_offset_y (float, 可选): 导航目标相对物体世界 Y 的偏移（米），默认 0.0。
        yaw (float, 可选): 机器人到达时的朝向（弧度），默认 0.0。
        frame_id (str, 可选): TF 帧，默认 ``"map"``。
        timeout (float, 可选): 最大等待时间（秒），默认 60.0。
        poll_period (float, 可选): 轮询间隔（秒），默认 0.1。
    """
    object_path = resolve_object_prim_path(
        params,
        objects=getattr(ctx, "objects", None),
        prim_param="object_prim_path",
        key_params="object_key",
        required=True,
        label="nav.navigate_to_object",
    )
    assert object_path  # required=True

    offset_x = float(params.get("approach_offset_x", -0.20))
    offset_y = float(params.get("approach_offset_y", 0.0))
    yaw = float(params.get("yaw", 0.0))
    # Default map: legacy warehouse demos (spawn≈world origin).
    # base_footprint: Isaac GT odom-safe (dex_hand / robot teleported away from origin).
    frame_id = str(params.get("frame_id", "map")).strip() or "map"
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    # Object pose is always resolved in world/map (Isaac GT: map≡world).
    obj_pose = resolve_nav_object_pose_for_task(
        ctx,
        object_prim_path=object_path,
        frame_id="map",
    )
    obj_x = float(obj_pose.position.x)
    obj_y = float(obj_pose.position.y)
    obj_z = float(obj_pose.position.z)
    nav_x = obj_x + offset_x
    nav_y = obj_y + offset_y

    goal_x, goal_y, goal_yaw = nav_x, nav_y, yaw
    send_frame = frame_id
    # Isaac GT: /odom relative to Play-start; map→odom often identity.
    # World XY as map goals is wrong if spawn≠world origin. Instead:
    #   Isaac world → body (dx,dy,dyaw) → compose onto TF map←base → send frame=map.
    # Never ask Nav2 to transform base_footprint goals (sim/wall stamp → ABORT 6).
    if frame_id in ("base_footprint", "base_link", "odom_relative"):
        base_path = getattr(ctx, "base_link_entity_path", None) or ""
        if not base_path:
            raise ValueError(
                "nav.navigate_to_object odom_relative/base_footprint requires "
                "base_link_entity_path on context"
            )
        (bx, by, _bz), bq = get_entity_pose_world_service(base_path)
        dx_b, dy_b, dyaw_b = _world_xy_yaw_to_base_footprint(
            target_x=nav_x,
            target_y=nav_y,
            target_yaw=yaw,
            base_xy=(bx, by),
            base_quat_xyzw=bq,
        )
        send_frame = "map"
        mx = my = 0.0
        myaw = 0.0
        tf = None
        lookup = getattr(ctx.interface, "lookup_transform", None)
        if callable(lookup):
            # After Isaac STOP→PLAY, odom TF can lag PhysX; retry before body-delta fallback.
            tf_deadline = time.monotonic() + 6.0
            while time.monotonic() < tf_deadline and tf is None:
                for base_frame in ("base_footprint", "base_link"):
                    try:
                        tf = lookup("map", base_frame, timeout=1.0)
                    except Exception:
                        tf = None
                    if tf is not None:
                        break
                if tf is None:
                    sim = getattr(ctx, "sim_time", None)
                    if sim is not None and hasattr(sim, "sleep"):
                        sim.sleep(0.25)
                    else:
                        time.sleep(0.25)
        if tf is not None:
            t = tf.transform.translation
            r = tf.transform.rotation
            mx, my = float(t.x), float(t.y)
            myaw = _yaw_from_quat_xyzw((float(r.x), float(r.y), float(r.z), float(r.w)))
            c, s = math.cos(myaw), math.sin(myaw)
            goal_x = mx + c * dx_b - s * dy_b
            goal_y = my + s * dx_b + c * dy_b
            goal_yaw = math.atan2(
                math.sin(myaw + dyaw_b), math.cos(myaw + dyaw_b)
            )
        else:
            # TF not ready (common right after sim reset): assume map pose ≈ odom origin
            goal_x, goal_y, goal_yaw = dx_b, dy_b, dyaw_b
            print(
                "[Nav] WARN: map←base TF unavailable; "
                "using body delta as map goal (ok if odom still at Play origin)"
            )
        print(
            f"[Nav] Object world=({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f}) "
            f"approach_world=({nav_x:.3f}, {nav_y:.3f}) | "
            f"base_world=({bx:.3f}, {by:.3f}) body=({dx_b:.3f},{dy_b:.3f},{dyaw_b:.3f}) | "
            f"map_base=({mx:.3f},{my:.3f},{myaw:.3f}) → map goal "
            f"({goal_x:.3f}, {goal_y:.3f}) yaw={goal_yaw:.3f}"
        )
    else:
        print(
            f"[Nav] Object world pos: ({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
            f"  →  nav target: ({goal_x:.3f}, {goal_y:.3f}) yaw={goal_yaw:.3f} "
            f"frame={send_frame}"
        )

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    success = ctx.interface.navigate_to_pose(
        goal_x, goal_y, goal_yaw,
        frame_id=send_frame,
        timeout=timeout,
        poll_period=poll_period,
        time_now_fn=time_now_fn,
        sleep_fn=sleep_fn,
    )
    if not success:
        raise RuntimeError(
            f"Navigation to object approach failed "
            f"(frame={send_frame} goal=({goal_x:.3f}, {goal_y:.3f}) "
            f"approach_world=({nav_x:.3f}, {nav_y:.3f}))"
        )

    print("[Nav] Arrived at object approach position — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def skill_navigate_relative(
    ctx: _NavContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """按基座当前朝向做水平面内相对位移。

    使用 Isaac 查询的基座世界位姿；以当前前向 ``forward``（body +X 投影到世界 XY）、
    左向 ``left``（``(-forward_y, forward_x)``）构造目标：

    ``goal = pos + dx * forward + dy * left``，``goal_yaw = current_yaw + dyaw``。

    params:
        relative_position (list[float], 必填): 三元相对位移 ``[dx, dy, dyaw]``。
            - ``dx``: 沿当前前向(+) / 后向(-)位移（米）
            - ``dy``: 沿当前左向(+) / 右向(-)位移（米）
            - ``dyaw``: 在当前偏航基础上叠加旋转（弧度）
        frame_id (str, 可选): Nav2 目标帧，默认 ``map``。
        timeout (float, 可选): 默认 ``60.0``。
        poll_period (float, 可选): 默认 ``0.1``。
    """
    rel = params.get("relative_position", None)
    if not isinstance(rel, (list, tuple)) or len(rel) != 3:
        raise ValueError(
            "nav.navigate_relative requires params.relative_position=[dx, dy, dyaw] (len=3)"
        )
    relative_x = float(rel[0])
    relative_y = float(rel[1])
    relative_yaw = float(rel[2])
    frame_id = str(params.get("frame_id", "map"))
    timeout = float(params.get("timeout", 60.0))
    poll_period = float(params.get("poll_period", 0.1))

    base_path = getattr(ctx, "base_link_entity_path", None) or ""
    if not base_path:
        raise ValueError("nav.navigate_relative requires a resolved base_link_entity_path on context")

    (bx, by, _bz), quat = get_entity_pose_world_service(base_path)
    fx, fy, _fz = _quat_rotate_vector_xyzw(quat, (1.0, 0.0, 0.0))
    horiz = math.hypot(fx, fy)
    if horiz < 1e-6:
        fx, fy = 1.0, 0.0
        horiz = 1.0
    fx, fy = fx / horiz, fy / horiz
    lx, ly = -fy, fx
    gx = bx + relative_x * fx + relative_y * lx
    gy = by + relative_x * fy + relative_y * ly
    current_yaw = math.atan2(fy, fx)
    yaw = current_yaw + relative_yaw

    sim_time = getattr(ctx, "sim_time", None)
    time_now_fn = sim_time.now_seconds if sim_time is not None else None
    sleep_fn = sim_time.sleep if sim_time is not None else None

    print(
        f"[Nav] Relative move (dx={relative_x:.3f}, dy={relative_y:.3f}, dyaw={relative_yaw:.3f}) "
        f"from ({bx:.3f},{by:.3f}) → ({gx:.3f},{gy:.3f}) yaw={yaw:.3f}"
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
            "nav.navigate_relative failed "
            f"(target=({gx:.3f}, {gy:.3f}), relative_x={relative_x}, relative_y={relative_y}, "
            f"relative_yaw={relative_yaw})"
        )
    print("[Nav] Relative move complete — refreshing base world pose")
    _refresh_base_pose(ctx)
    return [], _default_meta(ctx)


def _register_navigation_skills() -> None:
    register_skill("nav.send_nav_goal", skill_send_nav_goal)
    register_skill("nav.wait_nav_arrived", skill_wait_nav_arrived)
    register_skill("nav.navigate_to_pose", skill_navigate_to_pose)
    register_skill("nav.navigate_to_object", skill_navigate_to_object)
    register_skill("nav.navigate_relative", skill_navigate_relative)


_register_navigation_skills()
