"""抽屉相关 ``single_arm.drawer.*`` 技能（拉手 / 关抽屉 / 撤退）。

``pull_open`` 计算把手参考系、执行拉开序列，并改写 ``ctx.task_cfg.place`` 的苹果中间放置提示。
拉开末段距离由 :class:`~robot_action_composer.motion_generation.tasks.drawer.DrawerGeometryConfig`
的 ``pull_distance``（``single_arm.drawer``）决定。
拉开序列末尾若配置了非零 ``single_arm.drawer.ee_retreat_offset``，会追加与 ``place`` 同构的松爪 + 撤出段。
阶段状态在 ``ctx.drawer``（:class:`DrawerPhaseState`）；几何配置在 ``ctx.drawer_geometry``。

末端朝向复用 :func:`~robot_action_composer.motion_generation.tasks.object_orientation.compose_aligned_ee_orientation`
（默认 ``ee_orientation_frame=object``、``object_orientation_mode=full``、``aligned_object_yaw=0``）。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from geometry_msgs.msg import Pose  # pyright: ignore[reportMissingImports]

from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    pose_from_tuple,
    quat_conjugate,
    quat_multiply,
    quat_normalize,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    SendMode,
    StageTarget,
    execute_stage_sequence,
    pose_tol_ori_to_orient_deg,
)

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_object_pose_from_service,
)
from robot_action_composer.motion_generation.tasks.object_orientation import (  # pyright: ignore[reportMissingImports]
    compose_aligned_ee_orientation,
)
from robot_action_composer.task_runtime.object_binding import (
    resolve_grasp_prim_path,
    resolve_object_local_orient_from_grasp_prim,
    resolve_pick_object_binding,
)
from robot_action_composer.task_runtime.object_resolution_replay import resolve_object_pose_for_task

from robot_action_composer.motion_generation.tasks.drawer import (  # pyright: ignore[reportMissingImports]
    DrawerGeometryConfig,
    _apply_target_pose_offset,
    build_single_arm_close_drawer_sequence,
    build_single_arm_pull_drawer_sequence,
)
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
)
from robot_action_composer.task_runtime.context import (
    DrawerPhaseState,
    QueueRuntimeContext,
    queue_pick_arm_is_right,
)
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.types import ExecutionMeta


def _stamped_mode(ctx: QueueRuntimeContext) -> SendMode:
    return SendMode.STAMPED if ctx.use_stamped else SendMode.UNSTAMPED


def _require_drawer_geometry(ctx: QueueRuntimeContext) -> DrawerGeometryConfig:
    d = ctx.drawer_geometry
    if d is None:
        raise TypeError(
            "single_arm.drawer.* skills require drawer_geometry "
            "(set single_arm.drawer.object_prim_path / object_key and drawer fields in task YAML overlays)"
        )
    return d


def _as_vec3(raw: Any, *, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if raw is None:
        return default
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        return (float(raw[0]), float(raw[1]), float(raw[2]))
    raise TypeError(f"expected length-3 [x,y,z], got {raw!r}")


def _as_quat(raw: Any, *, default: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    if raw is None:
        return default
    if isinstance(raw, (list, tuple)) and len(raw) == 4:
        return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))
    raise TypeError(f"expected length-4 xyzw quat, got {raw!r}")


def _resolve_drawer_handle_binding(
    ctx: QueueRuntimeContext, dcfg: DrawerGeometryConfig
) -> tuple[str, tuple[float, float, float]]:
    """Resolve drawer layer prim + handle offset (``grasp_id`` / legacy ``object_position_offset``)."""
    params: dict[str, Any] = {
        "object_prim_path": dcfg.object_prim_path,
        "object_key": dcfg.object_key,
        "grasp_id": dcfg.grasp_id,
        "grasp_prim_path": dcfg.grasp_prim_path,
    }
    has_grasp = bool(str(dcfg.grasp_id or "").strip() or str(dcfg.grasp_prim_path or "").strip())
    off = _as_vec3(dcfg.object_position_offset, default=(0.0, 0.0, 0.0))
    if not has_grasp or off != (0.0, 0.0, 0.0):
        params["object_position_offset"] = off
    return resolve_pick_object_binding(
        params,
        objects=getattr(ctx, "objects", None),
        active_object=None,
        cache=getattr(ctx, "grasp_offset_cache", None),
    )


def _rotate_vector_by_quat(
    vec: tuple[float, float, float], quat_xyzw: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    q = quat_normalize(quat_xyzw)
    q_inv = quat_conjugate(q)
    v_quat = (vec[0], vec[1], vec[2], 0.0)
    rotated = quat_multiply(quat_multiply(q, v_quat), q_inv)
    return (rotated[0], rotated[1], rotated[2])


def _pose_orientation_xyzw(pose: Pose) -> tuple[float, float, float, float]:
    return (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def _clone_pose_with_orientation(
    pose: Pose, quat_xyzw: tuple[float, float, float, float]
) -> Pose:
    out = Pose()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    qx, qy, qz, qw = quat_normalize(quat_xyzw)
    out.orientation.x = qx
    out.orientation.y = qy
    out.orientation.z = qz
    out.orientation.w = qw
    return out


def _pick_arm_side(ctx: QueueRuntimeContext) -> ArmSide:
    return ArmSide.RIGHT if queue_pick_arm_is_right(ctx) else ArmSide.LEFT


def _primary_arm_handler(ctx: QueueRuntimeContext) -> Any:
    return (
        ctx.interface.right_arm_handler
        if queue_pick_arm_is_right(ctx)
        else ctx.interface.left_arm_handler
    )


def _pose_shift_world(pose: Pose, delta_world: tuple[float, float, float]) -> Pose:
    dx, dy, dz = delta_world
    out = Pose()
    out.orientation = pose.orientation
    out.position.x = float(pose.position.x) + dx
    out.position.y = float(pose.position.y) + dy
    out.position.z = float(pose.position.z) + dz
    return out


def _close_push_release_and_tool_retreat(
    ctx: QueueRuntimeContext,
    *,
    dcfg: DrawerGeometryConfig,
    sequence: list[StageTarget],
    arm_side: ArmSide,
    handler: Any,
    ee_base_orientation: tuple[float, float, float, float],
) -> None:
    """在关抽屉笛卡尔段与（可选）拉手参考到位之后执行：松爪 + ``single_arm.drawer.ee_retreat_offset`` 工具系撤出。"""
    rt = dcfg.ee_retreat_offset
    if rt is None or not any(abs(float(x)) > 1e-12 for x in rt):
        return
    rcfg = ctx.robot_cfg
    gh = (
        ctx.interface.right_gripper_handler
        if queue_pick_arm_is_right(ctx)
        else ctx.interface.left_gripper_handler
    )
    if gh:
        gh.send_target_command(int(ctx.gripper_open))
    ctx.sim_time.sleep(rcfg.gripper_action_wait)

    world_d = _rotate_vector_by_quat(_as_vec3(rt, default=(0.0, 0.0, 0.0)), ee_base_orientation)
    last = sequence[-1]
    arm_t = last.right if arm_side == ArmSide.RIGHT else last.left
    if arm_t is None:
        return
    retreat_pose = _pose_shift_world(arm_t.pose, world_d)
    if _stamped_mode(ctx) == SendMode.STAMPED:
        handler.send_target_stamped(ctx.frame_id, retreat_pose)
    else:
        handler.send_target(retreat_pose)
    _wait_pick_arm_arrive(ctx)


def _wait_pick_arm_arrive(ctx: QueueRuntimeContext) -> None:
    """与 :func:`execute_stage_sequence` 一致：粗定位下发后必须等到位再跑后续段（否则会先收到 Grasp）。"""
    rcfg = ctx.robot_cfg
    tc = ctx.task_cfg
    pose_tol_pos = pose_tol_ori = None
    if hasattr(tc, "common"):
        pose_tol_pos = getattr(tc.common, "pose_tol_pos", None)
        pose_tol_ori = getattr(tc.common, "pose_tol_ori", None)
    part = "right_arm" if queue_pick_arm_is_right(ctx) else "left_arm"
    ctx.interface.wait_until_arrive(
        part=part,
        timeout=rcfg.arrival_timeout,
        poll_period=rcfg.arrival_poll,
        time_now_fn=ctx.sim_time.now_seconds,
        sleep_fn=ctx.sim_time.sleep,
        arm_pose_threshold=pose_tol_pos,
        arm_orient_threshold=pose_tol_ori_to_orient_deg(pose_tol_ori),
    )


def _require_drawer_phase(ctx: QueueRuntimeContext) -> DrawerPhaseState:
    d = ctx.drawer
    if d is None:
        raise RuntimeError("Drawer phase state missing; run single_arm.drawer.pull_open first")
    return d


def _resolve_drawer_source_pose(ctx: QueueRuntimeContext, *, path_drawer: str) -> Pose:
    arm_label = ctx.task_cfg.common.arm.strip().lower()
    if ctx.object_resolution is not None:
        return resolve_object_pose_for_task(
            ctx,
            object_prim_path=path_drawer,
            include_orientation=True,
            arm_side=arm_label,
            object_role="drawer_source",
            entity_state_timeout=SERVICE_CALL_TIMEOUT,
            retries=SERVICE_CALL_RETRIES,
            retry_delay=SERVICE_RETRY_DELAY,
        )
    return get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path_drawer,
        include_orientation=True,
    )


def _resolve_align_object_pose(
    ctx: QueueRuntimeContext,
    *,
    dcfg: DrawerGeometryConfig,
    path_drawer: str,
    drawer_pose: Pose,
) -> Pose:
    """Pose whose orientation feeds ``compose_aligned_ee_orientation`` (position unused)."""
    orient_from = str(dcfg.orient_from or "handle").strip().lower()
    if orient_from in ("", "drawer", "body", "cabinet"):
        return drawer_pose
    if orient_from not in ("handle", "grasp", "handle_pose"):
        raise ValueError(
            f"unsupported single_arm.drawer.orient_from={dcfg.orient_from!r}; "
            "expected 'handle' or 'drawer'"
        )
    grasp_prim = resolve_grasp_prim_path(
        {
            "object_prim_path": path_drawer,
            "object_key": dcfg.object_key,
            "grasp_id": dcfg.grasp_id,
            "grasp_prim_path": dcfg.grasp_prim_path,
        },
        objects=getattr(ctx, "objects", None),
        object_prim_path=path_drawer,
    )
    if not grasp_prim:
        print(
            "[TaskQ] WARN: orient_from=handle but no grasp_id/grasp_prim_path; "
            "falling back to drawer body orientation"
        )
        return drawer_pose
    q_drawer = _pose_orientation_xyzw(drawer_pose)
    q_local = resolve_object_local_orient_from_grasp_prim(path_drawer, grasp_prim)
    q_handle = quat_normalize(quat_multiply(q_drawer, q_local))
    return _clone_pose_with_orientation(drawer_pose, q_handle)


def skill_drawer_pull_open(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    dcfg = _require_drawer_geometry(ctx)
    path_drawer, handle_local = _resolve_drawer_handle_binding(ctx, dcfg)
    if not path_drawer:
        raise ValueError(
            "object_prim_path / object_key is required for single_arm.drawer.pull_open "
            "(single_arm.drawer)"
        )

    # 刚体位姿：朝向用于拉开轴；拉手位置用局部偏移旋入（勿把偏移后的位姿当对齐参考）。
    drawer_pose = _resolve_drawer_source_pose(ctx, path_drawer=path_drawer)
    q_drawer = _pose_orientation_xyzw(drawer_pose)

    align_pose = _resolve_align_object_pose(
        ctx, dcfg=dcfg, path_drawer=path_drawer, drawer_pose=drawer_pose
    )
    ee_base = _as_quat(dcfg.ee_base_orientation, default=(0.5, 0.5, 0.5, -0.5))
    ee_base_ori_drawer = compose_aligned_ee_orientation(
        ee_base,
        align_pose,
        ee_orientation_frame=str(dcfg.ee_orientation_frame or "object"),
        object_orientation_mode=str(dcfg.object_orientation_mode or "full"),
        aligned_object_yaw=dcfg.aligned_object_yaw,
        object_prim_path=path_drawer,
    )

    pull_axis = _as_vec3(dcfg.pull_axis_local, default=(0.0, -1.0, 0.0))
    dir_drawer = _rotate_vector_by_quat(pull_axis, q_drawer)

    # 拉手目标点：抽屉刚体位置 + 物体系偏移（grasp / legacy offset）
    source_target_pose_d = Pose()
    source_target_pose_d.position.x = float(drawer_pose.position.x)
    source_target_pose_d.position.y = float(drawer_pose.position.y)
    source_target_pose_d.position.z = float(drawer_pose.position.z)
    source_target_pose_d.orientation = drawer_pose.orientation
    handle_offset = _rotate_vector_by_quat(handle_local, q_drawer)
    _apply_target_pose_offset(source_target_pose_d, handle_offset)
    place_pose_ref = (
        float(source_target_pose_d.position.x),
        float(source_target_pose_d.position.y),
        float(source_target_pose_d.position.z),
    )

    ctx.drawer = DrawerPhaseState(
        place_pose_ref=place_pose_ref,
        ee_base_orientation_xyzw=ee_base_ori_drawer,
        pull_direction_xyz=dir_drawer,
    )

    tc = ctx.task_cfg
    if not isinstance(tc, QueueSingleArmSlice):
        raise TypeError(f"drawer pull_open expects QueueSingleArmSlice on ctx.task_cfg, got {type(tc)}")
    pull_dist = float(dcfg.pull_distance)

    gripper_open = ctx.gripper_open
    gripper_closed = ctx.gripper_closed
    sequence = build_single_arm_pull_drawer_sequence(
        target_pose=source_target_pose_d,
        drawer=dcfg,
        arm_side=_pick_arm_side(ctx),
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        ee_base_orientation=ee_base_ori_drawer,
        pull_direction_xyz=dir_drawer,
        pull_distance=pull_dist,
    )

    apple_place = (
        place_pose_ref[0] + dir_drawer[0] * 0.04,
        place_pose_ref[1] + dir_drawer[1] * 0.04,
        place_pose_ref[2] + 0.15,
    )
    ctx.task_cfg = replace(
        tc,
        place=replace(
            tc.place,
            place_position=apple_place,
            object_prim_path="",
        ),
    )
    ctx.gripper_for_return_home = ctx.gripper_closed
    print(f"[TaskQ] drawer pull ref (apple place hint) -> place_position={apple_place}")

    return sequence, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ drawer pull timeout",
    )


def skill_drawer_close_push(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    dcfg = _require_drawer_geometry(ctx)
    drw = _require_drawer_phase(ctx)
    tc = ctx.task_cfg
    if not isinstance(tc, QueueSingleArmSlice):
        raise TypeError(f"drawer close_push expects QueueSingleArmSlice on ctx.task_cfg, got {type(tc)}")

    path_drawer, handle_local = _resolve_drawer_handle_binding(ctx, dcfg)
    place_pose_ref = drw.place_pose_ref
    ee_base_ori = drw.ee_base_orientation_xyzw
    dir_vec = drw.pull_direction_xyz

    source_target_pose_d = _resolve_drawer_source_pose(ctx, path_drawer=path_drawer)
    close_delta = _as_quat(dcfg.close_ee_delta_xyzw, default=(0.0, -0.2164396, 0.0, 0.976296))
    ee_base_ori = quat_normalize(quat_multiply(ee_base_ori, close_delta))
    drw.ee_base_orientation_xyzw = ee_base_ori

    handle_off = _rotate_vector_by_quat(
        handle_local,
        _pose_orientation_xyzw(source_target_pose_d),
    )
    _apply_target_pose_offset(source_target_pose_d, handle_off)
    handler = _primary_arm_handler(ctx)
    if handler is None:
        raise RuntimeError("No arm handler for drawer close staging move")

    arm_side = _pick_arm_side(ctx)
    sequence = build_single_arm_close_drawer_sequence(
        target_pose=source_target_pose_d,
        drawer=dcfg,
        arm_side=arm_side,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        ee_base_orientation=ee_base_ori,
        pull_direction_xyz=dir_vec,
    )
    if not sequence:
        raise RuntimeError("close_push: empty close-drawer cartesian sequence")
    st0 = sequence[0]
    arm0 = st0.right if arm_side == ArmSide.RIGHT else st0.left
    if arm0 is None:
        raise RuntimeError("close_push: first stage has no target for the pick arm")
    # 粗定位必须与笛卡尔首段一致；帧名需与序列执行一致，否则到位判定与后续段错位。
    # 无 prepare 时 sequence[1:] 首段为 Grasp：若此处仅用 sleep、未到 Close-in 就会先闭合夹爪。
    staging_frame = ctx.frame_id
    if _stamped_mode(ctx) == SendMode.STAMPED:
        handler.send_target_stamped(staging_frame, arm0.pose)
    else:
        handler.send_target(arm0.pose)
    _wait_pick_arm_arrive(ctx)

    rcfg = ctx.robot_cfg
    # 与 runner._execute_block 一致：必须传入 common 位姿阈值，否则 wait_until_arrive 用接口默认，
    # Close-in 易被误判已到位，下一档 Grasp 会过早闭合。
    execute_stage_sequence(
        interface=ctx.interface,
        sequence=sequence[1:],
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        arrival_timeout=rcfg.arrival_timeout,
        arrival_poll=rcfg.arrival_poll,
        time_now_fn=ctx.sim_time.now_seconds,
        sleep_fn=ctx.sim_time.sleep,
        gripper_action_wait=rcfg.gripper_action_wait,
        warn_prefix="TaskQ drawer close sequence timeout",
        pose_tol_pos=tc.common.pose_tol_pos,
        pose_tol_ori=tc.common.pose_tol_ori,
    )

    # 先保持夹爪闭合移到 pull_open 记录的拉手参考位（与推关抽屉同一套末端姿态），再松爪 + ee_retreat_offset。
    # 若先松爪后撤再「合上」，会在抽屉尚未离开拉手区间时显得像「关抽屉前就张开后退」。
    if _stamped_mode(ctx) == SendMode.STAMPED:
        handler.send_target_stamped(ctx.frame_id, pose_from_tuple(place_pose_ref, ee_base_ori))
    else:
        handler.send_target(pose_from_tuple(place_pose_ref, ee_base_ori))
    _wait_pick_arm_arrive(ctx)

    _close_push_release_and_tool_retreat(
        ctx,
        dcfg=dcfg,
        sequence=sequence,
        arm_side=arm_side,
        handler=handler,
        ee_base_orientation=ee_base_ori,
    )

    return [], ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ drawer close inline complete",
    )


def register_drawer_skills() -> None:
    register_skill("single_arm.drawer.pull_open", skill_drawer_pull_open)
    register_skill("single_arm.drawer.close_push", skill_drawer_close_push)


register_drawer_skills()
