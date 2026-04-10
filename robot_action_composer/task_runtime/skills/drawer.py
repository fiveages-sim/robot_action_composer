"""抽屉相关 ``single_arm.drawer.*`` 技能（拉手 / 关抽屉 / 撤退）。

``pull_open`` 计算把手参考系、执行拉开序列，并改写 ``ctx.task_cfg.place`` 的苹果中间放置提示。
阶段状态在 ``ctx.drawer``（:class:`DrawerPhaseState`）；几何配置在 ``ctx.drawer_geometry``。
"""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from lerobot_robot_ros2.utils.pose_utils import (  # pyright: ignore[reportMissingImports]
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
)

from robot_action_composer.isaac_sim import get_object_pose_from_service  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.tasks.drawer import (  # pyright: ignore[reportMissingImports]
    DrawerGeometryConfig,
    _apply_target_pose_offset,
    build_single_arm_back_home_sequence,
    build_single_arm_close_drawer_sequence,
    build_single_arm_pull_drawer_sequence,
)
from robot_action_composer.task_runtime.config.single_arm import QueueSingleArmSlice  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.context import (
    DrawerPhaseState,
    QueueRuntimeContext,
    queue_pick_arm_is_right,
    queue_primary_ee_frame_id,
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
            "(set source_object_path_drawer and drawer fields in task YAML / merged flat)"
        )
    return d


def _rotate_vector_by_quat(
    vec: tuple[float, float, float], quat_xyzw: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    q = quat_normalize(quat_xyzw)
    q_inv = quat_conjugate(q)
    v_quat = (vec[0], vec[1], vec[2], 0.0)
    rotated = quat_multiply(quat_multiply(q, v_quat), q_inv)
    return (rotated[0], rotated[1], rotated[2])


def _pick_arm_side(ctx: QueueRuntimeContext) -> ArmSide:
    return ArmSide.RIGHT if queue_pick_arm_is_right(ctx) else ArmSide.LEFT


def _primary_arm_handler(ctx: QueueRuntimeContext) -> Any:
    return (
        ctx.interface.right_arm_handler
        if queue_pick_arm_is_right(ctx)
        else ctx.interface.left_arm_handler
    )


def _require_drawer_phase(ctx: QueueRuntimeContext) -> DrawerPhaseState:
    d = ctx.drawer
    if d is None:
        raise RuntimeError("Drawer phase state missing; run single_arm.drawer.pull_open first")
    return d


def skill_drawer_pull_open(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    dcfg = _require_drawer_geometry(ctx)
    path_drawer = dcfg.source_object_path_drawer
    if not path_drawer:
        raise ValueError("source_object_path_drawer is required for single_arm.drawer.pull_open")

    source_target_pose_d = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path_drawer,
        include_orientation=True,
    )
    hx = (dcfg.handle_extent_max[0] + dcfg.handle_extent_min[0]) / 2 * dcfg.drawer_scale
    hy = (dcfg.handle_extent_max[1] + dcfg.handle_extent_min[1]) / 2 * dcfg.drawer_scale
    hz = (dcfg.handle_extent_max[2] + dcfg.handle_extent_min[2]) / 2 * dcfg.drawer_scale
    handle_offset = _rotate_vector_by_quat(
        (hx, hy, hz),
        (
            float(source_target_pose_d.orientation.x),
            float(source_target_pose_d.orientation.y),
            float(source_target_pose_d.orientation.z),
            float(source_target_pose_d.orientation.w),
        ),
    )
    _apply_target_pose_offset(source_target_pose_d, handle_offset)
    place_pose_ref = (
        float(source_target_pose_d.position.x),
        float(source_target_pose_d.position.y),
        float(source_target_pose_d.position.z),
    )
    grasp_ori_drawer = quat_multiply(
        (
            float(source_target_pose_d.orientation.x),
            float(source_target_pose_d.orientation.y),
            float(source_target_pose_d.orientation.z),
            float(source_target_pose_d.orientation.w),
        ),
        (0.5, 0.5, 0.5, -0.5),
    )
    dir_drawer = _rotate_vector_by_quat(
        (0, -1, 0),
        (
            float(source_target_pose_d.orientation.x),
            float(source_target_pose_d.orientation.y),
            float(source_target_pose_d.orientation.z),
            float(source_target_pose_d.orientation.w),
        ),
    )

    ctx.drawer = DrawerPhaseState(
        place_pose_ref=place_pose_ref,
        grasp_orientation_xyzw=grasp_ori_drawer,
        grasp_direction_vector=dir_drawer,
        handle_offset_rotated=handle_offset,
    )

    tc = ctx.task_cfg
    if not isinstance(tc, QueueSingleArmSlice):
        raise TypeError(f"drawer pull_open expects QueueSingleArmSlice on ctx.task_cfg, got {type(tc)}")

    gripper_open = ctx.gripper_open
    gripper_closed = ctx.gripper_closed
    sequence = build_single_arm_pull_drawer_sequence(
        target_pose=source_target_pose_d,
        pick=tc.pick,
        drawer=dcfg,
        arm_side=_pick_arm_side(ctx),
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        grasp_orientation=grasp_ori_drawer,
        grasp_direction_vector=dir_drawer,
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
            place_object_entity_path="",
            run_place_before_return=True,
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

    path_drawer = dcfg.source_object_path_drawer
    place_pose_ref = drw.place_pose_ref
    grasp_ori = drw.grasp_orientation_xyzw
    dir_vec = drw.grasp_direction_vector
    handle_off = drw.handle_offset_rotated

    source_target_pose_d = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path_drawer,
        include_orientation=True,
    )
    grasp_ori = quat_multiply(grasp_ori, (0, -0.2164396, 0, 0.976296))
    drw.grasp_orientation_xyzw = grasp_ori

    _apply_target_pose_offset(source_target_pose_d, handle_off)
    handler = _primary_arm_handler(ctx)
    if handler is None:
        raise RuntimeError("No arm handler for drawer close staging move")
    staging_pose = pose_from_tuple(
        (
            float(source_target_pose_d.position.x),
            float(source_target_pose_d.position.y),
            float(source_target_pose_d.position.z),
        ),
        grasp_ori,
    )
    handler.send_target_stamped("base_link", staging_pose)
    time.sleep(3)

    sequence = build_single_arm_close_drawer_sequence(
        target_pose=source_target_pose_d,
        pick=tc.pick,
        drawer=dcfg,
        arm_side=_pick_arm_side(ctx),
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        grasp_orientation=grasp_ori,
        grasp_direction_vector=dir_vec,
    )
    rcfg = ctx.robot_cfg
    execute_stage_sequence(
        interface=ctx.interface,
        sequence=sequence,
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        arrival_timeout=rcfg.arrival_timeout,
        arrival_poll=rcfg.arrival_poll,
        time_now_fn=ctx.sim_time.now_seconds,
        sleep_fn=ctx.sim_time.sleep,
        gripper_action_wait=rcfg.gripper_action_wait,
        warn_prefix="TaskQ drawer close sequence timeout",
    )

    handler.send_target_stamped("base_link", pose_from_tuple(place_pose_ref, grasp_ori))
    time.sleep(3)

    return [], ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ drawer close inline complete",
    )


def skill_drawer_retreat_to_home(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    _require_drawer_geometry(ctx)
    drw = _require_drawer_phase(ctx)
    tc = ctx.task_cfg
    if not isinstance(tc, QueueSingleArmSlice):
        raise TypeError(f"drawer retreat_to_home expects QueueSingleArmSlice on ctx.task_cfg, got {type(tc)}")

    sequence = build_single_arm_back_home_sequence(
        place_position=drw.place_pose_ref,
        home_pose=ctx.source_home_pose,
        place=tc.place,
        arm_side=_pick_arm_side(ctx),
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        grasp_orientation=drw.grasp_orientation_xyzw,
    )
    if ctx.use_stamped:
        for st in sequence:
            if "ReturnHome" in st.name:
                st.frame_id = queue_primary_ee_frame_id(ctx)

    ctx.gripper_for_return_home = ctx.gripper_open
    return sequence, ExecutionMeta(
        send_mode=_stamped_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ drawer retreat timeout",
    )


def register_drawer_skills() -> None:
    register_skill("single_arm.drawer.pull_open", skill_drawer_pull_open)
    register_skill("single_arm.drawer.close_push", skill_drawer_close_push)
    register_skill("single_arm.drawer.retreat_to_home", skill_drawer_retreat_to_home)


register_drawer_skills()
