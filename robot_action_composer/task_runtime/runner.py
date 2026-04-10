"""Unified task queue runner: connect → FSM → ``task_queue`` (blocks + parallel)."""

from __future__ import annotations

import random
import signal
import sys
import threading
from collections.abc import Mapping
from typing import Any, Sequence

from ros2_robot_interface import FSM_HOLD, FSM_OCS2  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import execute_stage_sequence  # pyright: ignore[reportMissingImports]

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SimTimeHelper,
    get_entity_pose_world_service,
    randomize_object_xyz_after_reset,
    reset_simulation_and_randomize_object,
    set_prim_orientation_local,
)

from robot_action_composer.ros_interface_utils import (  # pyright: ignore[reportMissingImports]
    arm_handler_pose_or_raise,
    build_ros2_interface_from_robot_cfg,
)

from robot_action_composer.motion_generation.tasks.movej_return import (  # pyright: ignore[reportMissingImports]
    capture_initial_arm_joint_positions,
    capture_initial_body_joint_positions,
)
from robot_action_composer.motion_generation.tasks.drawer import euler_to_quaternion  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.merge.flat_presets import reset_fields_from_first_pick_block
from robot_action_composer.task_runtime.registry import get_skill
from robot_action_composer.task_runtime.config.merged import MergedQueueConfig
from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec, QueueBlock, block_spec_from_mapping


def _execute_block(
    ctx: Any,
    spec: BlockSpec,
    *,
    runner_prefix: str,
    idx_label: str,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
) -> None:
    skill_fn = get_skill(spec.skill)
    stages, meta = skill_fn(ctx, spec.params)
    label = f"{spec.skill!r}" if spec.id is None else f"{spec.skill!r} (id={spec.id!r})"
    if not stages:
        print(f"[{runner_prefix}] {idx_label} {label} -> (no stages / inline-only)")
        return
    print(f"[{runner_prefix}] {idx_label} {label} -> {len(stages)} stage(s)")
    exec_kw: dict[str, Any] = dict(
        interface=ctx.interface,
        sequence=stages,
        send_mode=meta.send_mode,
        frame_id=meta.frame_id,
        arrival_timeout=ctx.robot_cfg.arrival_timeout,
        arrival_poll=ctx.robot_cfg.arrival_poll,
        time_now_fn=ctx.sim_time.now_seconds,
        sleep_fn=ctx.sim_time.sleep,
        gripper_action_wait=ctx.robot_cfg.gripper_action_wait,
        left_arrival_guard_stage=meta.left_arrival_guard_stage,
        warn_prefix=meta.warn_prefix,
    )
    if execute_stage_kwargs:
        exec_kw.update(dict(execute_stage_kwargs))
    execute_stage_sequence(**exec_kw)


def _execute_parallel(
    ctx: Any,
    par_spec: ParallelSpec,
    *,
    runner_prefix: str,
    idx_label: str,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
) -> None:
    n = len(par_spec.skills)
    print(f"[{runner_prefix}] {idx_label} parallel({n}) -> dispatching {n} skill(s)")
    errors: list[BaseException] = []
    lock = threading.Lock()

    def worker(sub_spec: BlockSpec, sub_label: str) -> None:
        try:
            _execute_block(
                ctx,
                sub_spec,
                runner_prefix=runner_prefix,
                idx_label=sub_label,
                execute_stage_kwargs=execute_stage_kwargs,
            )
        except Exception as exc:  # noqa: BLE001
            with lock:
                errors.append(exc)

    threads = [
        threading.Thread(
            target=worker,
            args=(sub_spec, f"{idx_label}[{i}]"),
            daemon=True,
        )
        for i, sub_spec in enumerate(par_spec.skills)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    if errors:
        raise errors[0]
    print(f"[{runner_prefix}] {idx_label} parallel({n}) -> all done")


def _make_interface(robot_cfg: Any) -> Any:
    return build_ros2_interface_from_robot_cfg(robot_cfg)


def _optional_arm_home_pose(handler: Any, *, label: str) -> Any | None:
    """单臂台架机器人可能只有一侧 :attr:`interface.*_arm_handler`；无 handler 或尚无位姿时返回 ``None``。"""
    if handler is None:
        return None
    pose = handler.get_pose()
    if pose is None:
        print(f"[TaskQ] WARN: no EE pose yet for {label} (optional arm); leaving home_pose=None")
        return None
    return pose


def _resolve_gripper_open_close(mode: str) -> tuple[float, float]:
    if mode == "target_command":
        return 1.0, 0.0
    raise ValueError(
        "task queue only supports gripper_control_mode='target_command' "
        "without explicit gripper values"
    )


def build_queue_runtime_context(
    *,
    interface: Any,
    robot_cfg: Any,
    sim_time: Any,
    runtime: MergedQueueConfig,
    use_stamped: bool,
    left_initial_joint_positions: list[float] | None,
    right_initial_joint_positions: list[float] | None,
    body_initial_joint_positions: list[float] | None,
) -> QueueRuntimeContext:
    """FSM / connect 之后构造 :class:`QueueRuntimeContext`。"""
    queue_task = runtime.single_arm
    gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
    frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"
    base_world_pos, base_world_quat = get_entity_pose_world_service(robot_cfg.base_link_entity_path)

    initial_arm = queue_task.common.arm.strip().lower()
    if initial_arm not in {"left", "right"}:
        raise ValueError("arm must be 'left' or 'right'")
    pick_is_right = initial_arm == "right"
    source_ee_prefix = "right_ee" if pick_is_right else "left_ee"
    source_handler = interface.right_arm_handler if pick_is_right else interface.left_arm_handler
    source_home_pose = arm_handler_pose_or_raise(source_handler, label=source_ee_prefix)

    left_handler = interface.left_arm_handler
    right_handler = interface.right_arm_handler
    need_dual_arm_homes = runtime.carry is not None or runtime.handover is not None
    if need_dual_arm_homes:
        left_home_pose = arm_handler_pose_or_raise(left_handler, label="left_ee")
        right_home_pose = arm_handler_pose_or_raise(right_handler, label="right_ee")
    else:
        left_home_pose = _optional_arm_home_pose(left_handler, label="left_ee")
        right_home_pose = _optional_arm_home_pose(right_handler, label="right_ee")

    return QueueRuntimeContext(
        interface=interface,
        robot_cfg=robot_cfg,
        sim_time=sim_time,
        task_cfg=queue_task,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        use_stamped=use_stamped,
        frame_id=frame_id,
        source_home_pose=source_home_pose,
        base_world_pos=base_world_pos,
        base_world_quat=base_world_quat,
        gripper_for_return_home=gripper_closed,
        left_initial_joint_positions=left_initial_joint_positions,
        right_initial_joint_positions=right_initial_joint_positions,
        body_initial_joint_positions=body_initial_joint_positions,
        left_home_pose=left_home_pose,
        right_home_pose=right_home_pose,
        carry_task_cfg=runtime.carry,
        handover_sync=runtime.handover,
        drawer_geometry=runtime.drawer,
    )


def reset_queue_task_environment(
    *,
    specs: Sequence[QueueBlock],
    runtime: MergedQueueConfig,
    robot_cfg: Any,
    sim_time: SimTimeHelper,
) -> None:
    if runtime.drawer is not None:
        dcfg = runtime.drawer
        reset_source_path, reset_xyz_offset = reset_fields_from_first_pick_block(specs, runtime.single_arm)
        apple_path = reset_source_path
        drawer_prim = dcfg.source_object_path_drawer
        drawer_all = dcfg.source_object_path_drawer_all
        if not apple_path or not drawer_prim or not drawer_all:
            raise ValueError(
                "drawer task queue requires source_object_entity_path (task_cfg or single_arm.pick params), "
                "source_object_path_drawer, source_object_path_drawer_all"
            )
        reset_simulation_and_randomize_object(
            apple_path,
            xyz_offset=reset_xyz_offset,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        rot_z = random.uniform(-0.1, 0.1)
        set_prim_orientation_local(drawer_all, euler_to_quaternion(0, 0, rot_z))
        randomize_object_xyz_after_reset(
            drawer_all,
            xyz_offset=dcfg.object_xyz_random_offset_drawer,
        )
        sim_time.sleep(1)
        return
    if runtime.carry is not None:
        reset_simulation_and_randomize_object(
            runtime.carry.source_object_entity_path,
            xyz_offset=runtime.carry.object_xyz_random_offset,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        return
    if runtime.handover is not None:
        pk = runtime.single_arm.pick
        if not pk.source_object_entity_path:
            raise ValueError(
                "handover task env reset requires source_object_entity_path on single_arm.pick "
                "(skill_defaults.single_arm.pick or skill_params for pick block)"
            )
        reset_simulation_and_randomize_object(
            pk.source_object_entity_path,
            xyz_offset=pk.object_xyz_random_offset,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        return
    reset_source_path, reset_xyz_offset = reset_fields_from_first_pick_block(specs, runtime.single_arm)
    if not reset_source_path:
        raise ValueError(
            "source_object_entity_path is required for env reset "
            "(set on task_cfg or under skill_defaults / skill_params for single_arm.pick)"
        )
    reset_simulation_and_randomize_object(
        reset_source_path,
        xyz_offset=reset_xyz_offset,
        post_reset_wait=robot_cfg.post_reset_wait,
        sleep_fn=sim_time.sleep,
    )


def run_task_queue(
    *,
    robot_cfg: Any,
    runtime: MergedQueueConfig,
    robot_id: str,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
) -> None:
    """Single entry: connect → FSM → execute ``task_queue`` blocks (and parallel groups)."""
    queue_tc = runtime.single_arm

    specs: list[QueueBlock] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
    if not specs:
        raise ValueError("task queue blocks list is empty")

    interface = _make_interface(robot_cfg)
    sim_time = SimTimeHelper()
    robot_connected = False

    def shutdown_handler(sig, frame) -> None:  # noqa: ARG001
        try:
            if robot_connected:
                interface.disconnect()
        finally:
            sim_time.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        interface.connect()
        robot_connected = True
        print("[OK] Robot connected (unified task queue)")
        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(interface)
        body_initial_joint_positions = capture_initial_body_joint_positions(interface)

        interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        interface.send_fsm_command(FSM_OCS2)

        if reset_env:
            reset_queue_task_environment(specs=specs, runtime=runtime, robot_cfg=robot_cfg, sim_time=sim_time)

        ctx = build_queue_runtime_context(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            runtime=runtime,
            use_stamped=use_stamped,
            left_initial_joint_positions=left_initial_joint_positions,
            right_initial_joint_positions=right_initial_joint_positions,
            body_initial_joint_positions=body_initial_joint_positions,
        )

        for idx, spec in enumerate(specs):
            lbl = f"block {idx + 1}/{len(specs)}"
            if isinstance(spec, ParallelSpec):
                _execute_parallel(
                    ctx,
                    spec,
                    runner_prefix="TaskQ",
                    idx_label=lbl,
                    execute_stage_kwargs=execute_stage_kwargs,
                )
            else:
                _execute_block(
                    ctx,
                    spec,
                    runner_prefix="TaskQ",
                    idx_label=lbl,
                    execute_stage_kwargs=execute_stage_kwargs,
                )

        interface.send_fsm_command(FSM_HOLD)
        print("[OK] Task queue completed")
    except Exception as err:
        print(f"[ERROR] Task queue failed: {err}")
    finally:
        sim_time.shutdown()
        if robot_connected:
            interface.disconnect()
            print("[OK] Robot disconnected")