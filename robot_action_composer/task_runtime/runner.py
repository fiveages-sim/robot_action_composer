"""Execute a linear single-arm task queue (vendor-agnostic skills: ``single_arm.*``)."""

from __future__ import annotations

import random
import signal
import sys
import threading
from typing import Any, Sequence

from ros2_robot_interface import FSM_HOLD, FSM_OCS2  # pyright: ignore[reportMissingImports]

from robot_action_composer.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    execute_stage_sequence,
)

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

from robot_action_composer.motion_generation.movej_return import (  # pyright: ignore[reportMissingImports]
    capture_initial_arm_joint_positions,
    capture_initial_body_joint_positions,
)
from robot_action_composer.motion_generation.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.drawer import DrawerPickPlaceTaskConfig, euler_to_quaternion  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.handover import HandoverTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.pick_place import PickPlaceFlowTaskConfig  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import BimanualMotionContext, HandoverMotionContext, SingleArmMotionContext
from robot_action_composer.task_runtime.registry import get_skill
from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec, QueueBlock, block_spec_from_mapping


def _execute_block(
    ctx: Any,
    spec: BlockSpec,
    *,
    runner_prefix: str,
    idx_label: str,
) -> None:
    """执行单个 BlockSpec：调用 skill 函数，若有 stages 则驱动 execute_stage_sequence。

    导航类 skill 在 skill_fn 内部阻塞完成，返回空 stages；
    运动类 skill 返回 stages 后由 execute_stage_sequence 驱动。
    """
    skill_fn = get_skill(spec.skill)
    stages, meta = skill_fn(ctx, spec.params)
    label = f"{spec.skill!r}" if spec.id is None else f"{spec.skill!r} (id={spec.id!r})"
    if not stages:
        print(f"[{runner_prefix}] {idx_label} {label} -> (no stages / inline-only)")
        return
    print(f"[{runner_prefix}] {idx_label} {label} -> {len(stages)} stage(s)")
    execute_stage_sequence(
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


def _execute_parallel(
    ctx: Any,
    par_spec: ParallelSpec,
    *,
    runner_prefix: str,
    idx_label: str,
) -> None:
    """并行执行 ParallelSpec 中的所有子 BlockSpec，全部完成后返回。

    每个子 skill 在独立的守护线程中运行。
    若任意子 skill 抛出异常，第一个异常将在所有线程结束后重新抛出。

    线程安全前提：
    - 导航 skill 使用 Nav2 action client，臂运动使用各自的 arm handler，互相独立。
    - ``ctx.base_world_pos/quat`` 仅在导航 skill 完成后写入，并行期间臂运动不读该字段。
    """
    n = len(par_spec.skills)
    print(f"[{runner_prefix}] {idx_label} parallel({n}) -> dispatching {n} skill(s)")
    errors: list[BaseException] = []
    lock = threading.Lock()

    def worker(sub_spec: BlockSpec, sub_label: str) -> None:
        try:
            _execute_block(ctx, sub_spec, runner_prefix=runner_prefix, idx_label=sub_label)
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


def _resolve_gripper_open_close(mode: str) -> tuple[float, float]:
    if mode == "target_command":
        return 1.0, 0.0
    raise ValueError(
        "single-arm task queue only supports gripper_control_mode='target_command' "
        "without explicit gripper values"
    )


def run_single_arm_task_queue(
    *,
    robot_cfg: Any,
    task_cfg: PickPlaceFlowTaskConfig,
    robot_id: str,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
) -> None:
    """Run ordered skills for one single-arm cycle (connect → FSM → blocks).

    ``task_cfg.mode`` must be ``single_arm``. Typical ``task_queue`` ends with
    Cartesian ``single_arm.return_home`` then optional ``single_arm.movej_return_initial``.

    Initial joint positions are cached on ``ctx`` at connect time for the MoveJ skill.
    """
    mode = task_cfg.mode.lower()
    if mode != "single_arm":
        raise ValueError(f"run_single_arm_task_queue requires mode='single_arm', got {task_cfg.mode!r}")

    specs: list[BlockSpec] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
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
        print("[OK] Robot connected (task queue)")
        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(interface)
        body_initial_joint_positions = capture_initial_body_joint_positions(interface)

        gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
        frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"

        interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        interface.send_fsm_command(FSM_OCS2)

        base_world_pos, base_world_quat = get_entity_pose_world_service(robot_cfg.base_link_entity_path)

        initial_arm = task_cfg.initial_grasp_arm.lower()
        if initial_arm not in {"left", "right"}:
            raise ValueError("initial_grasp_arm must be 'left' or 'right'")
        source_is_right = initial_arm == "right"
        arm_side = ArmSide.RIGHT if source_is_right else ArmSide.LEFT
        source_ee_prefix = "right_ee" if source_is_right else "left_ee"
        source_handler = (
            interface.right_arm_handler if source_is_right else interface.left_arm_handler
        )
        source_home_pose = arm_handler_pose_or_raise(source_handler, label=source_ee_prefix)
        source_object_path = task_cfg.source_object_entity_path
        if not source_object_path:
            raise ValueError("source_object_entity_path is required for single-arm task queue")

        if reset_env:
            reset_simulation_and_randomize_object(
                source_object_path,
                xyz_offset=task_cfg.object_xyz_random_offset,
                post_reset_wait=robot_cfg.post_reset_wait,
                sleep_fn=sim_time.sleep,
            )

        ee_frame_id = (source_handler.frame_id if source_handler else None) or frame_id

        ctx = SingleArmMotionContext(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            task_cfg=task_cfg,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            use_stamped=use_stamped,
            frame_id=frame_id,
            ee_frame_id=ee_frame_id,
            arm_side=arm_side,
            source_is_right=source_is_right,
            source_home_pose=source_home_pose,
            base_world_pos=base_world_pos,
            base_world_quat=base_world_quat,
            gripper_for_return_home=gripper_closed,
            source_ee_prefix=source_ee_prefix,
            left_initial_joint_positions=left_initial_joint_positions,
            right_initial_joint_positions=right_initial_joint_positions,
            body_initial_joint_positions=body_initial_joint_positions,
        )

        for idx, spec in enumerate(specs):
            lbl = f"block {idx + 1}/{len(specs)}"
            if isinstance(spec, ParallelSpec):
                _execute_parallel(ctx, spec, runner_prefix="TaskQ", idx_label=lbl)
            else:
                _execute_block(ctx, spec, runner_prefix="TaskQ", idx_label=lbl)

        interface.send_fsm_command(FSM_HOLD)
        print("[OK] Task queue demo completed")
    except Exception as err:
        print(f"[ERROR] Task queue failed: {err}")
    finally:
        sim_time.shutdown()
        if robot_connected:
            interface.disconnect()
            print("[OK] Robot disconnected")


def run_drawer_pick_place_task_queue(
    *,
    robot_cfg: Any,
    task_cfg: DrawerPickPlaceTaskConfig,
    robot_id: str,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
) -> None:
    """Drawer + apple pick/place as a task queue (``single_arm.drawer.*`` + shared pick/place).

    Environment reset matches :func:`motion_generation.drawer.run_drawer_demo`.
    Add ``single_arm.movej_return_initial`` last in ``task_queue`` for joint-space
    home (cached at connect on ``ctx``). Legacy ``run_drawer_demo`` has no MoveJ.
    """
    if not isinstance(task_cfg, DrawerPickPlaceTaskConfig):
        raise TypeError(f"run_drawer_pick_place_task_queue expects DrawerPickPlaceTaskConfig, got {type(task_cfg)}")
    mode = task_cfg.mode.lower()
    if mode != "single_arm":
        raise ValueError(f"drawer task queue currently requires mode='single_arm', got {task_cfg.mode!r}")

    specs: list[BlockSpec] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
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
        print("[OK] Robot connected (drawer task queue)")
        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(interface)
        body_initial_joint_positions = capture_initial_body_joint_positions(interface)

        gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
        frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"

        interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        interface.send_fsm_command(FSM_OCS2)

        base_world_pos, base_world_quat = get_entity_pose_world_service(robot_cfg.base_link_entity_path)

        initial_arm = task_cfg.initial_grasp_arm.lower()
        if initial_arm not in {"left", "right"}:
            raise ValueError("initial_grasp_arm must be 'left' or 'right'")
        source_is_right = initial_arm == "right"
        arm_side = ArmSide.RIGHT if source_is_right else ArmSide.LEFT
        source_ee_prefix = "right_ee" if source_is_right else "left_ee"
        source_handler = (
            interface.right_arm_handler if source_is_right else interface.left_arm_handler
        )
        source_home_pose = arm_handler_pose_or_raise(source_handler, label=source_ee_prefix)
        apple_path = task_cfg.source_object_entity_path
        drawer_prim = task_cfg.source_object_path_drawer
        drawer_all = task_cfg.source_object_path_drawer_all
        if not apple_path or not drawer_prim or not drawer_all:
            raise ValueError(
                "drawer task queue requires source_object_entity_path, "
                "source_object_path_drawer, source_object_path_drawer_all"
            )

        if reset_env:
            reset_simulation_and_randomize_object(
                apple_path,
                xyz_offset=task_cfg.object_xyz_random_offset,
                post_reset_wait=robot_cfg.post_reset_wait,
                sleep_fn=sim_time.sleep,
            )
            rot_z = random.uniform(-0.1, 0.1)
            set_prim_orientation_local(drawer_all, euler_to_quaternion(0, 0, rot_z))
            randomize_object_xyz_after_reset(
                drawer_all,
                xyz_offset=task_cfg.object_xyz_random_offset_drawer,
            )
            sim_time.sleep(1)

        ee_frame_id = (source_handler.frame_id if source_handler else None) or frame_id

        ctx = SingleArmMotionContext(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            task_cfg=task_cfg,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            use_stamped=use_stamped,
            frame_id=frame_id,
            ee_frame_id=ee_frame_id,
            arm_side=arm_side,
            source_is_right=source_is_right,
            source_home_pose=source_home_pose,
            base_world_pos=base_world_pos,
            base_world_quat=base_world_quat,
            gripper_for_return_home=gripper_closed,
            source_ee_prefix=source_ee_prefix,
            left_initial_joint_positions=left_initial_joint_positions,
            right_initial_joint_positions=right_initial_joint_positions,
            body_initial_joint_positions=body_initial_joint_positions,
        )

        for idx, spec in enumerate(specs):
            lbl = f"block {idx + 1}/{len(specs)}"
            if isinstance(spec, ParallelSpec):
                _execute_parallel(ctx, spec, runner_prefix="TaskQ/drawer", idx_label=lbl)
            else:
                _execute_block(ctx, spec, runner_prefix="TaskQ/drawer", idx_label=lbl)

        interface.send_fsm_command(FSM_HOLD)
        print("[OK] Drawer task queue demo completed")
    except Exception as err:
        print(f"[ERROR] Drawer task queue failed: {err}")
    finally:
        sim_time.shutdown()
        if robot_connected:
            interface.disconnect()
            print("[OK] Robot disconnected")


def run_bimanual_task_queue(
    *,
    robot_cfg: Any,
    task_cfg: BimanualCarryTaskConfig,
    robot_id: str,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
) -> None:
    """Run ordered skills for one bimanual cycle (connect → FSM → blocks)."""
    specs: list[BlockSpec] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
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
        print("[OK] Robot connected (bimanual task queue)")
        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(interface)
        body_initial_joint_positions = capture_initial_body_joint_positions(interface)
        gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
        frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"

        if reset_env:
            reset_simulation_and_randomize_object(
                task_cfg.source_object_entity_path,
                xyz_offset=task_cfg.object_xyz_random_offset,
                post_reset_wait=robot_cfg.post_reset_wait,
                sleep_fn=sim_time.sleep,
            )

        interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        interface.send_fsm_command(FSM_OCS2)

        base_world_pos, base_world_quat = get_entity_pose_world_service(robot_cfg.base_link_entity_path)
        left_handler = interface.left_arm_handler
        right_handler = interface.right_arm_handler
        left_home_pose = arm_handler_pose_or_raise(left_handler, label="left_ee")
        right_home_pose = arm_handler_pose_or_raise(right_handler, label="right_ee")
        ee_frame_id = (left_handler.frame_id if left_handler else None) or frame_id

        ctx = BimanualMotionContext(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            task_cfg=task_cfg,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            use_stamped=use_stamped,
            frame_id=frame_id,
            ee_frame_id=ee_frame_id,
            left_home_pose=left_home_pose,
            right_home_pose=right_home_pose,
            base_world_pos=base_world_pos,
            base_world_quat=base_world_quat,
            left_initial_joint_positions=left_initial_joint_positions,
            right_initial_joint_positions=right_initial_joint_positions,
        )

        for idx, spec in enumerate(specs):
            lbl = f"block {idx + 1}/{len(specs)}"
            if isinstance(spec, ParallelSpec):
                _execute_parallel(ctx, spec, runner_prefix="TaskQ/bimanual", idx_label=lbl)
            else:
                _execute_block(ctx, spec, runner_prefix="TaskQ/bimanual", idx_label=lbl)

        interface.send_fsm_command(FSM_HOLD)
        print("[OK] Bimanual task queue demo completed")
    except Exception as err:
        print(f"[ERROR] Bimanual task queue failed: {err}")
    finally:
        sim_time.shutdown()
        if robot_connected:
            interface.disconnect()
            print("[OK] Robot disconnected")


def run_handover_task_queue(
    *,
    robot_cfg: Any,
    task_cfg: HandoverTaskConfig,
    robot_id: str,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
) -> None:
    """Run ordered skills for one bimanual handover cycle (connect → FSM → blocks)."""
    specs: list[BlockSpec] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
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
        print("[OK] Robot connected (handover task queue)")
        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(interface)
        body_initial_joint_positions = capture_initial_body_joint_positions(interface)
        initial_arm = task_cfg.initial_grasp_arm.lower()
        if initial_arm not in {"left", "right"}:
            raise ValueError("initial_grasp_arm must be 'left' or 'right'")
        source_is_right = initial_arm == "right"
        gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
        frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"

        if reset_env:
            reset_simulation_and_randomize_object(
                task_cfg.source_object_entity_path,
                xyz_offset=task_cfg.object_xyz_random_offset,
                post_reset_wait=robot_cfg.post_reset_wait,
                sleep_fn=sim_time.sleep,
            )

        interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        interface.send_fsm_command(FSM_OCS2)

        base_world_pos, base_world_quat = get_entity_pose_world_service(robot_cfg.base_link_entity_path)
        left_handler = interface.left_arm_handler
        right_handler = interface.right_arm_handler
        left_home_pose = arm_handler_pose_or_raise(left_handler, label="left_ee")
        right_home_pose = arm_handler_pose_or_raise(right_handler, label="right_ee")
        source_home_pose = right_home_pose if source_is_right else left_home_pose
        receiver_home_pose = left_home_pose if source_is_right else right_home_pose
        source_handler = (
            interface.right_arm_handler if source_is_right else interface.left_arm_handler
        )
        ee_frame_id = (source_handler.frame_id if source_handler else None) or frame_id

        ctx = HandoverMotionContext(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            task_cfg=task_cfg,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            use_stamped=use_stamped,
            frame_id=frame_id,
            ee_frame_id=ee_frame_id,
            source_is_right=source_is_right,
            source_home_pose=source_home_pose,
            receiver_home_pose=receiver_home_pose,
            base_world_pos=base_world_pos,
            base_world_quat=base_world_quat,
            left_initial_joint_positions=left_initial_joint_positions,
            right_initial_joint_positions=right_initial_joint_positions,
        )

        for idx, spec in enumerate(specs):
            lbl = f"block {idx + 1}/{len(specs)}"
            if isinstance(spec, ParallelSpec):
                _execute_parallel(ctx, spec, runner_prefix="TaskQ/handover", idx_label=lbl)
            else:
                _execute_block(ctx, spec, runner_prefix="TaskQ/handover", idx_label=lbl)

        interface.send_fsm_command(FSM_HOLD)
        print("[OK] Handover task queue demo completed")
    except Exception as err:
        print(f"[ERROR] Handover task queue failed: {err}")
    finally:
        sim_time.shutdown()
        if robot_connected:
            interface.disconnect()
            print("[OK] Robot disconnected")