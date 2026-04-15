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

from robot_action_composer.ros_interface_utils import build_ros2_interface_from_robot_cfg  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.drawer import euler_to_quaternion  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.bimanual_parallel_pick import (  # pyright: ignore[reportMissingImports]
    BimanualParallelPickTaskConfig,
    parallel_pick_cfg_from_params,
)
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
    task_slice = getattr(ctx, "task_cfg", None)
    common = getattr(task_slice, "common", None) if task_slice is not None else None
    if common is not None:
        exec_kw["pose_tol_pos"] = common.pose_tol_pos
        exec_kw["pose_tol_ori"] = common.pose_tol_ori
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


def effective_base_link_entity_path(*, robot_cfg: Any, runtime: MergedQueueConfig) -> str:
    """任务 :class:`MergedQueueConfig` 中的 ``base_link_entity_path`` 优先于 ``robot_cfg``。"""
    override = runtime.base_link_entity_path
    if isinstance(override, str):
        s = override.strip()
        if s:
            return s
    path = getattr(robot_cfg, "base_link_entity_path", None)
    if isinstance(path, str):
        ps = path.strip()
        if ps:
            return ps
    raise ValueError(
        "Isaac base prim path is required: set robot_cfg.base_link_entity_path "
        "or task base_task_overrides.base_link_entity_path"
    )


def _resolve_gripper_open_close(mode: str) -> tuple[float, float]:
    if mode == "target_command":
        return 1.0, 0.0
    raise ValueError(
        "task queue only supports gripper_control_mode='target_command' "
        "without explicit gripper values"
    )


def _is_parallel_pick_skill(skill: str) -> bool:
    return skill == "dual_arm.parallel_pick" or skill.startswith("dual_arm.parallel_pick_")


def _extract_parallel_pick_cfg_from_specs(specs: Sequence[QueueBlock]) -> BimanualParallelPickTaskConfig | None:
    def _try_from_block(block: BlockSpec) -> BimanualParallelPickTaskConfig | None:
        if not _is_parallel_pick_skill(block.skill):
            return None
        try:
            return parallel_pick_cfg_from_params(block.params)
        except Exception:
            return None

    for s in specs:
        if isinstance(s, ParallelSpec):
            for sub in s.skills:
                cfg = _try_from_block(sub)
                if cfg is not None:
                    return cfg
            continue
        if isinstance(s, BlockSpec):
            cfg = _try_from_block(s)
            if cfg is not None:
                return cfg
    return None


def build_queue_runtime_context(
    *,
    interface: Any,
    robot_cfg: Any,
    sim_time: Any,
    runtime: MergedQueueConfig,
    use_stamped: bool,
) -> QueueRuntimeContext:
    """FSM / connect 之后构造 :class:`QueueRuntimeContext`。"""
    queue_task = runtime.single_arm
    gripper_open, gripper_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
    base_path = effective_base_link_entity_path(robot_cfg=robot_cfg, runtime=runtime)
    frame_id = base_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"
    base_world_pos, base_world_quat = get_entity_pose_world_service(base_path)

    initial_arm = queue_task.common.arm.strip().lower()
    if initial_arm not in {"left", "right"}:
        raise ValueError("arm must be 'left' or 'right'")

    return QueueRuntimeContext(
        interface=interface,
        robot_cfg=robot_cfg,
        sim_time=sim_time,
        task_cfg=queue_task,
        base_link_entity_path=base_path,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        use_stamped=use_stamped,
        frame_id=frame_id,
        base_world_pos=base_world_pos,
        base_world_quat=base_world_quat,
        gripper_for_return_home=gripper_closed,
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
    parallel_pick_cfg = _extract_parallel_pick_cfg_from_specs(specs)
    if parallel_pick_cfg is not None:
        reset_simulation_and_randomize_object(
            parallel_pick_cfg.left_pick.source_object_entity_path,
            xyz_offset=parallel_pick_cfg.left_pick.object_xyz_random_offset,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        reset_simulation_and_randomize_object(
            parallel_pick_cfg.right_pick.source_object_entity_path,
            xyz_offset=parallel_pick_cfg.right_pick.object_xyz_random_offset,
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
