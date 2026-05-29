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
    reset_simulation_state,
)

from robot_action_composer.ros_interface_utils import build_ros2_interface_from_robot_cfg  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.bimanual_parallel_pick import (  # pyright: ignore[reportMissingImports]
    BimanualParallelPickTaskConfig,
    parallel_pick_cfg_from_params,
)
from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.merge.utils import reset_settle_entity_path_from_queue
from robot_action_composer.task_runtime.registry import get_skill
from robot_action_composer.task_runtime.config.merged import MergedQueueConfig
from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec, QueueBlock, block_spec_from_mapping


def _skip_repeat_reason(ctx: Any) -> str | None:
    """Return a short reason string if flagged blocks should be skipped in this run."""
    if bool(getattr(ctx, "not_first_scene_in_batch", False)):
        return "not first scene in __all__ batch"
    if bool(getattr(ctx, "consecutive_same_scene", False)):
        return "consecutive same scene in chain"
    return None


def _should_skip_repeat_block(ctx: Any, spec: BlockSpec) -> bool:
    if not spec.skip_when_not_first_scene:
        return False
    return _skip_repeat_reason(ctx) is not None


def _block_label(spec: BlockSpec) -> str:
    return f"{spec.skill!r}" if spec.id is None else f"{spec.skill!r} (id={spec.id!r})"


def _execute_block(
    ctx: Any,
    spec: BlockSpec,
    *,
    runner_prefix: str,
    idx_label: str,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
) -> None:
    label = _block_label(spec)
    skip_reason = _skip_repeat_reason(ctx)
    if skip_reason is not None and spec.skip_when_not_first_scene:
        print(
            f"[{runner_prefix}] {idx_label} {label} -> skipped "
            f"(skip_when_not_first_scene, {skip_reason})",
        )
        return
    if spec.start_delay_s > 0.0:
        print(
            f"[{runner_prefix}] {idx_label} {spec.skill!r} delaying {spec.start_delay_s:.3f}s before dispatch",
        )
        ctx.sim_time.sleep(spec.start_delay_s)
    skill_fn = get_skill(spec.skill)
    stages, meta = skill_fn(ctx, spec.params)
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
    active: list[tuple[int, BlockSpec]] = [
        (i, sub) for i, sub in enumerate(par_spec.skills) if not _should_skip_repeat_block(ctx, sub)
    ]
    skipped = len(par_spec.skills) - len(active)
    if skipped:
        for i, sub in enumerate(par_spec.skills):
            if _should_skip_repeat_block(ctx, sub):
                reason = _skip_repeat_reason(ctx) or "repeat batch"
                print(
                    f"[{runner_prefix}] {idx_label}[{i}] {_block_label(sub)} -> skipped "
                    f"(skip_when_not_first_scene, {reason})",
                )
    if not active:
        print(f"[{runner_prefix}] {idx_label} parallel({len(par_spec.skills)}) -> all sub-steps skipped")
        return
    n = len(active)
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
            args=(sub_spec, f"{idx_label}[{orig_i}]"),
            daemon=True,
        )
        for orig_i, sub_spec in active
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
        "or task runtime_defaults.base_link_entity_path"
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
    not_first_scene_in_batch: bool = False,
    consecutive_same_scene: bool = False,
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
        place_task_cfg=runtime.place,
        handover_sync=runtime.handover,
        drawer_geometry=runtime.drawer,
        not_first_scene_in_batch=not_first_scene_in_batch,
        consecutive_same_scene=consecutive_same_scene,
    )


def reset_queue_task_environment(
    *,
    specs: Sequence[QueueBlock],
    runtime: MergedQueueConfig,
    robot_cfg: Any,
    sim_time: SimTimeHelper,
) -> None:
    """Isaac：仅 reset/play + settle，**不**在 reset 内随机物体平移。

    需要扰动 prim 时，把 ``env.randomize_object_local_xyz`` 放在 ``task_queue`` **第一个**顺序块
    （若首块为 ``parallel``，则作为其子步骤之一，在并行开始前单独一步更稳妥）。
    """
    if runtime.drawer is not None:
        dcfg = runtime.drawer
        reset_source_path = reset_settle_entity_path_from_queue(specs, runtime.single_arm)
        apple_path = reset_source_path
        drawer_prim = dcfg.object_prim_path
        if not apple_path or not drawer_prim:
            raise ValueError(
                "drawer task queue requires object_prim_path (task_cfg or single_arm.pick params), "
                "and single_arm.drawer.object_prim_path (drawer layer prim)"
            )
        reset_simulation_state(
            apple_path,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        sim_time.sleep(1)
        return
    if runtime.carry is not None:
        reset_simulation_state(
            runtime.carry.object_prim_path,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        return
    if runtime.handover is not None:
        pk = runtime.single_arm.pick
        if not pk.object_prim_path:
            raise ValueError(
                "handover task env reset requires object_prim_path on single_arm.pick "
                "(skill_defaults.single_arm.pick or skill_params for pick block)"
            )
        reset_simulation_state(
            pk.object_prim_path,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        return
    parallel_pick_cfg = _extract_parallel_pick_cfg_from_specs(specs)
    if parallel_pick_cfg is not None:
        reset_simulation_state(
            parallel_pick_cfg.left_pick.object_prim_path,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        reset_simulation_state(
            parallel_pick_cfg.right_pick.object_prim_path,
            post_reset_wait=robot_cfg.post_reset_wait,
            sleep_fn=sim_time.sleep,
        )
        return
    reset_source_path = reset_settle_entity_path_from_queue(specs, runtime.single_arm)
    if not reset_source_path:
        raise ValueError(
            "object_prim_path is required for env reset "
            "(set on task_cfg or under skill_defaults / skill_params for single_arm.pick)"
        )
    reset_simulation_state(
        reset_source_path,
        post_reset_wait=robot_cfg.post_reset_wait,
        sleep_fn=sim_time.sleep,
    )


def run_task_queue(
    *,
    robot_cfg: Any,
    runtime: MergedQueueConfig,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
    not_first_scene_in_batch: bool = False,
    consecutive_same_scene: bool = False,
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
            not_first_scene_in_batch=not_first_scene_in_batch,
            consecutive_same_scene=consecutive_same_scene,
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



def run_task_queue_on_connected_interface(
    *,
    interface: Any,
    sim_time: SimTimeHelper,
    robot_cfg: Any,
    runtime: MergedQueueConfig,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = True,
    use_stamped: bool = True,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
    not_first_scene_in_batch: bool = False,
    consecutive_same_scene: bool = False,
) -> None:
    """Execute one task_queue on an already-connected interface (no connect/disconnect)."""
    specs: list[QueueBlock] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
    if not specs:
        raise ValueError("task queue blocks list is empty")

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
        not_first_scene_in_batch=not_first_scene_in_batch,
        consecutive_same_scene=consecutive_same_scene,
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