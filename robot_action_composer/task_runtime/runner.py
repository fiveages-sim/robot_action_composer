"""Unified task queue runner: connect → FSM → ``task_queue`` (blocks + parallel)."""

from __future__ import annotations

import random
import signal
import sys
import termios
import threading
from collections.abc import Mapping
from typing import Any, Sequence, TextIO

from ros2_robot_interface import FSM_HOLD  # pyright: ignore[reportMissingImports]

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
from robot_action_composer.task_runtime.context import (
    CurrentBlockMeta,
    QueueRuntimeContext,
    set_thread_block_meta,
)
from robot_action_composer.task_runtime.merge.utils import reset_settle_entity_path_from_queue
from robot_action_composer.task_runtime.registry import get_skill
from robot_action_composer.task_runtime.config.merged import MergedQueueConfig
from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec, QueueBlock, block_spec_from_mapping


def _skip_repeat_reason(ctx: Any) -> str | None:
    """Return a short reason string if flagged blocks should be skipped in this run."""
    if bool(getattr(ctx, "not_first_scene_in_batch", False)):
        return "not first scene in __all__ batch"
    if bool(getattr(ctx, "consecutive_same_task_in_chain", False)):
        return "consecutive same task in chain"
    return None


def _should_skip_repeat_block(ctx: Any, spec: BlockSpec) -> bool:
    if not spec.skip_when_not_first_scene:
        return False
    return _skip_repeat_reason(ctx) is not None


def _block_label(spec: BlockSpec) -> str:
    return f"{spec.skill!r}" if spec.id is None else f"{spec.skill!r} (id={spec.id!r})"


def _block_param_key(spec: BlockSpec) -> str:
    return spec.id if spec.id else spec.skill


def _is_navigation_skill(spec: BlockSpec) -> bool:
    return spec.skill.startswith("nav.")


def _set_current_block(
    ctx: Any,
    spec: BlockSpec,
    *,
    task_key: str,
    scene: str,
    block_index: int,
    parallel_index: int | None,
) -> None:
    meta = CurrentBlockMeta(
        task_key=task_key,
        scene=scene,
        skill=spec.skill,
        block_key=_block_param_key(spec),
        block_index=block_index,
        parallel_index=parallel_index,
    )
    ctx.current_block = meta
    set_thread_block_meta(meta)


def _execute_block(
    ctx: Any,
    spec: BlockSpec,
    *,
    runner_prefix: str,
    idx_label: str,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
    task_key: str = "",
    scene: str = "",
    block_index: int = 0,
    parallel_index: int | None = None,
) -> None:
    if task_key:
        _set_current_block(
            ctx,
            spec,
            task_key=task_key,
            scene=scene,
            block_index=block_index,
            parallel_index=parallel_index,
        )
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
        # runtime_defaults.max_stage_duration 覆盖机器人默认 arrival_timeout
        # （否则 arm_movel_duration=4 时仍会在默认 3s 就判超时）
        msd = getattr(common, "max_stage_duration", None)
        if msd is not None:
            try:
                exec_kw["arrival_timeout"] = float(msd)
            except (TypeError, ValueError):
                pass
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
    task_key: str = "",
    scene: str = "",
    block_index: int = 0,
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
    _execute_parallel_active_subset(
        ctx,
        par_spec,
        active,
        runner_prefix=runner_prefix,
        idx_label=idx_label,
        execute_stage_kwargs=execute_stage_kwargs,
        task_key=task_key,
        scene=scene,
        block_index=block_index,
    )


def _execute_parallel_active_subset(
    ctx: Any,
    par_spec: ParallelSpec,
    active_specs: Sequence[tuple[int, BlockSpec]],
    *,
    runner_prefix: str,
    idx_label: str,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
    task_key: str = "",
    scene: str = "",
    block_index: int = 0,
) -> None:
    if not active_specs:
        print(f"[{runner_prefix}] {idx_label} parallel({len(par_spec.skills)}) -> all sub-steps skipped")
        return

    n = len(active_specs)
    print(f"[{runner_prefix}] {idx_label} parallel({n}) -> dispatching {n} skill(s)")
    errors: list[BaseException] = []
    lock = threading.Lock()

    def worker(sub_spec: BlockSpec, sub_label: str, parallel_index: int) -> None:
        try:
            _execute_block(
                ctx,
                sub_spec,
                runner_prefix=runner_prefix,
                idx_label=sub_label,
                execute_stage_kwargs=execute_stage_kwargs,
                task_key=task_key,
                scene=scene,
                block_index=block_index,
                parallel_index=parallel_index,
            )
        except Exception as exc:  # noqa: BLE001
            with lock:
                errors.append(exc)
        finally:
            set_thread_block_meta(None)

    threads = [
        threading.Thread(
            target=worker,
            args=(sub_spec, f"{idx_label}[{orig_i}]", orig_i),
            daemon=True,
        )
        for orig_i, sub_spec in active_specs
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
    consecutive_same_task_in_chain: bool = False,
    use_isaac_base_pose: bool = True,
    task_key: str = "",
    scene: str = "",
    object_resolution: Any | None = None,
) -> QueueRuntimeContext:
    """FSM / connect 之后构造 :class:`QueueRuntimeContext`。"""
    queue_task = runtime.single_arm
    base_open, base_closed = _resolve_gripper_open_close(robot_cfg.gripper_control_mode)
    common = queue_task.common
    gripper_open = (
        float(common.gripper_open) if common.gripper_open is not None else base_open
    )
    gripper_closed = (
        float(common.gripper_closed) if common.gripper_closed is not None else base_closed
    )
    base_path = effective_base_link_entity_path(robot_cfg=robot_cfg, runtime=runtime)
    frame_id = base_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"
    if use_isaac_base_pose:
        base_world_pos, base_world_quat = get_entity_pose_world_service(base_path)
    else:
        base_world_pos = (0.0, 0.0, 0.0)
        base_world_quat = (0.0, 0.0, 0.0, 1.0)

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
        consecutive_same_task_in_chain=consecutive_same_task_in_chain,
        task_key=task_key,
        scene=scene,
        object_resolution=object_resolution,
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
    consecutive_same_task_in_chain: bool = False,
    task_key: str = "",
    scene: str = "",
    object_resolution: Any | None = None,
    use_isaac_base_pose: bool = True,
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

        if reset_env:
            reset_queue_task_environment(specs=specs, runtime=runtime, robot_cfg=robot_cfg, sim_time=sim_time)

        ctx = build_queue_runtime_context(
            interface=interface,
            robot_cfg=robot_cfg,
            sim_time=sim_time,
            runtime=runtime,
            use_stamped=use_stamped,
            not_first_scene_in_batch=not_first_scene_in_batch,
            consecutive_same_task_in_chain=consecutive_same_task_in_chain,
            use_isaac_base_pose=use_isaac_base_pose,
            task_key=task_key,
            scene=scene,
            object_resolution=object_resolution,
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
                    task_key=task_key,
                    scene=scene,
                    block_index=idx,
                )
            else:
                _execute_block(
                    ctx,
                    spec,
                    runner_prefix="TaskQ",
                    idx_label=lbl,
                    execute_stage_kwargs=execute_stage_kwargs,
                    task_key=task_key,
                    scene=scene,
                    block_index=idx,
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
    consecutive_same_task_in_chain: bool = False,
    use_isaac_base_pose: bool = True,
    task_key: str = "",
    scene: str = "",
    object_resolution: Any | None = None,
) -> None:
    """Execute one task_queue on an already-connected interface (no connect/disconnect)."""
    specs: list[QueueBlock] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
    if not specs:
        raise ValueError("task queue blocks list is empty")

    if reset_env:
        reset_queue_task_environment(specs=specs, runtime=runtime, robot_cfg=robot_cfg, sim_time=sim_time)

    ctx = build_queue_runtime_context(
        interface=interface,
        robot_cfg=robot_cfg,
        sim_time=sim_time,
        runtime=runtime,
        use_stamped=use_stamped,
        not_first_scene_in_batch=not_first_scene_in_batch,
        consecutive_same_task_in_chain=consecutive_same_task_in_chain,
        use_isaac_base_pose=use_isaac_base_pose,
        task_key=task_key,
        scene=scene,
        object_resolution=object_resolution,
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
                task_key=task_key,
                scene=scene,
                block_index=idx,
            )
        else:
            _execute_block(
                ctx,
                spec,
                runner_prefix="TaskQ",
                idx_label=lbl,
                execute_stage_kwargs=execute_stage_kwargs,
                task_key=task_key,
                scene=scene,
                block_index=idx,
            )

    interface.send_fsm_command(FSM_HOLD)
    print("[OK] Task queue completed")


def _format_interactive_block_prompt(spec: QueueBlock, lbl: str) -> str:
    if isinstance(spec, ParallelSpec):
        skill_part = f"parallel({len(spec.skills)})"
        id_part = ""
    else:
        skill_part = spec.skill
        id_part = f", id={spec.id!r}" if spec.id else ""
    return f"[{lbl}] Next block: skill={skill_part}{id_part}"


def _drain_stdin_buffer(stream: TextIO | None = None) -> None:
    try:
        target = stream if stream is not None else sys.stdin
        if target.isatty():
            termios.tcflush(target.fileno(), termios.TCIFLUSH)
    except (AttributeError, OSError, termios.error):
        return


def _prompt_nav_block_action(spec: BlockSpec, lbl: str) -> str:
    """Return 'run', 'skip', or 'quit' for an interactive navigation block."""
    label = _block_label(spec)
    while True:
        _drain_stdin_buffer()
        print(f"\n[{lbl}] Navigation block: {label}")
        print("  Press Enter/r to run, s to skip this navigation skill, or q to stop.")
        raw = input("> ").strip().lower()
        if raw in {"", "r", "run", "y", "yes"}:
            return "run"
        if raw in {"s", "skip"}:
            return "skip"
        if raw in {"q", "quit"}:
            return "quit"
        print("  Please enter Enter/r, s, or q.")


def _filter_interactive_parallel_navigation(
    ctx: Any,
    par_spec: ParallelSpec,
    lbl: str,
) -> tuple[str, list[tuple[int, BlockSpec]]]:
    """Return ('run'|'quit', active sub-steps) after prompting for nav sub-steps."""
    active: list[tuple[int, BlockSpec]] = []
    for sub_idx, sub in enumerate(par_spec.skills):
        sub_lbl = f"{lbl}[{sub_idx}]"
        if _should_skip_repeat_block(ctx, sub):
            reason = _skip_repeat_reason(ctx) or "repeat batch"
            print(
                f"[TaskQ] {sub_lbl} {_block_label(sub)} -> skipped "
                f"(skip_when_not_first_scene, {reason})",
            )
            continue
        if _is_navigation_skill(sub):
            action = _prompt_nav_block_action(sub, sub_lbl)
            if action == "quit":
                return "quit", active
            if action == "skip":
                print(f"[TaskQ] {sub_lbl} {_block_label(sub)} -> skipped (user skipped navigation)")
                continue
        active.append((sub_idx, sub))
    return "run", active


def _wait_interactive_block_confirm(spec: QueueBlock, lbl: str) -> bool:
    _drain_stdin_buffer()
    print(f"\n{_format_interactive_block_prompt(spec, lbl)}")
    print("  Press Enter to run this block, or 'q' to stop (FSM_HOLD + exit).")
    raw = input("> ").strip().lower()
    return raw != "q"


def run_task_queue_interactive_on_connected_interface(
    *,
    interface: Any,
    sim_time: SimTimeHelper,
    robot_cfg: Any,
    runtime: MergedQueueConfig,
    blocks: Sequence[BlockSpec | dict[str, Any]],
    reset_env: bool = False,
    use_stamped: bool = True,
    use_isaac_base_pose: bool = True,
    execute_stage_kwargs: Mapping[str, Any] | None = None,
    not_first_scene_in_batch: bool = False,
    consecutive_same_task_in_chain: bool = False,
    task_key: str = "",
    scene: str = "",
    object_resolution: Any | None = None,
) -> None:
    """逐 block 等待 Enter 后执行；输入 ``q`` 则 ``FSM_HOLD`` 并提前退出。"""
    specs: list[QueueBlock] = [b if isinstance(b, BlockSpec) else block_spec_from_mapping(b) for b in blocks]
    if not specs:
        raise ValueError("task queue blocks list is empty")

    if reset_env:
        reset_queue_task_environment(specs=specs, runtime=runtime, robot_cfg=robot_cfg, sim_time=sim_time)

    ctx = build_queue_runtime_context(
        interface=interface,
        robot_cfg=robot_cfg,
        sim_time=sim_time,
        runtime=runtime,
        use_stamped=use_stamped,
        not_first_scene_in_batch=not_first_scene_in_batch,
        consecutive_same_task_in_chain=consecutive_same_task_in_chain,
        use_isaac_base_pose=use_isaac_base_pose,
        task_key=task_key,
        scene=scene,
        object_resolution=object_resolution,
    )

    total = len(specs)
    for idx, spec in enumerate(specs):
        lbl = f"block {idx + 1}/{total}"

        if isinstance(spec, ParallelSpec):
            if not _wait_interactive_block_confirm(spec, lbl):
                print(f"[info] Stopped before {lbl} (user quit)")
                interface.send_fsm_command(FSM_HOLD)
                return
            action, active = _filter_interactive_parallel_navigation(ctx, spec, lbl)
            if action == "quit":
                print(f"[info] Stopped before {lbl} (user quit)")
                interface.send_fsm_command(FSM_HOLD)
                return
            _execute_parallel_active_subset(
                ctx,
                spec,
                active,
                runner_prefix="TaskQ",
                idx_label=lbl,
                execute_stage_kwargs=execute_stage_kwargs,
                task_key=task_key,
                scene=scene,
                block_index=idx,
            )
            continue

        if _should_skip_repeat_block(ctx, spec):
            _execute_block(
                ctx,
                spec,
                runner_prefix="TaskQ",
                idx_label=lbl,
                execute_stage_kwargs=execute_stage_kwargs,
                task_key=task_key,
                scene=scene,
                block_index=idx,
            )
            continue

        if _is_navigation_skill(spec):
            action = _prompt_nav_block_action(spec, lbl)
            if action == "quit":
                print(f"[info] Stopped before {lbl} (user quit)")
                interface.send_fsm_command(FSM_HOLD)
                return
            if action == "skip":
                print(f"[TaskQ] {lbl} {_block_label(spec)} -> skipped (user skipped navigation)")
                continue
        elif not _wait_interactive_block_confirm(spec, lbl):
            print(f"[info] Stopped before {lbl} (user quit)")
            interface.send_fsm_command(FSM_HOLD)
            return

        _execute_block(
            ctx,
            spec,
            runner_prefix="TaskQ",
            idx_label=lbl,
            execute_stage_kwargs=execute_stage_kwargs,
            task_key=task_key,
            scene=scene,
            block_index=idx,
        )

    interface.send_fsm_command(FSM_HOLD)
    print("[OK] Interactive task queue completed")