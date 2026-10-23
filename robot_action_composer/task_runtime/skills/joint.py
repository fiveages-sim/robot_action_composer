"""关节空间 skill（``joint.*``）：MoveJ 到显式配置、或回到 ``robot.cache_joint_state`` 缓存。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.movej_return import (  # pyright: ignore[reportMissingImports]
    capture_initial_arm_joint_positions,
    capture_initial_body_joint_positions,
    movej_return_to_initial_state,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.registry import register_skill  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.types import ExecutionMeta  # pyright: ignore[reportMissingImports]


def _joint_meta(ctx: QueueRuntimeContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if getattr(ctx, "use_stamped", True) else SendMode.UNSTAMPED,
        frame_id=getattr(ctx, "frame_id", "arm_base"),
        warn_prefix="joint",
    )


def _is_null_joint(v: Any) -> bool:
    return v is None or (isinstance(v, str) and str(v).strip().lower() in ("", "null", "none", "~", "-"))


def _parse_optional_float_list(raw: Any, *, label: str) -> list[float | None] | None:
    """解析关节列表；允许 ``null`` 表示「该关节不改」。空 / 缺省 → ``None``。"""
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"joint.movej_to_config: {label} must be a list, got {type(raw).__name__}")
    if len(raw) == 0:
        return None
    out: list[float | None] = []
    for i, v in enumerate(raw):
        if _is_null_joint(v):
            out.append(None)
        else:
            try:
                out.append(float(v))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"joint.movej_to_config: {label}[{i}] must be float or null") from exc
    return out


def _parse_sparse_joint_map(raw: Any, *, label: str) -> dict[int, float] | None:
    """解析 ``{joint_index: value}``（0-based）；键可为 int 或数字字符串。"""
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ValueError(f"joint.movej_to_config: {label} must be a mapping, got {type(raw).__name__}")
    if not raw:
        return None
    out: dict[int, float] = {}
    for k, v in raw.items():
        try:
            idx = int(k)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"joint.movej_to_config: {label} key {k!r} must be int index") from exc
        if idx < 0:
            raise ValueError(f"joint.movej_to_config: {label} index {idx} must be >= 0")
        out[idx] = float(v)
    return out


def _capture_head_joint_positions(
    interface: Any,
    *,
    wait_timeout: float = 2.0,
    poll_period: float = 0.05,
) -> list[float] | None:
    import time

    deadline = time.monotonic() + max(0.0, wait_timeout)
    while True:
        try:
            categorized = interface.get_joint_state(categorized=True) or {}
            positions = categorized.get("head", {}).get("positions")
            if positions:
                return [float(x) for x in positions]
        except Exception:
            pass
        if time.monotonic() >= deadline:
            return None
        time.sleep(max(0.0, poll_period))


def _merge_absolute_with_current(
    current: Sequence[float] | None,
    absolute: Sequence[float | None] | None,
    *,
    group: str,
) -> list[float] | None:
    """绝对目标：``null`` 槽位保留当前值。全 ``null`` 且无其它有效值 → ``None``（不下发该组）。"""
    if absolute is None:
        return None
    needs_current = any(v is None for v in absolute)
    if needs_current and not current:
        raise RuntimeError(
            f"joint.movej_to_config: {group} has null entries but current joint state unavailable"
        )
    cur = [float(x) for x in (current or [])]
    n = max(len(absolute), len(cur))
    out: list[float] = []
    any_explicit = False
    for i in range(n):
        if i < len(absolute) and absolute[i] is not None:
            out.append(float(absolute[i]))  # type: ignore[arg-type]
            any_explicit = True
        elif i < len(cur):
            out.append(cur[i])
        else:
            raise RuntimeError(
                f"joint.movej_to_config: {group}[{i}] is null/missing and no current value"
            )
    return out if any_explicit else None


def _apply_deltas(
    base: Sequence[float] | None,
    deltas: Sequence[float | None] | None,
    sparse: Mapping[int, float] | None,
    *,
    group: str,
) -> list[float] | None:
    """在 ``base``（通常为当前角）上叠加列表 / 稀疏增量。"""
    if deltas is None and sparse is None:
        return list(base) if base is not None else None
    if not base:
        raise RuntimeError(
            f"joint.movej_to_config: {group} relative motion requires current joint state"
        )
    out = [float(x) for x in base]
    if deltas is not None:
        if len(deltas) > len(out):
            raise ValueError(
                f"joint.movej_to_config: {group}_position_deltas length {len(deltas)} "
                f"> current dof {len(out)}"
            )
        for i, d in enumerate(deltas):
            if d is None:
                continue
            out[i] = out[i] + float(d)
    if sparse:
        for idx, d in sparse.items():
            if idx >= len(out):
                raise ValueError(
                    f"joint.movej_to_config: {group}_joint_deltas index {idx} "
                    f">= current dof {len(out)}"
                )
            out[idx] = out[idx] + float(d)
    return out


def _collect_commanded_joint_indices(
    absolute: Sequence[float | None] | None,
    deltas: Sequence[float | None] | None,
    sparse_deltas: Mapping[int, float] | None,
    resolved_len: int,
) -> frozenset[int]:
    """YAML 里显式写过的关节索引（绝对非 null / 增量非 null / sparse 键）。"""
    commanded: set[int] = set()
    if absolute is not None:
        for i, v in enumerate(absolute):
            if v is not None:
                commanded.add(i)
    if deltas is not None:
        for i, d in enumerate(deltas):
            if d is not None:
                commanded.add(i)
    if sparse_deltas:
        commanded.update(int(k) for k in sparse_deltas)
    if not commanded and resolved_len > 0:
        return frozenset(range(resolved_len))
    return frozenset(commanded)


def _resolve_group_targets(
    *,
    group: str,
    current: Sequence[float] | None,
    absolute: Sequence[float | None] | None,
    deltas: Sequence[float | None] | None,
    sparse_deltas: Mapping[int, float] | None,
) -> tuple[list[float] | None, frozenset[int]]:
    """合并绝对（含 null=保持）与相对增量，得到最终下发目标与需判到位的关节索引。"""
    has_abs = absolute is not None
    has_rel = deltas is not None or sparse_deltas is not None
    if not has_abs and not has_rel:
        return None, frozenset()

    if has_abs:
        base = _merge_absolute_with_current(current, absolute, group=group)
    else:
        if not current:
            raise RuntimeError(
                f"joint.movej_to_config: {group} relative-only requires current joint state"
            )
        base = [float(x) for x in current]

    if has_rel:
        resolved = _apply_deltas(base, deltas, sparse_deltas, group=group)
    else:
        resolved = base

    if resolved is None:
        return None, frozenset()
    check_idx = _collect_commanded_joint_indices(
        absolute, deltas, sparse_deltas, len(resolved)
    )
    return resolved, check_idx


def skill_movej_to_config(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """全身/单臂关节空间运动到指定配置（body / 左臂 / 右臂 均可选）。

    执行顺序：合并目标 → ``send_coordinated_joint_positions``（由 interface 按
    WBC/分体自动 FSM）→ 等待到位。不主动切回 OCS2；后续笛卡尔 skill 会自行切换。
    三组关节均为可选：只填右臂时仅驱动右臂，body 和左臂维持当前位置。

    params:
        body_positions / left_arm_positions / right_arm_positions / head_positions:
            绝对目标；元素可为 ``null`` 表示该关节保持当前值（部分关节设定）。
        body_position_deltas / left_arm_position_deltas / right_arm_position_deltas /
        head_position_deltas:
            相对当前角的增量（rad）；``null`` 表示该关节增量 0。可与绝对目标同时使用（先绝对再加增量）。
        body_joint_deltas / left_arm_joint_deltas / right_arm_joint_deltas / head_joint_deltas:
            稀疏增量 ``{关节索引: delta}``（0-based），便于「只转腰偏航 +π」。
        arrival_timeout / joint_tolerance / skip_fsm_change / body_movej_duration:
            ``skip_fsm_change`` 传给 ``send_coordinated_joint_positions(auto_switch_fsm=False)``，
            跳过 interface 侧自动 FSM；其余同前。
    """
    body_abs = _parse_optional_float_list(params.get("body_positions"), label="body_positions")
    left_abs = _parse_optional_float_list(params.get("left_arm_positions"), label="left_arm_positions")
    right_abs = _parse_optional_float_list(
        params.get("right_arm_positions"), label="right_arm_positions"
    )
    head_abs = _parse_optional_float_list(params.get("head_positions"), label="head_positions")

    body_deltas = _parse_optional_float_list(
        params.get("body_position_deltas"), label="body_position_deltas"
    )
    left_deltas = _parse_optional_float_list(
        params.get("left_arm_position_deltas"), label="left_arm_position_deltas"
    )
    right_deltas = _parse_optional_float_list(
        params.get("right_arm_position_deltas"), label="right_arm_position_deltas"
    )
    head_deltas = _parse_optional_float_list(
        params.get("head_position_deltas"), label="head_position_deltas"
    )

    body_sparse = _parse_sparse_joint_map(params.get("body_joint_deltas"), label="body_joint_deltas")
    left_sparse = _parse_sparse_joint_map(
        params.get("left_arm_joint_deltas"), label="left_arm_joint_deltas"
    )
    right_sparse = _parse_sparse_joint_map(
        params.get("right_arm_joint_deltas"), label="right_arm_joint_deltas"
    )
    head_sparse = _parse_sparse_joint_map(
        params.get("head_joint_deltas"), label="head_joint_deltas"
    )

    arrival_timeout = float(params.get("arrival_timeout", 30.0))
    joint_tolerance = float(params.get("joint_tolerance", 0.05))
    skip_fsm_change = bool(params.get("skip_fsm_change", False))
    body_movej_duration = params.get("body_movej_duration", None)

    interface = ctx.interface
    sim_time = getattr(ctx, "sim_time", None)
    sleep_fn = sim_time.sleep if sim_time else None
    time_now_fn = sim_time.now_seconds if sim_time else None

    needs_current = any(
        [
            body_deltas is not None,
            left_deltas is not None,
            right_deltas is not None,
            head_deltas is not None,
            body_sparse is not None,
            left_sparse is not None,
            right_sparse is not None,
            head_sparse is not None,
            body_abs is not None and any(v is None for v in body_abs),
            left_abs is not None and any(v is None for v in left_abs),
            right_abs is not None and any(v is None for v in right_abs),
            head_abs is not None and any(v is None for v in head_abs),
            # relative-only groups
            body_abs is None and (body_deltas is not None or body_sparse is not None),
            left_abs is None and (left_deltas is not None or left_sparse is not None),
            right_abs is None and (right_deltas is not None or right_sparse is not None),
            head_abs is None and (head_deltas is not None or head_sparse is not None),
        ]
    )

    # 相对/部分绝对：采样当前角后与绝对目标合并为同一组下发/判到位目标；
    # FSM 由 send_coordinated_joint_positions 按 WBC/分体自动处理，skill 不显式 MOVEJ。
    cur_body = cur_left = cur_right = cur_head = None
    if needs_current:
        cur_body = capture_initial_body_joint_positions(interface)
        cur_left, cur_right = capture_initial_arm_joint_positions(interface)
        if head_abs is not None or head_deltas is not None or head_sparse is not None:
            cur_head = _capture_head_joint_positions(interface)

    body_positions, body_check = _resolve_group_targets(
        group="body",
        current=cur_body,
        absolute=body_abs,
        deltas=body_deltas,
        sparse_deltas=body_sparse,
    )
    left_positions, left_check = _resolve_group_targets(
        group="left_arm",
        current=cur_left,
        absolute=left_abs,
        deltas=left_deltas,
        sparse_deltas=left_sparse,
    )
    right_positions, right_check = _resolve_group_targets(
        group="right_arm",
        current=cur_right,
        absolute=right_abs,
        deltas=right_deltas,
        sparse_deltas=right_sparse,
    )
    head_positions, _head_check = _resolve_group_targets(
        group="head",
        current=cur_head,
        absolute=head_abs,
        deltas=head_deltas,
        sparse_deltas=head_sparse,
    )

    if body_movej_duration is not None:
        body_ctrl = str(getattr(interface, "body_controller", "")).strip()
        if not body_ctrl:
            raise ValueError(
                "joint.movej_to_config: cannot infer body controller node; "
                "ensure interface.body_controller is available (body_joint_controller_topic on ROS2RobotInterfaceConfig)"
            )
        duration_val = float(body_movej_duration)
        ok = interface.set_node_parameters(
            full_node_name=body_ctrl,
            parameters={"movej_duration": duration_val},
        )
        if ok:
            print(f"[MoveJ] Set {body_ctrl}.movej_duration = {duration_val}")
        else:
            print(f"[MoveJ] WARN: failed to set {body_ctrl}.movej_duration = {duration_val}")

    moved = False
    if body_positions or left_positions or right_positions or head_positions:
        try:
            interface.send_coordinated_joint_positions(
                body_positions=body_positions or None,
                left_arm_positions=left_positions or None,
                right_arm_positions=right_positions or None,
                head_positions=head_positions or None,
                auto_switch_fsm=not skip_fsm_change,
            )
            moved = True
            print(
                f"[MoveJ] coordinated  body={body_positions is not None} "
                f"L={left_positions is not None} R={right_positions is not None} "
                f"head={head_positions is not None}"
            )
            if body_positions is not None:
                print(f"[MoveJ] body target={body_positions} check_idx={sorted(body_check)}")
        except Exception as exc:
            print(f"[MoveJ] WARN: send_coordinated_joint_positions failed: {exc}")

    if moved and (left_positions or right_positions or body_positions):
        result = interface.wait_until_joint_arrive(
            left_target_positions=left_positions or None,
            right_target_positions=right_positions or None,
            body_target_positions=body_positions or None,
            left_check_indices=sorted(left_check) if left_check else None,
            right_check_indices=sorted(right_check) if right_check else None,
            body_check_indices=sorted(body_check) if body_check else None,
            timeout=arrival_timeout,
            poll_period=0.05,
            joint_tolerance=joint_tolerance,
            angular_wrap=True,
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
            if result.get("body_joint_errors"):
                print(f"[MoveJ] body per-joint err (idx:rad): {result['body_joint_errors']}")

    return [], _joint_meta(ctx)


def _scratch_joint_list(raw: Any) -> list[float] | None:
    if raw is None:
        return None
    if isinstance(raw, (list, tuple)):
        if len(raw) == 0:
            return None
        return [float(x) for x in raw]
    return None


def skill_goto_cached_joints(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """关节空间运动到 ``robot.cache_joint_state`` 写入的缓存（与笛卡尔 ``goto_cache_pose`` 对称）。

    params:
        key (str, 可选): 与缓存时一致，默认 ``saved_joint_state``。
    """
    key = str(params.get("key", "saved_joint_state")).strip() or "saved_joint_state"

    raw = ctx.scratch_get(key)
    if not isinstance(raw, Mapping):
        raise ValueError(
            f"joint.goto_cached_joints: scratch[{key!r}] missing or not a mapping; "
            "run robot.cache_joint_state with the same key earlier in the task_queue"
        )

    left = _scratch_joint_list(raw.get("left"))
    right = _scratch_joint_list(raw.get("right"))
    body = _scratch_joint_list(raw.get("body"))

    if not left and not right and not body:
        raise ValueError(
            f"joint.goto_cached_joints: scratch[{key!r}] has no non-empty left/right/body joint lists"
        )

    rcfg = ctx.robot_cfg
    moved = movej_return_to_initial_state(
        interface=ctx.interface,
        left_initial_positions=left,
        right_initial_positions=right,
        body_initial_positions=body,
        arrival_timeout=float(rcfg.arrival_timeout),
        arrival_poll=float(rcfg.arrival_poll),
        sim_time=ctx.sim_time,
    )
    if not moved:
        print("[WARN] joint.goto_cached_joints: movej_return_to_initial_state did not command motion.")

    return [], _joint_meta(ctx)


def _register_joint_skills() -> None:
    register_skill("joint.movej_to_config", skill_movej_to_config)
    register_skill("joint.goto_cached_joints", skill_goto_cached_joints)


_register_joint_skills()
