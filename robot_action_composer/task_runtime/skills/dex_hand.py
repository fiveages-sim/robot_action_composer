"""Dex-hand queue skills: privileged HUG auto-grasp via ``run_auto_grasp``.

Registers ``dex_hand.auto_grasp``. Reuses motion ``ctx.interface`` and per-robot
``grasp_generation.yaml`` (same library as Viser / ``grasp-generation auto``).
Does **not** clear or move table props.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    SendMode,
)
from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.types import ExecutionMeta


def _default_meta(ctx: QueueRuntimeContext) -> ExecutionMeta:
    return ExecutionMeta(
        send_mode=SendMode.STAMPED if getattr(ctx, "use_stamped", True) else SendMode.UNSTAMPED,
        frame_id=getattr(ctx, "frame_id", "arm_base"),
        warn_prefix="dex_hand",
    )


def _resolve_workspace(params: Mapping[str, Any]) -> Path:
    raw = params.get("workspace")
    if raw:
        return Path(str(raw)).expanduser().resolve()
    cwd = Path.cwd().resolve()
    if (cwd / "robots").is_dir():
        return cwd
    for parent in cwd.parents:
        robots = parent / "robots"
        if robots.is_dir() and any(robots.glob("**/grasp_generation.yaml")):
            return parent
    return cwd


def _resolve_robot_key(params: Mapping[str, Any], workspace: Path) -> str:
    key = str(params.get("robot_key") or "").strip()
    if key:
        return key
    from robot_action_composer.grasp_generation.config import discover_grasp_robots

    robots = discover_grasp_robots(workspace)
    if len(robots) == 1:
        return next(iter(robots.keys()))
    if "fiveages_w2" in robots:
        return "fiveages_w2"
    known = ", ".join(sorted(robots)) or "(none)"
    raise ValueError(
        "dex_hand.auto_grasp requires params.robot_key "
        f"(or a single grasp_generation.yaml under workspace/robots/). known={known}"
    )


def _apply_object_params(
    cfg: Any,
    params: Mapping[str, Any],
) -> tuple[Any, str]:
    """Return (cfg, object_key); inline prims patch or create a catalog entry."""
    object_key = str(params.get("object_key") or "").strip()
    pose_prim = str(params.get("pose_prim") or "").strip()
    placement_in = str(params.get("placement_prim") or "").strip()
    click_prim = str(params.get("click_prim") or "").strip()
    click_offset = params.get("click_offset", params.get("click_offset_m"))

    has_inline = bool(pose_prim or placement_in or click_prim or click_offset is not None)
    if object_key and not has_inline:
        return cfg, object_key
    if not object_key and not pose_prim:
        raise ValueError(
            "dex_hand.auto_grasp needs params.object_key "
            "(from grasp_objects) or params.pose_prim"
        )

    key = object_key or "_task_inline"
    base: dict[str, Any] = {}
    prev = (cfg.grasp_objects or {}).get(key)
    if isinstance(prev, Mapping):
        base = dict(prev)
    if pose_prim:
        base["pose_prim"] = pose_prim
    if placement_in:
        base["placement_prim"] = placement_in
    elif pose_prim and "placement_prim" not in base:
        base["placement_prim"] = pose_prim
    if click_prim:
        base["click_prim"] = click_prim
    if click_offset is not None:
        base["click_offset"] = list(click_offset)
    if not str(base.get("pose_prim") or "").strip():
        raise ValueError("dex_hand.auto_grasp: pose_prim missing after merge")
    base.setdefault("placement_prim", base["pose_prim"])

    objs = dict(cfg.grasp_objects or {})
    objs[key] = base
    return replace(cfg, grasp_objects=objs), key


def skill_auto_grasp(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[Any], ExecutionMeta]:
    """Privileged HUG grasp: capture → UV → propose → wrist → retarget → close.

    params:
        robot_key (str): ``robots/<key>/grasp_generation.yaml``（可省略若 workspace 仅一台）。
        object_key (str): ``grasp_objects`` 键；或与 ``pose_prim`` 二选一 / 组合覆盖。
        pose_prim / placement_prim / click_prim / click_offset: 任务级物体覆盖。
        arm / close_executor / squeeze_amount: 可选覆盖 yaml。
        workspace (str): 含 ``robots/`` 的根（默认 ``cwd``，通常 ``examples/IsaacSim``）。
        dry_run (bool): 只 propose + dry plan。
        do_close (bool): 默认 True；False 则只跟腕。
    """
    from robot_action_composer.grasp_generation.auto_grasp import run_auto_grasp
    from robot_action_composer.grasp_generation.config import load_grasp_config

    workspace = _resolve_workspace(params)
    robot_key = _resolve_robot_key(params, workspace)

    overrides: dict[str, Any] = {}
    if params.get("arm") is not None:
        overrides["arm"] = str(params["arm"]).lower()
    if params.get("close_executor") is not None:
        overrides["close_executor"] = str(params["close_executor"]).upper()
    if params.get("squeeze_amount") is not None:
        overrides["squeeze_amount"] = float(params["squeeze_amount"])

    cfg = load_grasp_config(
        workspace=workspace,
        robot_key=robot_key,
        overrides=overrides or None,
    )
    cfg, object_key = _apply_object_params(cfg, params)

    dry_run = bool(params.get("dry_run", False))
    do_close = bool(params.get("do_close", True))
    execute = not dry_run

    print(
        f"[dex_hand] auto_grasp robot={robot_key!r} object={object_key!r} "
        f"workspace={workspace} dry_run={dry_run} do_close={do_close}"
    )

    result = run_auto_grasp(
        cfg=cfg,
        workspace=workspace,
        object_key=object_key,
        interface=ctx.interface,
        execute=execute,
        dry_run=dry_run,
        do_close=do_close and execute,
        status=lambda msg: print(f"[dex_hand] {msg}"),
    )
    if not result.ok:
        raise RuntimeError(f"dex_hand.auto_grasp failed: {result.message}")

    print(
        f"[dex_hand] OK UV=({result.u_norm:.3f},{result.v_norm:.3f}) "
        f"pose={result.pose_method} — {result.message}"
    )
    ctx.scratch_put(
        "dex_hand.last_auto_grasp",
        {
            "object_key": result.object_key,
            "u_norm": result.u_norm,
            "v_norm": result.v_norm,
            "proposal": str(result.proposal_path) if result.proposal_path else None,
            "plan": str(result.plan_path) if result.plan_path else None,
            "message": result.message,
        },
    )
    return [], _default_meta(ctx)


def _register() -> None:
    register_skill("dex_hand.auto_grasp", skill_auto_grasp)


_register()
