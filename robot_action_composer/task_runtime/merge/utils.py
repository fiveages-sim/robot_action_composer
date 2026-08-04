"""Queue merge helpers."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import fields
from typing import Any, TypeVar

T = TypeVar("T")


def kwargs_for_dataclass(cls: type[T], raw: Mapping[str, Any]) -> dict[str, Any]:
    names = {f.name for f in fields(cls)}
    return {k: v for k, v in raw.items() if k in names}


def iter_leaf_block_specs(specs: Sequence[Any]) -> Any:
    from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec

    for s in specs:
        if isinstance(s, ParallelSpec):
            for sub in s.skills:
                yield sub
        elif isinstance(s, BlockSpec):
            yield s


def _default_pick_source_path(
    task_cfg: Any,
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    active_object: str | None = None,
) -> str:
    from robot_action_composer.task_runtime.object_binding import resolve_object_prim_path

    pick = getattr(task_cfg, "pick", None)
    if pick is not None:
        params = {
            "object_prim_path": getattr(pick, "object_prim_path", "") or "",
            "object_key": getattr(pick, "object_key", None),
        }
        resolved = resolve_object_prim_path(
            params,
            objects=objects,
            active_object=active_object,
            use_active_object=True,
            required=False,
            label="reset pick source",
        )
        if resolved:
            return resolved
    top = getattr(task_cfg, "object_prim_path", None)
    if isinstance(top, str) and top.strip():
        return top.strip()
    return ""


def reset_settle_entity_path_from_queue(
    specs: Sequence[Any],
    task_cfg: Any,
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    active_object: str | None = None,
) -> str:
    """首个 ``single_arm.pick`` 块若覆写 prim/object_key 则用之，否则用 ``task_cfg.pick`` 默认。"""
    from robot_action_composer.task_runtime.object_binding import resolve_object_prim_path
    from robot_action_composer.task_runtime.types import BlockSpec

    base = _default_pick_source_path(
        task_cfg, objects=objects, active_object=active_object
    )
    for s in iter_leaf_block_specs(specs):
        if isinstance(s, BlockSpec) and s.skill == "single_arm.pick":
            p = dict(s.params)
            resolved = resolve_object_prim_path(
                p,
                objects=objects,
                active_object=active_object,
                use_active_object=False,
                required=False,
                label="reset pick block",
            )
            if resolved:
                return resolved
            break
    return base

