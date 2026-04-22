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


def _default_pick_source_path(task_cfg: Any) -> str:
    pick = getattr(task_cfg, "pick", None)
    if pick is not None:
        src = getattr(pick, "object_prim_path", "") or ""
        s = str(src).strip()
        if s:
            return s
    top = getattr(task_cfg, "object_prim_path", None)
    if isinstance(top, str) and top.strip():
        return top.strip()
    return ""


def reset_settle_entity_path_from_queue(specs: Sequence[Any], task_cfg: Any) -> str:
    """首个 ``single_arm.pick`` 块若覆写 ``object_prim_path`` 则用之，否则用 ``task_cfg.pick`` 默认。"""
    from robot_action_composer.task_runtime.types import BlockSpec

    base = _default_pick_source_path(task_cfg)
    for s in iter_leaf_block_specs(specs):
        if isinstance(s, BlockSpec) and s.skill == "single_arm.pick":
            p = dict(s.params)
            raw = p.get("object_prim_path")
            if isinstance(raw, str) and raw.strip():
                return raw.strip()
            break
    return base

