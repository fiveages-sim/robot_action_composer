"""将 ``skill_defaults`` / ``skill_params`` 片段叠入扁平 preset（供队列 CLI 与 ``MergedQueueConfig`` 构造）。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import fields
from typing import Any, TypeVar

T = TypeVar("T")


def kwargs_for_dataclass(cls: type[T], raw: Mapping[str, Any]) -> dict[str, Any]:
    names = {f.name for f in fields(cls)}
    return {k: v for k, v in raw.items() if k in names}


def merge_flat_with_skill_pick_place(
    base_flat: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Overlay ``skill_defaults['single_arm.pick']`` then ``['single_arm.place']`` on flat task kwargs.

    当 **pick 与 place 同时提供 ``arm``** 时，不把 place 的 ``arm`` 写入本合并结果（避免盖掉
    ``QueueSliceCommon.arm``，例如交接：抓右臂 / 放左臂）。place 的 ``arm`` 仍由
    :func:`merge_task_queue_skill_params` 在 **``single_arm.place`` 块** 上从
    ``skill_defaults['single_arm.place']`` 注入，不依赖全局扁平表。
    """
    sd = dict(skill_defaults or {})
    pick_sd = dict(sd.get("single_arm.pick") or {})
    place_sd = dict(sd.get("single_arm.place") or {})
    if "arm" in pick_sd and "arm" in place_sd:
        place_sd = {k: v for k, v in place_sd.items() if k != "arm"}
    return {**dict(base_flat), **pick_sd, **place_sd}


def skill_pick_place_defaults_from_scene(scene_preset: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Shape ``scene_preset['skill_params']`` into keys expected by :func:`merge_flat_with_skill_pick_place`."""
    if not scene_preset:
        return None
    sp = scene_preset.get("skill_params")
    if not isinstance(sp, Mapping):
        return None
    pick = sp.get("single_arm.pick")
    place = sp.get("single_arm.place")
    if not pick and not place:
        return None
    return {
        "single_arm.pick": dict(pick or {}),
        "single_arm.place": dict(place or {}),
    }


def merge_flat_with_skill_carry(
    base_flat: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Overlay ``skill_defaults['dual_arm.carry']`` onto flat task kwargs (搬箱 / carry 运行时字段)。"""
    sd = dict(skill_defaults or {})
    carry_sd = dict(sd.get("dual_arm.carry") or {})
    return {**dict(base_flat), **carry_sd}


def merge_flat_with_skill_place(
    base_flat: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Overlay ``skill_defaults['dual_arm.place']``（如 ``place_object_entity_path``）到扁平 preset。"""
    sd = dict(skill_defaults or {})
    place_sd = dict(sd.get("dual_arm.place") or {})
    return {**dict(base_flat), **place_sd}


def skill_carry_defaults_from_scene(scene_preset: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Shape ``scene_preset['skill_params']['dual_arm.carry']`` for :func:`merge_flat_with_skill_carry`."""
    if not scene_preset:
        return None
    sp = scene_preset.get("skill_params")
    if not isinstance(sp, Mapping):
        return None
    carry = sp.get("dual_arm.carry")
    if not carry:
        return None
    return {"dual_arm.carry": dict(carry)}


def skill_place_defaults_from_scene(scene_preset: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Shape ``scene_preset['skill_params']['dual_arm.place']`` for :func:`merge_flat_with_skill_place`."""
    if not scene_preset:
        return None
    sp = scene_preset.get("skill_params")
    if not isinstance(sp, Mapping):
        return None
    place = sp.get("dual_arm.place")
    if not place:
        return None
    return {"dual_arm.place": dict(place)}


def merge_flat_with_skill_handover(
    base_flat: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Overlay ``skill_defaults['dual_arm.handover']`` 到扁平任务字段（构建 ``HandoverSyncConfig`` 子集）。"""
    sd = dict(skill_defaults or {})
    ho_sd = dict(sd.get("dual_arm.handover") or {})
    return {**dict(base_flat), **ho_sd}


def merge_flat_with_skill_drawer(
    base_flat: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Overlay ``skill_defaults['single_arm.drawer']``（抽屉几何，供 ``MergedQueueConfig.drawer``）。"""
    sd = dict(skill_defaults or {})
    drawer_sd = dict(sd.get("single_arm.drawer") or {})
    return {**dict(base_flat), **drawer_sd}


def skill_handover_defaults_from_scene(scene_preset: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Shape ``scene_preset['skill_params']['dual_arm.handover']`` for :func:`merge_flat_with_skill_handover`."""
    if not scene_preset:
        return None
    sp = scene_preset.get("skill_params")
    if not isinstance(sp, Mapping):
        return None
    ho = sp.get("dual_arm.handover")
    if not ho:
        return None
    return {"dual_arm.handover": dict(ho)}


def skill_drawer_defaults_from_scene(scene_preset: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Shape ``scene_preset['skill_params']['single_arm.drawer']`` for :func:`merge_flat_with_skill_drawer`."""
    if not scene_preset:
        return None
    sp = scene_preset.get("skill_params")
    if not isinstance(sp, Mapping):
        return None
    dr = sp.get("single_arm.drawer")
    if not dr:
        return None
    return {"single_arm.drawer": dict(dr)}


def merge_scene_skill_overlays_into_flat(
    scene_flat: Mapping[str, Any],
    scene_preset: Mapping[str, Any] | None,
    *,
    include_place: bool = True,
) -> dict[str, Any]:
    """Apply ``skill_params``：pick/place → handover → carry → (可选) place → drawer（与 motion / record CLI 顺序一致）。

    ``include_place=False`` 时不叠 ``dual_arm.place``，供构建 **仅搬运** 用扁平表（与放置几何分离）。
    """
    out = dict(scene_flat)
    sp_overlay = skill_pick_place_defaults_from_scene(scene_preset)
    if sp_overlay:
        out = merge_flat_with_skill_pick_place(out, sp_overlay)
    handover_overlay = skill_handover_defaults_from_scene(scene_preset)
    if handover_overlay:
        out = merge_flat_with_skill_handover(out, handover_overlay)
    carry_overlay = skill_carry_defaults_from_scene(scene_preset)
    if carry_overlay:
        out = merge_flat_with_skill_carry(out, carry_overlay)
    if include_place:
        place_overlay = skill_place_defaults_from_scene(scene_preset)
        if place_overlay:
            out = merge_flat_with_skill_place(out, place_overlay)
    drawer_overlay = skill_drawer_defaults_from_scene(scene_preset)
    if drawer_overlay:
        out = merge_flat_with_skill_drawer(out, drawer_overlay)
    return out


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
        src = getattr(pick, "source_object_entity_path", "") or ""
        s = str(src).strip()
        if s:
            return s
    top = getattr(task_cfg, "source_object_entity_path", None)
    if isinstance(top, str) and top.strip():
        return top.strip()
    return ""


def reset_settle_entity_path_from_queue(specs: Sequence[Any], task_cfg: Any) -> str:
    """首个 ``single_arm.pick`` 块若覆写 ``source_object_entity_path`` 则用之，否则用 ``task_cfg.pick`` 默认。"""
    from robot_action_composer.task_runtime.types import BlockSpec

    base = _default_pick_source_path(task_cfg)
    for s in iter_leaf_block_specs(specs):
        if isinstance(s, BlockSpec) and s.skill == "single_arm.pick":
            p = dict(s.params)
            raw = p.get("source_object_entity_path")
            if isinstance(raw, str) and raw.strip():
                return raw.strip()
            break
    return base
