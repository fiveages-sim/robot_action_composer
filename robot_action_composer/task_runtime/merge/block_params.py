"""将 ``skill_defaults`` 与场景 ``skill_params`` 合并进 ``task_queue`` 各块的 ``params``。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace as _dc_replace
from typing import Any

from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec

_DRAWER_SKILL_PREFIX = "single_arm.drawer."


def _inherits_dual_arm_carry_block(param_key: str) -> bool:
    """``dual_arm.carry*`` 仅铺 ``dual_arm.carry``（与 ``dual_arm.place*`` 独立）。"""
    return param_key == "dual_arm.carry" or param_key.startswith("dual_arm.carry_")


def _inherits_dual_arm_place_block(param_key: str) -> bool:
    """``dual_arm.place*`` 仅铺 ``dual_arm.place``（:class:`~.bimanual_place.BimanualPlaceTaskConfig`）。"""
    return param_key == "dual_arm.place" or (
        param_key.startswith("dual_arm.place_") and not param_key.startswith("dual_arm.place_relative")
    )


def _inherits_dual_arm_place_relative_block(param_key: str) -> bool:
    """``dual_arm.place_relative``：先铺 ``dual_arm.carry`` 再铺 ``dual_arm.place_relative``（相对末端放置参数）。"""
    return param_key == "dual_arm.place_relative"


def _inherits_dual_arm_parallel_pick_block(param_key: str) -> bool:
    """``dual_arm.parallel_pick`` / ``parallel_pick_*`` 共用 ``dual_arm.parallel_pick`` 默认。"""
    return param_key == "dual_arm.parallel_pick" or param_key.startswith("dual_arm.parallel_pick_")


def _is_parallel_pick_skill(skill: str) -> bool:
    return skill == "dual_arm.parallel_pick" or skill.startswith("dual_arm.parallel_pick_")


def _deep_merge_mapping(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = dict(base)
    for k, v in overlay.items():
        prev = out.get(k)
        if isinstance(prev, Mapping) and isinstance(v, Mapping):
            out[k] = _deep_merge_mapping(prev, v)
        else:
            out[k] = v
    return out


def _merge_parallel_pick_params(
    base_p: Mapping[str, Any],
    inline_p: Mapping[str, Any],
    scene_p: Mapping[str, Any],
) -> dict[str, Any]:
    # dual_arm.parallel_pick 的 left_pick/right_pick 需要深合并，避免 scene 仅覆盖路径时丢失其它字段。
    merged = _deep_merge_mapping(base_p, inline_p)
    return _deep_merge_mapping(merged, scene_p)


def _skill_defaults_for_param_key(param_key: str, skill_defaults: dict[str, Any]) -> dict[str, Any]:
    if param_key.startswith(_DRAWER_SKILL_PREFIX):
        return {
            **dict(skill_defaults.get("single_arm.drawer") or {}),
            **dict(skill_defaults.get(param_key) or {}),
        }
    if _inherits_dual_arm_carry_block(param_key):
        return {
            **dict(skill_defaults.get("dual_arm.carry") or {}),
            **dict(skill_defaults.get(param_key) or {}),
        }
    if _inherits_dual_arm_parallel_pick_block(param_key):
        return {
            **dict(skill_defaults.get("dual_arm.parallel_pick") or {}),
            **dict(skill_defaults.get(param_key) or {}),
        }
    if _inherits_dual_arm_place_block(param_key):
        return {
            **dict(skill_defaults.get("dual_arm.place") or {}),
            **dict(skill_defaults.get(param_key) or {}),
        }
    if _inherits_dual_arm_place_relative_block(param_key):
        return {
            **dict(skill_defaults.get("dual_arm.carry") or {}),
            **dict(skill_defaults.get("dual_arm.place_relative") or {}),
            **dict(skill_defaults.get(param_key) or {}),
        }
    return dict(skill_defaults.get(param_key) or {})


def _skill_defaults_for_block(skill: str, param_key: str, skill_defaults: dict[str, Any]) -> dict[str, Any]:
    """带 ``id`` 的 ``single_arm.pick`` / ``place`` / ``pregrasp`` 先铺对应 ``single_arm.*`` 再叠 ``skill_defaults[id]``。"""
    if skill == "dual_arm.handover_sync":
        merged = dict(skill_defaults.get("dual_arm.handover") or {})
        if param_key != "dual_arm.handover_sync":
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "dual_arm.carry" or skill.startswith("dual_arm.carry_"):
        merged = dict(skill_defaults.get("dual_arm.carry") or {})
        if param_key != skill:
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "dual_arm.place_relative":
        merged = {
            **dict(skill_defaults.get("dual_arm.carry") or {}),
            **dict(skill_defaults.get("dual_arm.place_relative") or {}),
        }
        if param_key != skill:
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "dual_arm.bimanual_align_mid_y":
        merged = dict(skill_defaults.get("dual_arm.bimanual_align_mid_y") or {})
        if param_key != skill:
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "dual_arm.place" or (
        skill.startswith("dual_arm.place_") and not skill.startswith("dual_arm.place_relative")
    ):
        merged = dict(skill_defaults.get("dual_arm.place") or {})
        if param_key != skill:
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "dual_arm.parallel_pick" or skill.startswith("dual_arm.parallel_pick_"):
        merged = dict(skill_defaults.get("dual_arm.parallel_pick") or {})
        if param_key != skill:
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "single_arm.pick":
        merged = dict(skill_defaults.get("single_arm.pick") or {})
        if param_key != "single_arm.pick":
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "single_arm.place":
        merged = dict(skill_defaults.get("single_arm.place") or {})
        if param_key != "single_arm.place":
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    if skill == "single_arm.pregrasp":
        pick_sd = dict(skill_defaults.get("single_arm.pick") or {})
        merged: dict[str, Any] = {}
        arm_v = pick_sd.get("arm")
        if isinstance(arm_v, str) and arm_v.strip():
            merged["arm"] = arm_v.strip()
        merged.update(dict(skill_defaults.get("single_arm.pregrasp") or {}))
        if param_key != "single_arm.pregrasp":
            merged.update(skill_defaults.get(param_key) or {})
        return merged
    return _skill_defaults_for_param_key(param_key, skill_defaults)


def _scene_params_for_param_key(param_key: str, scene_skill_params: dict[str, Any]) -> dict[str, Any]:
    if param_key.startswith(_DRAWER_SKILL_PREFIX):
        return {
            **dict(scene_skill_params.get("single_arm.drawer") or {}),
            **dict(scene_skill_params.get(param_key) or {}),
        }
    if _inherits_dual_arm_carry_block(param_key):
        return {
            **dict(scene_skill_params.get("dual_arm.carry") or {}),
            **dict(scene_skill_params.get(param_key) or {}),
        }
    if _inherits_dual_arm_place_block(param_key):
        return {
            **dict(scene_skill_params.get("dual_arm.place") or {}),
            **dict(scene_skill_params.get(param_key) or {}),
        }
    if _inherits_dual_arm_place_relative_block(param_key):
        return {
            **dict(scene_skill_params.get("dual_arm.carry") or {}),
            **dict(scene_skill_params.get("dual_arm.place_relative") or {}),
            **dict(scene_skill_params.get(param_key) or {}),
        }
    if _inherits_dual_arm_parallel_pick_block(param_key):
        return {
            **dict(scene_skill_params.get("dual_arm.parallel_pick") or {}),
            **dict(scene_skill_params.get(param_key) or {}),
        }
    return dict(scene_skill_params.get(param_key) or {})


def _scene_params_for_block(skill: str, param_key: str, scene_skill_params: dict[str, Any]) -> dict[str, Any]:
    if skill == "dual_arm.handover_sync":
        merged = dict(scene_skill_params.get("dual_arm.handover") or {})
        if param_key != "dual_arm.handover_sync":
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "dual_arm.carry" or skill.startswith("dual_arm.carry_"):
        merged = dict(scene_skill_params.get("dual_arm.carry") or {})
        if param_key != skill:
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "dual_arm.place_relative":
        merged = {
            **dict(scene_skill_params.get("dual_arm.carry") or {}),
            **dict(scene_skill_params.get("dual_arm.place_relative") or {}),
        }
        if param_key != skill:
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "dual_arm.bimanual_align_mid_y":
        merged = dict(scene_skill_params.get("dual_arm.bimanual_align_mid_y") or {})
        if param_key != skill:
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "dual_arm.place" or (
        skill.startswith("dual_arm.place_") and not skill.startswith("dual_arm.place_relative")
    ):
        merged = dict(scene_skill_params.get("dual_arm.place") or {})
        if param_key != skill:
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "dual_arm.parallel_pick" or skill.startswith("dual_arm.parallel_pick_"):
        merged = dict(scene_skill_params.get("dual_arm.parallel_pick") or {})
        if param_key != skill:
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "single_arm.pick":
        merged = dict(scene_skill_params.get("single_arm.pick") or {})
        if param_key != "single_arm.pick":
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "single_arm.place":
        merged = dict(scene_skill_params.get("single_arm.place") or {})
        if param_key != "single_arm.place":
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    if skill == "single_arm.pregrasp":
        pick_sp = dict(scene_skill_params.get("single_arm.pick") or {})
        merged = {}
        arm_v = pick_sp.get("arm")
        if isinstance(arm_v, str) and arm_v.strip():
            merged["arm"] = arm_v.strip()
        merged.update(dict(scene_skill_params.get("single_arm.pregrasp") or {}))
        if param_key != "single_arm.pregrasp":
            merged.update(scene_skill_params.get(param_key) or {})
        return merged
    return _scene_params_for_param_key(param_key, scene_skill_params)


def merge_task_queue_skill_params(
    blocks: list[Any],
    skill_defaults: dict[str, Any],
    scene_skill_params: dict[str, Any],
) -> list[Any]:
    """三层合并 task_queue block 的最终 params（与 motion CLI 逻辑一致）。

    合并优先级（后者覆盖前者）：
      1. ``skill_defaults``（``dual_arm.handover_sync`` 先铺 ``dual_arm.handover``；``dual_arm.carry*`` 铺
         ``dual_arm.carry``；``dual_arm.place*``（不含 ``place_relative``）铺 ``dual_arm.place``；
         ``dual_arm.place_relative`` 先铺 ``dual_arm.carry`` 再铺 ``dual_arm.place_relative``；
         ``dual_arm.bimanual_align_mid_y`` 铺 ``dual_arm.bimanual_align_mid_y``（带 ``id`` 时再叠 ``skill_defaults[id]``）；
         ``env.*`` 仅铺同名 ``skill_defaults`` / 场景键；
         ``single_arm.pick`` / ``place`` / ``pregrasp`` 带 ``id`` 时先铺对应 ``single_arm.*``；drawer 规则不变）
      2. ``block.params``
      3. ``scene_skill_params``（与 1 对称）

    ``param_key`` 为 block 的 ``id``（若存在），否则为 ``skill`` 名。
    """
    if not skill_defaults and not scene_skill_params:
        return blocks

    def _merge_one_block(block: Any) -> Any:
        if isinstance(block, dict):
            if "parallel" in block:
                merged_subs = [_merge_one_block(s) for s in block["parallel"]]
                return {**block, "parallel": merged_subs}
            skill = block.get("skill", "")
            param_key: str = block.get("id") or skill
            base_p = _skill_defaults_for_block(skill, param_key, skill_defaults)
            inline_p: dict[str, Any] = dict(block.get("params") or {})
            scene_p = _scene_params_for_block(skill, param_key, scene_skill_params)
            if _is_parallel_pick_skill(skill):
                merged_p = _merge_parallel_pick_params(base_p, inline_p, scene_p)
            else:
                merged_p = {**base_p, **inline_p, **scene_p}
            if merged_p:
                merged = dict(block)
                merged["params"] = merged_p
                return merged
            return block
        if isinstance(block, ParallelSpec):
            merged_skills = tuple(_merge_one_block(s) for s in block.skills)
            if merged_skills != block.skills:
                return ParallelSpec(skills=merged_skills)  # type: ignore[arg-type]
            return block
        if isinstance(block, BlockSpec):
            param_key = block.param_key
            base_p = _skill_defaults_for_block(block.skill, param_key, skill_defaults)
            inline_p = dict(block.params)
            scene_p = _scene_params_for_block(block.skill, param_key, scene_skill_params)
            if _is_parallel_pick_skill(block.skill):
                merged_p = _merge_parallel_pick_params(base_p, inline_p, scene_p)
            else:
                merged_p = {**base_p, **inline_p, **scene_p}
            if merged_p != dict(block.params):
                return _dc_replace(block, params=merged_p)
            return block
        return block

    return [_merge_one_block(b) for b in blocks]
