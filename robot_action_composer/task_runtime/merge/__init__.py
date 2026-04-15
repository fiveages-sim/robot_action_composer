"""队列 preset 合并：扁平 YAML 片段与 task_queue 块级 params。"""

from __future__ import annotations

from robot_action_composer.task_runtime.merge.block_params import merge_task_queue_skill_params
from robot_action_composer.task_runtime.merge.flat_presets import (
    iter_leaf_block_specs,
    kwargs_for_dataclass,
    merge_flat_with_skill_carry,
    merge_flat_with_skill_drawer,
    merge_flat_with_skill_handover,
    merge_flat_with_skill_pick_place,
    merge_flat_with_skill_place,
    merge_scene_skill_overlays_into_flat,
    reset_settle_entity_path_from_queue,
    skill_carry_defaults_from_scene,
    skill_place_defaults_from_scene,
    skill_drawer_defaults_from_scene,
    skill_handover_defaults_from_scene,
    skill_pick_place_defaults_from_scene,
)

__all__ = [
    "iter_leaf_block_specs",
    "kwargs_for_dataclass",
    "merge_flat_with_skill_carry",
    "merge_flat_with_skill_drawer",
    "merge_flat_with_skill_handover",
    "merge_flat_with_skill_pick_place",
    "merge_flat_with_skill_place",
    "merge_scene_skill_overlays_into_flat",
    "merge_task_queue_skill_params",
    "reset_settle_entity_path_from_queue",
    "skill_carry_defaults_from_scene",
    "skill_place_defaults_from_scene",
    "skill_drawer_defaults_from_scene",
    "skill_handover_defaults_from_scene",
    "skill_pick_place_defaults_from_scene",
]
