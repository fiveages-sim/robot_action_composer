"""单臂队列切片：common / pick / place 三段 dataclass（由合并后的扁平 preset 解析）。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, replace
from typing import Any

from robot_action_composer.task_runtime.merge.flat_presets import kwargs_for_dataclass


@dataclass(frozen=True)
class QueueSliceCommon:
    """流程级公共字段（臂别、阶段超时、位姿容差等）。"""

    arm: str = "left"
    max_stage_duration: float = 2.0
    pose_tol_pos: float = 0.025
    pose_tol_ori: float = 0.08
    require_orientation_reach: bool = False
    use_object_orientation: bool = False


@dataclass(frozen=True)
class QueueSlicePick:
    """当前工作臂抓取几何（由 ``common.arm`` 决定左/右）。"""

    target_pose_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    approach_clearance: float = 0.2
    grasp_clearance: float = 0.01
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    retreat_direction_extra: float = 0.0
    retreat_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    retreat_xyz: tuple[float, float, float] | None = None
    source_object_entity_path: str = ""
    grasp_orientation: tuple[float, float, float, float] = (-0.7, 0.7, 0.0, 0.0)
    grasp_direction: str = "top"
    grasp_direction_vector: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class QueueSlicePlace:
    run_place_before_return: bool = False
    place_object_entity_path: str = ""
    place_pose_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    place_direction: str = "top"
    place_direction_vector: tuple[float, float, float] | None = None
    place_approach_clearance: float = 0.0
    place_insert_clearance: float = 0.0
    place_position: tuple[float, float, float] | None = None
    place_orientation: tuple[float, float, float, float] | None = None
    post_release_retract_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


QUEUE_SINGLE_ARM_FLAT_KEYS: frozenset[str] = (
    {f.name for f in fields(QueueSliceCommon)}
    | {f.name for f in fields(QueueSlicePick)}
    | {f.name for f in fields(QueueSlicePlace)}
)


@dataclass(frozen=True)
class QueueSingleArmSlice:
    """一次队列运行中的单臂配置切片（嵌套 common + pick + place）。"""

    common: QueueSliceCommon
    pick: QueueSlicePick
    place: QueueSlicePlace

    @property
    def arm(self) -> str:
        return self.common.arm

    @property
    def source_object_entity_path(self) -> str:
        return self.pick.source_object_entity_path

    @classmethod
    def from_merged_key_dict(cls, flat: Mapping[str, Any]) -> QueueSingleArmSlice:
        """从与 carry/handover 等混在同一 mapping 的合并键中解析（仅使用单臂相关字段名）。"""
        flat_n = dict(flat)
        common = QueueSliceCommon(**kwargs_for_dataclass(QueueSliceCommon, flat_n))
        pick = QueueSlicePick(**kwargs_for_dataclass(QueueSlicePick, flat_n))
        place_kw = kwargs_for_dataclass(QueueSlicePlace, flat_n)
        return cls(
            common=common,
            pick=pick,
            place=QueueSlicePlace(**place_kw),
        )


def format_queue_single_arm_summary(scene: str, task: QueueSingleArmSlice) -> str:
    """CLI 一行摘要。"""
    p, pl, c = task.pick, task.place, task.common
    parts = [
        f"[Scene] {scene} -> {p.source_object_entity_path}",
        f"arm={c.arm}, direction={p.grasp_direction}",
        f"orientation={p.grasp_orientation}",
        f"approach_clearance={p.approach_clearance}, grasp_clearance={p.grasp_clearance}",
        f"target_pose_offset={p.target_pose_offset}",
        f"retreat_direction_extra={p.retreat_direction_extra}, retreat_offset={p.retreat_offset}",
    ]
    if pl.run_place_before_return:
        if pl.place_object_entity_path:
            parts.append(f"place_object={pl.place_object_entity_path}, place_offset={pl.place_pose_offset}")
        else:
            parts.append(f"place_position={pl.place_position}")
        parts.append(f"place_orientation={pl.place_orientation}")
        parts.append(
            f"place_dir={pl.place_direction}, place_approach={pl.place_approach_clearance}, "
            f"place_insert={pl.place_insert_clearance}"
        )
    return ", ".join(parts)


def overlay_single_arm_pick_place(
    queue: QueueSingleArmSlice,
    pick_sd: Mapping[str, Any],
    place_sd: Mapping[str, Any],
) -> QueueSingleArmSlice:
    """与 ``merge_pick_place_skill_overlay`` 的 arm 规则一致：pick/place 同时指定 arm 时去掉 place 的 arm。"""
    pick_d = dict(pick_sd)
    place_d = dict(place_sd)
    if "arm" in pick_d and "arm" in place_d:
        place_d = {k: v for k, v in place_d.items() if k != "arm"}
    q = overlay_queue_single_arm_from_params(queue, pick_d)
    return overlay_queue_single_arm_from_params(q, place_d)


def overlay_queue_single_arm_from_params(
    queue: QueueSingleArmSlice,
    params: Mapping[str, Any],
) -> QueueSingleArmSlice:
    if not params:
        return queue
    p = dict(params)
    co = kwargs_for_dataclass(QueueSliceCommon, p)
    pc = kwargs_for_dataclass(QueueSlicePick, p)
    pl = kwargs_for_dataclass(QueueSlicePlace, p)
    new_common = replace(queue.common, **co) if co else queue.common
    new_pick = replace(queue.pick, **pc) if pc else queue.pick
    new_place = replace(queue.place, **pl) if pl else queue.place
    return QueueSingleArmSlice(common=new_common, pick=new_pick, place=new_place)
