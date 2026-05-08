"""单臂队列切片：common / pick / place 三段 dataclass（由结构化配置叠层解析）。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, replace
from typing import Any

from robot_action_composer.task_runtime.merge.utils import kwargs_for_dataclass


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

    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    prepare_offset: tuple[float, float, float] | None = None
    pick_clearance: float = 0.01
    ee_lift_offset: tuple[float, float, float] | None = None
    ee_retreat_offset: tuple[float, float, float] | None = None
    retreat_open_gripper: bool = False
    object_prim_path: str = ""
    ee_base_orientation: tuple[float, float, float, float] = (-0.7, 0.7, 0.0, 0.0)
    ee_pick_axis: str = "+z"
    ee_pick_direction_vector: tuple[float, float, float] | None = None
    motion_frame_id: str | None = None
    tf_lookup_timeout: float | None = None
    arm_movel_duration: float | None = None


@dataclass(frozen=True)
class QueueSlicePlace:
    object_prim_path: str = ""
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ee_place_axis: str | None = None
    prepare_offset: tuple[float, float, float] | None = None
    place_insert_clearance: float = 0.0
    place_position: tuple[float, float, float] | None = None
    ee_base_orientation: tuple[float, float, float, float] | None = None
    ee_retreat_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    motion_frame_id: str | None = None
    tf_lookup_timeout: float | None = None
    arm_movel_duration: float | None = None


QUEUE_SINGLE_ARM_KEYS: frozenset[str] = (
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
    def object_prim_path(self) -> str:
        return self.pick.object_prim_path

    @classmethod
    def from_overrides(cls, overrides: Mapping[str, Any]) -> QueueSingleArmSlice:
        """从结构化叠层后的 root overrides 解析单臂切片。"""
        ov = dict(overrides)
        common = QueueSliceCommon(**kwargs_for_dataclass(QueueSliceCommon, ov))
        pick = QueueSlicePick(**kwargs_for_dataclass(QueueSlicePick, ov))
        place_kw = kwargs_for_dataclass(QueueSlicePlace, ov)
        return cls(
            common=common,
            pick=pick,
            place=QueueSlicePlace(**place_kw),
        )


def format_queue_single_arm_summary(scene: str, task: QueueSingleArmSlice) -> str:
    """CLI 一行摘要。"""
    p, pl, c = task.pick, task.place, task.common
    parts = [
        f"[Scene] {scene} -> {p.object_prim_path}",
        f"arm={c.arm}, ee_pick_axis={p.ee_pick_axis}",
        f"orientation={p.ee_base_orientation}",
        f"prepare_offset={p.prepare_offset}, pick_clearance={p.pick_clearance}",
        f"ee_lift_offset={p.ee_lift_offset}, ee_retreat_offset={p.ee_retreat_offset}",
        f"object_position_offset={p.object_position_offset}",
        f"motion_frame_id={p.motion_frame_id}, arm_movel_duration={p.arm_movel_duration}",
    ]
    if pl.object_prim_path:
        parts.append(f"place_object={pl.object_prim_path}, place_offset={pl.object_position_offset}")
    else:
        parts.append(f"place_position={pl.place_position}")
    parts.append(f"ee_base_orientation={pl.ee_base_orientation}")
    parts.append(
        f"ee_place_axis={pl.ee_place_axis}, prepare_offset={pl.prepare_offset}, "
        f"place_insert={pl.place_insert_clearance}"
    )
    parts.append(f"place_motion_frame_id={pl.motion_frame_id}, place_arm_movel_duration={pl.arm_movel_duration}")
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
    q = overlay_queue_single_arm_pick_from_params(queue, pick_d)
    return overlay_queue_single_arm_place_from_params(q, place_d)


def overlay_queue_single_arm_pick_from_params(
    queue: QueueSingleArmSlice,
    params: Mapping[str, Any],
) -> QueueSingleArmSlice:
    """只将 ``params`` 叠到 ``common`` + ``pick``（不改 place）。"""
    if not params:
        return queue
    p = dict(params)
    co = kwargs_for_dataclass(QueueSliceCommon, p)
    pc = kwargs_for_dataclass(QueueSlicePick, p)
    new_common = replace(queue.common, **co) if co else queue.common
    new_pick = replace(queue.pick, **pc) if pc else queue.pick
    return QueueSingleArmSlice(common=new_common, pick=new_pick, place=queue.place)


def overlay_queue_single_arm_place_from_params(
    queue: QueueSingleArmSlice,
    params: Mapping[str, Any],
) -> QueueSingleArmSlice:
    """只将 ``params`` 叠到 ``common`` + ``place``（不改 pick）。"""
    if not params:
        return queue
    p = dict(params)
    co = kwargs_for_dataclass(QueueSliceCommon, p)
    pl = kwargs_for_dataclass(QueueSlicePlace, p)
    new_common = replace(queue.common, **co) if co else queue.common
    new_place = replace(queue.place, **pl) if pl else queue.place
    return QueueSingleArmSlice(common=new_common, pick=queue.pick, place=new_place)


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
