#!/usr/bin/env python3
"""双臂放置任务配置与笛卡尔序列（与 :mod:`~.bimanual_carry` 几何对称、参数独立）。

YAML 可写**完整** ``place_*``，或使用**简化**形式：仅 ``place_object_entity_path`` + ``place_offset``
（相对放置参考中心的位移）；其余与 ``dual_arm.carry`` 一致，由合并逻辑从 carry 拷贝（需同任务定义 carry）。
"""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_bimanual_place_sequence,
)

# ``build_bimanual_place_sequence``：0～2 段逆 retreat/lift + 松爪 + Y 张开 + 后撤（共 3～5 段）
PLACE_ADVANCE_STAGE_COUNT_MAX = 2
PLACE_RELEASE_AND_SPREAD_TAIL_STAGE_COUNT = 3
PLACE_TOTAL_STAGES_MAX = PLACE_ADVANCE_STAGE_COUNT_MAX + PLACE_RELEASE_AND_SPREAD_TAIL_STAGE_COUNT


@dataclass(frozen=True)
class BimanualPlaceTaskConfig:
    """双臂对称放置几何（``skill_defaults.dual_arm.place`` → ``MergedQueueConfig.place``）。

    与 :class:`~.bimanual_carry.BimanualCarryTaskConfig` 字段一一对应，使用 ``place_*`` 前缀，
    便于与搬运参数独立调参。
    """

    place_object_entity_path: str
    place_half_span_y: float
    place_prepare_offset: tuple[float, float, float]
    place_left_orientation: tuple[float, float, float, float]
    place_right_orientation: tuple[float, float, float, float]
    place_approach_clearance_y: float = 0.0
    place_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    place_lift_xyz: tuple[float, float, float] | None = None
    place_retreat_xyz: tuple[float, float, float] | None = None


def format_bimanual_place_task_cfg_summary(scene: str, task_cfg: BimanualPlaceTaskConfig) -> str:
    return (
        f"[Scene] {scene} place -> {task_cfg.place_object_entity_path}, "
        f"place_half_span_y={task_cfg.place_half_span_y}, "
        f"place_prepare_offset={task_cfg.place_prepare_offset}, "
        f"place_lift_xyz={task_cfg.place_lift_xyz!r}, place_retreat_xyz={task_cfg.place_retreat_xyz!r}"
    )


def slice_place_stages_for_queue(
    full: list[StageTarget],
) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    """前 0～2 段为逆 carry 的 advance；接着 1 段 release；最后 2 段 spread + retreat。"""
    tail = PLACE_RELEASE_AND_SPREAD_TAIL_STAGE_COUNT
    if len(full) < tail:
        raise ValueError(f"place sequence too short: need >= {tail} stages, got {len(full)}")
    n_adv = len(full) - tail
    if n_adv > PLACE_ADVANCE_STAGE_COUNT_MAX:
        raise ValueError(
            f"place advance too long: at most {PLACE_ADVANCE_STAGE_COUNT_MAX} stages, got {n_adv}",
        )
    return full[0:n_adv], full[n_adv : n_adv + 1], full[n_adv + 1 :]


def build_bimanual_place_record_sequence(
    *,
    place_task_cfg: BimanualPlaceTaskConfig,
    object_center: Pose,
    gripper_open: float,
    gripper_closed: float,
    output_frame_id: str | None = None,
) -> list[StageTarget]:
    """由 :class:`BimanualPlaceTaskConfig` 构建完整放置 :class:`StageTarget` 列表。"""
    return build_bimanual_place_sequence(
        object_center=object_center,
        carry_half_span_y=place_task_cfg.place_half_span_y,
        carry_prepare_offset=place_task_cfg.place_prepare_offset,
        carry_approach_clearance_y=place_task_cfg.place_approach_clearance_y,
        carry_xyz=place_task_cfg.place_xyz,
        carry_left_orientation=place_task_cfg.place_left_orientation,
        carry_right_orientation=place_task_cfg.place_right_orientation,
        carry_lift_xyz=place_task_cfg.place_lift_xyz,
        carry_retreat_xyz=place_task_cfg.place_retreat_xyz,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        output_frame_id=output_frame_id,
    )
