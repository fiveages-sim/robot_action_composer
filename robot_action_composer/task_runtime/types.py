"""Shared types for IsaacSim task queue runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from numbers import Real
from typing import Any, Mapping, Union

from robot_action_composer.motion_generation.sequence.cartesian_stages import SendMode  # pyright: ignore[reportMissingImports]


@dataclass(frozen=True)
class ExecutionMeta:
    """Per-block execution options for :func:`execute_stage_sequence`."""

    send_mode: SendMode
    frame_id: str
    warn_prefix: str = "TaskQueue"
    left_arrival_guard_stage: str | None = None


@dataclass(frozen=True)
class BlockSpec:
    """One executable step in a task queue.

    ``skill`` is the registered skill name to execute.

    ``id`` is an optional logical name used as the parameter lookup key in
    ``skill_defaults`` and ``scene_presets.skill_params``.  This allows the
    same skill to appear multiple times in the queue with different
    configurations — e.g. two ``nav.navigate_to_object`` steps where one
    navigates to the pick location and another to the place location::

        task_queue:
          - skill: nav.navigate_to_object
            id: nav_to_pick
          - skill: nav.navigate_to_object
            id: nav_to_place

        skill_defaults:
          nav_to_pick:
            approach_offset_x: -0.5
            yaw: 0.0
          nav_to_place:
            approach_offset_x: 0.3
            yaw: 1.5708

    When ``id`` is absent the skill name is used as the lookup key (backward
    compatible).
    """

    skill: str
    params: Mapping[str, Any] = field(default_factory=dict)
    id: str | None = None
    start_delay_s: float = 0.0
    #: 为 True 时，在批量执行的非首段跳过本块（``__all__`` 第 2+ scene；chain 连续同 task_key）。
    skip_when_not_first_scene: bool = False

    @property
    def param_key(self) -> str:
        """Key used to look up defaults/scene overrides: ``id`` if set, else ``skill``."""
        return self.id if self.id else self.skill


@dataclass(frozen=True)
class ParallelSpec:
    """一组并行执行的 skill 步骤。

    所有子步骤同时派发，整个并行块在**所有**子步骤完成后才继续执行下一个块。
    若任意子步骤抛出异常，第一个异常将被重新抛出。

    YAML 语法示例::

        task_queue:
          - parallel:
              - skill: nav.navigate_to_object
                id: nav_to_pick
              - skill: joint.movej_to_config   # 导航期间同步运动到预抓取位置
                id: move_to_pre_grasp
          - skill: single_arm.pick

    子步骤中只允许 :class:`BlockSpec`（不支持嵌套 ``parallel:``）。
    """

    skills: tuple[BlockSpec, ...]


# 队列中合法的 block 类型
QueueBlock = Union[BlockSpec, ParallelSpec]


def block_spec_from_mapping(raw: Mapping[str, Any]) -> QueueBlock:
    if not isinstance(raw, Mapping):
        raise TypeError(f"block spec must be a mapping, got {type(raw).__name__}")

    # parallel 块：{parallel: [block, block, ...]}
    if "parallel" in raw:
        sub_raw = raw["parallel"]
        if not isinstance(sub_raw, (list, tuple)):
            raise TypeError('"parallel" value must be a list of block specs')
        skills: list[BlockSpec] = []
        for item in sub_raw:
            parsed = block_spec_from_mapping(item)
            if isinstance(parsed, ParallelSpec):
                raise ValueError("Nested parallel blocks are not supported")
            skills.append(parsed)
        return ParallelSpec(skills=tuple(skills))

    sk = raw.get("skill")
    if not isinstance(sk, str) or not sk.strip():
        raise ValueError('block spec requires non-empty string "skill"')
    params = raw.get("params", {})
    if params is not None and not isinstance(params, Mapping):
        raise TypeError('"params" must be a mapping when present')
    raw_id = raw.get("id")
    block_id = str(raw_id).strip() if raw_id is not None else None
    if block_id == "":
        block_id = None
    raw_delay = raw.get("start_delay_s", 0.0)
    if not isinstance(raw_delay, Real):
        raise TypeError('"start_delay_s" must be a number when present')
    start_delay_s = float(raw_delay)
    if start_delay_s < 0.0:
        raise ValueError('"start_delay_s" must be >= 0')
    skip_nf = raw.get("skip_when_not_first_scene", False)
    if not isinstance(skip_nf, bool):
        raise TypeError('"skip_when_not_first_scene" must be a boolean when present')
    return BlockSpec(
        skill=sk.strip(),
        params=dict(params or {}),
        id=block_id,
        start_delay_s=start_delay_s,
        skip_when_not_first_scene=skip_nf,
    )
