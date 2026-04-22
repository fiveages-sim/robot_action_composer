"""可组合的任务队列运行时（``run_task_queue`` + 技能注册表）。

- 执行入口：``from robot_action_composer.task_runtime.runner import run_task_queue``
- 注册技能：``import robot_action_composer.task_runtime.skills``
- 合并配置：:mod:`robot_action_composer.task_runtime.config`、:mod:`robot_action_composer.task_runtime.merge`
"""

from __future__ import annotations

from robot_action_composer.task_runtime.context import (  # pyright: ignore[reportMissingImports]
    DrawerPhaseState,
    QueueRuntimeContext,
)
from robot_action_composer.task_runtime.types import (  # pyright: ignore[reportMissingImports]
    BlockSpec,
    ExecutionMeta,
    block_spec_from_mapping,
)

__all__ = [
    "BlockSpec",
    "DrawerPhaseState",
    "ExecutionMeta",
    "QueueRuntimeContext",
    "block_spec_from_mapping",
]
