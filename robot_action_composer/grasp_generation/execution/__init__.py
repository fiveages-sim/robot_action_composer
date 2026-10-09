"""Close-phase execution (B0–B5) for multi-hand grasp pipelines.

Layout::

    execution/
      types.py          ExecutorId, CloseRequest, logs
      profiles/         hand-specific joint groups + contact links
      contact/          ROS contact Bool / wrench adapters
      executors/        B0–B5 strategies
      runner.py         run_close() entry used by Viser / CLI
      close_log.py      JSONL + CSV for ablation tables
      wrist.py          B5 micro-press via OCS2

Primary stack today: Wuji Hand2 right + FiveAges W2. New hands add a profile
under ``profiles/`` and (if needed) a contact adapter; executors stay shared.
"""

from __future__ import annotations

from .executors import EXECUTOR_CHOICES, EXECUTOR_HELP_MD, EXECUTOR_LABELS, get_executor
from .runner import close_request_from_config, ensure_q_retarget, run_close
from .types import CloseRequest, CloseRunResult, ExecutorId

__all__ = [
    "ExecutorId",
    "CloseRequest",
    "CloseRunResult",
    "EXECUTOR_CHOICES",
    "EXECUTOR_LABELS",
    "EXECUTOR_HELP_MD",
    "get_executor",
    "run_close",
    "close_request_from_config",
    "ensure_q_retarget",
]
