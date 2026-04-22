"""技能注册表：``task_queue`` 块中的 ``skill`` 字符串 → ``build_stages`` 可调用对象。"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

SkillFn = Callable[[Any, Mapping[str, Any]], Any]

_REGISTRY: dict[str, SkillFn] = {}


def register_skill(name: str, fn: SkillFn) -> None:
    _REGISTRY[name] = fn


def get_skill(name: str) -> SkillFn:
    if name not in _REGISTRY:
        raise KeyError(f"Unknown task_queue skill {name!r}; registered: {sorted(_REGISTRY)!r}")
    return _REGISTRY[name]
