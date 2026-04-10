"""Built-in skills（import 副作用：注册到 :mod:`robot_action_composer.task_runtime.registry`）。"""

from __future__ import annotations

from . import drawer as drawer  # noqa: F401
from . import dual_arm as dual_arm  # noqa: F401
from . import navigation as navigation  # noqa: F401
from . import single_arm as single_arm  # noqa: F401

__all__ = ["drawer", "dual_arm", "navigation", "single_arm"]
