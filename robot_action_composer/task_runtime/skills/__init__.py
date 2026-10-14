"""Built-in skills（import 副作用：注册到 :mod:`robot_action_composer.task_runtime.registry`）。"""

from __future__ import annotations

from . import dex_hand as dex_hand  # noqa: F401
from . import drawer as drawer  # noqa: F401
from . import dual_arm as dual_arm  # noqa: F401
from . import env as env  # noqa: F401
from . import joint as joint  # noqa: F401
from . import navigation as navigation  # noqa: F401
from . import session as session  # noqa: F401
from . import single_arm as single_arm  # noqa: F401

__all__ = [
    "dex_hand",
    "drawer",
    "dual_arm",
    "env",
    "joint",
    "navigation",
    "session",
    "single_arm",
]
