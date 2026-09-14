"""Optional grasp-generation package (HUG etc.).

Importing this package does NOT require HUG to be installed.
CLI entry: ``grasp-generation`` (see ``docs/GRASP_GENERATION.md``).
"""

from __future__ import annotations

__all__ = [
    "is_hug_available",
    "HugAdapter",
    "HugPaths",
    "HugGraspProposal",
    "GraspGenerationConfig",
    "load_grasp_config",
]


def is_hug_available() -> bool:
    """Return True if the ``hug`` package can be imported."""
    try:
        import hug  # noqa: F401

        return True
    except ImportError:
        return False


def __getattr__(name: str):
    if name in ("HugAdapter", "HugPaths", "HugGraspProposal"):
        from . import hug_adapter

        return getattr(hug_adapter, name)
    if name in ("GraspGenerationConfig", "load_grasp_config"):
        from . import config

        return getattr(config, name)
    raise AttributeError(name)
