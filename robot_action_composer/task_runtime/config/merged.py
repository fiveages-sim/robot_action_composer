"""合并后的队列一次运行配置：单臂切片 + 可选 carry / handover / drawer 几何。"""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import TYPE_CHECKING, Any, Mapping

from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
    format_queue_single_arm_summary,
)
from robot_action_composer.task_runtime.merge.flat_presets import kwargs_for_dataclass  # pyright: ignore[reportMissingImports]

if TYPE_CHECKING:
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]


def _dataclass_from_combined(cls: type[Any], combined: Mapping[str, Any]) -> Any:
    kw: dict[str, Any] = {}
    for f in fields(cls):
        if f.name in combined:
            kw[f.name] = combined[f.name]
        elif f.default is not MISSING:
            kw[f.name] = f.default
        elif f.default_factory is not MISSING:
            kw[f.name] = f.default_factory()
        else:
            raise TypeError(f"{cls.__name__} missing required field {f.name!r}")
    return cls(**kw)


def _try_handover(flat: Mapping[str, Any]) -> HandoverSyncConfig | None:
    if flat.get("handover_position") is None:
        return None
    try:
        return _dataclass_from_combined(HandoverSyncConfig, flat)
    except TypeError:
        return None


def _try_carry(flat: Mapping[str, Any]) -> BimanualCarryTaskConfig | None:
    kw = kwargs_for_dataclass(BimanualCarryTaskConfig, flat)
    if "carry_half_span_y" not in kw:
        return None
    try:
        return _dataclass_from_combined(BimanualCarryTaskConfig, kw)
    except TypeError:
        return None


def _try_drawer(flat: Mapping[str, Any]) -> Any | None:
    if not flat.get("source_object_path_drawer"):
        return None
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]

    return DrawerGeometryConfig(**kwargs_for_dataclass(DrawerGeometryConfig, flat))


@dataclass(frozen=True)
class MergedQueueConfig:
    """扁平 merged_flat 解析结果，供 :func:`~robot_action_composer.task_runtime.runner.run_task_queue`。"""

    single_arm: QueueSingleArmSlice
    carry: BimanualCarryTaskConfig | None = None
    handover: HandoverSyncConfig | None = None
    drawer: DrawerGeometryConfig | None = None
    # 可选：任务 YAML（如 base_task_overrides / 场景扁平字段）覆盖 robot_config 的 Isaac base prim 路径
    base_link_entity_path: str | None = None


def _optional_base_link_entity_path(merged_flat: Mapping[str, Any]) -> str | None:
    raw = merged_flat.get("base_link_entity_path")
    if raw is None:
        return None
    if not isinstance(raw, str):
        return None
    s = raw.strip()
    return s or None


def build_merged_queue_from_flat(merged_flat: Mapping[str, Any]) -> MergedQueueConfig:
    base_path = _optional_base_link_entity_path(merged_flat)
    single = QueueSingleArmSlice.from_flat(merged_flat)
    handover = _try_handover(merged_flat)
    carry = _try_carry(merged_flat)
    drawer = _try_drawer(merged_flat)
    return MergedQueueConfig(
        single_arm=single,
        carry=carry,
        handover=handover,
        drawer=drawer,
        base_link_entity_path=base_path,
    )


def format_merged_queue_summary(scene: str, cfg: MergedQueueConfig) -> str:
    parts = [format_queue_single_arm_summary(scene, cfg.single_arm)]
    if cfg.base_link_entity_path:
        parts.append(f"base_link_entity_path={cfg.base_link_entity_path}")
    if cfg.drawer is not None:
        from robot_action_composer.motion_generation.tasks.drawer import format_drawer_task_cfg_summary  # pyright: ignore[reportMissingImports]

        parts.append(format_drawer_task_cfg_summary(scene, cfg.drawer, cfg.single_arm))
    if cfg.carry is not None:
        from robot_action_composer.motion_generation.tasks.bimanual_carry import format_bimanual_carry_task_cfg_summary  # pyright: ignore[reportMissingImports]

        parts.append(format_bimanual_carry_task_cfg_summary(scene, cfg.carry))
    if cfg.handover is not None:
        from robot_action_composer.motion_generation.tasks.handover import format_handover_sync_summary  # pyright: ignore[reportMissingImports]

        parts.append(format_handover_sync_summary(scene, cfg.handover))
    return " | ".join(parts)
