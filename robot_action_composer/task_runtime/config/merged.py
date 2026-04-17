"""合并后的队列一次运行配置：单臂切片 + 可选 carry / place / handover / drawer 几何。"""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import TYPE_CHECKING, Any, Mapping

from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.bimanual_place import BimanualPlaceTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
    format_queue_single_arm_summary,
    overlay_queue_single_arm_from_params,
    overlay_single_arm_pick_place,
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
    if "object_dual_arm_half_span_y" not in kw:
        return None
    try:
        return _dataclass_from_combined(BimanualCarryTaskConfig, kw)
    except TypeError:
        return None


def _xyz3_tuple(v: Any) -> tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise TypeError(f"expected length-3 sequence [x,y,z], got {v!r}")
    return (float(v[0]), float(v[1]), float(v[2]))


def _place_flat_looks_like_simple(flat: Mapping[str, Any]) -> bool:
    """仅 ``place_object_entity_path`` + ``place_offset``、无 ``place_half_span_y`` 的简化放置。"""
    po = flat.get("place_object_entity_path")
    if po is None or (isinstance(po, str) and not po.strip()):
        return False
    return "place_offset" in flat and "place_half_span_y" not in flat


def _try_place(
    flat: Mapping[str, Any],
    carry: BimanualCarryTaskConfig | None,
) -> BimanualPlaceTaskConfig | None:
    """完整 ``place_*`` 或简化 ``place_object_entity_path`` + ``place_offset``（几何从 carry 复用）。"""
    names = {f.name for f in fields(BimanualPlaceTaskConfig)}
    po = flat.get("place_object_entity_path")
    if po is None or (isinstance(po, str) and not str(po).strip()):
        return None

    # 完整几何：含 place_half_span_y
    if "place_half_span_y" in flat:
        kw = kwargs_for_dataclass(BimanualPlaceTaskConfig, flat)
        if "place_half_span_y" not in kw:
            return None
        kw.pop("place_offset", None)
        try:
            return _dataclass_from_combined(BimanualPlaceTaskConfig, kw)
        except TypeError:
            return None

    # 简化：place_offset + 同任务中的 carry
    if not _place_flat_looks_like_simple(flat):
        return None
    if carry is None:
        return None

    try:
        off = _xyz3_tuple(flat["place_offset"])
    except TypeError as e:
        raise ValueError(
            "dual_arm.place 简化形式需要 place_offset 为长度 3 的数值列表 [x,y,z]（米，物体系）。"
        ) from e

    merged: dict[str, Any] = {
        "place_object_entity_path": str(po).strip(),
        "place_half_span_y": carry.object_dual_arm_half_span_y,
        "place_prepare_offset": carry.carry_prepare_offset,
        "place_left_orientation": carry.left_base_orientation,
        "place_right_orientation": carry.right_base_orientation,
        "place_lift_xyz": carry.ee_lift_offset,
        "place_retreat_xyz": carry.ee_retreat_offset,
        "place_approach_clearance_y": carry.arm_merge_distance_y,
        "place_xyz": off,
    }
    for k, v in flat.items():
        if k in ("place_object_entity_path", "place_offset"):
            continue
        if k in names:
            merged[k] = v
    kw = kwargs_for_dataclass(BimanualPlaceTaskConfig, merged)
    try:
        return _dataclass_from_combined(BimanualPlaceTaskConfig, kw)
    except TypeError:
        return None


def _resolve_place_config(
    place_flat: Mapping[str, Any] | None,
    carry: BimanualCarryTaskConfig | None,
) -> BimanualPlaceTaskConfig | None:
    if not place_flat:
        return None
    cfg = _try_place(place_flat, carry=carry)
    if cfg is None and _place_flat_looks_like_simple(place_flat) and carry is None:
        raise ValueError(
            "dual_arm.place 简化形式（place_object_entity_path + place_offset）需要同一任务中配置 "
            "dual_arm.carry，以便复用双臂姿态、半宽、预接近与抬升/后撤等几何。"
        )
    return cfg


def _try_drawer(flat: Mapping[str, Any]) -> Any | None:
    if not flat.get("source_object_path_drawer"):
        return None
    from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]

    return DrawerGeometryConfig(**kwargs_for_dataclass(DrawerGeometryConfig, flat))


@dataclass(frozen=True)
class MergedQueueConfig:
    """队列一次运行配置（单臂切片 + 可选 carry / place / handover / drawer），供 run_task_queue。"""

    single_arm: QueueSingleArmSlice
    carry: BimanualCarryTaskConfig | None = None
    place: BimanualPlaceTaskConfig | None = None
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


def merge_pick_place_skill_overlay(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    """single_arm.pick / single_arm.place 叠入同一 dict 时的 arm 规则（与 CLI 原 merge 一致）。"""
    pick_sd = dict(overlay.get("single_arm.pick") or {})
    place_sd = dict(overlay.get("single_arm.place") or {})
    if "arm" in pick_sd and "arm" in place_sd:
        place_sd = {k: v for k, v in place_sd.items() if k != "arm"}
    return {**base, **pick_sd, **place_sd}


def _merged_optional_task_fields_flat(cfg: MergedQueueConfig) -> dict[str, Any]:
    """carry / place / handover / drawer / base_link，不含 single_arm（用于与 scene 合并后做 _try_*）。"""
    out: dict[str, Any] = {}
    if cfg.base_link_entity_path is not None:
        out["base_link_entity_path"] = cfg.base_link_entity_path
    if cfg.carry is not None:
        for f in fields(BimanualCarryTaskConfig):
            out[f.name] = getattr(cfg.carry, f.name)
    if cfg.place is not None:
        for f in fields(BimanualPlaceTaskConfig):
            out[f.name] = getattr(cfg.place, f.name)
    if cfg.handover is not None:
        for f in fields(HandoverSyncConfig):
            out[f.name] = getattr(cfg.handover, f.name)
    if cfg.drawer is not None:
        from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]

        for f in fields(DrawerGeometryConfig):
            out[f.name] = getattr(cfg.drawer, f.name)
    return out


def merge_scene_preset_into_merged_queue(
    base: MergedQueueConfig,
    preset_raw: Mapping[str, object],
) -> MergedQueueConfig:
    """录制管线：在已有 runtime 上叠加场景 preset，单臂用 overlay，避免与 carry 等混用 to_flat/from_flat 往返。"""
    from robot_action_composer.task_config_io import flatten_queue_task_overrides  # pyright: ignore[reportMissingImports]

    scene_flat = flatten_queue_task_overrides(preset_raw)
    scene_sp = dict(preset_raw.get("skill_params") or {})

    single = overlay_queue_single_arm_from_params(base.single_arm, scene_flat)
    single = overlay_single_arm_pick_place(
        single,
        dict(scene_sp.get("single_arm.pick") or {}),
        dict(scene_sp.get("single_arm.place") or {}),
    )

    merged_flat = {**_merged_optional_task_fields_flat(base), **scene_flat}
    merged_flat = merge_pick_place_skill_overlay(
        merged_flat,
        {
            "single_arm.pick": dict(scene_sp.get("single_arm.pick") or {}),
            "single_arm.place": dict(scene_sp.get("single_arm.place") or {}),
        },
    )
    merged_flat = {**merged_flat, **dict(scene_sp.get("dual_arm.handover") or {})}
    merged_flat = {**merged_flat, **dict(scene_sp.get("dual_arm.carry") or {})}
    merged_flat = {**merged_flat, **dict(scene_sp.get("single_arm.drawer") or {})}

    base_path = _optional_base_link_entity_path(merged_flat)
    handover = _try_handover(merged_flat)
    carry = _try_carry(merged_flat)
    drawer = _try_drawer(merged_flat)
    scene_place_sp = dict(scene_sp.get("dual_arm.place") or {})
    base_place = (
        {f.name: getattr(base.place, f.name) for f in fields(BimanualPlaceTaskConfig)}
        if base.place is not None
        else {}
    )
    place_flat = {**base_place, **scene_place_sp}
    place = _resolve_place_config(place_flat, carry=carry)

    return MergedQueueConfig(
        single_arm=single,
        carry=carry,
        place=place,
        handover=handover,
        drawer=drawer,
        base_link_entity_path=base_path,
    )


def build_merged_queue_from_flat(
    merged_flat: Mapping[str, Any],
    place_flat: Mapping[str, Any] | None = None,
) -> MergedQueueConfig:
    """解析队列配置。

    ``merged_flat`` 仅用于通用/单臂/carry/handover/drawer；``place_flat`` 独立解析
    ``BimanualPlaceTaskConfig``（可直接来自 ``skill_defaults.dual_arm.place`` + 场景覆盖）。
    """
    base_path = _optional_base_link_entity_path(merged_flat)
    single = QueueSingleArmSlice.from_merged_key_dict(merged_flat)
    handover = _try_handover(merged_flat)
    carry = _try_carry(merged_flat)
    drawer = _try_drawer(merged_flat)
    place = _resolve_place_config(place_flat, carry=carry)
    return MergedQueueConfig(
        single_arm=single,
        carry=carry,
        place=place,
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
    if cfg.place is not None:
        from robot_action_composer.motion_generation.tasks.bimanual_place import format_bimanual_place_task_cfg_summary  # pyright: ignore[reportMissingImports]

        parts.append(format_bimanual_place_task_cfg_summary(scene, cfg.place))
    if cfg.handover is not None:
        from robot_action_composer.motion_generation.tasks.handover import format_handover_sync_summary  # pyright: ignore[reportMissingImports]

        parts.append(format_handover_sync_summary(scene, cfg.handover))
    return " | ".join(parts)
