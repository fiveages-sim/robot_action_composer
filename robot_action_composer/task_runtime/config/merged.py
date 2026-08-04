"""合并后的队列一次运行配置：单臂切片 + 可选 carry / place / handover / drawer 几何。"""

from __future__ import annotations

from dataclasses import MISSING, dataclass, fields
from typing import TYPE_CHECKING, Any, Mapping

from robot_action_composer.motion_generation.tasks.bimanual_carry import BimanualCarryTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.bimanual_place import BimanualPlaceTaskConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_config_io import validate_runtime_defaults_keys  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
    format_queue_single_arm_summary,
    overlay_queue_single_arm_from_params,
    overlay_single_arm_pick_place,
)
from robot_action_composer.task_runtime.merge.utils import kwargs_for_dataclass  # pyright: ignore[reportMissingImports]

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


def _try_handover(values: Mapping[str, Any]) -> HandoverSyncConfig | None:
    if values.get("handover_position") is None:
        return None
    try:
        return _dataclass_from_combined(HandoverSyncConfig, values)
    except TypeError:
        return None


def _try_carry(values: Mapping[str, Any]) -> BimanualCarryTaskConfig | None:
    kw = kwargs_for_dataclass(BimanualCarryTaskConfig, values)
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


def _place_values_looks_like_simple(values: Mapping[str, Any]) -> bool:
    """仅 ``place_object_prim_path`` + ``place_offset``、无 ``place_half_span_y`` 的简化放置。"""
    po = values.get("place_object_prim_path")
    if po is None or (isinstance(po, str) and not po.strip()):
        return False
    return "place_offset" in values and "place_half_span_y" not in values


def _try_place(
    values: Mapping[str, Any],
    carry: BimanualCarryTaskConfig | None,
) -> BimanualPlaceTaskConfig | None:
    """完整 ``place_*`` 或简化 ``place_object_prim_path`` + ``place_offset``（几何从 carry 复用）。"""
    names = {f.name for f in fields(BimanualPlaceTaskConfig)}
    po = values.get("place_object_prim_path")
    if po is None or (isinstance(po, str) and not str(po).strip()):
        return None

    # 完整几何：含 place_half_span_y
    if "place_half_span_y" in values:
        kw = kwargs_for_dataclass(BimanualPlaceTaskConfig, values)
        if "place_half_span_y" not in kw:
            return None
        kw.pop("place_offset", None)
        try:
            return _dataclass_from_combined(BimanualPlaceTaskConfig, kw)
        except TypeError:
            return None

    # 简化：place_offset + 同任务中的 carry
    if not _place_values_looks_like_simple(values):
        return None
    if carry is None:
        return None

    try:
        off = _xyz3_tuple(values["place_offset"])
    except TypeError as e:
        raise ValueError(
            "dual_arm.place 简化形式需要 place_offset 为长度 3 的数值列表 [x,y,z]（米，物体系）。"
        ) from e

    merged: dict[str, Any] = {
        "place_object_prim_path": str(po).strip(),
        "place_half_span_y": carry.object_dual_arm_half_span_y,
        "place_prepare_offset": carry.carry_prepare_offset,
        "place_left_orientation": carry.left_base_orientation,
        "place_right_orientation": carry.right_base_orientation,
        "place_pregrasp_lower_xyz": carry.ee_pregrasp_lift_offset,
        "place_lift_xyz": carry.ee_lift_offset,
        "place_retreat_xyz": carry.ee_retreat_offset,
        "place_approach_clearance_y": carry.arm_merge_distance_y,
        "place_xyz": off,
    }
    for k, v in values.items():
        if k in ("place_object_prim_path", "place_offset"):
            continue
        if k in names:
            merged[k] = v
    kw = kwargs_for_dataclass(BimanualPlaceTaskConfig, merged)
    try:
        return _dataclass_from_combined(BimanualPlaceTaskConfig, kw)
    except TypeError:
        return None


def _resolve_place_config(
    place_values: Mapping[str, Any] | None,
    carry: BimanualCarryTaskConfig | None,
) -> BimanualPlaceTaskConfig | None:
    if not place_values:
        return None
    cfg = _try_place(place_values, carry=carry)
    if cfg is None and _place_values_looks_like_simple(place_values) and carry is None:
        raise ValueError(
            "dual_arm.place 简化形式（place_object_prim_path + place_offset）需要同一任务中配置 "
            "dual_arm.carry，以便复用双臂姿态、半宽、预接近与抬升/后撤等几何。"
        )
    return cfg


def _try_drawer(
    values: Mapping[str, Any],
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    active_object: str | None = None,
) -> Any | None:
    flat = dict(values)
    raw = flat.get("object_prim_path")
    if raw is None or (isinstance(raw, str) and not str(raw).strip()):
        from robot_action_composer.task_runtime.object_binding import (  # pyright: ignore[reportMissingImports]
            resolve_object_prim_path,
        )

        resolved = resolve_object_prim_path(
            flat,
            objects=objects,
            prim_param="object_prim_path",
            key_params="object_key",
            active_object=active_object,
            use_active_object=False,
            required=False,
            label="single_arm.drawer",
        )
        if resolved:
            flat["object_prim_path"] = resolved
        else:
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
    # 可选：任务 YAML（如 runtime_defaults / 场景 root 覆盖）覆盖 robot_config 的 Isaac base prim 路径
    base_link_entity_path: str | None = None
    #: 合并后的物体清单（``.meta`` ∪ 任务 ``objects`` ∪ scene ``objects``）
    objects: Mapping[str, Mapping[str, Any]] | None = None
    #: 技能省略 ``object_key`` 时的默认物体
    active_object: str | None = None


def _optional_base_link_entity_path(overrides: Mapping[str, Any]) -> str | None:
    raw = overrides.get("base_link_entity_path")
    if raw is None:
        return None
    if not isinstance(raw, str):
        return None
    s = raw.strip()
    return s or None


def build_merged_queue_config(
    *,
    runtime_defaults: Mapping[str, Any],
    skill_defaults: Mapping[str, Any] | None = None,
    scene_preset: Mapping[str, object] | None = None,
    base_runtime: MergedQueueConfig | None = None,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    active_object: str | None = None,
) -> MergedQueueConfig:
    """Build runtime config from structured overlays."""
    root_base = validate_runtime_defaults_keys(runtime_defaults or {}, context="runtime_defaults")
    root_scene = validate_runtime_defaults_keys(scene_preset or {}, context="scene preset root")
    sd = dict(skill_defaults or {})
    scene_sp = dict((scene_preset or {}).get("skill_params") or {})

    if base_runtime is None:
        single = QueueSingleArmSlice.from_overrides(root_base)
        single = overlay_single_arm_pick_place(
            single,
            dict(sd.get("single_arm.pick") or {}),
            dict(sd.get("single_arm.place") or {}),
        )
    else:
        single = base_runtime.single_arm
    single = overlay_queue_single_arm_from_params(single, root_scene)
    single = overlay_single_arm_pick_place(
        single,
        dict(scene_sp.get("single_arm.pick") or {}),
        dict(scene_sp.get("single_arm.place") or {}),
    )

    carry_base = (
        {f.name: getattr(base_runtime.carry, f.name) for f in fields(BimanualCarryTaskConfig)}
        if base_runtime and base_runtime.carry is not None
        else {}
    )
    carry_flat = {
        **carry_base,
        **dict(sd.get("dual_arm.carry") or {}),
        **dict(scene_sp.get("dual_arm.carry") or {}),
    }
    carry = _try_carry(carry_flat)

    handover_base = (
        {f.name: getattr(base_runtime.handover, f.name) for f in fields(HandoverSyncConfig)}
        if base_runtime and base_runtime.handover is not None
        else {}
    )
    handover_flat = {
        **handover_base,
        **dict(sd.get("dual_arm.handover") or {}),
        **dict(scene_sp.get("dual_arm.handover") or {}),
    }
    handover = _try_handover(handover_flat)

    drawer_base = {}
    if base_runtime and base_runtime.drawer is not None:
        from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]

        drawer_base = {f.name: getattr(base_runtime.drawer, f.name) for f in fields(DrawerGeometryConfig)}
    objs = objects if objects is not None else (base_runtime.objects if base_runtime is not None else None)
    act = active_object if active_object is not None else (
        base_runtime.active_object if base_runtime is not None else None
    )

    drawer_flat = {
        **drawer_base,
        **dict(sd.get("single_arm.drawer") or {}),
        **dict(scene_sp.get("single_arm.drawer") or {}),
    }
    drawer = _try_drawer(drawer_flat, objects=objs, active_object=act)

    place_base = (
        {f.name: getattr(base_runtime.place, f.name) for f in fields(BimanualPlaceTaskConfig)}
        if base_runtime and base_runtime.place is not None
        else {}
    )
    place_values = {
        **place_base,
        **dict(sd.get("dual_arm.place") or {}),
        **dict(scene_sp.get("dual_arm.place") or {}),
    }
    place = _resolve_place_config(place_values if place_values else None, carry=carry)

    base_link_raw = root_scene.get("base_link_entity_path", root_base.get("base_link_entity_path"))
    base_path = _optional_base_link_entity_path({"base_link_entity_path": base_link_raw}) if base_link_raw is not None else (
        base_runtime.base_link_entity_path if base_runtime is not None else None
    )
    return MergedQueueConfig(
        single_arm=single,
        carry=carry,
        place=place,
        handover=handover,
        drawer=drawer,
        base_link_entity_path=base_path,
        objects=objs,
        active_object=act,
    )


def merge_scene_preset_into_merged_queue(
    base: MergedQueueConfig,
    preset_raw: Mapping[str, object],
) -> MergedQueueConfig:
    """Overlay scene preset on top of existing runtime."""
    return build_merged_queue_config(
        runtime_defaults={},
        skill_defaults={},
        scene_preset=preset_raw,
        base_runtime=base,
    )


def format_merged_queue_summary(scene: str, cfg: MergedQueueConfig) -> str:
    parts = [format_queue_single_arm_summary(scene, cfg.single_arm)]
    if cfg.base_link_entity_path:
        parts.append(f"base_link_entity_path={cfg.base_link_entity_path}")
    if cfg.objects:
        parts.append(f"objects={sorted(cfg.objects)}")
    if cfg.active_object:
        parts.append(f"active_object={cfg.active_object}")
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
