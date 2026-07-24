"""Task/scene object bindings and USD grasp-frame → object-local offset resolution."""

from __future__ import annotations

from collections.abc import Mapping, MutableMapping
from pathlib import Path
from typing import Any

_IDENTITY_XYZ = (0.0, 0.0, 0.0)
_IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)

GraspOffsetCache = MutableMapping[tuple[str, str], tuple[float, float, float]]


def normalize_prim_path(path: str) -> str:
    s = str(path or "").strip().rstrip("/")
    if not s:
        return ""
    if not s.startswith("/"):
        s = "/" + s
    while "//" in s:
        s = s.replace("//", "/")
    return s


def deep_merge_dicts(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    """Recursive dict merge; ``overlay`` wins. Non-dict values are replaced."""
    out: dict[str, Any] = dict(base)
    for key, value in overlay.items():
        if (
            key in out
            and isinstance(out[key], Mapping)
            and isinstance(value, Mapping)
        ):
            out[key] = deep_merge_dicts(out[key], value)
        else:
            out[key] = value
    return out


def load_leaf_meta_objects(leaf_dir: Path | str | None) -> dict[str, dict[str, Any]]:
    """Load ``<leaf>/.meta/objects/*.yaml`` into ``{object_key: {grasps, ...}}``.

    Each file may use ``object_key:`` or default to the file stem. ``grasps`` and
    optional ``object_prim_path`` are preserved.
    """
    if leaf_dir is None:
        return {}
    root = Path(leaf_dir)
    objects_dir = root / ".meta" / "objects"
    if not objects_dir.is_dir():
        return {}

    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as err:
        raise ImportError(
            "Loading .meta/objects requires PyYAML. Install with: pip install pyyaml"
        ) from err

    registry: dict[str, dict[str, Any]] = {}
    for path in sorted(objects_dir.glob("*.yaml")) + sorted(objects_dir.glob("*.yml")):
        if not path.is_file():
            continue
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if data is None:
            continue
        if not isinstance(data, dict):
            raise TypeError(f".meta/objects file must be a mapping: {path}")
        key_raw = data.get("object_key")
        key = str(key_raw).strip() if key_raw is not None else path.stem
        if not key:
            raise ValueError(f".meta/objects file missing object_key: {path}")
        entry = {k: v for k, v in data.items() if k != "object_key"}
        if key in registry:
            registry[key] = deep_merge_dicts(registry[key], entry)
        else:
            registry[key] = dict(entry)
    return registry


def merge_objects_registry(
    *,
    meta_objects: Mapping[str, Any] | None = None,
    task_objects: Mapping[str, Any] | None = None,
    scene_objects: Mapping[str, Any] | None = None,
) -> dict[str, dict[str, Any]]:
    """Merge leaf ``.meta`` ← task ``objects`` ← scene ``objects`` (later wins)."""
    merged: dict[str, Any] = {}
    for layer in (meta_objects, task_objects, scene_objects):
        if not layer:
            continue
        if not isinstance(layer, Mapping):
            raise TypeError(f"objects registry layer must be a mapping, got {type(layer).__name__}")
        for key, value in layer.items():
            ok = str(key).strip()
            if not ok:
                continue
            if not isinstance(value, Mapping):
                raise TypeError(f"objects[{ok!r}] must be a mapping, got {type(value).__name__}")
            prev = merged.get(ok) or {}
            merged[ok] = deep_merge_dicts(prev, dict(value))
    # Normalize grasps to str→str
    out: dict[str, dict[str, Any]] = {}
    for key, entry in merged.items():
        e = dict(entry)
        grasps_raw = e.get("grasps")
        if isinstance(grasps_raw, Mapping):
            e["grasps"] = {str(gk).strip(): str(gv).strip().strip("/") for gk, gv in grasps_raw.items() if str(gk).strip()}
        out[key] = e
    return out


def resolve_active_object(
    *,
    task_active: Any = None,
    scene_active: Any = None,
) -> str | None:
    for raw in (scene_active, task_active):
        if raw is None:
            continue
        s = str(raw).strip()
        if s:
            return s
    return None


def _compose_local_chain_translation(
    object_prim_path: str,
    grasp_prim_path: str,
) -> tuple[float, float, float]:
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_multiply,
        quat_normalize,
        rotate_vector_by_quat,
    )
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        try_get_prim_orient_local_xyzw,
        try_get_prim_translate_local,
    )

    obj = normalize_prim_path(object_prim_path)
    grasp = normalize_prim_path(grasp_prim_path)
    if not obj or not grasp:
        raise ValueError("object_prim_path and grasp_prim_path are required")
    if grasp == obj:
        return _IDENTITY_XYZ
    prefix = obj + "/"
    if not grasp.startswith(prefix):
        raise ValueError(
            f"grasp_prim_path {grasp!r} must be a descendant of object_prim_path {obj!r}"
        )
    segments = [s for s in grasp[len(prefix) :].split("/") if s]
    if not segments:
        return _IDENTITY_XYZ

    acc_t = _IDENTITY_XYZ
    acc_q = _IDENTITY_XYZW
    current = obj
    for seg in segments:
        current = f"{current}/{seg}"
        local_t = try_get_prim_translate_local(current)
        if local_t is None:
            local_t = _IDENTITY_XYZ
        local_q = try_get_prim_orient_local_xyzw(current)
        if local_q is None:
            local_q = _IDENTITY_XYZW
        else:
            local_q = quat_normalize(local_q)
        # Object←child = Object←parent * Parent←child
        rotated = rotate_vector_by_quat(local_t, acc_q)
        acc_t = (acc_t[0] + rotated[0], acc_t[1] + rotated[1], acc_t[2] + rotated[2])
        acc_q = quat_normalize(quat_multiply(acc_q, local_q))
    return float(acc_t[0]), float(acc_t[1]), float(acc_t[2])


def resolve_object_local_offset_from_grasp_prim(
    object_prim_path: str,
    grasp_prim_path: str,
    *,
    cache: GraspOffsetCache | None = None,
) -> tuple[float, float, float]:
    """Compose local xforms from ``object`` down to ``grasp`` → object-local offset.

    Missing ``xformOp:translate`` / ``xformOp:orient`` on a segment → identity for that op.
    """
    obj = normalize_prim_path(object_prim_path)
    grasp = normalize_prim_path(grasp_prim_path)
    key = (obj, grasp)
    if cache is not None and key in cache:
        return cache[key]
    print(f"[ObjectBind] resolving grasp offset via /get_prim_attribute: {obj} → {grasp}")
    offset = _compose_local_chain_translation(obj, grasp)
    print(
        f"[ObjectBind] grasp offset xyz=({offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f})"
    )
    if cache is not None:
        cache[key] = offset
    return offset


def _join_object_grasp(object_prim_path: str, relative: str) -> str:
    obj = normalize_prim_path(object_prim_path)
    rel = str(relative or "").strip().strip("/")
    if not obj:
        raise ValueError("object_prim_path is required to join grasp relative path")
    if not rel:
        raise ValueError("grasp relative path is empty")
    if rel.startswith("/"):
        return normalize_prim_path(rel)
    return normalize_prim_path(f"{obj}/{rel}")


def _lookup_object_entry(
    objects: Mapping[str, Mapping[str, Any]] | None,
    object_key: str | None,
) -> dict[str, Any] | None:
    if not objects or not object_key:
        return None
    entry = objects.get(object_key)
    if entry is None:
        raise KeyError(f"Unknown object_key {object_key!r}; known: {sorted(objects)}")
    if not isinstance(entry, Mapping):
        raise TypeError(f"objects[{object_key!r}] must be a mapping")
    return dict(entry)


def resolve_object_prim_path(
    params: Mapping[str, Any],
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    prim_param: str = "object_prim_path",
    key_params: str | tuple[str, ...] = "object_key",
    active_object: str | None = None,
    use_active_object: bool = False,
    required: bool = False,
    label: str = "object prim",
) -> str | None:
    """Resolve a scene prim path from skill params and/or ``objects`` registry.

    Priority:
    1. Explicit ``params[prim_param]`` (non-empty string)
    2. First non-empty among ``params[key]`` for ``key_params`` → ``objects[key].object_prim_path``
    3. If ``use_active_object`` and ``active_object`` → registry entry prim

    Returns ``None`` when nothing resolves and ``required`` is False.
    """
    if isinstance(key_params, str):
        key_names: tuple[str, ...] = (key_params,)
    else:
        key_names = tuple(key_params)

    prim_from_params = normalize_prim_path(str(params.get(prim_param) or ""))
    if prim_from_params:
        return prim_from_params

    object_key: str | None = None
    for kp in key_names:
        object_key = str(params.get(kp) or "").strip() or None
        if object_key:
            break
    if object_key is None and use_active_object and active_object:
        object_key = str(active_object).strip() or None

    entry = _lookup_object_entry(objects, object_key)
    if entry is not None:
        prim = normalize_prim_path(str(entry.get("object_prim_path") or ""))
        if prim:
            return prim
        if required:
            raise ValueError(
                f"{label}: object_key={object_key!r} has no object_prim_path "
                "(set on skill, task objects, or .meta/objects)"
            )
        return None

    if required:
        keys_hint = "/".join(key_names)
        raise ValueError(
            f"{label}: require '{prim_param}' or '{keys_hint}' "
            "(with objects registry binding)"
        )
    return None


def _parse_object_position_offset(raw_off: Any) -> tuple[float, float, float]:
    if isinstance(raw_off, (list, tuple)) and len(raw_off) == 3:
        return (float(raw_off[0]), float(raw_off[1]), float(raw_off[2]))
    raise TypeError(f"object_position_offset must be length-3 [x,y,z], got {raw_off!r}")


def resolve_pick_object_binding(
    params: Mapping[str, Any],
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    active_object: str | None = None,
    cache: GraspOffsetCache | None = None,
) -> tuple[str, tuple[float, float, float]]:
    """Resolve ``(object_prim_path, object_position_offset)`` for pick-like params.

    Resolution:
    1. Resolve ``object_prim_path`` from params / objects registry.
    2. If ``grasp_prim_path`` or ``grasp_id`` is set → auto offset from USD local chain.
    3. If ``object_position_offset`` is explicit:
       - with grasp → **add** (same object-local frame);
       - without grasp → use as-is (legacy).
    4. Else if grasp resolved → grasp offset; else ``(0,0,0)``.
    """
    object_key = str(params.get("object_key") or "").strip() or None
    if object_key is None and active_object:
        object_key = str(active_object).strip() or None
    entry = _lookup_object_entry(objects, object_key)

    prim_from_params = normalize_prim_path(str(params.get("object_prim_path") or ""))
    prim_from_entry = ""
    if entry is not None:
        prim_from_entry = normalize_prim_path(str(entry.get("object_prim_path") or ""))
    object_prim_path = prim_from_params or prim_from_entry

    extra_offset: tuple[float, float, float] | None = None
    if "object_position_offset" in params:
        if not object_prim_path:
            raise ValueError(
                "object_prim_path is required when object_position_offset is set "
                "(provide it on the skill or via objects binding)"
            )
        extra_offset = _parse_object_position_offset(params.get("object_position_offset"))

    grasp_prim = normalize_prim_path(str(params.get("grasp_prim_path") or ""))
    grasp_id = str(params.get("grasp_id") or "").strip() or None

    if grasp_id and not grasp_prim:
        if entry is None:
            raise ValueError(
                f"grasp_id={grasp_id!r} requires objects registry entry "
                f"(object_key={object_key!r} or active_object)"
            )
        grasps = entry.get("grasps") or {}
        if not isinstance(grasps, Mapping) or grasp_id not in grasps:
            known = sorted(grasps) if isinstance(grasps, Mapping) else []
            raise KeyError(f"Unknown grasp_id {grasp_id!r} for object_key={object_key!r}; known: {known}")
        if not object_prim_path:
            raise ValueError(
                f"object_prim_path missing for object_key={object_key!r} "
                "(set on skill, task objects, or .meta/objects)"
            )
        grasp_prim = _join_object_grasp(object_prim_path, str(grasps[grasp_id]))

    grasp_offset: tuple[float, float, float] | None = None
    if grasp_prim:
        if not object_prim_path:
            raise ValueError(
                "object_prim_path is required with grasp_prim_path "
                "(provide it on the skill or via objects binding)"
            )
        grasp_offset = resolve_object_local_offset_from_grasp_prim(
            object_prim_path, grasp_prim, cache=cache
        )

    if grasp_offset is not None and extra_offset is not None:
        return object_prim_path, (
            grasp_offset[0] + extra_offset[0],
            grasp_offset[1] + extra_offset[1],
            grasp_offset[2] + extra_offset[2],
        )
    if extra_offset is not None:
        return object_prim_path, extra_offset
    if grasp_offset is not None:
        return object_prim_path, grasp_offset

    if not object_prim_path:
        raise ValueError(
            "object_prim_path is required (skill params, objects binding, or legacy YAML)"
        )
    return object_prim_path, _IDENTITY_XYZ


def build_objects_for_task_entry(
    task_entry: Mapping[str, Any],
    *,
    scene: str,
) -> tuple[dict[str, dict[str, Any]], str | None]:
    """Merge ``.meta`` + task + scene objects for one task/scene segment.

    Expects ``task_entry['_config_dir']`` (leaf dir) when ``.meta/objects`` should load.
    """
    leaf = task_entry.get("_config_dir")
    meta = load_leaf_meta_objects(leaf)
    task_objects = task_entry.get("objects") if isinstance(task_entry.get("objects"), Mapping) else {}
    scene_presets = task_entry.get("scene_presets") or {}
    scene_sd = scene_presets.get(scene) if isinstance(scene_presets, Mapping) else None
    scene_objects = {}
    scene_active = None
    if isinstance(scene_sd, Mapping):
        if isinstance(scene_sd.get("objects"), Mapping):
            scene_objects = scene_sd["objects"]
        scene_active = scene_sd.get("active_object")
    registry = merge_objects_registry(
        meta_objects=meta,
        task_objects=task_objects,
        scene_objects=scene_objects,
    )
    active = resolve_active_object(
        task_active=task_entry.get("active_object"),
        scene_active=scene_active,
    )
    return registry, active


__all__ = [
    "GraspOffsetCache",
    "build_objects_for_task_entry",
    "deep_merge_dicts",
    "load_leaf_meta_objects",
    "merge_objects_registry",
    "normalize_prim_path",
    "resolve_active_object",
    "resolve_object_local_offset_from_grasp_prim",
    "resolve_object_prim_path",
    "resolve_pick_object_binding",
]
