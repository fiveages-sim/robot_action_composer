"""Task/scene object bindings and USD grasp-frame → object-local offset resolution."""

from __future__ import annotations

from collections.abc import Mapping, MutableMapping
from pathlib import Path
from typing import Any

_IDENTITY_XYZ = (0.0, 0.0, 0.0)
_IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)

GraspOffsetCache = MutableMapping[tuple[str, str], tuple[float, float, float]]
#: Cache for parent→tip relative pose: (xyz, xyzw).
PrimRelativePoseCache = MutableMapping[
    tuple[str, str], tuple[tuple[float, float, float], tuple[float, float, float, float]]
]


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


def _register_object_entry(
    registry: dict[str, dict[str, Any]],
    object_key: str,
    entry: Mapping[str, Any],
) -> None:
    key = str(object_key).strip()
    if not key:
        raise ValueError("object_key must be a non-empty string")
    if not isinstance(entry, Mapping):
        raise TypeError(f"objects[{key!r}] must be a mapping, got {type(entry).__name__}")
    payload = {k: v for k, v in entry.items() if k != "object_key"}
    if key in registry:
        registry[key] = deep_merge_dicts(registry[key], payload)
    else:
        registry[key] = dict(payload)


def _entries_from_meta_objects_file(
    data: Mapping[str, Any],
    *,
    path: Path,
) -> dict[str, Mapping[str, Any]]:
    """Parse one ``.meta/objects`` YAML into ``{object_key: entry}``.

    Supported shapes:
    1. Single object (legacy): ``object_key`` / stem + ``object_prim_path`` / ``grasps``
    2. Explicit multi: top-level ``objects: {key: {…}, …}``
    3. Implicit multi: ``{key: {object_prim_path|grasps: …}, …}`` (no root ``object_key`` /
       ``object_prim_path``)
    """
    objects_block = data.get("objects")
    if isinstance(objects_block, Mapping):
        out: dict[str, Mapping[str, Any]] = {}
        for raw_key, entry in objects_block.items():
            key = str(raw_key).strip()
            if not key:
                raise ValueError(f".meta/objects file has empty objects key: {path}")
            if not isinstance(entry, Mapping):
                raise TypeError(
                    f".meta/objects[{key!r}] must be a mapping in {path}, "
                    f"got {type(entry).__name__}"
                )
            out[key] = entry
        return out

    # Single-object file: root carries prim / grasps (optional explicit object_key).
    if "object_key" in data or "object_prim_path" in data or "grasps" in data:
        key_raw = data.get("object_key")
        key = str(key_raw).strip() if key_raw is not None else path.stem
        if not key:
            raise ValueError(f".meta/objects file missing object_key: {path}")
        return {key: data}

    # Implicit multi-object map: every value must itself be an object entry.
    if data and all(isinstance(v, Mapping) for v in data.values()):
        out = {}
        for raw_key, entry in data.items():
            key = str(raw_key).strip()
            if not key:
                raise ValueError(f".meta/objects file has empty object key: {path}")
            out[key] = entry
        return out

    raise TypeError(
        f".meta/objects file must be a single object, an objects: map, "
        f"or a map of object entries: {path}"
    )


def load_leaf_meta_objects(leaf_dir: Path | str | None) -> dict[str, dict[str, Any]]:
    """Load ``<leaf>/.meta/objects/*.yaml`` into ``{object_key: {grasps, ...}}``.

    Each file may declare one object (``object_key`` / file stem) or many objects
    (top-level ``objects:`` map, or a bare ``{key: entry, …}`` map). ``grasps`` and
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
        for key, entry in _entries_from_meta_objects_file(data, path=path).items():
            _register_object_entry(registry, key, entry)
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
        ensure_get_prim_attribute_service,
        try_get_prim_orient_local_xyzw,
        try_get_prim_translate_local,
    )

    obj = normalize_prim_path(object_prim_path)
    grasp = normalize_prim_path(grasp_prim_path)
    if not obj or not grasp:
        raise ValueError("object_prim_path and grasp_prim_path are required")
    if grasp == obj:
        return _IDENTITY_XYZ
    ensure_get_prim_attribute_service()
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


def _compose_local_chain_orient(
    object_prim_path: str,
    grasp_prim_path: str,
) -> tuple[float, float, float, float]:
    """Compose local ``xformOp:orient`` from ``object`` down to ``grasp`` → object←grasp quat."""
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_multiply,
        quat_normalize,
    )
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        ensure_get_prim_attribute_service,
        try_get_prim_orient_local_xyzw,
    )

    obj = normalize_prim_path(object_prim_path)
    grasp = normalize_prim_path(grasp_prim_path)
    if not obj or not grasp:
        raise ValueError("object_prim_path and grasp_prim_path are required")
    if grasp == obj:
        return _IDENTITY_XYZW
    ensure_get_prim_attribute_service()
    prefix = obj + "/"
    if not grasp.startswith(prefix):
        raise ValueError(
            f"grasp_prim_path {grasp!r} must be a descendant of object_prim_path {obj!r}"
        )
    segments = [s for s in grasp[len(prefix) :].split("/") if s]
    if not segments:
        return _IDENTITY_XYZW

    acc_q = _IDENTITY_XYZW
    current = obj
    for seg in segments:
        current = f"{current}/{seg}"
        local_q = try_get_prim_orient_local_xyzw(current)
        if local_q is None:
            local_q = _IDENTITY_XYZW
        else:
            local_q = quat_normalize(local_q)
        acc_q = quat_normalize(quat_multiply(acc_q, local_q))
    return float(acc_q[0]), float(acc_q[1]), float(acc_q[2]), float(acc_q[3])


def resolve_grasp_prim_path(
    params: Mapping[str, Any],
    *,
    objects: Mapping[str, Mapping[str, Any]] | None = None,
    object_prim_path: str | None = None,
) -> str | None:
    """Resolve full grasp/handle prim path from ``grasp_prim_path`` / ``grasp_id`` + objects."""
    grasp_prim = normalize_prim_path(str(params.get("grasp_prim_path") or ""))
    if grasp_prim:
        return grasp_prim
    grasp_id = str(params.get("grasp_id") or "").strip() or None
    if not grasp_id:
        return None
    object_key = str(params.get("object_key") or "").strip() or None
    entry = _lookup_object_entry(objects, object_key)
    if entry is None:
        raise ValueError(
            f"grasp_id={grasp_id!r} requires objects registry entry "
            f"(object_key={object_key!r})"
        )
    grasps = entry.get("grasps") or {}
    if not isinstance(grasps, Mapping) or grasp_id not in grasps:
        known = sorted(grasps) if isinstance(grasps, Mapping) else []
        raise KeyError(f"Unknown grasp_id {grasp_id!r} for object_key={object_key!r}; known: {known}")
    prim = normalize_prim_path(object_prim_path or str(params.get("object_prim_path") or ""))
    if not prim and entry is not None:
        prim = normalize_prim_path(str(entry.get("object_prim_path") or ""))
    if not prim:
        raise ValueError(
            f"object_prim_path missing for grasp_id={grasp_id!r} "
            "(set on skill, task objects, or .meta/objects)"
        )
    return _join_object_grasp(prim, str(grasps[grasp_id]))


def resolve_object_local_offset_from_grasp_prim(
    object_prim_path: str,
    grasp_prim_path: str,
    *,
    cache: GraspOffsetCache | None = None,
) -> tuple[float, float, float]:
    """Compose local xforms from ``object`` down to ``grasp`` → object-local offset.

    Reads USD-authored static locals via ``/get_prim_attribute``, then the pick skill
    stacks them onto the live rigid-body pose from ``/get_entity_state``.

    - **Service unavailable** → raise (probed inside the local-chain composer).
    - **Attribute not authored** on a segment → identity for that op (origin is OK).
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
    norm = (offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2) ** 0.5
    if norm < 1e-6:
        print(
            f"[ObjectBind] WARN: grasp offset for {obj} → {grasp} is ~zero; "
            "xformOp attributes may be missing (would grasp the object origin)."
        )
    elif norm > 2.0:
        print(
            f"[ObjectBind] WARN: grasp offset for {obj} → {grasp} is {norm:.3f} m; "
            "check the grasp prim path / authored transforms."
        )
    if cache is not None:
        cache[key] = offset
    return offset


def resolve_object_local_orient_from_grasp_prim(
    object_prim_path: str,
    grasp_prim_path: str,
    *,
    cache: MutableMapping[tuple[str, str], tuple[float, float, float, float]] | None = None,
) -> tuple[float, float, float, float]:
    """Compose local orients from ``object`` down to ``grasp`` → object←grasp quat xyzw.

    Service unavailable → raise (probed inside the local-chain composer);
    missing ``xformOp:orient`` → identity quat.
    """
    obj = normalize_prim_path(object_prim_path)
    grasp = normalize_prim_path(grasp_prim_path)
    key = (obj, grasp)
    if cache is not None and key in cache:
        return cache[key]
    print(f"[ObjectBind] resolving grasp orient via /get_prim_attribute: {obj} → {grasp}")
    orient = _compose_local_chain_orient(obj, grasp)
    print(
        f"[ObjectBind] grasp orient xyzw=({orient[0]:.6f}, {orient[1]:.6f}, "
        f"{orient[2]:.6f}, {orient[3]:.6f})"
    )
    if cache is not None:
        cache[key] = orient
    return orient


def _invert_pose_xyzw(
    xyz: tuple[float, float, float],
    xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_conjugate,
        quat_normalize,
        rotate_vector_by_quat,
    )

    q = quat_normalize(xyzw)
    q_inv = quat_conjugate(q)
    t_inv = rotate_vector_by_quat((-xyz[0], -xyz[1], -xyz[2]), q_inv)
    return t_inv, q_inv


def _multiply_pose_xyzw(
    a_xyz: tuple[float, float, float],
    a_xyzw: tuple[float, float, float, float],
    b_xyz: tuple[float, float, float],
    b_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """``T_a * T_b`` (apply b then a), quaternions xyzw."""
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_multiply,
        quat_normalize,
        rotate_vector_by_quat,
    )

    a_q = quat_normalize(a_xyzw)
    b_q = quat_normalize(b_xyzw)
    rotated = rotate_vector_by_quat(b_xyz, a_q)
    out_t = (a_xyz[0] + rotated[0], a_xyz[1] + rotated[1], a_xyz[2] + rotated[2])
    out_q = quat_normalize(quat_multiply(a_q, b_q))
    return out_t, out_q


def _common_prim_ancestor(path_a: str, path_b: str) -> str:
    sa = [s for s in path_a.strip("/").split("/") if s]
    sb = [s for s in path_b.strip("/").split("/") if s]
    i = 0
    while i < len(sa) and i < len(sb) and sa[i] == sb[i]:
        i += 1
    if i == 0:
        raise ValueError(f"no common ancestor between {path_a!r} and {path_b!r}")
    return "/" + "/".join(sa[:i])


def _compose_descendant_pose(
    ancestor_prim_path: str,
    descendant_prim_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose local xforms from ``ancestor`` down to ``descendant``."""
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_multiply,
        quat_normalize,
        rotate_vector_by_quat,
    )
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        try_get_prim_local_rotation_xyzw,
        try_get_prim_translate_local,
    )

    ancestor = normalize_prim_path(ancestor_prim_path)
    descendant = normalize_prim_path(descendant_prim_path)
    if descendant == ancestor:
        return _IDENTITY_XYZ, _IDENTITY_XYZW
    prefix = ancestor + "/"
    if not descendant.startswith(prefix):
        raise ValueError(
            f"descendant {descendant!r} must be under ancestor {ancestor!r}"
        )
    segments = [s for s in descendant[len(prefix) :].split("/") if s]
    acc_t = _IDENTITY_XYZ
    acc_q = _IDENTITY_XYZW
    current = ancestor
    for seg in segments:
        current = f"{current}/{seg}"
        local_t = try_get_prim_translate_local(current)
        if local_t is None:
            local_t = _IDENTITY_XYZ
        # orient 或 rotateXYZ（pad_left 等挂载常用 rotateXYZ，漏读会少 90°）
        local_q = try_get_prim_local_rotation_xyzw(current)
        if local_q is None:
            local_q = _IDENTITY_XYZW
        else:
            local_q = quat_normalize(local_q)
        rotated = rotate_vector_by_quat(local_t, acc_q)
        acc_t = (acc_t[0] + rotated[0], acc_t[1] + rotated[1], acc_t[2] + rotated[2])
        acc_q = quat_normalize(quat_multiply(acc_q, local_q))
    return (
        (float(acc_t[0]), float(acc_t[1]), float(acc_t[2])),
        (float(acc_q[0]), float(acc_q[1]), float(acc_q[2]), float(acc_q[3])),
    )


def _compose_local_chain_pose(
    parent_prim_path: str,
    tip_prim_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose tip pose in parent frame via USD local xforms.

    Supports tip as a descendant of parent, or a cousin under a shared ancestor
    (e.g. ``link7/tcp_origin`` ↔ ``link7/tcp/.../pad_tcp``).
    """
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        ensure_get_prim_attribute_service,
    )

    parent = normalize_prim_path(parent_prim_path)
    tip = normalize_prim_path(tip_prim_path)
    if not parent or not tip:
        raise ValueError("parent_prim_path and tip_prim_path are required")
    if tip == parent:
        return _IDENTITY_XYZ, _IDENTITY_XYZW
    ensure_get_prim_attribute_service()
    if tip.startswith(parent + "/"):
        return _compose_descendant_pose(parent, tip)
    ancestor = _common_prim_ancestor(parent, tip)
    anc_parent_t, anc_parent_q = _compose_descendant_pose(ancestor, parent)
    anc_tip_t, anc_tip_q = _compose_descendant_pose(ancestor, tip)
    parent_anc_t, parent_anc_q = _invert_pose_xyzw(anc_parent_t, anc_parent_q)
    return _multiply_pose_xyzw(parent_anc_t, parent_anc_q, anc_tip_t, anc_tip_q)


def _relative_pose_from_entity_world(
    parent_prim_path: str,
    tip_prim_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """``tip`` in ``parent`` from live ``/get_entity_state`` world poses (includes rotateXYZ + joints)."""
    from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
        quat_conjugate,
        quat_multiply,
        quat_normalize,
        rotate_vector_by_quat_inverse,
    )
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        get_entity_pose_world_service,
    )

    parent_t, parent_q = get_entity_pose_world_service(parent_prim_path)
    tip_t, tip_q = get_entity_pose_world_service(tip_prim_path)
    parent_q = quat_normalize(parent_q)
    tip_q = quat_normalize(tip_q)
    rel_world = (
        tip_t[0] - parent_t[0],
        tip_t[1] - parent_t[1],
        tip_t[2] - parent_t[2],
    )
    xyz = rotate_vector_by_quat_inverse(rel_world, parent_q)
    xyzw = quat_normalize(quat_multiply(quat_conjugate(parent_q), tip_q))
    return (
        (float(xyz[0]), float(xyz[1]), float(xyz[2])),
        (float(xyzw[0]), float(xyzw[1]), float(xyzw[2]), float(xyzw[3])),
    )


def resolve_prim_relative_pose(
    parent_prim_path: str,
    tip_prim_path: str,
    *,
    cache: PrimRelativePoseCache | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Resolve tip pose in parent frame for ``ee_tip`` conversion.

    Prefer live ``/get_entity_state`` (full xform + articulation). Fall back to
    ``/get_prim_attribute`` local chain (now includes ``rotateXYZ``).
    """
    parent = normalize_prim_path(parent_prim_path)
    tip = normalize_prim_path(tip_prim_path)
    key = (parent, tip)
    if cache is not None and key in cache:
        return cache[key]
    try:
        print(f"[ObjectBind] resolving ee_tip via /get_entity_state: {parent} → {tip}")
        xyz, xyzw = _relative_pose_from_entity_world(parent, tip)
        print(
            f"[ObjectBind] ee_tip (entity) xyz=({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}) "
            f"xyzw=({xyzw[0]:.6f}, {xyzw[1]:.6f}, {xyzw[2]:.6f}, {xyzw[3]:.6f})"
        )
    except Exception as exc:
        print(
            f"[ObjectBind] ee_tip entity-state failed ({exc}); "
            "falling back to /get_prim_attribute local chain"
        )
        xyz, xyzw = _compose_local_chain_pose(parent, tip)
        print(
            f"[ObjectBind] ee_tip (local) xyz=({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}) "
            f"xyzw=({xyzw[0]:.6f}, {xyzw[1]:.6f}, {xyzw[2]:.6f}, {xyzw[3]:.6f})"
        )
    if cache is not None:
        cache[key] = (xyz, xyzw)
    return xyz, xyzw


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
    "PrimRelativePoseCache",
    "build_objects_for_task_entry",
    "deep_merge_dicts",
    "load_leaf_meta_objects",
    "merge_objects_registry",
    "normalize_prim_path",
    "resolve_active_object",
    "resolve_grasp_prim_path",
    "resolve_object_local_offset_from_grasp_prim",
    "resolve_object_local_orient_from_grasp_prim",
    "resolve_object_prim_path",
    "resolve_pick_object_binding",
    "resolve_prim_relative_pose",
]
