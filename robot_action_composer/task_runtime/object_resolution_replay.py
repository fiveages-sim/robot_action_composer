"""Object pose resolution with pretty JSON live recording and replay."""

from __future__ import annotations

import json
import threading
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_entity_pose_world_service,
    get_object_pose_from_service,
)
from robot_action_composer.task_runtime.context import (  # pyright: ignore[reportMissingImports]
    ObjectResolutionSession,
    QueueRuntimeContext,
    get_effective_block_meta,
)

ObjectResolutionKey = tuple[str, str, str, str, str]
_record_file_lock = threading.Lock()
_NAV_WORLD_FRAMES = {"map", "world"}


def make_resolution_key(
    *,
    task_key: str,
    scene: str,
    block_key: str,
    arm_side: str,
    object_role: str,
) -> ObjectResolutionKey:
    return (task_key, scene, block_key, arm_side, object_role)


def pose_to_dict(pose: Pose) -> dict[str, Any]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def pose_from_dict(raw: Mapping[str, Any]) -> Pose:
    pos = raw.get("position")
    ori = raw.get("orientation")
    if not isinstance(pos, Mapping) or not isinstance(ori, Mapping):
        raise ValueError("pose record requires position and orientation mappings")
    pose = Pose()
    pose.position.x = float(pos["x"])
    pose.position.y = float(pos["y"])
    pose.position.z = float(pos["z"])
    pose.orientation.x = float(ori["x"])
    pose.orientation.y = float(ori["y"])
    pose.orientation.z = float(ori["z"])
    pose.orientation.w = float(ori["w"])
    return pose


def pose_from_world_tuple(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose


def _nav_world_frame_id(frame_id: str) -> str:
    nav_frame = str(frame_id).strip() or "map"
    if nav_frame not in _NAV_WORLD_FRAMES:
        raise ValueError(
            "navigation object resolution only supports frame_id 'map' or 'world' "
            f"because Isaac entity state returns world coordinates, got {nav_frame!r}"
        )
    return nav_frame


def _record_scene(record: Mapping[str, Any], *, path: Path, idx: int) -> str:
    raw = record.get("scene")
    if raw is None:
        raise ValueError(
            f"object resolution JSON record missing scene at {path}:records[{idx}]; "
            "re-record object-resolution JSON with scene-aware format"
        )
    scene = str(raw).strip()
    if not scene:
        raise ValueError(f"object resolution JSON record has empty scene at {path}:records[{idx}]")
    return scene


def _record_key(record: Mapping[str, Any], *, path: Path, idx: int) -> ObjectResolutionKey:
    return make_resolution_key(
        task_key=str(record["task_key"]),
        scene=_record_scene(record, path=path, idx=idx),
        block_key=str(record["block_key"]),
        arm_side=str(record["arm_side"]),
        object_role=str(record["object_role"]),
    )


def _records_from_json_payload(raw: Any, *, path: Path) -> list[dict[str, Any]]:
    if isinstance(raw, dict) and isinstance(raw.get("records"), list):
        records_raw = raw["records"]
    elif isinstance(raw, list):
        records_raw = raw
    elif isinstance(raw, dict):
        records_raw = [raw]
    else:
        raise ValueError(f"object resolution JSON must be an object or list: {path}")

    records: list[dict[str, Any]] = []
    for idx, record in enumerate(records_raw):
        if not isinstance(record, dict):
            raise ValueError(f"object resolution JSON record must be an object at {path}:records[{idx}]")
        records.append(record)
    return records


def load_json_index(path: str | Path) -> dict[ObjectResolutionKey, dict[str, Any]]:
    """Load object-resolution records from pretty JSON."""
    json_path = Path(path)
    if json_path.suffix.lower() != ".json":
        raise ValueError(f"object resolution record must be a .json file: {json_path}")
    if not json_path.is_file():
        raise FileNotFoundError(f"object resolution JSON not found: {json_path}")
    index: dict[ObjectResolutionKey, dict[str, Any]] = {}

    try:
        raw = json.loads(json_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid object resolution JSON: {json_path}") from exc
    for idx, record in enumerate(_records_from_json_payload(raw, path=json_path)):
        key = _record_key(record, path=json_path, idx=idx)
        if key in index:
            print(
                f"[ObjectResolution] duplicate key {key!r} at {json_path}:records[{idx}]; "
                "using latest record"
            )
        index[key] = record
    return index


def append_json_record(path: str | Path, record: Mapping[str, Any]) -> None:
    """Append one record to pretty JSON."""
    json_path = Path(path)
    if json_path.suffix.lower() != ".json":
        raise ValueError(f"object resolution record must be a .json file: {json_path}")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    with _record_file_lock:
        records: list[dict[str, Any]] = []
        if json_path.exists() and json_path.stat().st_size > 0:
            try:
                raw = json.loads(json_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid object resolution JSON: {json_path}") from exc
            records = _records_from_json_payload(raw, path=json_path)
        records.append(dict(record))
        payload = {"records": records}
        json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def build_object_resolution_session(
    *,
    mode: str,
    record_json_path: str | None = None,
    replay_json_path: str | None = None,
) -> ObjectResolutionSession:
    session = ObjectResolutionSession(
        mode=mode,
        record_json_path=record_json_path,
        replay_json_path=replay_json_path,
    )
    if mode == "replay":
        if not replay_json_path:
            raise ValueError("replay mode requires replay_json_path")
        session.replay_index = load_json_index(replay_json_path)
    return session


def resolve_nav_object_pose_for_task(
    ctx: QueueRuntimeContext,
    *,
    object_prim_path: str,
    frame_id: str = "map",
    object_role: str = "nav_target",
    entity_state_timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> Pose:
    """Resolve a navigation target object's pose in the navigation frame.

    Unlike ``resolve_object_pose_for_task``, this function does not convert the
    object pose into ``ctx.frame_id`` / robot base frame. It records and replays
    the pose used by navigation, normally ``frame_id='map'``. In Isaac GT setups
    the service returns world coordinates and ``map`` is configured to coincide
    with ``world``.
    """
    session = ctx.object_resolution
    block = get_effective_block_meta(ctx)
    path = str(object_prim_path).strip()
    if not path:
        role_for_error = str(object_role).strip() or "nav_target"
        raise ValueError(f"{role_for_error} requires non-empty object_prim_path")

    # Preserve the current non-recording behavior: ordinary navigation tasks
    # should still query Isaac directly when no object-resolution session is active.
    if session is None or block is None:
        position, orientation = get_entity_pose_world_service(
            path,
            timeout=entity_state_timeout,
            retries=retries,
            retry_delay=retry_delay,
        )
        return pose_from_world_tuple(position, orientation)

    scene = str(block.scene or ctx.scene).strip()
    if not scene:
        raise RuntimeError("navigation object resolution requires current scene")

    role = str(object_role).strip() or "nav_target"
    nav_frame = _nav_world_frame_id(frame_id)
    key = make_resolution_key(
        task_key=ctx.task_key,
        scene=scene,
        block_key=block.block_key,
        arm_side="none",
        object_role=role,
    )

    if session.mode == "replay":
        record = session.replay_index.get(key)
        if record is None:
            raise KeyError(
                "navigation object resolution replay miss: "
                f"task_key={ctx.task_key!r} scene={scene!r} block_key={block.block_key!r} "
                f"arm_side='none' object_role={role!r}"
            )
        record_frame = str(record.get("frame_id", "")).strip()
        if record_frame and record_frame != nav_frame:
            raise ValueError(
                "navigation object resolution frame mismatch: "
                f"record frame_id={record_frame!r}, requested frame_id={nav_frame!r}"
            )
        pose_raw = record.get("pose")
        if not isinstance(pose_raw, Mapping):
            raise ValueError(f"navigation object resolution replay record missing pose for key {key!r}")
        return pose_from_dict(pose_raw)

    if session.mode != "live":
        raise ValueError(f"unsupported object resolution mode: {session.mode!r}")

    position, orientation = get_entity_pose_world_service(
        path,
        timeout=entity_state_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    pose = pose_from_world_tuple(position, orientation)

    if session.record_json_path:
        append_json_record(
            session.record_json_path,
            {
                "task_key": ctx.task_key,
                "scene": scene,
                "block_key": block.block_key,
                "skill": block.skill,
                "block_index": block.block_index,
                "parallel_index": block.parallel_index,
                "arm_side": "none",
                "object_role": role,
                "object_prim_path": path,
                "include_orientation": True,
                "frame_id": nav_frame,
                "pose": pose_to_dict(pose),
            },
        )
    return pose


def resolve_object_pose_for_task(
    ctx: QueueRuntimeContext,
    *,
    object_prim_path: str,
    include_orientation: bool,
    arm_side: str,
    object_role: str,
    entity_state_timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> Pose:
    session = ctx.object_resolution
    block = get_effective_block_meta(ctx)
    if session is None or block is None:
        raise RuntimeError("object resolution requires ctx.object_resolution and ctx.current_block")
    scene = str(block.scene or ctx.scene).strip()
    if not scene:
        raise RuntimeError("object resolution requires current scene")
    key = make_resolution_key(
        task_key=ctx.task_key,
        scene=scene,
        block_key=block.block_key,
        arm_side=arm_side,
        object_role=object_role,
    )

    if session.mode == "replay":
        record = session.replay_index.get(key)
        if record is None:
            raise KeyError(
                "object resolution replay miss: "
                f"task_key={ctx.task_key!r} scene={scene!r} block_key={block.block_key!r} "
                f"arm_side={arm_side!r} object_role={object_role!r}"
            )
        pose_raw = record.get("pose")
        if not isinstance(pose_raw, Mapping):
            raise ValueError(
                f"object resolution replay record missing pose for key {key!r}"
            )
        return pose_from_dict(pose_raw)

    if session.mode != "live":
        raise ValueError(f"unsupported object resolution mode: {session.mode!r}")

    path = str(object_prim_path).strip()
    if not path:
        raise ValueError(f"{object_role} requires non-empty object_prim_path")

    pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        path,
        include_orientation=include_orientation,
        entity_state_timeout=entity_state_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )

    if session.record_json_path:
        append_json_record(
            session.record_json_path,
            {
                "task_key": ctx.task_key,
                "scene": scene,
                "block_key": block.block_key,
                "skill": block.skill,
                "block_index": block.block_index,
                "parallel_index": block.parallel_index,
                "arm_side": arm_side,
                "object_role": object_role,
                "object_prim_path": path,
                "include_orientation": include_orientation,
                "frame_id": str(ctx.frame_id),
                "pose": pose_to_dict(pose),
            },
        )
    return pose
