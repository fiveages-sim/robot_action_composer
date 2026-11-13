#!/usr/bin/env python3
"""Common helpers for Isaac ROS2 simulation control."""

from __future__ import annotations

import random
import threading
import time
from typing import Any, Callable, TypeVar

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from isaac_ros2_messages.srv import GetPrimAttribute, SetPrimAttribute
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
    rotate_vector_by_quat_inverse,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from simulation_interfaces.msg import Result as SimResult
from simulation_interfaces.srv import GetEntityState, SetEntityState, SetSimulationState

SERVICE_CALL_TIMEOUT = 8.0
SERVICE_CALL_RETRIES = 3
SERVICE_RETRY_DELAY = 0.2
# Probe USD attrs via /get_prim_attribute: keep short — missing attrs are common and
# should not burn retries×8s (that looked like a task-queue hang).
PRIM_ATTR_PROBE_TIMEOUT = 2.0
PRIM_ATTR_PROBE_RETRIES = 1
T = TypeVar("T")

_service_node: Node | None = None
_service_clients: dict[str, object] = {}
# RLock: node/client create and call+spin share one lock (serialize reuse).
_service_lock = threading.RLock()


def _ensure_service_node() -> Node:
    """Shared Isaac service client node.

    Uses **wall clock** (``use_sim_time=False``). ``wait_for_service`` under sim time
    can block forever if this node is not spun on ``/clock`` while waiting.
    """
    global _service_node
    with _service_lock:
        if _service_node is not None:
            return _service_node
        if not rclpy.ok():
            rclpy.init()
        _service_node = rclpy.create_node(
            "lerobot_isaac_service_client",
            parameter_overrides=[Parameter("use_sim_time", value=False)],
            automatically_declare_parameters_from_overrides=True,
        )
        return _service_node


def _get_service_client(service_type: type, service_name: str) -> tuple[Node, object]:
    """Reuse one client per service name (avoid destroy-after-timeout → Isaac sendResponse spam)."""
    node = _ensure_service_node()
    with _service_lock:
        client = _service_clients.get(service_name)
        if client is None:
            client = node.create_client(service_type, service_name)
            _service_clients[service_name] = client
        return node, client


def _wait_for_service_wall(client: object, node: Node, timeout: float) -> bool:
    """Wait for a service using wall time + ``spin_once`` (discovery / responses)."""
    deadline = time.monotonic() + max(0.1, float(timeout))
    while time.monotonic() < deadline:
        if client.service_is_ready():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return bool(client.service_is_ready())


def _call_service_once(
    service_type: type,
    service_name: str,
    request: object,
    *,
    timeout: float = SERVICE_CALL_TIMEOUT,
) -> object:
    node, client = _get_service_client(service_type, service_name)
    with _service_lock:
        if not _wait_for_service_wall(client, node, timeout):
            raise RuntimeError(f"Service '{service_name}' unavailable within {timeout:.1f}s")
        future = client.call_async(request)
        start = time.monotonic()
        while not future.done():
            if (time.monotonic() - start) > timeout:
                raise RuntimeError(f"Service '{service_name}' timed out after {timeout:.1f}s")
            rclpy.spin_once(node, timeout_sec=0.01)
        if future.exception() is not None:
            raise RuntimeError(f"Service '{service_name}' call failed: {future.exception()}")
        return future.result()


def _call_service_with_retry(
    call_fn: Callable[[], T],
    description: str,
    *,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> T:
    attempts = max(1, int(retries))
    last_error: Exception | None = None
    for attempt_idx in range(attempts):
        try:
            return call_fn()
        except Exception as exc:
            last_error = exc
            if attempt_idx < (attempts - 1):
                time.sleep(max(0.0, retry_delay))
    raise RuntimeError(
        f"{description} failed after {attempts} attempts: {last_error or 'unknown error'}"
    )


class SimTimeHelper:
    """Provide ROS(sim) time utilities."""

    def __init__(self, poll_period: float = 0.01) -> None:
        self._poll_period = poll_period
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(
            "lerobot_sim_time_helper",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        use_sim_time = self._node.get_parameter("use_sim_time").value
        print(f"[Clock] use_sim_time={use_sim_time}")

    def now_seconds(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, duration: float) -> None:
        if duration <= 0.0:
            return
        start = self.now_seconds()
        while (self.now_seconds() - start) < duration:
            time.sleep(self._poll_period)

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None


def _ensure_sim_result_ok(result_msg: Any, *, what: str) -> None:
    """Raise unless ``simulation_interfaces/Result.result == RESULT_OK``.

    Isaac often fills ``error_message`` with a success sentence (e.g. SetEntityState
    \"Successfully set state...\"). Treat the numeric ``result`` code as authoritative.
    """
    code = int(getattr(result_msg, "result", SimResult.RESULT_OPERATION_FAILED))
    if code == int(SimResult.RESULT_OK):
        return
    detail = str(getattr(result_msg, "error_message", "") or "").strip()
    raise RuntimeError(f"{what} unsuccessful (code={code})" + (f": {detail}" if detail else ""))


def set_simulation_state(
    state: int,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    request = SetSimulationState.Request()
    request.state.state = int(state)
    result = _call_service_with_retry(
        lambda: _call_service_once(
            SetSimulationState,
            "/set_simulation_state",
            request,
            timeout=timeout,
        ),
        f"set_simulation_state({state})",
        retries=retries,
        retry_delay=retry_delay,
    )
    _ensure_sim_result_ok(result.result, what=f"set_simulation_state({state})")


def _parse_prim_attr_vec(value: str, *, expected_len: int, path: str, attribute: str) -> np.ndarray:
    vec = np.fromstring(value.strip().strip("[]"), sep=",")
    if vec.shape[0] != expected_len:
        raise RuntimeError(
            f"Invalid {attribute} for '{path}': expected {expected_len} values, got {value!r}"
        )
    return vec


def ensure_get_prim_attribute_service(
    *,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
) -> None:
    """Raise if ``/get_prim_attribute`` is not advertised (ROS2ServicePrim missing).

    ``isaacsim.ros2.sim_control`` does **not** provide this service; the robot ROS
    OmniGraph needs an ``isaacsim.ros2.bridge.ROS2ServicePrim`` node (see W2 / G1D).
    """
    node, client = _get_service_client(GetPrimAttribute, "/get_prim_attribute")
    if not _wait_for_service_wall(client, node, timeout):
        raise RuntimeError(
            "/get_prim_attribute unavailable within "
            f"{timeout:.1f}s — add OmniGraph node isaacsim.ros2.bridge.ROS2ServicePrim "
            "to the robot ROS payload (not provided by isaacsim.ros2.sim_control)."
        )


def try_get_prim_attribute(
    path: str,
    attribute: str,
    *,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
    retries: int = PRIM_ATTR_PROBE_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> str | None:
    """Return attribute value string, or ``None`` if the attribute is not authored.

    Distinguishes failure modes:
    - **Service down / call error** → raise (caller must not treat as identity).
    - **Attribute missing** (``success=False`` / empty) → ``None`` (identity is OK).
    """
    request = GetPrimAttribute.Request()
    request.path = path
    request.attribute = attribute
    result = _call_service_with_retry(
        lambda: _call_service_once(
            GetPrimAttribute,
            "/get_prim_attribute",
            request,
            timeout=timeout,
        ),
        f"try_get_prim_attribute('{path}', '{attribute}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    if not getattr(result, "success", False):
        return None
    raw = getattr(result, "value", None)
    if raw is None:
        return None
    text = str(raw).strip()
    return text or None


def get_prim_translate_local(
    path: str,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> tuple[float, float, float]:
    request = GetPrimAttribute.Request()
    request.path = path
    request.attribute = "xformOp:translate"
    result = _call_service_with_retry(
        lambda: _call_service_once(
            GetPrimAttribute,
            "/get_prim_attribute",
            request,
            timeout=timeout,
        ),
        f"get_prim_translate_local('{path}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    if not result.success:
        raise RuntimeError(
            f"get_prim_attribute unsuccessful for '{path}': {result.message or 'unknown error'}"
        )
    vec = _parse_prim_attr_vec(result.value, expected_len=3, path=path, attribute="xformOp:translate")
    return float(vec[0]), float(vec[1]), float(vec[2])


def try_get_prim_translate_local(
    path: str,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
    retries: int = PRIM_ATTR_PROBE_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> tuple[float, float, float] | None:
    """Local translate, or ``None`` if attribute not authored (treat as identity).

    Service unavailable raises (via ``try_get_prim_attribute``); do not map that to identity.
    """
    raw = try_get_prim_attribute(
        path,
        "xformOp:translate",
        timeout=timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    if raw is None:
        return None
    try:
        vec = _parse_prim_attr_vec(raw, expected_len=3, path=path, attribute="xformOp:translate")
    except RuntimeError:
        return None
    return float(vec[0]), float(vec[1]), float(vec[2])


def try_get_prim_orient_local_xyzw(
    path: str,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
    retries: int = PRIM_ATTR_PROBE_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> tuple[float, float, float, float] | None:
    """Local ``xformOp:orient`` as xyzw, or ``None`` if not authored (treat as identity).

    Omniverse authors orient as **wxyz**; return value is ROS-style **xyzw**.
    Service unavailable raises; missing attribute → ``None``.
    """
    raw = try_get_prim_attribute(
        path,
        "xformOp:orient",
        timeout=timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    if raw is None:
        return None
    try:
        vec = _parse_prim_attr_vec(raw, expected_len=4, path=path, attribute="xformOp:orient")
    except RuntimeError:
        return None
    w, x, y, z = float(vec[0]), float(vec[1]), float(vec[2]), float(vec[3])
    return x, y, z, w


def _quat_xyzw_from_rotate_xyz_deg(
    rx_deg: float, ry_deg: float, rz_deg: float
) -> tuple[float, float, float, float]:
    """USD ``xformOp:rotateXYZ`` (degrees, apply X then Y then Z) → ROS xyzw."""
    import math

    hx = math.radians(float(rx_deg)) * 0.5
    hy = math.radians(float(ry_deg)) * 0.5
    hz = math.radians(float(rz_deg)) * 0.5
    qx = (math.sin(hx), 0.0, 0.0, math.cos(hx))
    qy = (0.0, math.sin(hy), 0.0, math.cos(hy))
    qz = (0.0, 0.0, math.sin(hz), math.cos(hz))
    # Apply Rx then Ry then Rz → q = qz ⊗ qy ⊗ qx
    return quat_normalize(quat_multiply(qz, quat_multiply(qy, qx)))


def try_get_prim_local_rotation_xyzw(
    path: str,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
    retries: int = PRIM_ATTR_PROBE_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> tuple[float, float, float, float] | None:
    """Local rotation as xyzw from ``orient`` or ``rotateXYZ`` (common on pad mounts).

    Missing both → ``None`` (caller may treat as identity). Prefer ``orient`` when set.
    """
    q = try_get_prim_orient_local_xyzw(
        path, timeout=timeout, retries=retries, retry_delay=retry_delay
    )
    if q is not None:
        return q
    raw = try_get_prim_attribute(
        path,
        "xformOp:rotateXYZ",
        timeout=timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    if raw is None:
        return None
    try:
        vec = _parse_prim_attr_vec(raw, expected_len=3, path=path, attribute="xformOp:rotateXYZ")
    except RuntimeError:
        return None
    return _quat_xyzw_from_rotate_xyz_deg(float(vec[0]), float(vec[1]), float(vec[2]))


def set_prim_translate_local(
    path: str,
    xyz: tuple[float, float, float],
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    x, y, z = xyz
    request = SetPrimAttribute.Request()
    request.path = path
    request.attribute = "xformOp:translate"
    request.value = f"[{x}, {y}, {z}]"
    result = _call_service_with_retry(
        lambda: _call_service_once(
            SetPrimAttribute,
            "/set_prim_attribute",
            request,
            timeout=timeout,
        ),
        f"set_prim_translate_local('{path}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    if not result.success:
        raise RuntimeError(
            f"set_prim_attribute unsuccessful for '{path}': {result.message or 'unknown error'}"
        )


def set_prim_orientation_local(
    path: str,
    quat_wxyz: tuple[float, float, float, float],
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    w, x, y, z = quat_wxyz
    request = SetPrimAttribute.Request()
    request.path = path
    request.attribute = "xformOp:orient"
    request.value = f"[{w}, {x}, {y}, {z}]"
    result = _call_service_with_retry(
        lambda: _call_service_once(
            SetPrimAttribute,
            "/set_prim_attribute",
            request,
            timeout=timeout,
        ),
        f"set_prim_translate_local('{path}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    if not result.success:
        raise RuntimeError(
            f"set_prim_attribute unsuccessful for '{path}': {result.message or 'unknown error'}"
        )
    

def set_entity_pose_world_service(
    entity: str,
    position: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    *,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    """Teleport an Isaac entity in world frame and zero linear/angular velocity."""
    request = SetEntityState.Request()
    request.entity = entity
    px, py, pz = position
    qx, qy, qz, qw = orientation_xyzw
    request.state.pose.position.x = float(px)
    request.state.pose.position.y = float(py)
    request.state.pose.position.z = float(pz)
    request.state.pose.orientation.x = float(qx)
    request.state.pose.orientation.y = float(qy)
    request.state.pose.orientation.z = float(qz)
    request.state.pose.orientation.w = float(qw)
    # Explicit zero twist so a mid-sim teleport does not keep ejection velocity.
    request.state.twist.linear.x = 0.0
    request.state.twist.linear.y = 0.0
    request.state.twist.linear.z = 0.0
    request.state.twist.angular.x = 0.0
    request.state.twist.angular.y = 0.0
    request.state.twist.angular.z = 0.0
    result = _call_service_with_retry(
        lambda: _call_service_once(
            SetEntityState,
            "/set_entity_state",
            request,
            timeout=timeout,
        ),
        f"set_entity_pose_world_service('{entity}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    _ensure_sim_result_ok(result.result, what=f"set_entity_state('{entity}')")


def randomize_object_xyz_after_reset(
    object_prim_path: str,
    enabled: bool = True,
    xyz_offset: tuple[float, float, float] | float = 0.04,
    *,
    also_move_prim_paths: tuple[str, ...] | list[str] = (),
    seat_translate_offsets: dict[str, tuple[float, float, float]] | None = None,
    pause_during_move: bool = False,
    settle_after_s: float = 0.8,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    """Apply a small random **local** translation to the object prim after reset.

    Each axis uses an **independent uniform** draw in ``[-w, +w]`` added to the
    current local translation, where ``w`` is the corresponding component of
    ``xyz_offset`` (or the scalar for x/y and 0 for z when ``xyz_offset`` is a float).
    This is **not** a Gaussian / normal distribution.

    Loose child objects (e.g. a dynamic test tube sitting in a rack) are **not**
    USD children of the rack. Moving only the rack while PhysX is running leaves
    the tube behind or launches it out of the hole. Pass companions via:

    - ``also_move_prim_paths``: add the **same parent-frame translate delta** (correct
      when ``xformOpOrder`` puts translate before orient, as in the Sinopec scene).
    - ``seat_translate_offsets``: ``{prim: (dx,dy,dz)}`` parent-frame offset from the
      **new** primary translate. Use this to re-seat a tube that already popped out
      during STOP→PLAY contact resolution (measured delta would keep it on the table).

    Teleports use ``/set_entity_state`` while the timeline stays **Playing**. Do not
    PAUSE for these writes: G1D ``ROS2ServicePrim`` is wired to ``OnPlaybackTick``,
    so ``/set_prim_attribute`` disappears while paused and the task queue dies.
    ``pause_during_move`` is kept for experiments but defaults to false.
    """
    if not enabled:
        return
    if isinstance(xyz_offset, tuple):
        if len(xyz_offset) != 3:
            raise ValueError(f"xyz_offset must be a 3-tuple, got: {xyz_offset}")
        off_x, off_y, off_z = xyz_offset
    else:
        off_x, off_y, off_z = float(xyz_offset), float(xyz_offset), 0.0
    cur_x, cur_y, cur_z = get_prim_translate_local(
        object_prim_path,
        timeout=timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    # Uniform on each axis (not Gaussian).
    new_x = cur_x + random.uniform(-off_x, off_x)
    new_y = cur_y + random.uniform(-off_y, off_y)
    new_z = cur_z + random.uniform(-off_z, off_z)
    dx, dy, dz = new_x - cur_x, new_y - cur_y, new_z - cur_z

    seat_map = {
        str(p).strip(): (float(o[0]), float(o[1]), float(o[2]))
        for p, o in (seat_translate_offsets or {}).items()
        if str(p).strip()
    }
    companions = []
    for raw in also_move_prim_paths:
        path = str(raw or "").strip()
        if path and path != object_prim_path and path not in companions:
            companions.append(path)
    for path in seat_map:
        if path != object_prim_path and path not in companions:
            companions.append(path)

    # Parent-frame → world for seat offsets: both Sinopec racks/tube are siblings under
    # /World/room/scene with xformOpOrder [translate, orient, scale], so origin translate
    # lives in the parent frame. World delta = R_parent @ parent_delta.
    parent_path = str(object_prim_path).rsplit("/", 1)[0]
    parent_q = try_get_prim_local_rotation_xyzw(
        parent_path, timeout=min(timeout, PRIM_ATTR_PROBE_TIMEOUT), retries=1, retry_delay=retry_delay
    )
    if parent_q is None:
        parent_q = (0.0, 0.0, 0.0, 1.0)

    def _parent_delta_to_world(delta_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        return rotate_vector_by_quat(delta_xyz, parent_q)

    def _try_set_prim_translate(path: str, xyz: tuple[float, float, float]) -> None:
        try:
            set_prim_translate_local(
                path, xyz, timeout=timeout, retries=retries, retry_delay=retry_delay
            )
        except Exception as exc:
            # Entity teleport below is authoritative for PhysX; USD write is best-effort.
            print(f"[WARN] set_prim_translate_local soft-fail '{path}': {exc}")

    if pause_during_move:
        print(
            "[WARN] pause_during_move=True stops OnPlaybackTick; "
            "/set_prim_attribute will be unavailable on G1D — prefer false"
        )
        set_simulation_state(2, timeout=timeout, retries=retries, retry_delay=retry_delay)

    companion_world_targets: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    try:
        (rack_wx, rack_wy, rack_wz), rack_wq = get_entity_pose_world_service(
            object_prim_path, timeout=timeout, retries=retries, retry_delay=retry_delay
        )
        # Primary world origin shifts by R_parent @ (new_local - old_local).
        wdx, wdy, wdz = _parent_delta_to_world((dx, dy, dz))
        new_rack_w = (rack_wx + wdx, rack_wy + wdy, rack_wz + wdz)

        # PhysX first (works while Playing via simulation_interfaces), then USD mirror.
        set_entity_pose_world_service(
            object_prim_path,
            new_rack_w,
            rack_wq,
            timeout=timeout,
            retries=retries,
            retry_delay=retry_delay,
        )
        _try_set_prim_translate(object_prim_path, (new_x, new_y, new_z))

        for path in companions:
            if path in seat_map:
                ox, oy, oz = seat_map[path]
                target_local = (new_x + ox, new_y + oy, new_z + oz)
                sowx, sowy, sowz = _parent_delta_to_world((ox, oy, oz))
                target_world = (new_rack_w[0] + sowx, new_rack_w[1] + sowy, new_rack_w[2] + sowz)
                try:
                    _pos, cq = get_entity_pose_world_service(
                        path, timeout=timeout, retries=1, retry_delay=retry_delay
                    )
                except Exception:
                    cq = (0.0, 0.0, 0.0, 1.0)
            else:
                try:
                    cx, cy, cz = get_prim_translate_local(
                        path, timeout=timeout, retries=retries, retry_delay=retry_delay
                    )
                    target_local = (cx + dx, cy + dy, cz + dz)
                except Exception:
                    target_local = (new_x + dx, new_y + dy, new_z + dz)
                (cwx, cwy, cwz), cq = get_entity_pose_world_service(
                    path, timeout=timeout, retries=retries, retry_delay=retry_delay
                )
                target_world = (cwx + wdx, cwy + wdy, cwz + wdz)
            companion_world_targets[path] = (target_world, cq)
            set_entity_pose_world_service(
                path,
                target_world,
                cq,
                timeout=timeout,
                retries=retries,
                retry_delay=retry_delay,
            )
            _try_set_prim_translate(path, target_local)
    finally:
        if pause_during_move:
            set_simulation_state(1, timeout=timeout, retries=retries, retry_delay=retry_delay)

    print(
        f"[OK] Randomized object local xyz: "
        f"({cur_x:.3f}, {cur_y:.3f}, {cur_z:.3f}) -> ({new_x:.3f}, {new_y:.3f}, {new_z:.3f})"
    )
    if companions:
        print(
            f"[OK] Kept {len(companions)} companion prim(s) with rack motion: "
            + ", ".join(companions)
        )

    if companions and settle_after_s > 0.0:
        time.sleep(max(0.05, float(settle_after_s) * 0.25))
        for path, (target_world, cq) in companion_world_targets.items():
            try:
                # Re-assert in case contact resolved during the first steps after teleport.
                set_entity_pose_world_service(
                    path,
                    target_world,
                    cq,
                    timeout=timeout,
                    retries=retries,
                    retry_delay=retry_delay,
                )
            except Exception as exc:
                print(f"[WARN] Post-move companion re-seat soft-fail '{path}': {exc}")
        wait_entity_position_stable(
            companions[0],
            max_wait=max(0.5, float(settle_after_s)),
            sample_interval=0.05,
            stable_samples=3,
            position_epsilon=0.003,
            timeout=timeout,
            retries=retries,
            retry_delay=retry_delay,
            sleep_fn=time.sleep,
        )


def wait_entity_pose_readable(
    entity: str,
    *,
    max_wait: float = 10.0,
    poll_interval: float = 0.25,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = 1,
    retry_delay: float = SERVICE_RETRY_DELAY,
    sleep_fn: Callable[[float], None] | None = None,
) -> bool:
    """Poll ``/get_entity_state`` until PhysX views are readable after STOP→PLAY.

    Isaac reset briefly invalidates articulation / RigidBody views; callers that
    immediately query poses or send Nav2 goals see intermittent failures.
    """
    sleep = sleep_fn or time.sleep
    deadline = time.monotonic() + max(0.0, float(max_wait))
    last_err: Exception | None = None
    while time.monotonic() < deadline:
        try:
            get_entity_pose_world_service(
                entity,
                timeout=timeout,
                retries=max(1, int(retries)),
                retry_delay=retry_delay,
            )
            print(f"[OK] Entity pose readable after reset: '{entity}'")
            return True
        except Exception as exc:
            last_err = exc
            sleep(max(0.05, float(poll_interval)))
    print(
        f"[WARN] Entity pose still unreadable after {max_wait:.1f}s: '{entity}'"
        + (f" ({last_err})" if last_err is not None else "")
    )
    return False


def _timeline_stop_then_play(
    *,
    sim_state_reset: int,
    sim_state_playing: int,
    sim_service_timeout: float,
    retries: int,
    retry_delay: float,
    stop_to_play_gap_s: float = 0.4,
) -> None:
    """STOP, then PLAY.

    Wall-clock gap (not sim time): while stopped the sim clock does not advance,
    and ``ArticulationController`` clears its handle on timeline STOP. PLAY too
    soon recreates that handle before PhysX is back.
    """
    set_simulation_state(
        sim_state_reset,
        timeout=sim_service_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    time.sleep(max(0.0, float(stop_to_play_gap_s)))
    set_simulation_state(
        sim_state_playing,
        timeout=sim_service_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )


def reset_simulation_state(
    settle_entity_path: str,
    *,
    sim_state_reset: int = 0,
    sim_state_playing: int = 1,
    sim_service_timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
    post_reset_wait: float = 1.0,
    sleep_fn: Callable[[float], None] | None = None,
    enable_settle_wait: bool = True,
    settle_max_wait: float = 2.0,
    settle_sample_interval: float = 0.05,
    settle_stable_samples: int = 3,
    settle_position_epsilon: float = 0.002,
    also_settle_entity_path: str = "",
    physx_ready_max_wait: float = 8.0,
) -> None:
    """Reset/play Isaac sim，等待 ``post_reset_wait``，再可选对 ``settle_entity_path`` 做位姿稳定检测。

    STOP and PLAY once. A short wall-clock gap sits between them so
    ``ArticulationController`` drops its handle on STOP before PLAY creates a
    new one. Then wait until ``settle_entity_path`` (and optional
    ``also_settle_entity_path``) answer ``/get_entity_state``.

    **不**修改任何物体 prim 的平移；需要随机化时在任务队列**第一个顺序块**使用技能
    ``env.randomize_object_local_xyz``（或自行调用 ``randomize_object_xyz_after_reset``）。
    """
    _timeline_stop_then_play(
        sim_state_reset=sim_state_reset,
        sim_state_playing=sim_state_playing,
        sim_service_timeout=sim_service_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    print("[OK] Simulation reset completed")
    (sleep_fn or time.sleep)(post_reset_wait)

    ready_budget = max(0.0, float(physx_ready_max_wait))
    if settle_entity_path and ready_budget > 0.0:
        wait_entity_pose_readable(
            settle_entity_path,
            max_wait=ready_budget,
            timeout=sim_service_timeout,
            retries=1,
            retry_delay=retry_delay,
            sleep_fn=sleep_fn,
        )
    also = (also_settle_entity_path or "").strip()
    if also and also != settle_entity_path and ready_budget > 0.0:
        wait_entity_pose_readable(
            also,
            max_wait=ready_budget,
            timeout=sim_service_timeout,
            retries=1,
            retry_delay=retry_delay,
            sleep_fn=sleep_fn,
        )

    if enable_settle_wait:
        wait_entity_position_stable(
            settle_entity_path,
            max_wait=settle_max_wait,
            sample_interval=settle_sample_interval,
            stable_samples=settle_stable_samples,
            position_epsilon=settle_position_epsilon,
            timeout=sim_service_timeout,
            retries=retries,
            retry_delay=retry_delay,
            sleep_fn=sleep_fn,
        )
        if also and also != settle_entity_path:
            wait_entity_position_stable(
                also,
                max_wait=settle_max_wait,
                sample_interval=settle_sample_interval,
                stable_samples=settle_stable_samples,
                position_epsilon=settle_position_epsilon,
                timeout=sim_service_timeout,
                retries=retries,
                retry_delay=retry_delay,
                sleep_fn=sleep_fn,
            )


def wait_entity_position_stable(
    entity: str,
    *,
    max_wait: float = 2.0,
    sample_interval: float = 0.05,
    stable_samples: int = 3,
    position_epsilon: float = 0.002,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
    sleep_fn: Callable[[float], None] | None = None,
) -> bool:
    """Wait until entity world position changes remain within tolerance."""
    required_stable = max(1, int(stable_samples))
    interval = max(0.0, float(sample_interval))
    settle_deadline = time.monotonic() + max(0.0, float(max_wait))
    sleep = sleep_fn or time.sleep

    try:
        (prev_x, prev_y, prev_z), _ = get_entity_pose_world_service(
            entity, timeout=timeout, retries=retries, retry_delay=retry_delay
        )
    except Exception as exc:
        print(f"[WARN] Failed to read initial pose for settle check '{entity}': {exc}")
        return False

    stable_count = 0
    while time.monotonic() < settle_deadline:
        sleep(interval)
        try:
            (cur_x, cur_y, cur_z), _ = get_entity_pose_world_service(
                entity, timeout=timeout, retries=retries, retry_delay=retry_delay
            )
        except Exception:
            stable_count = 0
            continue

        dx = cur_x - prev_x
        dy = cur_y - prev_y
        dz = cur_z - prev_z
        dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
        prev_x, prev_y, prev_z = cur_x, cur_y, cur_z
        if dist <= position_epsilon:
            stable_count += 1
            if stable_count >= required_stable:
                print(
                    f"[OK] Entity pose settled: '{entity}', "
                    f"delta<={position_epsilon:.4f}m for {required_stable} samples"
                )
                return True
        else:
            stable_count = 0

    print(
        f"[WARN] Entity pose not fully settled within {max_wait:.2f}s: '{entity}' "
        f"(threshold={position_epsilon:.4f}m, samples={required_stable})"
    )
    return False


def get_entity_pose_world_service(
    entity: str,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    request = GetEntityState.Request()
    request.entity = entity
    result = _call_service_with_retry(
        lambda: _call_service_once(
            GetEntityState,
            "/get_entity_state",
            request,
            timeout=timeout,
        ),
        f"get_entity_pose_world_service('{entity}')",
        retries=retries,
        retry_delay=retry_delay,
    )
    _ensure_sim_result_ok(result.result, what=f"get_entity_state('{entity}')")
    pose = result.state.pose
    return (
        (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def get_object_pose_from_service(
    base_world_pos: tuple[float, float, float],
    base_world_quat: tuple[float, float, float, float],
    object_prim_path: str,
    *,
    include_orientation: bool = False,
    entity_state_timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> Pose:
    base_wx, base_wy, base_wz = base_world_pos
    base_q = base_world_quat
    (obj_wx, obj_wy, obj_wz), obj_q = get_entity_pose_world_service(
        object_prim_path,
        timeout=entity_state_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    rel_world = (obj_wx - base_wx, obj_wy - base_wy, obj_wz - base_wz)
    rel_x, rel_y, rel_z = rotate_vector_by_quat_inverse(rel_world, base_q)

    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    if include_orientation:
        q_obj_in_base = quat_multiply(
            quat_conjugate(quat_normalize(base_q)),
            quat_normalize(obj_q),
        )
        qx, qy, qz, qw = quat_normalize(q_obj_in_base)

    pose = Pose()
    pose.position.x = rel_x
    pose.position.y = rel_y
    pose.position.z = rel_z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose
