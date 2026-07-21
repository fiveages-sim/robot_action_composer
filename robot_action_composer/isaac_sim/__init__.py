#!/usr/bin/env python3
"""Common helpers for Isaac ROS2 simulation control."""

from __future__ import annotations

import random
import threading
import time
from typing import Callable, TypeVar

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from isaac_ros2_messages.srv import GetPrimAttribute, SetPrimAttribute
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat_inverse,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from simulation_interfaces.srv import GetEntityState, SetSimulationState

SERVICE_CALL_TIMEOUT = 8.0
SERVICE_CALL_RETRIES = 3
SERVICE_RETRY_DELAY = 0.2
# Probe USD attrs via /get_prim_attribute: keep short — missing attrs are common and
# should not burn retries×8s (that looked like a task-queue hang).
PRIM_ATTR_PROBE_TIMEOUT = 2.0
PRIM_ATTR_PROBE_RETRIES = 1
T = TypeVar("T")

_service_node: Node | None = None
_service_lock = threading.Lock()


def _ensure_service_node() -> Node:
    """Shared Isaac service client.

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
    node = _ensure_service_node()
    client = node.create_client(service_type, service_name)
    try:
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
    finally:
        try:
            node.destroy_client(client)
        except Exception:
            pass


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
    if result.result.error_message:
        raise RuntimeError(
            f"set_simulation_state({state}) unsuccessful: {result.result.error_message}"
        )


def _parse_prim_attr_vec(value: str, *, expected_len: int, path: str, attribute: str) -> np.ndarray:
    vec = np.fromstring(value.strip().strip("[]"), sep=",")
    if vec.shape[0] != expected_len:
        raise RuntimeError(
            f"Invalid {attribute} for '{path}': expected {expected_len} values, got {value!r}"
        )
    return vec


def try_get_prim_attribute(
    path: str,
    attribute: str,
    *,
    timeout: float = PRIM_ATTR_PROBE_TIMEOUT,
    retries: int = PRIM_ATTR_PROBE_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> str | None:
    """Return attribute value string, or ``None`` if missing / service unsuccessful."""
    request = GetPrimAttribute.Request()
    request.path = path
    request.attribute = attribute
    try:
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
    except Exception as exc:
        print(
            f"[Isaac] WARN: get_prim_attribute failed for {path!r} "
            f"{attribute!r}: {exc}"
        )
        return None
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
    """Local translate, or ``None`` if attribute missing (treat as identity translate)."""
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
    """Local ``xformOp:orient`` as xyzw, or ``None`` if missing (treat as identity).

    Omniverse authors orient as **wxyz**; return value is ROS-style **xyzw**.
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
    

def randomize_object_xyz_after_reset(
    object_prim_path: str,
    enabled: bool = True,
    xyz_offset: tuple[float, float, float] | float = 0.04,
    timeout: float = SERVICE_CALL_TIMEOUT,
    retries: int = SERVICE_CALL_RETRIES,
    retry_delay: float = SERVICE_RETRY_DELAY,
) -> None:
    """Apply a small random **local** translation to the object prim after reset.

    Each axis uses an **independent uniform** draw in ``[-w, +w]`` added to the
    current local translation, where ``w`` is the corresponding component of
    ``xyz_offset`` (or the scalar for x/y and 0 for z when ``xyz_offset`` is a float).
    This is **not** a Gaussian / normal distribution.
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
    set_prim_translate_local(
        object_prim_path,
        (new_x, new_y, new_z),
        timeout=timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    print(
        f"[OK] Randomized object local xyz: "
        f"({cur_x:.3f}, {cur_y:.3f}, {cur_z:.3f}) -> ({new_x:.3f}, {new_y:.3f}, {new_z:.3f})"
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
) -> None:
    """Reset/play Isaac sim，等待 ``post_reset_wait``，再可选对 ``settle_entity_path`` 做位姿稳定检测。

    **不**修改任何物体 prim 的平移；需要随机化时在任务队列**第一个顺序块**使用技能
    ``env.randomize_object_local_xyz``（或自行调用 ``randomize_object_xyz_after_reset``）。
    """
    set_simulation_state(
        sim_state_reset,
        timeout=sim_service_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    set_simulation_state(
        sim_state_playing,
        timeout=sim_service_timeout,
        retries=retries,
        retry_delay=retry_delay,
    )
    print("[OK] Simulation reset completed")
    (sleep_fn or time.sleep)(post_reset_wait)
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
    if result.result.error_message:
        raise RuntimeError(
            f"get_entity_state unsuccessful for '{entity}': {result.result.error_message}"
        )
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
