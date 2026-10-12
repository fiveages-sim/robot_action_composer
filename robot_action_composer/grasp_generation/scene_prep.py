"""Scene prep / reset for grasp datagen.

``/set_prim_attribute`` only works while **PLAYING**, but writing xforms /
visibility / rigidBody while PLAYING invalidates PhysX SimulationView
(``getDofPositionTargets`` / ``getTransforms`` spam).

Required pattern::

    STOP → PLAY                 # load stage (env) height
    write xforms while PLAY     # near XY + yaw; keep live Z (not yaml home Z)
    STOP → PLAY                 # recreate SimulationView after writes
    settle

Skip the final STOP→PLAY and the robot arm freezes even though HUG still runs.
"""

from __future__ import annotations

import json
import math
import random
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

from .config import GraspGenerationConfig
from .objects import (
    GraspObjectSpec,
    get_grasp_object,
    iter_grasp_objects,
    scene_prep_vec2,
)


@dataclass
class ScenePrepResult:
    target_key: str
    target_translate: tuple[float, float, float]
    target_yaw_deg: float
    isolated: list[str]
    settle_s: float
    mode: str
    did_stop_play: bool = True


def _prep_cfg(cfg: GraspGenerationConfig) -> Mapping[str, Any]:
    raw = cfg.scene_prep or {}
    return raw if isinstance(raw, Mapping) else {}


def ensure_sim_playing() -> None:
    from simulation_interfaces.msg import SimulationState

    from robot_action_composer.isaac_sim import set_simulation_state

    set_simulation_state(int(SimulationState.STATE_PLAYING))
    print("[scene_prep] ensure sim PLAYING")


def hard_reset_physx(*, settle_s: float = 1.5) -> None:
    from simulation_interfaces.msg import SimulationState

    from robot_action_composer.isaac_sim import set_simulation_state

    print("[scene_prep] HARD reset PhysX: STOPPED → PLAYING")
    set_simulation_state(int(SimulationState.STATE_STOPPED))
    time.sleep(0.3)
    set_simulation_state(int(SimulationState.STATE_PLAYING))
    if settle_s > 0:
        time.sleep(float(settle_s))
    print("[scene_prep] HARD reset done — Go home before Auto")


def refresh_physx_view(*, settle_s: float = 0.5) -> None:
    ensure_sim_playing()
    if settle_s > 0:
        time.sleep(float(settle_s))


def _wait_set_prim_service(timeout_s: float = 12.0) -> None:
    from isaac_ros2_messages.srv import SetPrimAttribute

    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        _get_service_client,
        _service_lock,
        _wait_for_service_wall,
    )

    node, client = _get_service_client(SetPrimAttribute, "/set_prim_attribute")
    with _service_lock:
        ok = _wait_for_service_wall(client, node, float(timeout_s))
    if not ok:
        raise RuntimeError(
            f"/set_prim_attribute unavailable within {timeout_s:.1f}s — "
            "is Isaac playing and isaac_ros2 bridge up?"
        )


def run_prim_batch(
    writes_fn: Callable[[], None],
    *,
    settle_s: float = 1.0,
    leave_stopped: bool = False,
    label: str = "prim batch",
    heal_first: bool = True,
) -> None:
    """STOP→PLAY → write while PLAYING → STOP→PLAY (recreate PhysX view).

    Final STOP→PLAY is mandatory after prim writes; otherwise Isaac spam
    ``Simulation view object is invalidated`` and the arm cannot move.
    Use ``keep_live_z`` in scene_prep so the recreate does not re-lift props.
    """
    from simulation_interfaces.msg import SimulationState

    from robot_action_composer.isaac_sim import set_simulation_state

    if heal_first:
        print(f"[scene_prep] STOP→PLAY before {label} (load stage height)")
        set_simulation_state(int(SimulationState.STATE_STOPPED))
        time.sleep(0.3)
        set_simulation_state(int(SimulationState.STATE_PLAYING))
        time.sleep(0.45)
    else:
        print(f"[scene_prep] PLAYING for {label}")
        set_simulation_state(int(SimulationState.STATE_PLAYING))
        time.sleep(0.35)

    _wait_set_prim_service(timeout_s=12.0)
    writes_fn()
    print(f"[scene_prep] writes done for {label}")

    # Recreate PhysX SimulationView after xform / visibility / RB edits.
    print(f"[scene_prep] STOP→PLAY after {label} (recreate PhysX view)")
    set_simulation_state(int(SimulationState.STATE_STOPPED))
    time.sleep(0.35)
    if leave_stopped:
        print(f"[scene_prep] left STOPPED after {label}")
        return
    set_simulation_state(int(SimulationState.STATE_PLAYING))
    print(f"[scene_prep] PLAYING — PhysX view recreated ({label})")
    if settle_s > 0:
        time.sleep(float(settle_s))


def run_stopped_batch(
    writes_fn: Callable[[], None],
    *,
    settle_s: float = 1.0,
    leave_stopped: bool = False,
    label: str = "xform batch",
) -> None:
    run_prim_batch(
        writes_fn,
        settle_s=settle_s,
        leave_stopped=leave_stopped,
        label=label,
    )


def _set_prim_json_attr(prim_path: str, attribute: str, value: Any) -> None:
    from isaac_ros2_messages.srv import SetPrimAttribute

    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        SERVICE_CALL_RETRIES,
        SERVICE_CALL_TIMEOUT,
        SERVICE_RETRY_DELAY,
        _call_service_once,
        _call_service_with_retry,
    )

    request = SetPrimAttribute.Request()
    request.path = prim_path
    request.attribute = attribute
    request.value = json.dumps(value)
    result = _call_service_with_retry(
        lambda: _call_service_once(
            SetPrimAttribute,
            "/set_prim_attribute",
            request,
            timeout=SERVICE_CALL_TIMEOUT,
        ),
        f"set_prim_attr('{prim_path}', {attribute}={value!r})",
        retries=SERVICE_CALL_RETRIES,
        retry_delay=SERVICE_RETRY_DELAY,
    )
    if not getattr(result, "success", False):
        raise RuntimeError(
            f"set_prim_attribute unsuccessful for '{prim_path}' "
            f"{attribute}={value!r}: {getattr(result, 'message', None) or 'unknown'}"
        )


def _try_set_visibility(prim_path: str, visible: bool) -> None:
    try:
        _set_prim_json_attr(
            prim_path, "visibility", "inherited" if visible else "invisible"
        )
    except Exception as exc:
        print(f"[scene_prep] WARN visibility {prim_path}: {exc}")


def _try_set_rigid_body(prim_path: str, enabled: bool) -> None:
    try:
        _set_prim_json_attr(prim_path, "physics:rigidBodyEnabled", bool(enabled))
    except Exception as exc:
        print(f"[scene_prep] WARN rigidBody {prim_path}: {exc}")


def _home_translate(spec: GraspObjectSpec) -> tuple[float, float, float]:
    if spec.home_translate is not None:
        return spec.home_translate
    from robot_action_composer.isaac_sim import get_prim_translate_local

    return get_prim_translate_local(spec.placement_prim)


def _live_translate(spec: GraspObjectSpec) -> tuple[float, float, float]:
    """Stage local translate (env height), fallback to yaml home."""
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        try_get_prim_translate_local,
    )

    live = try_get_prim_translate_local(spec.placement_prim)
    if live is not None:
        return live
    return _home_translate(spec)


def _home_or_live_orient_wxyz(
    spec: GraspObjectSpec,
) -> tuple[float, float, float, float]:
    if spec.home_orient_wxyz is not None:
        return spec.home_orient_wxyz
    from robot_action_composer.isaac_sim import try_get_prim_orient_local_xyzw

    q_xyzw = try_get_prim_orient_local_xyzw(spec.placement_prim)
    if q_xyzw is None:
        return (1.0, 0.0, 0.0, 0.0)
    x, y, z, w = q_xyzw
    return (w, x, y, z)


def _quat_wxyz_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _yaw_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(float(yaw_deg)) * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _apply_home_pose(spec: GraspObjectSpec) -> tuple[float, float, float]:
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        set_prim_orientation_local,
        set_prim_translate_local,
    )

    xyz = _home_translate(spec)
    set_prim_translate_local(spec.placement_prim, xyz)
    if spec.home_orient_wxyz is not None:
        set_prim_orientation_local(spec.placement_prim, spec.home_orient_wxyz)
    return xyz


def _activate_spec(spec: GraspObjectSpec, *, active: bool) -> None:
    _try_set_visibility(spec.placement_prim, active)
    rb = spec.pose_prim or spec.placement_prim
    _try_set_rigid_body(rb, active)
    if rb != spec.placement_prim:
        _try_set_rigid_body(spec.placement_prim, active)


def reset_scene_objects(
    cfg: GraspGenerationConfig,
    *,
    leave_stopped: bool = True,
    settle_s: Optional[float] = None,
) -> list[str]:
    """PLAY → all objects to yaml ``home_*`` + visible → optional STOP."""
    prep = _prep_cfg(cfg)
    wait = float(settle_s if settle_s is not None else prep.get("settle_s", 1.0))
    restored: list[str] = []
    errors: list[str] = []

    def _writes() -> None:
        for spec in iter_grasp_objects(cfg):
            try:
                xyz = _apply_home_pose(spec)
                _activate_spec(spec, active=True)
                restored.append(spec.key)
                print(f"[scene_prep] reset {spec.key} → home {xyz} (visible)")
            except Exception as exc:
                errors.append(f"{spec.key}: {exc}")
                print(f"[scene_prep] ERROR reset {spec.key}: {exc}")

    run_prim_batch(
        _writes,
        settle_s=wait,
        leave_stopped=leave_stopped,
        label="reset objects",
        heal_first=True,
    )
    if errors and not restored:
        raise RuntimeError("reset_scene_objects failed: " + "; ".join(errors))
    if errors:
        print(f"[scene_prep] WARN partial reset: {errors}")
    return restored


def prepare_scene_for_grasp(
    cfg: GraspGenerationConfig,
    target_key: str,
    *,
    settle_s: Optional[float] = None,
    rng: Optional[random.Random] = None,
) -> ScenePrepResult:
    """Heal → exile distractors → near-spawn target (XY + yaw, keep live Z)."""
    from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
        set_prim_orientation_local,
        set_prim_translate_local,
    )

    prep = _prep_cfg(cfg)
    target = get_grasp_object(cfg, target_key)
    others = [s for s in iter_grasp_objects(cfg) if s.key != target.key]
    rng = rng or random.Random()
    wait = float(settle_s if settle_s is not None else prep.get("settle_s", 1.5))
    mode = str(prep.get("isolate_mode", "exile") or "exile").strip().lower()
    if mode not in ("exile", "hide"):
        print(f"[scene_prep] WARN isolate_mode={mode!r} → exile")
        mode = "exile"

    keep_live_z = bool(prep.get("keep_live_z", True))
    near_yaw = float(prep.get("near_yaw_deg", 30.0) or 0.0)
    xy_min = scene_prep_vec2(
        prep.get("near_xy_min"), (-12.0, -32.0), label="scene_prep.near_xy_min"
    )
    xy_max = scene_prep_vec2(
        prep.get("near_xy_max"), (8.0, -10.0), label="scene_prep.near_xy_max"
    )
    exile_base = scene_prep_vec2(
        prep.get("exile_xy_base"), (120.0, 120.0), label="scene_prep.exile_xy_base"
    )
    spacing = float(prep.get("exile_spacing", 30.0))
    x0, x1 = sorted((xy_min[0], xy_max[0]))
    y0, y1 = sorted((xy_min[1], xy_max[1]))

    isolated: list[str] = []
    target_xyz = _home_translate(target)
    target_yaw_deg = 0.0
    errors: list[str] = []

    def _writes() -> None:
        nonlocal target_xyz, target_yaw_deg
        for i, spec in enumerate(others):
            try:
                live = _live_translate(spec)
                z = live[2] if keep_live_z else _home_translate(spec)[2]
                if mode == "exile":
                    xyz = (
                        exile_base[0] + (i % 4) * spacing,
                        exile_base[1] + (i // 4) * spacing,
                        z,
                    )
                    set_prim_translate_local(spec.placement_prim, xyz)
                    print(f"[scene_prep] exile {spec.key} → {xyz} (keep_z={keep_live_z})")
                else:
                    # Hide only: park at live XY/Z (do not lift to yaml home Z).
                    set_prim_translate_local(spec.placement_prim, live)
                    print(f"[scene_prep] park {spec.key} at live {live}")
                _activate_spec(spec, active=False)
                isolated.append(spec.key)
            except Exception as exc:
                errors.append(f"{spec.key}: {exc}")
                print(f"[scene_prep] ERROR isolate {spec.key}: {exc}")

        try:
            live_t = _live_translate(target)
            z = live_t[2] if keep_live_z else _home_translate(target)[2]
            if bool(prep.get("near_spawn", True)):
                target_xyz = (rng.uniform(x0, x1), rng.uniform(y0, y1), z)
            else:
                target_xyz = (live_t[0], live_t[1], z)
            set_prim_translate_local(target.placement_prim, target_xyz)

            base_q = _home_or_live_orient_wxyz(target)
            if near_yaw > 1e-6 and bool(prep.get("near_spawn", True)):
                target_yaw_deg = rng.uniform(-near_yaw, near_yaw)
                q = _quat_wxyz_mul(_yaw_wxyz(target_yaw_deg), base_q)
            else:
                target_yaw_deg = 0.0
                q = base_q
            set_prim_orientation_local(target.placement_prim, q)
            _activate_spec(target, active=True)
            print(
                f"[scene_prep] target {target.key} → xyz={target_xyz} "
                f"yaw={target_yaw_deg:+.1f}deg (keep_z={keep_live_z})"
            )
        except Exception as exc:
            errors.append(f"{target.key}: {exc}")
            print(f"[scene_prep] ERROR target {target.key}: {exc}")
            raise

    run_prim_batch(
        _writes,
        settle_s=wait,
        leave_stopped=False,
        label="scene prep",
        heal_first=True,
    )

    if errors and not isolated:
        raise RuntimeError("scene_prep failed: " + "; ".join(errors))
    if errors:
        print(f"[scene_prep] WARN partial isolate: {errors}")

    return ScenePrepResult(
        target_key=target.key,
        target_translate=target_xyz,
        target_yaw_deg=target_yaw_deg,
        isolated=isolated,
        settle_s=wait,
        mode=mode,
        did_stop_play=True,
    )


def restore_scene(cfg: GraspGenerationConfig) -> list[str]:
    return reset_scene_objects(cfg, leave_stopped=False)
