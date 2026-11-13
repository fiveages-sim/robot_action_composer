"""仿真环境类技能（``env.*``）：与机械臂笛卡尔段解耦。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from robot_action_composer.isaac_sim import randomize_object_xyz_after_reset  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    SendMode,
    StageTarget,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.object_binding import resolve_object_prim_path
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.types import ExecutionMeta


def _param_vec3(
    params: Mapping[str, Any],
    key: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    v = params.get(key)
    if v is None:
        return default
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"{key} must be a length-3 list [x, y, z]")
    return (float(v[0]), float(v[1]), float(v[2]))


def skill_randomize_object_local_xyz(
    ctx: QueueRuntimeContext, params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """对指定 Isaac prim 施加**局部**平移随机抖动（与 ``dual_arm.carry`` 几何无关）。

    典型用法：在 ``reset_env: true`` 时，把本技能作为 ``task_queue`` **第一个**顺序块（在 ``parallel`` 之前），
    以便 sim reset 完成后再对目标 prim 做扰动。

    调用 :func:`robot_action_composer.isaac_sim.randomize_object_xyz_after_reset`：
    各轴独立均匀分布 ``[-w_i, +w_i]``，``w_i`` 为 ``xyz_offset`` 对应分量。

    params:
        object_prim_path: 可选，要扰动的物体 / 刚体 prim 全路径（显式优先）。
        object_key: 可选，从任务 ``objects`` / ``.meta/objects`` 解析 prim；
            也可回退 ``active_object``。
        xyz_offset: 各轴半幅（米），默认 ``[0.04, 0.04, 0.0]``。
        enabled: 默认 ``true``；``false`` 时跳过（便于 YAML 开关）。
        also_object_keys / also_object_prim_paths: 随主物体一起平移的伴生 prim
            （例如架上的动态试管）。仅同 delta 时用；若 reset 已把伴生弹飞，请改用
            ``seat_object_key`` + ``seat_translate_offset``。
        seat_object_key / seat_object_prim_path: 需要按相对主物体的父系偏移「坐回」的伴生。
        seat_translate_offset: 父系平移偏移 ``[dx,dy,dz]``（与 USD ``xformOp:translate``
            同空间；Sinopec 管架/试管为 scene 下兄弟节点）。
        pause_during_move: 默认 ``false``。G1D 上 PAUSE 会停掉 OnPlaybackTick，
            ``/set_prim_attribute`` 随之消失；传送改走 Playing 下的 ``/set_entity_state``。
        settle_after_s: 伴生写入后等待稳定的墙钟秒数，默认 ``0.8``。
        force_kinematic: 默认 ``false``。对试管架等夹具设 ``true``，避免 Playing
            中途 XY 传送后与桌面接触把架子掀翻。
        snap_authored_orientation: 默认 ``true``。用 USD ``xformOp:orient`` 重建
            世界姿态，避免沿用已翻倒的 entity pose。
    """
    objects = getattr(ctx, "objects", None)
    path = resolve_object_prim_path(
        params,
        objects=objects,
        prim_param="object_prim_path",
        key_params="object_key",
        active_object=getattr(ctx, "active_object", None),
        use_active_object=True,
        required=True,
        label="env.randomize_object_local_xyz",
    )
    assert path  # required=True
    enabled = bool(params.get("enabled", True))
    off = _param_vec3(params, "xyz_offset", (0.04, 0.04, 0.0))

    also_paths: list[str] = []
    raw_also_paths = params.get("also_object_prim_paths")
    if isinstance(raw_also_paths, (list, tuple)):
        for item in raw_also_paths:
            p = str(item or "").strip()
            if p:
                also_paths.append(p)
    raw_also_keys = params.get("also_object_keys")
    if isinstance(raw_also_keys, (list, tuple)):
        for key in raw_also_keys:
            p = resolve_object_prim_path(
                {"object_key": key},
                objects=objects,
                required=True,
                label=f"env.randomize_object_local_xyz also_object_key={key!r}",
            )
            assert p
            also_paths.append(p)

    seat_offsets: dict[str, tuple[float, float, float]] = {}
    seat_path = resolve_object_prim_path(
        params,
        objects=objects,
        prim_param="seat_object_prim_path",
        key_params="seat_object_key",
        use_active_object=False,
        required=False,
        label="env.randomize_object_local_xyz seat",
    )
    if seat_path:
        seat_offsets[seat_path] = _param_vec3(
            params, "seat_translate_offset", (0.0, 0.0, 0.0)
        )
        if seat_path not in also_paths:
            also_paths.append(seat_path)

    pause_during_move = bool(params.get("pause_during_move", False))
    settle_after_s = float(params.get("settle_after_s", 0.8))
    force_kinematic = bool(params.get("force_kinematic", False))
    snap_authored_orientation = bool(params.get("snap_authored_orientation", True))

    randomize_object_xyz_after_reset(
        path,
        enabled=enabled,
        xyz_offset=off,
        also_move_prim_paths=tuple(also_paths),
        seat_translate_offsets=seat_offsets or None,
        pause_during_move=pause_during_move,
        settle_after_s=settle_after_s,
        force_kinematic=force_kinematic,
        snap_authored_orientation=snap_authored_orientation,
    )

    fid = str(ctx.frame_id).strip() or "base_link"
    return [], ExecutionMeta(
        send_mode=SendMode.UNSTAMPED,
        frame_id=fid,
        warn_prefix="TaskQ env.randomize_object_local_xyz",
    )


def register_env_skills() -> None:
    register_skill("env.randomize_object_local_xyz", skill_randomize_object_local_xyz)


register_env_skills()
