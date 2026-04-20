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
    以便 sim reset 完成后再对**显式** ``object_prim_path`` 做扰动。

    调用 :func:`robot_action_composer.isaac_sim.randomize_object_xyz_after_reset`：
    各轴独立均匀分布 ``[-w_i, +w_i]``，``w_i`` 为 ``xyz_offset`` 对应分量。

    params:
        object_prim_path: 必填，要扰动的物体 / 刚体 prim 全路径（勿默认用 carry 的 source）。
        xyz_offset: 各轴半幅（米），默认 ``[0.04, 0.04, 0.0]``。
        enabled: 默认 ``true``；``false`` 时跳过（便于 YAML 开关）。
    """
    raw = params.get("object_prim_path")
    if raw is None or (isinstance(raw, str) and not str(raw).strip()):
        raise ValueError(
            "env.randomize_object_local_xyz requires non-empty object_prim_path "
            "(which prim to jitter; not inferred from dual_arm.carry)",
        )
    path = str(raw).strip()
    enabled = bool(params.get("enabled", True))
    off = _param_vec3(params, "xyz_offset", (0.04, 0.04, 0.0))

    randomize_object_xyz_after_reset(path, enabled=enabled, xyz_offset=off)

    fid = str(ctx.frame_id).strip() or "base_link"
    return [], ExecutionMeta(
        send_mode=SendMode.UNSTAMPED,
        frame_id=fid,
        warn_prefix="TaskQ env.randomize_object_local_xyz",
    )


def register_env_skills() -> None:
    register_skill("env.randomize_object_local_xyz", skill_randomize_object_local_xyz)


register_env_skills()
