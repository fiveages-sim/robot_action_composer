#!/usr/bin/env python3
"""Bimanual carry task config and Cartesian sequence for task-queue execution."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Pose

from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    euler_rpy_to_quat_xyzw,
    quat_multiply,
    quat_normalize,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    StageTarget,
    build_bimanual_carry_sequence,
)
from robot_action_composer.motion_generation.tasks.object_orientation import (  # pyright: ignore[reportMissingImports]
    compose_aligned_ee_orientation,
    filter_object_pose_orientation,
    object_orientation_xyzw,
)


@dataclass(frozen=True)
class BimanualCarryTaskConfig:
    """双臂对称搬运几何（由结构化 ``dual_arm.carry`` 叠层得到）。"""

    object_prim_path: str
    # 沿 span 轴单侧半宽（米），默认沿 Y；与 object_position_offset[1] 相加后左右对称展开
    object_dual_arm_half_span_y: float
    left_base_orientation: tuple[float, float, float, float]
    right_base_orientation: tuple[float, float, float, float]
    # 可选：与 ``dual_arm.bimanual_align`` 的 ``orientation_delta_rpy`` 相同语义——
    # ``[roll, pitch, yaw]`` 弧度，左乘 q_delta * q_ee，左右臂共用同一增量（在 ``carry_*_orientation`` 所在系，通常即 motion 系）
    orientation_delta_rpy: tuple[float, float, float] | None = None
    # object_span_axis=y 时优先使用；不写则回退到 orientation_delta_rpy，保持老配置兼容。
    lateral_orientation_delta_rpy: tuple[float, float, float] | None = None
    # object_span_axis=x 时优先使用；用于顶部竖直下抓的手腕朝向。
    vertical_orientation_delta_rpy: tuple[float, float, float] | None = None
    # 预闭合前：在**末端工具系**下相对 Forward 远点的平移（与 ``ee_lift_offset`` 的 tool 语义一致）；不写 / 全 0 则省略 Approach
    carry_prepare_offset: tuple[float, float, float] | None = None
    # y 横抓时：Forward 阶段沿 span 轴额外张开的余量（米）。
    arm_merge_distance_y: float = 0.0
    # object_span_axis=x 时，Forward 位姿高于 CloseIn 的竖直预接近高度；不写则复用 arm_merge_distance_y。
    vertical_approach_clearance_z: float | None = None
    # 物体固连系下相对原点的平移：``(x, 与半宽合并的 y 分量, z)``；中线为 ``(x,0,z)``，左右沿 +Y，经 ``object_position.orientation`` 旋到 motion 系
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # true 时读取并使用 Isaac 物体实时姿态；false 时保持历史行为，仅使用物体实时位置。
    use_object_orientation: bool = False
    # ``full``：完整使用物体四元数；``tilt``：只保留 roll/pitch，忽略 yaw；``none``：读取后按单位姿态处理。
    object_orientation_mode: str = "full"
    # ``motion``：半宽沿 motion_frame_id 的 object_span_axis；``object``：半宽沿物体实时局部 object_span_axis。
    object_span_frame: str = "motion"
    # ``y``：横向左右夹抓（默认）；``x``：前后/竖向夹抓；``z``：上下方向（通常不用于双臂夹抓）。
    object_span_axis: str = "y"
    # ``motion``：末端姿态使用 base_orientation；``object``：末端姿态随物体实时姿态一起旋转。
    ee_orientation_frame: str = "motion"
    # ``auto``：y 横抓沿用 tool 位移；x 竖直下抓用 motion 位移，保证 lift 的 Z 是 arm_base/world 上方。
    carry_linear_displacement_frame: str = "auto"
    # 不写 / ``null``：不生成该段；另一段仍按 (0,0,0) 参与合成其终点。位移统一按工具系解释。
    ee_pregrasp_lift_offset: tuple[float, float, float] | None = None
    ee_lift_offset: tuple[float, float, float] | None = None
    ee_retreat_offset: tuple[float, float, float] | None = None
    # 可选：双臂 movel 时长，执行前写入 ``arm_controller.movel_duration``（类比 ``body_movej_duration``）
    arm_movel_duration: float | None = None
    # 可选：笛卡尔目标/header 坐标系；不写则与队列 ``ctx.frame_id``（多为 base_link）一致。例 ``arm_base``，与 place 一致时需 TF
    motion_frame_id: str | None = None
    tf_lookup_timeout: float | None = None


def format_bimanual_carry_task_cfg_summary(
    scene: str, task_cfg: BimanualCarryTaskConfig,
) -> str:
    return (
        f"[Scene] {scene} -> {task_cfg.object_prim_path}, "
        f"object_dual_arm_half_span_y={task_cfg.object_dual_arm_half_span_y}, "
        f"carry_prepare_offset={task_cfg.carry_prepare_offset}, "
        f"ee_pregrasp_lift_offset={task_cfg.ee_pregrasp_lift_offset!r}, "
        f"ee_lift_offset={task_cfg.ee_lift_offset!r}, ee_retreat_offset={task_cfg.ee_retreat_offset!r}, "
        f"orientation_delta_rpy={task_cfg.orientation_delta_rpy!r}, "
        f"arm_movel_duration={task_cfg.arm_movel_duration!r}, motion_frame_id={task_cfg.motion_frame_id!r}"
    )


def _apply_carry_orientation_delta_rpy(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
    delta_rpy: tuple[float, float, float] | None,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
    """RPY→四元数用 :func:`ros2_robot_interface.utils.quat_pose.euler_rpy_to_quat_xyzw`；左乘与归一化同 :func:`~ros2_robot_interface.utils.quat_pose.quat_multiply` / ``quat_normalize``。"""
    if delta_rpy is None:
        return left_xyzw, right_xyzw
    dr, dp, dyaw = float(delta_rpy[0]), float(delta_rpy[1]), float(delta_rpy[2])
    if abs(dr) < 1e-12 and abs(dp) < 1e-12 and abs(dyaw) < 1e-12:
        return left_xyzw, right_xyzw
    qd = euler_rpy_to_quat_xyzw(dr, dp, dyaw)
    return (
        quat_normalize(quat_multiply(qd, left_xyzw)),
        quat_normalize(quat_multiply(qd, right_xyzw)),
    )


def _carry_axis_key(object_span_axis: str) -> str:
    raw = str(object_span_axis or "y").strip().lower().lstrip("+-")
    if raw in ("x", "forward", "front_back", "front-back", "longitudinal", "top_down", "top-down", "down"):
        return "x"
    if raw in ("y", "side", "sideways", "left_right", "left-right", "lateral"):
        return "y"
    if raw in ("z", "up", "vertical"):
        return "z"
    raise ValueError(f"unsupported object_span_axis {object_span_axis!r}; expected 'x', 'y', or 'z'")


def _select_carry_orientation_delta_rpy(
    task_cfg: BimanualCarryTaskConfig,
) -> tuple[float, float, float] | None:
    axis = _carry_axis_key(task_cfg.object_span_axis)
    if axis == "x" and task_cfg.vertical_orientation_delta_rpy is not None:
        return task_cfg.vertical_orientation_delta_rpy
    if axis == "y" and task_cfg.lateral_orientation_delta_rpy is not None:
        return task_cfg.lateral_orientation_delta_rpy
    return task_cfg.orientation_delta_rpy


def _select_carry_linear_displacement_frame(task_cfg: BimanualCarryTaskConfig) -> str:
    raw = str(task_cfg.carry_linear_displacement_frame or "auto").strip().lower()
    if raw not in ("", "auto"):
        return raw
    axis = _carry_axis_key(task_cfg.object_span_axis)
    return "motion" if axis == "x" else "tool"


def _object_orientation_xyzw(p: Pose) -> tuple[float, float, float, float]:
    return object_orientation_xyzw(p)


def _filter_object_orientation_for_carry(
    object_position: Pose,
    mode: str,
) -> Pose:
    return filter_object_pose_orientation(object_position, mode)


def _apply_object_orientation_to_ee(
    object_position: Pose,
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
    ee_orientation_frame: str,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
    # carry 历史语义：ee_orientation_frame=object 时用完整物体姿态（mode 已在上层过滤）。
    return (
        compose_aligned_ee_orientation(
            left_xyzw,
            object_position,
            ee_orientation_frame=ee_orientation_frame,
            object_orientation_mode="full",
        ),
        compose_aligned_ee_orientation(
            right_xyzw,
            object_position,
            ee_orientation_frame=ee_orientation_frame,
            object_orientation_mode="full",
        ),
    )


def build_bimanual_carry_record_sequence(
    *,
    carry_task_cfg: BimanualCarryTaskConfig,
    object_position: Pose,
    gripper_open: float,
    gripper_closed: float,
    output_frame_id: str | None = None,
    carry_lift_motion_delta: tuple[float, float, float] | None = None,
    carry_retreat_motion_delta: tuple[float, float, float] | None = None,
) -> list[StageTarget]:
    """Build the full bimanual carry sequence as StageTarget list.

    ``carry_lift_motion_delta`` / ``carry_retreat_motion_delta``：调用方预计算的位移增量（兼容入口）。
    为空时使用 ``carry_task_cfg`` 内的 ``ee_lift_offset`` / ``ee_retreat_offset``。
    """
    object_position_for_geometry = _filter_object_orientation_for_carry(
        object_position,
        carry_task_cfg.object_orientation_mode,
    )
    lo, ro = _apply_carry_orientation_delta_rpy(
        carry_task_cfg.left_base_orientation,
        carry_task_cfg.right_base_orientation,
        _select_carry_orientation_delta_rpy(carry_task_cfg),
    )
    lo, ro = _apply_object_orientation_to_ee(
        object_position_for_geometry,
        lo,
        ro,
        carry_task_cfg.ee_orientation_frame,
    )
    return build_bimanual_carry_sequence(
        object_position=object_position_for_geometry,
        object_dual_arm_half_span_y=carry_task_cfg.object_dual_arm_half_span_y,
        carry_prepare_offset=carry_task_cfg.carry_prepare_offset,
        arm_merge_distance_y=carry_task_cfg.arm_merge_distance_y,
        vertical_approach_clearance_z=carry_task_cfg.vertical_approach_clearance_z,
        object_position_offset=carry_task_cfg.object_position_offset,
        object_span_frame=carry_task_cfg.object_span_frame,
        object_span_axis=carry_task_cfg.object_span_axis,
        carry_left_orientation=lo,
        carry_right_orientation=ro,
        ee_pregrasp_lift_offset=carry_task_cfg.ee_pregrasp_lift_offset,
        ee_lift_offset=carry_task_cfg.ee_lift_offset,
        ee_retreat_offset=carry_task_cfg.ee_retreat_offset,
        carry_linear_displacement_frame=_select_carry_linear_displacement_frame(carry_task_cfg),
        carry_lift_motion_delta=carry_lift_motion_delta,
        carry_retreat_motion_delta=carry_retreat_motion_delta,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        output_frame_id=output_frame_id,
    )
