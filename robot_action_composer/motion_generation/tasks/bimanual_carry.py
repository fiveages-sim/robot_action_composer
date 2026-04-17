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
    carry_prepare_offset_is_active,
)

# ``build_bimanual_carry_sequence``：可选 Approach + Forward + CloseIn | Grasp | 可选 Lift / Retreat
CARRY_APPROACH_STAGE_COUNT_MAX = 3
CARRY_GRASP_STAGE_COUNT = 1
CARRY_TAIL_STAGE_COUNT_MAX = 2
CARRY_TOTAL_STAGES_MAX = (
    CARRY_APPROACH_STAGE_COUNT_MAX + CARRY_GRASP_STAGE_COUNT + CARRY_TAIL_STAGE_COUNT_MAX
)


@dataclass(frozen=True)
class BimanualCarryTaskConfig:
    """双臂对称搬运几何（扁平 YAML / ``dual_arm.carry`` 合并到 ``MergedQueueConfig.carry``）。"""

    source_object_entity_path: str
    # 沿 motion 输出系 **Y**（通常为 arm_base Y）单侧半宽（米），与 object_position_offset[1] 相加后左右对称展开
    object_dual_arm_half_span_y: float
    left_base_orientation: tuple[float, float, float, float]
    right_base_orientation: tuple[float, float, float, float]
    # 可选：与 ``dual_arm.bimanual_align`` / ``dual_arm.bimanual_align_mid_y`` 的 ``orientation_delta_rpy`` 相同语义——
    # ``[roll, pitch, yaw]`` 弧度，左乘 q_delta * q_ee，左右臂共用同一增量（在 ``carry_*_orientation`` 所在系，通常即 motion 系）
    orientation_delta_rpy: tuple[float, float, float] | None = None
    # 预闭合前：在**末端工具系**下相对 Forward 远点的平移（与 ``ee_lift_offset`` 的 tool 语义一致）；不写 / 全 0 则省略 Approach
    carry_prepare_offset: tuple[float, float, float] | None = None
    # 仅 Approach/Forward：在半宽上再沿输出系 **Y** 张开的余量（米）
    arm_merge_distance_y: float = 0.0
    # 物体固连系下相对原点的平移：``(x, 与半宽合并的 y 分量, z)``；中线为 ``(x,0,z)``，左右沿 +Y，经 ``object_position.orientation`` 旋到 motion 系
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 不写 / ``null``：不生成该段；另一段仍按 (0,0,0) 参与合成其终点。含义见 ``carry_linear_displacement_frame``
    ee_lift_offset: tuple[float, float, float] | None = None
    ee_retreat_offset: tuple[float, float, float] | None = None
    # ``tool``：``carry_*_xyz`` 为末端工具系，经 ``carry_*_orientation`` 旋到输出系；``motion``：输出系轴向平移（左右同矢量）
    carry_linear_displacement_frame: str = "tool"
    # 仅 ``carry_linear_displacement_frame: world`` 等扩展用：世界系 frame_id，由队列侧 TF 后传入 ``carry_*_motion_delta``
    carry_linear_world_frame_id: str | None = None
    # 可选：双臂 movel 时长，执行前写入 ``arm_controller.movel_duration``（类比 ``body_movej_duration``）
    arm_movel_duration: float | None = None
    # 可选：笛卡尔目标/header 坐标系；不写则与队列 ``ctx.frame_id``（多为 base_link）一致。例 ``arm_base``，与 place 一致时需 TF
    motion_frame_id: str | None = None
    tf_lookup_timeout: float | None = None


def carry_approach_stage_count(cfg: BimanualCarryTaskConfig) -> int:
    """无预接近偏移时为 2（Forward+CloseIn）；否则为 3（Approach+Forward+CloseIn）。"""
    return 3 if carry_prepare_offset_is_active(cfg.carry_prepare_offset) else 2


def format_bimanual_carry_task_cfg_summary(
    scene: str, task_cfg: BimanualCarryTaskConfig,
) -> str:
    return (
        f"[Scene] {scene} -> {task_cfg.source_object_entity_path}, "
        f"object_dual_arm_half_span_y={task_cfg.object_dual_arm_half_span_y}, "
        f"carry_prepare_offset={task_cfg.carry_prepare_offset}, "
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


def slice_carry_stages_for_queue(
    full: list[StageTarget],
    *,
    approach_stage_count: int,
) -> tuple[list[StageTarget], list[StageTarget], list[StageTarget]]:
    """前 ``approach_stage_count`` + 1 段为 approach / grasp；余下 0～2 段为可选 lift / retreat。"""
    if approach_stage_count not in (2, 3):
        raise ValueError(f"approach_stage_count must be 2 or 3, got {approach_stage_count}")
    i = approach_stage_count
    j = i + CARRY_GRASP_STAGE_COUNT
    if len(full) < j:
        raise ValueError(f"carry sequence too short: need >= {j} stages, got {len(full)}")
    tail = len(full) - j
    if tail > CARRY_TAIL_STAGE_COUNT_MAX:
        raise ValueError(f"carry tail too long: at most {CARRY_TAIL_STAGE_COUNT_MAX} lift/retreat, got {tail}")
    return full[0:i], full[i:j], full[j:]


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

    ``carry_lift_motion_delta`` / ``carry_retreat_motion_delta``：已由调用方在 **motion 输出系** 下算好的平移
    （用于 ``carry_linear_displacement_frame: world`` 等需在队列里做 TF 的情形）；非 ``None`` 时优先于
    ``carry_task_cfg`` 内的 ``carry_*_xyz`` 参与 lift/retreat 位移合成。
    """
    lo, ro = _apply_carry_orientation_delta_rpy(
        carry_task_cfg.left_base_orientation,
        carry_task_cfg.right_base_orientation,
        carry_task_cfg.orientation_delta_rpy,
    )
    frame = str(carry_task_cfg.carry_linear_displacement_frame or "tool").strip().lower()
    return build_bimanual_carry_sequence(
        object_position=object_position,
        object_dual_arm_half_span_y=carry_task_cfg.object_dual_arm_half_span_y,
        carry_prepare_offset=carry_task_cfg.carry_prepare_offset,
        arm_merge_distance_y=carry_task_cfg.arm_merge_distance_y,
        object_position_offset=carry_task_cfg.object_position_offset,
        carry_left_orientation=lo,
        carry_right_orientation=ro,
        ee_lift_offset=carry_task_cfg.ee_lift_offset,
        ee_retreat_offset=carry_task_cfg.ee_retreat_offset,
        carry_linear_displacement_frame=frame,
        carry_lift_motion_delta=carry_lift_motion_delta,
        carry_retreat_motion_delta=carry_retreat_motion_delta,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        output_frame_id=output_frame_id,
    )
