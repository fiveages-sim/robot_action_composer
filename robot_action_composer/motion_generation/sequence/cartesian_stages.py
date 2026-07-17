"""Cartesian stage sequences and execution on :class:`ROS2RobotInterface`.

本模块位于 :mod:`robot_action_composer.motion_generation.sequence`；同级的
:mod:`robot_action_composer.motion_generation.tasks` 中的模块在其上组合具体任务序列。

Pose-native builders for common motion sequences (pick, place, handover, carry)
and a configurable executor for unstamped / stamped / dual-arm stamped targets.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import TYPE_CHECKING, Any, Callable

from geometry_msgs.msg import Pose

from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    euler_rpy_to_quat_xyzw,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

if TYPE_CHECKING:
    from ros2_robot_interface.ros_interface import ROS2RobotInterface

logger = logging.getLogger(__name__)

DirectionVec = tuple[float, float, float]

PICK_STAGE_SUFFIXES: tuple[str, ...] = ("Approach", "CloseIn", "Grasp", "Lift", "Retreat")
PLACE_STAGE_SUFFIXES: tuple[str, ...] = ("Place", "Release", "PostReleaseRetreat")
HANDOVER_STAGE_SUFFIXES: tuple[str, ...] = ("SyncMove", "ReceiverGrasp", "SourceRelease")
CARRY_STAGE_SUFFIXES: tuple[str, ...] = (
    "Approach",
    "Forward",
    "CloseIn",
    "PregraspLift",
    "Grasp",
    "Lift",
    "Retreat",
)


def _tool_offset_xyz_to_world_lr(
    vec_tool: tuple[float, float, float],
    q_left_xyzw: tuple[float, float, float, float],
    q_right_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """将工具系位移分别用左、右抓取四元数旋转到 ``output_frame_id`` / motion 系。"""
    return (
        rotate_vector_by_quat(vec_tool, q_left_xyzw),
        rotate_vector_by_quat(vec_tool, q_right_xyzw),
    )


def _pose_xyz_orientation(
    x: float,
    y: float,
    z: float,
    q_xyzw: tuple[float, float, float, float],
) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]
    return p


def _object_orientation_xyzw(p: Pose) -> tuple[float, float, float, float]:
    """从物体 :class:`Pose` 读取姿态四元数 xyzw（与 ``object_position`` 所用坐标系一致，一般为 TF 后的 motion 系）。"""
    return quat_normalize(
        (
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
        ),
    )


def _bimanual_carry_lr_xyz_triple_from_object_body(
    object_position: Pose,
    object_position_offset: tuple[float, float, float],
    object_dual_arm_half_span_y: float,
    arm_merge_distance_y: float,
    vertical_approach_clearance_z: float | None,
    carry_prepare_offset: tuple[float, float, float] | None,
    *,
    object_span_frame: str = "motion",
    object_span_axis: str = "y",
    carry_left_orientation: tuple[float, float, float, float],
    carry_right_orientation: tuple[float, float, float, float],
) -> tuple[
    tuple[tuple[float, float, float], tuple[float, float, float]],
    tuple[tuple[float, float, float], tuple[float, float, float]],
    tuple[tuple[float, float, float], tuple[float, float, float]],
]:
    """抓取几何：中线用物体姿态，``y`` 横向插入；``x`` 顶部竖直下抓。

    - 名义抓取中线相对物体原点：物体系 ``(object_position_offset[0], 0, object_position_offset[2])``，经 ``R(q)`` 映到输出系后加到物体原点位置。
    - ``object_span_axis='y'``：历史横抓方案。左右半宽和 Forward 余量沿 span 轴展开，然后 CloseIn 横向夹入。
    - ``object_span_axis='x'``：顶部下抓方案。左右半宽沿 X 展开，Forward 位姿在 CloseIn 正上方
      ``vertical_approach_clearance_z``，然后 CloseIn 竖直向下。
    - ``carry_prepare_offset``：与各臂 **末端工具系** 下的平移（与 ``ee_lift_offset`` 在 ``tool`` 帧语义一致），
      同一 ``(dx,dy,dz)`` 分别经 ``carry_left_orientation`` / ``carry_right_orientation`` 旋到输出系后加到 Forward 远点。
    """
    q = _object_orientation_xyzw(object_position)
    cx = float(object_position.position.x)
    cy = float(object_position.position.y)
    cz = float(object_position.position.z)
    ocx, ocy, ocz = float(object_position_offset[0]), float(object_position_offset[1]), float(object_position_offset[2])
    half = float(object_dual_arm_half_span_y) + ocy
    clear = float(arm_merge_distance_y)

    d_mid = rotate_vector_by_quat((ocx, 0.0, ocz), q)
    p_mid = (cx + d_mid[0], cy + d_mid[1], cz + d_mid[2])

    axis_raw = str(object_span_axis or "y").strip().lower()
    sign = -1.0 if axis_raw.startswith("-") else 1.0
    axis = axis_raw.lstrip("+-")
    if axis in ("x", "forward", "front_back", "front-back", "longitudinal", "top_down", "top-down", "down"):
        span_axis = "x"
        axis_unit = (sign, 0.0, 0.0)
    elif axis in ("y", "side", "sideways", "left_right", "left-right", "lateral"):
        span_axis = "y"
        axis_unit = (0.0, sign, 0.0)
    elif axis in ("z", "up", "vertical"):
        span_axis = "z"
        axis_unit = (0.0, 0.0, sign)
    else:
        raise ValueError(f"unsupported object_span_axis {object_span_axis!r}; expected 'x', 'y', or 'z'")

    span_frame = str(object_span_frame or "motion").strip().lower()
    if span_frame in ("", "motion", "base", "world"):
        d_half = (axis_unit[0] * half, axis_unit[1] * half, axis_unit[2] * half)
        d_clear = (axis_unit[0] * clear, axis_unit[1] * clear, axis_unit[2] * clear)
    elif span_frame in ("object", "object_body", "body", "local"):
        d_half = rotate_vector_by_quat((axis_unit[0] * half, axis_unit[1] * half, axis_unit[2] * half), q)
        d_clear = rotate_vector_by_quat((axis_unit[0] * clear, axis_unit[1] * clear, axis_unit[2] * clear), q)
    else:
        raise ValueError(f"unsupported object_span_frame {object_span_frame!r}; expected 'motion' or 'object'")

    pl_close = (p_mid[0] + d_half[0], p_mid[1] + d_half[1], p_mid[2] + d_half[2])
    pr_close = (p_mid[0] - d_half[0], p_mid[1] - d_half[1], p_mid[2] - d_half[2])
    if span_axis == "x":
        z_clear = float(vertical_approach_clearance_z) if vertical_approach_clearance_z is not None else clear
        pl_fwd = (pl_close[0], pl_close[1], pl_close[2] + z_clear)
        pr_fwd = (pr_close[0], pr_close[1], pr_close[2] + z_clear)
    else:
        pl_fwd = (
            p_mid[0] + d_half[0] + d_clear[0],
            p_mid[1] + d_half[1] + d_clear[1],
            p_mid[2] + d_half[2] + d_clear[2],
        )
        pr_fwd = (
            p_mid[0] - d_half[0] - d_clear[0],
            p_mid[1] - d_half[1] - d_clear[1],
            p_mid[2] - d_half[2] - d_clear[2],
        )

    if carry_prepare_offset_is_active(carry_prepare_offset):
        ox = float(carry_prepare_offset[0])
        oy = float(carry_prepare_offset[1])
        oz = float(carry_prepare_offset[2])
        vec = (ox, oy, oz)
        d_l = rotate_vector_by_quat(vec, carry_left_orientation)
        d_r = rotate_vector_by_quat(vec, carry_right_orientation)
        pl_app = (pl_fwd[0] + d_l[0], pl_fwd[1] + d_l[1], pl_fwd[2] + d_l[2])
        pr_app = (pr_fwd[0] + d_r[0], pr_fwd[1] + d_r[1], pr_fwd[2] + d_r[2])
    else:
        pl_app, pr_app = pl_fwd, pr_fwd

    return (pl_close, pr_close), (pl_fwd, pr_fwd), (pl_app, pr_app)


def _lr_poses_from_xyz_pair(
    pl: tuple[float, float, float],
    pr: tuple[float, float, float],
    carry_left_orientation: tuple[float, float, float, float],
    carry_right_orientation: tuple[float, float, float, float],
) -> tuple[Pose, Pose]:
    return (
        _pose_xyz_orientation(pl[0], pl[1], pl[2], carry_left_orientation),
        _pose_xyz_orientation(pr[0], pr[1], pr[2], carry_right_orientation),
    )


def carry_prepare_offset_is_active(carry_prepare_offset: tuple[float, float, float] | None) -> bool:
    """为真时生成独立「远」预接近段；``None`` 或三轴全为 0 时与 Forward 位姿重合，由序列中省略该段。"""
    if carry_prepare_offset is None:
        return False
    return any(abs(float(x)) > 1e-12 for x in carry_prepare_offset)
# 与 :func:`build_bimanual_carry_sequence` 互逆：持箱到位 → 下降 → 张开 → Y 向张开 → 后撤
BIMANUAL_PLACE_SUFFIXES: tuple[str, ...] = (
    "AdvanceToLift",
    "LowerToCloseIn",
    "Release",
    "PostReleaseLower",
    "SpreadY",
    "RetreatOpen",
)
# 相对当前末端：平移(持箱) → 松爪 → 沿左右连线外张 → 后撤（与货架 prim 无关）
PLACE_RELATIVE_SUFFIXES: tuple[str, ...] = (
    "TranslateClosed",
    "ReleaseOpen",
    "PostReleaseLower",
    "SpreadOpen",
    "RetreatOpen",
)

_AXIS_LABEL_TO_UNIT_VEC: dict[str, DirectionVec] = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
    # Backward-compatible aliases.
    "top": (0.0, 0.0, 1.0),
    "front": (-1.0, 0.0, 0.0),
    "back": (1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ArmSide(Enum):
    LEFT = "left"
    RIGHT = "right"


class SendMode(Enum):
    """Which ROS2 transport to use when publishing arm targets."""
    UNSTAMPED = "unstamped"
    STAMPED = "stamped"
    DUAL_ARM_STAMPED = "dual_stamped"


class GripperMode(Enum):
    """Which gripper command interface to use."""
    JOINT_POSITION = "joint_position"
    TARGET_COMMAND = "target_command"


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class ArmTarget:
    """Arm-agnostic target: a Pose plus a gripper value."""
    pose: Pose
    gripper: float


@dataclass(frozen=True)
class ArmStage:
    """单臂侧一段（尚未绑定 left/right）：名称 + 目标 + 是否在发送后等待夹爪稳定。"""

    name: str
    target: ArmTarget
    wait_gripper_settle: bool = False
    skip_arrival_check: bool = False
    skip_gripper_command: bool = False


@dataclass
class StageTarget:
    """A single execution stage with optional left/right arm targets."""

    name: str
    left: ArmTarget | None = None
    right: ArmTarget | None = None
    frame_id: str | None = None
    #: 为 True 时，在发臂目标与夹爪指令后额外 ``sleep(gripper_action_wait)``（替代按名字子串猜测）。
    wait_gripper_settle: bool = False
    #: 为 True 时跳过本段夹爪命令下发（仅发送左右臂 pose）。
    skip_gripper_command: bool = False
    #: 为 True 时跳过 ``wait_until_arrive`` 位姿到位检查，仅靠 ``wait_gripper_settle`` 的固定等待。
    #: 用于 Grasp 阶段：夹爪夹到物体后接触力会把 TCP 顶离目标，基于位姿的到位判断会持续超时。
    skip_arrival_check: bool = False

    def to_action_dict(
        self,
        left_ee_prefix: str = "left_ee",
        left_gripper_key: str = "left_gripper.pos",
        right_ee_prefix: str = "right_ee",
        right_gripper_key: str = "right_gripper.pos",
    ) -> dict[str, float]:
        """Flatten to a ``{key: float}`` dict (for recording / LeRobot compatibility)."""
        d: dict[str, float] = {}
        if self.left:
            d.update(_arm_target_to_flat_dict(self.left, left_ee_prefix, left_gripper_key))
        if self.right:
            d.update(_arm_target_to_flat_dict(self.right, right_ee_prefix, right_gripper_key))
        return d


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _arm_target_to_flat_dict(
    target: ArmTarget, ee_prefix: str, gripper_key: str,
) -> dict[str, float]:
    return {
        f"{ee_prefix}.pos.x": target.pose.position.x,
        f"{ee_prefix}.pos.y": target.pose.position.y,
        f"{ee_prefix}.pos.z": target.pose.position.z,
        f"{ee_prefix}.quat.x": target.pose.orientation.x,
        f"{ee_prefix}.quat.y": target.pose.orientation.y,
        f"{ee_prefix}.quat.z": target.pose.orientation.z,
        f"{ee_prefix}.quat.w": target.pose.orientation.w,
        gripper_key: target.gripper,
    }


def _stage_name(prefix: str, index: int, suffix: str) -> str:
    return f"{prefix}-{index}-{suffix}"


def _stages_with_output_frame_id(
    stages: list[StageTarget],
    output_frame_id: str | None,
) -> list[StageTarget]:
    """为 stamped 发送写入每段 ``frame_id``，与 ``execute_stage_sequence`` 中 ``effective_frame_id`` 一致。"""
    if output_frame_id is None:
        return stages
    fid = str(output_frame_id).strip()
    if not fid:
        return stages
    return [replace(s, frame_id=fid) for s in stages]


def _resolve_axis_direction_unit_vec(
    *,
    axis_label: str,
    explicit_direction_vector: DirectionVec | None,
) -> DirectionVec:
    if explicit_direction_vector is not None:
        vx, vy, vz = (
            float(explicit_direction_vector[0]),
            float(explicit_direction_vector[1]),
            float(explicit_direction_vector[2]),
        )
    else:
        key = axis_label.lower()
        if key not in _AXIS_LABEL_TO_UNIT_VEC:
            raise ValueError(
                f"Unsupported axis label: {axis_label}. "
                f"Expected one of: {', '.join(sorted(_AXIS_LABEL_TO_UNIT_VEC))}"
            )
        vx, vy, vz = _AXIS_LABEL_TO_UNIT_VEC[key]

    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-8:
        raise ValueError("explicit_direction_vector norm is too small")
    return (vx / norm, vy / norm, vz / norm)


def _make_pose_from_target(
    target_pose: Pose,
    *,
    offset: float,
    direction_vec: DirectionVec,
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    dx, dy, dz = direction_vec
    pose.position.x = target_pose.position.x + dx * offset
    pose.position.y = target_pose.position.y + dy * offset
    pose.position.z = target_pose.position.z + dz * offset
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


def _make_pose(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


_PLACE_POSE_POS_EPS_SQ = 1e-10


def _poses_same_position(a: Pose, b: Pose, *, eps_sq: float = _PLACE_POSE_POS_EPS_SQ) -> bool:
    dx = float(a.position.x - b.position.x)
    dy = float(a.position.y - b.position.y)
    dz = float(a.position.z - b.position.z)
    return (dx * dx + dy * dy + dz * dz) <= eps_sq


# ---------------------------------------------------------------------------
# Sequence builders (arm-agnostic → list[ArmStage])
# ---------------------------------------------------------------------------

def build_single_arm_pick_sequence(
    *,
    target_pose: Pose,
    ee_base_orientation: tuple[float, float, float, float],
    prepare_offset: tuple[float, float, float] | None = None,
    pick_clearance: float = 0.01,
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    retreat_offset: tuple[float, float, float] | None = None,
    retreat_xyz: tuple[float, float, float] | None = None,
    retreat_open_gripper: bool = False,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Pickup",
) -> list[ArmStage]:
    """Build pick stages: Approach (optional) → CloseIn → Grasp → Lift (optional) → Retreat (optional).

    Used by ``single_arm.pick`` and ``dual_arm.parallel_pick`` only. Drawer pull uses
    :func:`robot_action_composer.motion_generation.tasks.drawer.build_single_arm_pull_drawer_sequence`.

    Lift / Retreat (aligned with :func:`build_bimanual_carry_sequence`):

    - ``retreat_offset`` (motion/output frame): **Lift** delta from CloseIn; ``None`` → skip Lift
      (YAML ``ee_lift_offset`` omitted).
    - ``retreat_xyz`` (motion/output frame): **Retreat** delta from the post-lift pose (CloseIn when
      Lift skipped); ``None`` → skip Retreat (YAML ``ee_retreat_offset`` omitted).
    """
    # pick_clearance 采用工具系语义：沿 ee_base_orientation 的局部 +Z 偏移到 close-in。
    cdx, cdy, cdz = rotate_vector_by_quat((0.0, 0.0, pick_clearance), ee_base_orientation)
    close_in_pose = Pose()
    close_in_pose.position.x = target_pose.position.x + cdx
    close_in_pose.position.y = target_pose.position.y + cdy
    close_in_pose.position.z = target_pose.position.z + cdz
    close_in_pose.orientation.x, close_in_pose.orientation.y, close_in_pose.orientation.z, close_in_pose.orientation.w = (
        ee_base_orientation
    )
    close_in_pose.position.x += object_position_offset[0]
    close_in_pose.position.y += object_position_offset[1]
    close_in_pose.position.z += object_position_offset[2]
    has_prepare = (
        prepare_offset is not None
        and any(abs(float(x)) > 1e-12 for x in prepare_offset)
    )
    if has_prepare and prepare_offset is not None:
        pdx, pdy, pdz = rotate_vector_by_quat(prepare_offset, ee_base_orientation)
        approach_pose = Pose()
        approach_pose.position.x = close_in_pose.position.x + pdx
        approach_pose.position.y = close_in_pose.position.y + pdy
        approach_pose.position.z = close_in_pose.position.z + pdz
        approach_pose.orientation = close_in_pose.orientation
    else:
        approach_pose = close_in_pose
    lift_dx, lift_dy, lift_dz = (
        (float(retreat_offset[0]), float(retreat_offset[1]), float(retreat_offset[2]))
        if retreat_offset is not None
        else (0.0, 0.0, 0.0)
    )
    lift_pose = Pose()
    lift_pose.position.x = close_in_pose.position.x + lift_dx
    lift_pose.position.y = close_in_pose.position.y + lift_dy
    lift_pose.position.z = close_in_pose.position.z + lift_dz
    lift_pose.orientation = close_in_pose.orientation
    has_lift = retreat_offset is not None
    retreat_pose: Pose | None
    if retreat_xyz is not None:
        rdx, rdy, rdz = (float(retreat_xyz[0]), float(retreat_xyz[1]), float(retreat_xyz[2]))
        retreat_pose = Pose()
        retreat_pose.position.x = lift_pose.position.x + rdx
        retreat_pose.position.y = lift_pose.position.y + rdy
        retreat_pose.position.z = lift_pose.position.z + rdz
        retreat_pose.orientation = lift_pose.orientation
    else:
        retreat_pose = None
    stages: list[ArmStage] = []
    i = 1
    if has_prepare:
        stages.append(
            ArmStage(
                _stage_name(stage_prefix, i, PICK_STAGE_SUFFIXES[0]),
                ArmTarget(pose=approach_pose, gripper=gripper_open),
            )
        )
        i += 1
    stages.append(
        ArmStage(
            _stage_name(stage_prefix, i, PICK_STAGE_SUFFIXES[1]),
            ArmTarget(pose=close_in_pose, gripper=gripper_open),
        )
    )
    i += 1
    stages.append(
        ArmStage(
            _stage_name(stage_prefix, i, PICK_STAGE_SUFFIXES[2]),
            ArmTarget(pose=close_in_pose, gripper=gripper_closed),
            wait_gripper_settle=True,
            skip_arrival_check=True,
        )
    )
    if has_lift:
        i += 1
        stages.append(
            ArmStage(
                _stage_name(stage_prefix, i, PICK_STAGE_SUFFIXES[3]),
                ArmTarget(pose=lift_pose, gripper=gripper_closed),
            )
        )
    if retreat_pose is not None:
        i += 1
        stages.append(
            ArmStage(
                _stage_name(stage_prefix, i, PICK_STAGE_SUFFIXES[4]),
                ArmTarget(
                    pose=retreat_pose,
                    gripper=gripper_open if retreat_open_gripper else gripper_closed,
                ),
            )
        )
    return stages


def build_single_arm_place_sequence(
    *,
    place_position: tuple[float, float, float],
    place_orientation: tuple[float, float, float, float],
    ee_retreat_offset: tuple[float, float, float],
    gripper_open: float,
    place_axis: str = "top",
    prepare_offset: tuple[float, float, float] | None = None,
    place_insert_clearance: float = 0.0,
    stage_prefix: str = "Place",
    start_index: int = 1,
) -> list[ArmStage]:
    """Build place stages from a target pose, mirroring pick ``prepare → close-in`` semantics.

    The place *target* is ``place_position`` / ``place_orientation`` (object or slot frame).
    For consistency with single-arm pick, place offsets are interpreted in **tool frame**
    and resolved by ``place_axis``:

    - ``place_insert_clearance``: offset along local place axis from the nominal place target.
    - ``prepare_offset``: tool-frame vector offset from insert pose (same semantics as pick
      ``prepare_offset``), rotated by ``place_orientation``.
    - ``ee_retreat_offset``: local tool-frame vector after release, rotated by
      ``place_orientation`` before being applied.

    When approach and insert poses coincide (e.g. both clearances ``0``), the explicit approach
    stage is omitted (3 stages: Place / Release / PostReleaseRetreat).
    """
    local_axis = _resolve_axis_direction_unit_vec(
        axis_label=place_axis,
        explicit_direction_vector=None,
    )
    target_pose = _make_pose(place_position, place_orientation)
    local_insert = (
        local_axis[0] * float(place_insert_clearance),
        local_axis[1] * float(place_insert_clearance),
        local_axis[2] * float(place_insert_clearance),
    )
    idx, idy, idz = rotate_vector_by_quat(local_insert, place_orientation)
    final_pose = _make_pose(
        (
            float(target_pose.position.x) + idx,
            float(target_pose.position.y) + idy,
            float(target_pose.position.z) + idz,
        ),
        place_orientation,
    )
    has_prepare = prepare_offset is not None and any(abs(float(x)) > 1e-12 for x in prepare_offset)
    if has_prepare and prepare_offset is not None:
        adx, ady, adz = rotate_vector_by_quat(prepare_offset, place_orientation)
        approach_pose = _make_pose(
            (
                float(final_pose.position.x) + adx,
                float(final_pose.position.y) + ady,
                float(final_pose.position.z) + adz,
            ),
            place_orientation,
        )
    else:
        approach_pose = final_pose
    ox, oy, oz = rotate_vector_by_quat(ee_retreat_offset, place_orientation)
    retract_pose = _make_pose(
        (
            float(final_pose.position.x) + ox,
            float(final_pose.position.y) + oy,
            float(final_pose.position.z) + oz,
        ),
        place_orientation,
    )

    need_approach = has_prepare and (not _poses_same_position(approach_pose, final_pose))
    idx = start_index
    stages: list[ArmStage] = []

    if need_approach:
        stages.append(
            ArmStage(
                _stage_name(stage_prefix, idx, "Approach"),
                ArmTarget(pose=approach_pose, gripper=0.0),
                skip_gripper_command=True,
            ),
        )
        idx += 1

    stages.append(
        ArmStage(
            _stage_name(stage_prefix, idx, PLACE_STAGE_SUFFIXES[0]),
            ArmTarget(pose=final_pose, gripper=0.0),
            skip_gripper_command=True,
        ),
    )
    idx += 1
    stages.append(
        ArmStage(
            _stage_name(stage_prefix, idx, PLACE_STAGE_SUFFIXES[1]),
            ArmTarget(pose=final_pose, gripper=gripper_open),
            wait_gripper_settle=True,
        ),
    )
    idx += 1
    stages.append(
        ArmStage(
            _stage_name(stage_prefix, idx, PLACE_STAGE_SUFFIXES[2]),
            ArmTarget(pose=retract_pose, gripper=gripper_open),
        ),
    )
    return stages


def build_single_arm_return_home_sequence(
    *,
    home_pose: Pose,
    gripper: float,
    stage_name: str = "Return-1-ReturnHome",
) -> list[ArmStage]:
    return [ArmStage(stage_name, ArmTarget(pose=home_pose, gripper=gripper))]


# ---------------------------------------------------------------------------
# Arm assignment / composition (ArmStage → StageTarget)
# ---------------------------------------------------------------------------

def assign_to_arm(
    sequence: list[ArmStage],
    side: ArmSide,
) -> list[StageTarget]:
    """Assign an arm-agnostic sequence to a specific arm side."""
    result: list[StageTarget] = []
    for spec in sequence:
        if side == ArmSide.LEFT:
            result.append(
                StageTarget(
                    name=spec.name,
                    left=spec.target,
                    wait_gripper_settle=spec.wait_gripper_settle,
                    skip_arrival_check=spec.skip_arrival_check,
                    skip_gripper_command=spec.skip_gripper_command,
                ),
            )
        else:
            result.append(
                StageTarget(
                    name=spec.name,
                    right=spec.target,
                    wait_gripper_settle=spec.wait_gripper_settle,
                    skip_arrival_check=spec.skip_arrival_check,
                    skip_gripper_command=spec.skip_gripper_command,
                ),
            )
    return result


def compose_bimanual_synchronized_sequence(
    left_sequence: list[ArmStage],
    right_sequence: list[ArmStage],
) -> list[StageTarget]:
    """Merge two arm-agnostic sequences into a synchronised bimanual sequence."""
    if len(left_sequence) != len(right_sequence):
        raise ValueError(
            "Left/right sequences must have the same number of stages "
            "for synchronized composition"
        )
    merged: list[StageTarget] = []
    for idx, (left_spec, right_spec) in enumerate(zip(left_sequence, right_sequence), start=1):
        stage_label = f"{idx:02d}-{left_spec.name}|{right_spec.name}"
        merged.append(
            StageTarget(
                name=stage_label,
                left=left_spec.target,
                right=right_spec.target,
                wait_gripper_settle=(
                    left_spec.wait_gripper_settle or right_spec.wait_gripper_settle
                ),
            ),
        )
    return merged


def build_handover_sequence(
    *,
    source_handover_pose: Pose,
    receiver_handover_pose: Pose,
    source_arm: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Handover",
) -> list[StageTarget]:
    """Build a bimanual handover sequence (inherently two-armed)."""
    receiver_open = ArmTarget(pose=receiver_handover_pose, gripper=gripper_open)
    receiver_closed = ArmTarget(pose=receiver_handover_pose, gripper=gripper_closed)
    source_closed = ArmTarget(pose=source_handover_pose, gripper=gripper_closed)
    source_open = ArmTarget(pose=source_handover_pose, gripper=gripper_open)

    def _target(
        idx: int,
        suffix: str,
        src: ArmTarget,
        rcv: ArmTarget,
        *,
        wait_gripper_settle: bool = False,
    ) -> StageTarget:
        name = _stage_name(stage_prefix, idx, suffix)
        if source_arm == ArmSide.LEFT:
            return StageTarget(
                name=name,
                left=src,
                right=rcv,
                wait_gripper_settle=wait_gripper_settle,
            )
        return StageTarget(
            name=name,
            left=rcv,
            right=src,
            wait_gripper_settle=wait_gripper_settle,
        )

    return [
        _target(1, HANDOVER_STAGE_SUFFIXES[0], source_closed, receiver_open),
        _target(
            2,
            HANDOVER_STAGE_SUFFIXES[1],
            source_closed,
            receiver_closed,
            wait_gripper_settle=True,
        ),
        _target(
            3,
            HANDOVER_STAGE_SUFFIXES[2],
            source_open,
            receiver_closed,
            wait_gripper_settle=True,
        ),
    ]


def build_bimanual_carry_sequence(
    *,
    object_position: Pose,
    object_dual_arm_half_span_y: float,
    carry_prepare_offset: tuple[float, float, float] | None = None,
    arm_merge_distance_y: float = 0.0,
    vertical_approach_clearance_z: float | None = None,
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    object_span_frame: str = "motion",
    object_span_axis: str = "y",
    carry_left_orientation: tuple[float, float, float, float],
    carry_right_orientation: tuple[float, float, float, float],
    ee_pregrasp_lift_offset: tuple[float, float, float] | None = None,
    ee_lift_offset: tuple[float, float, float] | None = None,
    ee_retreat_offset: tuple[float, float, float] | None = None,
    carry_linear_displacement_frame: str = "tool",
    carry_lift_motion_delta: tuple[float, float, float] | None = None,
    carry_retreat_motion_delta: tuple[float, float, float] | None = None,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Carry",
    output_frame_id: str | None = None,
) -> list[StageTarget]:
    """Build a bimanual symmetric carry sequence.

    Both arms approach, close in, optionally pregrasp-lift while keeping grippers open,
    grasp, then optional lift / retreat.

    **抓取几何**：``object_position_offset[0]`` / ``object_position_offset[2]`` 与物体系 X/Z 对齐，经 ``R(q)`` 映到输出系后定抓取中线。
    ``object_span_axis='y'`` 沿左右横向展开并横插；``object_span_axis='x'`` 沿 X 分开双手，
    Forward 位姿在 CloseIn 上方 ``vertical_approach_clearance_z``，再竖直下抓。
    ``carry_prepare_offset`` 为末端工具系 ``(dx,dy,dz)``，左右各用 ``carry_*_orientation`` 旋到输出系后加到 Forward 远点。

    ``ee_pregrasp_lift_offset`` / ``ee_lift_offset`` / ``ee_retreat_offset`` 为 ``None``
    （YAML 不写）时**不生成**对应段；
    几何上仍用 ``(0,0,0)`` 合成另一段的终点（例如仅后撤时终点相对抓取点只加 ``ee_retreat_offset``）。

    **Lift/Retreat 位移系**（``carry_linear_displacement_frame``）：``motion`` 时 ``carry_*_xyz`` 为输出系
    轴向分量，左右同加；``tool`` 时为末端工具系，经 ``carry_*_orientation`` 旋到输出系；若传入
    ``carry_lift_motion_delta`` / ``carry_retreat_motion_delta``（已为输出系向量），则优先用于该段。

    ``carry_prepare_offset`` 为 ``None`` 或 ``[0,0,0]`` 时**不生成**第一段 Approach（与 Forward 重合），
    序列以 Forward → CloseIn 起。

    ``output_frame_id``：写入各 :class:`StageTarget` 的 ``frame_id``，供 stamped / dual_stamped
    与位姿数值所用系一致（一般为队列 ``ctx.frame_id``）。
    """
    (pl_close, pr_close), (pl_fwd, pr_fwd), (pl_app, pr_app) = _bimanual_carry_lr_xyz_triple_from_object_body(
        object_position,
        object_position_offset,
        object_dual_arm_half_span_y,
        arm_merge_distance_y,
        vertical_approach_clearance_z,
        carry_prepare_offset,
        object_span_frame=object_span_frame,
        object_span_axis=object_span_axis,
        carry_left_orientation=carry_left_orientation,
        carry_right_orientation=carry_right_orientation,
    )
    has_prepare = carry_prepare_offset_is_active(carry_prepare_offset)
    closein_l, closein_r = _lr_poses_from_xyz_pair(
        pl_close, pr_close, carry_left_orientation, carry_right_orientation,
    )
    forward_l, forward_r = _lr_poses_from_xyz_pair(
        pl_fwd, pr_fwd, carry_left_orientation, carry_right_orientation,
    )
    approach_l, approach_r = _lr_poses_from_xyz_pair(
        pl_app, pr_app, carry_left_orientation, carry_right_orientation,
    )
    # 4-PregraspLift (optional): keep gripper open and move up in tool/motion frame.
    eepx, eepy, eepz = (
        (
            float(ee_pregrasp_lift_offset[0]),
            float(ee_pregrasp_lift_offset[1]),
            float(ee_pregrasp_lift_offset[2]),
        )
        if ee_pregrasp_lift_offset is not None
        else (0.0, 0.0, 0.0)
    )
    # 5-Grasp: close grippers at current close-in/pregrasp-lift pose.
    eff_lx, eff_ly, eff_lz = (
        (float(ee_lift_offset[0]), float(ee_lift_offset[1]), float(ee_lift_offset[2]))
        if ee_lift_offset is not None
        else (0.0, 0.0, 0.0)
    )
    eff_rx, eff_ry, eff_rz = (
        (float(ee_retreat_offset[0]), float(ee_retreat_offset[1]), float(ee_retreat_offset[2]))
        if ee_retreat_offset is not None
        else (0.0, 0.0, 0.0)
    )
    glx, gly, glz = pl_close[0], pl_close[1], pl_close[2]
    grx, gry, grz = pr_close[0], pr_close[1], pr_close[2]
    lin_frame = str(carry_linear_displacement_frame or "tool").strip().lower()
    if carry_lift_motion_delta is not None:
        dl = dr = (
            float(carry_lift_motion_delta[0]),
            float(carry_lift_motion_delta[1]),
            float(carry_lift_motion_delta[2]),
        )
    elif ee_lift_offset is not None and lin_frame == "motion":
        dl = dr = (eff_lx, eff_ly, eff_lz)
    elif ee_lift_offset is not None:
        dl, dr = _tool_offset_xyz_to_world_lr(
            (eff_lx, eff_ly, eff_lz), carry_left_orientation, carry_right_orientation,
        )
    else:
        z0 = (0.0, 0.0, 0.0)
        dl = dr = z0
    if carry_retreat_motion_delta is not None:
        tl = tr = (
            float(carry_retreat_motion_delta[0]),
            float(carry_retreat_motion_delta[1]),
            float(carry_retreat_motion_delta[2]),
        )
    elif ee_retreat_offset is not None and lin_frame == "motion":
        tl = tr = (eff_rx, eff_ry, eff_rz)
    elif ee_retreat_offset is not None:
        tl, tr = _tool_offset_xyz_to_world_lr(
            (eff_rx, eff_ry, eff_rz), carry_left_orientation, carry_right_orientation,
        )
    else:
        tl = tr = (0.0, 0.0, 0.0)
    if ee_pregrasp_lift_offset is not None and lin_frame == "motion":
        pdl = pdr = (eepx, eepy, eepz)
    elif ee_pregrasp_lift_offset is not None:
        pdl, pdr = _tool_offset_xyz_to_world_lr(
            (eepx, eepy, eepz), carry_left_orientation, carry_right_orientation,
        )
    else:
        pdl = pdr = (0.0, 0.0, 0.0)
    pregrasp_l = _pose_xyz_orientation(glx + pdl[0], gly + pdl[1], glz + pdl[2], carry_left_orientation)
    pregrasp_r = _pose_xyz_orientation(grx + pdr[0], gry + pdr[1], grz + pdr[2], carry_right_orientation)
    lift_l = _pose_xyz_orientation(
        glx + pdl[0] + dl[0],
        gly + pdl[1] + dl[1],
        glz + pdl[2] + dl[2],
        carry_left_orientation,
    )
    lift_r = _pose_xyz_orientation(
        grx + pdr[0] + dr[0],
        gry + pdr[1] + dr[1],
        grz + pdr[2] + dr[2],
        carry_right_orientation,
    )
    retreat_l = _pose_xyz_orientation(
        glx + pdl[0] + dl[0] + tl[0],
        gly + pdl[1] + dl[1] + tl[1],
        glz + pdl[2] + dl[2] + tl[2],
        carry_left_orientation,
    )
    retreat_r = _pose_xyz_orientation(
        grx + pdr[0] + dr[0] + tr[0],
        gry + pdr[1] + dr[1] + tr[1],
        grz + pdr[2] + dr[2] + tr[2],
        carry_right_orientation,
    )

    stages: list[StageTarget] = []
    si = 0
    if has_prepare:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, CARRY_STAGE_SUFFIXES[0]),
                left=ArmTarget(pose=approach_l, gripper=gripper_open),
                right=ArmTarget(pose=approach_r, gripper=gripper_open),
            ),
        )
    si += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, si, CARRY_STAGE_SUFFIXES[1]),
            left=ArmTarget(pose=forward_l, gripper=gripper_open),
            right=ArmTarget(pose=forward_r, gripper=gripper_open),
        ),
    )
    si += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, si, CARRY_STAGE_SUFFIXES[2]),
            left=ArmTarget(pose=closein_l, gripper=gripper_open),
            right=ArmTarget(pose=closein_r, gripper=gripper_open),
        ),
    )
    if ee_pregrasp_lift_offset is not None:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, CARRY_STAGE_SUFFIXES[3]),
                left=ArmTarget(pose=pregrasp_l, gripper=gripper_open),
                right=ArmTarget(pose=pregrasp_r, gripper=gripper_open),
            ),
        )
    si += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, si, CARRY_STAGE_SUFFIXES[4]),
            left=ArmTarget(
                pose=pregrasp_l if ee_pregrasp_lift_offset is not None else closein_l,
                gripper=gripper_closed,
            ),
            right=ArmTarget(
                pose=pregrasp_r if ee_pregrasp_lift_offset is not None else closein_r,
                gripper=gripper_closed,
            ),
            wait_gripper_settle=True,
        ),
    )
    idx = si + 1
    if ee_lift_offset is not None:
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, idx, CARRY_STAGE_SUFFIXES[5]),
                left=ArmTarget(pose=lift_l, gripper=gripper_closed),
                right=ArmTarget(pose=lift_r, gripper=gripper_closed),
            ),
        )
        idx += 1
    if ee_retreat_offset is not None:
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, idx, CARRY_STAGE_SUFFIXES[6]),
                left=ArmTarget(pose=retreat_l, gripper=gripper_closed),
                right=ArmTarget(pose=retreat_r, gripper=gripper_closed),
            ),
        )
    return _stages_with_output_frame_id(stages, output_frame_id)


def build_bimanual_place_sequence(
    *,
    object_position: Pose,
    object_dual_arm_half_span_y: float,
    carry_prepare_offset: tuple[float, float, float] | None = None,
    arm_merge_distance_y: float = 0.0,
    vertical_approach_clearance_z: float | None = None,
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    object_span_frame: str = "motion",
    object_span_axis: str = "y",
    carry_left_orientation: tuple[float, float, float, float],
    carry_right_orientation: tuple[float, float, float, float],
    ee_pregrasp_lift_offset: tuple[float, float, float] | None = None,
    ee_lift_offset: tuple[float, float, float] | None = None,
    ee_retreat_offset: tuple[float, float, float] | None = None,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Place",
    output_frame_id: str | None = None,
) -> list[StageTarget]:
    """双臂对称放置：与 :func:`build_bimanual_carry_sequence` 几何一致，阶段顺序为其逆（持箱后释放）。

    起始姿态与 carry 末段一致：若有后撤段则双手在 retreat，否则在 lift（或未抬升则已在合拢位）。
    ``ee_pregrasp_lift_offset`` / ``ee_lift_offset`` / ``ee_retreat_offset`` 为 ``None`` 时不生成与 carry 对应逆段（与 YAML 不写即跳过一致）。

    ``carry_prepare_offset`` 为 ``None`` 或全 0 时，末段 ``RetreatOpen`` 与 ``SpreadY`` 位姿重合，不生成该末段
    （与 :func:`build_bimanual_carry_sequence` 省略 Approach 一致）。

    ``output_frame_id``：各段 ``StageTarget.frame_id``，与 stamped 位姿系一致（一般为 ``ctx.frame_id``）。

    与 :func:`build_bimanual_carry_sequence` 相同：抓取几何见该函数；``carry_prepare_offset`` 与
    ``ee_pregrasp_lift_offset`` / ``ee_lift_offset`` / ``ee_retreat_offset`` 均为末端工具系位移（经各自姿态旋到输出系）。
    """
    (pl_close, pr_close), (pl_fwd, pr_fwd), (pl_app, pr_app) = _bimanual_carry_lr_xyz_triple_from_object_body(
        object_position,
        object_position_offset,
        object_dual_arm_half_span_y,
        arm_merge_distance_y,
        vertical_approach_clearance_z,
        carry_prepare_offset,
        object_span_frame=object_span_frame,
        object_span_axis=object_span_axis,
        carry_left_orientation=carry_left_orientation,
        carry_right_orientation=carry_right_orientation,
    )
    has_prepare = carry_prepare_offset_is_active(carry_prepare_offset)
    closein_l, closein_r = _lr_poses_from_xyz_pair(
        pl_close, pr_close, carry_left_orientation, carry_right_orientation,
    )
    forward_l, forward_r = _lr_poses_from_xyz_pair(
        pl_fwd, pr_fwd, carry_left_orientation, carry_right_orientation,
    )
    approach_l, approach_r = _lr_poses_from_xyz_pair(
        pl_app, pr_app, carry_left_orientation, carry_right_orientation,
    )
    has_pregrasp_lift = ee_pregrasp_lift_offset is not None
    has_lift = ee_lift_offset is not None
    has_retreat = ee_retreat_offset is not None
    eff_px, eff_py, eff_pz = (
        (
            float(ee_pregrasp_lift_offset[0]),
            float(ee_pregrasp_lift_offset[1]),
            float(ee_pregrasp_lift_offset[2]),
        )
        if ee_pregrasp_lift_offset is not None
        else (0.0, 0.0, 0.0)
    )
    eff_lx, eff_ly, eff_lz = (
        (float(ee_lift_offset[0]), float(ee_lift_offset[1]), float(ee_lift_offset[2]))
        if ee_lift_offset is not None
        else (0.0, 0.0, 0.0)
    )
    eff_rx, eff_ry, eff_rz = (
        (float(ee_retreat_offset[0]), float(ee_retreat_offset[1]), float(ee_retreat_offset[2]))
        if ee_retreat_offset is not None
        else (0.0, 0.0, 0.0)
    )
    glx, gly, glz = pl_close[0], pl_close[1], pl_close[2]
    grx, gry, grz = pr_close[0], pr_close[1], pr_close[2]
    if ee_pregrasp_lift_offset is not None:
        pdl, pdr = _tool_offset_xyz_to_world_lr(
            (eff_px, eff_py, eff_pz), carry_left_orientation, carry_right_orientation,
        )
    else:
        pdl = pdr = (0.0, 0.0, 0.0)
    if ee_lift_offset is not None:
        dl, dr = _tool_offset_xyz_to_world_lr(
            (eff_lx, eff_ly, eff_lz), carry_left_orientation, carry_right_orientation,
        )
    else:
        z0 = (0.0, 0.0, 0.0)
        dl = dr = z0
    if ee_retreat_offset is not None:
        tl, tr = _tool_offset_xyz_to_world_lr(
            (eff_rx, eff_ry, eff_rz), carry_left_orientation, carry_right_orientation,
        )
    else:
        tl = tr = (0.0, 0.0, 0.0)
    pregrasp_l = _pose_xyz_orientation(glx + pdl[0], gly + pdl[1], glz + pdl[2], carry_left_orientation)
    pregrasp_r = _pose_xyz_orientation(grx + pdr[0], gry + pdr[1], grz + pdr[2], carry_right_orientation)
    lift_l = _pose_xyz_orientation(
        glx + pdl[0] + dl[0],
        gly + pdl[1] + dl[1],
        glz + pdl[2] + dl[2],
        carry_left_orientation,
    )
    lift_r = _pose_xyz_orientation(
        grx + pdr[0] + dr[0],
        gry + pdr[1] + dr[1],
        grz + pdr[2] + dr[2],
        carry_right_orientation,
    )
    retreat_l = _pose_xyz_orientation(
        glx + pdl[0] + dl[0] + tl[0],
        gly + pdl[1] + dl[1] + tl[1],
        glz + pdl[2] + dl[2] + tl[2],
        carry_left_orientation,
    )
    retreat_r = _pose_xyz_orientation(
        grx + pdr[0] + dr[0] + tr[0],
        gry + pdr[1] + dr[1] + tr[1],
        grz + pdr[2] + dr[2] + tr[2],
        carry_right_orientation,
    )

    stages: list[StageTarget] = []
    si = 0
    if has_retreat:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[0]),
                left=ArmTarget(pose=retreat_l, gripper=gripper_closed),
                right=ArmTarget(pose=retreat_r, gripper=gripper_closed),
            ),
        )
    if has_lift:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[1]),
                left=ArmTarget(
                    pose=pregrasp_l if has_pregrasp_lift else closein_l,
                    gripper=gripper_closed,
                ),
                right=ArmTarget(
                    pose=pregrasp_r if has_pregrasp_lift else closein_r,
                    gripper=gripper_closed,
                ),
            ),
        )
    si += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[2]),
            left=ArmTarget(
                pose=pregrasp_l if has_pregrasp_lift else closein_l,
                gripper=gripper_open,
            ),
            right=ArmTarget(
                pose=pregrasp_r if has_pregrasp_lift else closein_r,
                gripper=gripper_open,
            ),
            wait_gripper_settle=True,
        ),
    )
    if has_pregrasp_lift:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[3]),
                left=ArmTarget(pose=closein_l, gripper=gripper_open),
                right=ArmTarget(pose=closein_r, gripper=gripper_open),
            ),
        )
    si += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[4]),
            left=ArmTarget(pose=forward_l, gripper=gripper_open),
            right=ArmTarget(pose=forward_r, gripper=gripper_open),
        ),
    )
    if has_prepare:
        si += 1
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, si, BIMANUAL_PLACE_SUFFIXES[5]),
                left=ArmTarget(pose=approach_l, gripper=gripper_open),
                right=ArmTarget(pose=approach_r, gripper=gripper_open),
            ),
        )
    return _stages_with_output_frame_id(stages, output_frame_id)


def _pose_translate_copy(p: Pose, dx: float, dy: float, dz: float) -> Pose:
    out = Pose()
    out.position.x = p.position.x + dx
    out.position.y = p.position.y + dy
    out.position.z = p.position.z + dz
    out.orientation.x = p.orientation.x
    out.orientation.y = p.orientation.y
    out.orientation.z = p.orientation.z
    out.orientation.w = p.orientation.w
    return out


def _lr_unit_xy_left_from_right(left: Pose, right: Pose) -> tuple[float, float]:
    """XY 平面内由右腕指向左腕的单位向量（用于两侧对称外张）。"""
    dx = left.position.x - right.position.x
    dy = left.position.y - right.position.y
    h = math.hypot(dx, dy)
    if h < 1e-9:
        return 0.0, 1.0
    return dx / h, dy / h


def build_bimanual_place_relative_sequence(
    *,
    left_current: Pose,
    right_current: Pose,
    translation_xyz: tuple[float, float, float],
    spread_half: float,
    post_release_lower_xyz: tuple[float, float, float] | None,
    retreat_xyz: tuple[float, float, float],
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "PlaceRel",
    output_frame_id: str | None = None,
    orientation_delta_rpy: tuple[float, float, float] | None = None,
    post_release_right_delta_xyz: tuple[float, float, float] | None = None,
    post_release_right_orientation_delta_rpy: tuple[float, float, float] | None = None,
) -> list[StageTarget]:
    """基于**当前**左右末端位姿的相对放置（与物体/货架 prim 无关）。

    调用方保证 ``left_current`` / ``right_current`` 与发送目标使用**同一**坐标系
    （通常为 ``ctx.frame_id``；若使用随腰转动的系，则由 ``dual_arm.place_relative`` 先做 TF 再传入）。

    ``output_frame_id``：写入各段 ``frame_id``，须与位姿数值及 ``ExecutionMeta.frame_id`` 一致。

    1. 左右同加一段平移（夹爪仍闭合）。
       - 未配置 ``post_release_lower_xyz``：该段就是完整 ``translation_xyz``（一段到参考 offset）。
       - 配置了 ``post_release_lower_xyz``：该段先走 ``translation_xyz - post_release_lower_xyz``。
    2. （可选）左右手同时左乘 ``orientation_delta_rpy``，用于松爪前整体调整姿态。
    3. 同一位姿松爪（``wait_gripper_settle``）。
    4. （可选）在松爪后再同加 ``post_release_lower_xyz``（米），使两段合计到达 ``translation_xyz``（两段到参考 offset）。
    5. （可选）再单独对右手同加 ``post_release_right_delta_xyz``，用于先避开卡点再外抽。
    6. （可选）再单独对右手左乘 ``post_release_right_orientation_delta_rpy``，用于小角度避让。
    7. 沿「右→左」在 XY 平面的方向各外张 ``spread_half``（米）。
    8. 再对左右同加 ``retreat_xyz`` 后撤。

    未配置姿态增量时，姿态全程保持与平移前一致（仅位置变）。
    """
    tx, ty, tz = translation_xyz
    if post_release_lower_xyz is not None:
        lx, ly, lz = post_release_lower_xyz
        # 两段到位：先走 (translation - lower)，开爪后再补 lower 到达最终参考 offset。
        pre_tx, pre_ty, pre_tz = tx - lx, ty - ly, tz - lz
        l1 = _pose_translate_copy(left_current, pre_tx, pre_ty, pre_tz)
        r1 = _pose_translate_copy(right_current, pre_tx, pre_ty, pre_tz)
        l_mid = _pose_translate_copy(l1, lx, ly, lz)
        r_mid = _pose_translate_copy(r1, lx, ly, lz)
    else:
        l1 = _pose_translate_copy(left_current, tx, ty, tz)
        r1 = _pose_translate_copy(right_current, tx, ty, tz)
        l_mid, r_mid = l1, r1

    if orientation_delta_rpy is not None:
        dr, dp, dyaw = orientation_delta_rpy
        qd = quat_normalize(euler_rpy_to_quat_xyzw(dr, dp, dyaw))

        def _rotate_pose(p: Pose) -> Pose:
            q = quat_normalize(quat_multiply(qd, _object_orientation_xyzw(p)))
            return _pose_xyz_orientation(p.position.x, p.position.y, p.position.z, q)

        l1 = _rotate_pose(l1)
        r1 = _rotate_pose(r1)
        l_mid = _rotate_pose(l_mid)
        r_mid = _rotate_pose(r_mid)

    stages = [
        StageTarget(
            name=_stage_name(stage_prefix, 1, PLACE_RELATIVE_SUFFIXES[0]),
            left=ArmTarget(pose=l1, gripper=gripper_closed),
            right=ArmTarget(pose=r1, gripper=gripper_closed),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 2, PLACE_RELATIVE_SUFFIXES[1]),
            left=ArmTarget(pose=l1, gripper=gripper_open),
            right=ArmTarget(pose=r1, gripper=gripper_open),
            wait_gripper_settle=True,
        ),
    ]
    idx = 3
    if post_release_lower_xyz is not None:
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, idx, PLACE_RELATIVE_SUFFIXES[2]),
                left=ArmTarget(pose=l_mid, gripper=gripper_open),
                right=ArmTarget(pose=r_mid, gripper=gripper_open),
            ),
        )
        idx += 1
    if post_release_right_delta_xyz is not None:
        rdx, rdy, rdz = post_release_right_delta_xyz
        r_mid = _pose_translate_copy(r_mid, rdx, rdy, rdz)
    if post_release_right_orientation_delta_rpy is not None:
        dr, dp, dyaw = post_release_right_orientation_delta_rpy
        qd = quat_normalize(euler_rpy_to_quat_xyzw(dr, dp, dyaw))
        qr = quat_normalize(quat_multiply(qd, _object_orientation_xyzw(r_mid)))
        r_mid = _pose_xyz_orientation(
            r_mid.position.x,
            r_mid.position.y,
            r_mid.position.z,
            qr,
        )
    if post_release_right_delta_xyz is not None or post_release_right_orientation_delta_rpy is not None:
        stages.append(
            StageTarget(
                name=_stage_name(stage_prefix, idx, "PostReleaseRightAdjust"),
                left=ArmTarget(pose=l_mid, gripper=gripper_open),
                right=ArmTarget(pose=r_mid, gripper=gripper_open),
            ),
        )
        idx += 1

    ux, uy = _lr_unit_xy_left_from_right(l_mid, r_mid)
    sh = float(spread_half)
    l2 = _pose_translate_copy(l_mid, ux * sh, uy * sh, 0.0)
    r2 = _pose_translate_copy(r_mid, -ux * sh, -uy * sh, 0.0)

    rx, ry, rz = retreat_xyz
    l3 = _pose_translate_copy(l2, rx, ry, rz)
    r3 = _pose_translate_copy(r2, rx, ry, rz)

    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, idx, PLACE_RELATIVE_SUFFIXES[3]),
            left=ArmTarget(pose=l2, gripper=gripper_open),
            right=ArmTarget(pose=r2, gripper=gripper_open),
        ),
    )
    idx += 1
    stages.append(
        StageTarget(
            name=_stage_name(stage_prefix, idx, PLACE_RELATIVE_SUFFIXES[4]),
            left=ArmTarget(pose=l3, gripper=gripper_open),
            right=ArmTarget(pose=r3, gripper=gripper_open),
        ),
    )
    return _stages_with_output_frame_id(stages, output_frame_id)


# ---------------------------------------------------------------------------
# Executor helpers
# ---------------------------------------------------------------------------

def _send_arm_targets(
    interface: ROS2RobotInterface,
    stage: StageTarget,
    send_mode: SendMode,
    frame_id: str,
) -> None:
    if send_mode == SendMode.DUAL_ARM_STAMPED and stage.left and stage.right:
        interface.send_dual_arm_target_stamped(
            stage.left.pose, stage.right.pose, frame_id,
        )
        return

    use_stamped = send_mode in (SendMode.STAMPED, SendMode.DUAL_ARM_STAMPED)
    if stage.left:
        if use_stamped:
            interface.left_arm_handler.send_target_stamped(frame_id, stage.left.pose)
        else:
            interface.left_arm_handler.send_target(stage.left.pose)
    if stage.right and interface.right_arm_handler:
        if use_stamped:
            interface.right_arm_handler.send_target_stamped(frame_id, stage.right.pose)
        else:
            interface.right_arm_handler.send_target(stage.right.pose)


def _send_one_gripper_command(handler: Any, gripper: float, gripper_mode: GripperMode) -> None:
    """Send gripper target; prefer ``target_percent`` when the handler exposes it."""
    g = float(gripper)
    if gripper_mode == GripperMode.TARGET_COMMAND:
        if getattr(handler, "target_percent_pub", None) is not None:
            handler.send_position_percent(g)
            return
        if g >= 1.0 - 1e-6:
            handler.send_target_command(1)
            return
        if g <= 1e-6:
            handler.send_target_command(0)
            return
        if getattr(handler, "command_pub", None) is not None:
            cfg = handler.config
            lo = float(cfg.gripper_min_position)
            hi = float(cfg.gripper_max_position)
            handler.send_joint_positions(lo + g * (hi - lo))
            return
        handler.send_target_command(1 if g >= 0.5 else 0)
        return
    handler.send_joint_positions(g)


def _send_gripper_commands(
    interface: ROS2RobotInterface,
    stage: StageTarget,
    gripper_mode: GripperMode,
) -> None:
    if stage.skip_gripper_command:
        return
    if stage.left and interface.left_gripper_handler:
        _send_one_gripper_command(
            interface.left_gripper_handler, stage.left.gripper, gripper_mode
        )
    if stage.right and interface.right_gripper_handler:
        _send_one_gripper_command(
            interface.right_gripper_handler, stage.right.gripper, gripper_mode
        )


# ---------------------------------------------------------------------------
# Main executor
# ---------------------------------------------------------------------------

def pose_tol_ori_to_orient_deg(pose_tol_ori: float | None) -> float | None:
    """将 ``QueueSliceCommon.pose_tol_ori``（四元数距离 ``1-|dot|``）转为 handler 使用的角度（度）。

    ``ArmHandler.check_arrival`` 的 ``orient_threshold`` 单位是度。
    若传入值 ``> 1``，视为已是度（兼容误填），原样返回。
    """
    if pose_tol_ori is None:
        return None
    try:
        d = float(pose_tol_ori)
    except (TypeError, ValueError):
        return None
    if d > 1.0:
        return d
    d = max(0.0, min(1.0, d))
    abs_dot = max(0.0, min(1.0, 1.0 - d))
    return math.degrees(2.0 * math.acos(abs_dot))


def execute_stage_sequence(
    *,
    interface: ROS2RobotInterface,
    sequence: list[StageTarget],
    send_mode: SendMode = SendMode.UNSTAMPED,
    frame_id: str = "arm_base",
    gripper_mode: GripperMode = GripperMode.TARGET_COMMAND,
    arrival_timeout: float,
    arrival_poll: float,
    time_now_fn: Callable[[], float],
    sleep_fn: Callable[[float], None],
    gripper_action_wait: float,
    left_arrival_guard_stage: str | None = None,
    warn_prefix: str = "Stage timeout",
    pose_tol_pos: float | None = None,
    pose_tol_ori: float | None = None,
    on_stage_start: Callable[[str, StageTarget], None] | None = None,
    on_stage_poll: Callable[[str, str, dict[str, Any] | None, float], None] | None = None,
    on_stage_end: Callable[[str, dict[str, Any], dict[str, Any]], None] | None = None,
) -> None:
    """Execute a sequence of :class:`StageTarget` stages.

    Which arms are waited on is determined automatically from which
    :class:`ArmTarget` slots are populated in each stage — no need for
    ``wait_both_arms`` / ``single_arm_part`` parameters.

    After each stage, if :attr:`StageTarget.wait_gripper_settle` is True, calls
    ``sleep_fn(gripper_action_wait)`` so the gripper can finish closing/opening
    (set by built-in pick/place/handover/carry builders; custom stages may set it explicitly).

    ``pose_tol_pos`` / ``pose_tol_ori``（米 / 无量纲姿态距离，与 ``QueueSliceCommon`` 一致）若给出，
    则传给 ``ROS2RobotInterface.wait_until_arrive`` 用于左右臂笛卡尔到位判定；否则使用接口默认阈值。
    ``pose_tol_ori`` 会换算成度再交给 handler（见 :func:`pose_tol_ori_to_orient_deg`）。
    """
    arm_orient_threshold = pose_tol_ori_to_orient_deg(pose_tol_ori)
    for stage in sequence:
        logger.info("[Stage] %s", stage.name)
        if on_stage_start is not None:
            on_stage_start(stage.name, stage)

        effective_frame_id = stage.frame_id if stage.frame_id is not None else frame_id
        _send_arm_targets(interface, stage, send_mode, effective_frame_id)
        _send_gripper_commands(interface, stage, gripper_mode)

        arrive_results: dict[str, dict[str, Any]] = {}

        if not stage.skip_arrival_check:
            if stage.left:
                arrive_results["left_arm"] = interface.wait_until_arrive(
                    part="left_arm",
                    timeout=arrival_timeout,
                    poll_period=arrival_poll,
                    time_now_fn=time_now_fn,
                    sleep_fn=sleep_fn,
                    arm_pose_threshold=pose_tol_pos,
                    arm_orient_threshold=arm_orient_threshold,
                    on_poll=(
                        (lambda _r, _e, _sn=stage.name: on_stage_poll(_sn, "left_arm", _r, _e))
                        if on_stage_poll is not None
                        else None
                    ),
                )
            if stage.right:
                arrive_results["right_arm"] = interface.wait_until_arrive(
                    part="right_arm",
                    timeout=arrival_timeout,
                    poll_period=arrival_poll,
                    time_now_fn=time_now_fn,
                    sleep_fn=sleep_fn,
                    arm_pose_threshold=pose_tol_pos,
                    arm_orient_threshold=arm_orient_threshold,
                    on_poll=(
                        (lambda _r, _e, _sn=stage.name: on_stage_poll(_sn, "right_arm", _r, _e))
                        if on_stage_poll is not None
                        else None
                    ),
                )

        if stage.wait_gripper_settle:
            sleep_fn(gripper_action_wait)

        not_arrived = {
            part: r for part, r in arrive_results.items()
            if not r.get("arrived", False)
        }
        if not_arrived:
            parts: list[str] = []
            for part, r in not_arrived.items():
                inner = r.get("result") or {}
                pos_d = inner.get("position_distance")
                ori_d = inner.get("orientation_distance")
                elapsed = r.get("elapsed")
                if isinstance(pos_d, float) and isinstance(ori_d, float):
                    parts.append(
                        f"{part}(pos_dist={pos_d:.4f}, ori_dist={ori_d:.4f}, elapsed={elapsed:.2f}s)"
                    )
                else:
                    parts.append(f"{part}(no_feedback, elapsed={elapsed:.2f}s)" if elapsed else f"{part}(no_feedback)")
            logger.warning("%s [%s]: %s", warn_prefix, stage.name, ", ".join(parts))

        left_arrive = arrive_results.get("left_arm", {"arrived": True})
        right_arrive = arrive_results.get("right_arm", {"arrived": True})

        if (
            left_arrival_guard_stage
            and stage.name == left_arrival_guard_stage
            and not left_arrive.get("arrived", False)
        ):
            raise RuntimeError(
                "Left arm did not arrive at guarded stage; skip subsequent grasp."
            )
        if on_stage_end is not None:
            on_stage_end(stage.name, left_arrive, right_arrive)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

__all__ = [
    "ArmSide",
    "ArmStage",
    "ArmTarget",
    "DirectionVec",
    "GripperMode",
    "SendMode",
    "StageTarget",
    "BIMANUAL_PLACE_SUFFIXES",
    "CARRY_STAGE_SUFFIXES",
    "carry_prepare_offset_is_active",
    "HANDOVER_STAGE_SUFFIXES",
    "PICK_STAGE_SUFFIXES",
    "PLACE_STAGE_SUFFIXES",
    "assign_to_arm",
    "build_bimanual_carry_sequence",
    "build_bimanual_place_sequence",
    "build_handover_sequence",
    "build_single_arm_pick_sequence",
    "build_single_arm_place_sequence",
    "build_single_arm_return_home_sequence",
    "compose_bimanual_synchronized_sequence",
    "execute_stage_sequence",
    "pose_tol_ori_to_orient_deg",
]
