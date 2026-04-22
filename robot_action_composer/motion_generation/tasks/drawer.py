#!/usr/bin/env python3
"""Drawer geometry config and Cartesian stage helpers for ``single_arm.drawer.*`` queue skills.

Drawer Cartesian 序列只读 :class:`DrawerGeometryConfig`，
**不**再混入 :class:`~robot_action_composer.task_runtime.config.single_arm.QueueSlicePick` 的
``object_position_offset``；拉开末段后的松爪与工具系撤出统一由 ``ee_retreat_offset`` 表达。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ros2_robot_interface.utils.quat_pose import rotate_vector_by_quat  # pyright: ignore[reportMissingImports]

from geometry_msgs.msg import Pose  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    PLACE_STAGE_SUFFIXES,
    ArmSide,
    ArmStage,
    ArmTarget,
    StageTarget,
    assign_to_arm,
    build_single_arm_pick_sequence,
)
from robot_action_composer.task_runtime.config.single_arm import (  # pyright: ignore[reportMissingImports]
    QueueSingleArmSlice,
)


@dataclass(frozen=True)
class DrawerGeometryConfig:
    """Drawer cabinet / handle prim paths (structured overlays → ``MergedQueueConfig.drawer``)."""

    object_prim_path: str = ""
    #: 拉手参考点在 **抽屉 Prim 局部坐标** 下的位移（米）。通常取包围盒中心 ``(handle_extent_max+handle_extent_min)/2*drawer_scale`` 各轴，离线算好写入，避免运行时再从 min/max 推导。
    object_position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: 拉手闭合段沿工具系 +Z 的位移（米），语义同 ``pick_clearance``；写入 ``skill_defaults.single_arm.drawer``。
    drawer_clearance: float = 0.01
    #: 拉开抽屉时沿拉手拉出方向末段行程（米），语义同 pick 的 ``retreat_direction_extra``；在 ``skill_defaults.single_arm.drawer`` 中配置。
    pull_distance: float = 0.18
    #: 拉手预接近（工具系 ``[x,y,z]``，语义与 ``single_arm.pick.prepare_offset`` 相同，但**仅**用于 ``pull_open`` / ``close_push``）。省略或 ``None`` 时不生成预接近段（**不**读取 ``pick.prepare_offset``）。
    prepare_offset: tuple[float, float, float] | None = None
    #: **工具系**松爪后平移（经 ``pull_open`` / ``close_push`` 各自阶段的末端姿态旋入运动系）。
    #: - ``pull_open``：拉开 Cartesian 末段结束后，生成 **Release + PostReleaseRetreat**（与 ``place`` 序列同构；**只**读本键）。
    #: - ``close_push``：内联到位到 ``place_pose_ref``（夹爪仍闭合）后，先松爪再按该向量平移（与 ``single_arm.place.ee_retreat_offset`` 语义相同；**不**读 ``place`` 切片）。
    #: ``None`` 或全 0：不追加 / 不执行撤出段。
    ee_retreat_offset: tuple[float, float, float] | None = None


def _effective_drawer_prepare_offset(drawer: DrawerGeometryConfig) -> tuple[float, float, float]:
    if drawer.prepare_offset is not None:
        return drawer.prepare_offset
    return (0.0, 0.0, 0.0)


def format_drawer_task_cfg_summary(scene: str, drawer: DrawerGeometryConfig, single: QueueSingleArmSlice) -> str:
    p, c = single.pick, single.common
    return (
        f"[Drawer] scene={scene} object={p.object_prim_path} "
        f"object_prim_path={drawer.object_prim_path} | "
        f"arm={c.arm}, ee_pick_axis={p.ee_pick_axis}, pull_distance={drawer.pull_distance}"
    )


def _apply_target_pose_offset(pose: Any, offset: tuple[float, float, float]) -> Any:
    ox, oy, oz = offset
    pose.position.x += ox
    pose.position.y += oy
    pose.position.z += oz
    return pose


def _clone_pose(src: Pose) -> Pose:
    dst = Pose()
    dst.position.x = float(src.position.x)
    dst.position.y = float(src.position.y)
    dst.position.z = float(src.position.z)
    dst.orientation.x = float(src.orientation.x)
    dst.orientation.y = float(src.orientation.y)
    dst.orientation.z = float(src.orientation.z)
    dst.orientation.w = float(src.orientation.w)
    return dst


def _append_pull_open_release_retreat_stages(
    arm_seq: list[ArmStage],
    *,
    drawer: DrawerGeometryConfig,
    ee_base_orientation: tuple[float, float, float, float],
    gripper_open: float,
    stage_prefix: str,
) -> list[ArmStage]:
    rt = drawer.ee_retreat_offset
    if rt is None or not any(abs(float(x)) > 1e-12 for x in rt):
        return arm_seq
    ox, oy, oz = rotate_vector_by_quat(rt, ee_base_orientation)
    last_pose = arm_seq[-1].target.pose
    retract = _clone_pose(last_pose)
    retract.position.x += ox
    retract.position.y += oy
    retract.position.z += oz
    idx = len(arm_seq) + 1
    release = _clone_pose(last_pose)
    out = list(arm_seq)
    out.append(
        ArmStage(
            f"{stage_prefix}-{idx}-{PLACE_STAGE_SUFFIXES[1]}",
            ArmTarget(pose=release, gripper=gripper_open),
            wait_gripper_settle=True,
        )
    )
    idx += 1
    out.append(
        ArmStage(
            f"{stage_prefix}-{idx}-{PLACE_STAGE_SUFFIXES[2]}",
            ArmTarget(pose=retract, gripper=gripper_open),
        )
    )
    return out


def build_single_arm_pull_drawer_sequence(
    *,
    target_pose: Any,
    drawer: DrawerGeometryConfig,
    arm_side: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    ee_base_orientation: tuple[float, float, float, float],
    pull_direction_xyz: tuple[float, float, float],
    pull_distance: float,
) -> list[StageTarget]:
    prep = _effective_drawer_prepare_offset(drawer)
    stage_prefix = "PickPlaceFlow"
    # 拉手已在技能层用 ``object_position_offset``（Prim 局部）旋入 ``target_pose``；此处勿再叠加。
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            ee_base_orientation=ee_base_orientation,
            prepare_offset=prep,
            pick_clearance=drawer.drawer_clearance,
            ee_pick_direction_vector=pull_direction_xyz,
            object_position_offset=(0.0, 0.0, 0.0),
            retreat_direction_extra=pull_distance,
            retreat_offset=(0.0, 0.0, 0.0),
            retreat_xyz=None,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix=stage_prefix,
        )
    )
    arm_seq = _append_pull_open_release_retreat_stages(
        arm_seq,
        drawer=drawer,
        ee_base_orientation=ee_base_orientation,
        gripper_open=gripper_open,
        stage_prefix=stage_prefix,
    )
    return assign_to_arm(arm_seq, arm_side)


def build_single_arm_close_drawer_sequence(
    *,
    target_pose: Any,
    drawer: DrawerGeometryConfig,
    arm_side: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    ee_base_orientation: tuple[float, float, float, float],
    pull_direction_xyz: tuple[float, float, float],
) -> list[StageTarget]:
    prep = _effective_drawer_prepare_offset(drawer)
    # 同上：``target_pose`` 已含 Prim 局部 ``object_position_offset`` 的旋转叠加。
    arm_seq: list[ArmStage] = list(
        build_single_arm_pick_sequence(
            target_pose=target_pose,
            ee_base_orientation=ee_base_orientation,
            prepare_offset=prep,
            pick_clearance=drawer.drawer_clearance,
            ee_pick_direction_vector=pull_direction_xyz,
            object_position_offset=(0.0, 0.0, 0.0),
            retreat_direction_extra=0,
            retreat_offset=(0.0, 0.0, 0.0),
            retreat_xyz=None,
            gripper_open=gripper_open,
            gripper_closed=gripper_closed,
            stage_prefix="PickPlaceFlow",
        )
    )
    return assign_to_arm(arm_seq, arm_side)
