#!/usr/bin/env python3
"""Bimanual handover: sync-segment config + Cartesian builder for task-queue execution."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    StageTarget,
    build_handover_sequence,
)


def _pose_from_tuple(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


@dataclass(frozen=True)
class HandoverSyncConfig:
    """仅双臂同步交接段几何（``build_handover_sequence``）。

    抓取 / 放置 / 环境重置 / 臂别等由 ``QueueSingleArmSlice`` 与 ``skill_defaults.single_arm.*`` 提供，
    勿在此重复 ``QueueSlicePick`` / ``QueueSlicePlace`` 已有字段。
    """

    handover_position: tuple[float, float, float]
    source_handover_orientation: tuple[float, float, float, float]
    receiver_handover_orientation: tuple[float, float, float, float]
    receiver_handover_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


def format_handover_sync_summary(scene: str, sync: HandoverSyncConfig) -> str:
    return (
        f"[Handover sync] {scene} pos={sync.handover_position}, "
        f"src_ori={sync.source_handover_orientation}, rcv_ori={sync.receiver_handover_orientation}, "
        f"rcv_off={sync.receiver_handover_offset}"
    )


def build_handover_sync_sequence(
    *,
    sync_cfg: HandoverSyncConfig,
    source_is_right: bool,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Handover",
) -> list[StageTarget]:
    """双臂同步交接段（``build_handover_sequence``），不含单臂 pick / place。

    与 ``single_arm.pick`` + ``dual_arm.handover_sync`` + ``single_arm.place`` 组合使用；
    ``source_is_right`` 须与 ``QueueSliceCommon.arm``（抓取侧）一致。
    """
    source_arm = ArmSide.RIGHT if source_is_right else ArmSide.LEFT
    source_handover_pose = _pose_from_tuple(
        sync_cfg.handover_position,
        sync_cfg.source_handover_orientation,
    )
    receiver_handover_pose = _pose_from_tuple(
        sync_cfg.handover_position,
        sync_cfg.receiver_handover_orientation,
    )
    rx, ry, rz = sync_cfg.receiver_handover_offset
    receiver_handover_pose.position.x += rx
    receiver_handover_pose.position.y += ry
    receiver_handover_pose.position.z += rz
    return build_handover_sequence(
        source_handover_pose=source_handover_pose,
        receiver_handover_pose=receiver_handover_pose,
        source_arm=source_arm,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        stage_prefix=stage_prefix,
    )
