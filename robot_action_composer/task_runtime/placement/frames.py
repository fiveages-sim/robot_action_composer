"""End-effector frame resolution and eef↔tcp conversion helpers."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from geometry_msgs.msg import Pose
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmTarget,
    StageTarget,
)

from .geometry import pose_quat_xyzw


def pick_tf_timeout(raw: Any) -> float:
    if raw is None:
        return 2.0
    try:
        return float(raw)
    except (TypeError, ValueError):
        return 2.0


def expand_arm_frame(name: str, arm: str) -> str:
    return name.replace("{arm}", arm).strip()


def explicit_frame_name(raw: Any) -> str | None:
    """``None`` 表示未写；空字符串表示显式关掉。"""
    if raw is None:
        return None
    return str(raw).strip()


def resolve_ee_frames(
    ctx: Any,
    arm: str,
    ee_target_frame: Any,
    ee_command_frame: Any,
) -> tuple[str, str] | None:
    """技能参数优先，否则用 ``robot.yaml`` 的默认末端 frame。无需换算时返回 ``None``。"""
    target = explicit_frame_name(ee_target_frame)
    if target is None:
        target = str(getattr(ctx.robot_cfg, "ee_target_frame", "") or "").strip()
    if not target:
        return None
    command = explicit_frame_name(ee_command_frame)
    if command is None:
        command = str(getattr(ctx.robot_cfg, "ee_command_frame", "") or "").strip() or None
    command_frame = expand_arm_frame(command or "{arm}_tcp", arm)
    target_frame = expand_arm_frame(target, arm)
    if target_frame == command_frame:
        return None
    return target_frame, command_frame


def stages_in_command_ee_frame(
    ctx: Any,
    stages: list[StageTarget],
    *,
    arm_side: ArmSide,
    ee_target_frame: Any,
    ee_command_frame: Any,
    tf_lookup_timeout: Any,
    label: str,
    arm_sides: tuple[ArmSide, ...] | None = None,
) -> list[StageTarget]:
    """把「目标末端 link 的位姿」换成运控跟踪 link 的位姿。

    技能未写 ``ee_target_frame`` 时用机器人默认值；机器人和技能都空则原样下发。
    ``arm_sides`` 可一次处理多臂（默认只用 ``arm_side``）。
    """
    sides = arm_sides if arm_sides else (arm_side,)
    convert = getattr(ctx.interface, "convert_ee_frame_pose", None)
    timeout = pick_tf_timeout(tf_lookup_timeout)
    rewritten = list(stages)
    for side in sides:
        resolved = resolve_ee_frames(ctx, side.value, ee_target_frame, ee_command_frame)
        if resolved is None:
            continue
        target_frame, command_frame = resolved
        if convert is None:
            raise RuntimeError(
                f"{label}: ee_target_frame={target_frame!r} requires "
                "ROS2RobotInterface.convert_ee_frame_pose"
            )
        side_attr = "right" if side is ArmSide.RIGHT else "left"
        updated: list[StageTarget] = []
        converted = 0
        for stage in rewritten:
            arm_target = getattr(stage, side_attr)
            if arm_target is None:
                updated.append(stage)
                continue
            command_pose = convert(
                arm_target.pose,
                target_frame,
                command_frame,
                timeout=timeout,
            )
            new_arm = ArmTarget(pose=command_pose, gripper=float(arm_target.gripper))
            converted += 1
            if side_attr == "right":
                updated.append(replace(stage, right=new_arm))
            else:
                updated.append(replace(stage, left=new_arm))
        rewritten = updated
        print(
            f"[{label}] arm={side.value} ee_target_frame={target_frame} -> "
            f"command_frame={command_frame} (live TF, {converted} stages)"
        )
    return rewritten


def feedback_pose_as_eef(
    ctx: Any,
    tcp_pose: Pose,
    arm_side: ArmSide,
    timeout: float,
) -> Pose | None:
    """运控反馈按 tcp 解；若配置了 eef/tcp，用 live TF 得到夹指系位姿。无配置则 ``None``。"""
    resolved = resolve_ee_frames(ctx, arm_side.value, None, None)
    if resolved is None:
        return None
    target_frame, command_frame = resolved
    lookup = getattr(ctx.interface, "lookup_transform", None)
    if lookup is None:
        raise RuntimeError("placement: held eef/tcp diagnostics need lookup_transform")
    eef_in_tcp = lookup(command_frame, target_frame, timeout=timeout)
    if eef_in_tcp is None:
        raise RuntimeError(
            f"placement: TF {command_frame!r} -> {target_frame!r} failed"
        )
    translation = eef_in_tcp.transform.translation
    rotation = eef_in_tcp.transform.rotation
    tcp_q = pose_quat_xyzw(tcp_pose)
    offset = rotate_vector_by_quat(
        (translation.x, translation.y, translation.z),
        tcp_q,
    )
    eef_q = quat_normalize(
        quat_multiply(tcp_q, (rotation.x, rotation.y, rotation.z, rotation.w))
    )
    out = Pose()
    out.position.x = float(tcp_pose.position.x) + offset[0]
    out.position.y = float(tcp_pose.position.y) + offset[1]
    out.position.z = float(tcp_pose.position.z) + offset[2]
    out.orientation.x = eef_q[0]
    out.orientation.y = eef_q[1]
    out.orientation.z = eef_q[2]
    out.orientation.w = eef_q[3]
    return out
