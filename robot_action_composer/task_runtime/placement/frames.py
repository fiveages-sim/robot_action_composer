"""End-effector frame resolution and eef↔tcp conversion helpers."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from geometry_msgs.msg import Pose
from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmSide,
    ArmTarget,
    StageTarget,
)
from robot_action_composer.task_runtime.object_binding import (  # pyright: ignore[reportMissingImports]
    resolve_prim_relative_pose,
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


def resolve_ee_tip_pair(ctx: Any, arm: str) -> Any | None:
    """Return ``EeTipPrimPair`` for ``arm`` from ``robot.yaml`` ``ee_tip``, or ``None``."""
    tip_cfg = getattr(ctx.robot_cfg, "ee_tip", None)
    if not tip_cfg:
        return None
    side = str(arm or "").strip().lower()
    if side not in {"left", "right"}:
        return None
    pair = tip_cfg.get(side) if isinstance(tip_cfg, dict) else None
    if pair is None:
        return None
    parent = str(getattr(pair, "parent_prim", "") or "").strip()
    tip = str(getattr(pair, "tip_prim", "") or "").strip()
    if not parent or not tip:
        return None
    return pair


def resolve_ee_command_frame_name(ctx: Any, arm: str, ee_command_frame: Any) -> str | None:
    """Resolved OCS2 / command TF name, or ``None`` if unset."""
    command = explicit_frame_name(ee_command_frame)
    if command is None:
        command = str(getattr(ctx.robot_cfg, "ee_command_frame", "") or "").strip() or None
    if not command:
        return None
    return expand_arm_frame(command, arm)


def resolve_ee_tip_parent_frame(ctx: Any, arm: str) -> str | None:
    """TF name of ``ee_tip`` parent after USD remap (default ``{arm}_tcp``)."""
    pair = resolve_ee_tip_pair(ctx, arm)
    if pair is None:
        return None
    raw = str(getattr(pair, "parent_frame", "") or "").strip() or "{arm}_tcp"
    return expand_arm_frame(raw, arm)


def resolve_ee_frames(
    ctx: Any,
    arm: str,
    ee_target_frame: Any,
    ee_command_frame: Any,
) -> tuple[str, str] | None:
    """技能参数优先，否则用 ``robot.yaml`` 的默认末端 frame。无需换算时返回 ``None``。

    仅用于无 ``ee_tip`` 的 TF 路径。有 ``ee_tip`` 时由
    :func:`resolve_ee_tip_to_command_frames` 做 parent→command 第二跳。
    """
    if resolve_ee_tip_pair(ctx, arm) is not None:
        return None
    target = explicit_frame_name(ee_target_frame)
    if target is None:
        target = str(getattr(ctx.robot_cfg, "ee_target_frame", "") or "").strip()
    if not target:
        return None
    command = resolve_ee_command_frame_name(ctx, arm, ee_command_frame)
    command_frame = command or expand_arm_frame("{arm}_tcp", arm)
    target_frame = expand_arm_frame(target, arm)
    if target_frame == command_frame:
        return None
    return target_frame, command_frame


def resolve_ee_tip_to_command_frames(
    ctx: Any,
    arm: str,
    ee_command_frame: Any,
) -> tuple[str, str] | None:
    """After USD tip→parent: live TF ``parent_frame`` → OCS2 ``ee_command_frame``."""
    parent_frame = resolve_ee_tip_parent_frame(ctx, arm)
    if parent_frame is None:
        return None
    command_frame = resolve_ee_command_frame_name(ctx, arm, ee_command_frame)
    if not command_frame or command_frame == parent_frame:
        return None
    return parent_frame, command_frame


def _invert_parent_tip_pose(
    tip_in_parent_xyz: tuple[float, float, float],
    tip_in_parent_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """``T_parent_tip`` → ``T_tip_parent`` (command-in-tip), matching TF ``lookup(tip, parent)``."""
    q = quat_normalize(tip_in_parent_xyzw)
    q_inv = quat_conjugate(q)
    t_inv = rotate_vector_by_quat(
        (-tip_in_parent_xyz[0], -tip_in_parent_xyz[1], -tip_in_parent_xyz[2]),
        q_inv,
    )
    return t_inv, q_inv


def convert_tip_pose_to_parent_usd(
    tip_pose_in_ref: Pose,
    *,
    tip_in_parent_xyz: tuple[float, float, float],
    tip_in_parent_xyzw: tuple[float, float, float, float],
) -> Pose:
    """Same math as ``convert_ee_frame_pose`` with static ``T_parent_tip`` from USD."""
    parent_in_tip_t, parent_in_tip_q = _invert_parent_tip_pose(
        tip_in_parent_xyz, tip_in_parent_xyzw
    )
    tip_q = pose_quat_xyzw(tip_pose_in_ref)
    offset_in_ref = rotate_vector_by_quat(parent_in_tip_t, tip_q)
    parent_q = quat_normalize(quat_multiply(tip_q, parent_in_tip_q))
    out = Pose()
    out.position.x = float(tip_pose_in_ref.position.x) + offset_in_ref[0]
    out.position.y = float(tip_pose_in_ref.position.y) + offset_in_ref[1]
    out.position.z = float(tip_pose_in_ref.position.z) + offset_in_ref[2]
    out.orientation.x = parent_q[0]
    out.orientation.y = parent_q[1]
    out.orientation.z = parent_q[2]
    out.orientation.w = parent_q[3]
    return out


def stages_in_command_ee_frame_from_usd(
    ctx: Any,
    stages: list[StageTarget],
    *,
    arm_side: ArmSide,
    label: str,
    arm_sides: tuple[ArmSide, ...] | None = None,
) -> list[StageTarget]:
    """Convert tip-frame stage poses to parent/tcp using ``robot.yaml`` ``ee_tip`` USD chain."""
    sides = arm_sides if arm_sides else (arm_side,)
    cache = getattr(ctx, "ee_tip_pose_cache", None)
    if cache is None:
        cache = {}
        try:
            ctx.ee_tip_pose_cache = cache
        except Exception:
            pass
    rewritten = list(stages)
    for side in sides:
        pair = resolve_ee_tip_pair(ctx, side.value)
        if pair is None:
            continue
        tip_xyz, tip_xyzw = resolve_prim_relative_pose(
            pair.parent_prim,
            pair.tip_prim,
            cache=cache,
        )
        side_attr = "right" if side is ArmSide.RIGHT else "left"
        updated: list[StageTarget] = []
        converted = 0
        logged = False
        for stage in rewritten:
            arm_target = getattr(stage, side_attr)
            if arm_target is None:
                updated.append(stage)
                continue
            tip_pose = arm_target.pose
            command_pose = convert_tip_pose_to_parent_usd(
                tip_pose,
                tip_in_parent_xyz=tip_xyz,
                tip_in_parent_xyzw=tip_xyzw,
            )
            if not logged:
                tq = pose_quat_xyzw(tip_pose)
                pq = pose_quat_xyzw(command_pose)
                print(
                    f"[{label}] arm={side.value} tip→parent sample: "
                    f"tip_xyzw=({tq[0]:.4f},{tq[1]:.4f},{tq[2]:.4f},{tq[3]:.4f}) "
                    f"parent_xyzw=({pq[0]:.4f},{pq[1]:.4f},{pq[2]:.4f},{pq[3]:.4f}) "
                    f"T_parent_tip_xyzw=({tip_xyzw[0]:.4f},{tip_xyzw[1]:.4f},"
                    f"{tip_xyzw[2]:.4f},{tip_xyzw[3]:.4f})"
                )
                logged = True
            new_arm = ArmTarget(pose=command_pose, gripper=float(arm_target.gripper))
            converted += 1
            if side_attr == "right":
                updated.append(replace(stage, right=new_arm))
            else:
                updated.append(replace(stage, left=new_arm))
        rewritten = updated
        print(
            f"[{label}] arm={side.value} ee_tip tip={pair.tip_prim} -> "
            f"parent={pair.parent_prim} (USD local chain, {converted} stages)"
        )
    return rewritten


def _rewrite_arm_stages_with_convert(
    stages: list[StageTarget],
    *,
    side: ArmSide,
    convert: Any,
    source_frame: str,
    command_frame: str,
    timeout: float,
) -> tuple[list[StageTarget], int]:
    side_attr = "right" if side is ArmSide.RIGHT else "left"
    updated: list[StageTarget] = []
    converted = 0
    for stage in stages:
        arm_target = getattr(stage, side_attr)
        if arm_target is None:
            updated.append(stage)
            continue
        command_pose = convert(
            arm_target.pose,
            source_frame,
            command_frame,
            timeout=timeout,
        )
        new_arm = ArmTarget(pose=command_pose, gripper=float(arm_target.gripper))
        converted += 1
        if side_attr == "right":
            updated.append(replace(stage, right=new_arm))
        else:
            updated.append(replace(stage, left=new_arm))
    return updated, converted


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

    有 ``ee_tip``：USD tip→parent，再用 live TF ``parent_frame``→``ee_command_frame``
    （W2 OCS2 多为 ``{arm}_eef``，不是 tcp）。无 ``ee_tip``：``ee_target_frame``→
    ``ee_command_frame`` live TF。都空则原样下发。
    """
    sides = arm_sides if arm_sides else (arm_side,)
    usd_sides = tuple(s for s in sides if resolve_ee_tip_pair(ctx, s.value) is not None)
    rewritten = list(stages)
    if usd_sides:
        rewritten = stages_in_command_ee_frame_from_usd(
            ctx,
            rewritten,
            arm_side=arm_side,
            label=label,
            arm_sides=usd_sides,
        )

    convert = getattr(ctx.interface, "convert_ee_frame_pose", None)
    timeout = pick_tf_timeout(tf_lookup_timeout)
    for side in sides:
        if side in usd_sides:
            resolved = resolve_ee_tip_to_command_frames(
                ctx, side.value, ee_command_frame
            )
            hop_label = "ee_tip.parent_frame"
        else:
            resolved = resolve_ee_frames(
                ctx, side.value, ee_target_frame, ee_command_frame
            )
            hop_label = "ee_target_frame"
        if resolved is None:
            continue
        source_frame, command_frame = resolved
        if convert is None:
            raise RuntimeError(
                f"{label}: {hop_label}={source_frame!r} -> {command_frame!r} requires "
                "ROS2RobotInterface.convert_ee_frame_pose"
            )
        rewritten, converted = _rewrite_arm_stages_with_convert(
            rewritten,
            side=side,
            convert=convert,
            source_frame=source_frame,
            command_frame=command_frame,
            timeout=timeout,
        )
        print(
            f"[{label}] arm={side.value} {hop_label}={source_frame} -> "
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
