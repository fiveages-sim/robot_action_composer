"""Dual-arm queue skills (``dual_arm.*``): carry + handover sync + cache goto."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Pose

from ros2_robot_interface.utils.quat_pose import (  # pyright: ignore[reportMissingImports]
    euler_rpy_to_quat_xyzw,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat,
)

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmTarget,
    SendMode,
    StageTarget,
    build_bimanual_place_relative_sequence,
)

from robot_action_composer.isaac_sim import (  # pyright: ignore[reportMissingImports]
    SERVICE_CALL_RETRIES,
    SERVICE_CALL_TIMEOUT,
    SERVICE_RETRY_DELAY,
    get_object_pose_from_service,
)

from robot_action_composer.motion_generation.tasks.bimanual_carry import (  # pyright: ignore[reportMissingImports]
    BimanualCarryTaskConfig,
    build_bimanual_carry_record_sequence,
)
from robot_action_composer.motion_generation.tasks.bimanual_parallel_pick import (  # pyright: ignore[reportMissingImports]
    BimanualParallelPickTaskConfig,
    build_bimanual_parallel_pick_record_sequence,
    parallel_pick_cfg_from_params,
)
from robot_action_composer.motion_generation.tasks.handover import build_handover_sync_sequence  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.pick_place import (  # pyright: ignore[reportMissingImports]
    apply_object_local_offset_to_pose,
)

from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.object_resolution_replay import resolve_object_pose_for_task
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.skills.single_arm import (
    _is_bimanual_ee_cache,
    _pick_execution_frame_id,
    _pick_tf_timeout,
    _scratch_pose_entry_to_geometry_pose,
)
from robot_action_composer.task_runtime.types import ExecutionMeta


def _dual_mode(ctx: QueueRuntimeContext) -> SendMode:
    return SendMode.DUAL_ARM_STAMPED if ctx.use_stamped else SendMode.UNSTAMPED


def _require_carry(ctx: QueueRuntimeContext) -> BimanualCarryTaskConfig:
    c = ctx.carry_task_cfg
    if c is None:
        raise TypeError("dual_arm.carry* skills require carry_task_cfg (bimanual / carry fields in YAML)")
    return c


# 与运控约定一致（ROS 动态参数名）
_ROS_ARM_MOVEL_DURATION_PARAM = "movel_duration"


def _apply_arm_movel_duration(
    ctx: QueueRuntimeContext,
    *,
    duration: Any,
    label: str,
) -> None:
    """在双臂笛卡尔段执行前写 ``arm_controller`` 的 ``movel_duration``（类比 ``body_movej_duration``）。"""
    if duration is None:
        return
    try:
        d = float(duration)
    except (TypeError, ValueError):
        print(f"[{label}] WARN: arm_movel_duration must be numeric, got {duration!r}")
        return
    iface = ctx.interface
    node = str(getattr(iface, "arm_controller", "") or "").strip()
    if not node:
        print(
            f"[{label}] WARN: arm_movel_duration={d} but arm_controller is empty; "
            "configure unified_arm_joint_controller_topic / left_arm_joint_controller_topic on ROS2RobotInterface.",
        )
        return
    p = _ROS_ARM_MOVEL_DURATION_PARAM
    ok = iface.set_node_parameters(full_node_name=node, parameters={p: d})
    if ok:
        print(f"[{label}] Set {node}.{p} = {d}")
    else:
        print(f"[{label}] WARN: failed to set {node}.{p} = {d}")


def _apply_arm_movel_duration_from_carry_cfg(
    ctx: QueueRuntimeContext, cfg: BimanualCarryTaskConfig, *, label: str,
) -> None:
    _apply_arm_movel_duration(ctx, duration=cfg.arm_movel_duration, label=label)


def _ee_pose_source_frame(lh: Any, rh: Any, ctx_frame_id: str, *, label: str) -> str:
    """推断 ``get_pose()`` 数值所在的坐标系（优先用订阅 ``PoseStamped.header.frame_id``）。"""

    def _fid(h: Any) -> str | None:
        gf = getattr(h, "get_frame_id", None)
        if not callable(gf):
            return None
        raw = gf()
        if raw is None:
            return None
        s = str(raw).strip()
        return s or None

    lf, rf = _fid(lh), _fid(rh)
    fallback = str(ctx_frame_id).strip() or "base_link"
    if lf and rf and lf != rf:
        raise RuntimeError(f"{label}: left EE frame_id {lf!r} != right EE frame_id {rf!r}")
    if lf:
        return lf
    if rf:
        return rf
    return fallback


def _motion_frame_poses_from_params(
    iface: Any,
    l0_raw: Pose,
    r0_raw: Pose,
    pose_frame: str,
    params: Mapping[str, Any],
    *,
    label: str,
) -> tuple[Pose, Pose, str]:
    """将当前左右末端位姿变到 ``motion_frame_id``（与 ``pose_frame`` 相同时直接拷贝）。"""
    raw_mf = params.get("motion_frame_id", params.get("relative_frame_id"))
    if raw_mf is None or (isinstance(raw_mf, str) and not str(raw_mf).strip()):
        motion_frame = pose_frame
    else:
        motion_frame = str(raw_mf).strip()

    try:
        tft = float(params.get("tf_lookup_timeout", 2.0))
    except (TypeError, ValueError):
        tft = 2.0

    if motion_frame == pose_frame:
        return _clone_pose(l0_raw), _clone_pose(r0_raw), motion_frame
    if not hasattr(iface, "transform_pose"):
        raise TypeError(f"{label}: motion_frame_id requires ROS2RobotInterface.transform_pose (TF buffer)")
    l0 = iface.transform_pose(l0_raw, pose_frame, motion_frame, timeout=tft)
    r0 = iface.transform_pose(r0_raw, pose_frame, motion_frame, timeout=tft)
    if l0 is None or r0 is None:
        raise RuntimeError(
            f"{label}: TF {pose_frame!r} -> {motion_frame!r} failed (timeout={tft}s). "
            "Check frames and that the interface is connected with TF listener."
        )
    return l0, r0, motion_frame


def _clone_pose(p: Pose) -> Pose:
    q = Pose()
    q.position.x = p.position.x
    q.position.y = p.position.y
    q.position.z = p.position.z
    q.orientation.x = p.orientation.x
    q.orientation.y = p.orientation.y
    q.orientation.z = p.orientation.z
    q.orientation.w = p.orientation.w
    return q


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


def _pose_quat_xyzw(p: Pose) -> tuple[float, float, float, float]:
    return (
        float(p.orientation.x),
        float(p.orientation.y),
        float(p.orientation.z),
        float(p.orientation.w),
    )


def _set_pose_quat_xyzw(p: Pose, q: tuple[float, float, float, float]) -> None:
    x, y, z, w = q
    p.orientation.x = x
    p.orientation.y = y
    p.orientation.z = z
    p.orientation.w = w


def _carry_base_frame_id(ctx: QueueRuntimeContext) -> str:
    """物体解算 ``get_object_pose_from_service`` 所用系，与 ``ctx.frame_id`` 一致。"""
    s = str(ctx.frame_id).strip()
    return s or "base_link"


def _carry_execution_frame_id(cfg: BimanualCarryTaskConfig, ctx: QueueRuntimeContext) -> str:
    raw = getattr(cfg, "motion_frame_id", None)
    if raw is None or (isinstance(raw, str) and not str(raw).strip()):
        return _carry_base_frame_id(ctx)
    return str(raw).strip()


def _carry_tf_timeout(cfg: BimanualCarryTaskConfig) -> float:
    t = getattr(cfg, "tf_lookup_timeout", None)
    if t is None:
        return 2.0
    try:
        return float(t)
    except (TypeError, ValueError):
        return 2.0


def _object_position_in_carry_execution_frame(
    ctx: QueueRuntimeContext,
    cfg: BimanualCarryTaskConfig,
    object_position_base: Pose,
) -> Pose:
    """Isaac 物体位姿在 ``ctx.frame_id`` 下；若 ``motion_frame_id`` 不同则 TF 到目标系（与 ``place_relative`` 语义一致）。"""
    base_f = _carry_base_frame_id(ctx)
    exec_f = _carry_execution_frame_id(cfg, ctx)
    if exec_f == base_f:
        return object_position_base
    iface = ctx.interface
    if not hasattr(iface, "transform_pose"):
        raise TypeError("dual_arm.carry: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)")
    out = iface.transform_pose(
        object_position_base,
        base_f,
        exec_f,
        timeout=_carry_tf_timeout(cfg),
    )
    if out is None:
        raise RuntimeError(
            f"dual_arm.carry: TF {base_f!r} -> {exec_f!r} failed for object pose; check motion_frame_id and TF.",
        )
    return out


def _carry_full_stages(
    ctx: QueueRuntimeContext,
    cfg: BimanualCarryTaskConfig,
    object_position: Any,
    *,
    output_frame_id: str,
) -> list[StageTarget]:
    return build_bimanual_carry_record_sequence(
        carry_task_cfg=cfg,
        object_position=object_position,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        output_frame_id=output_frame_id,
    )


def _require_handover_sync(ctx: QueueRuntimeContext):
    h = ctx.handover_sync
    if h is None:
        raise TypeError(
            "dual_arm.handover requires handover_sync (merged from skill_defaults.dual_arm.handover; "
            "need handover_position + orientations). Pick side arm comes from common.arm / single_arm.pick."
        )
    return h


def _require_parallel_pick_cfg(params: Mapping[str, Any]) -> BimanualParallelPickTaskConfig:
    return parallel_pick_cfg_from_params(params)


def _resolve_parallel_pick_target_poses(
    ctx: QueueRuntimeContext,
    cfg: BimanualParallelPickTaskConfig,
    *,
    execution_frame_id: str,
    tf_timeout: float,
) -> dict[str, Any]:
    """物体系偏移与 ``single_arm.pick`` 一致；必要时将目标位姿从 ``ctx.frame_id`` 变到 ``execution_frame_id``。"""
    src = str(ctx.frame_id).strip() or "base_link"
    exec_f = str(execution_frame_id).strip() or src

    def _one(side: str, prim: str, off: tuple[float, float, float]) -> Pose:
        if ctx.object_resolution is not None:
            pose = resolve_object_pose_for_task(
                ctx,
                object_prim_path=prim,
                include_orientation=True,
                arm_side=side,
                object_role=f"{side}_pick",
                entity_state_timeout=SERVICE_CALL_TIMEOUT,
                retries=SERVICE_CALL_RETRIES,
                retry_delay=SERVICE_RETRY_DELAY,
            )
        else:
            pose = get_object_pose_from_service(
                ctx.base_world_pos,
                ctx.base_world_quat,
                prim,
                include_orientation=True,
            )
        pose = apply_object_local_offset_to_pose(pose, off)
        if exec_f != src:
            iface = ctx.interface
            if not hasattr(iface, "transform_pose"):
                raise TypeError(
                    "dual_arm.parallel_pick: motion_frame_id requires ROS2RobotInterface.transform_pose (TF)"
                )
            transformed = iface.transform_pose(pose, src, exec_f, timeout=tf_timeout)
            if transformed is None:
                raise RuntimeError(
                    f"dual_arm.parallel_pick: TF {src!r} -> {exec_f!r} failed for {side} object pose; "
                    "check motion_frame_id and TF."
                )
            return transformed
        return pose

    left_pose = _one("left", cfg.left_pick.object_prim_path, cfg.left_pick.object_position_offset)
    right_pose = _one("right", cfg.right_pick.object_prim_path, cfg.right_pick.object_position_offset)
    return {"left": left_pose, "right": right_pose}


def _parallel_pick_full_stages(
    ctx: QueueRuntimeContext,
    cfg: BimanualParallelPickTaskConfig,
    target_poses: Mapping[str, Any],
) -> list[StageTarget]:
    left_pose = target_poses.get("left")
    right_pose = target_poses.get("right")
    if left_pose is None or right_pose is None:
        raise ValueError("parallel pick requires both left and right target poses")
    return build_bimanual_parallel_pick_record_sequence(
        task_cfg=cfg,
        left_target_pose=left_pose,
        right_target_pose=right_pose,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
    )


def skill_place(
    ctx: QueueRuntimeContext, params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂相对放置：读当前左右末端位姿 → 同向平移(持箱) → 松爪 → 沿双手连线外张 → 后撤。

    不依赖货架 prim 与 ``MergedQueueConfig.place``。可选从 ``ctx.carry_task_cfg`` 取
    ``spread_half`` / ``retreat_xyz`` 的默认（与搬运几何一致）。

    params:
        translation_xyz: 平移 [x,y,z]（米），在 **motion 坐标系** 下同加；默认 [0,0,0]。
        post_release_lower_xyz: 可选。在松爪后、外张前对左右同加的位移 [x,y,z]（motion 系）。
            未显式给出时，若 ``carry.ee_pregrasp_lift_offset`` 存在，则默认取其相反数（用于先抬后抓的对称放置下降）。
            且在 place 未显式配置下降分段时，若 ``carry.ee_lift_offset`` 存在，闭爪段优先映射为其相反数，
            开爪段自动改为“到目标平移的剩余量”（保证两段合计仍到参考 offset）。
        spread_half: 单侧沿「右→左」在 motion 系 XY 平面的外张距离（米）；默认取 carry 的 ``arm_merge_distance_y`` 或 0.04。
        retreat_xyz: 外张后左右再同加的位移 [x,y,z]（motion 系）；默认取 carry 的 ``ee_retreat_offset`` 或 [-0.2,0,0]。
        reference_object_prim_path: 可选。若提供，则以该 prim 在 ``motion_frame_id`` 下的位置作为放置参考点。
        object_position_offset: 可选。与 ``reference_object_prim_path`` 联用，按参考物体自身坐标系
            的偏移（米）补偿，再转换到 motion 系叠加到参考点（即偏移方向随参考物体姿态变化）。
            最终会自动换算为本次的 ``translation_xyz``（即参考点目标 - 当前双手中点）。
        motion_frame_id / relative_frame_id: 可选。当前末端数值的**源系**优先取左右臂订阅到的 ``frame_id``，
            若无则退回 ``ctx.frame_id``（Isaac 下常为 base prim 名如 ``base_link``）。与 ``motion_frame_id`` 不同时
            用 ``ROS2RobotInterface.transform_pose`` 变到 motion 系再算几何；``ExecutionMeta.frame_id`` 为 motion 系。
        tf_lookup_timeout: TF 查询超时（秒），默认 2.0。
        arm_movel_duration: 可选；本块优先，否则沿用 ``dual_arm.carry`` 的 ``arm_movel_duration``。
    """
    carry = ctx.carry_task_cfg
    _apply_arm_movel_duration(
        ctx,
        duration=params.get("arm_movel_duration", getattr(carry, "arm_movel_duration", None) if carry else None),
        label="dual_arm.place",
    )
    iface = ctx.interface
    lh, rh = iface.left_arm_handler, iface.right_arm_handler
    if lh is None or rh is None:
        raise TypeError("dual_arm.place requires left and right arm handlers")
    l0_raw = lh.get_pose()
    r0_raw = rh.get_pose()
    if l0_raw is None or r0_raw is None:
        raise RuntimeError("dual_arm.place: could not read current left/right EE poses")

    pose_frame = _ee_pose_source_frame(lh, rh, ctx.frame_id, label="dual_arm.place")
    l0, r0, motion_frame = _motion_frame_poses_from_params(
        iface, l0_raw, r0_raw, pose_frame, params, label="dual_arm.place",
    )

    default_spread = 0.04
    default_retreat: tuple[float, float, float] = (-0.2, 0.0, 0.0)
    if carry is not None:
        default_spread = float(carry.arm_merge_distance_y) if carry.arm_merge_distance_y else 0.04
        if carry.ee_retreat_offset is not None:
            default_retreat = tuple(float(x) for x in carry.ee_retreat_offset)
        else:
            default_retreat = (-0.2, 0.0, 0.0)

    tr = _param_vec3(params, "translation_xyz", (0.0, 0.0, 0.0))
    ref_path_raw = params.get("reference_object_prim_path")
    if isinstance(ref_path_raw, str) and ref_path_raw.strip():
        ref_path = ref_path_raw.strip()
        ref_off_obj = _param_vec3(params, "object_position_offset", (0.0, 0.0, 0.0))
        # 参考物位姿服务输出在 ctx.frame_id；必要时转换到 motion_frame 再参与几何计算。
        if ctx.object_resolution is not None:
            ref_pose = resolve_object_pose_for_task(
                ctx,
                object_prim_path=ref_path,
                include_orientation=True,
                arm_side="none",
                object_role="place_reference",
                entity_state_timeout=SERVICE_CALL_TIMEOUT,
                retries=SERVICE_CALL_RETRIES,
                retry_delay=SERVICE_RETRY_DELAY,
            )
        else:
            ref_pose = get_object_pose_from_service(
                ctx.base_world_pos,
                ctx.base_world_quat,
                ref_path,
                include_orientation=True,
            )
        ref_motion = ref_pose
        if motion_frame != ctx.frame_id:
            if not hasattr(iface, "transform_pose"):
                raise TypeError(
                    "dual_arm.place: reference_object_prim_path with motion_frame_id "
                    "requires ROS2RobotInterface.transform_pose",
                )
            try:
                tft = float(params.get("tf_lookup_timeout", 2.0))
            except (TypeError, ValueError):
                tft = 2.0
            ref_motion = iface.transform_pose(ref_pose, ctx.frame_id, motion_frame, timeout=tft)
            if ref_motion is None:
                raise RuntimeError(
                    f"dual_arm.place: TF {ctx.frame_id!r} -> {motion_frame!r} failed "
                    f"for reference_object_prim_path={ref_path!r}",
                )

        mid_x = (l0.position.x + r0.position.x) * 0.5
        mid_y = (l0.position.y + r0.position.y) * 0.5
        mid_z = (l0.position.z + r0.position.z) * 0.5
        ref_q = (
            float(ref_motion.orientation.x),
            float(ref_motion.orientation.y),
            float(ref_motion.orientation.z),
            float(ref_motion.orientation.w),
        )
        ref_off_motion = rotate_vector_by_quat(ref_off_obj, ref_q)
        tx = float(ref_motion.position.x) + ref_off_motion[0]
        ty = float(ref_motion.position.y) + ref_off_motion[1]
        tz = float(ref_motion.position.z) + ref_off_motion[2]
        tr = (tx - mid_x, ty - mid_y, tz - mid_z)

    spread_half = float(params.get("spread_half", params.get("spread_y_half", default_spread)))
    ret = _param_vec3(params, "retreat_xyz", default_retreat)
    post_release_lower = None
    if "post_release_lower_xyz" in params:
        post_release_lower = _param_vec3(params, "post_release_lower_xyz", (0.0, 0.0, 0.0))
    elif carry is not None and getattr(carry, "ee_pregrasp_lift_offset", None) is not None:
        pr = tuple(float(x) for x in carry.ee_pregrasp_lift_offset)
        # 对称于 carry 的 pregrasp-lift：place 在开爪后默认下移同等位移。
        post_release_lower = (-pr[0], -pr[1], -pr[2])
    # place 未显式配置下降分段时，优先将 carry.ee_lift_offset 映射到闭爪下降段。
    # 当前序列的闭爪段位移 = translation_xyz - post_release_lower_xyz，
    # 因此这里把开爪段改成剩余量，确保闭爪段等于 -carry.ee_lift_offset，
    # 同时两段合计仍精确到 reference offset。
    if (
        post_release_lower is not None
        and "post_release_lower_xyz" not in params
        and carry is not None
        and getattr(carry, "ee_lift_offset", None) is not None
    ):
        cl = tuple(float(x) for x in carry.ee_lift_offset)
        closed_lower = (-cl[0], -cl[1], -cl[2])
        post_release_lower = (
            tr[0] - closed_lower[0],
            tr[1] - closed_lower[1],
            tr[2] - closed_lower[2],
        )
    if post_release_lower is not None and max(abs(post_release_lower[0]), abs(post_release_lower[1]), abs(post_release_lower[2])) < 1e-12:
        post_release_lower = None

    stages = build_bimanual_place_relative_sequence(
        left_current=l0,
        right_current=r0,
        translation_xyz=tr,
        spread_half=spread_half,
        post_release_lower_xyz=post_release_lower,
        retreat_xyz=ret,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="PlaceRel",
        output_frame_id=motion_frame,
    )
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=motion_frame,
        warn_prefix="TaskQ dual_arm place timeout",
    )


def skill_bimanual_align(
    ctx: QueueRuntimeContext, params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """持箱时双臂对齐：将 **左右末端中点** 移到目标位置（可选姿态增量），``motion_frame_id`` 下计算。

    左右手 **同加** 位置增量，相对几何不变。目标仅支持 ``align_position: [x,y,z]`` 一次指定三轴。

    姿态：可选 ``orientation_delta_rpy``（``[roll, pitch, yaw]`` 弧度），左乘当前左、右末端四元数。

    params:
        align_position: 必填 ``[x, y, z]``（米），双臂中心目标位置（``motion_frame_id`` 下）。
        orientation_delta_rpy, min_abs_orientation_rpy, motion_frame_id,
        min_abs_delta_y / min_abs_delta_x / min_abs_delta_z, arm_movel_duration: 同前。
    """
    label = "dual_arm.bimanual_align"
    iface = ctx.interface
    lh, rh = iface.left_arm_handler, iface.right_arm_handler
    if lh is None or rh is None:
        raise TypeError(f"{label} requires left and right arm handlers")
    l0_raw = lh.get_pose()
    r0_raw = rh.get_pose()
    if l0_raw is None or r0_raw is None:
        raise RuntimeError(f"{label}: could not read current left/right EE poses")

    pose_frame = _ee_pose_source_frame(lh, rh, ctx.frame_id, label=label)
    l0, r0, motion_frame = _motion_frame_poses_from_params(
        iface, l0_raw, r0_raw, pose_frame, params, label=label,
    )

    ap = params.get("align_position")
    if not isinstance(ap, (list, tuple)) or len(ap) != 3:
        raise ValueError("align_position is required and must be a length-3 list [x, y, z]")
    target_mid_x = float(ap[0])
    target_mid_y = float(ap[1])
    target_mid_z = float(ap[2])

    try:
        min_abs = float(params.get("min_abs_delta_y", 1e-4))
    except (TypeError, ValueError):
        min_abs = 1e-4
    try:
        min_abs_x = float(params.get("min_abs_delta_x", min_abs))
    except (TypeError, ValueError):
        min_abs_x = min_abs
    try:
        min_abs_z = float(params.get("min_abs_delta_z", min_abs))
    except (TypeError, ValueError):
        min_abs_z = min_abs

    mid_x = (l0.position.x + r0.position.x) * 0.5
    mid_y = (l0.position.y + r0.position.y) * 0.5
    mid_z = (l0.position.z + r0.position.z) * 0.5
    delta_x = target_mid_x - mid_x
    delta_y = target_mid_y - mid_y
    delta_z = target_mid_z - mid_z

    dr, dp, dyaw = _param_vec3(params, "orientation_delta_rpy", (0.0, 0.0, 0.0))
    try:
        min_abs_ori = float(params.get("min_abs_orientation_rpy", 1e-4))
    except (TypeError, ValueError):
        min_abs_ori = 1e-4
    has_ori = max(abs(dr), abs(dp), abs(dyaw)) >= min_abs_ori
    move_x = abs(delta_x) >= min_abs_x
    move_y = abs(delta_y) >= min_abs
    move_z = abs(delta_z) >= min_abs_z
    if not (move_y or move_x or move_z or has_ori):
        return [], ExecutionMeta(
            send_mode=_dual_mode(ctx),
            frame_id=motion_frame,
            warn_prefix=f"TaskQ {label} timeout",
        )

    c_opt = ctx.carry_task_cfg
    _apply_arm_movel_duration(
        ctx,
        duration=params.get("arm_movel_duration", getattr(c_opt, "arm_movel_duration", None) if c_opt else None),
        label=label,
    )

    l1 = _clone_pose(l0)
    r1 = _clone_pose(r0)
    if move_y:
        l1.position.y += delta_y
        r1.position.y += delta_y
    if move_x:
        l1.position.x += delta_x
        r1.position.x += delta_x
    if move_z:
        l1.position.z += delta_z
        r1.position.z += delta_z
    if has_ori:
        qd = quat_normalize(euler_rpy_to_quat_xyzw(dr, dp, dyaw))
        ql = quat_normalize(quat_multiply(qd, _pose_quat_xyzw(l1)))
        qr = quat_normalize(quat_multiply(qd, _pose_quat_xyzw(r1)))
        _set_pose_quat_xyzw(l1, ql)
        _set_pose_quat_xyzw(r1, qr)

    g_cmd = float(ctx.gripper_closed)

    fid = str(motion_frame).strip() or None
    stages = [
        StageTarget(
            name="BimanualAlign-1-Move",
            left=ArmTarget(pose=l1, gripper=g_cmd),
            right=ArmTarget(pose=r1, gripper=g_cmd),
            frame_id=fid,
            skip_gripper_command=True,
        ),
    ]
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=motion_frame,
        warn_prefix=f"TaskQ {label} timeout",
    )


def skill_carry(ctx: QueueRuntimeContext, _params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：一次执行完整搬运序列。"""
    cfg = _require_carry(ctx)
    _apply_arm_movel_duration_from_carry_cfg(ctx, cfg, label="dual_arm.carry")
    if ctx.object_resolution is not None:
        oc_base = resolve_object_pose_for_task(
            ctx,
            object_prim_path=cfg.object_prim_path,
            include_orientation=False,
            arm_side="none",
            object_role="carry_object",
            entity_state_timeout=SERVICE_CALL_TIMEOUT,
            retries=SERVICE_CALL_RETRIES,
            retry_delay=SERVICE_RETRY_DELAY,
        )
    else:
        oc_base = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.object_prim_path,
            include_orientation=False,
        )
    oc = _object_position_in_carry_execution_frame(ctx, cfg, oc_base)
    ctx.carry_object_position = oc
    exec_f = _carry_execution_frame_id(cfg, ctx)
    stages = _carry_full_stages(ctx, cfg, oc, output_frame_id=exec_f)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ dual_arm carry timeout",
    )


def skill_parallel_pick(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    exec_l = _pick_execution_frame_id(ctx, cfg.left_pick.motion_frame_id)
    exec_r = _pick_execution_frame_id(ctx, cfg.right_pick.motion_frame_id)
    if exec_l != exec_r:
        raise ValueError(
            "dual_arm.parallel_pick: left_pick.motion_frame_id and right_pick.motion_frame_id must "
            f"resolve to the same frame (got {exec_l!r} vs {exec_r!r})"
        )
    exec_f = exec_l
    tf_timeout = max(
        _pick_tf_timeout(cfg.left_pick.tf_lookup_timeout),
        _pick_tf_timeout(cfg.right_pick.tf_lookup_timeout),
    )
    movel = cfg.left_pick.arm_movel_duration
    if movel is None:
        movel = cfg.right_pick.arm_movel_duration
    _apply_arm_movel_duration(ctx, duration=movel, label="dual_arm.parallel_pick")
    target_poses = _resolve_parallel_pick_target_poses(
        ctx, cfg, execution_frame_id=exec_f, tf_timeout=tf_timeout,
    )
    ctx.parallel_pick_target_poses = dict(target_poses)
    stages = _parallel_pick_full_stages(ctx, cfg, target_poses)
    ctx.gripper_for_return_home = ctx.gripper_closed
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=exec_f,
        warn_prefix="TaskQ dual_arm parallel pick timeout",
    )


def skill_goto_cache_pose(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂同步笛卡尔运动到 ``robot.cache_ee_pose``（``which: both``）写入的左右缓存位姿。"""
    key = str(params.get("key", "saved_ee_pose")).strip() or "saved_ee_pose"
    raw = ctx.scratch_get(key)
    if not isinstance(raw, Mapping) or not _is_bimanual_ee_cache(raw):
        raise ValueError(
            f"dual_arm.goto_cache_pose: cache[{key!r}] must be bimanual "
            "(use robot.cache_ee_pose with which: both)"
        )
    left_sub = raw.get("left")
    right_sub = raw.get("right")
    if not isinstance(left_sub, Mapping) or not isinstance(right_sub, Mapping):
        raise ValueError(
            f"dual_arm.goto_cache_pose: cache[{key!r}] needs valid left and right entries"
        )

    def _grip(sub: Mapping[str, Any], side_key: str) -> float:
        sk = f"{side_key}_gripper"
        if params.get(sk) is not None:
            return float(params[sk])
        if params.get("gripper") is not None:
            return float(params["gripper"])
        return float(sub.get("gripper", ctx.gripper_for_return_home))

    left_pose = _scratch_pose_entry_to_geometry_pose(left_sub)
    right_pose = _scratch_pose_entry_to_geometry_pose(right_sub)
    gl = _grip(left_sub, "left")
    gr = _grip(right_sub, "right")

    stages = [
        StageTarget(
            name="TaskQ-DualGotoCachePose",
            left=ArmTarget(pose=left_pose, gripper=gl),
            right=ArmTarget(pose=right_pose, gripper=gr),
        )
    ]
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm goto cache pose timeout",
    )


def skill_handover_sync(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂同步交接（``build_handover_sequence``）；抓取 / 放置由 ``single_arm.pick`` / ``place`` 承担。

    交接「给出」侧由 **抓取臂** 决定：与 ``single_arm.pick.arm`` 一致（合并进 ``QueueSingleArmSlice.common.arm``），
    另一侧为接收臂；无需在 ``dual_arm.handover`` 里单独配置 source/receiver。
    """
    hcfg = _require_handover_sync(ctx)
    # 与 pick 块合并后的臂别（来自 single_arm.pick.arm）
    arm = ctx.task_cfg.common.arm.strip().lower()
    if arm not in ("left", "right"):
        raise ValueError(f"common.arm must be 'left' or 'right' for handover pick side, got {arm!r}")
    source_is_right = arm == "right"
    stages = build_handover_sync_sequence(
        sync_cfg=hcfg,
        source_is_right=source_is_right,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix="Handover",
    )
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        left_arrival_guard_stage="Handover-1-SyncMove" if source_is_right else None,
        warn_prefix="TaskQ dual_arm handover sync timeout",
    )


def register_dual_arm_skills() -> None:
    register_skill("dual_arm.carry", skill_carry)
    register_skill("dual_arm.parallel_pick", skill_parallel_pick)
    register_skill("dual_arm.bimanual_align", skill_bimanual_align)
    register_skill("dual_arm.place", skill_place)
    register_skill("dual_arm.handover", skill_handover_sync)
    register_skill("dual_arm.goto_cache_pose", skill_goto_cache_pose)


register_dual_arm_skills()
