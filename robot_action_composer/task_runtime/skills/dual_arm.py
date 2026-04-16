"""Dual-arm queue skills (``dual_arm.*``): carry + handover sync + cache goto."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (  # pyright: ignore[reportMissingImports]
    ArmTarget,
    SendMode,
    StageTarget,
    build_bimanual_place_relative_sequence,
)

from robot_action_composer.isaac_sim import get_object_pose_from_service  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.tasks.bimanual_carry import (  # pyright: ignore[reportMissingImports]
    BimanualCarryTaskConfig,
    build_bimanual_carry_record_sequence,
    carry_approach_stage_count,
    slice_carry_stages_for_queue,
)
from robot_action_composer.motion_generation.tasks.bimanual_place import (  # pyright: ignore[reportMissingImports]
    BimanualPlaceTaskConfig,
    build_bimanual_place_record_sequence,
    place_trailing_after_advance_stage_count,
    slice_place_stages_for_queue,
)
from robot_action_composer.motion_generation.tasks.drawer import euler_to_quaternion  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.bimanual_parallel_pick import (  # pyright: ignore[reportMissingImports]
    BimanualParallelPickTaskConfig,
    build_bimanual_parallel_pick_record_sequence,
    parallel_pick_cfg_from_params,
    slice_parallel_pick_stages_for_queue,
)
from robot_action_composer.motion_generation.tasks.handover import build_handover_sync_sequence  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.pick_place import _apply_target_pose_offset  # pyright: ignore[reportMissingImports]

from robot_action_composer.task_runtime.context import QueueRuntimeContext
from robot_action_composer.task_runtime.registry import register_skill
from robot_action_composer.task_runtime.skills.single_arm import (
    _is_bimanual_ee_cache,
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


def _rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """与 :func:`robot_action_composer.motion_generation.tasks.drawer.euler_to_quaternion` 相同 RPY → ``geometry_msgs`` 四元数 x,y,z,w。"""
    w, x, y, z = euler_to_quaternion(roll, pitch, yaw)
    return (x, y, z, w)


def _quat_mul_xyzw(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Hamilton 积 ``q1 * q2``（对应旋转矩阵 ``R(q1) R(q2)``）。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return (x, y, z, w)


def _norm_quat_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


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


def _require_place(ctx: QueueRuntimeContext) -> BimanualPlaceTaskConfig:
    p = ctx.place_task_cfg
    if p is None:
        raise TypeError(
            "dual_arm.place* skills require place_task_cfg (MergedQueueConfig.place / skill_defaults.dual_arm.place)"
        )
    return p


def _carry_full_stages(ctx: QueueRuntimeContext, cfg: BimanualCarryTaskConfig, object_center: Any) -> list[StageTarget]:
    return build_bimanual_carry_record_sequence(
        carry_task_cfg=cfg,
        object_center=object_center,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        output_frame_id=ctx.frame_id,
    )


def _place_full_stages(
    ctx: QueueRuntimeContext, cfg: BimanualPlaceTaskConfig, object_center: Any,
) -> list[StageTarget]:
    return build_bimanual_place_record_sequence(
        place_task_cfg=cfg,
        object_center=object_center,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        output_frame_id=ctx.frame_id,
    )


def _require_handover_sync(ctx: QueueRuntimeContext):
    h = ctx.handover_sync
    if h is None:
        raise TypeError(
            "dual_arm.handover_sync requires handover_sync (merged from skill_defaults.dual_arm.handover; "
            "need handover_position + orientations). Pick side arm comes from common.arm / single_arm.pick."
        )
    return h


def _require_parallel_pick_cfg(params: Mapping[str, Any]) -> BimanualParallelPickTaskConfig:
    return parallel_pick_cfg_from_params(params)


def _resolve_parallel_pick_target_poses(
    ctx: QueueRuntimeContext, cfg: BimanualParallelPickTaskConfig,
) -> dict[str, Any]:
    left_pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.left_pick.source_object_entity_path,
        include_orientation=False,
    )
    right_pose = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.right_pick.source_object_entity_path,
        include_orientation=False,
    )
    left_pose = _apply_target_pose_offset(left_pose, cfg.left_pick.target_pose_offset)
    right_pose = _apply_target_pose_offset(right_pose, cfg.right_pick.target_pose_offset)
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


def skill_carry_approach(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Approach → Forward → CloseIn（夹爪张开）。写入 ``ctx.carry_object_center`` 供后续段复用。"""
    cfg = _require_carry(ctx)
    _apply_arm_movel_duration_from_carry_cfg(ctx, cfg, label="dual_arm.carry_approach")
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.source_object_entity_path,
        include_orientation=False,
    )
    ctx.carry_object_center = object_center
    full = _carry_full_stages(ctx, cfg, object_center)
    approach, _g, _lr = slice_carry_stages_for_queue(
        full, approach_stage_count=carry_approach_stage_count(cfg),
    )
    return approach, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry approach timeout",
    )


def skill_carry_grasp(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Grasp（闭合）。须先于本块执行 ``dual_arm.carry_approach`` 以填充 ``ctx.carry_object_center``。"""
    cfg = _require_carry(ctx)
    _apply_arm_movel_duration_from_carry_cfg(ctx, cfg, label="dual_arm.carry_grasp")
    oc = ctx.carry_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.source_object_entity_path,
            include_orientation=False,
        )
        ctx.carry_object_center = oc
    full = _carry_full_stages(ctx, cfg, oc)
    _a, grasp, _lr = slice_carry_stages_for_queue(
        full, approach_stage_count=carry_approach_stage_count(cfg),
    )
    return grasp, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry grasp timeout",
    )


def skill_carry_lift_retreat(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：Lift → Retreat。"""
    cfg = _require_carry(ctx)
    _apply_arm_movel_duration_from_carry_cfg(ctx, cfg, label="dual_arm.carry_lift_retreat")
    oc = ctx.carry_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.source_object_entity_path,
            include_orientation=False,
        )
        ctx.carry_object_center = oc
    full = _carry_full_stages(ctx, cfg, oc)
    _a, _g, lift_retreat = slice_carry_stages_for_queue(
        full, approach_stage_count=carry_approach_stage_count(cfg),
    )
    return lift_retreat, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry lift/retreat timeout",
    )


def skill_place_advance(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂放置：先「送入」两段（retreat→lift、lift→close-in），夹爪闭合持箱。写入 ``ctx.place_object_center``。"""
    cfg = _require_place(ctx)
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.place_object_entity_path,
        include_orientation=False,
    )
    ctx.place_object_center = object_center
    full = _place_full_stages(ctx, cfg, object_center)
    advance, _rel, _sr = slice_place_stages_for_queue(
        full, trailing_after_advance=place_trailing_after_advance_stage_count(cfg),
    )
    return advance, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm place advance timeout",
    )


def skill_place_release(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂放置：在合拢位松爪（须先于本块执行 ``dual_arm.place_advance`` 以填充 ``ctx.place_object_center``）。"""
    cfg = _require_place(ctx)
    oc = ctx.place_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.place_object_entity_path,
            include_orientation=False,
        )
        ctx.place_object_center = oc
    full = _place_full_stages(ctx, cfg, oc)
    _adv, release, _sr = slice_place_stages_for_queue(
        full, trailing_after_advance=place_trailing_after_advance_stage_count(cfg),
    )
    return release, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm place release timeout",
    )


def skill_place_spread_retreat(
    ctx: QueueRuntimeContext, _params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂放置：Y 向张开后再后撤至预接近位（夹爪张开）。"""
    cfg = _require_place(ctx)
    oc = ctx.place_object_center
    if oc is None:
        oc = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            cfg.place_object_entity_path,
            include_orientation=False,
        )
        ctx.place_object_center = oc
    full = _place_full_stages(ctx, cfg, oc)
    _adv, _rel, spread_retreat = slice_place_stages_for_queue(
        full, trailing_after_advance=place_trailing_after_advance_stage_count(cfg),
    )
    return spread_retreat, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm place spread/retreat timeout",
    )


def skill_place_relative(
    ctx: QueueRuntimeContext, params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂相对放置：读当前左右末端位姿 → 同向平移(持箱) → 松爪 → 沿双手连线外张 → 后撤。

    不依赖货架 prim 与 ``MergedQueueConfig.place``。可选从 ``ctx.carry_task_cfg`` 取
    ``spread_half`` / ``retreat_xyz`` 的默认（与搬运几何一致）。

    params:
        translation_xyz: 平移 [x,y,z]（米），在 **motion 坐标系** 下同加；默认 [0,0,0]。
        spread_half: 单侧沿「右→左」在 motion 系 XY 平面的外张距离（米）；默认取 carry 的 ``carry_approach_clearance_y`` 或 0.04。
        retreat_xyz: 外张后左右再同加的位移 [x,y,z]（motion 系）；默认取 carry 的 ``carry_retreat_xyz`` 或 [-0.2,0,0]。
        reference_object_entity_path: 可选。若提供，则以该 prim 在 ``motion_frame_id`` 下的位置作为放置参考点。
        reference_offset_xyz: 可选。与 ``reference_object_entity_path`` 联用，在参考点上再加偏移（米）。
            最终会自动换算为本次的 ``translation_xyz``（即参考点目标 - 当前双手中点）。
        motion_frame_id / relative_frame_id: 可选。当前末端数值的**源系**优先取左右臂订阅到的 ``frame_id``，
            若无则退回 ``ctx.frame_id``（Isaac 下常为 base prim 名如 ``base_link``）。与 ``motion_frame_id`` 不同时
            用 ``ROS2RobotInterface.transform_pose`` 变到 motion 系再算几何；``ExecutionMeta.frame_id`` 为 motion 系。
        tf_lookup_timeout: TF 查询超时（秒），默认 2.0。
        stage_prefix: 阶段名前缀，默认 ``PlaceRel``。
        arm_movel_duration: 可选；本块优先，否则沿用 ``dual_arm.carry`` 的 ``arm_movel_duration``。
    """
    carry = ctx.carry_task_cfg
    _apply_arm_movel_duration(
        ctx,
        duration=params.get("arm_movel_duration", getattr(carry, "arm_movel_duration", None) if carry else None),
        label="dual_arm.place_relative",
    )
    iface = ctx.interface
    lh, rh = iface.left_arm_handler, iface.right_arm_handler
    if lh is None or rh is None:
        raise TypeError("dual_arm.place_relative requires left and right arm handlers")
    l0_raw = lh.get_pose()
    r0_raw = rh.get_pose()
    if l0_raw is None or r0_raw is None:
        raise RuntimeError("dual_arm.place_relative: could not read current left/right EE poses")

    pose_frame = _ee_pose_source_frame(lh, rh, ctx.frame_id, label="dual_arm.place_relative")
    l0, r0, motion_frame = _motion_frame_poses_from_params(
        iface, l0_raw, r0_raw, pose_frame, params, label="dual_arm.place_relative",
    )

    default_spread = 0.04
    default_retreat: tuple[float, float, float] = (-0.2, 0.0, 0.0)
    if carry is not None:
        default_spread = float(carry.carry_approach_clearance_y) if carry.carry_approach_clearance_y else 0.04
        if carry.carry_retreat_xyz is not None:
            default_retreat = tuple(float(x) for x in carry.carry_retreat_xyz)
        else:
            default_retreat = (-0.2, 0.0, 0.0)

    tr = _param_vec3(params, "translation_xyz", (0.0, 0.0, 0.0))
    ref_path_raw = params.get("reference_object_entity_path")
    if isinstance(ref_path_raw, str) and ref_path_raw.strip():
        ref_path = ref_path_raw.strip()
        ref_off = _param_vec3(params, "reference_offset_xyz", (0.0, 0.0, 0.0))
        # 参考物位姿服务输出在 ctx.frame_id；必要时转换到 motion_frame 再参与几何计算。
        ref_pose = get_object_pose_from_service(
            ctx.base_world_pos,
            ctx.base_world_quat,
            ref_path,
            include_orientation=False,
        )
        ref_motion = ref_pose
        if motion_frame != ctx.frame_id:
            if not hasattr(iface, "transform_pose"):
                raise TypeError(
                    "dual_arm.place_relative: reference_object_entity_path with motion_frame_id "
                    "requires ROS2RobotInterface.transform_pose",
                )
            try:
                tft = float(params.get("tf_lookup_timeout", 2.0))
            except (TypeError, ValueError):
                tft = 2.0
            ref_motion = iface.transform_pose(ref_pose, ctx.frame_id, motion_frame, timeout=tft)
            if ref_motion is None:
                raise RuntimeError(
                    f"dual_arm.place_relative: TF {ctx.frame_id!r} -> {motion_frame!r} failed "
                    f"for reference_object_entity_path={ref_path!r}",
                )

        mid_x = (l0.position.x + r0.position.x) * 0.5
        mid_y = (l0.position.y + r0.position.y) * 0.5
        mid_z = (l0.position.z + r0.position.z) * 0.5
        tx = float(ref_motion.position.x) + ref_off[0]
        ty = float(ref_motion.position.y) + ref_off[1]
        tz = float(ref_motion.position.z) + ref_off[2]
        tr = (tx - mid_x, ty - mid_y, tz - mid_z)

    spread_half = float(params.get("spread_half", params.get("spread_y_half", default_spread)))
    ret = _param_vec3(params, "retreat_xyz", default_retreat)
    prefix = str(params.get("stage_prefix", "PlaceRel")).strip() or "PlaceRel"

    stages = build_bimanual_place_relative_sequence(
        left_current=l0,
        right_current=r0,
        translation_xyz=tr,
        spread_half=spread_half,
        retreat_xyz=ret,
        gripper_open=ctx.gripper_open,
        gripper_closed=ctx.gripper_closed,
        stage_prefix=prefix,
        output_frame_id=motion_frame,
    )
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=motion_frame,
        warn_prefix="TaskQ dual_arm place_relative timeout",
    )


def skill_bimanual_align_mid_y(
    ctx: QueueRuntimeContext, params: Mapping[str, Any],
) -> tuple[list[StageTarget], ExecutionMeta]:
    """持箱时对齐：可选 **Y 中点** 与 **双手同向姿态增量** 在同一段完成。

    Y：左右手 **同加** ``delta_y = target_mid_y - mid_y``，故 ``y_L - y_R`` 不变，X/Z 不变。

    姿态：可选 ``orientation_delta_rpy``（``[roll, pitch, yaw]`` 弧度），约定与
    :func:`robot_action_composer.motion_generation.tasks.drawer.euler_to_quaternion` 一致；
    在 ``motion_frame_id`` 下得到增量四元数 **左乘** 当前左、右末端姿态（左右 **相同** 增量，相对几何不变）。

    可选 ``motion_frame_id``（与 ``dual_arm.place_relative`` 相同 TF 语义）；默认在订阅/推断的 pose 系下计算。

    params:
        target_mid_y: 目标中点 Y（米），默认 ``0.0``（常见为 arm_base 下身体中线附近）。
        orientation_delta_rpy: 可选 ``[roll, pitch, yaw]`` 弧度；全零或不写则不改姿态。
        min_abs_orientation_rpy: 若 ``orientation_delta_rpy`` 各分量绝对值均小于该值（弧度），视为无姿态增量，默认 ``1e-4``。
        motion_frame_id / relative_frame_id, tf_lookup_timeout: 同 ``place_relative``。
        gripper: 可选，覆盖左右 ``ArmTarget.gripper``；默认 ``ctx.gripper_closed``（持箱）。
        min_abs_delta_y: 仅 Y 对齐时：小于该阈值（米）则跳过 Y；若仍配置了有效 ``orientation_delta_rpy`` 仍会生成段。默认 ``1e-4``。
        stage_name: 单段名，默认 ``DualAlignMidY-1-Move``。
        arm_movel_duration: 可选；本块 ``params`` 优先，缺省则沿用 ``dual_arm.carry`` 的 ``arm_movel_duration``。
    """
    iface = ctx.interface
    lh, rh = iface.left_arm_handler, iface.right_arm_handler
    if lh is None or rh is None:
        raise TypeError("dual_arm.bimanual_align_mid_y requires left and right arm handlers")
    l0_raw = lh.get_pose()
    r0_raw = rh.get_pose()
    if l0_raw is None or r0_raw is None:
        raise RuntimeError("dual_arm.bimanual_align_mid_y: could not read current left/right EE poses")

    label = "dual_arm.bimanual_align_mid_y"
    pose_frame = _ee_pose_source_frame(lh, rh, ctx.frame_id, label=label)
    l0, r0, motion_frame = _motion_frame_poses_from_params(
        iface, l0_raw, r0_raw, pose_frame, params, label=label,
    )

    try:
        target_mid_y = float(params.get("target_mid_y", 0.0))
    except (TypeError, ValueError) as e:
        raise ValueError("target_mid_y must be a float") from e
    try:
        min_abs = float(params.get("min_abs_delta_y", 1e-4))
    except (TypeError, ValueError):
        min_abs = 1e-4

    mid_y = (l0.position.y + r0.position.y) * 0.5
    delta_y = target_mid_y - mid_y
    dr, dp, dyaw = _param_vec3(params, "orientation_delta_rpy", (0.0, 0.0, 0.0))
    try:
        min_abs_ori = float(params.get("min_abs_orientation_rpy", 1e-4))
    except (TypeError, ValueError):
        min_abs_ori = 1e-4
    has_ori = max(abs(dr), abs(dp), abs(dyaw)) >= min_abs_ori
    if abs(delta_y) < min_abs and not has_ori:
        return [], ExecutionMeta(
            send_mode=_dual_mode(ctx),
            frame_id=motion_frame,
            warn_prefix="TaskQ dual_arm bimanual_align_mid_y timeout",
        )

    c_opt = ctx.carry_task_cfg
    _apply_arm_movel_duration(
        ctx,
        duration=params.get("arm_movel_duration", getattr(c_opt, "arm_movel_duration", None) if c_opt else None),
        label="dual_arm.bimanual_align_mid_y",
    )

    l1 = _clone_pose(l0)
    r1 = _clone_pose(r0)
    if abs(delta_y) >= min_abs:
        l1.position.y += delta_y
        r1.position.y += delta_y
    if has_ori:
        qd = _norm_quat_xyzw(_rpy_to_quat_xyzw(dr, dp, dyaw))
        ql = _norm_quat_xyzw(_quat_mul_xyzw(qd, _pose_quat_xyzw(l1)))
        qr = _norm_quat_xyzw(_quat_mul_xyzw(qd, _pose_quat_xyzw(r1)))
        _set_pose_quat_xyzw(l1, ql)
        _set_pose_quat_xyzw(r1, qr)

    g_raw = params.get("gripper")
    if g_raw is None:
        g_cmd = float(ctx.gripper_closed)
    else:
        g_cmd = float(g_raw)

    stage_name = str(params.get("stage_name", "DualAlignMidY-1-Move")).strip() or "DualAlignMidY-1-Move"
    fid = str(motion_frame).strip() or None
    stages = [
        StageTarget(
            name=stage_name,
            left=ArmTarget(pose=l1, gripper=g_cmd),
            right=ArmTarget(pose=r1, gripper=g_cmd),
            frame_id=fid,
        ),
    ]
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=motion_frame,
        warn_prefix="TaskQ dual_arm bimanual_align_mid_y timeout",
    )


def skill_place(ctx: QueueRuntimeContext, _params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂放置：一次执行全部 5 段（与 ``place_advance`` / ``release`` / ``spread_retreat`` 等价）。"""
    cfg = _require_place(ctx)
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.place_object_entity_path,
        include_orientation=False,
    )
    ctx.place_object_center = object_center
    stages = _place_full_stages(ctx, cfg, object_center)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm place timeout",
    )


def skill_carry(ctx: QueueRuntimeContext, _params: Mapping[str, Any]) -> tuple[list[StageTarget], ExecutionMeta]:
    """双臂搬运：一次执行全部 6 段（兼容旧队列；新任务推荐 ``carry_approach`` / ``grasp`` / ``lift_retreat``）。"""
    cfg = _require_carry(ctx)
    _apply_arm_movel_duration_from_carry_cfg(ctx, cfg, label="dual_arm.carry")
    object_center = get_object_pose_from_service(
        ctx.base_world_pos,
        ctx.base_world_quat,
        cfg.source_object_entity_path,
        include_orientation=False,
    )
    ctx.carry_object_center = object_center
    stages = _carry_full_stages(ctx, cfg, object_center)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm carry timeout",
    )


def skill_parallel_pick_approach(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
    ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    approach, _g, _r = slice_parallel_pick_stages_for_queue(full)
    return approach, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick approach timeout",
    )


def skill_parallel_pick_grasp(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = ctx.parallel_pick_target_poses
    if target_poses is None:
        target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
        ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    _a, grasp, _r = slice_parallel_pick_stages_for_queue(full)
    return grasp, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick grasp timeout",
    )


def skill_parallel_pick_retreat(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = ctx.parallel_pick_target_poses
    if target_poses is None:
        target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
        ctx.parallel_pick_target_poses = dict(target_poses)
    full = _parallel_pick_full_stages(ctx, cfg, target_poses)
    _a, _g, retreat = slice_parallel_pick_stages_for_queue(full)
    return retreat, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
        warn_prefix="TaskQ dual_arm parallel pick retreat timeout",
    )


def skill_parallel_pick(
    ctx: QueueRuntimeContext, params: Mapping[str, Any]
) -> tuple[list[StageTarget], ExecutionMeta]:
    cfg = _require_parallel_pick_cfg(params)
    target_poses = _resolve_parallel_pick_target_poses(ctx, cfg)
    ctx.parallel_pick_target_poses = dict(target_poses)
    stages = _parallel_pick_full_stages(ctx, cfg, target_poses)
    return stages, ExecutionMeta(
        send_mode=_dual_mode(ctx),
        frame_id=ctx.frame_id,
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
            name=str(params.get("stage_name", "TaskQ-DualGotoCachePose")),
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
    """双臂同步交接（``build_handover_sequence``）；抓取 / 放置由 ``single_arm.pick`` / ``place`` 承担。"""
    hcfg = _require_handover_sync(ctx)
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
    register_skill("dual_arm.carry_approach", skill_carry_approach)
    register_skill("dual_arm.carry_grasp", skill_carry_grasp)
    register_skill("dual_arm.carry_lift_retreat", skill_carry_lift_retreat)
    register_skill("dual_arm.carry", skill_carry)
    register_skill("dual_arm.parallel_pick_approach", skill_parallel_pick_approach)
    register_skill("dual_arm.parallel_pick_grasp", skill_parallel_pick_grasp)
    register_skill("dual_arm.parallel_pick_retreat", skill_parallel_pick_retreat)
    register_skill("dual_arm.parallel_pick", skill_parallel_pick)
    register_skill("dual_arm.place_advance", skill_place_advance)
    register_skill("dual_arm.place_release", skill_place_release)
    register_skill("dual_arm.place_spread_retreat", skill_place_spread_retreat)
    register_skill("dual_arm.place_relative", skill_place_relative)
    register_skill("dual_arm.bimanual_align_mid_y", skill_bimanual_align_mid_y)
    register_skill("dual_arm.place", skill_place)
    register_skill("dual_arm.handover_sync", skill_handover_sync)
    register_skill("dual_arm.goto_cache_pose", skill_goto_cache_pose)


register_dual_arm_skills()
