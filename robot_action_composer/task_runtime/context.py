"""队列执行期上下文：在 ``task_queue`` 各 skill 之间传递。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from robot_action_composer.motion_generation.tasks.drawer import DrawerGeometryConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.motion_generation.tasks.handover import HandoverSyncConfig  # pyright: ignore[reportMissingImports]


@dataclass
class DrawerPhaseState:
    """跨 ``single_arm.drawer.*`` 技能传递的会话状态（拉手 → 关抽屉 → 撤退）。

    与 **柜门、开关柜** 等同类任务：建议同样使用独立命名空间（例如未来的
    :class:`DoorPhaseState` + ``ctx.door``），避免往通用上下文中平铺字段。
    """

    place_pose_ref: tuple[float, float, float]
    grasp_orientation_xyzw: tuple[float, float, float, float]
    grasp_direction_vector: tuple[float, float, float]
    handle_offset_rotated: tuple[float, float, float]


@dataclass
class QueueRuntimeContext:
    """一次 ``run_task_queue`` 的会话：接口快照 + :class:`QueueSingleArmSlice` + 可选子任务配置。

    **scratch**：跨 skill 的通用键值暂存区（任务 YAML 可用 ``session.scratch_put``、``robot.cache_ee_pose`` 等写入；
    自定义 skill 内用 :meth:`scratch_get` / :meth:`scratch_put` 或直接使用 ``ctx.scratch``）。
    笛卡尔「回程」位姿由 ``robot.cache_ee_pose`` + ``goto_cache_pose`` 显式缓存，不再在 Runner 连接时写入 home。
    """

    interface: Any  # ROS2RobotInterface
    robot_cfg: Any
    sim_time: Any
    task_cfg: Any  # QueueSingleArmSlice
    base_link_entity_path: str  # 已解析的 Isaac base prim（任务可覆盖 robot_cfg）
    gripper_open: float
    gripper_closed: float
    use_stamped: bool
    frame_id: str
    base_world_pos: Any
    base_world_quat: Any
    gripper_for_return_home: float = 0.0
    drawer: DrawerPhaseState | None = None
    carry_task_cfg: Any | None = None
    carry_object_center: Any | None = None  # dual_arm.carry_approach 写入；后续搬运段复用
    handover_sync: HandoverSyncConfig | None = None
    drawer_geometry: DrawerGeometryConfig | None = None
    scratch: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.gripper_for_return_home == 0.0:
            self.gripper_for_return_home = float(self.gripper_closed)

    def scratch_put(self, key: str, value: Any) -> None:
        """写入暂存区（与 ``session.scratch_put`` skill 一致）。"""
        self.scratch[str(key)] = value

    def scratch_get(self, key: str, default: Any = None) -> Any:
        """读取暂存区；缺省键返回 ``default``。"""
        return self.scratch.get(str(key), default)


def queue_pick_arm_is_right(ctx: QueueRuntimeContext) -> bool:
    """当前 ``task_cfg.common.arm`` 是否为右侧（抓取侧 / 主序列侧）。"""
    a = ctx.task_cfg.common.arm.strip().lower()
    if a not in {"left", "right"}:
        raise ValueError(f"arm must be 'left' or 'right', got {ctx.task_cfg.common.arm!r}")
    return a == "right"


def queue_primary_ee_frame_id(ctx: QueueRuntimeContext) -> str:
    """抓取侧末端链路的 ``frame_id``（无 handler 时退回 ``ctx.frame_id``）。"""
    h = (
        ctx.interface.right_arm_handler
        if queue_pick_arm_is_right(ctx)
        else ctx.interface.left_arm_handler
    )
    fid = h.frame_id if h else None
    return fid or ctx.frame_id