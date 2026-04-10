"""队列执行期上下文：在 ``task_queue`` 各 skill 之间传递。"""

from __future__ import annotations

from dataclasses import dataclass
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
    """一次 ``run_task_queue`` 的会话：接口快照 + :class:`QueueSingleArmSlice` + 可选子任务配置。"""

    interface: Any  # ROS2RobotInterface
    robot_cfg: Any
    sim_time: Any
    task_cfg: Any  # QueueSingleArmSlice
    gripper_open: float
    gripper_closed: float
    use_stamped: bool
    frame_id: str
    ee_frame_id: str
    arm_side: Any  # ArmSide
    source_is_right: bool
    source_home_pose: Any
    base_world_pos: Any
    base_world_quat: Any
    gripper_for_return_home: float = 0.0
    source_ee_prefix: str = "left_ee"
    drawer: DrawerPhaseState | None = None
    left_initial_joint_positions: list[float] | None = None
    right_initial_joint_positions: list[float] | None = None
    body_initial_joint_positions: list[float] | None = None
    left_home_pose: Any | None = None
    right_home_pose: Any | None = None
    receiver_home_pose: Any | None = None
    carry_task_cfg: Any | None = None
    carry_object_center: Any | None = None  # dual_arm.carry_approach 写入；后续搬运段复用
    handover_sync: HandoverSyncConfig | None = None
    drawer_geometry: DrawerGeometryConfig | None = None

    def __post_init__(self) -> None:
        if self.gripper_for_return_home == 0.0:
            self.gripper_for_return_home = float(self.gripper_closed)