# robot-action-composer

在 **ROS 2** 上编排 **任务队列**（YAML 块 + 技能注册表），把几何与参数变成 **`StageTarget` 序列**并通过 **`ROS2RobotInterface`** 执行；可选接入 **LeRobot 数据集录制**。

---

## 文档

| 文档 | 内容 |
|------|------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 分层设计：`motion_generation` / `task_runtime` / CLI / 录制等 |
| [docs/TASK_CONFIG_YAML.md](docs/TASK_CONFIG_YAML.md) | 任务 YAML 格式、`skill_defaults` / `skill_params`、加载规则 |
| [docs/ROBOT_CONFIG.md](docs/ROBOT_CONFIG.md) | `robot_config.py`、`ROBOT_CFG` 与 `robots/` 目录约定 |

本仓库中与 **Isaac Sim 环境、USD、工作空间** 相关的步骤见 monorepo 内 **`examples/IsaacSim/README.md`**。

---

## 包内大致分工

**运动层（`motion_generation`）**  
通用笛卡尔 builder（`sequence/cartesian_stages.py`）与任务专用几何（`tasks/`：抽屉、交接同步段、搬运、MoveJ 回零等）。

**编排层（`task_runtime`）**  
`run_task_queue`：连接 → FSM → 按 `task_queue` 调注册技能 → `execute_stage_sequence`。内置技能见 `task_runtime/skills/`（单臂、双臂、抽屉、导航等）。

**配置与发现**  
`task_config_io`：`discover_task_configs`、`flatten_queue_task_overrides`（仅 YAML，禁止根级嵌套 `pick`/`place`/…）。  
`discovery/registry_loader`：`load_motion_entries` / `load_robot_entries`，扫描给定根目录下的 `robots/*/robot_config.py` 与 `task_configs/*.yaml`。

**录制（可选 extra `[recording]`）**  
构建 **`ROS2Robot`** 写 LeRobot 数据集；片内运动仍走 **`robot.ros2_interface`**，与纯运动路径共用同一套接口对象。

**约定**  
- 纯运动/队列路径只依赖 **`ROS2RobotInterface`**（`ctx.interface`），不依赖 **`ROS2Robot`** 类。  
- **导航**：`task_runtime/skills/navigation.py` 提供 Nav2 相关封装，基座位姿等可配合 `isaac_sim` 实体服务。

---

## 安装

在 monorepo 中（路径按你的仓库为准）：

```bash
pip install -e submodules/ros2_robot_interface
pip install -e submodules/robot_action_composer
```

需要录制时再装：

```bash
pip install -e lerobot_robot_ros2
pip install -e "submodules/robot_action_composer[recording]"
```

**环境**：Python ≥ 3.10；`rclpy`、消息类型、`cv_bridge` 等来自当前 **ROS 2** 发行版环境。

---

## 备注

部分模块（如 `isaac_sim`、`motion_generation/tasks/drawer`）仍引用 **`lerobot_robot_ros2.utils.pose_utils`** 做四元数等工具函数（**不**依赖 `ROS2Robot` 类）。若要去掉该依赖，可将这些 helper 迁入本包。
