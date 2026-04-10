# 机器人配置：`robot_config.py` 与目录约定

本文件说明与 **`robot_action_composer`** 的 **运动 / 录制 / 推理** 入口配合时，每个机器人目录应包含哪些文件、**`robot_config.py`** 需导出哪些符号，以及 **`ROBOT_CFG`** 在运动运行时会被用到哪些字段。

任务 YAML 格式见 **[TASK_CONFIG_YAML.md](TASK_CONFIG_YAML.md)**。包内分层见 **[ARCHITECTURE.md](ARCHITECTURE.md)**。

---

## 1. 目录约定

对每个机器人，约定一个**子目录**（名称任意，通常与机型一致），其下至少包含：

| 路径 | 作用 |
|------|------|
| **`robot_config.py`** | 导出 `ROBOT_KEY`、`ROBOT_LABEL`、`ROBOT_CFG` |
| **`task_configs/`** | 若干 `*.yaml` / `*.yml` 任务文件（见 `task_config_io.discover_task_configs`） |

**发现逻辑**（`robot_action_composer.discovery.registry_loader.load_motion_entries`）：

- 遍历传入的 **`isaac_dir / "robots"`**（在 monorepo 中一般为 `examples/IsaacSim/robots`）。
- 跳过以 `__` 开头的目录。
- 若存在 **`robot_config.py`** 且存在 **`task_configs/`** 目录，则加载该机器人并扫描任务 YAML。

各机器人子目录内可有 **`README.md`**（相机、话题、仿真路径等），与 composer 加载无强耦合。

---

## 2. `robot_config.py` 导出约定

模块**必须**提供以下**模块级**属性：

| 符号 | 类型 | 含义 |
|------|------|------|
| **`ROBOT_KEY`** | `str` | 注册表键（小写+下划线，如 `dobot_cr5`），用于 CLI / 推理侧选择机器人 |
| **`ROBOT_LABEL`** | `str` | 人类可读名称 |
| **`ROBOT_CFG`** | 任意对象（通常为 `@dataclass`） | 传给 `run_task_queue(robot_cfg=...)`、构造 `ROS2RobotInterface`、录制与推理 |

`ROBOT_CFG` 的具体类型由各示例仓库自定义；**`ros2_robot_interface`** 中的 **`ROS2RobotInterfaceConfig`** 通常作为其字段 **`ros2_interface`**（或通过 `build_ros2_interface_from_robot_cfg` 所识别的结构）出现。

---

## 3. 运动运行时对 `ROBOT_CFG` 的依赖

**`task_runtime.runner.run_task_queue`** 与 **`build_queue_runtime_context`** 等路径会直接读取（属性访问）的常见字段包括：

- **`gripper_control_mode`**：例如 `target_command`，用于解析开合夹爪数值。
- **`base_link_entity_path`**：Isaac 中 base link 的 prim 路径；用于 stamped 帧名截取与实体位姿服务。
- **`fsm_switch_delay`**、**`post_reset_wait`**：FSM 切换与 reset 后等待。
- **`arrival_timeout`**、**`arrival_poll`**、**`gripper_action_wait`**：阶段到达与夹爪节拍。

环境 reset 等还会用到任务侧合并配置（如 `MergedQueueConfig` 中的 pick 物体路径），与 **`ROBOT_CFG`** 分工不同：前者多来自 **YAML**，后者多描述**机器人与接口**。

**录制**（`dataset_recording`）通常还要求 **`cameras`**、**`depth_camera_name`**、**`depth_info_topic`** 等字段；若缺失，仅影响观测与深度相关功能，不一定影响纯运动队列。

---

## 4. 与 `ROS2RobotInterface` 的衔接

**`robot_action_composer.ros_interface_utils.build_ros2_interface_from_robot_cfg`** 根据各项目的 **`ROBOT_CFG`** 形状构造 **`ROS2RobotInterface`**。具体字段以 **`ros2_robot_interface`** 包内 **`ROS2RobotInterfaceConfig`** 及示例 `robot_config.py` 为准。

---

## 5. 参考示例

在 monorepo 中可直接对照：

- `examples/IsaacSim/robots/DobotCR5/robot_config.py`（单臂 + 相机）
- `examples/IsaacSim/robots/Agibot_G1/robot_config.py`（双臂等）

将新机器人放入同一 `robots/` 树下并满足上述约定后，`load_motion_entries` 即可自动发现。
