# 机器人目录与配置约定

每个 `robots/<RobotName>/` 目录包含任务编排与（可选）LeRobot 录制/推理配置。

## 目录布局

支持两种路径（仅一层厂商分组）：

```
robots/FiveAges_W2/          # 扁平
  robot.yaml
  task_configs/

robots/galbot/Galbot_One/    # 按厂商分组
  robot.yaml
  task_configs/
```

每个机器人目录需包含 `robot.yaml`（或遗留 motion 配置）与 `task_configs/`。`robots/<Vendor>/` 仅作分组时自身不应同时具备上述文件。

| 文件 | 内容 | 依赖 | 用途 |
|------|------|------|------|
| `robot.yaml` | `key`、`label`、motion 字段、`ros2_interface`、可选 `ros2_stack` | `pyyaml`；构建时懒加载 `ros2_robot_interface` | `motion-generation`、任务队列执行、`ros2-stack` |
| `lerobot_config.py` | `LEROBOT_CFG` | 无（类型定义在 composer 内） | `record_datasets`、IsaacSim `inference.py` |
| `task_configs/<leaf>/.meta/ros2_stack.yaml` | 场景级运控/导航启动覆盖 | `pyyaml` | `ros2-stack`、`motion-generation --ensure-ros2-stack`（见 [`ROS2_STACK.md`](ROS2_STACK.md)） |

解析结果为 `MotionRobotConfig`（见 `robot_action_composer.config.robot_profiles`）。

## `robot.yaml` 格式

```yaml
key: my_robot
label: My Robot
base_link_entity_path: /World/.../base_link   # Isaac Sim 专用
gripper_action_wait: 2.0                      # 仅在与默认值不同时填写

ros2_interface:                               # 可选；省略则 preset=auto
  preset: auto                                # auto | single_arm | ocs2_single_arm
  pose_position_threshold: 0.02
  pose_orientation_threshold: 0.05

ros2_stack:                                   # 可选；场景运控/导航启动默认，见 ROS2_STACK.md
  defaults:
    args:
      robot: my_robot
      hardware: isaac
  motion:
    required: true
    preset: ocs2-fullbody                     # ocs2-fullbody | ocs2-split-body | ocs2-demo
  navigation:
    required: auto
    profile: default                          # default | map_only
```

motion 相关字段可写在根级，或集中在 `motion:` 下（与根级合并，根级优先）：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `gripper_control_mode` | `target_command` | 夹爪控制模式 |
| `base_link_entity_path` | `""` | Isaac Sim base_link Prim 路径 |
| `fsm_switch_delay` | `0.1` | 状态机切换延迟（秒） |
| `post_reset_wait` | `1.0` | reset 后等待（秒） |
| `arrival_timeout` | `3.0` | 到位超时（秒） |
| `arrival_poll` | `0.05` | 到位轮询间隔（秒） |
| `gripper_action_wait` | `0.3` | 夹爪动作后等待（秒） |

### `ros2_interface.preset`

`ros2_robot_interface` 在 `connect()` 时会自动探测双臂、夹爪、arm/body/head topic 等。

| preset | 用途 |
|--------|------|
| `auto`（默认） | 仅设置到位阈值，其余靠 auto-detect |
| `single_arm` | 单臂非标准夹爪（如 Realman RM75） |
| `ocs2_single_arm` | 单臂 + `ocs2_arm_controller`（如 Dobot CR5） |

**示例 — 双臂标准栈（最简）：**

```yaml
key: galbot_one
label: Galbot One
base_link_entity_path: /World/Galbot_One/Chassis/base_link/base_link
```

**示例 — 单臂 OCS2 + 自定义夹爪：**

```yaml
key: dobot_cr5
label: Dobot CR5
base_link_entity_path: /World/CR5/base_link
ros2_interface:
  preset: ocs2_single_arm
  gripper_joint_name: gripper_joint
  gripper_command_topic: gripper_joint/position_command
  left_gripper_controller_name: gripper_controller
  left_gripper_target_percent_topic: /gripper_controller/target_percent
```

发现结果中的 `robot_dir_relpath`（相对 `robots/`）用于 CLI 按厂商文件夹分组展示；扁平布局（无 `/`）时菜单与原先一致。

## 字段划分（motion vs lerobot）

| 字段 | motion (`robot.yaml`) | lerobot (`lerobot_config.py`) |
|------|:---------------------:|:-----------------------------:|
| `ros2_interface` | ✓ | （录制时复用 motion 侧） |
| `gripper_control_mode` | ✓ | ✓ |
| `base_link_entity_path` | ✓ | — |
| `post_reset_wait`, `fsm_switch_delay` | ✓ | ✓（推理） |
| `arrival_*`, `gripper_action_wait` | ✓ | — |
| `robot_id`, `cameras`, `depth_*` | — | ✓ |

## 安装与环境

- **仅任务编排**：`./init.sh all-motion` 或菜单 3 / 5（无需 PyPI `lerobot`）。
- **录制 / 推理**：`./init.sh install-lerobot` 或菜单 4 / 6 / `./init.sh all`。

## 发现机制

- `load_motion_entries()` 扫描 `robots/<Robot>/` 与 `robots/<Vendor>/<Robot>/`（一层分组），优先加载 `robot.yaml`；兼容 `motion_config.py`、`robot_config.py`（带 deprecation 警告）。
- `load_lerobot_profile()` 加载 `lerobot_config.py`（录制 / 推理入口调用）。

## 过渡期

仍支持遗留 `motion_config.py`（`ROBOT_KEY` / `ROBOT_LABEL` / `MOTION_CFG`）与合并的 `robot_config.py`；请迁移为 `robot.yaml` + 可选 `lerobot_config.py`。
