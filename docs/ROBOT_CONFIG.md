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
| `ee_target_frame` | `""` | 任务末端位姿默认所属 link，如 `{arm}_eef`。空则直接当运控 TCP 下发 |
| `ee_command_frame` | `""` | 运控实际跟踪的 link。`ee_target_frame` 非空且此处为空时用 `{arm}_tcp` |
| `ee_tip` | — | USD tip→command 静态链（与 `ee_target_frame` TF 互斥）。见下文 |

### `ee_tip`（USD tip→parent，可再 TF→OCS2）

场景夹爪朝向与 ROS description 默认腕系不一致时（例如把 `link7/tcp` 拧过再挂 EE），用 USD `/get_prim_attribute` 读 parent→tip，把 tip 系目标换成 parent。若运控跟踪的不是 parent（W2 OCS2 多为 `{arm}_eef`），再设 `ee_command_frame`，用 live TF 做第二跳：`parent_frame`→command。

| 字段 | 说明 |
|------|------|
| `ee_tip.{left\|right}.parent_prim` | tip→command 的 USD parent（可为 description 腕系 `tcp_origin`，或挂 EE 的 `link7/tcp`） |
| `ee_tip.{left\|right}.tip_prim` | 抓取语义 tip（如 `.../pad_tcp`） |
| `ee_tip.{left\|right}.parent_frame` | parent 对应的 TF 名；空则 `{arm}_tcp` |
| `ee_command_frame` | OCS2 跟踪 link；有 `ee_tip` 时做 parent→command（如 `{arm}_eef`） |

```yaml
ee_command_frame: "{arm}_eef"
ee_tip:
  right:
    parent_prim: /World/Robot/Right_Arm/link7/tcp_origin
    tip_prim: /World/Robot/Right_Arm/link7/tcp/RG75/.../pad_tcp
    parent_frame: "{arm}_tcp"
```

有 `ee_tip` 时不再走 `ee_target_frame`（任务位姿按 tip 解释）。`single_arm.pick` 链路：

1. handle（或 object）世界朝向相对机器人世界朝向 → 机器人系下朝向  
2. 套到 tip（未写 `ee_base_orientation` 时 tip // handle）  
3. USD `tip→parent(tcp_origin)`  
4. live TF `parent_frame→ee_command_frame`（如 `{arm}_tcp→{arm}_eef`）后下发

### `ros2_interface.preset`

`ros2_robot_interface` 在 `connect()` 时会自动探测双臂、夹爪、arm/body/head topic 等。单臂未在图上看到关节命令话题时，会按约定回退到 `/ocs2_arm_controller/target_joint_position`（与默认 MoveL/MoveC action 一致），因此像 Dobot CR5 / FiveAges W2 这类标准栈通常直接用 `auto` 即可。

| preset | 用途 |
|--------|------|
| `auto`（默认） | 仅设置到位阈值，其余靠 auto-detect（含单臂 MoveJ 约定回退） |
| `single_arm` | 单臂非标准夹爪（如 Realman RM75） |
| `ocs2_single_arm` | 显式预置单臂 `ocs2_arm_controller` 话题；命名空间或需在 detect 前写死关节/夹爪话题时用 |

**示例 — 双臂 / 单臂标准栈（最简，preset=auto）：**

```yaml
key: galbot_one
label: Galbot One
base_link_entity_path: /World/Galbot_One/Chassis/base_link/base_link
```

**示例 — 单臂非标准夹爪（需 `single_arm` / `ocs2_single_arm` 覆盖）：**

```yaml
key: realman_rm75
label: Realman RM75
ros2_interface:
  preset: single_arm
  gripper_joint_name: ...
  gripper_command_topic: ...
```

发现结果中的 `robot_dir_relpath`（相对 `robots/`）用于 CLI 按厂商文件夹分组展示；扁平布局（无 `/`）时菜单与原先一致。

## 场景项目（`fa-project-usd`）的 `robot.yaml`

FaSim 的 USD 项目目录也可以直接当 `motion-generation` 工作区。识别规则：目录含 **`env/`**，
任务 YAML 放 **`<项目>/task_configs/**`**，机器人配置由 **`<项目>/robot.yaml`** 以增量覆盖的方式引用共享 profile：

```yaml
# <项目>/robot.yaml
extends: fiveages_w2                              # 共享 profile 的 robot key
robots_root: ${LEROBOT_ROOT}/examples/IsaacSim/robots

default_env_scene: env/Anigon/default_v2.usda     # 任务没写 env_scene 时的兜底
base_link_entity_path: /World/robot/FiveAges_W2/Linkhou_S2/base_footprint/base_link
post_reset_wait: 3.0

ros2_stack:
  motion:
    required: false                               # 运控由 USD 内 ROS=fiveages 拉起
  navigation:
    required: auto
    profile: map_only
    args:
      map: german_poc
```

规则：

- `extends` 先按 `robots_root` 下各 `robot.yaml` 的 `key` 匹配；含 `/` 或以 `.` 开头时按相对路径解析。
- 覆盖只合并 motion 字段与 `ros2_interface` / `ros2_stack`；**不允许改 `key`**，否则报错。
- `robots_root` 支持 `${VAR}` / `${VAR:-default}` / `~`；优先级：CLI `--robots-root` > 项目 `robot.yaml` > 环境变量（`ROBOT_ACTION_COMPOSER_ROBOTS_ROOT`、`LEROBOT_ISAACSIM_ROBOTS`）。
- `${LEROBOT_ROOT}` 未导出时会自动用当前 lerobot 检出目录（editable 安装的上级目录），所以正常克隆下来不用先 export。
- 项目里的 `robots/` 如果只有 USD 图层，不会被当作旧版 `robots/<Robot>/` 布局；旧式布局要求真的存在带 `robot.yaml` + `task_configs/` 的 profile 目录。

## 字段划分（motion vs lerobot）

| 字段 | motion (`robot.yaml`) | lerobot (`lerobot_config.py`) |
|------|:---------------------:|:-----------------------------:|
| `ros2_interface` | ✓ | （录制时复用 motion 侧） |
| `gripper_control_mode` | ✓ | ✓ |
| `base_link_entity_path` | ✓ | — |
| `post_reset_wait`, `fsm_switch_delay` | ✓ | ✓（推理） |
| `arrival_*`, `gripper_action_wait`, `ee_target_frame`, `ee_command_frame`, `ee_tip` | ✓ | — |
| `robot_id`, `cameras`, `depth_*` | — | ✓ |

## 安装与环境

- **仅任务编排**：`./init.sh all-motion` 或菜单 6（无需 PyPI `lerobot`，也不装灵巧手）。
- **录制 / 推理**：`./init.sh install-lerobot` 或菜单 5 / 7。
- **可选灵巧手**：`./init.sh all-grasp` 或菜单 `g`。详见仓库根 `docs/SETUP.md`。

## 发现机制

- 旧式工作区：`load_motion_entries()` 扫描 `robots/<Robot>/` 与 `robots/<Vendor>/<Robot>/`（一层分组），优先加载 `robot.yaml`；兼容 `motion_config.py`、`robot_config.py`（带 deprecation 警告）。
- USD 项目工作区：`load_workspace_layout()` 先用项目 `robot.yaml` 解析 `extends` / `robots_root`，再扫描 `<项目>/task_configs/**`；每条 entry 带绝对 `robot_dir`、`scene_project`、`default_env_scene`。
- `load_lerobot_profile()` 加载 `lerobot_config.py`（录制 / 推理入口调用），位置始终是共享 profile 目录（不在 USD 项目里）。

## 过渡期

仍支持遗留 `motion_config.py`（`ROBOT_KEY` / `ROBOT_LABEL` / `MOTION_CFG`）与合并的 `robot_config.py`；请迁移为 `robot.yaml` + 可选 `lerobot_config.py`。
