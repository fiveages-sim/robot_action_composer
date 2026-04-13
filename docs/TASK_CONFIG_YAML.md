# 任务配置：YAML 说明

本文件描述 **`robot_action_composer`** 如何加载任务 YAML，以及字段约定。  
**机器人目录与 `robot_config.py`** 见同目录下的 **[ROBOT_CONFIG.md](ROBOT_CONFIG.md)**。

在 monorepo 中，典型资产根目录为 **`examples/IsaacSim`**，任务文件位于 `examples/IsaacSim/robots/<Robot>/task_configs/`；`motion_main` / `record_main` / `registry_loader` 会把该路径传给 `discover_task_configs`。

---

## 格式

任务发现由 **`robot_action_composer.task_config_io.discover_task_configs`** 完成，**仅**加载给定目录下的 **`*.yaml` / `*.yml`**。可选嵌套 `pick` / `place` 以及 `carry` / `drawer` 等由 **`flatten_queue_task_overrides`** 摊平；**禁止**在 `base_task_overrides` 或场景根下再写嵌套 **`handover:`**（已移除）。**双臂同步交接段**几何在 **`skill_defaults.dual_arm.handover`**（`HandoverSyncConfig`：交接点位置与双臂姿态）；**接收臂放置**用 **`skill_defaults.single_arm.place`**（`place_position` / `place_orientation` 等）。场景覆盖仍用 **`skill_params`** 下对应键。`motion_generation`、`record_datasets`、`inference.py` 在扁平化后按 **pick → handover → carry → drawer** 顺序叠 `skill_defaults`。

每个**基名**（如 `pick_place`）对应**一个** YAML 文件（`pick_place.yaml` 或 `pick_place.yml`）。**同一基名不能同时存在 `.yaml` 与 `.yml`**，否则会 `ValueError`。

**不再支持** Python 任务模块（`TASK_CONFIG` / `FLOW_CONFIG` 的 `.py` 文件）。

### `arm` 与 pick / place（避免全局扁平表互相覆盖）

可选流程级字段 **`base_link_entity_path`**（字符串）：Isaac 中机器人基座 link 的 **Prim 全路径**。写在 **`base_task_overrides`** 或场景预设根（与 `pose_tol_pos` 同级）时，会覆盖对应 **`robot_config.py`** 里 `ROBOT_CFG.base_link_entity_path`，用于 stamped `frame_id`、基座位姿查询与导航后刷新。未设置时仍使用机器人配置中的默认值。

构造 **`MergedQueueConfig`** 时会调用 **`merge_flat_with_skill_pick_place`**，把 ``skill_defaults.single_arm.pick`` 与 ``single_arm.place`` 依次叠到**同一张**扁平 preset 上。若二者**都**含有 **`arm`**（典型如 **handover**：抓臂与放臂不同），则 **place 的 `arm` 不会写入这张全局 flat**，以免后写覆盖 **`QueueSliceCommon.arm`** 导致 **`ctx.task_cfg.common.arm`**、handover 同步段等误判工作臂。

**place 的 `arm`** 仍随完整的 ``skill_defaults['single_arm.place']`` 经 **`merge_task_queue_skill_params`** 注入到 **`single_arm.place` 队列块** 的 `params`，执行时由 **`overlay_queue_single_arm_from_params`** 仅作用于该步。

若 **pick 未提供 `arm`**、仅 **place** 提供，则 place 的 `arm` 仍会进入全局 flat（常见单臂 pick+place 同一臂）。

---

## `task_queue` 与技能名

队列由 **`task_queue`** 列出若干**块**；每块至少含 **`skill:`**（注册名），可选 **`id:`**（用于 `skill_defaults` / `scene_presets.skill_params` 里按块覆写参数）、可选 **`params:`**（该块专用映射）。

- **并行块**：一项为 **`parallel:`**，值为子块列表（子块不可再嵌套 `parallel`）。见 **`types.ParallelSpec`** 注释中的 YAML 示例。
- **交接（handover）典型顺序**（参见各机器人 `handover.yaml`）：
  1. **`robot.cache_ee_pose`**（`which: both`，写入 `scratch[key]`）→ `single_arm.pregrasp` → `single_arm.pick` → `dual_arm.handover_sync` → `single_arm.place` → **`dual_arm.goto_cache_pose`**（同一 `key`，双臂同步回起势末端位姿）。
- **后退再回「队列起势」**：**`nav.navigate_backup`**（`distance_m`，沿 base +X 反向平移）→ **`parallel`**：`nav.send_nav_goal`（如 `x/y/yaw=0`）与 **`joint.movej_to_config`**（仅躯干时可设 **`skip_fsm_hold: true`**，避免并行时先发 `HOLD` 打断 Nav2）→ **`nav.wait_nav_arrived`** → **`dual_arm.goto_cache_pose`**（若任务开头已 **`robot.cache_ee_pose` `which: both`**）。参见 `FiveAges_W2/navigate_and_carry.yaml`。
- **会话暂存 / 状态快照**：**`session.scratch_put`**（`key` + `value`）、**`session.scratch_clear`**（可选 `prefix`）；**`robot.snapshot_state`**（可选 `key`，默认 `robot_state`，将关节状态与双臂末端位姿等写入 **`ctx.scratch`**，供后续自定义 skill 用 **`ctx.scratch_get`** 读取）。
- **缓存回程点**：**`robot.cache_ee_pose`** 写在 **`task_queue` 靠前位置**（常见为 reset/OCS2 后首步，或 pregrasp 之后），写入 **`ctx.scratch[key]`**；Runner **不再**在连接时缓存 Cartesian home 或初始关节角。
  - **`which`**（可选）：**`work`**（默认，按 `common.arm` 单臂，扁平 `pose+gripper`）、**`left`** / **`right`**（显式单臂）、**`both`**（双臂结构 `{left, right}`，每侧 `pose+gripper`；可选 **`left_gripper`** / **`right_gripper`** 覆盖 **`gripper`**）。
  - 回程：**`single_arm.goto_cache_pose`**（单臂扁平；若缓存为双臂结构则按 **`side`** 或 `common.arm` 选一侧，并用该侧 EE **`frame_id`**）；**`dual_arm.goto_cache_pose`**（要求 `which: both` 的缓存，双臂同步，可选 **`gripper`** / **`left_gripper`** / **`right_gripper`**）。
  - 数据集 **多回合录制** 若仍需关节空间回第一帧姿态，由 **`dataset_recording`** 在回合间内部调用 **`movej_return_to_initial_state`**（非 `task_queue` 技能）。参见 `DobotCR5/task_configs/pick_place.yaml`。
- **缓存关节角 / 关节回程**：**`robot.cache_joint_state`**（`key`、可选 **`include_body`**、**`wait_timeout`**）把当前左/右臂与可选躯干关节写入 **`scratch[key]`**（``{left, right, body}`` 列表或 null）；**`joint.goto_cached_joints`** 用同一 **`key`** 调用与录制相同的 MoveJ 到位逻辑；若下一步是笛卡尔 skill，设 **`resume_ocs2: true`**（与 **`joint.movej_to_config`** 一致）。

其他常用技能见 **`docs/ARCHITECTURE.md`** 第 4.2 节技能列表。

---

## 依赖

解析 YAML 需要 **PyYAML**：

```bash
pip install pyyaml
```

（子模块 `ros2_robot_interface` 的 `pyproject.toml` 已声明 `pyyaml`，随该环境安装即可。）

---

## 类型注意

- YAML 里没有 Python `tuple`，向量请写成列表，例如 `grasp_orientation: [0.7, 0.7, 0.0, 0.0]`。加载时会将**仅含数字的非空列表**规范化为 `tuple`，供队列运行时 dataclass（如 `QueueSingleArmSlice`、`HandoverSyncConfig`）使用。
- `null` → Python `None`。
- 布尔值为 `true` / `false`（小写，YAML 1.2 常见写法）。

---

## 示例（IsaacSim 仓库内路径）

- `examples/IsaacSim/robots/DobotCR5/task_configs/pick_place.yaml`
- `examples/IsaacSim/robots/DobotCR5/task_configs/drawer_pick_place.yaml`（抽屉拉开距离为 **`skill_defaults.single_arm.drawer.pull_distance`**（`DrawerGeometryConfig`）；抓苹果 retreat 用 **`single_arm.pick.retreat_direction_extra`**。）
- `examples/IsaacSim/robots/Agibot_G1/task_configs/pick_place.yaml`、`handover.yaml`
- `examples/IsaacSim/robots/Realman_RM75/task_configs/pick_place.yaml`
- `examples/IsaacSim/robots/Marvin_M6CCS/task_configs/handover.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/bimanual_carry.yaml`、`navigate_and_carry.yaml`
