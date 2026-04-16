# 任务配置：YAML 说明

本文件描述 **`robot_action_composer`** 如何加载任务 YAML，以及字段约定。  
**机器人目录与 `robot_config.py`** 见同目录下的 **[ROBOT_CONFIG.md](ROBOT_CONFIG.md)**。

在 monorepo 中，典型资产根目录为 **`examples/IsaacSim`**，任务文件位于 `examples/IsaacSim/robots/<Robot>/task_configs/`；`motion_main` / `record_main` / `registry_loader` 会把该路径传给 `discover_task_configs`。

---

## 格式

任务发现由 **`robot_action_composer.task_config_io.discover_task_configs`** 完成，**递归**加载 `task_configs/` 及其**子文件夹**内的 **`*.yaml` / `*.yml`**（路径中含 `.` 开头段或 `__pycache__` 的会被忽略），返回 **`TaskConfigDiscovery`**：`tasks` 为全局扁平的 `task_key -> 配置`；**`task_groups`** 按**一级子文件夹**分组（见下节）。可选嵌套 `pick` / `place` 以及 `carry` / `drawer` 等由 **`flatten_queue_task_overrides`** 摊平；**禁止**在 `base_task_overrides` 或场景根下再写嵌套 **`handover:`**（已移除）。**双臂同步交接段**几何在 **`skill_defaults.dual_arm.handover`**（`HandoverSyncConfig`：交接点位置与双臂姿态）；**接收臂放置**用 **`skill_defaults.single_arm.place`**（`place_position` / `place_orientation` 等）。场景覆盖仍用 **`skill_params`** 下对应键。`motion_generation`、`record_datasets`、`inference.py` 在扁平化后按 **pick → handover → carry → place → drawer** 顺序叠 `skill_defaults`。其中 **`dual_arm.carry`** 与 **`dual_arm.place`** 会分别解析为 **`MergedQueueConfig.carry`** 与 **`MergedQueueConfig.place`**（两套扁平表，避免放置几何覆盖搬运几何）。

在每个**目录**内，每个**基名**（如 `pick_place`）对应**一个** YAML 文件（`pick_place.yaml` 或 `pick_place.yml`）。**同一目录下同基名不能同时存在 `.yaml` 与 `.yml`**，否则会 `ValueError`。不同子目录下可以有相同**文件名**（不同基名路径），但各文件内的 **`task_key`** 在整台机器人范围内必须**唯一**，重复时会 `ValueError` 并指出两个文件路径。

### 一级文件夹与交互选单

**分组 id**：直接放在 `task_configs/` 下的 YAML 归入 **`task_groups[""]`**（CLI 菜单标签为 **Top level**）；路径为 `task_configs/<folder>/...` 的归入 **`<folder>`**（更深层级仍算在该 `<folder>` 下，便于在子目录中继续整理文件）。**程序引用仍只用全局唯一的 `task_key`**，与是否放在子文件夹无关。

当某机器人存在**多于一个**分组且该入口需要交互选任务时，会先提示 **Select task folder**，再 **Select task**；若只有一个分组，行为与原先单列表一致。`inference.py` 的 **`--task`** 等非交互参数不变。

**不再支持** Python 任务模块（`TASK_CONFIG` / `FLOW_CONFIG` 的 `.py` 文件）。

### `arm` 与 pick / place（避免全局扁平表互相覆盖）

可选流程级字段 **`base_link_entity_path`**（字符串）：Isaac 中机器人基座 link 的 **Prim 全路径**。写在 **`base_task_overrides`** 或场景预设根（与 `pose_tol_pos` 同级）时，会覆盖对应 **`robot_config.py`** 里 `ROBOT_CFG.base_link_entity_path`，用于 stamped `frame_id`、基座位姿查询与导航后刷新。未设置时仍使用机器人配置中的默认值。

**`pose_tol_pos` / `pose_tol_ori`**（写在 **`base_task_overrides`** 或场景根，进入 **`QueueSliceCommon`**）：队列里 **笛卡尔段** 等待左右臂到位时，会传给 **`execute_stage_sequence`** → **`ROS2RobotInterface.wait_until_arrive`**，覆盖各臂 handler 默认的位姿容差；未设置则仍用 **`ros2_robot_interface`** 配置里的默认阈值。

构造 **`MergedQueueConfig`** 时会调用 **`merge_flat_with_skill_pick_place`**，把 ``skill_defaults.single_arm.pick`` 与 ``single_arm.place`` 依次叠到**同一张**扁平 preset 上。若二者**都**含有 **`arm`**（典型如 **handover**：抓臂与放臂不同），则 **place 的 `arm` 不会写入这张全局 flat**，以免后写覆盖 **`QueueSliceCommon.arm`** 导致 **`ctx.task_cfg.common.arm`**、handover 同步段等误判工作臂。

**place 的 `arm`** 仍随完整的 ``skill_defaults['single_arm.place']`` 经 **`merge_task_queue_skill_params`** 注入到 **`single_arm.place` 队列块** 的 `params`，执行时由 **`overlay_queue_single_arm_from_params`** 仅作用于该步。

若 **pick 未提供 `arm`**、仅 **place** 提供，则 place 的 `arm` 仍会进入全局 flat（常见单臂 pick+place 同一臂）。

### `dual_arm.carry` 搬运几何（`BimanualCarryTaskConfig`）

写在 **`skill_defaults.dual_arm.carry`** 或 **`scene_presets.*.skill_params.dual_arm.carry`**；经 **`merge_flat_with_skill_carry`** 等叠入 **仅搬运用** 扁平 preset，由 **`build_merged_queue_from_flat(merged_flat_carry, merged_flat_place)`** 解析为 **`MergedQueueConfig.carry`**（与 **`place`** 分表，互不覆盖）。列表向量在 YAML 中加载后会规范为 **tuple**。

| 字段 | 含义 |
|------|------|
| `source_object_entity_path` | 搬运对象 Prim 路径 |
| `carry_half_span_y` | 相对物体中心，左右手在横向（典型为物体 **Y**）上的**半间距**基准（米）；与 `carry_xyz[1]` 相加得到有效半宽 |
| `carry_approach_clearance_y` | 仅 **Approach / Forward** 段在横向再张开的余量（米） |
| `carry_prepare_offset` | 预闭合段相对抓取点的偏移（米，三轴） |
| `carry_xyz` | 物体中心到名义抓取点的平移（物体系，米） |
| `carry_left_orientation` / `carry_right_orientation` | 左右末端姿态四元数 **x,y,z,w** |
| `carry_lift_xyz` / `carry_retreat_xyz` | 抬起与后退相对抓取点的平移（米）；不写或 `null` 可跳过对应段 |

物体位置扰动请用 **`env.randomize_object_local_xyz`**（与 **`dual_arm.carry`** 几何字段无关）。

### `dual_arm.place` 放置（`BimanualPlaceTaskConfig`）

**简化**：若只关心放置目标与相对位移，可只写 `place_object_entity_path` 与 `place_offset`（`[x,y,z]`，物体系米），**不写** `place_half_span_y` 等；姿态、`place_prepare_offset`、`place_lift_xyz` / `place_retreat_xyz` 等与 **`dual_arm.carry` 相同**（故同一任务须配置 carry）。仍可在场景 `skill_params.dual_arm.place` 里覆盖个别字段（如 `place_retreat_xyz`）。

### `dual_arm.place` 完整字段（`BimanualPlaceTaskConfig`）

写在 **`skill_defaults.dual_arm.place`** 或 **`scene_presets.*.skill_params.dual_arm.place`**；与 **`dual_arm.carry`** **独立合并**，解析为 **`MergedQueueConfig.place`**（字段前缀均为 **`place_*`**，与搬运的 **`carry_*`** 分离，可单独调参）。

| 字段 | 含义 |
|------|------|
| `place_object_entity_path` | 放置参考（货架/槽位等）Prim 路径 |
| `place_half_span_y` | 相对放置参考中心，左右手横向半间距基准（米）；与 `place_xyz[1]` 相加 |
| `place_approach_clearance_y` | 接近 / 张开段横向余量（米） |
| `place_prepare_offset` | 预接近相对名义点的偏移（米，三轴） |
| `place_xyz` | 参考中心到名义释放点的平移（物体系，米） |
| `place_left_orientation` / `place_right_orientation` | 左右末端四元数 **x,y,z,w** |
| `place_lift_xyz` / `place_retreat_xyz` | 抬升与后撤相对名义点的平移（米）；不写或 `null` 可跳过与 carry 对称的逆段 |

队列技能名：

- **`env.randomize_object_local_xyz`**：对**显式** `object_entity_path` 的 prim 做局部平移均匀随机（`xyz_offset`、`enabled`）；与 `dual_arm.carry` 几何无关。`run_task_queue(reset_env=true)` 时 **Isaac 仅 reset/play + settle，不在 reset 内随机物体**；若需要扰动，请把本技能放在 **`task_queue` 第一个顺序块**（若首块为 `parallel`，建议在其**之前**单独一步，避免与 reset  settle 竞态）。
- **`dual_arm.place_advance`**：两段笛卡尔（送入：抬升接近位 → 下降至合拢释放位），夹爪**闭合**持箱；写入 **`ctx.place_object_center`**。
- **`dual_arm.place_release`**：合拢位**张开**夹爪。
- **`dual_arm.place_spread_retreat`**：两段（**Y 向张开** → 后撤至预接近位），夹爪张开。
- **`dual_arm.place`**：上述 5 段一次执行（等价于 advance + release + spread_retreat）。
- **`dual_arm.place_relative`**：不查货架 prim；读**当前**左右末端位姿（系为 ``ctx.frame_id``，一般为 base）→ **同加** `translation_xyz`（持箱）→ 松爪 → 在 **XY 平面**沿「右→左」各外张 `spread_half` → 再同加 `retreat_xyz` 后撤；姿态不变。默认 `spread_half` / `retreat_xyz` 可取 `dual_arm.carry` 的 `carry_approach_clearance_y` / `carry_retreat_xyz`。
  - **`motion_frame_id`**（或 **`relative_frame_id`**）：可选。若设为随腰/躯干转的 TF 系（与 ``ctx.frame_id`` 不同），则先用 ``ROS2RobotInterface.transform_pose`` 将当前末端位姿变到该系，再算平移与外张，且本块目标以该系发布；这样腰转 90°/180° 时「左右外张」仍相对身体一致，无需手写符号翻转。可选 **`tf_lookup_timeout`**（秒，默认 2.0）。

笛卡尔段与 **`build_bimanual_carry_sequence`** 对称、阶段顺序为其逆（实现名见 **`BIMANUAL_PLACE_SUFFIXES`**）；相对放置见 **`build_bimanual_place_relative_sequence`**（**`PLACE_RELATIVE_SUFFIXES`**）。

---

## `task_queue` 与技能名

队列由 **`task_queue`** 列出若干**块**；每块至少含 **`skill:`**（注册名），可选 **`id:`**（用于 `skill_defaults` / `scene_presets.skill_params` 里按块覆写参数）、可选 **`params:`**（该块专用映射）。

- **并行块**：一项为 **`parallel:`**，值为子块列表（子块不可再嵌套 `parallel`）。见 **`types.ParallelSpec`** 注释中的 YAML 示例。
- **交接（handover）典型顺序**（参见各机器人 `handover.yaml`）：
  1. **`robot.cache_ee_pose`**（`which: both`，写入 `scratch[key]`）→ `single_arm.pregrasp` → `single_arm.pick` → `dual_arm.handover_sync` → `single_arm.place` → **`dual_arm.goto_cache_pose`**（同一 `key`，双臂同步回起势末端位姿）。
- **后退再回「队列起势」**：**`nav.navigate_backup`**（`distance_m`，沿 base +X 反向平移）→ **`parallel`**：`nav.send_nav_goal`（如 `x/y/yaw=0`）与 **`joint.movej_to_config`**（仅躯干时可设 **`skip_fsm_change: true`**，跳过该技能内 FSM 切换）→ **`nav.wait_nav_arrived`** → **`dual_arm.goto_cache_pose`**（若任务开头已 **`robot.cache_ee_pose` `which: both`**）。参见 `FiveAges_W2/navigate_and_carry.yaml`。
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
