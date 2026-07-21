# 任务配置 YAML 指南（精简版）

本文聚焦三件事：

1. 一个任务 YAML 应该怎么组织
2. 加载与合并规则是什么
3. 写 `task_queue` 时最常用的模式有哪些

**与 [`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md) 的分工**：本文不写各 `skill` 的参数字段含义与默认值；**每个技能能填什么键、语义是什么**，以技能参考为准。

技能参数明细请看：[`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md)  
架构与代码路径请看：包根目录 [`README.md`](../README.md)（**架构与设计**）

---

## 1) YAML 结构总览

一个任务 YAML 通常由三块核心内容组成：

- `task_queue`：流程本身（按什么顺序跑哪些技能）
- `skill_defaults`：该任务的默认技能参数
- `scene_presets`：按场景做增量覆盖（通常只改少数字段）

最小骨架示例：

```yaml
task_key: pick_place
label: Pick Place
default_scene: scene_a

runtime_defaults:
  base_link_entity_path: /World/robot/base_link
  max_stage_duration: 2.0
  pose_tol_pos: 0.025
  pose_tol_ori: 0.08

skill_defaults:
  single_arm.pick:
    arm: right
    object_prim_path: /World/object
  single_arm.place:

task_queue:
  - skill: robot.cache_ee_pose
    params: { key: prep_pose }
  - skill: single_arm.pick
  - skill: single_arm.place
  - skill: single_arm.goto_cache_pose
    params: { key: prep_pose }

scene_presets:
  scene_a:
    skill_params:
      single_arm.pick:
        object_prim_path: /World/object_a
  scene_b:
    skill_params:
      single_arm.pick:
        object_prim_path: /World/object_b
```

实践建议：

- 先排 `task_queue`，再补 `skill_defaults`
- 场景差异放 `scene_presets.*.skill_params`，避免复制整份 YAML
- 同一技能出现多次时给块加 `id`，便于按块覆盖

---

## 2) 加载与分组规则

### 文件发现

- 递归加载 `task_configs/` 下所有 `*.yaml` / `*.yml`
- 忽略隐藏路径段（`.` 开头）与 `__pycache__`
- 返回 `TaskConfigDiscovery`：
  - `tasks`：全局 `task_key -> 配置`
  - `task_groups`：按**叶子目录**相对路径分组（posix，如 `siemens/nested`）

**与 ROS2 栈启动配置的边界**：运控 / 导航启动写在叶子目录 **`.meta/ros2_stack.yaml`**（或 `robot.yaml` 的 `ros2_stack:`），**不是**任务编排文件。隐藏 `.meta/` 不会进入任务发现，也不触发叶子目录规则冲突。详见 [`ROS2_STACK.md`](ROS2_STACK.md)。

### 唯一性约束

- 同目录下，同基名不能同时有 `.yaml` 与 `.yml`
- 不同子目录可有同名文件
- 同机器人范围内，`task_key` 必须全局唯一
- **仅叶子目录可放 YAML**：若某目录既有任务 YAML，其子目录下也有任务 YAML，发现阶段报错  
  （例如同时存在 `siemens/a.yaml` 与 `siemens/nested/b.yaml` 非法；应把 YAML 只放在叶子，如仅保留 `siemens/nested/`）

### 分组交互

- 根目录文件属于 `task_groups[""]`（CLI 显示 Top level）
- `task_configs/<a>/<b>/foo.yaml` 属于分组 `a/b`（完整父目录路径）
- 有多个叶子分组时，CLI **逐级下钻**选目录（每层只显示下一段名），再到叶子后选 task；单层仅一个子节点时自动进入
- 链式任务整链锁定同一个叶子分组
- 非交互入口（例如 `--task`）仍按 `task_key` 定位

---

## 3) 字段与合并规则

### 顶层常见字段

- `task_key` / `label` / `default_scene`
- `use_stamped`
- `runtime_defaults`
- `skill_defaults`
- `task_queue`
- `scene_presets`
- `objects` / `active_object`（可选；物体清单与默认目标，见下）

### 物体清单（`objects` / `active_object`）

用于声明刚体 prim 与 USD 抓取 frame，避免每个 skill 手写 `object_position_offset`。

**任务 YAML 内联（推荐单任务）：**

```yaml
objects:
  wind_turbo_blade:
    object_prim_path: /World/scene/wind_turbo_blade/turbo_blade
    grasps:
      left: small_side/point_01   # 相对刚体的相对 path
      right: big_side/point_01
active_object: wind_turbo_blade

skill_defaults:
  dual_arm.parallel_pick:
    left_pick:
      grasp_id: left
```

**同叶子多任务复用：** 放到 `<leaf>/.meta/objects/<key>.yaml`（不进任务发现，同 `ros2_stack.yaml`）。合并顺序：

`.meta/objects` → 任务 `objects` → `scene_presets.<scene>.objects`（后写覆盖）。

**链式：** 整链锁定同一叶子；每段按上式**重新合并**该段 task+scene。跨 task 共享 grasps 请用 `.meta`，不要指望 task A 内联自动出现在 task B。

**兼容：** 仅写 `object_prim_path` + `object_position_offset` 的旧 YAML 行为不变。若 skill **显式**写了 `object_position_offset`，优先生效（可覆盖自动 grasp）。

导航 / 放置参考物也可复用同一清单：

```yaml
# .meta/objects/bg_cube.yaml
object_key: bg_cube
object_prim_path: /World/scene/bg_1/collision/Cube

skill_defaults:
  nav_to_bg_cube:
    object_key: bg_cube          # → objects[bg_cube].object_prim_path
  place_on_cube:
    reference_object_key: bg_cube  # 同理；也可写 object_key
    object_position_offset: [0.0, 0.3, 0.2]  # 本任务放置点，仍写在 skill 上
```

显式 `object_prim_path` / `reference_object_prim_path` 仍优先。导航**不会**回退到 `active_object`（避免误导航到抓取目标）。

Newton kit 下勿对子 frame 调 `/get_entity_state` 当世界位姿；自动 offset 走 `/get_prim_attribute` 局部链。缺 `xformOp:translate`/`orient` 的中间 prim 视为恒等。

### `runtime_defaults` 允许键（严格）

`runtime_defaults`（以及 `scene_presets.<scene>` 根层中计入 runtime 的键）仅允许：

- `base_link_entity_path`
- `max_stage_duration`
- `pose_tol_pos`
- `pose_tol_ori`
- （以及实现中已支持的 `gripper_open` / `gripper_closed`）

`skill_params` / `objects` / `active_object` 可写在 scene 根层，但**不**当作 `runtime_defaults` 校验。

其他抓取/放置几何仍写在：

- `skill_defaults.<skill_or_id>`
- `scene_presets.<scene>.skill_params.<skill_or_id>`
- 对应 block 的 `params`

### 合并优先级（低 -> 高）

1. `runtime_defaults`（仅 4 个基础键）
2. `skill_defaults`
3. `scene_presets.<scene>.skill_params`
4. 当前块 `params`

### 运行时切片（`MergedQueueConfig`）

- `single_arm.pick` / `single_arm.place` -> 单臂切片
- `dual_arm.carry` -> `MergedQueueConfig.carry`
- `dual_arm.place` -> `MergedQueueConfig.place`
- `single_arm.drawer` -> `MergedQueueConfig.drawer`

> `carry` 与 `place` 独立合并，避免几何参数互相覆盖。

### 关键补充规则

- pick/place 都有 `arm` 时：全局主臂以 pick 为准；place 的 `arm` 只作用于其块执行。
- `base_link_entity_path` 可在 `runtime_defaults` 或场景根覆盖机器人默认 base prim。
- `pose_tol_pos` / `pose_tol_ori` / `max_stage_duration` 可在 `runtime_defaults` 或场景根配置，作用于**笛卡尔**流程级判定：
  - `max_stage_duration`：覆盖机器人默认 `arrival_timeout`，作为每段笛卡尔到位等待上限（秒）。**不等于** `arm_movel_duration`（后者只改控制器轨迹时长）。
  - `pose_tol_pos`：位置容差（米）。
  - `pose_tol_ori`：姿态容差，单位是四元数距离 `1-|dot|`（无量纲）；runner 会换算成度再交给 `check_arrival`。常见写法 `0.03`～`0.08`，**不要**当成角度度数值。
  - 关节 skill（`joint.*`）仍用块内 `arrival_timeout`，不受 `max_stage_duration` 覆盖。

### 机物偏航对齐（pick / parallel_pick）

导航到位后，机物常有固定相对偏航（如侧对 `±π/2`）。`ee_base_orientation` 按该**标称**标定；运行时只补相对偏差。细节见 [SKILLS_REFERENCE §6.2](SKILLS_REFERENCE.md)。

```yaml
# 0/90/180° 档：自动吸附
dual_arm.parallel_pick:
  ee_orientation_frame: object
  object_orientation_mode: yaw
  aligned_object_yaw: auto
  left_pick: { object_prim_path: /World/obj, ee_base_orientation: [1,0,0,0], ... }
  right_pick: { ... }

# 斜向标称（如 45°）：显式角度；斜抓姿态写在 ee_base_orientation
dual_arm.parallel_pick:
  ee_orientation_frame: object
  object_orientation_mode: yaw
  aligned_object_yaw: 0.7854
  left_pick: { ... }
  right_pick: { ... }
```

### WBC 双臂耦合示例

抓取后切耦合，再只发左臂相对移动（右臂跟随）：

```yaml
runtime_defaults:
  max_stage_duration: 8.0
  pose_tol_pos: 0.05
  pose_tol_ori: 0.03

skill_defaults:
  dual_arm.parallel_pick:
    ee_orientation_frame: object
    object_orientation_mode: yaw
    aligned_object_yaw: auto
    left_pick: { ... }
    right_pick: { ... }
  enable_arms_coupled:
    command: ARMS_COUPLED
  coupled_left_move_relative:
    arm: left
    motion_frame_id: base_footprint
    position_delta: [0.2, 0.0, 0.3]
    orientation_delta_rpy: [0.0, -1.57, 0.0]
    arm_movel_duration: 4.0
    gripper: 0.0

task_queue:
  - skill: dual_arm.parallel_pick
  - skill: robot.send_mode_command
    id: enable_arms_coupled
  - skill: single_arm.move_relative
    id: coupled_left_move_relative
```

---

## 4) `task_queue` 常用模式

### handover（推荐）

`robot.cache_ee_pose(which=both)` -> `single_arm.pick` -> `dual_arm.handover_sync` -> `single_arm.place` -> `dual_arm.goto_cache_pose`

### 导航后回起势

`nav.navigate_backup` -> `parallel(nav.send_nav_goal + joint.movej_to_config)` -> `nav.wait_nav_arrived` -> `dual_arm.goto_cache_pose`

并行导航时如需避免 HOLD 打断 Nav2，可设：`skip_fsm_hold: true`

并行子步骤支持可选延迟（仿真时钟秒）：

```yaml
task_queue:
  - parallel:
      - skill: nav.navigate_to_object
        id: nav_to_carry
      - skill: joint.movej_to_config
        id: navigate_gesture
        start_delay_s: 2.0   # 延后 2 秒再派发该子技能
```

说明：

- `start_delay_s` 写在 **block 顶层**（与 `skill` / `id` 同级），不是 `params`。
- 该延迟使用 `sim_time.sleep(...)`，按仿真时钟推进。
- 可用于顺序块与并行子块；并行场景中通常最有用。

### 批量执行时跳过重复步骤（`skip_when_not_first_scene`）

在**同一会话**内连续跑多个 scene 或 chain segment 时，机器人状态可延续。若某些步骤只需在批量**首段**执行（例如开头 ``nav + movej``），可在块顶层设置：

```yaml
task_queue:
  - parallel:
      - skill: nav.navigate_to_object
        id: nav_to_box1
        skip_when_not_first_scene: true
      - skill: joint.movej_to_config
        id: pre_pick_movej
        skip_when_not_first_scene: true
```

- `skip_when_not_first_scene`（`bool`，默认 `false`）：写在 block 顶层，与 `skill` / `id` 同级。
- **单任务 + `__all__`**：同一会话内从第 2 个 scene preset 起跳过（如 `default` → `box2`，`box2` 会跳过带 flag 的块）。
- **多段 chain**：逐级下钻选 **叶子 task folder**（整链锁定同一叶子目录），再逐段选 task + scene；某段可选 ``__all__`` 展开该任务尚未用过的 scene（如 ``black_box_pick/__all__`` → ``default``、``box2``）。仅当本 segment 与前一段**连续且 task_key 相同**时跳过预备块；**不同 task**（搬运 → 黑盒）不跳过。同 task+scene 可重复加入（如首尾各一段搬运）。
- 单 scene、单段首次执行时照常运行；并行块中可对子步骤分别设置。

### real/object-resolution replay 中的导航跳过

real/object-resolution replay 使用交互式 task queue runner。遇到 `nav.*` 导航 skill 时，runner 会在执行前询问：

- 直接回车或输入 `r` / `run`：执行当前导航 skill。
- 输入 `s` / `skip`：跳过当前导航 skill。
- 输入 `q` / `quit`：停止当前交互队列，runner 会发送 `FSM_HOLD` 后退出。

如果导航 skill 位于 `parallel` 块中，runner 只对 `nav.*` 子 skill 逐个询问。选择跳过时只跳过该导航子 skill，parallel 块中的其它非导航子 skill 会保留，并继续按原并行逻辑执行。

已有的 `skip_when_not_first_scene: true` 优先级更高：在 `__all__` 的非首 scene 或 chain 连续同 task 场景中，如果某个 block 已经因该标记被跳过，runner 不会再弹出导航跳过询问。

### 缓存回程

- 笛卡尔：`robot.cache_ee_pose` + `single_arm.goto_cache_pose` / `dual_arm.goto_cache_pose`
- 关节：`robot.cache_joint_state` + `joint.goto_cached_joints`

---

## 5) 类型与 YAML 书写注意

- 向量请写列表（例如 `ee_base_orientation: [0.7, 0.7, 0.0, 0.0]`）
- 数字列表加载后会规范为 tuple（供 dataclass 使用）
- `null` -> Python `None`
- 布尔值使用 `true` / `false`

---

## 6) 常看文档

- 技能库与参数入口：[`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md)
- 运行时架构：包根 [`README.md`](../README.md)（**架构与设计**）
- 机器人配置约定：[`ROBOT_CONFIG.md`](ROBOT_CONFIG.md)

---

## 7) 示例路径

- `examples/IsaacSim/robots/DobotCR5/task_configs/pick_place.yaml`
- `examples/IsaacSim/robots/DobotCR5/task_configs/drawer_pick_place.yaml`
- `examples/IsaacSim/robots/Agibot_G1/task_configs/handover.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/bimanual_carry.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/navigate_and_carry.yaml`

---

## 8) 对象位姿录制与回放（统一入口）

使用 `examples/IsaacSim/motion_generation.py` 作为统一入口。仿真与真机命令形式相同，实际行为由 `robot_description` 中的 ros2_control 硬件插件自动检测决定（`sim` → live，可选录制；`real` → JSON 回放）。

```bash
python examples/IsaacSim/motion_generation.py \
  --robot FiveAges_W2 \
  --task-key poc_bimanual_2box_rev \
  --scene two_box
```

说明：

- `--object-resolution-json` 为可选覆盖路径。
- 仿真环境不传该参数时，会询问是否录制 object-resolution JSON；选择 yes 后写入 `examples/IsaacSim/robots/<robot_dir>/records/` 下的默认 pretty JSON 文件。
- 真机环境不传该参数时，会从 `examples/IsaacSim/robots/<robot_dir>/records/` 中选择匹配的 JSON；找不到时提示手动输入路径。
- 回放仅替代 Isaac object pose service 查询，不替代 `object_position_offset` 与 TF 变换。
- 真机回放按交互式习惯，每个 block 前按 Enter 继续，输入 `q` 则 `FSM_HOLD` 并退出。
- `dual_arm.parallel_pick` 分别记录 `left_pick` 与 `right_pick`。

Object-resolution replay key now includes scene:

```text
(task_key, scene, block_key, arm_side, object_role)
```

When sim runs with `scene="__all__"`, the generated filename may contain `__all__`, but every JSON record stores the concrete scene, such as `two_box1` or `two_box2`.

Real replay also accepts `scene="__all__"` for a single task. It loads one `*.object_resolution.json`, expands the task's `scene_presets` in order, and replays each concrete scene interactively. JSON records must contain `scene`; old object-resolution JSON without `scene` is not supported and must be re-recorded.
