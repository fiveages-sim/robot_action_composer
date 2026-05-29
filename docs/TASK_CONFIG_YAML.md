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
  - `task_groups`：按一级子目录分组

### 唯一性约束

- 同目录下，同基名不能同时有 `.yaml` 与 `.yml`
- 不同子目录可有同名文件
- 同机器人范围内，`task_key` 必须全局唯一

### 分组交互

- 根目录文件属于 `task_groups[""]`（CLI 显示 Top level）
- `task_configs/<folder>/...` 属于 `<folder>` 分组（更深层仍归该一级分组）
- 有多个分组时，CLI 先选 folder 再选 task
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

### `runtime_defaults` 允许键（严格）

`runtime_defaults`（以及 `scene_presets.<scene>` 根层）仅允许以下 4 个键：

- `base_link_entity_path`
- `max_stage_duration`
- `pose_tol_pos`
- `pose_tol_ori`

其他字段（抓取/放置几何、双臂参数、drawer/handover 参数等）必须写在：

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
- `pose_tol_pos` / `pose_tol_ori` / `max_stage_duration` 可在 `runtime_defaults` 或场景根配置，作用于流程级判定。

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
- **多段 chain**：仅当本 segment 与前一段**连续且 scene preset 名相同**时跳过；中间夹了不同 scene 再回来则不跳过。
- 单 scene、单段首次执行时照常运行；并行块中可对子步骤分别设置。

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
