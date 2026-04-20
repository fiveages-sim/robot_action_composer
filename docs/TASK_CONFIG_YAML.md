# 任务配置 YAML 指南（精简版）

本文聚焦三件事：

1. 一个任务 YAML 应该怎么组织
2. 加载与合并规则是什么
3. 写 `task_queue` 时最常用的模式有哪些

技能参数明细请看：[`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md)
架构与代码路径请看：[`ARCHITECTURE.md`](ARCHITECTURE.md)

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

base_task_overrides:
  pose_tol_pos: 0.025
  pose_tol_ori: 0.08

skill_defaults:
  single_arm.pick:
    arm: right
    object_prim_path: /World/object
  single_arm.place:
    run_place_before_return: true

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

- `task_key` / `label` / `robot_id` / `default_scene`
- `use_stamped`
- `base_task_overrides`
- `skill_defaults`
- `task_queue`
- `scene_presets`

### 合并优先级（低 -> 高）

1. `base_task_overrides`
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
- `base_link_entity_path` 可在 `base_task_overrides` 或场景根覆盖机器人默认 base prim。
- `pose_tol_pos` / `pose_tol_ori` 可在 `base_task_overrides` 或场景根配置，作用于笛卡尔到达判定。

---

## 4) `task_queue` 常用模式

### handover（推荐）

`robot.cache_ee_pose(which=both)` -> `single_arm.pick` -> `dual_arm.handover_sync` -> `single_arm.place` -> `dual_arm.goto_cache_pose`

### 导航后回起势

`nav.navigate_backup` -> `parallel(nav.send_nav_goal + joint.movej_to_config)` -> `nav.wait_nav_arrived` -> `dual_arm.goto_cache_pose`

并行导航时如需避免 HOLD 打断 Nav2，可设：`skip_fsm_hold: true`

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
- 运行时架构：[`ARCHITECTURE.md`](ARCHITECTURE.md)
- 机器人配置约定：[`ROBOT_CONFIG.md`](ROBOT_CONFIG.md)

---

## 7) 示例路径

- `examples/IsaacSim/robots/DobotCR5/task_configs/pick_place.yaml`
- `examples/IsaacSim/robots/DobotCR5/task_configs/drawer_pick_place.yaml`
- `examples/IsaacSim/robots/Agibot_G1/task_configs/handover.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/bimanual_carry.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/navigate_and_carry.yaml`
