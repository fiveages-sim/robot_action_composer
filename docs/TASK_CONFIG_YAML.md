# 任务配置：YAML 说明

本文件描述 **`robot_action_composer`** 如何加载任务 YAML，以及字段约定。  
**机器人目录与 `robot_config.py`** 见同目录下的 **[ROBOT_CONFIG.md](ROBOT_CONFIG.md)**。

在 monorepo 中，典型资产根目录为 **`examples/IsaacSim`**，任务文件位于 `examples/IsaacSim/robots/<Robot>/task_configs/`；`motion_main` / `record_main` / `registry_loader` 会把该路径传给 `discover_task_configs`。

---

## 格式

任务发现由 **`robot_action_composer.task_config_io.discover_task_configs`** 完成，**仅**加载给定目录下的 **`*.yaml` / `*.yml`**。可选嵌套 `pick` / `place` 以及 `carry` / `drawer` 等由 **`flatten_queue_task_overrides`** 摊平；**禁止**在 `base_task_overrides` 或场景根下再写嵌套 **`handover:`**（已移除）。**双臂同步交接段**几何在 **`skill_defaults.dual_arm.handover`**（`HandoverSyncConfig`：交接点位置与双臂姿态）；**接收臂放置**用 **`skill_defaults.single_arm.place`**（`place_position` / `place_orientation` 等）。场景覆盖仍用 **`skill_params`** 下对应键。`motion_generation`、`record_datasets`、`inference.py` 在扁平化后按 **pick → handover → carry → drawer** 顺序叠 `skill_defaults`。

每个**基名**（如 `pick_place`）对应**一个** YAML 文件（`pick_place.yaml` 或 `pick_place.yml`）。**同一基名不能同时存在 `.yaml` 与 `.yml`**，否则会 `ValueError`。

**不再支持** Python 任务模块（`TASK_CONFIG` / `FLOW_CONFIG` 的 `.py` 文件）。

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
- `examples/IsaacSim/robots/DobotCR5/task_configs/drawer_pick_place.yaml`
- `examples/IsaacSim/robots/Agibot_G1/task_configs/pick_place.yaml`、`handover.yaml`
- `examples/IsaacSim/robots/Realman_RM75/task_configs/pick_place.yaml`
- `examples/IsaacSim/robots/Marvin_M6CCS/task_configs/handover.yaml`
- `examples/IsaacSim/robots/FiveAges_W2/task_configs/bimanual_carry.yaml`
