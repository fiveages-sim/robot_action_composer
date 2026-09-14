# Grasp generation CLI（与 ros2-stack / motion-generation 并列）

`grasp-generation` 是 **新功能入口**：在运控已由 `ros2-stack` 拉起的前提下，用头相机 + HUG 生成相机系抓姿，再经 `ROS2RobotInterface` 把 **腕** 送到对应位姿。

**不替代** `motion-generation` 任务编排，也 **不修改** 现有 `ros2-stack` / task_queue 逻辑。

## 角色分工

| 组件 | 职责 |
|------|------|
| `ros2-stack` | 启动 OCS2 运控 / Nav2（照旧） |
| `motion-generation` | 既有 skill 任务队列（照旧） |
| `grasp-generation` | HUG 抓取辅助：采帧 → 选点 → propose → 跟腕 |

## Python 拆分

| 进程 | 解释器 | 原因 |
|------|--------|------|
| `grasp-generation serve` / `follow-wrist` | 与 ROS 一致的环境（通常 3.12 + composer） | `rclpy` / `ROS2RobotInterface` |
| HUG 推理 | `submodules/hug/.venv`（3.10） | torch / manotorch；由服务 **subprocess** 调用 |

## 安装（环境要点）

**开发与环境只使用本仓库** `/home/fiveages/lerobot_ros2`（见仓库根目录 [`docs/ENV_THIS_CHECKOUT.md`](../../../docs/ENV_THIS_CHECKOUT.md)）。

**不要**在 `submodules/hug/.venv`（Python 3.10）里装本 CLI；也 **不要**用系统 `pip`（Ubuntu 会报 `externally-managed-environment`）。

```bash
cd /home/fiveages/lerobot_ros2
bash scripts/setup_grasp_dev_env.sh
source .venv/bin/activate
which python ros2-stack grasp-generation
# python 必须落在本仓 .venv 下
```

每次开终端：

```bash
cd /home/fiveages/lerobot_ros2
source .venv/bin/activate   # 自动 source jazzy + ros2_ws + fa_w2_ws
```

配置：

- 包默认：`robot_action_composer/grasp_generation/default_config.yaml`
- 机器人覆盖：`examples/IsaacSim/robots/FiveAges_W2/grasp_generation.yaml`

## Viser 使用教程（对齐 HUG 官方）

布局与官方类似：**左侧**可拖动的 RGB-D 点云 + 绿色 MANO 骨架；**右侧**点图选物。

1. 点 **`1) Capture head RGB-D`** → 左侧出现彩色深度点云（鼠标拖转、滚轮缩放）；右下角出现头相机照片。
2. **在右下角照片上点击物体** → 自动 HUG propose → 左侧出现绿色虚拟手 + 腕坐标系（相机系 dry 位姿）。
3. 保持 **`execute_move` 不勾**，点 **`3) Plan / move wrist`** → DRY-RUN，刷新左侧手并打印 `base_link` 的 `xyz=...`。
4. 确认合理后勾选 **`execute_move`**，再点一次 **3)** 才真动右臂。

备用：滑条 `u_norm` / `v_norm` + **`2) HUG propose @ UV (manual)`**。  
`position_only` 建议保持勾选。`max_depth_m` / `pc_point_size` 可调点云显示。

```bash
# Ctrl+C 旧进程后重启以加载新 UI
grasp-generation serve --robot fiveages_w2 --workspace .
```

```bash
# 1) 仿真 Play + 头相机话题正常

# 2) 运控（本仓 .venv）
cd /home/fiveages/lerobot_ros2
source .venv/bin/activate
cd examples/IsaacSim
ros2-stack launch --robot fiveages_w2

# 3) 新开终端：同样 activate 后
cd /home/fiveages/lerobot_ros2 && source .venv/bin/activate
cd examples/IsaacSim
grasp-generation serve --robot fiveages_w2 --workspace .
```

一键看配置：

```bash
grasp-generation show-config --robot fiveages_w2 --workspace examples/IsaacSim
```

已有 `proposal.json` 时只做坐标系 + 跟腕：

```bash
grasp-generation follow-wrist --robot fiveages_w2 --workspace examples/IsaacSim \
  --proposal submodules/hug/data/w2_head_capture/proposal.json
# 真动臂：
grasp-generation follow-wrist ... --proposal ... --execute
```

默认 **`position_only: true`**：只跟 MANO 腕平移，保持当前 EE 姿态，降低第一期朝向/坐标系差带来的风险。

### Phase 2A.1 — 腕目标与 EE 偏差诊断

若 Viser 手位置对、但真机/仿真 EE 差一截（臂映射，非手指），先：

```bash
# ros2-stack 已运行；建议先 Plan 一次写出 wrist_plan.json
grasp-generation diagnose-frames --robot fiveages_w2 --workspace .
```

对比 `right_tcp` / `right_eef` 与 plan `wrist_xyz`。说明见仓库 [`docs/SIM_GRASP_EE_ALIGN.md`](../../../docs/SIM_GRASP_EE_ALIGN.md)。

## 模块

| 模块 | 作用 |
|------|------|
| `config.py` / yaml | 话题、TF 候选、prim、端口、HUG python 路径 |
| `hug_adapter` / `hug_worker` | HUG propose（3.10） |
| `hug_subprocess` | serve 侧调 HUG |
| `frames` / `se3` / `bridge` | `T_base_cam`、`T_base_wrist`、`send_target_stamped` |
| `service` | Viser UI |
| `cli/grasp_main.py` | `grasp-generation` 入口 |

## 与既有 smoke 脚本关系

`submodules/hug/scripts/smoke_w2_head_hug.sh` 仍可用于「只验证 HUG」。  
跟腕与运控对接请走 **`grasp-generation`**。
