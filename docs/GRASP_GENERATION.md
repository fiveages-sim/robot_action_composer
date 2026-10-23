# Grasp generation CLI（与 ros2-stack / motion-generation 并列）

`grasp-generation` 是可选入口：运控已由 `ros2-stack` 拉起时，用头相机生成相机系抓姿，再经 `ROS2RobotInterface` 跟腕、retarget、收拢。没装 HUG 或手型 sdk 时，`motion-generation` 的其它技能不受影响；跑到 `dex_hand.auto_grasp` 或本 CLI 时会提示缺哪一块。

**不替代** `motion-generation` 任务编排。task_queue 经 skill **`dex_hand.auto_grasp`** 调用同一抓取库。**不改** `ros2-stack`。

参考配置：`examples/IsaacSim/robots/FiveAges_W2/grasp_generation.yaml`（Wuji Hand2 + W2）。新机器人复制该文件，改 `robot_key`、相机、`hand_profile`、`grasp_objects`。

**文档分工：** 本文件讲 grasp CLI / Viser / 模块边界。从零安装见仓库根 [`docs/SETUP.md`](../../../docs/SETUP.md)。阶段笔记（腕对齐、参考手型、相机资产）在仓库根 [`docs/README.md`](../../../docs/README.md)。任务 YAML / 其它 skill 参数见 [`TASK_CONFIG_YAML.md`](TASK_CONFIG_YAML.md)、[`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md)。

| 组件 | 职责 |
|------|------|
| `ros2-stack` | 启动 OCS2 运控 / Nav2（照旧） |
| `motion-generation` | 任务队列；可用 skill **`dex_hand.auto_grasp`** 调用同一套抓取库 |
| `grasp-generation` | HUG 抓取辅助 + **Viser 调试**（采帧 → 选点 → propose → 跟腕 → close） |

## Python 拆分

| 进程 | 解释器 | 原因 |
|------|--------|------|
| `grasp-generation serve` / `follow-wrist` | 与 ROS 一致的环境（通常 3.12 + composer） | `rclpy` / `ROS2RobotInterface` |
| HUG 推理 | `submodules/hug/.venv`（3.10） | torch / manotorch；由服务 **subprocess** 调用 |

## 安装（环境要点）

父仓环境与 HUG 环境分开。步骤见仓库根 [`docs/SETUP.md`](../../../docs/SETUP.md)，坑见 [`docs/ENV_THIS_CHECKOUT.md`](../../../docs/ENV_THIS_CHECKOUT.md)。

**不要**在 `submodules/hug/.venv`（Python 3.10）里装本 CLI；也 **不要**用系统 `pip`（Ubuntu 会报 `externally-managed-environment`）。

```bash
# 仓库根目录
./init.sh all-grasp          # 编排 + viser/wuji-sdk + HUG venv
# 已有编排环境时只补抓取：
# ./init.sh install-grasp && ./init.sh hug-env
source .venv/bin/activate
which python ros2-stack grasp-generation
# python 必须落在本仓 .venv（或 .fa-env.toml 里的 conda）下，不能是 hug/.venv，也不能是 /usr
./init.sh doctor
```

每次开终端：

```bash
# 仓库根目录
source .venv/bin/activate    # 会 source .fa-env.toml 里配置的 ROS 2 工作空间
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

# 2) 运控（父仓环境）
source .venv/bin/activate
cd examples/IsaacSim
ros2-stack launch --robot fiveages_w2

# 3) 新开终端：同样 activate 后
source .venv/bin/activate
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

抓歪排查（HUG 紫点 + 绿 MANO，不启 Viser）：

```bash
# auto / dex_hand.auto_grasp 会自动写 capture_dir/hug_debug/ 与 hug_debug_overlay.png
grasp-generation dump-hug-debug --robot fiveages_w2 --workspace . \
  --from-dir submodules/hug/data/w2_wuji_head_capture
```

默认 **`position_only: true`**：只跟 MANO 腕平移，保持当前 EE 姿态，降低第一期朝向/坐标系差带来的风险。

### Phase 2A.1 — 腕目标与 EE 偏差诊断

若 Viser 手位置对、但真机/仿真 EE 差一截（臂映射，非手指），先：

```bash
# ros2-stack 已运行；建议先 Plan 一次写出 wrist_plan.json
grasp-generation diagnose-frames --robot fiveages_w2 --workspace .
```

对比 `right_tcp` / `right_eef` 与 plan `wrist_xyz`。说明见仓库 [`docs/SIM_GRASP_EE_ALIGN.md`](../../../docs/SIM_GRASP_EE_ALIGN.md)。

## 三条入口（共用库）

| 入口 | 用途 |
|------|------|
| `grasp-generation serve`（Viser） | 手动点图 / Privileged auto，调试 HUG、腕、retarget、close |
| `grasp-generation auto` | 无 UI 特权抓取（CLI） |
| motion skill `dex_hand.auto_grasp` | 任务队列内调用；**复用** `run_auto_grasp` + `ctx.interface` |

程序化稳定面：`load_grasp_config` + `run_auto_grasp`（见 `grasp_generation/config.py`、`auto_grasp.py`）。

**与运控手 FSM / GripperControlPanel：** `/fsm_command` 全局共享（RViz：HOLD / OCS2 / MOVEJ / HOME）。臂切 OCS2(3) 时手也进 MOVEJ；手非 MOVEJ 时 `target_command` / `target_joint_position` 会被丢弃。期望时序：

`仿真开 → 手 home_1 全开 → OCS2 → target_command(1) 保持开 → 跟腕（手不动）→ shape/retarget → close`

运控新增 `home_2`(半开) / `home_3`(握紧) 后，上游模板仍可能把 open 指到 home_2。**不要改 wuji_description**；用本机 `config/robot.profiles/wuji_hand2.yaml`（`control.patch`：`open_config:0` / `close_config:2`），Isaac 经 `robot.yaml` 的 `robot_profile:=` 传入。

## 模块边界与替换（MANO / Retarget / Execute）

抓取库拆成三段，**接口稳定面**如下；换模块时只改对应实现 + yaml，不要改 motion 其它 skill。

```
头相机 RGB-D
    │
    ▼
┌──────────────────┐
│ 1. MANO / Propose │  hug_worker / hug_subprocess → proposal.json
└────────┬─────────┘
         │ T_camera_wrist + landmarks + click_uv
         ▼
┌──────────────────┐
│ 2. Retarget       │  retarget/pipeline → q_retarget[n_dof]
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 3. Execute close  │  execution/runner + B0–B5 + hand profile
└──────────────────┘
```

| 段 | 代码位置 | 输入 | 输出 / 契约 |
|----|----------|------|-------------|
| **MANO 生成** | `hug_subprocess.py`、`hug_worker.py`、`dump_hug_debug.py` | 采帧目录 + `(u_norm,v_norm)` | `proposal.json`：至少含 `T_camera_wrist`（4×4）、可选 landmarks / click |
| **Retarget** | `retarget/pipeline.py`、`retarget/wuji_hand2.py`、`retarget/squeeze.py` | proposal + `hand_profile` | `list[float]` 长度 = profile `n_dof`；后缀序见 profile |
| **Execute** | `execution/runner.py`、`executors/`、`profiles/`、`contact/` | `q_retarget` + yaml close 参数 + 接触话题 | 下发 `/…/target_joint_position`；日志 `close_logs/*.jsonl` |

### 如何换 MANO 后端

1. 保持写出 **`proposal.json`** 字段约定（`T_camera_wrist` 为相机系腕目标）。  
2. 改 `hug_subprocess.run_hug_propose` 或换 worker；Viser / `auto_grasp` / skill **不必改**。  
3. 用 `grasp-generation dump-hug-debug --from-dir <capture>` 看紫点 UV + 绿 MANO 是否对齐。

### 如何换 Retarget / 新手

1. 在 `execution/profiles/` 注册 `HandExecutionProfile`（关节后缀、指分组、tip、接触 link、`topic_prefix_by_arm`）。  
2. 在 `retarget/` 实现 MANO→该手 `q`；`ensure_q_retarget` / `run_retarget_pipeline` 按 `cfg.hand_profile` 分发。  
3. yaml：`hand_profile: <name>`。今日仅 **`wuji_hand2`** 全通。

### 如何换 / 调 Execute（B0–B5）

1. 语义与参数表见 [`CLOSE_EXECUTORS.md`](CLOSE_EXECUTORS.md)（**B2 开环 squeeze ≠ B4 门控+ramp**）。  
2. 机器人侧只改 `robots/<Robot>/grasp_generation.yaml`（`close_executor`、`ramp_*`、`tip_pinch_*`、`soft_stop_force_n`…）。  
3. 接触：Isaac 开 `Sensor=contact_force`；`ignore_contact_partners` 勿含 `default_environment`。

### 其它模块

| 模块 | 作用 | 替换时保持 |
|------|------|------------|
| `config.py` / `robots/*/grasp_generation.yaml` | 机型解耦：相机、手、MANO→tip、profile、executor、`grasp_objects` | yaml 字段名 |
| `frames` / `se3` / `bridge` | 相机/腕 TF → `send_target_stamped` | plan 字段 |
| `objects` / `privileged_pose` / `project_uv` | 特权选物 UV（**不改场景摆放**） | `pose_prim` / `click_prim` |
| `auto_grasp` | capture → UV → HUG → wrist → close | `AutoGraspResult` |
| `service` / `cli/grasp_main.py` | Viser / CLI | — |

**新机器人：** 复制 `grasp_generation.yaml`；改 `robot_key`、相机 topic/prim、`mano_to_tip_rpy_deg`、`hand_profile`、`grasp_objects`。新手须 **同时** 加 retarget 后端与 `execution/profiles/`，并只选用这只手支持的 `close_executor`。接触力是手资产上的变体，B3–B5 才需要；新场景只要机器人（含手）和物体 prim 就能进编排。见仓库根 [`docs/ENV_AND_HAND_CONTRACT.md`](../../../docs/ENV_AND_HAND_CONTRACT.md)。

```bash
grasp-generation close-hand --robot fiveages_w2 --workspace . --mode B4 --trial-id t01
```

### 仿真测试环境（参考：FiveAges W2 + Wuji Hand2）

| 项 | 要求 |
|----|------|
| USD | FaSim `…/W2/env/wuji_hand2_new.usda`（东村桌子 + 苹果等；spawn `x=-2`） |
| 手 | 右手 `Sensor=contact_force`；话题 `/wuji_rh/*` |
| 运控 | `ros2-stack launch --robot fiveages_w2`；手 open→`home_1` 用本机 `robot.profiles/wuji_hand2.yaml` |
| 导航冒烟 | `--group dex_hand` + `task_configs/dex_hand/navigate_and_dex_grasp.yaml` |
| 配置 | `examples/IsaacSim/robots/FiveAges_W2/grasp_generation.yaml` |

冒烟顺序：Isaac Play → stack →（可选）`grasp-generation serve` 或 `motion-generation … navigate_and_dex_grasp`。

## Privileged auto（仿真特权选物）

不替代右侧点图选 UV。物体列表写在 `robots/FiveAges_W2/grasp_generation.yaml` 的 `grasp_objects`（`wuji_hand2_new` 桌面）。**不改场景摆放**（无 exile / 清场）；桌面物品保持 env 作者态。

Viser 文件夹 **Privileged auto**：

1. 下拉选物体（如 `apple`）
2. **dry_run_auto** / **auto_execute** / **Go home（最高优先）**
3. Auto：capture → 特权 UV → HUG →（可选）腕 + close

**流程：** Play → Go home → 选物 Auto。

**Go home（抓取 home）**：`HOLD → HOME → settle → HOLD → OCS2`，再双臂 **+Z** `grasp_home_lift_m`。

位姿：优先 `/get_entity_state`（重试）；失败回退 prim-attr 链；两者漂移 >5cm 打 WARN。

**HUG 紫点 — 推荐 `click_prim`（Stage 手动选点）：**

1. Stage 里选中物体根（`placement_prim`，如 `.../sm_food_beverage_beerBottle_a01_01`）。
2. 右键 → **Create → Xform**，命名 `Grasp_Point_1_1`（可多个：`_1_2`…）。
3. 选中该 Xform，用 Move 工具拖到抓取面最佳点（瓶身中段/把手中心等）。Orient 可保持单位四元数。
4. 复制完整 prim 路径到 yaml：
   ```yaml
   bottle:
     click_prim: /World/default_environment/EastRural_Table/sm_food_beverage_beerBottle_a01_01/Grasp_Point_1_1
     click_offset: [0.0, 0.0, 0.0]
   ```
5. **Save** 环境 USD（或保证每次进场景后 Xform 仍在），否则会话里建的点下次会丢。
6. 勾选 `dry_run_auto` → Auto → 看紫点是否落在 Xform；不对就挪 Stage 里的点，不必改 `click_offset`。

**备选 `click_offset`：** 无 `click_prim` 时用 `pose_prim` 中心 + `[x,y,z]` 米（局部系）。多数资产原点在底部，先抬 Z。调参同上 dry_run。

新物体：复制一段 `grasp_objects` → 填 `pose_prim` / `placement_prim` → 按上面建 `Grasp_Point` → 写 `click_prim`。

```bash
# 验 UV（不动臂）
grasp-generation auto --robot fiveages_w2 --workspace . --object apple --dry-run

# 真执行
grasp-generation auto --robot fiveages_w2 --workspace . --object apple --execute
```

### motion-generation 任务队列

导航靠近后再跑同一套 `run_auto_grasp`（skill 参数见 [`SKILLS_REFERENCE.md`](SKILLS_REFERENCE.md) §3b）。

W2 仿真导航走 **`navigation_isaac_gt`**：Isaac GT `/odom` + 静态地图（`map`→`world` 恒等），**不是** SLAM。`task_configs/dex_hand/.meta`：`map_only` + **EastRural 全空闲 map**（勿用仓库图，否则会绕「假障碍」跑远）+ Nav2 RViz。`approach_offset_*` 是**世界系 XY**（桌面高度只影响 HUG，不进底盘目标）。

```bash
# Isaac: wuji_hand2_new Play；机器人放到桌子西侧数米外
cd examples/IsaacSim
ros2-stack launch --robot fiveages_w2 --group dex_hand   # 改 map 后需重启 stack
motion-generation --robot fiveages_w2 --task-key navigate_and_dex_grasp --scene apple
```

## 与既有 smoke 脚本关系

`submodules/hug/scripts/smoke_w2_head_hug.sh` 仍可用于「只验证 HUG」。  
跟腕与运控对接请走 **`grasp-generation`**。
