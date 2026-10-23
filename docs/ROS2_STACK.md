# 场景级 ROS2 运控 / 导航启动（ros2_stack）

任务编排 YAML（`task_key` / `task_queue`）**不负责**启动 OCS2 / Nav2。场景默认启动配置写在：

1. [`robot.yaml`](ROBOT_CONFIG.md) 的 `ros2_stack:` 段（机器人通用）
2. 叶子任务目录下的 **`.meta/ros2_stack.yaml`**（场景覆盖）

`.meta/` 以 `.` 开头，任务发现会跳过，**不会**与「叶子目录不得有任务子 YAML」规则冲突。

## 合并顺序

```
robot.yaml → ros2_stack
    ↓ 深合并（args 按键合并）
<leaf>/.meta/ros2_stack.yaml
```

不要在任务编排 YAML 里写 `ros2_stack`。

## 运控 preset

| preset | launch | 默认 ready 子串 |
|--------|--------|-----------------|
| `ocs2-fullbody` | `ocs2_arm_controller` / `full_body.launch.py` | `ocs2_wbc_controller` |
| `ocs2-split-body` | `ocs2_arm_controller` / `split_body.launch.py` | `ocs2_arm_controller` |
| `ocs2-demo` | `ocs2_arm_controller` / `demo.launch.py` | `ocs2_arm_controller` |

## 导航 profile

| `navigation.profile` | 展开 |
|----------------------|------|
| `default` | `nav2_profile:=default` |
| `map_only` | `nav2_profile:=map_only` |

不要写 `args.nav2_profile`；用一等字段 `profile`。默认 launch：`robot_common_launch` / `navigation_isaac_gt.launch.py`。

`navigation.required: auto` 时：仅当任务 `task_queue` 含 `nav.*` skill 才启动导航。

## 可视化 / headless

| 配置字段 | 展开为 launch 参数 | 默认（未写时） |
|----------|-------------------|----------------|
| `motion.headless: true` | `launch_mode:=control_only`（OCS2 无 RViz） | `true` |
| `motion.headless: false` | `launch_mode:=full` | — |
| `navigation.headless: true` | `use_rviz:=false`（`navigation_isaac_gt`） | `true` |
| `navigation.headless: false` | `use_rviz:=true` | — |

也可直接写 `motion.args.launch_mode` / `navigation.args.use_rviz`；一等字段优先。CLI：`--set motion.headless=false`、`--set navigation.headless=false`。

参考 launch：`robot_common_launch` 的 `launch_mode`（`full` / `control_only` / `rviz_only`）与 `navigation_isaac_gt.launch.py` 的 `use_rviz`。

## CLI

界面语言与 `motion-generation` 共用（`--lang zh|en`，或 `MOTION_GENERATION_LANG` / `.motion_last.json` 中的 `lang` / 系统 `LANG`）。交互菜单（选机器人、任务文件夹、运控 preset、导航 profile、是/否）复用同一套 `cli/interactive.py` 与 i18n。

`ros2-stack launch` 会把上次成功启动写入工作空间 `.ros2_stack_last.json`（已 gitignore）。再次交互启动时默认可选「上次选择」一键复用；重新选择时也会以该记录为默认值。合并配置里的 preset / profile / map 会直接采用，默认不再逐项重选（仅询问是否修改选项，回车=否）。

### Tab 补全（bash）

**推荐：跟 conda / uv 的 ROS2 activate 钩子一起启用**（`./init.sh ros2-workspace` 会写入）。之后每次：

```bash
source .venv/bin/activate   # 或 conda activate <env>
# 会自动 source ROS2，并注册 ros2-stack 补全
```

无需再手动 `eval`。重装钩子：

```bash
./init.sh ros2-workspace          # 按当前 backend
./init.sh ros2-workspace --all    # conda + uv 都写
```

其它方式（任选）：

```bash
# 当前 shell 临时启用
eval "$(ros2-stack completion bash)"

# 用户级持久化（不依赖 activate）
mkdir -p ~/.local/share/bash-completion/completions
ros2-stack completion bash > ~/.local/share/bash-completion/completions/ros2-stack
```

可补全：子命令（`launch`/`status`/`stop`/`logs`）、`--robot`、`--group`、`--motion-preset`、`--nav-profile`、`--component`、`--lang` 等。

```bash
cd examples/IsaacSim
ros2-stack --lang zh launch   # 交互：上次选择 或 重新选机器人/任务组（配置默认直接用）
ros2-stack status
ros2-stack logs -f                        # 默认当前机器人（运行中 / 上次启动）
ros2-stack logs --robot fiveages_w2 -f    # 指定机器人跟踪日志
ros2-stack stop                           # 默认停当前托管中的栈（有 PID / 上次启动）
ros2-stack stop --robot fiveages_w2       # 指定机器人

ros2-stack launch --robot fiveages_w2 \
  --group "projets/siemens/Wind turbo blade" \
  --set navigation.map=wind_turbo_task1 \
  --motion-preset ocs2-fullbody \
  --nav-profile map_only \
  --force-nav
```

默认在**后台**拉起（stdout/stderr 写入日志文件），当前终端不会刷屏。日志与 PID 在工作空间：

```text
.ros2_stack/<robot>/motion.log
.ros2_stack/<robot>/navigation.log
```

也可用 `tail -f .ros2_stack/fiveages_w2/motion.log`。若希望像手动开终端那样看日志，加 `--new-terminal`（有 GUI 终端时）。

`stop` 只杀本工具登记过的 PID；手动 `ros2 launch` 起的进程不会被停掉。

## motion-generation

选好任务后、`connect` 之前可 ensure：

```bash
motion-generation --robot fiveages_w2 --task-key transfer_blade --ensure-ros2-stack
motion-generation --no-ensure-ros2-stack
motion-generation --motion-preset ocs2-split-body --nav-profile map_only
```

交互偏好「运行前检查 ROS2 栈」打开后（或 CLI `--ensure-ros2-stack`）会 ensure；默认可改选 preset / profile（回车保持配置）。`--no-ensure-ros2-stack` 覆盖偏好为关闭。

## 示例

`robot.yaml`：

```yaml
ros2_stack:
  defaults:
    args:
      robot: fiveages_w2
      hardware: isaac
  motion:
    required: true
    preset: ocs2-fullbody
    headless: true
    args:
      type: rg75
  navigation:
    required: auto
    profile: default
    headless: true
```
`Wind turbo blade/.meta/ros2_stack.yaml`：

```yaml
navigation:
  profile: map_only
  args:
    map: wind_turbo_task1
```
