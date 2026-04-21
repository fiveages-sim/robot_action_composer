# 技能库与配置项说明（按技能）

本文按技能名逐条说明：

- 技能效果（做什么）
- 关键参数（有哪些、含义是什么）
- 使用备注（常见搭配、前置条件）

> 约定：除特别说明外，参数均写在 `task_queue` 当前块的 `params` 中；
> 默认值可放 `skill_defaults`，场景差异放 `scene_presets.<scene>.skill_params`。
>
> 参数写法约定：
> - `字段名`（类型，默认值，必填/可选）：含义
> - 未标注“必填”时默认可选

---

## 1. 单臂技能（`single_arm.*`）

### 1.1 抓取（`single_arm.pick`）

- **效果**：执行单臂抓取序列（approach / close-in / grasp / retreat）。
- **参数**（来自 `single_arm.pick` 切片）：
  - **目标物体配置**
    - `object_prim_path`：抓取对象 Prim 路径。
    - `object_position_offset`：相对物体中心的抓取偏移（物体局部坐标系；会按物体当前姿态旋转后叠加到抓取目标）。
  - **运动系配置**
    - `ee_base_orientation`：抓取姿态四元数 `xyzw`。
    - `motion_frame_id`：抓取目标输出坐标系（可选；不写则沿用任务 `frame_id`）。
    - `tf_lookup_timeout`：当 `motion_frame_id` 与任务坐标系不一致时的 TF 查询超时（秒）。
  - **末端系配置**
    - `pick_clearance`：闭合抓取阶段偏移（工具系位移，沿末端局部轴向解释）。
    - `prepare_offset`：预接近偏移（工具系向量，语义与 `dual_arm.carry.carry_prepare_offset` 一致）。
    - `ee_lift_offset`：抓取后抬升位移（工具系向量；会按 `ee_base_orientation` 旋转后用于抬升段）。
    - `ee_retreat_offset`：抬升后后撤位移（工具系向量；会按 `ee_base_orientation` 旋转后用于后撤段）。
    - `ee_pick_axis`：末端抓取轴向字符串（与 `ee_place_axis` 对称）；`+x/-x/+y/-y/+z/-z`，默认 `+z`。
    - `ee_pick_direction_vector`：抓取后撤等所用的方向单位向量 `[x,y,z]`（可选；非空时覆盖 `ee_pick_axis`）。
  - **执行控制**
    - `arm`：`left|right`。
    - `arm_movel_duration`：可选；执行前写入 `arm_controller.movel_duration`。
- **备注**：`object_prim_path` 必填。

### 1.2 放置（`single_arm.place`）

- **效果**：执行单臂放置序列（在 `task_queue` 中出现即执行）；既支持放置到固定坐标（`place_position`），也支持基于参考物体坐标动态计算放置目标（`object_prim_path` + `object_position_offset`）。
- **参数**（来自 `single_arm.place` 切片）：
  - **目标物体配置**
    - `object_prim_path`：放置参考物体 Prim 路径（可解析放置位）。
    - `object_position_offset`：相对参考物体中心的放置偏移（物体局部坐标系；会按物体当前姿态旋转后叠加到放置目标）。
  - **运动系配置**
    - `place_position`：显式放置位置（运动系坐标；当未设置 `object_prim_path` 时，直接按该坐标放置）。
    - `ee_base_orientation`：放置姿态四元数 `xyzw`（未设置时默认继承 `single_arm.pick.ee_base_orientation`）。
    - `motion_frame_id`：放置目标输出坐标系（可选；不写则沿用任务 `frame_id`）。
    - `tf_lookup_timeout`：当 `motion_frame_id` 与任务坐标系不一致时的 TF 查询超时（秒）。
  - **末端系配置**
    - `ee_place_axis`：放置轴向（末端坐标系；使用 `+x/-x/+y/-y/+z/-z`，不设置时继承 `single_arm.pick.ee_pick_axis`）。
    - `place_insert_clearance`：放置闭合段偏移（工具系标量，沿 `ee_place_axis`）。
    - `prepare_offset`：放置预接近偏移（工具系三维向量 `[x,y,z]`；按 `ee_base_orientation` 旋转后叠加到放置目标）。
    - `ee_retreat_offset`：松爪后回撤偏移（工具系向量；按 `ee_base_orientation` 旋转后执行）。
  - **执行控制**
    - `arm`：`left|right`（不配置时默认继承 `single_arm.pick.arm`）。
    - `arm_movel_duration`：可选；执行前写入 `arm_controller.movel_duration`。
- **备注**：
  - 放置位置必须可解析；
  - 未设置 `ee_base_orientation` 时，默认继承 `single_arm.pick.ee_base_orientation`；
  - 未设置 `ee_place_axis` 时，默认继承 `single_arm.pick.ee_pick_axis`（默认 `+z`）。

### 1.3 回程缓存末端（`single_arm.goto_cache_pose`）

- **效果**：回到 `robot.cache_ee_pose` 记录的缓存末端位姿。
- **参数**：
  - **缓存与选择**
    - `key`：缓存键，默认 `saved_ee_pose`。
    - `side`：当缓存为 `which: both` 时选 `left|right`。
  - **末端系配置**
    - `gripper`：覆盖缓存中的夹爪目标。
    - `stage_name`：阶段名覆盖。
- **备注**：
  - 单臂缓存可直接用；
  - 双臂缓存必须指定 `side` 或依赖 `common.arm` 推断。

### 1.4 发送笛卡尔目标（`single_arm.send_cartesian_goal`）

- **效果**：将末端运动到指定笛卡尔位姿（单段 MoveL），可指定参考帧与工作臂。适合需要手动给定绝对坐标的场景，无 pick/place 的抓握与放置逻辑。
- **参数**：
  - **必填**
    - `arm`：工作臂，`left` 或 `right`。
    - `position`：目标位置，`[x, y, z]`（米）。
    - `orientation`：目标姿态四元数，`[x, y, z, w]`。
  - **可选**
    - `frame_id`：位姿所在参考帧，默认 `ctx.frame_id`（通常为 `arm_base`）。
    - `gripper`：目标夹爪值；**默认保持上一步结束时的夹爪状态**（`ctx.gripper_for_return_home`）。
    - `stage_name`：阶段名，默认 `TaskQ-SendCartesianGoal`。
- **YAML 示例**：
  ```yaml
  - skill: single_arm.send_cartesian_goal
    params:
      arm: right
      frame_id: arm_base
      position: [0.45, -0.15, 0.50]
      orientation: [-0.7, 0.7, 0.0, 0.0]
  ```
- **备注**：
  - 不依赖 Isaac Sim 物体 Prim，纯坐标驱动，适合固定目标点的运动。
  - 若机器人基座会移动（导航场景），`frame_id` 建议用随基座变换的帧（如 `arm_base`）而非世界系 `map`。

### 1.5 抽屉（`single_arm.drawer.*`）

- **配置**：抽屉几何字段写在 **`single_arm.drawer`**，与 **`pull_open`** / **`close_push`** 共用。
- **队列**：开关抽屉在 **`task_queue` 里是两条技能**（`single_arm.drawer.pull_open` 与 `single_arm.drawer.close_push`），中间通常插入抓取、放置等其他块。
- **参数**：
  - **Prim / 拉手**
    - `object_prim_path`：抽屉分层 Prim（必填）。
    - `object_position_offset`：拉手参考点在抽屉 Prim 局部系下的 `[x,y,z]`（米）。
  - **几何**
    - `drawer_clearance`：沿末端 **+Z** 的闭合段偏移（米）。
    - `prepare_offset`：预接近（工具系）；全 0 / 省略则无 Approach 段。
    - `pull_distance`：拉开末段沿拉手拉出方向的行程（米）。
    - `ee_retreat_offset`：松爪后工具系平移；非零时：`pull_open` 在拉开 Cartesian 末尾追加松爪与撤出段，`close_push` 在到位 **`place_pose_ref`**（夹爪仍闭合）后再松爪并撤出。省略或全 0 则无上述撤出段。

#### 1.4.1 拉开抽屉（`single_arm.drawer.pull_open`）

- **效果**：按几何生成拉开序列；写入 **`ctx.drawer`**，更新 **`ctx.task_cfg.place`** 放置提示；不改 **`ctx.task_cfg.pick`**。
- **备注**：需有效 `object_prim_path` 与 `object_position_offset`；臂别由 **`common.arm`** 决定。

#### 1.4.2 关上抽屉（`single_arm.drawer.close_push`）

- **效果**：须队列中已有 **`pull_open`**。粗定位 → 关抽屉 Cartesian → 闭合到位 **`place_pose_ref`** → 按 **`ee_retreat_offset`** 配置松爪并撤出。
- **备注**：必须排在 **`pull_open`** 之后。



https://github.com/user-attachments/assets/3ae2fb74-8933-49fc-972e-f9ab8237a53b



---

## 2. 双臂技能（`dual_arm.*`）

### 2.1 搬运（carry）

> 搬运统一使用单个 `dual_arm.carry`。  
> 配置主要来自 `dual_arm.carry`（即 `skill_defaults.dual_arm.carry` / `scene_presets.*.skill_params.dual_arm.carry`）。
> PoC 项目里你看到的大部分字段（如 `orientation_delta_rpy`、`ee_lift_offset`）都属于这层配置，而不是技能块 `params`。

`dual_arm.carry` 主要字段（与 `BimanualCarryTaskConfig` 对齐）：

- **目标物体配置**
  - `object_prim_path`：被搬运物体的 Prim 路径。
  - `object_dual_arm_half_span_y`：双臂抓取的“半宽”基准（沿 motion 输出系 Y 轴）。
  - `object_position_offset`：相对物体中心的抓取偏移（物体系坐标）。

- **运动系配置**
  - `left_base_orientation` / `right_base_orientation`：左右臂基础抓取姿态（四元数 `xyzw`）。
  - `orientation_delta_rpy`：对左右臂基础姿态同时施加的 RPY 增量（弧度）。
  - `arm_merge_distance_y`：接近阶段沿 Y 的额外张开余量。
  - `motion_frame_id`：carry 目标输出坐标系（例如 `arm_base`）。
  - `tf_lookup_timeout`：当 `motion_frame_id` 需要 TF 变换时的查询超时（秒）。

- **末端系配置**
  - `carry_prepare_offset`：预闭合前的工具系偏移；为空或全 0 时可省略预接近段。
  - `ee_lift_offset`：抓取后抬升位移（是否生成抬升段取决于该值是否为空）。
  - `ee_retreat_offset`：抬升后后撤位移（是否生成后撤段取决于该值是否为空）。
  - `arm_movel_duration`：执行前写入臂控制器的 `movel_duration`（调节笛卡尔段时长）。

#### `dual_arm.carry`

- **效果**：一次执行完整搬运序列（接近/闭合/抬升/后撤）。
- **参数**：无块级专用参数（读取 `dual_arm.carry` 配置）。

https://github.com/user-attachments/assets/1e4e9d34-e5ba-4aa7-8e70-6c5a9de7631f



### 2.2 双臂同时抓取（parallel_pick）

> 该技能从当前块 `params` 解析 `BimanualParallelPickTaskConfig`。  
> 必须提供 `left_pick` 与 `right_pick` 两个映射。

`left_pick` / `right_pick` 字段与 `single_arm.pick` 切片语义一致（每侧一套）：

- **目标物体配置**
  - `object_prim_path`：该侧抓取目标物体 Prim 路径（必填）。
  - `object_position_offset`：相对物体中心的抓取偏移（物体局部坐标系；与 `single_arm.pick` 相同）。

- **运动系配置**
  - `motion_frame_id`：该侧物体位姿解析后的输出坐标系（可选；不写则沿用任务 `frame_id`）。**左右两侧解析后必须相同**，否则双臂同步下发无法共用一个坐标系。
  - `tf_lookup_timeout`：TF 查询超时（秒）；本块取左右两侧的较大值用于两侧物体位姿变换。

- **末端系配置**
  - `ee_base_orientation`：抓取姿态四元数 `xyzw`。
  - `pick_clearance`：闭合抓取阶段偏移（工具系，沿末端局部 +Z 解释，与 `build_single_arm_pick_sequence` 一致）。
  - `prepare_offset`：预接近偏移（工具系三维向量；全 0 或省略时可省略预接近段）。
  - `ee_lift_offset` / `ee_retreat_offset`：抬升与后撤（工具系向量；会分别按该侧 `ee_base_orientation` 旋转到世界系后写入序列，与 `single_arm.pick` 一致）。
  - `ee_pick_axis` / `ee_pick_direction_vector`：与单臂 `QueueSlicePick` 相同（轴向或显式单位向量）。

- **执行控制**
  - `arm_movel_duration`：可选；优先取 `left_pick`，否则取 `right_pick`，写入 `arm_controller.movel_duration`。

#### `dual_arm.parallel_pick`

- **效果**：一次执行完整双臂同时抓取序列（两侧各走一条 `build_single_arm_pick_sequence`，再按阶段同步合成）。
- **参数**：读取 `left_pick` / `right_pick`（见上方字段列表）。
- **备注**：执行完成后会将 `gripper_for_return_home` 置为闭合值（与单臂 pick 行为一致）。



https://github.com/user-attachments/assets/3464ad42-e5b1-4035-9aa3-f78b69cb3029



### 2.3 放置（place）

#### `dual_arm.place`

- **效果**：执行双臂相对放置序列（同向平移 -> 松爪 -> 外张 -> 后撤）。默认按当前末端做相对位移；若提供参考物体配置（`reference_object_prim_path` + `object_position_offset`），则会先计算目标放置点，再自动换算本次平移量。
- **默认值复用**：当未显式配置放置参数时，会复用 `dual_arm.carry` 的部分几何/节奏配置作为默认值（因此 `place` 常可只写少量参数）。
- **参数**：
  - **目标物体配置**
    - `reference_object_prim_path`（`str`）：参考物体 Prim 路径。
    - `object_position_offset`（`[float,float,float]`）：参考物体坐标系下偏移。

  - **运动系配置**
    - `translation_xyz`（`[float,float,float]`，默认 `[0,0,0]`）：双臂同向平移位移。
    - `spread_half`（`float`，默认取 `dual_arm.carry.arm_merge_distance_y`，否则 `0.04`）：单侧外张距离。
    - `retreat_xyz`（`[float,float,float]`，默认取 `dual_arm.carry.ee_retreat_offset`，否则 `[-0.2, 0.0, 0.0]`）：外张后的后撤位移。
    - `motion_frame_id` / `relative_frame_id`（`str`）：几何计算与下发坐标系。
    - `tf_lookup_timeout`（`float`，默认 `2.0`）：TF 查询超时（秒）。

  - **末端系配置**
    - `stage_prefix`（`str`，默认 `PlaceRel`）：阶段名前缀。
    - `arm_movel_duration`（`float`，默认取 `dual_arm.carry.arm_movel_duration`）：覆盖笛卡尔段 movel 时长。
- **备注**：需要左右臂当前位姿可读；设置 `motion_frame_id` 时依赖 TF 变换；若未配置 `dual_arm.carry`，上述复用默认将回退到内置默认值。

### 2.4 双臂对齐

#### `dual_arm.bimanual_align`

- **效果**：保持双臂相对几何不变，整体把中点对齐到目标位置；可附加姿态增量。
- **参数**：
  - **运动系目标配置**
    - `align_position`（`[float,float,float]`）：`motion_frame_id` 坐标系下的中点目标位置（必填）。
    - `orientation_delta_rpy`（`[float,float,float]`）：在该运动系下定义的附加姿态增量（弧度）。
    - `motion_frame_id`（`str`）：对齐计算坐标系。
  - **执行控制**
    - `arm_movel_duration`（`float`）：覆盖笛卡尔段 movel 时长。
    - `stage_name`（`str`）：阶段名。

### 2.5 双臂交接物体

#### `dual_arm.handover`

- **效果**：执行双臂同步交接段（两臂同步到交接位 -> 接收侧闭合 -> 给出侧松开）。
- **参数**（来自 `skill_defaults.dual_arm.handover` / 场景覆盖）：
  - **运动系目标配置**
    - `handover_position`（`[x,y,z]`）：交接基准位置（米，`motion_frame_id` 坐标系）。
    - `left_handover_orientation` / `right_handover_orientation`（`[x,y,z,w]`）：左右臂在交接点的末端姿态四元数（xyzw）。
  - **接收侧补偿配置**
    - `receiver_handover_offset`（`[x,y,z]`，默认 `[0,0,0]`）：仅加到**接收臂**目标位；用于微调两手接触间隙（例如避免提前碰撞、补偿夹爪厚度/模型偏差）。
  - **执行控制**
    - `motion_frame_id`（`str`，可选）：交接目标输出坐标系；不写则沿用任务 `frame_id`。
    - `tf_lookup_timeout`（`float`，可选）：`motion_frame_id` 与任务坐标系不一致时的 TF 查询超时（秒）。
    - `arm_movel_duration`（`float`，可选）：执行前写入 `arm_controller.movel_duration`。
- **备注**：
  - 给出侧 / 接收侧无需单独配置：由 `single_arm.pick.arm`（合并后与 `common.arm` 一致）自动推导；抓取臂为给出侧，另一臂为接收侧。
  - 交接段仅负责“交换物体”，不包含后续放置；通常后接 `single_arm.place` 或 `dual_arm.goto_cache_pose`。



https://github.com/user-attachments/assets/db2da0f2-961a-4607-866d-6c431db5126f



### 2.6 回程

#### `dual_arm.goto_cache_pose`

- **效果**：双臂同步回到 `robot.cache_ee_pose(which=both)` 写入的左右缓存位姿。
- **参数**：
  - **缓存与选择**
    - `key`：缓存键，默认 `saved_ee_pose`。
  - **末端系配置**
    - `gripper`：统一覆盖左右夹爪。
    - `left_gripper` / `right_gripper`：按侧覆盖。
  - **执行控制**
    - `stage_name`：阶段名。
- **备注**：缓存必须是双臂结构 `{left,right}`。

---

## 3. 导航技能（`nav.*`）

### 3.1 `nav.send_nav_goal`

- **效果**：非阻塞发送导航目标。
- **参数**：
  - **目标位姿配置**
    - `x`（`float`，必填）：目标 X（米）。
    - `y`（`float`，必填）：目标 Y（米）。
    - `yaw`（`float`，默认 `0.0`）：目标偏航（弧度）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。

### 3.2 `nav.wait_nav_arrived`

- **效果**：阻塞等待当前导航完成，并刷新 `ctx.base_world_pos/quat`。
- **参数**：
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

### 3.3 `nav.navigate_to_pose`

- **效果**：发送并等待导航到固定目标（等价 send + wait）。
- **参数**：
  - **目标位姿配置**
    - `x`（`float`，必填）：目标 X（米）。
    - `y`（`float`，必填）：目标 Y（米）。
    - `yaw`（`float`，默认 `0.0`）：目标偏航（弧度）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

### 3.4 `nav.navigate_to_object`

- **效果**：先查对象世界坐标，再按偏移导航到对象附近。
- **参数**：
  - **目标物体配置**
    - `object_prim_path`（`str`）：目标物体 Prim 路径；未给时尝试 `ctx.task_cfg.object_prim_path`。
  - **目标位姿配置**
    - `approach_offset_x`（`float`，默认 `-0.20`）：导航点 X 偏移（米）。
    - `approach_offset_y`（`float`，默认 `0.0`）：导航点 Y 偏移（米）。
    - `yaw`（`float`，默认 `0.0`）：目标偏航（弧度）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

### 3.5 `nav.navigate_backup`

- **效果**：沿基座 +X 反方向后退指定距离。
- **参数**：
  - **目标位姿配置**
    - `distance_m`（`float`，默认 `0.5`）：后退距离（米）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

---

## 4. 关节空间技能（`joint.*`）

### 4.1 `joint.movej_to_config`

- **效果**：发送关节目标（躯干/左臂/右臂任意组合），等待到位，可选恢复 OCS2。
- **参数**：
  - **关节目标配置**
    - `body_positions`（`list[float]`）：躯干关节目标。
    - `left_arm_positions` / `right_arm_positions`（`list[float]`）：左右臂关节目标。
  - **到达判定配置**
    - `arrival_timeout`（`float`，默认 `30.0`）：关节到达超时（秒）。
    - `joint_tolerance`（`float`，默认 `0.05`）：关节到达容差（弧度）。
  - **执行控制**
    - `resume_ocs2`（`bool`，默认 `false`）：完成后是否切回 OCS2。
    - `skip_fsm_change`（`bool`，默认 `false`）：是否跳过开头 FSM 切换。
    - `body_movej_duration`（`float`）：动态设置 body 控制器 `movej_duration`。
- **备注**：`resume_ocs2=true` 常用于后续紧接笛卡尔技能。

### 4.2 `joint.goto_cached_joints`

- **效果**：回到 `robot.cache_joint_state` 缓存的关节角。
- **参数**：
  - **缓存与执行控制**
    - `key`（`str`，默认 `saved_joint_state`）：缓存键。
    - `resume_ocs2`（`bool`，默认 `false`）：完成后是否切回 OCS2。
- **备注**：缓存键必须与 `robot.cache_joint_state` 对应。

---

## 5. 会话与机器人状态技能（`session.*` / `robot.*`）

### 5.1 `session.scratch_put`

- **效果**：写入会话暂存字典 `ctx.scratch`。
- **参数**：
  - **键值配置**
    - `key`（`str`，必填）：写入键。
    - `value`（`any`，必填）：写入值。

### 5.2 `session.scratch_clear`

- **效果**：清空 scratch，或按前缀删除。
- **参数**：
  - **键值配置**
    - `prefix`（`str`）：前缀删除；为空时清空全部。

### 5.3 `robot.cache_ee_pose`

- **效果**：缓存末端位姿与夹爪目标，供 `goto_cache_pose` 使用。
- **参数**：
  - **缓存与选择**
    - `key`（`str`，默认 `saved_ee_pose`）：缓存键。
    - `which`（`str`，默认 `work`）：`work|left|right|both`。
  - **末端系配置**
    - `gripper`（`float`）：默认夹爪值。
    - `left_gripper` / `right_gripper`（`float`，仅 `which=both`）：按侧覆盖夹爪值。
- **备注**：
  - `which=both` 写入 `{left,right}` 结构；
  - 单臂缓存写入 `{pose,gripper}` 结构。

### 5.4 `robot.cache_joint_state`

- **效果**：缓存当前关节角，供 `joint.goto_cached_joints` 使用。
- **参数**：
  - **缓存与采样配置**
    - `key`（`str`，默认 `saved_joint_state`）：缓存键。
    - `include_body`（`bool`，默认 `true`）：是否包含躯干关节。
    - `wait_timeout`（`float`，默认 `2.0`）：采样等待超时（秒）。
    - `poll_period`（`float`，默认 `0.05`）：采样轮询周期（秒）。
- **备注**：写入结构为 `{left,right,body}`（值为列表或 null）。

### 5.5 `robot.snapshot_state`

- **效果**：抓取关节状态、末端位姿、body target、fsm 等快照写入 scratch。
- **参数**：
  - **缓存与采样配置**
    - `key`（`str`，默认 `robot_state`）：缓存键。
    - `include_joint_state`（`bool`，默认 `true`）：是否记录关节状态。
    - `joint_state_categorized`（`bool`，默认 `false`）：是否读取分类关节结构。
    - `include_left_pose` / `include_right_pose`（`bool`，默认 `true`）：是否记录左右末端位姿。
    - `include_body_target`（`bool`，默认 `true`）：是否记录 body target。
    - `include_fsm`（`bool`，默认 `true`）：是否记录 FSM 命令值。

---

## 6. 配置项放置建议（简）

- 技能默认参数：`skill_defaults`
- 场景差异参数：`scene_presets.<scene>.skill_params`
- 单步临时覆盖：该块 `params`
- 流程公共字段（非技能）：`base_task_overrides`

详细结构与合并规则见：[`TASK_CONFIG_YAML.md`](TASK_CONFIG_YAML.md)
