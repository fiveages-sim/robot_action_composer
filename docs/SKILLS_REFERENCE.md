# 技能库与配置项说明（按技能）

**与 [`TASK_CONFIG_YAML.md`](TASK_CONFIG_YAML.md) 的分工**：该文档说明 **YAML 文件长什么样、如何发现/合并、顶层字段与 `task_queue` 写法**；**本文只说明每个已注册 `skill` 的语义与参数**（效果、键名、必填与备注），不重复加载与合并算法。

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

### 1.1 末端移动到指定位姿（`single_arm.move_to_pose`）

- **效果**：将末端运动到指定笛卡尔位姿（单段 MoveL），可指定参考帧与工作臂。
- **参数**：
  - **执行控制**
    - `arm`：`left|right`，必填。
  - **运动系配置**
    - `position`：目标位置 `[x, y, z]`（米），必填。
    - `orientation`：目标姿态四元数 `[x, y, z, w]`，必填。
    - `motion_frame_id`：位姿参考帧；不写则沿用任务级 `frame_id`（通常为 `arm_base`）。
  - **末端系配置**
    - `gripper`：目标夹爪值，默认保持上一步结束时的状态。
- **备注**：位姿在 ``motion_frame_id`` 下给出绝对目标，非相对当前末端。

### 1.1b 相对当前末端平移旋转（`single_arm.move_relative`）

- **效果**：读取当前末端位姿，变换到 ``motion_frame_id`` 后叠加位置增量并左乘姿态增量（单段 MoveL）。
- **参数**：
  - **执行控制**
    - `arm`：`left|right`，必填。
  - **运动系配置**
    - `position_delta`：`[dx, dy, dz]`（米），默认 `[0,0,0]`。**沿 ``motion_frame_id`` 的坐标轴叠加**，不是末端工具系。
    - `orientation_delta_rpy`：`[roll, pitch, yaw]`（弧度），在 ``motion_frame_id`` 下定义，左乘当前四元数；例如绕 Y 轴 +90° 为 `[0, 1.5708, 0]`。
    - `motion_frame_id`：增量解释坐标系；默认与末端反馈 frame 一致。写 `arm_base` / `base_footprint` / `base_link` 等可相对基座平移（需 TF 可达）。
    - `tf_lookup_timeout`：与任务 frame 不一致时的 TF 超时（秒）。
  - **执行控制**
    - `gripper`：目标夹爪值，默认 ``gripper_for_return_home``。
    - `arm_movel_duration`：可选；执行前写入 `arm_controller.movel_duration`（轨迹规划时长，**不是**到位等待超时；后者见 `runtime_defaults.max_stage_duration`）。
- **备注**：
  - WBC 双臂耦合（`ARMS_COUPLED`）后，通常只发左臂目标，右臂由耦合跟随：`arm: left`。
  - `arm_movel_duration` 与到位等待是两套时间；慢轨迹时请同时加大 `max_stage_duration`。
- **YAML 示例**：
  ```yaml
  - skill: single_arm.move_relative
    params:
      arm: left
      motion_frame_id: base_footprint
      position_delta: [0.2, 0.0, 0.3]
      orientation_delta_rpy: [0.0, -1.57, 0.0]
      arm_movel_duration: 4.0
      gripper: 0.0
  ```

### 1.2 末端移动到目标物体相对位姿（`single_arm.move_to_object`）

- **效果**：基于物体位姿 + 物体系偏移，生成单个笛卡尔目标并移动到位。
- **参数**：
  - **目标物体配置**
    - `object_prim_path`：目标物体 Prim 路径（必填）。
    - `object_position_offset`：相对物体中心的偏移（物体局部坐标系；按物体当前姿态旋转后叠加）。
  - **运动系配置**
    - `ee_base_orientation`：目标末端姿态四元数 `xyzw`（直接作为到位姿态）。
    - `motion_frame_id`：目标输出坐标系（可选；不写则沿用任务 `frame_id`）。
    - `tf_lookup_timeout`：当 `motion_frame_id` 与任务坐标系不一致时的 TF 查询超时（秒）。
  - **执行控制**
    - `arm`：`left|right`（不写则沿用 `common.arm`）。
    - `arm_movel_duration`：可选；执行前写入 `arm_controller.movel_duration`。
    - `gripper`：可选；默认保持当前 `gripper_for_return_home`。

### 1.3 抓取（`single_arm.pick`）

- **效果**：执行单臂抓取序列（approach / close-in / grasp / retreat）。
- **参数**（来自 `single_arm.pick` 切片）：
  - **目标物体配置**
    - `object_prim_path`：抓取对象刚体 Prim（可与任务 `objects` 绑定补齐）。
    - `object_position_offset`：物体系抓取偏移。若 **显式写出** 则走旧路径；未写且提供 `grasp_id` / `grasp_prim_path` 时由 USD 局部 frame 自动解析。
    - `object_key` / `grasp_id` / `grasp_prim_path`：可选；见 [`TASK_CONFIG_YAML.md`](TASK_CONFIG_YAML.md) 物体清单。
  - **运动系配置**
    - `ee_base_orientation`：抓取姿态四元数 `xyzw`。语义为**某一标称机物相对朝向下的期望抓取方向**（可含斜抓；见 §6.2）。
    - `ee_orientation_frame`：`motion`（默认，直接用标定姿态）| `object`（按相对标称的物体姿态偏差左乘）。
    - `object_orientation_mode`：配合 `object` 帧；默认 `yaw`（仅绕竖直 Z）；亦可 `full` / `tilt` / `pitch` / `none`。
    - `aligned_object_yaw`：标称物体 yaw。数值（弧度），或 `auto`（吸附到最近的 `k·π/2`）。详见 §6.2。
    - `motion_frame_id`：抓取目标输出坐标系（可选；不写则沿用任务 `frame_id`）。
    - `tf_lookup_timeout`：当 `motion_frame_id` 与任务坐标系不一致时的 TF 查询超时（秒）。
  - **末端系配置**
    - `pick_clearance`：闭合抓取阶段偏移（工具系位移，沿末端局部轴向解释）。
    - `prepare_offset`：预接近偏移（工具系向量，语义与 `dual_arm.carry.carry_prepare_offset` 一致）。
    - `ee_lift_offset`：抓取后抬升位移（工具系向量；按**有效**末端姿态旋转后用于抬升段）。
    - `ee_retreat_offset`：抬升后后撤位移（工具系向量；按**有效**末端姿态旋转后用于后撤段）。
    - `ee_pick_axis`：末端抓取轴向字符串（与 `ee_place_axis` 对称）；`+x/-x/+y/-y/+z/-z`，默认 `+z`。
    - `ee_pick_direction_vector`：抓取后撤等所用的方向单位向量 `[x,y,z]`（可选；非空时覆盖 `ee_pick_axis`）。
  - **执行控制**
    - `arm`：`left|right`。
    - `arm_movel_duration`：可选；执行前写入 `arm_controller.movel_duration`。
- **备注**：
  - 旧 YAML（仅 `object_prim_path` + `object_position_offset`）行为不变；`object_position_offset` 始终在物体系。
  - `runtime_defaults.use_object_orientation: true` 时，若未写 `ee_orientation_frame`，等价于 `object`（兼容旧字段）。
  - 机物偏航对齐推荐配置见 §6.2。

### 1.4 放置（`single_arm.place`）

- **效果**：执行单臂放置序列（在 `task_queue` 中出现即执行）；既支持放置到固定坐标（`place_position`），也支持基于参考物体坐标动态计算放置目标（`object_prim_path` + `object_position_offset`）。
- **参数**（来自 `single_arm.place` 切片）：
  - **目标物体配置**
    - `object_prim_path`：放置参考物体 Prim 路径（可解析放置位）。
    - `object_position_offset`：相对参考物体中心的放置偏移（物体局部坐标系；会按物体当前姿态旋转后叠加到放置目标）。
  - **运动系配置**
    - `place_position`：显式放置位置（运动系坐标；当未设置 `object_prim_path` 时，直接按该坐标放置）。
    - `ee_base_orientation`：放置姿态四元数 `xyzw`（**未设置时默认保持当前末端姿态**，与 `dual_arm.place` 一致）。
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
  - 未设置 `ee_base_orientation` 时，读取当前末端姿态（`motion_frame_id` 下）作为放置姿态；
  - 未设置 `ee_place_axis` 时，默认继承 `single_arm.pick.ee_pick_axis`（默认 `+z`）。

### 1.5 回程缓存末端（`single_arm.goto_cache_pose`）

- **效果**：回到 `robot.cache_ee_pose` 记录的缓存末端位姿。
- **参数**：
  - **缓存与选择**
    - `key`：缓存键，默认 `saved_ee_pose`。
    - `side`：当缓存为 `which: both` 时选 `left|right`。
  - **末端系配置**
    - `gripper`：覆盖缓存中的夹爪目标。
- **备注**：
  - 单臂缓存可直接用；
  - 双臂缓存必须指定 `side` 或依赖 `common.arm` 推断。

### 1.6 抽屉（`single_arm.drawer.*`）


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

#### 1.6.1 拉开抽屉（`single_arm.drawer.pull_open`）

- **效果**：按几何生成拉开序列；写入 **`ctx.drawer`**，更新 **`ctx.task_cfg.place`** 放置提示；不改 **`ctx.task_cfg.pick`**。
- **备注**：需有效 `object_prim_path` 与 `object_position_offset`；臂别由 **`common.arm`** 决定。

#### 1.6.2 关上抽屉（`single_arm.drawer.close_push`）

- **效果**：须队列中已有 **`pull_open`**。粗定位 → 关抽屉 Cartesian → 闭合到位 **`place_pose_ref`** → 按 **`ee_retreat_offset`** 配置松爪并撤出。
- **备注**：必须排在 **`pull_open`** 之后。



https://github.com/user-attachments/assets/3ae2fb74-8933-49fc-972e-f9ab8237a53b



---

## 2. 双臂技能（`dual_arm.*`）

### 2.1 搬运（carry）

> 搬运统一使用单个 `dual_arm.carry`。  
> 配置主要来自 `dual_arm.carry`（即 `skill_defaults.dual_arm.carry` / `scene_presets.*.skill_params.dual_arm.carry`）。
> PoC 项目里你看到的大部分字段（如 `orientation_delta_rpy`、`ee_pregrasp_lift_offset`、`ee_lift_offset`）都属于这层配置，而不是技能块 `params`。

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
  - `ee_pregrasp_lift_offset`：**可选**；在 `CloseIn`（夹爪开）之后、`Grasp`（夹爪闭）之前插入一段抬升/平移（工具系向量）。
  - `ee_lift_offset`：抓取后抬升位移（是否生成抬升段取决于该值是否为空）。
  - `ee_retreat_offset`：抬升后后撤位移（是否生成后撤段取决于该值是否为空）。
  - `arm_movel_duration`：执行前写入臂控制器的 `movel_duration`（调节笛卡尔段时长）。

#### `dual_arm.carry`

- **效果**：一次执行完整搬运序列。
- **阶段顺序**：
  - 无 `ee_pregrasp_lift_offset`：`Approach`(可选) -> `Forward` -> `CloseIn` -> `Grasp` -> `Lift`(可选) -> `Retreat`(可选)
  - 有 `ee_pregrasp_lift_offset`：`Approach`(可选) -> `Forward` -> `CloseIn` -> `PregraspLift` -> `Grasp` -> `Lift`(可选) -> `Retreat`(可选)
- **参数**：无块级专用参数（读取 `dual_arm.carry` 配置）。

https://github.com/user-attachments/assets/1e4e9d34-e5ba-4aa7-8e70-6c5a9de7631f



### 2.2 双臂同时抓取（parallel_pick）

> 该技能从当前块 `params` 解析 `BimanualParallelPickTaskConfig`。  
> 必须提供 `left_pick` 与 `right_pick` 两个映射。

`left_pick` / `right_pick` 字段与 `single_arm.pick` 切片语义一致（每侧一套）：

- **目标物体配置**
  - `object_prim_path`：该侧抓取刚体 Prim（可与 `objects` / `active_object` 补齐）。
  - `object_position_offset`：物体系偏移；**显式写出**则覆盖自动 grasp。
  - `object_key` / `grasp_id` / `grasp_prim_path`：可选；见任务 YAML 物体清单。

- **运动系配置**
  - `motion_frame_id`：该侧物体位姿解析后的输出坐标系（可选；不写则沿用任务 `frame_id`）。**左右两侧解析后必须相同**，否则双臂同步下发无法共用一个坐标系。
  - `tf_lookup_timeout`：TF 查询超时（秒）；本块取左右两侧的较大值用于两侧物体位姿变换。
  - `ee_orientation_frame`（块级或每侧，默认 `motion`）：`motion` 直接用标定 `ee_base_orientation`；`object` 则按相对标称的物体姿态偏差左乘。
  - `object_orientation_mode`（块级或每侧，默认 `yaw`）：配合 `ee_orientation_frame=object`。推荐 `yaw`；亦可 `full` / `tilt` / `pitch` / `none`。
  - `aligned_object_yaw`（块级或每侧，默认 `auto`）：标称物体 yaw。`auto` = 吸附到最近的 `k·π/2`（0 / ±1.57 / ±3.14）；也可写显式弧度。校正量 = `wrap(yaw_object − aligned)`。

- **末端系配置**
  - `ee_base_orientation`：抓取姿态四元数 `xyzw`。语义为**标称机物相对朝向下的期望抓取方向**（含导航到位后的固定转角关系）；开启 `ee_orientation_frame: object` 后只再乘相对 `aligned_object_yaw` 的偏差。
  - `pick_clearance`：闭合抓取阶段偏移（工具系，沿末端局部 +Z 解释，与 `build_single_arm_pick_sequence` 一致）。
  - `prepare_offset`：预接近偏移（工具系三维向量；全 0 或省略时可省略预接近段）。
  - `ee_lift_offset` / `ee_retreat_offset`：抬升与后撤（工具系向量；按**有效**末端姿态旋到运动系，与 `single_arm.pick` 一致）。
  - `ee_pick_axis` / `ee_pick_direction_vector`：与单臂 `QueueSlicePick` 相同（轴向或显式单位向量）。

- **执行控制**
  - `arm_movel_duration`：可选；优先取 `left_pick`，否则取 `right_pick`，写入 `arm_controller.movel_duration`。

#### `dual_arm.parallel_pick`

- **效果**：一次执行完整双臂同时抓取序列（两侧各走一条 `build_single_arm_pick_sequence`，再按阶段同步合成）。
- **参数**：读取 `left_pick` / `right_pick`（见上方字段列表）；块级 `ee_orientation_frame` / `object_orientation_mode` / `aligned_object_yaw` 可两侧共用。
- **备注**：
  - 执行完成后会将 `gripper_for_return_home` 置为闭合值（与单臂 pick 行为一致）。
  - `object_position_offset` 始终在物体系；抓取点位置本身已随物体朝向变。
  - 机物偏航对齐、斜抓与 `aligned_object_yaw` 用法见 **§6.2**。
  - 常用写法：
    ```yaml
    # 导航 0/90/180° 档位：自动吸附标称偏航
    dual_arm.parallel_pick:
      ee_orientation_frame: object
      object_orientation_mode: yaw
      aligned_object_yaw: auto
      left_pick: { ... ee_base_orientation: [1,0,0,0], ... }
      right_pick: { ... }

    # 斜向接近（如标称相对偏航 45°）：显式角度，勿用 auto
    dual_arm.parallel_pick:
      ee_orientation_frame: object
      object_orientation_mode: yaw
      aligned_object_yaw: 0.7854
      left_pick: { ... }   # ee_base_orientation 在该 45° 标称下标定（可含斜抓姿态）
      right_pick: { ... }
    ```



https://github.com/user-attachments/assets/3464ad42-e5b1-4035-9aa3-f78b69cb3029



### 2.3 放置（place）

#### `dual_arm.place`

- **效果**：执行双臂相对放置序列（同向平移 -> 松爪 -> 可选开爪后下降 -> 外张 -> 后撤）。默认按当前末端做相对位移；若提供参考物体配置（`reference_object_prim_path` / `reference_object_key` + `object_position_offset`），则会先计算目标放置点，再自动换算本次平移量。
- **默认值复用**：当未显式配置放置参数时，会复用 `dual_arm.carry` 的部分几何/节奏配置作为默认值（因此 `place` 常可只写少量参数）。
- **参数**：
  - **目标物体配置**
    - `reference_object_prim_path`（`str`）：参考物体 Prim 路径。
    - `reference_object_key` / `object_key`（`str`）：从任务 `objects` / `.meta/objects` 解析 prim（显式 prim 优先）。
    - `object_position_offset`（`[float,float,float]`）：参考物体坐标系下偏移。

  - **运动系配置**
    - `translation_xyz`（`[float,float,float]`，默认 `[0,0,0]`）：双臂同向平移位移。
    - `post_release_lower_xyz`（`[float,float,float]`）：**可选**；松爪后、外张前的同向位移（常用于“先抬后抓”的对称下降）。
    - `spread_half`（`float`，默认取前序 `dual_arm.carry.arm_merge_distance_y`；无 carry 时为 `0`）：单侧外张距离；为 `0` 时跳过外张段。
    - `retreat_xyz`（`[float,float,float]`，默认取 `dual_arm.carry.ee_retreat_offset`，否则 `[-0.2, 0.0, 0.0]`）：外张后的后撤位移。
    - `motion_frame_id` / `relative_frame_id`（`str`）：几何计算与下发坐标系。
    - `tf_lookup_timeout`（`float`，默认 `2.0`）：TF 查询超时（秒）。

  - **末端系配置**
    - `arm_movel_duration`（`float`，默认取 `dual_arm.carry.arm_movel_duration`）：覆盖笛卡尔段 movel 时长。
- **备注**：需要左右臂当前位姿可读；设置 `motion_frame_id` 时依赖 TF 变换；若未配置 `dual_arm.carry`，上述复用默认将回退到内置默认值。
  - 当未显式给 `post_release_lower_xyz` 时：
    - 若 `dual_arm.carry.ee_pregrasp_lift_offset` 存在，默认取其相反数（实现开爪后对称下降）；
    - 且若 `dual_arm.carry.ee_lift_offset` 也存在，会优先映射到**闭爪下降段**；开爪下降段会自动变为剩余位移（两段合计仍到参考 offset）；
    - 否则不生成该下降段。

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
    - `object_prim_path`（`str`）：目标物体 Prim 路径。
    - `object_key`（`str`）：从任务 `objects` / `.meta/objects` 解析 prim（与 prim 二选一；显式 prim 优先）。不回退 `active_object`。
  - **目标位姿配置**
    - `approach_offset_x`（`float`，默认 `-0.20`）：导航点 X 偏移（米）。
    - `approach_offset_y`（`float`，默认 `0.0`）：导航点 Y 偏移（米）。
    - `yaw`（`float`，默认 `0.0`）：目标偏航（弧度）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

### 3.5 `nav.navigate_relative`

- **效果**：按基座当前朝向做相对导航位移。
- **参数**：
  - **目标位姿配置**
    - `relative_position`（`[float,float,float]`，必填）：相对运动三元组 `[dx, dy, dyaw]`。
      - `dx`：沿当前前向(+) / 后向(-)位移（米）。
      - `dy`：沿当前左向(+) / 右向(-)位移（米）。
      - `dyaw`：在当前偏航基础上叠加的旋转量（弧度）。
    - `frame_id`（`str`，默认 `map`）：目标坐标系。
  - **执行控制**
    - `timeout`（`float`，默认 `60.0`）：最大等待时间（秒）。
    - `poll_period`（`float`，默认 `0.1`）：轮询周期（秒）。

---

## 4. 关节空间技能（`joint.*`）

### 4.1 `joint.movej_to_config`

- **效果**：合并绝对/相对目标后经 `send_coordinated_joint_positions` 下发（躯干/左臂/右臂/头任意组合），等待到位。FSM 由 `ROS2RobotInterface` 按 WBC/分体自动切换；**不**主动恢复 OCS2（后续笛卡尔 skill 会自行切换）。
- **参数**：
  - **绝对目标**（整组或局部）
    - `body_positions` / `left_arm_positions` / `right_arm_positions` / `head_positions`：绝对目标列表。
      元素可为 `null`，表示**该关节保持当前值**（部分关节设定）。
  - **相对增量**（相对**当前**关节角，rad）
    - `body_position_deltas` / `left_arm_position_deltas` / `right_arm_position_deltas` / `head_position_deltas`：与绝对列表同长；`null` = 该关节增量 0。
    - `body_joint_deltas` / `left_arm_joint_deltas` / `right_arm_joint_deltas` / `head_joint_deltas`：稀疏写法 `{关节索引: delta}`（**0-based**），便于只动一两个关节。
  - **到达判定配置**
    - `arrival_timeout`（`float`，默认 `30.0`）：关节到达超时（秒）。
    - `joint_tolerance`（`float`，默认 `0.05`）：关节到达容差（弧度）。
  - **执行控制**
    - `skip_fsm_change`（`bool`，默认 `false`）：为 true 时传 `auto_switch_fsm=False`，跳过 interface 自动 FSM。
    - `body_movej_duration`（`float`）：动态设置 body 控制器 `movej_duration`。
- **合并规则**：先读当前角；绝对目标（`null` 用当前值填）与 deltas / sparse deltas 合并为**同一组**下发目标；到位判断使用合并后的目标，且**只检查 YAML 显式写过的关节索引**（`null` 保持位不参与判到位）；旋转关节用最短角距离。
- **备注**：WBC 抓取后腰角往往不是标称 0；腰转 180° 应用相对增量，不要写死绝对角。
- **YAML 示例**：
  ```yaml
  # 只相对转动腰偏航（第 4 关节，index=3）+π
  - skill: joint.movej_to_config
    params:
      body_joint_deltas: {3: 3.1416}
      body_movej_duration: 3.0

  # 等价列表写法
  - skill: joint.movej_to_config
    params:
      body_position_deltas: [null, null, null, 3.1416]

  # 部分绝对：只把腰偏航设到 π，其余腰关节保持当前
  - skill: joint.movej_to_config
    params:
      body_positions: [null, null, null, 3.1416]
  ```

### 4.2 `joint.goto_cached_joints`

- **效果**：回到 `robot.cache_joint_state` 缓存的关节角。
- **参数**：
  - `key`（`str`，默认 `saved_joint_state`）：缓存键。
- **备注**：缓存键必须与 `robot.cache_joint_state` 对应；不主动恢复 OCS2。

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

### 5.6 `robot.send_mode_command`

- **效果**：向 `/mode_command` 发布 WBC / 底盘模式字符串（无笛卡尔阶段，仅副作用）。
- **参数**：
  - **模式配置**
    - `command` / `mode`（`str` 或 `list`）：单条或列表。如 `ARMS_COUPLED`、`BODY_TRACKING`、`BODY_FREE`、`BODY_LOCK`、`BODY_HEAD_COUPLED`、`BASE_LOCK`、`BASE_UNLOCK`。
    - `commands`（`list`，可选）：多条模式命令，按顺序发布（适合「追踪 + 双臂耦合」）。
    - `wait_for_state`（`bool`，默认 `true`）：发完全部命令后，一次性轮询 `/ocs2_wbc_controller/current_state`，要求所有期望字段同时满足。
    - `wait_timeout`（`float`，默认 `5.0`）：整批确认超时（秒）；超时打 WARN 后继续。
    - `settle_time`（`float`，可选）：确认后额外固定等待；接口内置约 `0.1s`，更大时补睡差额。
- **备注**：
  - 只发 `/mode_command`，不切换 FSM；后续笛卡尔 skill 会按需自动切 OCS2。
  - `BODY_TRACKING` → `body_state==BODY_TRACKING`；`ARMS_COUPLED` → `bimanual_state==BIMANUAL_COUPLED`；多条合并检查（同字段以后者为准）。
  - 命令字符串需与底层控制器约定一致；接口对未知命令原样发布（无 current_state 映射则跳过等待）。
- **YAML 示例**：
  ```yaml
  skill_defaults:
    enable_arms_coupled:
      command: ARMS_COUPLED
    enable_tracking_and_coupled:
      commands: [BODY_TRACKING, ARMS_COUPLED]

  task_queue:
    - skill: dual_arm.parallel_pick
    - skill: robot.send_mode_command
      id: enable_arms_coupled
    - skill: single_arm.move_relative
      id: coupled_left_move_relative
  ```

---

## 6. 配置项放在哪一层（速查）

与 § 开头「约定」一致：**默认** → `skill_defaults`；**按场景覆盖** → `scene_presets.<scene>.skill_params`；**仅本块** → `task_queue` 里该块的 `params`；**与具体技能无关、整任务共用** → `runtime_defaults`（仅 `base_link_entity_path` / `max_stage_duration` / `pose_tol_pos` / `pose_tol_ori`）。  
**合并顺序、禁止项与示例**以 [`TASK_CONFIG_YAML.md`](TASK_CONFIG_YAML.md) 为准（本节不展开）。

### 6.1 笛卡尔到位判定（`runtime_defaults`）

笛卡尔 skill（pick / move_relative / parallel_pick 等）由 `runner` 调用 `execute_stage_sequence` 等待到位：

| 字段 | 含义 | 注意 |
|------|------|------|
| `max_stage_duration` | **覆盖**机器人配置里的 `arrival_timeout`，作为每段笛卡尔到位等待上限（秒） | 与 `arm_movel_duration`（轨迹规划时长）无关；慢轨迹时两者都要够大 |
| `pose_tol_pos` | 位置容差（米），传给 `check_arrival` | 直接使用 |
| `pose_tol_ori` | 姿态容差，语义为四元数距离 `1-\|dot\|`（无量纲，通常 `0.03`～`0.08`） | 运行时会换算成**度**再交给 handler；切勿把 `0.03` 当成 0.03° |

> 关节 skill（`joint.movej_to_config`）仍用块内自己的 `arrival_timeout`，不受 `max_stage_duration` 覆盖。

### 6.2 机物偏航对齐（`single_arm.pick` / `dual_arm.parallel_pick`）

实现：`motion_generation/tasks/object_orientation.py`（`compose_aligned_ee_orientation`）。

**标定语义**

- `ee_base_orientation`、`prepare_offset`、抬升/后撤等，按某一**标称机物相对朝向**标定（常见：正对、侧对 π/2，或斜向如 π/4）。
- `object_position_offset` 始终在**物体系**，抓点位置本身会随物体转。
- 开启 `ee_orientation_frame: object` 后，运行时只补偿**相对标称的偏差**，不把「导航带来的固定转角」再乘一遍。

**公式（`object_orientation_mode: yaw`）**

```text
aligned = aligned_object_yaw        # 或 auto → snap(yaw_object) 到最近的 k·π/2
yaw_corr = wrap(yaw_object − aligned)
q_ee     = R_z(yaw_corr) ⊗ ee_base
```

工具系偏移（prepare / lift / retreat）用 **有效** `q_ee` 旋到运动系。

**`aligned_object_yaw` 怎么填**

| 取值 | 含义 | 适用 |
|------|------|------|
| `auto`（推荐默认） | 把物体在运动系的 yaw 吸附到最近的 `0 / ±π/2 / ±π` | 导航朝向落在 0°/90°/180° 档 |
| 显式弧度（如 `0.7854`、`-1.5708`） | 固定标称相对偏航 | 斜向接近、或不要自动吸附时 |
| `0` | 标称即机物 X 对齐（旧语义） | 无固定导航转角时 |

**斜抓**

- **手指斜向**：写进 `ee_base_orientation`（在选定的标称相对偏航下标定）。
- **斜向接近**（标称不是 90° 网格）：`aligned_object_yaw: <弧度>`，**不要**用 `auto`（否则会吸到 0/±90/180）。

**完整示例**

```yaml
# 侧对抓取（nav yaw≈1.57）+ 自动档位
dual_arm.parallel_pick:
  ee_orientation_frame: object
  object_orientation_mode: yaw
  aligned_object_yaw: auto
  left_pick:
    object_prim_path: /World/obj
    object_position_offset: [0.0, -0.05, -0.06]
    ee_base_orientation: [1.0, 0.0, 0.0, 0.0]
  right_pick: { ... }

# 斜 45° 标称接近 + 斜抓姿态
single_arm.pick:
  ee_orientation_frame: object
  object_orientation_mode: yaw
  aligned_object_yaw: 0.7854
  ee_base_orientation: [<斜抓四元数>]
  object_prim_path: /World/obj
```
