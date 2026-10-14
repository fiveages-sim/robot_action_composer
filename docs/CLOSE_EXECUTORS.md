# Close 执行器 B0–B5（中文）

HUG propose → 跟腕之后的 **合手阶段** 消融集。论文对齐：TacDexGrasp 接触门控 + CoorGrasp-lite ramp。

> **B2 ≠ B4**  
> - **B2**：开环慢速插值到 `squeeze(q_retarget)`（整段用力合，无接触门控）。  
> - **B4**：先开环 MANO shape，再以 **接触门控** 向 `squeeze` 放大后的目标靠近，接触后 **freeze MCP + tip ramp/pinch**。  
> `squeeze_amount` 对 B4 只定义 **目标位姿放大**，不是 B2 那种全程开环挤压。  
> `post_shape_squeeze_scale` **仅作用于 B2**（见 `execution/runner.py`）。

## 一键 Close（Viser「5) Close」）

1. 可选世界系 +Z 抬腕（`pre_close_lift_m`）
2. 重算 MANO → 手关节 `q_retarget`
3. **开环**插值到 MANO shape（可见；默认不门控）
4. `shape_settle_s` 短停
5. 跑所选执行器（B3+ 用接触；**桌面/地面 partner 已过滤，物体路径勿进 ignore**）

`4) Retarget` = 只做开环 MANO shape，不需要勾 `execute_move`。

## 模式对照

| Id | 行为 | 适用 |
|----|------|------|
| **B0** | Shape → MANO（不合挤） | 看 retarget |
| **B1** | 慢速开环 → retarget | 时序消融 |
| **B2** | Shape → 慢速 → `squeeze(q)` **开环** | 旧基线用力合 |
| **B3** | Shape → 接触门控 `keep_tip` | 减少推飞 |
| **B4** | B3 + ramp + **tip_pinch** | **默认 / 推荐** |
| **B5** | B4 + 腕微压（`tip_neg_z`） | 悬空/打偏 |

`keep_tip`：接触后冻 MCP（拇指冻到 MCP），指尖 PIP/DIP（拇指仅 IP）继续卷。

### B4 关键参数（`robots/<Robot>/grasp_generation.yaml`）

| 参数 | 含义 | 典型 |
|------|------|------|
| `squeeze_amount` | B4 **目标**相对 retarget 的放大（非 B2 开环挤压） | `0.5` |
| `ramp_steps` / `ramp_amount` / `tip_ramp_boost` | 接触后微 ramp | `25` / `0.18` / `0.35` |
| `tip_pinch_steps` / `tip_pinch_amount` | ramp 后指尖加捏 | `12` / `0.25` |
| `soft_stop_force_n` | 任一接触力 ≥ 此值（N）则中止继续卷，防接触后「假张开」 | `15` |
| `freeze_mode` | `keep_tip` \| `full` | `keep_tip` |
| `min_contact_fingers` | 够几指接触后可提前结束 approach 进 ramp | `2` |
| `post_close_firm_s` | close 结束后反复下发闭合 `q`（再 lift） | `2.0` |
| `post_grasp_settle_s` | auto 路径 lift 前 hold | `3.0` |

## 接触话题路径（相对 `ROS2RobotInterface`）

```
Isaac PhysX ContactSensor
  → OmniGraph（FaSim Hand2 Sensor=contact_force）
  → /wuji_rh/.../in_contact/<link>、contact_wrench、contact_partners
  → grasp_generation.execution.contact.WujiRosContactSensor（独立 spin，不经 Interface）
  → B3–B5 freeze

手指指令仍走 ROS2RobotInterface
  → /right_hand_controller/target_joint_position
```

`ignore_contact_partners` **不要**写 `default_environment` / 裸 `table`：道具在  
`/World/default_environment/EastRural_Table/sm_*` 下，会被误忽略。

## 目录

```
grasp_generation/execution/
  profiles/     # hand_profile → 关节组 / tip / 接触 link
  contact/      # Wuji ROS 接触
  executors/    # B0–B5
  runner.py     # shape → settle → close
```

## Isaac 前提

Wuji Hand2：`Side=right` + `Sensor=contact_force`（PhysX）。  
见 FaSim `.../Wuji/Hand2/payloads/Sensor/CONTACT_FORCE_README.md`。

## 诊断

```bash
# close 过程 JSONL（cmd / q_meas / force / frozen）
python -m robot_action_composer.grasp_generation.diagnose_close_log --latest-dir \
  submodules/hug/data/w2_wuji_head_capture/close_logs
```

## CLI

```bash
grasp-generation close-hand --robot fiveages_w2 --workspace . --mode B4 --trial-id t01
```
