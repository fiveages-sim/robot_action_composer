# Close executors (B0–B5)

Ablation suite for the **close phase** after HUG propose → wrist plan.
Aligned with thesis roadmap B0–B5 (TacDexGrasp gate + CoorGrasp-lite ramp).

## One-click Close (default)

`5) Close` does:

1. Optional **world +Z lift** (`pre_close_lift_m`, table clearance)
2. Recompute MANO → Wuji `q_retarget`
3. **Open-loop** interpolate to MANO shape (visible; not contact-gated)
4. Brief settle (`shape_settle_s` ≈ 0.5 s)
5. Run selected executor (B3+ use contact gate; **table/ground partners ignored**)

Button `4)` = shape only (same open-loop MANO). Does **not** need `execute_move`.

## Modes

| Id | Behavior | Paper affinity |
|----|----------|----------------|
| **B0** | Gated shape → settle (no squeeze) | Shape baseline |
| **B1** | Same as B0 when auto-shape on; else slow open-loop → retarget | Timing ablation |
| **B2** | Shape → slow → `squeeze(q)` | Prior squeeze |
| **B3** | Shape → gate close (`keep_tip`) | TacDexGrasp stage |
| **B4** | B3 + ramp + **tip_pinch** | CoorGrasp-lite (best so far) |
| **B5** | B4 + wrist micro-press (`tip_neg_z`) | Hover / miss |

`keep_tip`: on contact freeze MCP, keep PIP/DIP curling (helps lime-scale objects).

## Contact ROS path (vs `ROS2RobotInterface`)

```
Isaac PhysX ContactSensor
  → OmniGraph ScriptNode (FaSim Hand2 Sensor=contact_force)
  → ROS2 topics /wuji_rh/in_contact/<link>  (std_msgs/Bool)
              /wuji_rh/contact_wrench/<link>
  → grasp_generation.execution.contact.WujiRosContactSensor
       (own rclpy node + spin thread; NOT through ROS2RobotInterface)
  → B3–B5 / gated shape freeze logic

Hand joint commands still use ROS2RobotInterface
  → /right_hand_controller/target_joint_position
```

`ROS2RobotInterface` does **not** subscribe to Wuji contact topics today; contact is a parallel subscriber inside the grasp execution package.

## Layout

```
grasp_generation/execution/
  profiles/     # hand_profile → joint groups + tip joints + contact links
  contact/      # /wuji_{rh|lh}/in_contact/<link>
  executors/    # B0–B5
  runner.py     # shape → settle → close
```

## Isaac prerequisite

Wuji Hand2: `Side=right` + `Sensor=contact_force` (PhysX).  
See FaSim `.../Wuji/Hand2/payloads/Sensor/CONTACT_FORCE_README.md`.

## Gaps → next improvements (effect-first)

| Priority | Gap | Effect on grasp |
|----------|-----|-----------------|
| 1 | **Coordinated tip force** (not uniform ramp) | Less slip on lime; less over-squeeze on soft banana |
| 2 | Object **Δpose** feedback during close | Detect push-away early; abort / replan wrist |
| 3 | Soft **wrist approach** along object normal (fix B5) | Recover hover / thin objects |
| 4 | Full CoorGrasp MPC / Tac3D / SOCP | Later; needs tactile + force fidelity |

## CLI

```bash
grasp-generation close-hand --robot fiveages_w2 --workspace . --mode B4 --trial-id t01
```
