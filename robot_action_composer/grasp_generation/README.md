# Grasp generation (optional)

Optional package + CLI for sim grasp (HUG → wrist → retarget → **close executors**).
Hard-dep free: composer installs without HUG.

| Piece | Role |
|-------|------|
| CLI ``grasp-generation`` | Beside ``ros2-stack`` / ``motion-generation``; does **not** replace them |
| ``HugAdapter`` / ``hug_worker`` | Propose in hug `.venv` (py3.10) |
| ``bridge`` / ``frames`` | ``T_camera_wrist`` → ``base_link`` → ``send_target_stamped`` |
| ``retarget/`` | Wuji Hand2 SDK retarget + squeeze amplify |
| ``execution/`` | Close-phase B0–B5 (contact gate / ramp / wrist); multi-hand ready |
| ``objects`` / ``privileged_pose`` / ``project_uv`` / ``auto_grasp`` | Sim-privilege select → UV → auto (table layout unchanged) |
| ``service`` | Viser: manual click path + **Privileged auto** folder |

文档（中文为主）：

- [docs/GRASP_GENERATION.md](../../docs/GRASP_GENERATION.md) — CLI / Viser / **MANO·Retarget·Execute 替换接口** / 仿真环境
- [docs/CLOSE_EXECUTORS.md](../../docs/CLOSE_EXECUTORS.md) — B0–B5（**B2≠B4**）、参数表、接触话题
- 任务 skill **`dex_hand.auto_grasp`**：[docs/SKILLS_REFERENCE.md](../../docs/SKILLS_REFERENCE.md) §3b

```bash
# 父仓环境，不要用系统 pip，也不要装进 hug/.venv
# 见仓库根 ./init.sh install-grasp
pip install "viser>=0.2"
# 参考手型 wuji_hand2 另需: pip install "wuji-sdk>=0.10.0"
# ros2-stack already up:
grasp-generation serve --robot fiveages_w2 --workspace examples/IsaacSim
```
