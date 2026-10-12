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
| ``objects`` / ``privileged_pose`` / ``project_uv`` / ``scene_prep`` / ``auto_grasp`` | Sim-privilege select → UV → auto (optional exile) |
| ``service`` | Viser: manual click path + **Privileged auto** folder |

Docs:

- [docs/GRASP_GENERATION.md](../../docs/GRASP_GENERATION.md)
- [docs/CLOSE_EXECUTORS.md](../../docs/CLOSE_EXECUTORS.md)

```bash
pip install -e ".[grasp]"
# ros2-stack already up:
grasp-generation serve --robot fiveages_w2 --workspace examples/IsaacSim
```
