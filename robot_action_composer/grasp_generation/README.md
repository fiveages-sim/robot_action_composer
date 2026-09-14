# Grasp generation (optional)

Optional package + CLI for sim grasp (HUG → wrist). Hard-dep free: composer installs without HUG.

| Piece | Role |
|-------|------|
| CLI ``grasp-generation`` | Beside ``ros2-stack`` / ``motion-generation``; does **not** replace them |
| ``HugAdapter`` / ``hug_worker`` | Propose in hug `.venv` (py3.10) |
| ``bridge`` / ``frames`` | ``T_camera_wrist`` → ``base_link`` → ``send_target_stamped`` |
| ``service`` | Viser: capture → UV → propose → plan/move |

Docs: [docs/GRASP_GENERATION.md](../../docs/GRASP_GENERATION.md)

```bash
pip install -e ".[grasp]"
# ros2-stack already up:
grasp-generation serve --robot fiveages_w2 --workspace examples/IsaacSim
```
