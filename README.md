# robot-action-composer

- **Motion / task queue**: uses **`ROS2RobotInterface` only** (no `ROS2Robot`). Runners call `interface.connect()` / `interface.disconnect()`; context field is `ctx.interface`.
- **Navigation** (`task_runtime/skills/navigation.py`): Nav2 helpers — `robot.send_nav_goal`, `robot.wait_nav_arrived`, `robot.navigate_to_pose`, `robot.navigate_to_object`, `robot.movej_to_config`; base pose refresh via `isaac_sim.get_entity_pose_world_service`.
- **Task assets** (`task_config_io.py`, `discovery/registry_loader.py`): YAML/py task discovery, `flatten_pick_place_task_overrides`, and `load_motion_entries` / `load_robot_entries` over `examples/IsaacSim/robots` (shared by motion, record, and `inference.py`).
- **Dataset recording** (`[recording]` extra): builds **`ROS2Robot`** for `get_observation()` / LeRobot datasets; motion during episodes uses **`robot.ros2_interface`** (same underlying connection).

## Install (monorepo)

```bash
pip install -e submodules/ros2_robot_interface
pip install -e submodules/robot_action_composer
# Recording:
pip install -e lerobot_robot_ros2
pip install -e "submodules/robot_action_composer[recording]"
```

ROS 2 Python (`rclpy`, messages, `cv_bridge`) comes from your ROS environment.

## Note

`drawer.py`, `isaac_sim`, and `drawer_queue` still import **`lerobot_robot_ros2.utils.pose_utils`** for quaternion helpers (not the `ROS2Robot` class). To drop that dependency, vendor those helpers into this package.
