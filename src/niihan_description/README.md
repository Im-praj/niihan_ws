# niihan_description

Robot model, Gazebo construction-site simulation, SLAM/navigation configuration and motion utilities for NIIHAN.

## Entry points

Use `ros2 launch niihan_description niihan_gazebo.launch.py` for configurable simulation and `niihan_full_system.launch.py` to include the dashboard. `world`, `headless`, `seed`, `mapping`, `launch_nav2`, `vortex_3d`, `cliff_detect`, `patrol` and `rqt_cam` control simulation features. Spawn coordinates are configurable in the canonical launcher. Patrol and the camera viewer default off.

## Data

- `urdf/`: Xacro robot structure and simulated sensors.
- `worlds/niihan_construction_site.sdf`: static mesh-based construction scene.
- `models/construction_site_chunk5/`: optional proprietary collision meshes; excluded from Git.
- `config/`: Nav2, SLAM and existing patrol presets. Preset route safety requires inspection against the selected map.
- `test/`: kinematics, point-cloud, TF, launch clock, navigation and world contracts.

The arbiter accepts `/niihan/cmd_vel/manual`, `/niihan/cmd_vel/recovery` and `/niihan/cmd_vel/nav`, prioritizes fresh commands and publishes `/niihan/cmd_vel`. The controller publishes limited `/niihan/gazebo_cmd_vel` for Gazebo. Neither is a certified hardware safety controller.

Read the [workspace README](../../README.md) and [deployment checks](../../docs/DEPLOYMENT.md) before deployment.
