# niihan_description

Robot model, Gazebo construction-site simulation, SLAM/navigation configuration and motion utilities for NIIHAN.

## Entry points

Use `ros2 launch niihan_description niihan_gazebo.launch.py` for configurable simulation and `niihan_full_system.launch.py` to include the dashboard. `world`, `headless`, `seed`, `mapping`, `launch_nav2`, `vortex_3d`, `cliff_detect`, `patrol` and `rqt_cam` control simulation features. Spawn coordinates are configurable in the canonical launcher. Patrol and the camera viewer default off.

## Data

- `urdf/`: Xacro robot structure and simulated sensors.
- `worlds/niihan_construction_site.sdf`: static mesh-based construction scene.
- `models/construction_site_chunk5/`: supplied collision meshes with their original proprietary license; publication authorized by the repository owner. See ../../docs/DEPENDENCIES.md.
- `config/`: Nav2, SLAM and existing patrol presets. Preset route safety requires inspection against the selected map.
- `test/`: kinematics, point-cloud, TF, launch clock, navigation and world contracts.

The arbiter accepts `/niihan/cmd_vel/manual`, `/niihan/cmd_vel/recovery` and `/niihan/cmd_vel/nav`, prioritizes fresh commands and publishes `/niihan/cmd_vel`. The controller publishes limited `/niihan/gazebo_cmd_vel` for Gazebo. Neither is a certified hardware safety controller.

Read the [workspace README](../../README.md) and [deployment checks](../../docs/DEPLOYMENT.md) before deployment.


### Industry seaport GLB world

The imported `worlds/niihan_industry_seaport.sdf` uses `worlds/industry_seaport_optimized.glb` for its static visuals and mesh collision. The wrapper rotates the source Y-up scene to Gazebo Z-up and offsets the source ground plane to z=0. Build and source the workspace, then launch with `world:=niihan_industry_seaport.sdf`, for example:

```bash
ros2 launch niihan_description niihan_gazebo.launch.py world:=niihan_industry_seaport.sdf
```

The existing default world remains unchanged. The imported environment has not yet been visually inspected in Gazebo or checked for robot spawn clearance and navigation performance.
