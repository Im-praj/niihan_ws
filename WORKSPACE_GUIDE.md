# Workspace Guide

Start with [README](README.md) for build and launch commands.

## Source and generated files

Edit code under `src/niihan_description` and `src/niihan_dashboard`. Never patch installed Python modules or generated launch files: the next build overwrites them. `build/`, `install/`, and `log/` are local generated outputs and are excluded from Git.

The upstream checkout directories contain their own repositories. Record their revisions and local changes before upgrades. They were preserved during cleanup, along with recordings and existing build outputs.

## Launch entrypoints

| Entrypoint | Behaviour |
| --- | --- |
| `niihan_full_system.launch.py` | Simulation, both maps, Nav2, dashboard; patrol off |
| `niihan_gazebo.launch.py` | Canonical simulator, configurable mapping/navigation/sensors |
| `gazebo.launch.py` | Compatibility alias for the canonical simulator |
| `mapping.launch.py` | Mapping and optional exploration; `autonav` defaults false |
| `niihan_mapping.launch.py` | Mapping and optional waypoint patrol; `autonav` defaults false |
| `niihan_navigation.launch.py` | Navigation only; needs TF, odometry, map and sensors |

All participating nodes must share `use_sim_time:=true` in simulation. A hardware launch must use real time and physical sensor/motor drivers; the full-system launch starts Gazebo and is not a hardware bringup.

## Changing the world

Place worlds in `src/niihan_description/worlds` and models in `src/niihan_description/models`. `setup.py` installs these directories recursively. Rebuild after adding assets, source the overlay and pass `world:=filename.sdf`. Keep mesh references relative to the installed world/model hierarchy or use the configured Gazebo resource paths.

The default spawn is `(-12, 0, 0.15)` with yaw 0. Always inspect terrain and robot clearance when choosing a different spawn. Existing patrol presets were not certified for the construction world; inspect targets against the current map before enabling autonomous motion.

## Outputs and troubleshooting

The 3D mapper writes `3d_map.pcd` under `${ROS_HOME:-~/.ros}/niihan_maps`. Use a separate `ROS_HOME` for each test run to avoid overwriting outputs. Save 2D maps with `nav2_map_server` after mapping.

If a node is missing, confirm `ros2 pkg prefix niihan_description` resolves to this workspace. If time or TF is missing, inspect `/clock`, `/odom`, `/tf`, `/tf_static` and `/scan` before enabling navigation. GPU/EGL availability affects simulated camera and LiDAR rendering.

Historical one-shot root patch/debug scripts and generated TF snapshots were removed after a backup; the cleanup manifest records the exact files. Runtime source changes and meaningful package regression tests were retained.
