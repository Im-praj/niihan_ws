# Repeatable evaluation

## Fixed baseline

Use the same source revisions, built overlay, ROS/Gazebo versions, host, graphics backend, world file, seed, spawn, sensor rates and command sequence. Keep other simulators and robot stacks stopped. The construction-site baseline requires the supplied local proprietary meshes and contains only static environment models and no external actor controller.

Before each run, archive the exact command, `git status`, first-party source diffs, upstream checkout revisions, world SHA-256 and dependency versions. A fixed random seed is necessary but insufficient for bit-identical asynchronous SLAM outputs.

## Three-run protocol

1. Create a fresh results directory and set `ROS_HOME` and `ROS_LOG_DIR` to that run's directories.
2. Launch `niihan_full_system.launch.py headless:=true seed:=42 rqt_cam:=false` with the same installed overlay.
3. Wait for `/clock`, sensor streams, `map -> odom -> base_footprint` TF and active Nav2 lifecycle nodes. Require a stable initial pose/map before sending commands.
4. Record `/clock`, `/scan`, `/odom`, `/tf`, `/tf_static`, `/map`, `/niihan/cmd_vel`, `/niihan/e_stop`, `/cliff_scan` and the mast point-cloud topic with `ros2 bag record`.
5. Use the same inspected route and initial state. Record goal acknowledgement, terminal result, final position error, elapsed simulation time, stop events and minimum obstacle clearance.
6. Save the 2D map with `ros2 run nav2_map_server map_saver_cli -f <run>/map` and stop gracefully to save the 3D map.
7. Repeat from a fresh simulator process and fresh run directory three times.

Define acceptance tolerances before running: completion count, final XYZ position error (yaw is unrestricted), route deviation, latency and obstacle clearance. Compare aligned maps and trajectories numerically. File hashes can establish input equality; they cannot establish behavioural equality when timestamps or map serialization differ.

## Implemented checks

The validation script checks static-world geometry, locally resolvable legacy mesh paths, physics step and plugin presence. A test evaluates the seed and headless launch arguments. The validation report distinguishes standalone world loads from complete robot missions. Repeated live mission comparison remains an acceptance task.
