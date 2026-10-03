# NIIHAN mapping and navigation pipeline

The supplied `niihan_full_system.launch.py`, `mapping.launch.py` and
`niihan_mapping.launch.py` run this pipeline:

- Gazebo differential-drive odometry owns `odom -> base_footprint` and `/odom`.
- `robot_state_publisher` owns the fixed robot/sensor transforms.
- One `slam_toolbox` owns `map -> odom` and publishes the 2D `/map` OccupancyGrid.
- One `vortex_3d_mapper` accumulates stamped mast clouds in `map` for the 3D display.
- Nav2 consumes the 2D map, live laser and 3D obstacle data, and the dashboard
  geofence mask. Both costmaps apply the same keepout filter.
- Controller output `/cmd_vel_nav` passes through the velocity smoother to
  `/niihan/cmd_vel/nav`; recovery commands also reach that arbiter input.

The full-system launch passes one `use_sim_time` argument to both bringup and
the dashboard. Simulation requires `true` and a running `/clock`. On hardware
all nodes must instead use wall time, and the hardware bringup must supply the
odometry and sensor topics. Setting `use_sim_time:=false` on a Gazebo launch
alone does not convert Gazebo sensor stamps to wall time.

The mapping wrappers disable the inner copies of their own SLAM/3D/cliff nodes.
This avoids duplicate cloud maps, cliff scans and unnecessary processing load.
SLAM's asynchronous scan queue is one scan, so unavailable transforms cannot
accumulate an ever-older sensor backlog. A brief cache warning for the very first
sensor frame during startup is different from persistent clock/TF errors.

## Planning and goal completion

The installed behavior trees replan at 2 Hz while following the active path and
retain Nav2's costmap clearing, spin, wait and backup recovery sequence. The
global costmap refreshes at 2 Hz. Smac 2D uses A* with obstacle cost penalties;
it seeks a short collision-free path on the current grid, with clearance costs.
This cannot guarantee the mathematical shortest path in an unknown/dynamic
world. A blocked/unreachable goal must fail instead of being reported reached.

The planner's endpoint tolerance is 0.1 m, and the controller continuously checks
position within 0.2 m and yaw within 0.25 rad. It no longer latches XY success
while a later map correction or rotation moves the robot outside tolerance.
The dashboard independently validates fresh position within 0.25 m and yaw
within 0.30 rad before completing each waypoint. The small margin over the
controller threshold allows TF latency and velocity-smoother settling. The through-poses tree only removes intermediate goals within 0.2 m.

The 3D lidar's modeled vertical field of view does not see the ground close to
the chassis. Cliff inference must exclude that blind region rather than mark
all missing floor returns as obstacles. These launch files pass the URDF's
actual vertical FOV and a 0.45 m robot exclusion radius to the detector.

## KISS-ICP and GLIM

Their source packages exist in this workspace, but the NIIHAN launch files do
not start either estimator. The README package list does not reflect an active
KISS/GLIM integration. Do not start their stock launch/configuration alongside
the pipeline above and expect them to fuse automatically:

- KISS-ICP defaults to `odom_lidar`, an empty base frame, and inverted odometry
  TF in its supplied launch. To use it as the odometry authority, configure
  `base_frame=base_footprint`, `lidar_odom_frame=odom`,
  `publish_odom_tf=true`, `invert_odom_tf=false`, and the NIIHAN cloud topic.
  Disable the Gazebo/hardware publisher of the same `odom -> base_footprint`
  transform and reconcile the `/odom` message source/velocity fields.
- GLIM's stock config subscribes to `/os_cloud_node/imu` and
  `/os_cloud_node/points`. Its RViz extension publishes both global/odometry TF
  and IMU-to-lidar TF. It needs the real NIIHAN topic names, calibrated
  extrinsics, base frame and matching ROS clock. If GLIM owns `map -> odom`,
  SLAM Toolbox cannot also publish it. GLIM's map is PointCloud2, not the
  OccupancyGrid needed by Nav2; a coherent 2D projection/localization design is
  required before switching estimators.

There must be one authority for each TF child frame, and every scan, cloud,
odometry and TF timestamp must share the same clock domain. Increasing TF
buffer length or stamping old sensor data with `now()` cannot repair that.

## Verification

Run package tests from a sourced ROS 2 Humble shell. Launch-contract tests check
both clock settings, absence of duplicate mapping nodes, geofence filters,
endpoint tolerances and replanning trees without sending motion commands.

For a controlled simulator run, use an isolated ROS domain and:

```bash
ros2 launch niihan_description niihan_full_system.launch.py headless:=true rqt_cam:=false
```

Wait for `/clock`, `/scan`, `/odom`, `/map` and `map -> base_footprint` before
sending a goal. Inspect `ros2 topic info /map --verbose` and TF frame authorities.
Verify a distant goal stays active until the robot enters its tolerance; verify
an obstacle causes replanning and an unreachable goal reports failure. Check a
saved geofence changes both costmaps and prevents routes outside its boundary.

The stock world uses GPU lidar. Headless rendering still needs EGL/OGRE support;
a simulator that publishes only `/clock` and odometry cannot validate mapping.

References: [Nav2 Humble launch source](https://api.nav2.org/nav2-humble/html/navigation__launch_8py_source.html),
[Nav2 transforms](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/),
[Smac 2D planner](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/).
