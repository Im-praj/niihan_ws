# NIIHAN rover: 3D SLAM and Nav2

ROS 2 Humble / Gazebo Harmonic simulation with GLIM CPU LiDAR-inertial SLAM, a projected occupancy grid, Nav2 waypoint missions and a local browser dashboard. The portable simulation uses the Unitree L2 scan geometry and chassis IMU. It is a simulation model, not a validated physical L2 driver/calibration.

## Supported environment

Ubuntu 22.04 amd64, ROS 2 Humble, Gazebo Sim 8 (Harmonic), Python 3.10, a working graphical display and OpenGL rendering. Windows/macOS/native ROS distributions are not validated. A fresh source clone can be tested on this host; that does not certify every GPU or clean operating-system installation.

Install ROS Humble and Gazebo Harmonic from their official instructions. Use the Harmonic-compatible `ros-humble-ros-gzharmonic` bridge, not a mismatched default Gazebo bridge. Install Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`), Xacro, robot state publisher, sensor_msgs_py, cv_bridge, image_transport, rosbag2 and colcon/rosdep. Python packages: numpy, scipy, PyYAML, aiohttp, websockets, pytest, pyserial; Node.js and GCC are used by validation.

GLIM source prerequisites: GTSAM 4.3 (upstream 4.3a0), gtsam_points 1.2.2 or a compatible version, Eigen, Boost, OpenMP, fmt, spdlog and OpenCV. See [official GLIM installation](https://koide3.github.io/glim/installation.html). Choose CPU dependencies/builds. CUDA and Iridescence are not required for this profile; the browser remains graphical with `BUILD_WITH_VIEWER=OFF`. This flag disables GLIM's separate desktop editor, not Gazebo or the dashboard. Record dependency versions before comparison runs.

## Clone and build

```bash
git clone --recurse-submodules https://github.com/Im-praj/niihan_ws.git
cd niihan_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/niihan_description src/niihan_dashboard src/niihan_bringup src/niihan_slam src/glim src/glim_ros2 --ignore-src -r -y
./scripts/build_simulation.sh
source install/setup.bash
./scripts/validate.sh
```

Submodule commits are pinned by Git and `.gitmodules` supplies public URLs. Nav2 and KISS-ICP source checkouts are optional: the supported build uses installed Humble Nav2. Existing local upstream modifications are preserved as `patches/*.patch`; they are not needed by the CPU build with GLIM's editor disabled. Do not build all optional packages indiscriminately.

## Visible simulation

```bash
source install/setup.bash
ros2 launch niihan_slam slam_simulation.launch.py \
  headless:=false seed:=42 output_directory:=$HOME/evidence/current-map
```

Open [dashboard](http://127.0.0.1:8080). HTTP and WebSocket bind to localhost. Three.js and OrbitControls are vendored, including their license. Both 3D cloud view and 2D navigation map are available; the front camera remains enabled in the minimal sensor profile. If WebGL is unavailable, the dashboard exposes the 2D fallback.

`seed:=42` fixes Gazebo's random seed for comparable tests; 42 has no special SLAM meaning. CPU scheduling, sensor delivery and numerical optimization still vary. Repeatability means passing measured tolerances over repeated runs, not byte-identical maps.

The default portable world uses distributable primitive geometry. The older construction world references proprietary GLB assets omitted from Git and can fail on a clone. Use the portable world for release checks. All launches default to visible simulation in the new profile.

## Data flow

L2-model 3D cloud + IMU → GLIM 3D odometry/local/global mapping → estimated full 3D `map → odom → base_footprint`, plus planar `odom → base_nav` for Nav2 → occupancy projection + gravity-filtered scan → Nav2 → command arbiter → health/e-stop/watchdog gateway → Gazebo drive. Wheel odometry is used for drive health only. `/niihan/ground_truth` is an isolated evaluation topic and is never an estimator input or navigation TF publisher.

Vortex is a cloud serialization helper here; it is not the SLAM estimator. Nav2 plans on a 2D grid derived from 3D geometry. This flat-site profile is not a slope, overhang or drop-off traversability system. The separate hardware stack is documented in [commissioning](src/niihan_bringup/COMMISSIONING.md).

## Save once and reuse

```bash
ros2 service call /niihan/slam/save_map std_srvs/srv/Trigger '{}'
```

This writes `map_3d.npz`, `map.pgm`, `map.yaml` to `output_directory`. Stop the mapping launch before starting localization:

```bash
ros2 launch niihan_slam slam_simulation.launch.py \
  mode:=localization map_file:=$HOME/evidence/current-map/map_3d.npz \
  headless:=false seed:=42 initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

Saved-map localization uses initial-pose-assisted planar registration of 3D points with overlap/RMSE gates. It is experimental and is not global place recognition. Weak fits stop fresh map TF updates; navigation must remain inhibited. Initial pose is in map coordinates; yaw is radians. A new mapping run need not overwrite the previous map. Save a new map only when deliberately remapping or revising it.

## Acceptance and evidence

See [release validation](docs/RELEASE_VALIDATION.md). Run the same three-waypoint route after a complete simulation restart at least three times; retain failures, logs, world-pose trajectories, timing, map files, visible screenshots and video. Record source commit, dependency versions and seed. Do not declare an all-milestone pass based only on node startup or unit tests. Hardware, real L2 timestamps/IMU extrinsics, RTK and physical emergency-stop tests require the actual rover.
