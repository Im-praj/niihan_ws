# NIIHAN rover: 3D SLAM and Nav2

ROS 2 Humble / Gazebo Harmonic simulation with GLIM CPU LiDAR-inertial SLAM, a projected occupancy grid, Nav2 waypoint missions and a local browser dashboard. The portable simulation uses a simplified L2-like GPU LiDAR (360 × 32 rays at 10 Hz) and chassis IMU. It is a simulation model, not a validated physical L2 driver/calibration.

## Source structure

| Path | Role |
|---|---|
| `src/niihan_description` | Robot, portable world, Gazebo bridges, arbitration |
| `src/niihan_bringup` | Sensor contracts, drive/sensor health, e-stop/watchdog, hardware hooks |
| `src/niihan_slam` | GLIM CPU configuration, map projection/save, planar TF, experimental map localization, RViz |
| `src/niihan_dashboard` | Local browser GUI, live camera, 3D/2D maps and missions |
| `src/glim`, `src/glim_ros2`, `src/navigation2`, `src/geometry2` | Pinned upstream source submodules |
| `scripts`, `patches`, `docs` | Build/validation/evidence tooling, compatibility fixes and operating instructions |

## Supported environment

Native targets (amd64):

| Ubuntu | ROS | Gazebo | Verification |
|---|---|---|---|
| 22.04 Jammy | Humble | Harmonic, non-default bridge | Local build and repeated simulation tested |
| 24.04 Noble | Jazzy | Harmonic, native `ros-jazzy-ros-gz` | Build/GUI/route acceptance pending |

Setup selects the native pairing automatically. This does not support Humble on 24.04 or Jazzy on 22.04 natively; use a matching OS container/VM for those combinations. A working graphical display and OpenGL rendering are required. Windows/macOS/native ROS distributions are not validated. A fresh source clone can be tested on this host; that does not certify every GPU or clean operating-system installation.

The setup script installs the native ROS and Gazebo prerequisites. For a manual Humble development environment, install ROS Humble and Gazebo Harmonic from their official instructions. Use the Harmonic-compatible `ros-humble-ros-gzharmonic` bridge, not a mismatched default Gazebo bridge. Install Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`), Xacro, robot state publisher, sensor_msgs_py, cv_bridge, image_transport, rosbag2 and colcon/rosdep. Python packages: numpy, scipy, PyYAML, aiohttp, websockets, pytest, pyserial; Node.js and GCC are used by validation.

GLIM source prerequisites: GTSAM 4.3a0 (`3ad4b4c3cb28394c9597f48fa02dad361c8450e3`), gtsam_points v1.2.2 (`9d32e7dbecf6015560d84b4901d6b0a6f483ec46`), Eigen, Boost, OpenMP, fmt, spdlog and OpenCV. See [official GLIM installation](https://koide3.github.io/glim/installation.html). Choose CPU dependencies/builds. CUDA and Iridescence are not required for this profile; the browser remains graphical with `BUILD_WITH_VIEWER=OFF`. This flag disables GLIM's separate desktop editor, not Gazebo or the dashboard. Record dependency versions before comparison runs.

## Quick setup and visible launch

On Ubuntu 22.04 or 24.04 amd64 with a graphical desktop:

```bash
git clone --branch feat/l2-3d-slam-release https://github.com/Im-praj/niihan_ws.git
cd niihan_ws
./setup.sh
./run.sh
```

If already cloned, use `git pull --ff-only origin feat/l2-3d-slam-release` instead of cloning again. Default setup downloads the revision-matched CPU binary archive from GitHub Releases after installing runtime prerequisites. `./setup.sh --source` explicitly compiles instead. Setup uses sudo for apt packages and official ROS/Gazebo repository configuration when needed. The source fallback compiles pinned CPU GTSAM/gtsam_points into `.deps/install` (Humble) or `.deps/jazzy/install` (Jazzy), builds the selected package set and runs validation. Compilation can take considerable time; leave the terminal open. It does not install graphics drivers. Logs and failure details are saved in `.setup/logs`. Setup stops on failure; do not run `run.sh` until it succeeds.

`./setup.sh --check` checks prerequisites/package lookup without installing. `--no-system` skips apt for a prepared host. `--use-system-deps` reuses existing compatible GTSAM/gtsam_points for an already commissioned development machine. `--clean` preserves build/install/log in `.setup/backups` before rebuilding; use this for incomplete old overlays. The scripts start with a clean ROS environment and isolate pytest from unrelated automatic plugin loading. They never delete source or modify system Python with pip.

`run.sh` starts visible Gazebo and RViz, then opens the local dashboard when its HTTP server responds. It rejects headless overrides and occupied dashboard ports. Optional launch arguments, such as `seed:=42`, can follow `./run.sh`. This convenience path does not change the incomplete release acceptance status described below. Package installation on a fresh OS and GPU rendering on another laptop remain unverified.

## Clone and build

```bash
git clone --recurse-submodules --branch feat/l2-3d-slam-release https://github.com/Im-praj/niihan_ws.git
cd niihan_ws
./setup.sh --source
./run.sh
```

Submodule commits are pinned by Git and `.gitmodules` supplies public URLs. The Humble build first compiles pinned geometry2 0.25.24 (TF core and ROS bindings), then the pinned Nav2 checkout with `patches/nav2-humble.patch`, including rejected-transform handling. Installed Nav2 supplies prerequisites; runtime uses the built overlay. KISS-ICP remains optional. GLIM editor patches are preserved but not needed with its editor disabled. Do not build all optional packages indiscriminately.

## Visible simulation

```bash
source install/setup.bash
ros2 launch niihan_slam slam_simulation.launch.py \
  headless:=false seed:=42 output_directory:=$HOME/evidence/current-map
```

Open [dashboard](http://127.0.0.1:8080). HTTP and WebSocket bind to localhost. Three.js and OrbitControls are vendored, including their license. RViz opens with robot/TF, 3D map, projected scan, occupancy and Nav2 path displays (`rviz:=false` can close that extra viewer). Both dashboard 3D cloud view and 2D navigation map are available; the front camera remains enabled in the minimal sensor profile. If WebGL is unavailable, the dashboard exposes the 2D fallback.

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

Goal acceptance verifies XYZ distance within 0.25 m with fresh localization; final yaw is unrestricted. Heading still steers the rover and orients its visualization. Map clicks use the current map-frame rover Z; this can be negative because the LIO map origin begins near the IMU. Nav2 plans XY ground travel and cannot climb to arbitrary Z targets. The dashboard renders the live robot_description URDF, including chassis, suspension, wheels, mast and L2. Both simulation sensor profiles contain exactly one forward RGB camera; obsolete PTZ, side/rear, thermal, depth and fiducial cameras are removed.

TF release pin: geometry2 0.25.24, commit 404b7224d623d614f18fa9738dbf1716403d857e. The installed mixed TF versions (core 0.25.22 / ROS 0.25.23) exhibited a lock-order deadlock between MessageFilter requests and TF insertion. The pinned official core invokes callbacks outside the request mutex. Thread backtraces and simulation acceptance recordings document this issue and its verification.

Mission controls: PAUSE waits for Nav2 cancellation acknowledgement and preserves the current waypoint; RESUME dispatches that waypoint again only after fresh pose, health and AUTO checks. CANCEL ends a paused or running mission. An E-stop or localization fault cancels rather than resumes the mission.

The 3D profile starts at world (-10,0), away from the construction world's raised access-lane edge at x=-12. The fixed map-frame acceptance loop stays on flat ground. The earlier curb attempt is retained as a failed terrain case: wheel rotation is not proof of vehicle motion. Step climbing and traversability require separate terrain detection/control validation. `spawn_x` and `spawn_y` can be overridden explicitly.

The 3D navigation behavior tree checks position arrival on every tick before replanning, with a 0.15 m XY tolerance. This prevents repeated FollowPath successes from being superseded by asynchronous replans in saved-map mode. The dashboard still verifies fresh full XYZ distance <=0.25 m before advancing the mission. No yaw completion condition is used.


## Cloud and vehicle pose contract

Every rendered `/niihan/slam/map_cloud` packet carries `frame_id`, `stamp`, `pose_stamp` and the vehicle pose (`x`, `y`, `z`, `yaw`, `roll`, `pitch`). The dashboard resolves `map → base_footprint` at the cloud timestamp, not the latest TF. A cloud without a matching transform is withheld. The 3D label displays that cloud's timestamp and X/Y/θ; the telemetry panel separately shows the current live pose. Angles are radians in messages and degrees in the GUI. The actual URDF model follows full roll/pitch/yaw and live wheel/suspension joints.

The cloud is an accumulated map snapshot, normally published at 1 Hz and reduced to the browser's point budget. This is not a promise to send every raw L2 scan to a browser. Raw 3D scans and IMU feed GLIM independently of browser updates. Saved-map snapshots use the last accepted registration timestamp. A single forward RGB camera is bridged directly in simulation; the common adapter does not relay it back into its input.

With `headless:=false`, the launch starts the world server and a separate visible Gazebo GUI client. RViz and the browser remain graphical. Separating server/client startup addresses intermittent combined-process starts that supplied no world service or clock.

## Recorded acceptance commands

Run these from a freshly sourced workspace on a graphical desktop. Each run directory must be new; failed runs are preserved. The harness records 1920×1080 desktop video on this host, source/submodule state, sensor/TF/lifecycle ownership, actual drive commands, trajectory pairs, cloud/pose timestamp checks and map files. GStreamer `gst-launch-1.0`, `ximagesrc`, VP8/WebM plugins and a valid `DISPLAY` are required for recording. Desktop video includes whichever windows are visible; separate approved Gazebo/RViz close-ups are still a checklist item.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
mkdir -p "$HOME/evidence"
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/mapping_01"
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/mapping_02"
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/mapping_03"
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/localization_01" \
  mode:=localization map_file:="$HOME/evidence/mapping_01/map/map_3d.npz"
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/safety_01" --probe=fault
python3 scripts/record_acceptance.py "$PWD" "$HOME/evidence/obstacle_01" --probe=obstacle
```

The acceptance trajectory uses Gazebo world pose only for evaluation. XYZ displacement and θ are compared at matching timestamps with initial coordinate offsets removed. No fitted trajectory rotation is used for position scoring. Limits are maximum relative XYZ error 0.20 m, maximum relative heading error 0.15 rad, and final XYZ goal error 0.25 m. Heading accuracy is checked; final waypoint heading is unrestricted. Runtime checks also require one camera publisher, one clock publisher, unique TF parents, no duplicate ROS nodes and active required Nav2 components. See [measured results and remaining limitations](docs/RELEASE_VALIDATION.md).

## Physical L2 commissioning boundary

The verified GLIM profile is `niihan_slam/slam_simulation.launch.py`. The older `niihan_bringup/niihan_hardware.launch.py` starts the legacy EKF/slam_toolbox application; it is not the GLIM 3D hardware profile. Physical L2 operation is not certified by these simulation runs. Real point timestamps/deskew, LiDAR-to-IMU extrinsics, IMU source/bias/gravity, serial/network driver settings and motor calibration must be measured on the actual rover before a hardware GLIM profile is accepted. The supplied `global_shutter=true` and chassis-IMU configuration describe the simulation sensor, not a physical L2 calibration. Ground-height projection is a flat-site baseline and does not certify slopes or drop-offs.

## Validation: incompatible user pytest plugins

If validation reports `ModuleNotFoundError: No module named '_pytest.scope'` from `anyio/pytest_plugin.py`, a user-installed AnyIO plugin is incompatible with the system pytest. This happens before project tests run; it is not a GLIM compilation error. The validation script disables automatic loading of unrelated third-party pytest plugins; this suite needs only built-in plugins. Update the release branch and rerun:

```bash
git pull --ff-only origin feat/l2-3d-slam-release
./scripts/validate.sh
```

For an older checkout, use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/validate.sh`. No package uninstall or system-wide Python upgrade is required.

If rosdep reports `Cannot locate rosdep definition for [ament_python]`, update the release branch. The manifests use `python3-setuptools` as the build dependency and retain `ament_python` only as the exported build type. Setup resolves the supported package dependency closure, excluding upstream system tests requiring Gazebo Classic. Skipping distributions other than Humble during `rosdep update --rosdistro humble` is normal.

A `nav2_route` compilation error at `knnSearch` involving `AccessorType*` is a nanoflann index-type mismatch between library versions. The compatibility patch explicitly selects `size_t` for both the KD-tree and result buffer. Update the release branch and rerun setup; the patch helper upgrades the previous patch per file without discarding local edits.

## Jazzy profile and binary distribution

Ubuntu 24.04 uses native ROS Jazzy Nav2 and geometry2 apt packages. It does not compile or patch the Humble copies in `src/navigation2`/`src/geometry2`. The GLIM and application builds use the same pinned GLIM source, with a separate dependency prefix; compiled Humble libraries are never reused as Jazzy binaries. The SLAM launch selects `nav2_3d_jazzy.yaml` and a BehaviorTree.CPP 4 XML. Jazzy plugin names, progress checkers and behavior costmap names are explicit; command outputs remain unstamped Twist for the existing arbiter.

Do not migrate an existing install directory between ROS versions. Setup records `.setup/build-profile` and rejects a mismatched profile; use a separate clone or `--clean` to preserve the old build. Apt package versions are recorded in `.setup/system-package-versions.txt`. Native Jazzy Nav2/TF packages are distribution binaries, not the custom Humble build. The CI release workflow builds platform-specific CPU archives with GLIM, its private shared libraries, the application and (on Humble) patched Nav2/TF. Archives are published only after second-directory install/import/link checks pass. Missing release assets fail clearly; they do not silently trigger compilation. The CI matrix builds both native targets, checks archive installation and uploads logs; build success alone does not replace visible simulation evidence. Jazzy has not been validated on this local Humble host and must not yet be described as release-certified.

## Binary-first end-user installation

`./setup.sh` installs apt prerequisites and downloads `niihan-ubuntu22.04-humble-amd64.tar.gz` or `niihan-ubuntu24.04-jazzy-amd64.tar.gz` from the release tagged `binaries-<full source commit>`. It verifies SHA-256 plus source commit, OS, ROS, architecture and Python metadata before installing. The old install is preserved under `.setup/backups`; runtime shared libraries are isolated under `.binary/deps/lib`. `./run.sh` remains the visible launch entry point. No C++ compilation occurs on this default path.

For developers and CI, use `./setup.sh --source`; optional `--no-system --use-system-deps` applies to a prepared development machine. If this commit's binary release is still building or failed, setup reports that instead of pretending a binary is available. See GitHub Actions and Releases. A successful package installation check does not certify simulation, physical L2 operation or every graphics driver. SHA-256 detects corruption; trust still comes from this repository's release publication.
