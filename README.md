# NIIHAN — Construction Site Rover

ROS 2 software for simulating and evaluating a surveillance rover in a construction-site environment. The workspace combines Gazebo Harmonic, 2D SLAM, point-cloud mapping, Nav2 and a browser dashboard.

**Status:** simulation development. Physical deployment and repeatable autonomous missions require the acceptance checks in [Deployment](docs/DEPLOYMENT.md). No hardware readiness claim is made.

## Workspace

| Component | Purpose |
| --- | --- |
| `src/niihan_description` | Robot Xacro, simulation worlds, sensor bridges, mapping and motion nodes |
| `src/niihan_dashboard` | Map, telemetry, missions, geofencing and browser controls |
| `src/navigation2` | Existing upstream Nav2 checkout; separate repository |
| `src/glim`, `src/glim_ros2`, `src/kiss-icp` | Existing optional upstream mapping checkouts |

Third-party checkout modifications are preserved. The first-party packages can be built independently; upstream repository revision and dependency locking remain a release task.

## Requirements and build

Validation environment: ROS 2 Humble, Python 3.10 and Gazebo Sim 8 (Harmonic). This workspace requires the **Harmonic-compatible** `ros_gz_sim` and `ros_gz_bridge`; the default Humble/Gazebo combination must not be assumed compatible.

Source ROS and resolve dependencies before building. `rosdep install` may need system package privileges.

```bash
cd ~/niihan_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/niihan_description src/niihan_dashboard --ignore-src -r -y
colcon build --symlink-install --packages-select niihan_description niihan_dashboard
source install/setup.bash
```

The build command uses installed Nav2 and SLAM dependencies. It does not rebuild every optional checkout. Generated `build/`, `install/` and `log/` directories are ignored by Git.

## Run

Simulation, 2D/3D mapping, navigation and dashboard:

```bash
ros2 launch niihan_description niihan_full_system.launch.py
```

Open [the local dashboard](http://127.0.0.1:8080). Its HTTP and WebSocket servers bind to localhost by default. The 3D viewer currently fetches Three.js from external CDNs; vendor those dependencies before an offline field release.

Headless simulation:

```bash
ros2 launch niihan_description niihan_gazebo.launch.py \
  headless:=true rqt_cam:=false seed:=42
```

`headless:=true` enables server-only operation and EGL rendering for camera/LiDAR sensors. Mapping-only evaluation with manual motion:

```bash
ros2 launch niihan_description mapping.launch.py autonav:=false rviz:=true
```

The default world is `niihan_construction_site.sdf`. Existing worlds remain selectable with `world:=niihan_patrol_base.world`. `gazebo.launch.py` is a compatibility entrypoint for the canonical simulator launcher.

## Construction-site scene

The default scene renders the supplied chunk5 ground and site geometry as both visible meshes and collisions. A distinct surrounding layout adds a tower crane, unfinished steel building, perimeter fencing, site office, storage container and material stacks. The western access lane contains the spawn at (-12, 0, 0.15).

The supplied meshes are simplified collision geometry with added materials; the original textured visual model was not supplied. The scene is distinct from `niihan_patrol_base.world`, which remains available as a primitive-only fallback.

The default world requires local chunk5 meshes. Their LICENSE declares proprietary ownership; the directory is ignored by Git. A public clone must supply authorized assets or use `world:=niihan_patrol_base.world`. Confirm redistribution rights before publication.

Use the same `seed:=42`, spawn and software versions for repeated evaluation. Identical maps and trajectories are not guaranteed; follow [Repeatability](docs/REPEATABILITY.md).

## Motion and safety

Manual, recovery and navigation commands enter `command_arbiter`, which publishes `/niihan/cmd_vel`. The drive controller limits wheel speeds and publishes the simulator command. The arbiter uses a steady-clock watchdog and clears stale commands on emergency-stop transitions. Automatic patrol is disabled by default.

Cliff scans and geofences are software aids that require environment-specific validation. Physical emergency stop, motor watchdog and braking must work independently of ROS, the browser and host computer.

## Verification

```bash
./scripts/validate.sh
```

The script checks both first-party Python test suites, browser map regressions, Xacro expansion and SDF validity. It expects ROS dependencies to be installed. Build and live mission acceptance are separate steps; see [Validation](docs/VALIDATION.md) for results and limitations.

## Documentation

- [Workspace guide](WORKSPACE_GUIDE.md)
- [Simulation package](src/niihan_description/README.md)
- [Dashboard package](src/niihan_dashboard/README.md)
- [Navigation pipeline](src/niihan_description/docs/navigation_pipeline.md)
- [Deployment checks](docs/DEPLOYMENT.md)
- [Repeatability procedure](docs/REPEATABILITY.md)

## Licensing

First-party package manifests declare Apache-2.0. Upstream repositories retain their respective licenses. Imported proprietary meshes are not covered by that declaration. Maintainer placeholders and an authoritative repository-wide license must be resolved before a public release.
