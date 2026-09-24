# NIIHAN ROS2 Workspace

## Overview
The `niihan_ws` is a comprehensive ROS2 workspace for the NIIHAN autonomous patrol robot. It provides simulation, mapping, navigation, and dashboard capabilities. The system features advanced 2D and 3D mapping, autonomous frontier exploration, waypoint patrolling, and a robust safety pipeline including synthetic cliff detection.

## Packages
* **`niihan_description`**: The core package containing the URDF, Gazebo Harmonic launch files, configurations (Nav2, SLAM Toolbox), and key autonomous control nodes.
* **`niihan_dashboard`**: Web-based UI and ROS bridge for monitoring robot status, 3D visualization, mission management, and geofencing.
* **`kiss-icp`**: Used for robust odometry and point cloud registration.
* **`glim_ros2`**: Additional mapping and localization utilities.

## Hardware & Sensor Suite
NIIHAN is equipped with a rich array of sensors, all bridged to ROS2 via Gazebo Harmonic (`ros_gz_bridge`):
* **LiDAR**: 2D LiDAR (`/scan`) and 3D Mast LiDAR (Unitree).
* **Cameras**: Panoramic RGB cameras (front, left, right, rear), PTZ camera, Thermal camera, and Dock fiducial camera.
* **Depth**: Orbbec and Realsense D435i depth cameras.
* **State**: IMU, GNSS, and contact bumpers for collision detection.

## Key Subsystems

### 1. Simulation & Bringup
The primary launch files are located in `niihan_description/launch/`:
* **`niihan_gazebo.launch.py`**: Spawns the robot in Gazebo Harmonic, loads the `niihan.urdf.xacro`, starts the `ros_gz_bridge` for all sensors, and initializes the differential drive controller.
* **`mapping.launch.py` / `niihan_mapping.launch.py`**: Master bringup for mapping. Launches simulation, 2D SLAM, 3D Vortex Mapper, Nav2, and RViz2.
* **`niihan_full_system.launch.py`**: Master launch file that brings up the entire stack, including the `niihan_dashboard`.

### 2. Mapping
The workspace employs a dual 2D/3D mapping strategy:
* **2D SLAM**: Handled by `slam_toolbox` (async online mode), publishing the standard `/map` occupancy grid.
* **3D Point Cloud Mapping (`vortex_3d_mapper.py`)**: A custom node replacing OctoMap. It subscribes to the 3D mast LiDAR, transforms points to the `map` frame (using quaternion math), and accumulates them into a voxel-filtered 3D point cloud map. It periodically saves the result as a `.pcd` file.

### 3. Navigation & Patrol
Uses the ROS2 **Nav2** stack for path planning, combined with custom high-level Python controllers:
* **`patrol_controller.py`**: A complex autonomous exploration and patrol node.
    * *Exploration Phase*: Analyzes the map boundaries and uses frontier exploration or bootstrap behaviors (driving toward the longest clear LiDAR ray) to fully observe the map.
    * *Patrol Phase*: Once the map is complete, it continuously cycles through predefined waypoints.
    * *Recovery*: Implements custom fallback behaviors (reversing and rotating towards clearance) to clear obstacles before relying on Nav2's recovery server.
* **`waypoint_patrol.py`**: A simpler waypoint controller that accepts YAML preset files for user-defined patrol routes.

### 4. Safety Systems
* **`cliff_detector.py`**: Crucial for safety in multi-level environments. It processes 3D LiDAR data to find areas lacking ground returns (voids/cliffs). These are published as virtual obstacles on `/cliff_scan`, allowing the Nav2 costmap to route the robot safely around drop-offs.
* **Dashboard Safety Managers**: `safety_manager.py` and `geofence_manager.py` (in `niihan_dashboard`) enforce operational boundaries and constraints from the UI backend.

## Getting Started

### Prerequisites
* ROS2
* Gazebo Harmonic
* Nav2, SLAM Toolbox

### Building the Workspace
```bash
cd ~/niihan_ws
colcon build --symlink-install
source install/setup.bash
```

### Launching the System
**Full System (Sim, Nav, Mapping, Dashboard):**
```bash
ros2 launch niihan_description niihan_full_system.launch.py
```

**Mapping & Exploration Mode:**
```bash
ros2 launch niihan_description mapping.launch.py rviz:=true autonav:=true
```

## Custom Node Executables Reference
* `patrol_controller`: Autonomous boundary exploration and waypoint patrol.
* `waypoint_patrol`: Preset-based waypoint navigation.
* `vortex_3d_mapper`: 3D voxel-based point cloud accumulator.
* `cliff_detector`: Synthetic cliff-scan generator for safety.
* `differential_drive_controller`: Custom kinematic controller for the NIIHAN chassis.
