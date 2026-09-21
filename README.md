<div align="center">
  <h1>NIIHAN Ground Control Station</h1>
  <p><strong>Integrated ROS 2 Hardware Simulation and Web Dashboard</strong></p>
</div>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-FF6600?style=flat&logo=gazebo" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=flat&logo=python" alt="Python 3.10">
  <img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg" alt="License">
</p>

---

## Overview
**NIIHAN** is a 4-wheel motorized differential/skid-steer ground robot engineered for robust autonomous patrol operations. This repository contains the ROS 2 packages that power both the physical robot logic (simulated in Gazebo Harmonic) and its **fully integrated, local web-based Ground Control Station (GCS)**.

The architecture strictly enforces clean ROS 2 interfaces, allowing Nav2 autonomous stacks to run efficiently while safely permitting manual operator override directly from a browser-based frontend.

## Key Features
- **Gazebo Harmonic Simulation**: A high-fidelity, computationally lightweight 3D environment including custom patrol base `.world` files.
- **4WD Skid-Steer Drive**: All four wheels are motorized and controlled via independent velocity signals calculated from `/cmd_vel` inputs, using the standard differential/skid-steer configuration.
- **Live Web Dashboard (GCS)**: A browser-based React-like UI communicating over WebSockets at 10Hz, completely decoupled from ROS 2 blocking loops.
- **Interactive Navigation Map**: Renders full 2D `OccupancyGrid` map data from SLAM algorithms in the browser. Supports click-to-go capabilities utilizing the Nav2 Action Client (`NavigateToPose` / `FollowWaypoints`).
- **Virtual Joystick with Deadman Switch**: Real-time velocity (`cmd_vel`) commands from the browser with strict timeout policies. If the browser disconnects, the robot halts in `0.3s`.
- **Live Camera Feed**: Dynamic JPEG compression for live streaming of the panoramic cameras directly to the browser without bogging down CPU bandwidth.
- **Hardware E-STOP**: Browser E-STOP sends immediate override to the ROS 2 bridge, canceling Nav2 path planners and zeroing differential drive commands simultaneously.

## Architecture

The repository consists of two primary packages:
1. `niihan_description`: Core physical robot definition. Includes Xacro/URDF, 4-wheel differential drive controllers, sensors (3D LiDAR, RealSense, IMU, GNSS, Thermal), SLAM configurations, and Gazebo Worlds.
2. `niihan_dashboard`: Lightweight Python ROS 2 Node that wraps the `cmd_vel`, `odom`, `/map` data array, and camera topics into an asyncio WebSocket server, serving a highly responsive HTML/CSS/JS frontend via a local HTTP server.


## Installation & Setup

### Prerequisites
* **OS**: Ubuntu 22.04 LTS
* **ROS 2**: Humble Hawksbill
* **Gazebo**: Harmonic
* **Python packages**: `websockets`, `aiohttp`, `cv_bridge`

### Build Instructions
Clone this repository into your ROS 2 workspace (e.g., `~/niihan_ws/src`), then compile:

```bash
cd ~/niihan_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Stack

We have consolidated the entire robotics stack into a single unified launch file.

### 1. Launch the Full System
This will simultaneously launch Gazebo Harmonic, RViz, SLAM Toolbox, Nav2, spawn the 4WD NIIHAN robot, and start the Web Dashboard backend.
```bash
source install/setup.bash
ros2 launch niihan_description niihan_full_system.launch.py
```

### 2. Access the Dashboard
Open your preferred web browser and navigate to:
[**http://localhost:8080**](http://localhost:8080)

## Operator Manual
1. **Connectivity**: Ensure the top-right indicator shows a green **CONNECTED** status.
2. **MANUAL Mode**: Ensure the UI is set to `MANUAL`. Click and drag the yellow joystick knob to drive the robot. The differential drive will actively constrain max velocities.
3. **AUTO Mode**: Switch to `AUTO`. The joystick will be disabled. Click **Set Map Goal**, then click and drag a heading on the map grid. The Nav2 stack will automatically calculate a path and drive.
4. **E-STOP**: The large red Emergency Stop button ignores current modes. When activated, all active Nav2 tasks are cancelled and zero-velocity vectors are repeatedly published. You must manually clear the E-STOP to resume operation.

---
*Maintained by the NIIHAN Robotics Team. Designed for safety, determinism, and extensibility.*
