# NIIHAN Workspace Guide

Welcome to the NIIHAN Workspace! This guide is written to help you understand how this workspace is organized, what everything does, and how to use it—even if you are new to ROS 2. 

This workspace is set up for a simulated robot using **ROS 2 Humble** and **Gazebo Harmonic**. It allows the robot to map its environment, navigate around obstacles, and perform autonomous patrols.

---

## 📂 What is Inside the Workspace?

Here is a simple breakdown of the main folders in this workspace and what each one does:

*   src/ (Source Code): This is the most important folder! It contains all the code, settings, and launch instructions that make the robot work. When you want to change how the robot behaves, you will edit files in here. The main package inside is called `niihan_description`.
*   build/: Think of this as a temporary construction site. When you compile (build) your code, intermediate files are created here. You don't need to touch this folder, and you can safely delete it if you want to rebuild everything from scratch.
*   install/: This is the finished product folder. After building, the ready-to-run programs are placed here. Before you can run the robot, you must tell your computer to look in this folder by running `source install/setup.bash`.
*   log/: When things run or build, the system writes down what happened (errors, warnings, and info) into log files saved here. If something breaks, checking these logs can help you find out why.
*   models/: This folder contains generated 3D models for the robot. However, the true "source of truth" for the robot's design is inside `src/niihan_description/urdf/`.

---

## 🚀 How to Start the Robot

To launch the full simulation (which includes mapping, navigation, 3D visualization in RViz, and autonomous patrol), follow these steps:

1.  Open your terminal.
2.  Go to the workspace: 
    ```bash
    cd ~/niihan_ws
    ```
3.  Set up your ROS 2 environment:
    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ```
4.  Launch the robot:
    ```bash
    ros2 launch niihan_description niihan_mapping.launch.py
    ```

**Useful Launch Options:**
*   To start without the visualizer (RViz) or autonomous navigation:
    ```bash
    ros2 launch niihan_description niihan_mapping.launch.py rviz:=false autonav:=false
    ```
*   To run it silently in the background (headless mode):
    ```bash
    ros2 launch niihan_description niihan_mapping.launch.py headless:=true
    ```

---

## 🎮 Manual Driving and Mapping (Teleop)

If you want to drive the robot yourself to build a map of the world:

1.  Launch the manual mapping mode:
    ```bash
    ros2 launch niihan_description teleop_mapping.launch.py
    ```
2.  In the terminal where you ran the command, use your keyboard to drive:
    *   `I` : Move Forward
    *   `J` / `L` : Turn Left / Turn Right
    *   `K` : Stop
    *   `M` / `.` : Reverse Turn Left / Right
    *   `Q` / `Z` : Increase / Decrease Speed

Once you've driven around and mapped the area, you can save the map by opening a new terminal and running:
```bash
cd ~/niihan_ws
./save_base_patrol_map.sh
```
This saves your map files in `~/.ros/niihan_maps/`.

---

## 🛠️ Where to Find Important Files

All your important files are inside the `src/niihan_description` folder. Here is what they do:

### 1. Robot Design (`urdf/` folder)
These files describe what the robot looks like, how heavy it is, and where its sensors are.
*   **`niihan.urdf.xacro`**: The main file that puts the whole robot together.
*   **`niihan_properties.xacro`**: Contains sizes, weights, and wheel speeds.
*   **`niihan_sensors_chassis.xacro` & `niihan_sensors_mast.xacro`**: Defines cameras, LiDARs, and other sensors.

### 2. Launch Instructions (`launch/` folder)
These scripts tell the computer exactly which programs to start.
*   **`niihan_mapping.launch.py`**: The main script you use to run everything.
*   **`niihan_gazebo.launch.py`**: Starts just the simulated world and the robot.

### 3. Settings and Brains (`config/` folder)
*   **`nav2_params.yaml`**: The settings for how the robot plans its path and avoids obstacles.
*   **`slam_toolbox_params.yaml`**: The settings for how the robot creates a map from its sensors.

### 4. Robot Behavior (`scripts/` or direct Python files)
*   **`patrol_controller.py`**: The main "brain" for autonomous movement. It tells the robot to explore, stay within boundaries, and recover if it gets stuck.

### 5. Simulated Worlds (`worlds/` folder)
*   **`niihan_patrol_base.world`**: The default 3D environment the robot drives around in, complete with fences and obstacles.

---

## 🔄 Rebuilding the Workspace

If you change any code or settings in the `src/` folder, you need to rebuild the workspace so your changes take effect:

```bash
cd /home/praj/niihan_ws
# Force stop any running simulations to avoid conflicts
pkill -9 -f 'gz sim' || true
pkill -9 -f 'ros_gz_bridge' || true
pkill -9 -f 'async_slam_toolbox_node' || true

# Setup ROS 2 and Build
source /opt/ros/humble/setup.bash
colcon build --packages-select niihan_description --symlink-install
source install/setup.bash

# Run the robot again
ros2 launch niihan_description niihan_mapping.launch.py
```

---

## 🧪 Helpful Diagnostic Tools

In the main workspace folder, there are several scripts to help you check if things are working:

*   `check_odom.py` - Tells you the robot's current position based on its wheels.
*   `check_pose.py` - Tells you where the robot thinks it is on the map.
*   `check_map.py` - Shows details about the generated map.
*   `test_nav.py` - Sends a quick navigation command to test if the robot can drive to a spot.

*(Note: These are quick tools to run manually, not part of the main launch.)*
