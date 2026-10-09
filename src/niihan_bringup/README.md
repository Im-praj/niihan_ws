# NIIHAN shared navigation, dashboard and hardware contracts

All backends include the same application launch: local EKF, LiDAR adapter, SLAM, Nav2, dashboard, GNSS monitor/global estimate, arbiter and health-gated drive output.

```bash
cd ~/niihan_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select niihan_description niihan_dashboard niihan_bringup
source install/setup.bash
ros2 launch niihan_bringup niihan_simulation.launch.py backend:=mock
ros2 launch niihan_bringup niihan_simulation.launch.py backend:=gazebo headless:=true
ros2 launch niihan_bringup niihan_hardware.launch.py
```

## Data contracts

- `/niihan/wheel/odom` + `/niihan/imu/data` feed the local EKF, which alone publishes `/odom` and `odom -> base_footprint`.
- Unitree cloud input (`cloud_topic`) is adapted to `/niihan/sensors/lidar/points` and projected to `/scan` in the base frame. These feed the same mapper, cliff detector and SLAM/Nav2 configuration.
- SLAM alone publishes `/map` and `map -> odom`.
- `/niihan/gnss/fix` carries NavSatFix; `/niihan/gnss/quality` explicitly distinguishes RTK fixed/float/standalone/no fix. NavSatFix status alone is not interpreted as RTK.
- Accepted fixes produce ENU `/niihan/gnss/odometry` and an auxiliary `/niihan/gnss/filtered` estimate, with no competing TF.
- `/niihan/gnss/rtcm` accepts complete CRC-valid RTCM3 frames from an authorized correction client.
- The arbiter's `/niihan/cmd_vel` passes through a motion gateway to `/niihan/drive/cmd_vel` only when commands and required sensor/drive health are fresh.
- `/niihan/health` and GNSS data are exposed by the same dashboard in every backend.

GNSS ENU is not automatically the SLAM map frame. Survey the datum/map alignment before geographic missions. GNSS is integrated as global position, quality monitoring and an optional motion prerequisite (`require_gnss:=true` requires fresh RTK-fixed data), not an uncalibrated replacement for SLAM. A single stationary F9P provides no heading; BNO game rotation yaw is relative.

See [commissioning and acceptance](COMMISSIONING.md) for device-specific gaps and field tests.

## Physical commissioning

Hardware mode never fabricates missing data. Missing serial devices retry and remain faults. Defaults inhibit movement. Actuation requires `hardware_calibrated:=true`, measured `wheel_radius`, `track_width`, valid `ticks_per_revolution`, and `drive_enabled:=true`.

Integrate and flash the portable STM32 core and V4 CANopen backend in [firmware](firmware/README.md) into an STM32Cube/HAL project. It uses USB/UART serial to the Pi and CANopen to ZLAC8015D V4. Host-compiled tests do not establish flashed-board readiness. Configure physical CAN transceiver, termination, bitrate, node ID, motor direction, encoder counts and actual inhibit output. The independent physical E-stop/braking circuit remains essential.

Install official `unitree_lidar_ros2` and `orbbec_camera` drivers for the selected ARM64 OS/ROS version. Enable `start_vendor_drivers:=true`, choose `camera_model:=astra2|femto_mega`, and configure actual cloud/color topics and sensor-frame TF. Sensor extrinsics in the existing robot model are provisional and must be calibrated. Orbbec color reaches the dashboard; depth-based obstacle fusion is not implemented.

BNO085 needs Adafruit Blinka/BNO08x Python libraries and enabled I2C on the Pi. F9P uses serial UBX NAV-PVT and requires authorized RTCM corrections for RTK. LTE carrier/power/network setup and correction credentials are installation work. No credentials are embedded.

The mapper uses a smaller 250,000-voxel default in this bringup. Pi CPU/memory/thermal performance still requires measurement. `require_cliff:=true` makes stale or unknown cliff perception inhibit movement; it defaults false pending ground-visibility calibration. Odometry, scan, cloud, IMU and drive freshness always gate motion.

## No-hardware scenarios

`backend:=mock` generates deterministic wheel odometry, IMU, LiDAR room geometry, a placeholder image and GNSS. `gnss_mode:=rtk_fixed|rtk_float|standalone|no_fix|dropout` exercises quality and loss scenarios. No mock node or clock publisher starts in hardware mode. Gazebo uses its physical sensor simulation and a distinct `simulated_fix` quality, not a fabricated RTK status.
