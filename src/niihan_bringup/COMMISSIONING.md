# Physical commissioning and acceptance

The simulation checks demonstrate software contracts. They do not certify industrial deployment or guarantee that attaching unconfigured hardware starts autonomous operation.

| Device | Implemented integration | Remaining physical work |
|---|---|---|
| Unitree L2 | Configurable cloud input, TF-based projection, mapping and navigation | Install official ARM64 ROS driver; configure transport and frame; measure extrinsics and blind spots |
| Astra 2 / Femto Mega | Official driver launch hooks, shared color/dashboard input | Select one camera, install vendor driver, validate USB bandwidth and power; depth obstacle fusion is not implemented |
| Pi 5 | Reduced mapper capacity and shared ROS stack | Verify ARM64 OS/ROS compatibility, power supply, cooling, CPU/RAM/storage wear and sustained workload |
| STM32 F446RE | CRC serial bridge, wheel odometry, watchdogs, portable CANopen core | Integrate Cube/HAL, CAN transceiver and disable output; flash and bench-test |
| ZLAC8015D V4 / ZLLG motors | Acknowledged velocity-mode SDOs, separate motor RPM targets, encoder feedback, fault handling | Confirm node ID/bitrate, direction, encoder scaling, measured wheel radius/track, braking and current limits |
| BNO085 | I2C acceleration/gyro/game quaternion driver | Enable I2C, install Blinka/BNO08x, measure mounting and noise; relative yaw is not an absolute heading |
| NC E-stop | Independent inhibit callback and host software stop | Engineer and verify independent power/drive inhibit and braking; do not rely on a ROS Bool |
| ZED-F9P | NAV-PVT polling, RTK quality, covariance, ENU estimate, validated RTCM input | Configure serial port/baud, antenna, correction/base or NTRIP client and credentials; survey datum/map alignment |
| SIM7600 LTE | ROS sensor contracts independent of network; stale data inhibits motion | Configure modem, SIM/APN, power, secure access and correction service; no modem provisioning/NTRIP client is supplied |

## Commission in order

1. Install dependencies and run `scripts/validate.sh`. Build the three owned packages and source that exact overlay.
2. Verify mock fixed/float/no-fix/dropout modes and the Gazebo construction world. Mock RTK labels are test scenarios; Gazebo reports `simulated_fix`.
3. With wheels raised and motor power isolated, verify serial packets, MCU watchdogs, CAN acknowledgements, node identity and actual inhibit polarity. The default hardware launch keeps movement disabled.
4. Verify the NC stop circuit independently of Pi/ROS, including cable break, loss of power, MCU freeze, CAN loss and braking distance. Determine whether this machinery requires additional safety-rated components through the actual system risk assessment.
5. Configure device paths, measured sensor TF, geometry and encoder counts. Example launch arguments are commissioning inputs, not certified dimensions:

   ```bash
   ros2 launch niihan_bringup niihan_hardware.launch.py \
     start_vendor_drivers:=true camera_model:=astra2 \
     stm32_port:=/dev/niihan-stm32 gnss_port:=/dev/niihan-gnss \
     wheel_radius:=MEASURED_METRES track_width:=MEASURED_METRES \
     ticks_per_revolution:=MEASURED_COUNTS hardware_calibrated:=true
   ```

6. Characterize wheel slip, IMU covariance, sensor timing, minimum detectable obstacles and ground coverage before enabling motion. Set `require_cliff:=true` only after validating coverage; unknown perception then blocks movement.
7. Configure the actual GNSS datum and correction stream. Use `require_gnss:=true` for missions that require RTK fixed. Geographic targets additionally need a surveyed relationship between ENU and the SLAM map; that mission conversion is not implemented.
8. Set `drive_enabled:=true` only in a controlled test area after the preceding checks. Test command loss, every required sensor dropout, controller restart, RTK loss and physical stop. Measure resulting stopping distances at maximum commissioned speed.
9. Run prolonged outdoor trials on representative construction terrain. Record navigation completion, localization drift, thermal throttling, CPU/RAM, network interruption, battery/power faults and repeatability across cold boots. Establish acceptance thresholds before industrial operation.

## Reproducibility limits

Mock kinematics and sensor geometry are deterministic at fixed 20 ms steps for a supplied command sequence. ROS scheduling, SLAM optimization, Gazebo physics and asynchronous navigation can vary across machines/runs. A fixed seed does not establish identical mission outcomes. Record ROS/package versions, settings, world, seed, command sequence, calibration and bags when evaluating repeatability.
