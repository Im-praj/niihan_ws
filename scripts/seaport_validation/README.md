# NIIHAN seaport navigation suite

Seven static Gazebo Harmonic worlds share the same quay, vessel, crane, warehouse, lanes, robot spawn and patrol waypoints. Cargo centres move up to 0.8 m east/west and 0.6 m north/south. World seeds and the controller procedure are held fixed.

The suite launches the real installed `niihan_description` Gazebo, GLIM CPU 3D SLAM and Nav2 stack. It sends ROS NavigateToPose actions. It does not replace robot motion with a geometric path animation. Truth is used for scoring only. GNSS endpoint correction uses Gazebo NavSat coordinates, WGS84 ENU conversion and the robot antenna lever arm from TF.

GPS is ideal, without added noise or satellite signal propagation. Twelve synthetic satellite sky markers are explicit display fixtures; they do not produce the GPS position. RTK is always false and DOP is unavailable. This setup does not demonstrate real GNSS accuracy.

Procedure per waypoint: navigate with the existing 0.20 m map goal tolerance, record uncorrected truth error, then run a bounded GPS final approach using simulated IMU heading. The final approach has a 0.05 m deadband, 1 m maximum correction radius, 0.08 m/s speed cap, 0.5 rad/s angular cap and 0.75 m minimum scan clearance. GPS, IMU, odometry and scan must stay fresh. Arrival needs one simulation second stopped inside the deadband. Final acceptance is 0.10 m in ground truth and GPS for all three waypoints. Wheel odometry and IMU angular rate provide motion verification, not LIO estimated velocity. The comparison measures the combined Nav2 and GPS final approach, rather than isolating GPS sensor contribution from the final controller.

The dashboard is an observer. Its idle manual and e-stop publications are remapped to an unused observer namespace so it cannot override the automated runner. Mission controls in this view are disabled. The actual robot command arbiter remains active. The final tests keep GLIM 3D SLAM and the 2D navigation projection enabled together. The cliff detector is disabled. Each run must save a non-empty 3D map. Each layout starts with a fresh online SLAM map, so this is not a test of reusing one frozen map in changed surroundings.

Run `bash scripts/run_suite.sh` from this package after sourcing `/opt/ros/humble/setup.bash` and `<workspace>/install/local_setup.bash`. The packaged runner writes repeat runs to runs/YYYYMMDD_HHMMSS and preserves the original results folder. Set NIIHAN_SEAPORT_OUTPUT to choose another output path. Use python3 scripts/aggregate.py <run_directory> to summarise a repeat run. World images are actual Gazebo camera captures. `pilot_01`, `dashboard_interference_diagnostic`, `nav2_correction_diagnostic` and `wheel_heading_diagnostic`, `imu_reverse_diagnostic`, `transport_startup_diagnostic` and `forward_approach_diagnostic` retain unsuccessful development attempts and are excluded from the final seven-layout statistics.

Each layout uses a separate ROS domain and a unique Gazebo partition. Cleanup tracks and terminates only descendants of processes started by the runner. This prevents stale simulator topics from affecting a later layout. The suite depends on the existing ROS environment and Python psutil, Pillow and websockets.

The final feedback law is v = clamp(0.5 times forward position error, +/-0.08 m/s) and w = clamp(2.0 times lateral position error, +/-0.5 rad/s). Position errors are transformed using the simulated IMU heading. It can reverse during the final approach. No truth pose is used by this feedback.

Final straight-route results are recorded in summary.json and results.csv. The identical world-frame waypoints are (-9, 0), (-6, 0) and (-3, 0) metres, starting from (-12, 0). Earlier turn and return tests are retained under results/prior_return_route and excluded from the final straight-route statistics.

Final measured outcome: 7/7 layouts and 21/21 waypoints passed. Mean horizontal arrival error 4.688 cm; maximum 4.887 cm with ideal simulated GPS. All seven GLIM maps passed finite-point and occupancy-grid checks. This supports only the tested clear straight corridor.

The shell wrapper locates the workspace relative to itself. psutil and Pillow are installed by setup. Frozen results and the Word draft are in docs/seaport_validation. run_suite_original.py is a historical acquisition snapshot; use run_suite.sh for new runs.
