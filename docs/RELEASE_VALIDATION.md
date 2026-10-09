# Release validation

The current 3D integration is under live acceptance testing. No complete-release claim is made until route repetition, saved-map restart, safety checks and clean-clone build have passed.

Acceptance route: (1,0), (1,1), (0.1,0.1), yaw 0, in the SLAM map frame. Criteria fixed before final repetitions: all waypoints completed; final position within 0.25 m; maximum relative XY error against Gazebo world pose <=0.20 m; three restarts with identical source/config/seed. Score TF poses with timestamp alignment, removing only initial translation. Wheel odometry is not ground truth. Preserve raw trajectories and failed runs.

The saved-map test must reload the NPZ without remapping, produce accepted registration and complete navigation. Test command rejection with stale health, an asserted e-stop, and continuing motion requests. Observe actual drive commands, not just GUI badges. Dynamic obstacle and sensor-loss tests are separate evidence items.

A clean-clone test starts from committed source and public submodules, builds into its own build/install directories in a fresh ROS shell, runs validation and launches the visible simulator. It shares documented system prerequisites; it is not a fresh-OS or cross-GPU certification. Proprietary GLB meshes are not required by the portable world.

The architecture document lists multiple candidate SLAM systems. GLIM is the integrated baseline; comparative FAST-LIO2/LIO-SAM/R3LIVE/LVI-SAM/RTAB-Map/KISS-ICP benchmarks are not claimed by this release.
