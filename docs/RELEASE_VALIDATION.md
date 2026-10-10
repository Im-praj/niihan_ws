# NIihan GLIM + Nav2 validation report — 10 October 2026

**Status: core simulation validated; full project acceptance is incomplete.**

## Verified baseline

Runtime source: `7790302754e1f3386b8ca56eb46be41ef0d6747c`. Validation-test correction: `24f323b`. Three independent, visible simulation restarts used seed 42 and the same flat-site route. Seed fixes simulator randomness; it does not guarantee identical scheduling or maps.

| Evidence run | Waypoints | Maximum relative XYZ error | Maximum relative θ error | Final XYZ goal error | Cloud/pose packets verified |
|---|---:|---:|---:|---:|---:|
| final_repeat_01 | 3/3 | 3.40 cm | 0.0934° | 13.65 cm | 46/46 |
| final_repeat_02 | 3/3 | 3.44 cm | 0.1013° | 14.86 cm | 45/45 |
| final_repeat_03 | 3/3 | 3.94 cm | 0.1009° | 14.32 cm | 44/44 |

All three passed route acceptance, pause/resume, e-stop drive-zero, map saving and runtime ownership/lifecycle checks. XYZ goal tolerance is 25 cm; final yaw is unrestricted. Accuracy scores compare timestamp-aligned displacement and heading against Gazebo truth after removing initial offsets. Position scoring does not fit a rotation. These are simulation drift measurements, not calibrated physical L2 accuracy.

GLIM consumes 3D cloud and IMU; Nav2 receives a projected occupancy grid and planar navigation frames. Ground truth is evaluation-only. The dashboard renders the actual repository URDF, one forward camera and cloud-time X/Y/Z/θ pose. Every one of 135 received map-cloud packets was independently checked against matching-time TF. Browser updates are accumulated map snapshots, approximately 1 Hz, not every raw LiDAR scan.

Repeatability applies to route completion and the stated tolerances. Maps are not byte-identical: occupied-cell IoU across runs is 0.225–0.494. High agreement of predominantly free cells is insufficient to claim repeatable obstacle geometry.

## Build and validation

A separate recursive clone from public GitHub/submodules built successfully: 3 geometry2 packages, 38 Nav2 packages and 6 GLIM/project packages. The clone has independent build/install directories and uses documented preinstalled system prerequisites. It is not a clean-OS or cross-GPU certification. Parent validation: 96 Python tests passed, plus JavaScript map checks, SDF, xacro and MCU-core checks. Public-clone validation: 94 Python tests passed, 2 proprietary-mesh checks skipped; the portable world does not require those meshes.

## Outstanding acceptance

- Latest saved-map restart `final_localization_01` did not pass: the visible Gazebo GUI client exited cleanly, and its required-process setting shut down the whole launch. The probe then lost its WebSocket connection. Startup resilience and this retest remain open.
- `final_safety_01` and `final_obstacle_01` were not reached after that failure. Earlier `safety_02`, `obstacle_02` and `saved_map_02` passed on older source; they do not replace final-baseline retests.
- Physical Unitree L2 GLIM commissioning is incomplete. The existing hardware launch still uses the legacy stack. Real scan timing/deskew, IMU extrinsics and bias, driver settings and motor calibration require hardware validation.
- Saved-map registration is initial-pose-assisted planar ICP of 3D points, not global place recognition or full 6-DOF relocalization.
- Waypoint dwell, extended endurance, slopes/drop-offs/curb traversability, comparative SLAM benchmarks and approved individual native Gazebo/RViz evidence views are not signed off.

## Evidence and publication

Logs, trajectories, maps, desktop WebM recordings and dashboard pictures are retained under `/home/praj/evidence/NIihan_3D_20261009_144316`; failures are preserved. The evidence root is owner-only. Source and README are published on `feat/l2-3d-slam-release`, not the default main branch.

The reproducible commands, dependency pins and hardware limits are in the repository README. This report does not declare all checklist milestones complete.
