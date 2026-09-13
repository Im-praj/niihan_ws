# Niihan Scenario Sweep Summary

Generated: 2026-08-29T12:14:47.418369Z

| Scenario | Runs | PASS | FAIL | INCONCLUSIVE | Dominant failure mode |
|---|---|---|---|---|---|
| s01_pedestrian_crossing | 4 | 0 | 0 | 4 | sweep axis not yet implemented in world/robot generation: ['angle_deg', 'distance_m', 'speed_mps'] |
| s02_blocked_corridor | 3 | 0 | 0 | 3 | requires missing subsystems: ['nav_stack'] |
| s03_gnss_degradation | 4 | 0 | 0 | 4 | requires missing subsystems: ['localization_confidence_estimator'] |
| s04_sensor_dropout | 4 | 0 | 0 | 4 | requires missing subsystems: ['fault_manager'] |
| s05_battery_low | 4 | 0 | 0 | 4 | requires missing subsystems: ['battery_model', 'dock_nav'] |
| s06_slope_tilt | 4 | 0 | 0 | 4 | sweep axis not yet implemented in world/robot generation: ['pitch_deg', 'roll_deg'] |
| s07_lost_command_link | 3 | 0 | 0 | 3 | requires missing subsystems: ['teleop_watchdog'] |
| s08_estop | 2 | 0 | 0 | 2 | requires missing subsystems: ['estop_subsystem'] |
| s09_dock_obstructed | 3 | 0 | 0 | 3 | requires missing subsystems: ['dock_nav'] |
| s10_lighting_sweep | 3 | 0 | 0 | 0 | - |
