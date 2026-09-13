# Niihan Scenario Sweep Summary

Generated: 2026-08-25T13:09:48.722257Z

| Scenario | Runs | PASS | FAIL | INCONCLUSIVE | Dominant failure mode |
|---|---|---|---|---|---|
| s01_pedestrian_crossing | 1 | 0 | 0 | 0 | - |
| s02_blocked_corridor | 1 | 0 | 0 | 1 | requires missing subsystems: ['nav_stack'] |
| s03_gnss_degradation | 1 | 0 | 0 | 1 | requires missing subsystems: ['geofence_monitor'] |
| s04_sensor_dropout | 1 | 0 | 0 | 1 | requires missing subsystems: ['fault_manager'] |
| s05_low_battery_dock_resume | 1 | 0 | 0 | 1 | requires missing subsystems: ['battery_model'] |
| s06_slope_tilt | 1 | 0 | 0 | 0 | - |
| s07_teleop_watchdog | 1 | 0 | 0 | 1 | requires missing subsystems: ['teleop_watchdog'] |
| s08_estop | 1 | 0 | 0 | 1 | requires missing subsystems: ['estop_subsystem'] |
| s09_dock_partial_obstruction | 1 | 0 | 0 | 0 | - |
| s10_lighting_sweep | 1 | 0 | 0 | 0 | - |
