# Niihan Scenario Sweep Summary

Generated: 2026-08-25T13:18:35.427188Z

| Scenario | Runs | PASS | FAIL | INCONCLUSIVE | Dominant failure mode |
|---|---|---|---|---|---|
| s01_pedestrian_crossing | 4 | 4 | 0 | 0 | - |
| s02_blocked_corridor | 4 | 0 | 0 | 4 | requires missing subsystems: ['nav_stack'] |
| s03_gnss_degradation | 4 | 0 | 0 | 4 | requires missing subsystems: ['geofence_monitor'] |
| s04_sensor_dropout | 4 | 0 | 0 | 4 | requires missing subsystems: ['fault_manager'] |
| s05_low_battery_dock_resume | 4 | 0 | 0 | 4 | requires missing subsystems: ['battery_model'] |
| s06_slope_tilt | 4 | 4 | 0 | 0 | - |
| s07_teleop_watchdog | 4 | 0 | 0 | 4 | requires missing subsystems: ['teleop_watchdog'] |
| s08_estop | 4 | 0 | 0 | 4 | requires missing subsystems: ['estop_subsystem'] |
| s09_dock_partial_obstruction | 4 | 4 | 0 | 0 | - |
| s10_lighting_sweep | 4 | 4 | 0 | 0 | - |
