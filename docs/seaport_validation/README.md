# Recorded seaport validation

The Word draft records seven straight-route tests on 20 October 2026: 7/7 routes and 21/21 waypoints passed, maximum horizontal arrival error 4.89 cm with ideal simulated GPS. It contains all seven Gazebo world captures, GLIM 3D point cloud and 2D navigation display.

The seven result folders contain recorded scoring, traces, endpoint commands, images, telemetry and saved 3D checkpoints. Absolute paths in original metadata identify the acquisition machine; locate saved maps by basename in the corresponding local checkpoints folder. protocol.json preserves the historical tested source hashes; later portability and documentation updates do not rewrite it. Runnable scripts are under scripts/seaport_validation at the workspace root. Summary CSV/JSON and audit/map checks accompany the report.

Earlier turn/return trials passed 3/7 and are disclosed separately in the report. The local downloadable evidence archive retains development logs. The final statistics exclude those attempts. The tested clear straight lane and ideal GPS do not establish general robustness or real receiver precision.
