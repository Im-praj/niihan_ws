> Superseded world geometry: the initial procedural scene duplicated the legacy layout. The corrected default now renders the supplied chunk5 meshes and adds a distinct 92-model site layout. Validation for the correction is recorded in WORLD_CORRECTION.md. Earlier map dimensions and three-repeat results apply only to the initial scene.

# NIIHAN workspace validation — 3 October 2026

Target: `/home/praj/niihan_ws`. Existing first-party and upstream source changes were preserved; no commit or push was made.

## Changes

- Imported the supplied construction assets and created `niihan_construction_site.sdf` as the default scene: fences, structural frame, machinery, storage materials, access road, dock and ground surfaces.
- Chose local visible procedural geometry. The supplied GLB world lacks visual geometry; its collision assets remain optional/local and are ignored for Git publication because their supplied license is proprietary.
- Removed the external moving-obstacle model from the repeatable static baseline and enabled Contact/NavSat systems for simulated robot sensors.
- Added seed and spawn controls, EGL rendering in headless mode, and one canonical simulation launch with a compatibility alias.
- Disabled automatic patrol by default. Removed the unused `/cmd_vel` simulator bridge. Dashboard HTTP/WebSocket servers default to loopback.
- Fixed Python shutdown handlers that previously raised errors after ROS context shutdown.
- Removed 44 backed-up unused scripts, empty placeholders, obsolete test drafts and generated TF snapshots. Generated build/install/log files, bytecode and recordings are untracked; existing runtime outputs and recordings remain on disk.
- Replaced the workspace README/guide and added package READMEs, deployment criteria, repeatability instructions and a validation script.
- Rebuilt both updated first-party packages in the normal workspace install overlay.

## Evidence

| Check | Result |
| --- | --- |
| Isolated first-party build | Both packages succeeded |
| Normal workspace build | Both packages succeeded |
| Python regression suites | 71 passed |
| Browser map regressions | 9 passed |
| Xacro expansion | Passed |
| Construction SDF validation | Valid |
| Standalone seeded world | Three independent runs completed 1,000 iterations each, exit code 0 |
| Full simulation | Clock, odometry, scan, mast cloud, map and TF received |
| Nav2 lifecycle | Managed nodes reached active state |
| Final Python shutdown | Owned simulation nodes and dashboard finished cleanly |
| 3D map persistence | Saved successfully, 71,611 points in final smoke run |
| Git whitespace checks | Passed for unstaged and staged diffs |

The final smoke observed 99 scans, 93 mast clouds and 31 maps over approximately 35 seconds of wall time. The last clock value was 9.879 simulation seconds; this sensor-heavy stack ran slower than real time on the test host. The observed occupancy grid was 226 × 394 cells. No motion commands were sent.

## Remaining limitations

- Full autonomous routes, arrival accuracy, obstacle avoidance, physical stop distance and map/trajectory equivalence across runs were not validated. Three successful world loads establish consistent startup, not identical mission outputs.
- Physical sensor/motor drivers, hardware bringup and independently functioning E-stop/watchdog still need acceptance testing. The simulation full-system launch is not a hardware deployment launch.
- Gazebo emitted EGL/DRI warnings while sensors nevertheless produced data. On shutdown its upstream launcher escalated from SIGINT to SIGTERM after five seconds; all owned Python nodes stopped cleanly. This simulator shutdown/performance issue remains to be characterized on the intended deployment host.
- Dashboard has no application authentication or TLS. Its 3D viewer still uses external CDN JavaScript.
- Root/Nav2/GLIM/GLIM ROS checkouts contain pre-existing local changes. Snapshot hashes alone cannot reproduce dirty source; a release must preserve patches and lock dependencies.
- Maintainer placeholders, authoritative repository licensing and proprietary mesh redistribution rights remain unresolved for public release.

## Run

```bash
cd /home/praj/niihan_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch niihan_description niihan_full_system.launch.py
```

Open the dashboard at `http://127.0.0.1:8080`. For server-only simulation add `headless:=true seed:=42 rqt_cam:=false`.

## Recovery and review

Cleanup recovery archives, the exact cleanup manifest, the original tracked changes patch, regression logs, full-stack logs and source/world hashes are supplied with this report. Generated file removals are staged in the Git index because they must stop being tracked; source edits remain available for review. Nothing was committed or published.
