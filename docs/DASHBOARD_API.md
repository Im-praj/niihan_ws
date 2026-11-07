# Dashboard API reference

Reviewed against the repository implementation on 2026-10-27. This describes implemented interfaces, not a claim that physical RTK commissioning or end-to-end acceptance is complete. Sources: `dashboard_node.py`, `ros_bridge.py`, `mission_manager.py`, `geofence_manager.py`, and `robot_model.py` under `src/niihan_dashboard/niihan_dashboard/`.

## Connection and conventions

| Interface | Default address | Purpose |
| --- | --- | --- |
| HTTP | `http://127.0.0.1:8080/` | Static dashboard assets; no REST control endpoints |
| WebSocket | `ws://127.0.0.1:8081/` | Bidirectional JSON text messages |

Launch with `ros2 launch niihan_dashboard dashboard.launch.py use_sim_time:=true`. Match the stack clock; use `false` for commissioned hardware. `bind_host` defaults to loopback. Neither server implements authentication, authorization, TLS, per-client control ownership, nor command audit persistence. Remote deployments need a secured tunnel or authenticated proxy, including WebSocket access. Multiple clients share the same robot and mission state.

Coordinates are metres in the configured global frame (default `map`); angles are radians, velocity m/s and rad/s. Timestamps are ROS-clock seconds: simulated time when enabled, not necessarily Unix wall time. GPS latitude/longitude use degrees; altitude comes directly from NavSatFix (confirm the driver datum). Missing sensor fields may be absent or null. Clients must not convert unavailable data to zero or treat cached data as fresh.

The server sends telemetry approximately every 100 ms. Other streams are sent when their generation changes and cached streams are sent to a newly connected client. This is a latest-value display interface, not a lossless recorder or a hard real-time transport. There are no subscriptions, replay cursors, protocol version negotiation or request IDs. Dispatch by `type`; tolerate additional fields. Reconnect and rebuild state from fresh messages. Do not automatically replay motion commands after reconnect.

## Telemetry

Envelope: `{"type":"telemetry","timestamp":12.3,...}`.

| Field | Contents / meaning |
| --- | --- |
| `pose` | `x,y,z,yaw`; `roll,pitch` added after valid TF. Global-frame rover base pose |
| `pose_valid` | Freshness/validity gate for TF pose; check before using coordinates |
| `velocity` | `linear_x,linear_y,angular_z` from `/odom` |
| `mode`, `estop` | `MANUAL` or `AUTO`; software stop boolean |
| `localization` | `source,health,confidence,age`; confidence can be null |
| `nav2_ready` | Dashboard readiness observation, not completion of a goal |
| `navigation` | `state,message,target`; optional `xyz_error_m,verification_source` |
| `mission` | `state,waypoints_count,current_index,waypoints,message`; index is zero-based |
| `geofence` | `enabled,vertices,polygon` with polygon points `{x,y}` |
| `gnss` | Receiver metrics described below |
| `hardware_health` | Health supervisor object; initial `ready:false,faults:["waiting_for_supervisor"]` |
| `slam` | Backend status object forwarded from `/niihan/slam/status`; initially backend unknown |
| `path_history` | Array of global `{x,y}` samples for display |

Waypoint objects contain `id,x,y,z,yaw,frame,status`. IDs are allocated by the server; obtain them from telemetry. Dashboard commands leave `yaw:null`; final heading does not constrain arrival. Navigation states emitted by the implementation include `IDLE,PENDING,RUNNING,PAUSING,PAUSED,CORRECTING,VERIFYING,COMPLETED,FAILED,CANCELLED`. Some states may be transient between telemetry updates. A successful command response acknowledges handling, not physical arrival: observe subsequent navigation/mission state.

### GPS / ZED-F9P RTK

| `gnss` field | Meaning |
| --- | --- |
| `available` | NavSatFix status and finite coordinates accepted |
| `latitude,longitude,altitude` | Raw fix coordinates; may be null |
| `covariance` | Nine row-major position covariance entries from NavSatFix, m² |
| `stamp` | Latest fix/metrics stamp; independent callbacks update this field |
| `quality` | Source quality string, e.g. standalone/float/fixed/simulated_fix/unknown |
| `satellite_count` | Receiver NAV-PVT numSV (satellites used in navigation solution), or null |
| `hdop,vdop` | Dimensionless NAV-DOP values, or null |
| `lock,rtk_lock` | Valid GNSS solution / valid fixed RTK solution |
| `fresh` | Dashboard freshness assessment; stale data clears locks and DOP/count |
| `horizontal_accuracy_m,vertical_accuracy_m` | Receiver-reported accuracy estimates, when provided; not independently proven errors |
| `simulation` | Explicit simulated source indicator, when provided |
| `timestamp_source` | E.g. receiver_utc, receive_time or simulation_clock |
| `map_pose` | Optional calibrated antenna-offset-corrected base `{x,y,z,stamp}` from RTK alignment |
| `goal_verification_required` | Whether RTK arrival verification is enabled |

These fields arrive asynchronously; the WebSocket object is not an atomic raw-fix/metrics measurement pair. Use the ROS topics for precision synchronization. A map_pose may remain cached after loss: it is not evidence of current lock. DOP/count are unavailable for ordinary Gazebo NavSat and remain null. Simulation fixture counts/accuracy are injected test values. A single antenna does not supply measured stationary heading. Physical accuracy still requires corrections, surveyed map alignment, measured antenna offset and synchronized clocks.

## Map, cloud, model and camera streams

| `type` | JSON fields |
| --- | --- |
| `map` | `frame_id,revision,width,height,resolution,origin:{x,y,yaw},data` |
| `pointcloud` | `frame_id,stamp,pose_stamp,pose:{x,y,z,yaw,roll,pitch},data` |
| `camera` | `image` (base64 JPEG, without data-URL prefix) |
| `robot_model` | `name,root,links,joints` derived from the live URDF |
| `joint_states` | `positions` object mapping joint name to position |

Map data is a flat ROS occupancy-grid array: index `y*width+x`, unknown -1, occupancy 0–100. Origin includes grid yaw. Revision is a process-local generation counter, not a persisted map ID. Pointcloud data is flat `[x,y,z,x,y,z,...]` in frame_id. Cloud and accompanying vehicle pose share the measurement timestamp; clouds are skipped if the matching TF is unavailable. Display sampling selects at most approximately 10,000 points and callbacks are limited to approximately 5 Hz. Use ROS PointCloud2 for full-resolution data.

Camera uses the single `/niihan/sensors/panoramic/front/image_raw` feed, rotated 180 degrees and encoded JPEG quality 50 by the bridge. Render with `data:image/jpeg;base64,` plus image. It currently has no camera timestamp, intrinsics or frame ID in the WebSocket envelope.

Model links: `{name,visuals:[{xyz:[x,y,z],rpy:[r,p,y],geometry,rgba:[r,g,b,a]}]}`. Geometry supports box `{kind:"box",size:[x,y,z]}`, cylinder `{kind:"cylinder",radius,length}` and sphere `{kind:"sphere",radius}`. Mesh visuals are currently unsupported by this exporter. Joints contain `name,parent,child,kind,axis,xyz,rpy`. Joint positions use metres for prismatic joints and radians for rotational joints.

## Commands

Send one JSON object with `action` and the listed arguments. All coordinate arguments must be finite numbers. Optional z defaults to the currently stored rover z; clients should send explicit z after checking pose validity. Yaw is not accepted as an arrival constraint.

| Action | Additional fields | Effect |
| --- | --- | --- |
| `set_mode` | `mode:"MANUAL"` or `"AUTO"` | Changes mode; cancels navigation on mode change |
| `joystick` | `linear_x,angular_z` (default 0) | Publishes manual Twist if gates allow |
| `nav_goal` | `x,y,z?` | Requests a map-frame Nav2 goal |
| `add_waypoint` | `x,y,z?` | Appends a waypoint |
| `update_waypoint` | `id,x,y,z?` | Replaces waypoint coordinates |
| `delete_waypoint` | `id` | Removes waypoint |
| `reorder_waypoint` | `id,direction:"up"` or `"down"` | Moves waypoint one position |
| `clear_mission` | none | Clears editable waypoints |
| `write_mission` | none | Validates existing mission and marks READY; not a disk upload |
| `start_mission` | none | Starts a READY mission |
| `pause_mission` | none | Requests cancellation of current Nav2 goal and pause |
| `resume_mission` | none | Resumes paused mission after readiness checks |
| `cancel_mission` | none | Requests cancellation; observe telemetry for completion |
| `set_geofence` | `polygon:[{x,y},...]` | Validates polygon and publishes keepout mask |
| `clear_geofence` | none | Disables the geofence |
| `estop` | none | Activates software stop and cancels navigation |
| `clear_estop` | none | Requires no pending navigation cancellation, fresh pose and usable geofence position |

Mission/geofence edits are rejected during active navigation. MissionManager also restricts editing running/paused missions. Navigation requires AUTO, no stop, fresh localization, Nav2 readiness, valid mission/route and configured health/RTK gates. Default geofence clearance is 0.45 m. The manual command path does not promise a speed clamp: validate requested speeds in the integration and commissioned drive configuration. Stop commands remain software controls, not substitutes for the physical emergency stop.

Normal response: `{"type":"command_response","action":"set_mode","success":true,"message":"Set mode applied."}`. write_mission uses `type:"mission_write_response"` with the same other fields. Accepted joystick commands return no acknowledgement. Rejections use success false and an explanatory message. Malformed JSON/unexpected exceptions instead return `{"error":"..."}` without type/action. Unknown actions are rejected. There is no correlation ID: serialize commands when responses matter, and do not retry motion merely because an acknowledgement was lost.

### Example mission sequence

Send each line separately, checking the response and fresh telemetry between commands. Coordinates are examples, not a validated route; select reachable coordinates inside the operating area.

```json
{"action":"set_mode","mode":"AUTO"}
{"action":"add_waypoint","x":1.0,"y":0.0,"z":0.0}
{"action":"write_mission"}
{"action":"start_mission"}
```

Default arrival uses 0.25 m XYZ tolerance. With RTK verification enabled the configured tolerance cannot exceed 0.10 m. Arrival requires a stopped pose held for at least the configured dwell (default one second), with at least three distinct measurement stamps. RTK mode requires fresh fixed/aligned data; loss during an RTK-dependent mission triggers stop/cancellation. Up to three bounded geofence-checked XY corrections (each at most 0.50 m) can be attempted. These are implemented policies, not a statement of completed physical validation or guaranteed accuracy.

## ROS interfaces for integrations beyond the browser

| Name | ROS type | Dashboard relationship |
| --- | --- | --- |
| `/odom` | nav_msgs/Odometry | Velocity input |
| TF map → base_footprint | tf2 | Live pose and timestamped cloud pose |
| `/map` | nav_msgs/OccupancyGrid | Occupancy input |
| `/vortex/point_cloud_map` | sensor_msgs/PointCloud2 | Default display cloud; configurable slam_cloud_topic |
| `/robot_description` | std_msgs/String | Actual model input |
| `/joint_states` | sensor_msgs/JointState | Joint input |
| `/niihan/sensors/panoramic/front/image_raw` | sensor_msgs/Image | Single camera input |
| `/niihan/imu/data` | sensor_msgs/Imu | Consumed internally; IMU values are not currently serialized in telemetry |
| `/niihan/gnss/fix` | sensor_msgs/NavSatFix | Raw GNSS input |
| `/niihan/gnss/quality`, `/niihan/gnss/metrics` | std_msgs/String | Quality string / JSON metrics input |
| `/niihan/gnss/map_pose` | nav_msgs/Odometry | Independently aligned RTK base pose input |
| `/niihan/health`, `/niihan/slam/status` | std_msgs/String | JSON status input |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | Navigation action client |
| `/niihan/cmd_vel/manual` | geometry_msgs/Twist | Manual command output |
| `/niihan/e_stop` | std_msgs/Bool | Software stop output |
| `/niihan/geofence_mask` | nav_msgs/OccupancyGrid | Keepout mask output |
| `/niihan/geofence_filter_info` | nav2_msgs/CostmapFilterInfo | Filter metadata output |
| `/niihan/trial/status` | std_msgs/String | Trial outcome JSON output |
| `/niihan/slam/save_map` | std_srvs/srv/Trigger | Map-save service used after trial outcomes |

Manual map saving is available via `ros2 service call /niihan/slam/save_map std_srvs/srv/Trigger '{}'`, when the map bridge is running. No browser save_map command exists. Map persistence/site selection is configured through SLAM launch arguments and map storage, not an HTTP upload API.

## Currently unavailable / integration gaps

No REST telemetry/control API, OpenAPI contract, authentication/token issuance, API versioning, per-request correlation, persistent mission CRUD, map download/upload endpoint, camera calibration endpoint, NTRIP configuration endpoint, raw IMU stream over WebSocket, battery telemetry or measured GNSS heading is implemented here. No firmware update or deployment API is provided. A consumer needing these must add and validate them rather than assume endpoint names.

This review was a source-level interface audit. It did not start motors, run a simulation or establish that every producer is healthy on a deployed robot. Existing acceptance evidence and unresolved commissioning requirements must be assessed separately.
