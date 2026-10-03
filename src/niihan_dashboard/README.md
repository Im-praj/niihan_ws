# niihan_dashboard

Browser telemetry, 2D/3D maps, navigation missions and geofence controls for the NIIHAN ROS 2 stack.

```bash
ros2 launch niihan_dashboard dashboard.launch.py use_sim_time:=true
```

Open http://127.0.0.1:8080. WebSocket commands use port 8081. `bind_host` defaults to `127.0.0.1`; changing it exposes unauthenticated motion controls. Remote operation requires a secured tunnel or authenticated reverse proxy and an appropriate network policy. Use `use_sim_time:=false` only with a separately validated real-time hardware stack.

The 3D view currently depends on CDN-hosted Three.js assets; the 2D map tests do not establish offline 3D rendering readiness. The dashboard software stop is an aid and does not replace the physical emergency stop.

`test/test_navigation_results.py` covers mission result/cancellation races. `test/test_web_viewer2d.js` checks map rendering, coordinate transforms and input behavior. Run both through `scripts/validate.sh` at the workspace root.
