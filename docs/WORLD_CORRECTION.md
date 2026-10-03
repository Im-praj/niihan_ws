# Construction world correction

The initial imported procedural world shared its model layout with niihan_patrol_base.world. Its new filename did not make it visually distinct. That implementation is superseded.

The installed default now uses the supplied chunk5 ground/site GLBs for visual and collision geometry, with a new surrounding construction layout: crane, unfinished steel building, office, container, barriers and material stacks. It has 92 static models, and the robot spawns at (-12, 0, 0.15). The original textured visual mesh was not supplied; this uses simplified collision meshes with materials. Proprietary assets remain local and ignored by Git.

Verification: both packages rebuilt, 72 Python tests passed, 9 browser checks passed, Xacro expanded and SDF validated. The installed world completed 1,000 Gazebo steps. A full-stack smoke received scans, mast point clouds, maps and TF, and Nav2 reached active state. A front-camera frame confirmed that the site geometry renders. The existing panoramic camera sensor has a 180-degree roll, so its feed is inverted; it was not changed during this world correction. Hardware deployment and repeatable autonomous routes remain unverified.

Stop the previous Gazebo/ROS launch with Ctrl+C, then run:

```bash
cd /home/praj/niihan_ws
./scripts/run_construction.sh
```

This script explicitly sources niihan_ws/install/setup.bash and selects world:=niihan_construction_site.sdf, avoiding another sourced workspace's package selection.
