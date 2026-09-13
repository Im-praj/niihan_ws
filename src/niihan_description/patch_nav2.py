import re

with open('config/nav2_params.yaml', 'r') as f:
    content = f.read()

# Fix scan ranges in both global and local costmap
bad_scan = """          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.45
          raytrace_min_range: 0.45
          obstacle_min_range: 0.0"""
good_scan = """          raytrace_max_range: 12.0
          raytrace_min_range: 0.45
          obstacle_max_range: 10.0
          obstacle_min_range: 0.45"""

content = content.replace(bad_scan, good_scan)

# Update inflation radius for better avoidance
content = content.replace('inflation_radius: 0.55', 'inflation_radius: 0.8')
content = content.replace('cost_scaling_factor: 3.0', 'cost_scaling_factor: 5.0')

# Also, voxel_layer vs obstacle_layer for 2D scan in local costmap? 
# Usually, a 2D scan works fine with ObstacleLayer in local costmap too, but voxel_layer is fine if z_voxels is set up.
# We will leave it as voxel_layer since it was already there.
# Let's increase local costmap size to 5x5 so the robot can plan around obstacles better.
content = content.replace('width: 3\n      height: 3', 'width: 5\n      height: 5')

with open('config/nav2_params.yaml', 'w') as f:
    f.write(content)
