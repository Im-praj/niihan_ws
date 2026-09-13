import re

with open('config/nav2_params.yaml', 'r') as f:
    content = f.read()

bad_scan = """          raytrace_max_range: 12.0
          raytrace_min_range: 0.45
          obstacle_max_range: 10.0
          obstacle_min_range: 0.45"""
good_scan = """          raytrace_max_range: 12.0
          raytrace_min_range: 0.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.0"""

content = content.replace(bad_scan, good_scan)
content = content.replace('inflation_radius: 0.8', 'inflation_radius: 0.55')
content = content.replace('cost_scaling_factor: 5.0', 'cost_scaling_factor: 3.0')
content = content.replace('width: 5\n      height: 5', 'width: 3\n      height: 3')

with open('config/nav2_params.yaml', 'w') as f:
    f.write(content)
