import re

with open('src/niihan_description/launch/niihan_full_system.launch.py', 'r') as f:
    content = f.read()

# We want to remove:
# - patrol_controller_node
# - patrol_controller_timer
# - slam_toolbox_node
# - octomap_node
# - vortex_3d_mapper_node
# - patrol_arg, mapping_arg, octomap_arg, vortex_3d_arg, etc from LaunchDescription

# It's easier to just take the file and manually edit it using replace_file_content or a script, but the file is 193 lines.
