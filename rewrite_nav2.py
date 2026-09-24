import re

path = "src/niihan_description/config/nav2_params.yaml"
with open(path, "r") as f:
    content = f.read()

# Remove static_layer from global_costmap plugins
content = re.sub(r'      - static_layer\n', '', content, count=1)

# We need to add rolling_window to global_costmap.
# Let's find global_costmap block and replace its robot_base_frame
def insert_rolling(match):
    return match.group(0) + "\n      rolling_window: true\n      width: 20\n      height: 20"

content = re.sub(r'(global_costmap:[\s\S]*?)      robot_base_frame: base_footprint', insert_rolling, content, count=1)

# Update topics and sensor frames
content = content.replace("/niihan/sensors/lidar/points", "/niihan/sensors/unitree_lidar")
content = content.replace("mast_lidar_link", "unitree_l2_link")

with open(path, "w") as f:
    f.write(content)
