import re

path = "src/niihan_description/urdf/niihan_sensors_mast.xacro"
with open(path, "r") as f:
    content = f.read()

# We want to keep everything from <!-- ============ REAR SENSOR CLUSTER ============ --> to the end.
# And the top <robot xmlns:xacro="...">
header = '<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n'
rear_cluster = content[content.find('<!-- ============ REAR SENSOR CLUSTER ============ -->'):]

with open(path, "w") as f:
    f.write(header + rear_cluster)
