import re
with open('models/robot.urdf', 'r') as f:
    data = f.read()

bad_pattern = r'<topic>/scan</topic><range>\s*<min>\s*0.45</min>\s*<max>12.0</max>\s*<resolution>0.01</resolution>\s*</range>'
good_pattern = """<topic>/scan</topic>
      <gz:frame_id>chassis_lidar_link</gz:frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.45</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>"""
        
data = re.sub(bad_pattern, good_pattern, data, flags=re.DOTALL)

with open('models/robot.urdf', 'w') as f:
    f.write(data)
