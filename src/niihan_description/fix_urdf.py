import re

with open('urdf/niihan.urdf.xacro', 'r') as f:
    content = f.read()

content = content.replace("<joint_name>left_rocker_joint</joint_name>", "")
content = content.replace("<joint_name>right_rocker_joint</joint_name>", "")
new_joints = """
      <joint_name>front_left_suspension_joint</joint_name>
      <joint_name>front_right_suspension_joint</joint_name>
      <joint_name>rear_left_suspension_joint</joint_name>
      <joint_name>rear_right_suspension_joint</joint_name>
"""
content = content.replace("</plugin>", new_joints + "</plugin>", 1)

with open('urdf/niihan.urdf.xacro', 'w') as f:
    f.write(content)
