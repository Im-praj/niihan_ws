with open('urdf/niihan.urdf.xacro', 'r') as f:
    content = f.read()

content = content.replace("""    
      <joint_name>front_left_suspension_joint</joint_name>
      <joint_name>front_right_suspension_joint</joint_name>
      <joint_name>rear_left_suspension_joint</joint_name>
      <joint_name>rear_right_suspension_joint</joint_name>
</plugin>""", "</plugin>")

content = content.replace("""      <joint_name>ptz_tilt_joint</joint_name>
      
      
    </plugin>""", """      <joint_name>ptz_tilt_joint</joint_name>
      <joint_name>front_left_suspension_joint</joint_name>
      <joint_name>front_right_suspension_joint</joint_name>
      <joint_name>rear_left_suspension_joint</joint_name>
      <joint_name>rear_right_suspension_joint</joint_name>
    </plugin>""")

with open('urdf/niihan.urdf.xacro', 'w') as f:
    f.write(content)
