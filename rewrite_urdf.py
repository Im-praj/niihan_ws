import re

def update_properties():
    path = "src/niihan_description/urdf/niihan_properties.xacro"
    with open(path, "r") as f:
        content = f.read()

    # Update dimensions
    content = re.sub(r'<xacro:property name="chassis_width" value="0.500"/>', '<xacro:property name="chassis_width" value="0.300"/>', content)
    content = re.sub(r'<xacro:property name="ground_clearance" value="0.120"/>', '<xacro:property name="ground_clearance" value="0.200"/>', content)
    content = re.sub(r'<xacro:property name="wheelbase" value="0.400"/>', '<xacro:property name="wheelbase" value="0.500"/>', content)
    content = re.sub(r'<xacro:property name="track_width" value="0.430"/>', '<xacro:property name="track_width" value="0.525"/>', content)
    
    # Update wheels
    content = re.sub(r'<xacro:property name="wheel_diameter" value="0.280"/> <!-- 280mm -->', '<xacro:property name="wheel_diameter" value="0.250"/> <!-- Placeholder: confirm with real measurement before build -->', content)
    content = re.sub(r'<xacro:property name="wheel_width" value="0.070"/> <!-- 70mm -->', '<xacro:property name="wheel_width" value="0.080"/> <!-- Placeholder: confirm with real measurement before build -->', content)

    # Add suspension properties
    suspension_props = """
  <!-- Suspension -->
  <xacro:property name="suspension_arm_length" value="0.150"/>
  <xacro:property name="suspension_arm_width" value="0.040"/>
  <xacro:property name="suspension_arm_mass" value="1.0"/>
"""
    content = content.replace("<!-- Wheels -->", suspension_props + "\n  <!-- Wheels -->")

    with open(path, "w") as f:
        f.write(content)


def update_chassis():
    path = "src/niihan_description/urdf/niihan_chassis.xacro"
    with open(path, "r") as f:
        content = f.read()

    # We need to replace the rocker macro with independent corner suspension macro
    # and update the wheel instantiations
    
    # Remove old niihan_rocker macro and instantiations
    pattern_rocker = re.compile(r'<xacro:macro name="niihan_rocker".*?</xacro:macro>\s*<xacro:niihan_rocker side="left" y_off="\$\{track_width/2\}"/>\s*<xacro:niihan_rocker side="right" y_off="\$\{-track_width/2\}"/>', re.DOTALL)
    
    # We will implement independent corner macro
    suspension_macro = """
  <xacro:macro name="niihan_suspension" params="prefix side_reflect front_reflect x_off y_off">
    <link name="${prefix}_arm">
      <visual>
        <origin xyz="${-front_reflect * suspension_arm_length/2} 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${suspension_arm_length} ${suspension_arm_width} ${suspension_arm_width}"/>
        </geometry>
        <material name="${prefix}_metal"><color rgba="0.6 0.6 0.6 1.0"/></material>
      </visual>
      <collision>
        <origin xyz="${-front_reflect * suspension_arm_length/2} 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${suspension_arm_length} ${suspension_arm_width} ${suspension_arm_width}"/>
        </geometry>
      </collision>
      <xacro:box_inertia mass="${suspension_arm_mass}" x="${suspension_arm_length}" y="${suspension_arm_width}" z="${suspension_arm_width}" ox="${-front_reflect * suspension_arm_length/2}" oy="0" oz="0"/>
    </link>
    <joint name="${prefix}_suspension_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_arm"/>
      <!-- Pivot point on the chassis -->
      <origin xyz="${x_off + front_reflect * suspension_arm_length} ${y_off} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.4" upper="0.4" effort="100.0" velocity="2.0"/>
      <dynamics damping="50.0" friction="0.5" spring_stiffness="2000.0" spring_reference="0.0"/>
    </joint>
  </xacro:macro>

  <xacro:niihan_suspension prefix="front_left" side_reflect="1" front_reflect="1" x_off="${wheelbase/2}" y_off="${track_width/2}"/>
  <xacro:niihan_suspension prefix="front_right" side_reflect="-1" front_reflect="1" x_off="${wheelbase/2}" y_off="${-track_width/2}"/>
  <xacro:niihan_suspension prefix="rear_left" side_reflect="1" front_reflect="-1" x_off="${-wheelbase/2}" y_off="${track_width/2}"/>
  <xacro:niihan_suspension prefix="rear_right" side_reflect="-1" front_reflect="-1" x_off="${-wheelbase/2}" y_off="${-track_width/2}"/>
"""
    content = pattern_rocker.sub(suspension_macro, content)

    # Now update wheel instantiations. They used to be on left_rocker, right_rocker and base_link.
    # Change them to be on the respective arms.
    # We will replace the wheel instantiation block.
    
    wheel_inst_pattern = re.compile(r'<!-- Front wheels.*?rear_right_wheel_joint"\/>', re.DOTALL)
    
    wheel_inst_old = """  <!-- Front wheels (passive) on suspension rockers. Low lateral friction for skid steering. -->
  <xacro:niihan_wheel name="front_left_wheel"  parent="left_rocker"  x_off="${wheelbase}" y_off="0" hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.1"/>
  <xacro:niihan_wheel name="front_right_wheel" parent="right_rocker" x_off="${wheelbase}" y_off="0" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.1"/>

  <!-- Rear wheels (driven) on base_link. High friction for drive traction. -->
  <xacro:niihan_wheel name="rear_left_wheel"   parent="base_link"  x_off="${-wheelbase/2}" y_off="${track_width/2}"  hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="rear_right_wheel"  parent="base_link"  x_off="${-wheelbase/2}" y_off="${-track_width/2}" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.8"/>"""
    
    wheel_inst_new = """  <!-- Wheels on independent suspension arms -->
  <xacro:niihan_wheel name="front_left_wheel"  parent="front_left_arm"  x_off="${-suspension_arm_length}" y_off="0" hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="front_right_wheel" parent="front_right_arm" x_off="${-suspension_arm_length}" y_off="0" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="rear_left_wheel"   parent="rear_left_arm"  x_off="${suspension_arm_length}" y_off="0"  hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="rear_right_wheel"  parent="rear_right_arm"  x_off="${suspension_arm_length}" y_off="0" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.8"/>
"""
    content = content.replace(wheel_inst_old, wheel_inst_new)

    # Add components: front_compute_box, power_control_box, battery_box, access_panel, rear_dock_contacts
    boxes = """
  <!-- Enclosures and Mass -->
  <link name="front_compute_box">
    <visual>
      <geometry><box size="0.12 0.10 0.05"/></geometry>
      <material name="compute_box_mat"><color rgba="0.2 0.2 0.2 1.0"/></material>
    </visual>
    <xacro:box_inertia mass="1.5" x="0.12" y="0.10" z="0.05"/>
  </link>
  <joint name="front_compute_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_compute_box"/>
    <origin xyz="${chassis_length/2 - 0.1} 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="power_control_box">
    <visual>
      <geometry><box size="0.15 0.12 0.06"/></geometry>
      <material name="power_box_mat"><color rgba="0.3 0.3 0.3 1.0"/></material>
    </visual>
    <xacro:box_inertia mass="2.0" x="0.15" y="0.12" z="0.06"/>
  </link>
  <joint name="power_control_joint" type="fixed">
    <parent link="base_link"/>
    <child link="power_control_box"/>
    <origin xyz="-0.05 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>
  
  <link name="estop_button">
    <visual>
      <geometry><cylinder radius="0.02" length="0.02"/></geometry>
      <material name="estop_red"><color rgba="0.8 0.1 0.1 1.0"/></material>
    </visual>
  </link>
  <joint name="estop_joint" type="fixed">
    <parent link="power_control_box"/>
    <child link="estop_button"/>
    <origin xyz="0 0 0.04" rpy="0 0 0"/>
  </joint>

  <link name="battery_box">
    <visual>
      <geometry><box size="0.26 0.165 0.21"/></geometry>
      <material name="battery_mat"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
    <xacro:box_inertia mass="14.0" x="0.26" y="0.165" z="0.21"/>
  </link>
  <joint name="battery_joint" type="fixed">
    <parent link="base_link"/>
    <child link="battery_box"/>
    <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Access panel -->
  <link name="access_panel">
    <visual>
      <geometry><box size="0.30 0.20 0.01"/></geometry>
      <material name="panel_mat"><color rgba="0.4 0.4 0.4 1.0"/></material>
    </visual>
  </link>
  <joint name="access_panel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="access_panel"/>
    <!-- Note to user: Fixed access panel per requirements. -->
    <origin xyz="0 0 ${chassis_height/2 + 0.005}" rpy="0 0 0"/>
  </joint>

  <link name="rear_dock_contacts">
    <visual>
      <geometry><box size="0.02 0.1 0.02"/></geometry>
      <material name="contacts_mat"><color rgba="0.8 0.8 0.1 1.0"/></material>
    </visual>
  </link>
  <joint name="rear_dock_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_dock_contacts"/>
    <origin xyz="${-chassis_length/2} 0 ${-chassis_height/2 + 0.02}" rpy="0 0 0"/>
  </joint>
"""
    content = content.replace("</robot>", boxes + "\n</robot>")
    
    with open(path, "w") as f:
        f.write(content)

update_properties()
update_chassis()
