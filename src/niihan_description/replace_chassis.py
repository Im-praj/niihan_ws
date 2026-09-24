import re

with open('urdf/niihan_chassis.xacro', 'r') as f:
    content = f.read()

# Replace the rocker macro and instances, and wheels
# We'll use regex to find from <xacro:macro name="niihan_rocker" to the end of the file (before </robot>)
pattern = r'<xacro:macro name="niihan_rocker".*?</robot>'
replacement = """<xacro:macro name="niihan_suspension" params="prefix side_reflect front_reflect x_off y_off">
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

  <xacro:macro name="niihan_wheel" params="name parent x_off y_off hub_y hub_offset mu1:=1.0 mu2:=1.0">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
        <material name="${name}_rubber_black"><color rgba="0.05 0.05 0.05 1.0"/></material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      </collision>
      <visual>
        <origin xyz="0 ${hub_y} 0" rpy="${pi/2} 0 0"/>
        <geometry><cylinder radius="0.075" length="0.018"/></geometry>
        <material name="${name}_hub_outer"><color rgba="0.10 0.11 0.12 1.0"/></material>
      </visual>
      <visual>
        <origin xyz="0 ${hub_y + hub_offset} 0" rpy="${pi/2} 0 0"/>
        <geometry><cylinder radius="0.035" length="0.022"/></geometry>
        <material name="${name}_hub_center"><color rgba="0.28 0.29 0.30 1.0"/></material>
      </visual>
      <visual>
        <origin xyz="0 ${hub_y + 2*hub_offset} 0" rpy="${pi/2} 0 0"/>
        <geometry><cylinder radius="0.012" length="0.025"/></geometry>
        <material name="${name}_hub_cap"><color rgba="0.06 0.07 0.08 1.0"/></material>
      </visual>
      <xacro:cylinder_inertia mass="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
    </link>
    <joint name="${name}_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${x_off} ${y_off} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.2" friction="0.1"/>
      <limit effort="${wheel_effort}" velocity="${wheel_velocity}"/>
    </joint>
    <gazebo reference="${name}">
      <material>Gazebo/Black</material>
      <mu1>${mu1}</mu1>
      <mu2>${mu2}</mu2>
      <fdir1>1 0 0</fdir1>
      <kp>10000000.0</kp>
      <kd>1.0</kd>
      <minDepth>0.001</minDepth>
    </gazebo>
  </xacro:macro>

  <xacro:niihan_wheel name="front_left_wheel"  parent="front_left_arm"  x_off="${-suspension_arm_length}" y_off="0" hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="front_right_wheel" parent="front_right_arm" x_off="${-suspension_arm_length}" y_off="0" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="rear_left_wheel"   parent="rear_left_arm"   x_off="${suspension_arm_length}"  y_off="0" hub_y="${wheel_width/2}"  hub_offset="0.01" mu1="0.8" mu2="0.8"/>
  <xacro:niihan_wheel name="rear_right_wheel"  parent="rear_right_arm"  x_off="${suspension_arm_length}"  y_off="0" hub_y="${-wheel_width/2}" hub_offset="-0.01" mu1="0.8" mu2="0.8"/>

  <link name="front_compute_box">
    <visual><geometry><box size="0.12 0.10 0.05"/></geometry><material name="compute_box_mat"><color rgba="0.2 0.2 0.2 1.0"/></material></visual>
    <xacro:box_inertia mass="${front_compute_mass}" x="0.12" y="0.10" z="0.05"/>
  </link>
  <joint name="front_compute_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_compute_box"/>
    <origin xyz="${chassis_length/2 - 0.1} 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="power_control_box">
    <visual><geometry><box size="0.15 0.12 0.06"/></geometry><material name="power_box_mat"><color rgba="0.3 0.3 0.3 1.0"/></material></visual>
    <xacro:box_inertia mass="${power_control_mass}" x="0.15" y="0.12" z="0.06"/>
  </link>
  <joint name="power_control_joint" type="fixed">
    <parent link="base_link"/>
    <child link="power_control_box"/>
    <origin xyz="-0.05 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>
  
  <link name="estop_button">
    <visual><geometry><cylinder radius="0.02" length="0.02"/></geometry><material name="estop_red"><color rgba="0.8 0.1 0.1 1.0"/></material></visual>
  </link>
  <joint name="estop_joint" type="fixed">
    <parent link="power_control_box"/>
    <child link="estop_button"/>
    <origin xyz="0 0 0.04" rpy="0 0 0"/>
  </joint>

  <link name="battery_box">
    <visual><geometry><box size="${battery_size_x} ${battery_size_y} ${battery_size_z}"/></geometry><material name="battery_mat"><color rgba="0.1 0.1 0.1 1.0"/></material></visual>
    <xacro:box_inertia mass="${battery_mass}" x="${battery_size_x}" y="${battery_size_y}" z="${battery_size_z}"/>
  </link>
  <joint name="battery_joint" type="fixed">
    <parent link="base_link"/>
    <child link="battery_box"/>
    <!-- Place battery low and rear -->
    <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
  </joint>

  <link name="access_panel">
    <visual><geometry><box size="0.30 0.20 0.01"/></geometry><material name="panel_mat"><color rgba="0.4 0.4 0.4 1.0"/></material></visual>
  </link>
  <joint name="access_panel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="access_panel"/>
    <origin xyz="0 0 ${chassis_height/2 + 0.005}" rpy="0 0 0"/>
  </joint>

  <link name="rear_dock_contacts">
    <visual><geometry><box size="0.02 0.1 0.02"/></geometry><material name="contacts_mat"><color rgba="0.8 0.8 0.1 1.0"/></material></visual>
  </link>
  <joint name="rear_dock_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_dock_contacts"/>
    <origin xyz="${-chassis_length/2} 0 ${-chassis_height/2 + 0.02}" rpy="0 0 0"/>
  </joint>

</robot>
"""

new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)
with open('urdf/niihan_chassis.xacro', 'w') as f:
    f.write(new_content)
