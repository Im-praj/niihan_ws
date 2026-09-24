with open('urdf/niihan_sensors_mast.xacro', 'r') as f:
    content = f.read()

# We want to insert the new sensors before the closing </robot> tag
new_sensors = """
  <!-- ============ REAR SENSOR CLUSTER ============ -->
  <link name="antenna_link">
    <visual>
      <geometry><cylinder radius="0.005" length="0.2"/></geometry>
      <material name="antenna_mat"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
  </link>
  <joint name="antenna_joint" type="fixed">
    <parent link="mast_link"/>
    <child link="antenna_link"/>
    <origin xyz="-0.05 0 ${mast_height}" rpy="0 0 0"/>
  </joint>

  <link name="speaker_link">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
      <material name="speaker_mat"><color rgba="0.2 0.2 0.2 1.0"/></material>
    </visual>
  </link>
  <joint name="speaker_joint" type="fixed">
    <parent link="mast_link"/>
    <child link="speaker_link"/>
    <origin xyz="-0.05 0 ${mast_height - 0.05}" rpy="0 0 0"/>
  </joint>

  <link name="unitree_l2_link">
    <visual>
      <geometry><cylinder radius="0.0375" length="0.065"/></geometry>
      <material name="unitree_mat"><color rgba="0.1 0.1 0.1 1.0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.0375" length="0.065"/></geometry>
    </collision>
    <xacro:cylinder_inertia mass="0.23" r="0.0375" h="0.065"/>
  </link>
  <joint name="unitree_l2_joint" type="fixed">
    <parent link="mast_link"/>
    <child link="unitree_l2_link"/>
    <origin xyz="-0.1 0 ${mast_height - 0.1}" rpy="0 0 0"/>
  </joint>
  <gazebo reference="unitree_l2_link">
    <sensor type="gpu_lidar" name="unitree_l2_sensor">
      <always_on>1</always_on>
      <update_rate>10.0</update_rate>
      <topic>/niihan/sensors/unitree_lidar</topic>
      <gz:frame_id>unitree_l2_link</gz:frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>560</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>140</samples>
            <resolution>1</resolution>
            <min_angle>-0.12217</min_angle>
            <max_angle>1.5708</max_angle>
          </vertical>
        </scan>
        <range><min>0.05</min><max>30.0</max><resolution>0.0045</resolution></range>
      </lidar>
    </sensor>
  </gazebo>

  <link name="orbbec_astra2_link">
    <visual>
      <geometry><box size="0.025 0.0648 0.0186"/></geometry>
      <material name="orbbec_mat"><color rgba="0.2 0.2 0.2 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="0.025 0.0648 0.0186"/></geometry>
    </collision>
    <xacro:box_inertia mass="0.038" x="0.025" y="0.0648" z="0.0186"/>
  </link>
  <joint name="orbbec_astra2_joint" type="fixed">
    <parent link="mast_link"/>
    <child link="orbbec_astra2_link"/>
    <origin xyz="-0.1 0 ${mast_height - 0.18}" rpy="0 0 3.14159"/>
  </joint>
  
  <link name="orbbec_color_optical_frame"/>
  <joint name="orbbec_color_optical_joint" type="fixed">
    <parent link="orbbec_astra2_link"/>
    <child link="orbbec_color_optical_frame"/>
    <origin xyz="0 0.015 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>

  <link name="orbbec_depth_optical_frame"/>
  <joint name="orbbec_depth_optical_joint" type="fixed">
    <parent link="orbbec_astra2_link"/>
    <child link="orbbec_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>

  <gazebo reference="orbbec_color_optical_frame">
    <sensor type="camera" name="orbbec_color_sensor">
      <always_on>1</always_on>
      <update_rate>30.0</update_rate>
      <topic>/niihan/sensors/orbbec/color/image_raw</topic>
      <gz:frame_id>orbbec_color_optical_frame</gz:frame_id>
      <camera name="orbbec_color">
        <horizontal_fov>1.303</horizontal_fov>
        <image>
          <width>1920</width>
          <height>1080</height>
          <format>R8G8B8</format>
        </image>
        <clip><near>0.1</near><far>100</far></clip>
      </camera>
    </sensor>
  </gazebo>

  <gazebo reference="orbbec_depth_optical_frame">
    <sensor type="depth_camera" name="orbbec_depth_sensor">
      <always_on>1</always_on>
      <update_rate>30.0</update_rate>
      <topic>/niihan/sensors/orbbec/depth/image_raw</topic>
      <gz:frame_id>orbbec_depth_optical_frame</gz:frame_id>
      <camera name="orbbec_depth">
        <horizontal_fov>1.015</horizontal_fov>
        <image>
          <width>1600</width>
          <height>1200</height>
        </image>
        <clip><near>0.6</near><far>8.0</far></clip>
      </camera>
    </sensor>
  </gazebo>
</robot>
"""

content = content.replace("</robot>", new_sensors)
with open('urdf/niihan_sensors_mast.xacro', 'w') as f:
    f.write(content)

