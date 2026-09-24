import re

with open('src/niihan_description/launch/niihan_full_system.launch.py', 'r') as f:
    text = f.read()

# Remove arguments
text = re.sub(r'    \w+_arg = DeclareLaunchArgument\(\s*\'(patrol|mapping|octomap|vortex_3d)\',.*?    \)\n\n', '', text, flags=re.DOTALL)
text = re.sub(r'        (patrol_arg|mapping_arg|octomap_arg|vortex_3d_arg),\n', '', text)

# Remove nodes
text = re.sub(r'    # 2D SLAM mapping node \(slam_toolbox\).*?    \)\n\n', '', text, flags=re.DOTALL)
text = re.sub(r'    # Lifecycle manager to activate slam_toolbox.*?    \)\n\n', '', text, flags=re.DOTALL)
text = re.sub(r'    octomap_node = Node\(.*?    \)\n\n', '', text, flags=re.DOTALL)
text = re.sub(r'    # Vortex 3D Point Cloud Mapper \(replaces OctoMap\).*?    \)\n\n', '', text, flags=re.DOTALL)
text = re.sub(r'        slam_toolbox_node,\n', '', text)
text = re.sub(r'        octomap_node,\n', '', text)
text = re.sub(r'        vortex_3d_mapper_node,\n', '', text)
text = re.sub(r'        patrol_controller_timer,\n', '', text)

# Add kiss_icp and glim and dashboard
kiss_glim = """
    # KISS-ICP Odometry
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kiss_icp'),
                'launch',
                'odometry.launch.py'
            ])
        ),
        launch_arguments={
            'topic': '/niihan/sensors/lidar/points',
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'publish_odom_tf': 'true',
        }.items()
    )

    # GLIM 3D SLAM
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_rosnode',
        output='screen',
        remappings=[
            ('/points', '/niihan/sensors/lidar/points'),
            ('/imu', '/niihan/imu/data')
        ],
        parameters=[{
            'use_sim_time': True,
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
        }]
    )

    # Dashboard Launch
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('niihan_dashboard'),
                'launch',
                'dashboard.launch.py'
            ])
        )
    )
"""

text = text.replace('    nav2_params_file =', kiss_glim + '\n    nav2_params_file =')

text = text.replace('        nav2_group,', '        nav2_group,\n        kiss_icp_launch,\n        glim_node,\n        dashboard_launch,')

with open('src/niihan_description/launch/niihan_full_system.launch.py', 'w') as f:
    f.write(text)
