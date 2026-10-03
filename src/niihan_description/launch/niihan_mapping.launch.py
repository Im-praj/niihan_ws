#!/usr/bin/env python3
"""Master Bringup for NIIHAN:
1. Launches Gazebo Harmonic simulation with the corridor world & moving pedestrian obstacle.
2. Publishes URDF robot description & spawns NIIHAN.
3. Bridges all sensor and control topics (2D/3D LiDAR, IMU, cameras, odom, cmd_vel).
4. Runs 2D SLAM mapping via slam_toolbox (generates /map).
5. Runs Vortex 3D point cloud mapper (accumulates 3D LiDAR into a voxel-filtered map).
6. Runs Cliff Detector (analyses 3D LiDAR for no-ground areas → virtual obstacles for Nav2).
7. Runs Waypoint Patrol Controller with user-defined A/B/C/D patrol points.
8. Launches RViz2 visualization with pre-configured 2D and 3D map views.
9. Runs Nav2 with cliff_scan as an additional obstacle source.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = get_package_share_directory('niihan_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the same ROS clock for simulation, mapping and navigation',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='niihan_construction_site.sdf',
        description='Gazebo world file to load (relative to niihan_description/worlds)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (default: false)',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualization (default: true)',
    )

    autonav_arg = DeclareLaunchArgument(
        'autonav',
        default_value='false',
        description='Launch waypoint patrol controller (default: false)',
    )

    preset_file_arg = DeclareLaunchArgument(
        'preset_file',
        default_value='',
        description='Optional YAML waypoint preset loaded by waypoint_patrol',
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 stack (default: true)',
    )

    vortex_3d_arg = DeclareLaunchArgument(
        'vortex_3d',
        default_value='true',
        description='Launch Vortex 3D point cloud mapper (default: true)',
    )

    cliff_detect_arg = DeclareLaunchArgument(
        'cliff_detect',
        default_value='true',
        description='Launch cliff/no-ground detector (default: true)',
    )

    # 1. Gazebo Harmonic Simulation & Robot Bringup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_description,
                'launch',
                'niihan_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'mapping': 'false',
            'patrol': 'false',
            'launch_nav2': 'false',
            'octomap': 'false',
            'vortex_3d': 'false',
            'cliff_detect': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # 2. 2D Mapping: slam_toolbox (Online Async SLAM)
    slam_params_file = os.path.join(pkg_description, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # 3. Vortex 3D Point Cloud Mapper (replaces OctoMap)
    vortex_3d_mapper_node = Node(
        package='niihan_description',
        executable='vortex_3d_mapper',
        name='vortex_3d_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'voxel_resolution': 0.05,
            'max_range': 25.0,
            'min_range': 0.5,
            'publish_rate': 1.0,
            'save_interval': 30.0,
            'map_frame': 'map',
            'max_points': 5000000,
            'downsample_resolution': 0.15,
        }],
        condition=IfCondition(LaunchConfiguration('vortex_3d')),
    )

    # OLD: OctoMap server (kept commented out for reference)
    # octomap_server_node = Node(
    #     package='octomap_server',
    #     executable='octomap_server_node',
    #     name='octomap_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'resolution': 0.08,
    #         'frame_id': 'map',
    #         'sensor_model/max_range': 25.0,
    #         ...
    #     }],
    #     remappings=[('cloud_in', '/niihan/sensors/lidar/points')],
    # )

    # 4. Cliff / No-Ground Detector
    cliff_detector_node = Node(
        package='niihan_description',
        executable='cliff_detector',
        name='cliff_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ground_height_threshold': 0.15,
            'vertical_fov_min': -0.12217,
            'vertical_fov_max': 0.90757,
            'robot_exclusion_radius': 0.45,
            'cliff_check_radius': 6.0,
            'cell_size': 0.5,
            'min_ground_points': 3,
            'scan_range': 6.0,
            'publish_rate': 5.0,
            'robot_frame': 'base_footprint',
            'lidar_topic': '/niihan/sensors/lidar/points',
        }],
        condition=IfCondition(LaunchConfiguration('cliff_detect')),
    )

    # 5. Waypoint Patrol Controller (user-defined A/B/C/D patrol points)
    waypoint_patrol_node = Node(
        package='niihan_description',
        executable='waypoint_patrol',
        name='waypoint_patrol',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': False,
            'preset_file': LaunchConfiguration('preset_file'),
            'wait_duration': 3.0,
            'waypoint_tolerance': 0.5,
            'obstacle_stop_distance': 0.6,
            'patrol_mode': 'loop',
            'auto_resume_interval': 5.0,
        }],
        condition=IfCondition(LaunchConfiguration('autonav')),
    )
    delayed_waypoint_patrol = TimerAction(
        period=2.0,
        actions=[waypoint_patrol_node],
    )

    # 6. Nav2 Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'niihan_navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # 7. RViz2 Visualizer
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'niihan_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        headless_arg,
        rviz_arg,
        autonav_arg,
        preset_file_arg,
        nav2_arg,
        vortex_3d_arg,
        cliff_detect_arg,
        gazebo_launch,
        slam_toolbox_node,
        vortex_3d_mapper_node,
        cliff_detector_node,
        nav2_launch,
        delayed_waypoint_patrol,
        rviz_node,
    ])
