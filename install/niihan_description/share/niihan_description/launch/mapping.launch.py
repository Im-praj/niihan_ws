#!/usr/bin/env python3
"""Master Bringup for NIIHAN:
1. Launches Gazebo Harmonic simulation with the corridor world & moving pedestrian obstacle.
2. Publishes URDF robot description & spawns NIIHAN.
3. Bridges all sensor and control topics (2D/3D LiDAR, IMU, cameras, odom, cmd_vel).
4. Runs 2D SLAM mapping via slam_toolbox (generates /map).
5. Runs 3D terrain mapping via octomap_server (generates 3D voxel grid /octomap_point_cloud_centers).
6. Runs Autonomous Obstacle Avoidance Patrol Controller (steers around moving & static obstacles) - default false.
7. Launches RViz2 visualization with pre-configured 2D and 3D map views.
8. Runs Nav2.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = get_package_share_directory('niihan_description')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='niihan_patrol_base.world',
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
        default_value='true',
        description='Launch autonomous obstacle avoidance patrol controller (default: true)',
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 stack (default: true)',
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
            'nav2': 'false',
            'octomap': 'false',
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
            {'use_sim_time': True}
        ],
    )

    # 3. Vortex 3D Point Cloud Mapper (replaces OctoMap)
    vortex_3d_mapper_node = Node(
        package='niihan_description',
        executable='vortex_3d_mapper',
        name='vortex_3d_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'voxel_resolution': 0.05,
            'max_range': 25.0,
            'min_range': 0.5,
            'publish_rate': 1.0,
            'save_interval': 30.0,
            'map_frame': 'map',
            'max_points': 5000000,
            'downsample_resolution': 0.15,
        }],
    )

    # 4. Autonomous Obstacle Avoidance Patrol Controller
    patrol_controller_node = Node(
        package='niihan_description',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('autonav')),
    )
    delayed_patrol_controller = TimerAction(
        period=8.0,
        actions=[patrol_controller_node],
    )

    # 5. Nav2 Stack
    nav2_params_file = os.path.join(pkg_description, 'config', 'nav2_params.yaml')
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/niihan/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # 6. RViz2 Visualizer
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'niihan_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        rviz_arg,
        autonav_arg,
        nav2_arg,
        nav2_launch,
        gazebo_launch,
        slam_toolbox_node,
        vortex_3d_mapper_node,
        delayed_patrol_controller,
        rviz_node,
    ])
