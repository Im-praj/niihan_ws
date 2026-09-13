#!/usr/bin/env python3
"""Launch manual teleoperation while building a persistent 2D map."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('niihan_description')
    package_prefix = get_package_prefix('niihan_description')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='niihan_patrol_base.world',
        description='Gazebo world file in niihan_description/worlds',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without its GUI',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 while mapping',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('niihan_description'),
                'launch',
                'niihan_gazebo.launch.py',
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
        }.items(),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': True},
        ],
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix=os.path.join(
            package_prefix, 'lib', 'niihan_description', 'teleop_mapping_terminal'),
        remappings=[('cmd_vel', '/niihan/cmd_vel')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_share, 'rviz', 'niihan_mapping.rviz')],
        prefix='env -u GTK_PATH',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        rviz_arg,
        gazebo_launch,
        slam_toolbox_node,
        teleop_node,
        rviz_node,
    ])