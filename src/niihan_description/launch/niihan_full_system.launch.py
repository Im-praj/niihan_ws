"""Simulation, both maps, Nav2 and dashboard on a shared ROS clock."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_dir = get_package_share_directory('niihan_description')
    dashboard_dir = get_package_share_directory('niihan_dashboard')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='niihan_construction_site.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('rqt_cam', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_dir, 'launch', 'niihan_gazebo.launch.py',
            )),
            launch_arguments={
                'mapping': 'true',
                'launch_nav2': 'true',
                'use_sim_time': use_sim_time,
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
                'seed': LaunchConfiguration('seed'),
                'rqt_cam': LaunchConfiguration('rqt_cam'),
                'patrol': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                dashboard_dir, 'launch', 'dashboard.launch.py',
            )),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
