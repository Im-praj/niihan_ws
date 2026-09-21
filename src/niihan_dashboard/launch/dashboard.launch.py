import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('niihan_dashboard')

    dashboard_node = Node(
        package='niihan_dashboard',
        executable='dashboard_node',
        name='niihan_dashboard_node',
        output='screen'
    )

    return LaunchDescription([
        dashboard_node
    ])
