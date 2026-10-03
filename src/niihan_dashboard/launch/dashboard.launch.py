from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Must match mapping and navigation clock',
        ),
        DeclareLaunchArgument('bind_host', default_value='127.0.0.1'),
        Node(
            package='niihan_dashboard',
            executable='dashboard_node',
            name='niihan_dashboard_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'bind_host': LaunchConfiguration('bind_host')}],
        ),
    ])
