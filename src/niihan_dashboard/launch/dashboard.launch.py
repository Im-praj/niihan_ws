from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the same clock as TF and Nav2; true for Gazebo.',
    )
    dashboard_node = Node(
        package='niihan_dashboard',
        executable='dashboard_node',
        name='niihan_dashboard_node',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([use_sim_time, dashboard_node])
