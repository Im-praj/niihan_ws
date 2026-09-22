import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    niihan_desc_dir = get_package_share_directory('niihan_description')
    niihan_dashboard_dir = get_package_share_directory('niihan_dashboard')

    # Include the main gazebo/system launch file from niihan_description
    niihan_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(niihan_desc_dir, 'launch', 'niihan_gazebo.launch.py')
        ),
        launch_arguments={
            'mapping': 'true',      # Enable mapping/slam by default
            'launch_nav2': 'true',  # Enable Nav2 by default
            'use_sim_time': 'true', # Enable simulation time
            'patrol': 'true',       # Enable autonomous patrol logic
        }.items()
    )

    # Include the dashboard launch file
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(niihan_dashboard_dir, 'launch', 'dashboard.launch.py')
        )
    )

    return LaunchDescription([
        niihan_gazebo_launch,
        dashboard_launch
    ])
