"""Mock or Gazebo hardware I/O feeding the identical application stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def backend(context):
    bringup=get_package_share_directory('niihan_bringup')
    description=get_package_share_directory('niihan_description')
    selected=LaunchConfiguration('backend').perform(context)
    clock={'use_sim_time':True}
    if selected=='mock':
        return [Node(package='niihan_bringup',executable='mock_hardware',name='mock_hardware',output='screen',parameters=[clock,{'gnss_mode':LaunchConfiguration('gnss_mode'),**{n:ParameterValue(LaunchConfiguration(n),value_type=float) for n in ['datum_latitude','datum_longitude','datum_altitude']}}]),
                Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',parameters=[clock,{'robot_description':ParameterValue(Command(['xacro ',os.path.join(description,'urdf','niihan.urdf.xacro')]),value_type=str)}])]
    if selected=='gazebo':
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(description,'launch','niihan_gazebo.launch.py')),launch_arguments={'use_sim_time':'true','external_stack':'true','mapping':'false','launch_nav2':'false','vortex_3d':'false','cliff_detect':'false','patrol':'false','rqt_cam':'false','world':LaunchConfiguration('world'),'headless':LaunchConfiguration('headless'),'seed':LaunchConfiguration('seed')}.items()),
                Node(package='niihan_bringup',executable='gazebo_drive',name='gazebo_drive',parameters=[clock])]
    raise RuntimeError('backend must be mock or gazebo')


def generate_launch_description():
    package=get_package_share_directory('niihan_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('backend',default_value='mock'),
        DeclareLaunchArgument('world',default_value='niihan_construction_site.sdf'),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('seed',default_value='42'),
        DeclareLaunchArgument('drive_enabled',default_value='true'),
        DeclareLaunchArgument('gnss_mode',default_value='rtk_fixed'),
        DeclareLaunchArgument('require_gnss',default_value='false'),
        DeclareLaunchArgument('require_cliff',default_value='false'),
        DeclareLaunchArgument('datum_latitude',default_value='12.9716'),
        DeclareLaunchArgument('datum_longitude',default_value='77.5946'),
        DeclareLaunchArgument('datum_altitude',default_value='0.0'),
        OpaqueFunction(function=backend),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(package,'launch','niihan_application.launch.py')),launch_arguments={
            'use_sim_time':'true','drive_enabled':LaunchConfiguration('drive_enabled'),'require_gnss':LaunchConfiguration('require_gnss'),'require_cliff':LaunchConfiguration('require_cliff'),
            'cloud_topic': '/niihan/raw/lidar/points',
            **{n:LaunchConfiguration(n) for n in ['datum_latitude','datum_longitude','datum_altitude']},
        }.items()),
    ])
