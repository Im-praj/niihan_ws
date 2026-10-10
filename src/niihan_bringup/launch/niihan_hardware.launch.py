"""Physical I/O only; missing devices remain faults, never become mock data."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def vendor_drivers(context):
    if LaunchConfiguration('start_vendor_drivers').perform(context).lower()!='true':return []
    camera=LaunchConfiguration('camera_model').perform(context)
    if camera not in ['astra2','femto_mega']:raise RuntimeError('camera_model must be astra2 or femto_mega')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('unitree_lidar_ros2'),'launch','launch.py'))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('orbbec_camera'),'launch',camera+'.launch.py')))]


def generate_launch_description():
    description=get_package_share_directory('niihan_description');bringup=get_package_share_directory('niihan_bringup')
    clock={'use_sim_time':False}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('drive_enabled',default_value='false'),
        DeclareLaunchArgument('hardware_calibrated',default_value='false'),
        DeclareLaunchArgument('wheel_radius',default_value='0.1'),
        DeclareLaunchArgument('track_width',default_value='0.525'),
        DeclareLaunchArgument('ticks_per_revolution',default_value='0'),
        DeclareLaunchArgument('stm32_port',default_value='/dev/niihan-stm32'),
        DeclareLaunchArgument('gnss_port',default_value='/dev/niihan-gnss'),
        DeclareLaunchArgument('start_vendor_drivers',default_value='false'),
        DeclareLaunchArgument('camera_model',default_value='astra2'),
        DeclareLaunchArgument('cloud_topic',default_value='/unilidar/cloud'),
        DeclareLaunchArgument('camera_topic',default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('require_gnss',default_value='false'),
        DeclareLaunchArgument('require_cliff',default_value='false'),
        DeclareLaunchArgument('datum_latitude',default_value='12.9716'),
        DeclareLaunchArgument('datum_longitude',default_value='77.5946'),
        DeclareLaunchArgument('datum_altitude',default_value='0.0'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',parameters=[clock,{'robot_description':ParameterValue(Command(['xacro ',os.path.join(description,'urdf','niihan.urdf.xacro')]),value_type=str)}]),
        Node(package='niihan_bringup',executable='stm32_bridge',name='stm32_bridge',output='screen',parameters=[clock,{'port':LaunchConfiguration('stm32_port'),'hardware_calibrated':ParameterValue(LaunchConfiguration('hardware_calibrated'),value_type=bool),'wheel_radius':ParameterValue(LaunchConfiguration('wheel_radius'),value_type=float),'track_width':ParameterValue(LaunchConfiguration('track_width'),value_type=float),'ticks_per_revolution':ParameterValue(LaunchConfiguration('ticks_per_revolution'),value_type=int)}]),
        Node(package='niihan_bringup',executable='f9p_driver',name='f9p_driver',output='screen',parameters=[clock,{'port':LaunchConfiguration('gnss_port')}]),
        Node(package='niihan_bringup',executable='bno085_driver',name='bno085_driver',output='screen',parameters=[clock]),
        OpaqueFunction(function=vendor_drivers),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup,'launch','niihan_application.launch.py')),launch_arguments=({n:LaunchConfiguration(n) for n in ['drive_enabled','cloud_topic','camera_topic','require_gnss','require_cliff','datum_latitude','datum_longitude','datum_altitude']} | {'use_sim_time':'false'}).items()),
    ])
