"""One application stack shared by physical and simulated I/O."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description=get_package_share_directory('niihan_description')
    bringup=get_package_share_directory('niihan_bringup')
    dashboard=get_package_share_directory('niihan_dashboard')
    clock={'use_sim_time':LaunchConfiguration('use_sim_time')}
    def node(package,executable,name,params=None,**kwargs):
        return Node(package=package,executable=executable,name=name,output='screen',parameters=[clock]+(params or []),**kwargs)
    def include(path,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments=args.items())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('drive_enabled',default_value='false'),
        DeclareLaunchArgument('require_gnss',default_value='false'),
        DeclareLaunchArgument('require_cliff',default_value='false'),
        DeclareLaunchArgument('vortex_3d',default_value='true'),
        DeclareLaunchArgument('cloud_topic',default_value='/niihan/raw/lidar/points'),
        DeclareLaunchArgument('camera_topic',default_value='/niihan/raw/camera/image_raw'),
        DeclareLaunchArgument('relay_camera',default_value='true'),
        DeclareLaunchArgument('datum_latitude',default_value='12.9716'),
        DeclareLaunchArgument('datum_longitude',default_value='77.5946'),
        DeclareLaunchArgument('datum_altitude',default_value='0.0'),
        node('niihan_bringup','sensor_adapter','sensor_adapter',[{'cloud_topic':LaunchConfiguration('cloud_topic'),'camera_topic':LaunchConfiguration('camera_topic'),'relay_camera':ParameterValue(LaunchConfiguration('relay_camera'),value_type=bool)}]),
        node('robot_localization','ekf_node','ekf_local',[os.path.join(bringup,'config','localization.yaml')],remappings=[('odometry/filtered','/odom')]),
        node('niihan_bringup','gnss_monitor','gnss_monitor',[{name:ParameterValue(LaunchConfiguration(name),value_type=float) for name in ['datum_latitude','datum_longitude','datum_altitude']}]),
        node('robot_localization','ekf_node','ekf_gnss',[os.path.join(bringup,'config','localization.yaml')],remappings=[('odometry/filtered','/niihan/gnss/filtered')]),
        node('niihan_description','command_arbiter','command_arbiter'),
        node('niihan_bringup','health_supervisor','health_supervisor',[{'require_gnss':ParameterValue(LaunchConfiguration('require_gnss'),value_type=bool),'require_cliff':ParameterValue(LaunchConfiguration('require_cliff'),value_type=bool)}]),
        node('niihan_bringup','motion_gateway','motion_gateway',[{'drive_enabled':ParameterValue(LaunchConfiguration('drive_enabled'),value_type=bool)}]),
        node('niihan_description','scan_tf_gate','scan_tf_gate'),
        node('slam_toolbox','async_slam_toolbox_node','slam_toolbox',[os.path.join(description,'config','slam_toolbox_params.yaml')]),
        node('niihan_description','cliff_detector','cliff_detector'),
        node('niihan_description','vortex_3d_mapper','vortex_3d_mapper',[{'max_points':250000,'voxel_resolution':0.15,'publish_rate':0.5}],condition=IfCondition(LaunchConfiguration('vortex_3d'))),
        include(os.path.join(description,'launch','niihan_navigation.launch.py'),{'use_sim_time':LaunchConfiguration('use_sim_time')}),
        include(os.path.join(dashboard,'launch','dashboard.launch.py'),{'use_sim_time':LaunchConfiguration('use_sim_time')}),
    ])
