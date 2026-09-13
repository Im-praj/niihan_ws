#!/usr/bin/env python3
"""Bring up NIIHAN in Gazebo Harmonic: starts gz sim with Sonoma Raceway,
publishes robot_description, starts robot_state_publisher, spawns NIIHAN entity
at track starting line, launches ros_gz_bridge for sensor/control/camera topics,
runs 2D SLAM mapping via slam_toolbox, and starts autonomous raceway patrol controller."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = get_package_share_directory('niihan_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'niihan.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='niihan_patrol_base.world',
        description='Gazebo world file to load (relative to niihan_description/worlds)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo Harmonic in headless mode',
    )

    autonav_arg = DeclareLaunchArgument(
        'autonav',
        default_value='true',
        description='Run autonomous track patrol & obstacle avoidance controller',
    )

    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Run online 2D SLAM mapping via slam_toolbox',
    )

    octomap_arg = DeclareLaunchArgument(
        'octomap',
        default_value='true',
        description='Run 3D OctoMap mapping from the 3D LiDAR',
    )

    rqt_cam_arg = DeclareLaunchArgument(
        'rqt_cam',
        default_value='false',
        description='Launch rqt_image_view for live front camera stream',
    )

    # Set Gazebo resource paths to resolve meshes, models, and ROS package URIs
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_description, '..'),
            ':',
            pkg_description,
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_description, '..'),
            ':',
            pkg_description,
            ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('niihan_description'), 'worlds', LaunchConfiguration('world'),
    ])

    # Gazebo Sim launch via ros_gz_sim: world file must be first positional argument
    gz_args_sub = PythonExpression([
        "'", world_path, " -r' if '", LaunchConfiguration('headless'), "' == 'false' else '", world_path, " -s -r'"
    ])

    log_world_path = LogInfo(
        msg=['[niihan_gazebo] Resolved world_path: ', world_path]
    )
    log_gz_args = LogInfo(
        msg=['[niihan_gazebo] Final gz_args: ', gz_args_sub]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': gz_args_sub,
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn NIIHAN robot entity at starting pose
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'niihan',
            '-topic', '/robot_description',
            '-x', '-8.0',
            '-y', '0.0',
            '-z', '0.02',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ROS <-> Gazebo Harmonic bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Clock & TF
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Actuation and Odometry
            '/niihan/gazebo_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/niihan/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # LiDARs
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/niihan/sensors/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # IMU & GNSS
            '/niihan/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/niihan/gnss/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',

            # Proximity and Bumper
            '/niihan/proximity/front_range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/niihan/proximity/rear_range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/niihan/bumper/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',

            # Panoramic RGB Cameras
            '/niihan/sensors/panoramic/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/panoramic/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/niihan/sensors/panoramic/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/panoramic/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/niihan/sensors/panoramic/rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/panoramic/rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/niihan/sensors/panoramic/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/panoramic/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # PTZ & Thermal & Dock Cameras
            '/niihan/sensors/ptz/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/ptz/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/niihan/sensors/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/sensors/thermal/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/niihan/dock/fiducial/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/niihan/dock/fiducial/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    differential_drive_controller = Node(
        package='niihan_description',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.14,
            'track_width': 0.43,
            'max_wheel_velocity': 10.0,
        }],
    )

    # 2D SLAM mapping node (slam_toolbox)
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
        condition=IfCondition(LaunchConfiguration('mapping')),
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.08,
            'frame_id': 'map',
            'sensor_model/max_range': 25.0,
            'sensor_model/hit': 0.7,
            'sensor_model/miss': 0.4,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
            'filter_ground': False,
            'pointcloud_min_z': -1.0,
            'pointcloud_max_z': 10.0,
            'occupancy_min_z': 0.05,
            'occupancy_max_z': 3.0,
        }],
        remappings=[
            ('cloud_in', '/niihan/sensors/lidar/points'),
        ],
        condition=IfCondition(LaunchConfiguration('octomap')),
    )

    # Autonomous raceway patrol controller (track loop + obstacle avoidance)
    patrol_controller_node = Node(
        package='niihan_description',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('autonav')),
    )

    patrol_controller_timer = TimerAction(
        period=4.0,
        actions=[patrol_controller_node],
    )

    # Optional live camera viewer
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/niihan/sensors/panoramic/front/image_raw'],
        condition=IfCondition(LaunchConfiguration('rqt_cam')),
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        world_arg,
        headless_arg,
        autonav_arg,
        mapping_arg,
        octomap_arg,
        rqt_cam_arg,
        log_world_path,
        log_gz_args,
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        bridge_node,
        differential_drive_controller,
        slam_toolbox_node,
        octomap_node,
        patrol_controller_timer,
        rqt_image_view_node,
    ])
