import re

with open('launch/niihan_gazebo.launch.py', 'r') as f:
    content = f.read()

# Add imports
content = content.replace('from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction',
                          'from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction, GroupAction')
content = content.replace('from launch_ros.actions import Node',
                          'from launch_ros.actions import Node, SetRemap')

# Add nav2_arg
nav2_arg_str = """
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Run Nav2 stack',
    )
"""
content = content.replace("    rqt_cam_arg = DeclareLaunchArgument(", nav2_arg_str + "    rqt_cam_arg = DeclareLaunchArgument(")

# Add Nav2 GroupAction
nav2_str = """
    nav2_params_file = os.path.join(pkg_description, 'config', 'nav2_params.yaml')
    nav2_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('nav2')),
        actions=[
            SetRemap(src='/cmd_vel', dst='/niihan/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )
"""
content = content.replace("    return LaunchDescription([", nav2_str + "\n    return LaunchDescription([")

# Add to LaunchDescription
content = content.replace("        mapping_arg,", "        mapping_arg,\n        nav2_arg,")
content = content.replace("        slam_toolbox_node,", "        slam_toolbox_node,\n        nav2_group,")

with open('launch/niihan_gazebo.launch.py', 'w') as f:
    f.write(content)
