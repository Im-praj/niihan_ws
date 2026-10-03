#!/usr/bin/env python3

import os
import subprocess
import sys


def main():
    if len(sys.argv) < 2:
        return 2

    terminal_environment = os.environ.copy()
    terminal_environment.pop('LD_LIBRARY_PATH', None)
    terminal_environment.pop('GTK_PATH', None)

    teleop_environment = terminal_environment.copy()
    teleop_environment['LD_LIBRARY_PATH'] = ':'.join([
        '/opt/ros/humble/opt/rviz_ogre_vendor/lib',
        '/opt/ros/humble/lib/x86_64-linux-gnu',
        '/opt/ros/humble/lib',
    ])

    args = sys.argv[1:]
    has_sim_time = False
    for arg in args:
        if 'use_sim_time' in arg:
            has_sim_time = True
            break

    ros_args = ['--ros-args', '-r', '/cmd_vel:=/niihan/cmd_vel/manual']
    if not has_sim_time:
        ros_args = ['--ros-args', '-p', 'use_sim_time:=true', '-r', '/cmd_vel:=/niihan/cmd_vel/manual']

    # GNOME Terminal owns the TTY; teleop keeps the ROS libraries it needs.
    return subprocess.call(
        ['gnome-terminal', '--wait', '--', 'env',
         *[f'{key}={value}' for key, value in teleop_environment.items()],
         *args,
         *ros_args],
        env=terminal_environment,
    )
