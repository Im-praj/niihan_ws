#!/usr/bin/env python3

import os
import sys


def main():
    args = sys.argv[1:]
    has_sim_time = False
    for arg in args:
        if 'use_sim_time' in arg:
            has_sim_time = True
            break

    ros_args = ['--ros-args', '-r', '/cmd_vel:=/niihan/cmd_vel/manual']
    if not has_sim_time:
        ros_args = ['--ros-args', '-p', 'use_sim_time:=true', '-r', '/cmd_vel:=/niihan/cmd_vel/manual']

    os.execvp(
        'ros2',
        [
            'ros2',
            'run',
            'teleop_twist_keyboard',
            'teleop_twist_keyboard',
            *args,
            *ros_args,
        ],
    )
