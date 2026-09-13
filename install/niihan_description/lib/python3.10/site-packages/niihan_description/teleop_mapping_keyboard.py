#!/usr/bin/env python3

import os
import sys


def main():
    os.execvp(
        'ros2',
        [
            'ros2',
            'run',
            'teleop_twist_keyboard',
            'teleop_twist_keyboard',
            *sys.argv[1:],
        ],
    )