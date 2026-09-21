#!/usr/bin/env python3
"""Convert robot velocity commands into front wheel velocity commands."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from niihan_description.differential_drive import twist_to_wheel_velocities


class DifferentialDriveController(Node):
    """Own the front-only differential-drive command interface."""

    def __init__(self):
        super().__init__('differential_drive_controller')
        self.declare_parameter('wheel_radius', 0.14)
        self.declare_parameter('track_width', 0.43)
        self.declare_parameter('max_wheel_velocity', 10.0)
        self.declare_parameter('gazebo_topic', '/niihan/gazebo_cmd_vel')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.max_wheel_velocity = self.get_parameter('max_wheel_velocity').value

        self.fl_pub = self.create_publisher(Float64, '/niihan/front_left_wheel_velocity', 10)
        self.fr_pub = self.create_publisher(Float64, '/niihan/front_right_wheel_velocity', 10)
        self.rl_pub = self.create_publisher(Float64, '/niihan/rear_left_wheel_velocity', 10)
        self.rr_pub = self.create_publisher(Float64, '/niihan/rear_right_wheel_velocity', 10)
        self.gazebo_pub = self.create_publisher(
            Twist, self.get_parameter('gazebo_topic').value, 10)
        self.create_subscription(Twist, '/niihan/cmd_vel', self.command_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)

    def command_callback(self, command: Twist):
        left, right = twist_to_wheel_velocities(
            command.linear.x,
            command.angular.z,
            self.wheel_radius,
            self.track_width,
            self.max_wheel_velocity,
        )

        left_message = Float64()
        left_message.data = left
        right_message = Float64()
        right_message.data = right
        self.fl_pub.publish(left_message)
        self.rl_pub.publish(left_message)
        self.fr_pub.publish(right_message)
        self.rr_pub.publish(right_message)

        # Calculate RPM
        import math
        left_rpm = (left * 60) / (2 * math.pi)
        right_rpm = (right * 60) / (2 * math.pi)

        # Log for debugging
        self.get_logger().debug(f'L: {left_rpm:.1f} RPM, R: {right_rpm:.1f} RPM')

        limited_command = Twist()
        limited_command.linear.x = self.wheel_radius * (left + right) / 2.0
        limited_command.angular.z = self.wheel_radius * (right - left) / self.track_width
        self.gazebo_pub.publish(limited_command)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()