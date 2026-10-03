"""Deliver fresh acquisition-stamped scans to SLAM after odometry TF arrives."""
from collections import deque
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class ScanTFGate(Node):
    def __init__(self):
        super().__init__('scan_tf_gate')
        for name, value in [('odom_frame', 'odom'), ('base_frame', 'base_footprint'),
                            ('max_scan_age', 0.5), ('max_wait_wall', 2.0),
                            ('tf_delivery_margin', 0.05)]:
            self.declare_parameter(name, value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_scan_age = self.get_parameter('max_scan_age').value
        self.max_wait_wall = self.get_parameter('max_wait_wall').value
        self.tf_delivery_margin = self.get_parameter('tf_delivery_margin').value
        self.pending = deque(maxlen=5)
        self.last_ros_time = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(LaserScan, '/scan/slam', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.receive, qos_profile_sensor_data)
        self.create_timer(0.02, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        self.pending.append((msg, time.monotonic()))

    def flush(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.pending.clear()
        self.last_ros_time = now
        waiting = deque(maxlen=self.pending.maxlen)
        while self.pending:
            msg, received = self.pending.popleft()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = now - stamp
            if stamp <= 0 or age > self.max_scan_age or time.monotonic() - received > self.max_wait_wall:
                continue
            if age < 0:
                waiting.append((msg, received))
                continue
            try:
                # Preserve the acquisition stamp. The small delivery margin lets
                # SLAM's independent TF subscriber receive this odometry first.
                self.tf_buffer.lookup_transform(self.odom_frame, msg.header.frame_id,
                                                Time.from_msg(msg.header.stamp))
                self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame,
                                                Time(nanoseconds=int((stamp + self.tf_delivery_margin) * 1e9)))
            except TransformException:
                waiting.append((msg, received))
                continue
            self.publisher.publish(msg)
        self.pending = waiting


def main(args=None):
    rclpy.init(args=args)
    node = ScanTFGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
