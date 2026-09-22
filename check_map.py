#!/usr/bin/env python3
"""Check if map topic is publishing.

Uses transient_local QoS to match SLAM Toolbox's /map publisher,
and includes a configurable timeout so it doesn't hang forever.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapChecker(Node):
    def __init__(self, timeout_sec=30.0):
        super().__init__('map_checker')

        # Match SLAM Toolbox / map_server QoS: transient_local + reliable
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos
        )
        self._received = False
        self._timeout = timeout_sec

        # Timeout timer
        self._timer = self.create_timer(timeout_sec, self._on_timeout)

        self.get_logger().info(
            f'Waiting for /map (timeout {timeout_sec}s, QoS=transient_local) ...'
        )

    def map_cb(self, msg):
        self._received = True
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        occ = sum(1 for c in msg.data if c > 50)
        free = sum(1 for c in msg.data if 0 <= c <= 50)
        unk = sum(1 for c in msg.data if c == -1)
        self.get_logger().info(
            f'Map: {w}x{h} @ {res}m/px  '
            f'occupied={occ} free={free} unknown={unk}'
        )

    def _on_timeout(self):
        if not self._received:
            self.get_logger().error(
                f'No /map received after {self._timeout}s. '
                f'Check that slam_toolbox is running and /scan is flowing.'
            )
            rclpy.shutdown()


def main():
    rclpy.init()
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.spin(MapChecker(timeout))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
