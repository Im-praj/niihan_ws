import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomChecker(Node):
    def __init__(self):
        super().__init__('odom_checker')
        self.sub = self.create_subscription(Odometry, '/niihan/odom', self.callback, 10)
        self.odom = None

    def callback(self, msg):
        self.odom = msg

rclpy.init()
node = OdomChecker()
while rclpy.ok() and node.odom is None:
    rclpy.spin_once(node)

pose = node.odom.pose.pose.position
print(f"Odom position: {pose.x}, {pose.y}")
node.destroy_node()
rclpy.shutdown()
