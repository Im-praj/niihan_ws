import rclpy
from rclpy.node import Node
import tf2_ros

class PoseChecker(Node):
    def __init__(self):
        super().__init__('pose_checker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def check(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0))
            print(f"Robot pose in map: {trans.transform.translation.x}, {trans.transform.translation.y}")
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

rclpy.init()
checker = PoseChecker()
import time
for _ in range(10):
    rclpy.spin_once(checker, timeout_sec=0.5)
    if checker.check():
        break
checker.destroy_node()
rclpy.shutdown()
