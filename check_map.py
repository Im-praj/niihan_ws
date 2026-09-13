import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback, 10)
        self.map_msg = None

    def callback(self, msg):
        self.map_msg = msg

rclpy.init()
checker = MapChecker()
while rclpy.ok() and checker.map_msg is None:
    rclpy.spin_once(checker)

info = checker.map_msg.info
print(f"Origin: {info.origin.position.x}, {info.origin.position.y}")
print(f"Res: {info.resolution}, W: {info.width}, H: {info.height}")
checker.destroy_node()
rclpy.shutdown()
