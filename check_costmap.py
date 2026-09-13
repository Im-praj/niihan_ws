import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.callback, 10)
        self.first = True

    def callback(self, msg):
        if not self.first:
            return
        self.first = False
        info = msg.info
        lethal = sum(1 for val in msg.data if val == 100)
        inscribed = sum(1 for val in msg.data if val == 99)
        print(f"Global Costmap: {info.width}x{info.height}, lethal(100)={lethal}, inscribed(99)={inscribed}")
        raise SystemExit

rclpy.init()
node = CostmapChecker()
try:
    rclpy.spin(node)
except SystemExit:
    pass
rclpy.shutdown()
