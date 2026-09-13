import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.cli = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetCostmap.Request()

    def send_request(self):
        return self.cli.call_async(self.req)

rclpy.init()
checker = CostmapChecker()
future = checker.send_request()
rclpy.spin_until_future_complete(checker, future)
costmap = future.result().map
meta = costmap.metadata
print(f"Origin: {meta.origin.position.x}, {meta.origin.position.y}")
print(f"Res: {meta.resolution}, W: {meta.size_x}, H: {meta.size_y}")

for goal_y in [0.0, 3.0, 6.0]:
    x_world = 0.0
    y_world = goal_y
    x_idx = int((x_world - meta.origin.position.x) / meta.resolution)
    y_idx = int((y_world - meta.origin.position.y) / meta.resolution)
    if 0 <= x_idx < meta.size_x and 0 <= y_idx < meta.size_y:
        print(f"Cost at (0, {goal_y}): {costmap.data[y_idx * meta.size_x + x_idx]}")
    else:
        print(f"Out of bounds for (0, {goal_y})")

checker.destroy_node()
rclpy.shutdown()
