import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import sys

class TestNav(Node):
    def __init__(self):
        super().__init__('test_nav')
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('waiting for action server...')

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)
        self.get_logger().info(f'Sending goal {x}, {y}')
        return self.cli.send_goal_async(goal)

rclpy.init()
node = TestNav()
# First goal to 0, 3 (Waypoint 1 in patrol)
future = node.send_goal(0.0, 3.0, 1.57)
rclpy.spin_until_future_complete(node, future)
goal_handle = future.result()
if not goal_handle.accepted:
    print('Goal rejected!')
    sys.exit(1)
print('Goal accepted!')
res_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, res_future)
status = res_future.result().status
print(f'Goal status: {status}')
if status == 4:
    print('Success!')
node.destroy_node()
rclpy.shutdown()
