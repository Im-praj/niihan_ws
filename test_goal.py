import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

class TestGoal(Node):
    def __init__(self, x, y):
        super().__init__('test_goal')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose
        
        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_cb)
        
    def goal_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            print("Goal rejected")
            sys.exit(1)
        print("Goal accepted")
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.res_cb)
        
    def res_cb(self, future):
        print("Result:", future.result().status)
        sys.exit(0)

rclpy.init()
node = TestGoal(sys.argv[1], sys.argv[2])
rclpy.spin(node)
