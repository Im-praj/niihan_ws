import rclpy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose
rclpy.init()
goal = NavigateToPose.Goal()
wp = Pose()
wp.position.x = 5.0
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.pose = wp
print(f"Goal x: {goal.pose.pose.position.x}")
