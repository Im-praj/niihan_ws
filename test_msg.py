import rclpy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose
goal = NavigateToPose.Goal()
wp = Pose()
wp.position.x = 1.2
wp.position.y = 3.4
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.pose = wp
print(goal.pose.pose.position.x)
