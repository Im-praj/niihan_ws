import rclpy
from sensor_msgs.msg import PointCloud2
rclpy.init()
node = rclpy.create_node('field_checker')
def cb(msg):
    print([f.name for f in msg.fields])
    rclpy.shutdown()
node.create_subscription(PointCloud2, '/niihan/sensors/lidar/points', cb, 10)
rclpy.spin(node)
