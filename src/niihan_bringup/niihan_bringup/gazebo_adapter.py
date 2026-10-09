"""Gazebo drive and quality adapters for common hardware contracts."""
from .common_nodes import spin
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, NavSatFix
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data


class GazeboDrive(Node):
    def __init__(self):
        super().__init__('gazebo_drive')
        self.command=self.create_publisher(Twist,'/niihan/gazebo_cmd_vel',10)
        self.create_subscription(Twist,'/niihan/drive/cmd_vel',self.command.publish,10)
        self.cloud=self.create_publisher(PointCloud2,'/niihan/raw/lidar/points',qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/niihan/sensors/unitree_lidar/points',self.lidar,qos_profile_sensor_data)
        self.camera=self.create_publisher(Image,'/niihan/raw/camera/image_raw',qos_profile_sensor_data)
        self.create_subscription(Image,'/niihan/sensors/orbbec/color/image_raw',self.camera.publish,qos_profile_sensor_data)
        self.health=self.create_publisher(Bool,'/niihan/drive/healthy',10)
        self.create_subscription(Odometry,'/niihan/wheel/odom',self.odom,10)
        self.quality=self.create_publisher(String,'/niihan/gnss/quality',10)
        self.gnss=self.create_publisher(NavSatFix,'/niihan/gnss/fix',qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'/niihan/raw/gnss/fix',self.fix,qos_profile_sensor_data)

    def lidar(self,msg):
        # Gazebo sensor scoped names are not URDF TF frame IDs.
        msg.header.frame_id="unitree_l2_link"
        self.cloud.publish(msg)

    def odom(self,msg):
        health=Bool();health.data=True;self.health.publish(health)

    def fix(self,msg):
        msg.header.frame_id='gnss_link'
        msg.position_covariance=[9.0,0.0,0.0,0.0,9.0,0.0,0.0,0.0,16.0]
        msg.position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gnss.publish(msg)
        # Gazebo has no RTK carrier-solution state; never label its NavSatFix as RTK fixed.
        quality=String();quality.data='simulated_fix' if msg.status.status>=0 else 'no_fix';self.quality.publish(quality)


def main(args=None):spin(GazeboDrive,args)
