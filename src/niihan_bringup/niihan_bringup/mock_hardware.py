"""Deterministic hardware-contract simulator without Gazebo or attached devices."""
import math
import time
import json
import numpy as np
from .common_nodes import spin, parameter
from .contracts import DriveState
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, PointField, Image, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class MockHardware(Node):
    def __init__(self):
        super().__init__('mock_hardware')
        self.clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.state=DriveState(enabled=True,permitted=True)
        self.elapsed=1.0
        self.x=self.y=self.yaw=0.0
        self.step=0
        self.joint_positions=[0.0]*4
        self.radius=parameter(self,'wheel_radius',0.125)
        self.track=parameter(self,'track_width',0.525)
        self.datum=tuple(parameter(self,n,v) for n,v in [('datum_latitude',12.9716),('datum_longitude',77.5946),('datum_altitude',0.0)])
        self.mode=parameter(self,'gnss_mode','rtk_fixed')
        if self.mode not in ['rtk_fixed','rtk_float','standalone','no_fix','dropout']:raise ValueError('Invalid GNSS simulation mode')
        self.publish_clock=parameter(self,'publish_clock',True)
        self.create_subscription(Twist,'/niihan/drive/cmd_vel',self.command,10)
        self.clock_pub=self.create_publisher(ClockMsg,'/clock',10)
        self.odom_pub=self.create_publisher(Odometry,'/niihan/wheel/odom',10)
        self.imu_pub=self.create_publisher(Imu,'/niihan/imu/data',qos_profile_sensor_data)
        self.gps_pub=self.create_publisher(NavSatFix,'/niihan/gnss/fix',qos_profile_sensor_data)
        self.cloud_pub=self.create_publisher(PointCloud2,'/niihan/raw/lidar/points',qos_profile_sensor_data)
        self.camera_pub=self.create_publisher(Image,'/niihan/raw/camera/image_raw',qos_profile_sensor_data)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
        self.health_pub=self.create_publisher(Bool,'/niihan/drive/healthy',10)
        self.quality_pub=self.create_publisher(String,'/niihan/gnss/quality',10)
        self.create_timer(0.02,self.tick,clock=self.clock)

    def command(self,msg):self.state.receive(msg.linear.x,msg.angular.z,time.monotonic())

    def stamp(self):return rclpy.time.Time(seconds=self.elapsed).to_msg()

    def tick(self):
        self.elapsed+=0.02;self.step+=1
        if self.publish_clock:
            c=ClockMsg();c.clock=self.stamp();self.clock_pub.publish(c)
        linear,angular=self.state.output(time.monotonic())
        self.x+=linear*math.cos(self.yaw)*0.02;self.y+=linear*math.sin(self.yaw)*0.02;self.yaw+=angular*0.02
        odom=Odometry();odom.header.stamp=self.stamp();odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.pose.pose.position.x=self.x;odom.pose.pose.position.y=self.y
        odom.pose.pose.orientation.z=math.sin(self.yaw/2);odom.pose.pose.orientation.w=math.cos(self.yaw/2)
        odom.twist.twist.linear.x=linear;odom.twist.twist.angular.z=angular
        for i in [0,7,35]:odom.pose.covariance[i]=0.02;odom.twist.covariance[i]=0.02
        self.odom_pub.publish(odom)
        imu=Imu();imu.header.stamp=self.stamp();imu.header.frame_id='imu_link';imu.orientation=odom.pose.pose.orientation
        imu.angular_velocity.z=angular;imu.linear_acceleration.z=9.80665
        for i in [0,4,8]:imu.orientation_covariance[i]=0.02;imu.angular_velocity_covariance[i]=0.002;imu.linear_acceleration_covariance[i]=0.04
        self.imu_pub.publish(imu)
        health=Bool();health.data=True;self.health_pub.publish(health)
        joints=JointState();joints.header.stamp=self.stamp();joints.name=['front_left_wheel_joint','rear_left_wheel_joint','front_right_wheel_joint','rear_right_wheel_joint'];joints.position=self.joint_positions.copy()
        joints.velocity=[(linear-angular*self.track/2)/self.radius]*2+[(linear+angular*self.track/2)/self.radius]*2
        self.joint_positions=[p+v*.02 for p,v in zip(self.joint_positions,joints.velocity)]
        joints.position=self.joint_positions.copy();self.joint_pub.publish(joints)
        if self.step%5:return
        # A room with visible vertical returns and floor points; sensor motion follows wheel state.
        values=np.linspace(-9,9,91)
        points=[]
        for h in [.3,.7,1.2]:
            points.extend((x,-9,h) for x in values);points.extend((x,9,h) for x in values)
            points.extend((-9,y,h) for y in values);points.extend((9,y,h) for y in values)
        points.extend((x,y,0.0) for x in np.arange(-4,4,.25) for y in np.arange(-4,4,.25))
        p=np.asarray(points,dtype=np.float32);p[:,0]-=self.x;p[:,1]-=self.y
        c,s=math.cos(self.yaw),math.sin(self.yaw);p[:,:2]=p[:,:2]@np.array([[c,-s],[s,c]])
        cloud=PointCloud2();cloud.header.stamp=self.stamp();cloud.header.frame_id='base_footprint';cloud.height=1;cloud.width=len(p);cloud.is_dense=True;cloud.point_step=12;cloud.row_step=12*len(p)
        cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z'])];cloud.data=p.tobytes();self.cloud_pub.publish(cloud)
        image=Image();image.header=cloud.header;image.height=48;image.width=64;image.encoding='rgb8';image.step=192;image.data=np.full((48,64,3),80,dtype=np.uint8).tobytes();self.camera_pub.publish(image)
        fix=NavSatFix();fix.header.stamp=self.stamp();fix.header.frame_id='gnss_link';fix.status.status=-1 if self.mode=='no_fix' else 0;fix.status.service=1
        fix.latitude=self.datum[0]+math.degrees(self.y/6378137);fix.longitude=self.datum[1]+math.degrees(self.x/(6378137*math.cos(math.radians(self.datum[0]))));fix.altitude=self.datum[2]
        variance={'rtk_fixed':.0004,'rtk_float':.25,'standalone':9.0,'no_fix':100.0}.get(self.mode,100.0)
        fix.position_covariance=[variance,0.0,0.0,0.0,variance,0.0,0.0,0.0,variance*2];fix.position_covariance_type=2
        if self.mode!='dropout':self.gps_pub.publish(fix)
        quality=String();quality.data=self.mode;self.quality_pub.publish(quality)


def main(args=None):spin(MockHardware,args)
