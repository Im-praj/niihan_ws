"""Physical device adapters. Never substitute synthetic data on device failure."""
import math
import time
import struct
import json
import numpy as np
import serial
from .common_nodes import spin, parameter
from .contracts import packet, unpack_packet, sequence_is_newer, UbxParser
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt8MultiArray


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.port=parameter(self,'port','/dev/niihan-stm32')
        self.radius=parameter(self,'wheel_radius',0.1)
        self.track=parameter(self,'track_width',0.525)
        self.ticks=parameter(self,'ticks_per_revolution',0)
        self.calibrated=parameter(self,'hardware_calibrated',False)
        if self.calibrated and (self.ticks<=0 or self.radius<=0 or self.track<=0):raise ValueError('Measured dimensions and encoder ticks/revolution are required')
        self.handle=None;self.buffer=bytearray();self.last_attempt=-math.inf
        self.seq=0;self.previous=None;self.uptime=None;self.last_feedback=-math.inf
        self.x=self.y=self.yaw=0.0;self.cmd=(0.0,0.0);self.command_time=-math.inf
        self.odom=self.create_publisher(Odometry,'/niihan/wheel/odom',10)
        self.health=self.create_publisher(Bool,'/niihan/drive/healthy',10)
        self.status=self.create_publisher(String,'/niihan/drive/status',10)
        self.create_subscription(Twist,'/niihan/drive/cmd_vel',self.command,10)
        self.create_timer(.02,self.tick,clock=self.steady)

    def command(self,msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.cmd=(msg.linear.x,msg.angular.z);self.command_time=time.monotonic()

    def tick(self):
        now=time.monotonic()
        try:
            if self.handle is None and now-self.last_attempt>2:
                self.last_attempt=now;self.handle=serial.Serial(self.port,115200,timeout=0,write_timeout=.02)
                self.buffer.clear();self.previous=None;self.uptime=None;self.command_time=-math.inf
            if self.handle:
                enabled=self.calibrated and now-self.command_time<.25 and now-self.last_feedback<.5
                linear,angular=self.cmd if enabled else (0.0,0.0)
                left=(linear-angular*self.track/2)/self.radius;right=(linear+angular*self.track/2)/self.radius
                self.handle.write(packet(f'N1,{self.seq},{left:.6f},{right:.6f},{int(enabled)}'));self.seq=(self.seq+1)&0xffffffff
                self.buffer.extend(self.handle.read(4096))
                if len(self.buffer)>8192:raise ValueError('serial buffer overflow')
                while b'\n' in self.buffer:
                    line,_,remaining=self.buffer.partition(b'\n');self.buffer=bytearray(remaining)
                    try:self.feedback(unpack_packet(line),now)
                    except (ValueError,IndexError,OverflowError):continue
        except (serial.SerialException,OSError,ValueError) as exc:
            if self.handle:self.handle.close()
            self.handle=None;self.last_feedback=-math.inf
            self.get_logger().warning(f'STM32 offline: {exc}',throttle_duration_sec=5)
        healthy=self.handle is not None and now-self.last_feedback<.5
        msg=Bool();msg.data=healthy;self.health.publish(msg)
        status=String();status.data=json.dumps({'connected':self.handle is not None,'feedback_fresh':healthy,'calibrated':self.calibrated,'driver':'ZLAC8015D V4','actuation_backend':'requires verified MCU integration'})
        self.status.publish(status)

    def feedback(self,fields,now):
        if len(fields)!=7 or fields[0]!='T1':raise ValueError('wrong telemetry version')
        seq,uptime,left,right,estop,fault=map(int,fields[1:])
        if not 0<=seq<=0xffffffff or not 0<=uptime<=0xffffffff or estop not in [0,1] or fault not in [0,1] or not -(2**31)<=left<2**31 or not -(2**31)<=right<2**31:raise ValueError('invalid telemetry')
        current=(seq,uptime,left,right)
        if self.previous is not None:
            prev=self.previous
            dt=((uptime-prev[1])&0xffffffff)/1000
            if not sequence_is_newer(seq,prev[0]) or not 0<dt<=.5:
                self.previous=None;self.last_feedback=-math.inf;return
            dl=((left-prev[2]+2**31)%2**32-2**31)
            dr=((right-prev[3]+2**31)%2**32-2**31)
            if self.calibrated:
                dl*=2*math.pi*self.radius/self.ticks;dr*=2*math.pi*self.radius/self.ticks
                distance=(dl+dr)/2;angle=(dr-dl)/self.track
                if abs(distance/dt)>2 or abs(angle/dt)>5:raise ValueError('implausible encoder jump')
                self.x+=distance*math.cos(self.yaw+angle/2);self.y+=distance*math.sin(self.yaw+angle/2);self.yaw+=angle
                msg=Odometry();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
                msg.pose.pose.position.x=self.x;msg.pose.pose.position.y=self.y;msg.pose.pose.orientation.z=math.sin(self.yaw/2);msg.pose.pose.orientation.w=math.cos(self.yaw/2)
                msg.twist.twist.linear.x=distance/dt;msg.twist.twist.angular.z=angle/dt
                for i in [0,7,35]:msg.pose.covariance[i]=.04;msg.twist.covariance[i]=.02
                self.odom.publish(msg)
        self.previous=current
        self.last_feedback=now if not estop and not fault else -math.inf

    def destroy_node(self):
        if self.handle:
            try:self.handle.write(packet(f'N1,{self.seq},0.000000,0.000000,0'))
            except (serial.SerialException,OSError):pass
            self.handle.close()
        return super().destroy_node()


class F9pDriver(Node):
    def __init__(self):
        super().__init__('f9p_driver')
        self.port=parameter(self,'port','/dev/niihan-gnss');self.baud=parameter(self,'baud',115200)
        self.serial=None;self.parser=UbxParser();self.last_attempt=-math.inf;self.last_tow=None
        self.pub=self.create_publisher(NavSatFix,'/niihan/gnss/fix',10)
        self.quality=self.create_publisher(String,'/niihan/gnss/quality',10)
        self.create_subscription(UInt8MultiArray,'/niihan/gnss/rtcm',self.corrections,10)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.steady)

    def corrections(self,msg):
        # Complete validated RTCM3 frames are required from a separate trusted NTRIP/base client.
        data=bytes(msg.data)
        if not self.serial or len(data)<6 or data[0]!=0xd3:return
        size=((data[1]&3)<<8)|data[2]
        if len(data)!=size+6:return
        crc=0
        for b in data[:-3]:
            crc^=b<<16
            for _ in range(8):
                crc<<=1
                if crc&0x1000000:crc^=0x1864cfb
        if (crc&0xffffff)!=int.from_bytes(data[-3:],'big'):return
        try:self.serial.write(data)
        except (serial.SerialException,OSError):self.serial.close();self.serial=None

    def tick(self):
        try:
            if self.serial is None and time.monotonic()-self.last_attempt>2:
                self.last_attempt=time.monotonic();self.serial=serial.Serial(self.port,self.baud,timeout=0,write_timeout=.05);self.parser=UbxParser();self.last_poll=-math.inf;self.last_tow=None
            if self.serial:
                if time.monotonic()-self.last_poll>=.2:
                    self.serial.write(bytes.fromhex("b562010700000819"));self.last_poll=time.monotonic()
                for cls,ident,payload in self.parser.feed(self.serial.read(4096)):
                    if (cls,ident)==(1,7) and len(payload)==92:self.pvt(payload)
        except (serial.SerialException,OSError) as exc:
            if self.serial:self.serial.close()
            self.serial=None;self.get_logger().warning(f'F9P offline: {exc}',throttle_duration_sec=5)

    def pvt(self,p):
        tow=struct.unpack_from('<I',p,0)[0]
        if tow>=604800000:return
        if self.last_tow is not None and not 0<(tow-self.last_tow)%604800000<302400000:return
        self.last_tow=tow
        fix_type,flags=p[20],p[21];valid=bool(flags&1) and fix_type in (3,4)
        longitude,latitude,height=struct.unpack_from('<iii',p,24)
        horizontal,vertical=struct.unpack_from('<II',p,40)
        msg=NavSatFix();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='gnss_link';msg.status.status=0 if valid else -1;msg.status.service=15
        msg.longitude=longitude*1e-7;msg.latitude=latitude*1e-7;msg.altitude=height*.001
        msg.position_covariance=[(horizontal*.001)**2,0.0,0.0,0.0,(horizontal*.001)**2,0.0,0.0,0.0,(vertical*.001)**2];msg.position_covariance_type=2
        carrier=(flags>>6)&3;state='no_fix' if not valid else {0:'standalone',1:'rtk_float',2:'rtk_fixed'}.get(carrier,'unknown')
        quality=String();quality.data=state;self.quality.publish(quality);self.pub.publish(msg)

    def destroy_node(self):
        if self.serial:self.serial.close()
        return super().destroy_node()


class BnoDriver(Node):
    def __init__(self):
        super().__init__('bno085_driver')
        self.sensor=None;self.last_attempt=-math.inf
        self.pub=self.create_publisher(Imu,'/niihan/imu/data',10)
        self.create_timer(.02,self.tick)

    def tick(self):
        try:
            if self.sensor is None and time.monotonic()-self.last_attempt>2:
                self.last_attempt=time.monotonic()
                import board
                from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_GAME_ROTATION_VECTOR
                from adafruit_bno08x.i2c import BNO08X_I2C
                self.sensor=BNO08X_I2C(board.I2C())
                for report in [BNO_REPORT_ACCELEROMETER,BNO_REPORT_GYROSCOPE,BNO_REPORT_GAME_ROTATION_VECTOR]:self.sensor.enable_feature(report)
            if self.sensor:
                a=self.sensor.acceleration;g=self.sensor.gyro;q=self.sensor.game_quaternion
                if not all(math.isfinite(v) for v in (*a,*g,*q)):return
                msg=Imu();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='imu_link'
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=a
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=g
                msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w=q
                for i in [0,4,8]:msg.orientation_covariance[i]=.02;msg.angular_velocity_covariance[i]=.002;msg.linear_acceleration_covariance[i]=.04
                self.pub.publish(msg)
        except (ImportError,OSError,RuntimeError,ValueError,TypeError) as exc:
            self.sensor=None;self.get_logger().warning(f'BNO085 offline: {exc}',throttle_duration_sec=5)


def stm32_main(args=None):spin(Stm32Bridge,args)
def f9p_main(args=None):spin(F9pDriver,args)
def bno_main(args=None):spin(BnoDriver,args)
