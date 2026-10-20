#!/usr/bin/env python3
"""Simulation-only UBX receiver fixture. Gazebo NavSat supplies positions.
Satellites/DOP/RTK flags below are test inputs, not a satellite constellation.
"""
import os,struct,sys,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from niihan_bringup.contracts import ubx_checksum
class Fixture(Node):
    def __init__(self,fd):
        super().__init__('f9p_protocol_fixture');self.fd=fd
        self.declare_parameter('use_sim_time',True) if not self.has_parameter('use_sim_time') else self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.declare_parameter('quality_state','fixed')
        self.create_subscription(NavSatFix,'/niihan/raw/gnss/fix',self.fix,qos_profile_sensor_data)
        os.set_blocking(fd,False);self.create_timer(.05,self.drain)
    def drain(self):
        try:os.read(self.fd,4096)
        except (BlockingIOError,OSError):pass
    def send(self,ident,payload):
        body=bytes([1,ident])+struct.pack('<H',len(payload))+payload
        try:os.write(self.fd,b'\xb5\x62'+body+bytes(ubx_checksum(body)))
        except (BlockingIOError,OSError):pass
    def fix(self,msg):
        state=self.get_parameter('quality_state').value
        if state=='dropout':return
        tow=(msg.header.stamp.sec*1000+msg.header.stamp.nanosec//1000000)%604800000
        dop=bytearray(18);struct.pack_into('<I',dop,0,tow);struct.pack_into('<HH',dop,10,110,65)
        pvt=bytearray(92);struct.pack_into('<I',pvt,0,tow);pvt[20]=3;pvt[21]=1|({'fixed':2,'float':1}.get(state,0)<<6);pvt[23]=24
        if state=='no_fix':pvt[21]=0;pvt[23]=0
        struct.pack_into('<iii',pvt,24,round(msg.longitude*1e7),round(msg.latitude*1e7),round(msg.altitude*1000))
        struct.pack_into('<II',pvt,40,20,30)
        self.send(4,dop);self.send(7,pvt)
rclpy.init();node=Fixture(int(sys.argv[1]))
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
