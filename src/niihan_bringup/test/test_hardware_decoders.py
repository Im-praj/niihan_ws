"""Physical decoder tests use messages only; no serial device is opened."""
import struct
from types import SimpleNamespace
from niihan_bringup.hardware_nodes import F9pDriver

class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)

def test_nav_pvt_fixed_float_no_fix_and_duplicate_epoch():
    receiver=SimpleNamespace(last_tow=None,pub=Publisher(),quality=Publisher(),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:__import__('builtin_interfaces.msg',fromlist=['Time']).Time(sec=1))))
    payload=bytearray(92)
    struct.pack_into('<I',payload,0,1000);payload[20]=3;payload[21]=1|(2<<6)
    struct.pack_into('<iii',payload,24,775946000,129716000,50000)
    struct.pack_into('<II',payload,40,20,30)
    F9pDriver.pvt(receiver,payload)
    assert receiver.quality.messages[-1].data=='rtk_fixed'
    assert abs(receiver.pub.messages[-1].longitude-77.5946)<1e-6
    assert receiver.pub.messages[-1].position_covariance[0]==.0004
    F9pDriver.pvt(receiver,payload);assert len(receiver.pub.messages)==1
    struct.pack_into('<I',payload,0,1200);payload[21]=1|(1<<6)
    F9pDriver.pvt(receiver,payload);assert receiver.quality.messages[-1].data=='rtk_float'
    struct.pack_into('<I',payload,0,1400);payload[20]=5
    F9pDriver.pvt(receiver,payload);assert receiver.pub.messages[-1].status.status==-1
