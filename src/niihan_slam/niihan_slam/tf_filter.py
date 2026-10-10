"""Relay coherent LIO TF and a yaw-only base for planar navigation."""
import copy, math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TfFilter(Node):
    def __init__(self):
        super().__init__('lio_tf_filter')
        self.declare_parameter('mapping',True);self.mapping=self.get_parameter('mapping').value
        self.pub=self.create_publisher(TFMessage,'/tf',100)
        self.base=None;self.global_tf=None
        self.create_subscription(TFMessage,'/niihan/lio/tf',self.forward,100)
    def forward(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='base_footprint':self.base=t
            elif t.child_frame_id=='odom':self.global_tf=t
        if self.base is None:return
        if self.mapping and (self.global_tf is None or self.global_tf.header.stamp!=self.base.header.stamp):return
        nav=copy.deepcopy(self.base);nav.child_frame_id='base_nav';nav.transform.translation.z=0.
        q=nav.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        q.x=q.y=0.;q.z=math.sin(yaw/2);q.w=math.cos(yaw/2)
        transforms=[self.base,nav]+([self.global_tf] if self.mapping else [])
        self.pub.publish(TFMessage(transforms=transforms))

def main():
    rclpy.init();node=TfFilter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
