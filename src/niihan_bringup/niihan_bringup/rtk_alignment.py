"""Independent RTK base pose in a surveyed SLAM map; never publishes TF."""
import json,math,time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from .contracts import enu
from .common_nodes import spin,parameter

def load_alignment(file,environment):
    d=json.loads(Path(file).read_text())
    if d.get('calibrated') is not True or d.get('environment_id')!=environment:raise ValueError('A surveyed alignment for this environment is required')
    a=np.asarray(d['datum'],float);b=np.asarray(d['map_translation'],float);yaw=float(d['map_yaw'])
    if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(np.r_[a,b,yaw]).all():raise ValueError('Invalid GNSS map alignment')
    enu(*a,tuple(a))
    return d

def map_antenna(latitude,longitude,altitude,alignment):
    xyz=enu(latitude,longitude,altitude,alignment['datum']);yaw=alignment['map_yaw'];c,s=math.cos(yaw),math.sin(yaw)
    return np.array([c*xyz[0]-s*xyz[1],s*xyz[0]+c*xyz[1],xyz[2]])+alignment['map_translation']

def rotation(q):
    v=np.array([q.x,q.y,q.z,q.w],float);norm=np.linalg.norm(v)
    if not np.isfinite(v).all() or norm<1e-9:raise ValueError('Invalid rotation')
    x,y,z,w=v/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class RtkAlignment(Node):
    def __init__(self):
        super().__init__('rtk_alignment')
        file=parameter(self,'alignment_file','');environment=parameter(self,'environment_id','portable-site')
        self.alignment=load_alignment(file,environment) if file else None
        if self.alignment and self.alignment.get('source_simulation') and not self.get_parameter('use_sim_time').value:raise ValueError('Simulation alignment cannot commission physical hardware')
        self.maximum_horizontal=float(parameter(self,'maximum_horizontal_std',.05));self.maximum_vertical=float(parameter(self,'maximum_vertical_std',.10))
        self.allow_simulated=bool(parameter(self,'allow_simulated_rtk',False))
        if self.allow_simulated and not self.get_parameter('use_sim_time').value:raise ValueError('Simulated RTK is forbidden without a simulation clock')
        self.metrics={};self.metrics_history={};self.received=-math.inf
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        self.pose=self.create_publisher(Odometry,'/niihan/gnss/map_pose',10)
        self.status=self.create_publisher(String,'/niihan/gnss/alignment',10)
        self.create_subscription(String,'/niihan/gnss/metrics',self.quality,10)
        self.create_subscription(NavSatFix,'/niihan/gnss/fix',self.fix,qos_profile_sensor_data)
        self.pending=deque(maxlen=20);self.create_timer(.05,self.publish_pending)
        self.create_timer(1.,lambda:self.status.publish(String(data=json.dumps({'calibrated':self.alignment is not None,'environment_id':environment}))))
    def quality(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            self.metrics=data;self.received=time.monotonic()
            if isinstance(data.get('stamp'),(int,float)):
                self.metrics_history[data['stamp']]=data
                while len(self.metrics_history)>40:self.metrics_history.pop(next(iter(self.metrics_history)))
            if data.get('rtk_lock') is not True:self.pending.clear()
        except ValueError:self.metrics={}
    def fix(self,msg):self.pending.append(msg)
    def publish_pending(self):
        if self.alignment is None or self.metrics.get('rtk_lock') is not True or time.monotonic()-self.received>1.:return
        now=self.get_clock().now().nanoseconds/1e9
        for msg in list(self.pending):
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if now-stamp>.5:self.pending.remove(msg);continue
            metrics=self.metrics_history.get(stamp)
            if metrics is None or not -.05<=now-stamp<=.5:continue
            if metrics.get('quality')!='rtk_fixed' or metrics.get('rtk_lock') is not True:continue
            if metrics.get('simulation') is not False and not (self.allow_simulated and metrics.get('simulation') is True):continue
            if metrics.get('simulation') is False and metrics.get('timestamp_source')!='receiver_utc':continue
            cov=np.asarray(msg.position_covariance).reshape(3,3)
            if msg.status.status<0 or msg.position_covariance_type==0 or not np.isfinite(cov).all():continue
            if np.linalg.eigvalsh((cov+cov.T)/2).min()<0 or not 0<max(cov[0,0],cov[1,1])<=self.maximum_horizontal**2 or not 0<cov[2,2]<=self.maximum_vertical**2:continue
            try:
                epoch=rclpy.time.Time.from_msg(msg.header.stamp)
                # GPS determines translation. Only body orientation is needed for
                # its lever arm; slow map registration must not delay every fix.
                local=self.tf.lookup_transform('odom','base_footprint',epoch).transform
                global_tf=self.tf.lookup_transform('map','odom',rclpy.time.Time())
                global_stamp=global_tf.header.stamp.sec+global_tf.header.stamp.nanosec/1e9
                if not -.1<=now-global_stamp<=1.:continue
                mount=self.tf.lookup_transform('base_footprint','gnss_link',rclpy.time.Time()).transform.translation
                global_r=rotation(global_tf.transform.rotation);body_r=global_r@rotation(local.rotation)
                xyz=map_antenna(msg.latitude,msg.longitude,msg.altitude,self.alignment)-body_r@np.array([mount.x,mount.y,mount.z])
                gt=global_tf.transform.translation;lt=local.translation
                estimate=global_r@np.array([lt.x,lt.y,lt.z])+np.array([gt.x,gt.y,gt.z])
                if not np.isfinite(xyz).all() or np.linalg.norm(xyz-estimate)>.5:continue
                out=Odometry();out.header=msg.header;out.header.frame_id='map';out.child_frame_id='base_footprint'
                out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,xyz);out.pose.pose.orientation.w=1.
                yaw=self.alignment['map_yaw'];c,ss=math.cos(yaw),math.sin(yaw);rot=np.array([[c,-ss,0],[ss,c,0],[0,0,1.]])
                cov=rot@cov@rot.T
                for i in range(3):
                    for j in range(3):out.pose.covariance[i*6+j]=float(cov[i,j])
                out.pose.covariance[21]=out.pose.covariance[28]=out.pose.covariance[35]=1e6
                self.pose.publish(out);self.pending.remove(msg)
            except (TransformException,ValueError):continue

def main(args=None):spin(RtkAlignment,args)
