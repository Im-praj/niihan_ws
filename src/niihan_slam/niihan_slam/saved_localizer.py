"""Initial-pose-assisted 3D cloud registration against a saved flat-site map.

Not global place recognition. Only accepted fits publish map->odom; weak fits
expire so dashboard/localization freshness inhibits navigation.
"""
import json,math
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from niihan_description.point_cloud_utils import xyz_array
from .grid import rotation,icp_planar

class SavedLocalizer(Node):
    def __init__(self):
        super().__init__('saved_map_localizer')
        self.declare_parameter('map_file','');self.declare_parameter('initial_x',0.);self.declare_parameter('initial_y',0.);self.declare_parameter('initial_yaw',0.)
        file=self.get_parameter('map_file').value
        if not file:raise ValueError('map_file must name a saved map_3d.npz')
        saved=np.load(file);p=saved['points'];self.map_points=p[np.isfinite(p).all(axis=1)].astype(np.float32);self.tree=cKDTree(self.map_points);self.ground=float(saved['ground_z']);self.z_offset=None
        self.pose=np.array([self.get_parameter('initial_x').value,self.get_parameter('initial_y').value,self.get_parameter('initial_yaw').value])
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self);self.broadcaster=TransformBroadcaster(self)
        self.status=self.create_publisher(String,'/niihan/localizer/status',10)
        self.create_subscription(PointCloud2,'/niihan/raw/lidar/points',self.cloud,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial,10)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid=saved['grid'];self.map=OccupancyGrid();self.map.header.frame_id='map';self.map.info.resolution=float(saved['resolution']);self.map.info.width=grid.shape[1];self.map.info.height=grid.shape[0];self.map.info.origin.position.x=self.map.info.origin.position.y=float(saved['origin']);self.map.info.origin.orientation.w=1.;self.map.data=grid.ravel().tolist();self.create_timer(1.,self.publish_map)
        self.cloud_pub=self.create_publisher(PointCloud2,'/niihan/slam/map_cloud',qos_profile_sensor_data)
        self.last=0.;self.last_stamp=None
    def publish_map(self):
        self.map.header.stamp=self.get_clock().now().to_msg();self.map_pub.publish(self.map)
        from niihan_description.vortex_3d_mapper import _make_pointcloud2
        if self.last_stamp is not None:self.cloud_pub.publish(_make_pointcloud2(self.map_points[::max(1,len(self.map_points)//30000)],"map",self.last_stamp))
    def initial(self,msg):
        if msg.header.frame_id!='map':return
        try:
            t=self.tf.lookup_transform('odom','base_footprint',rclpy.time.Time()).transform
            q=msg.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));oq=t.rotation;oyaw=math.atan2(2*(oq.w*oq.z+oq.x*oq.y),1-2*(oq.y*oq.y+oq.z*oq.z));a=yaw-oyaw
            r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
            self.pose=np.array([*(np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])-r@np.array([t.translation.x,t.translation.y])),a])
        except TransformException:return
    def cloud(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp-self.last<.4:return
        try:
            t=self.tf.lookup_transform('odom',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
            p=xyz_array(msg);p=p[np.isfinite(p).all(axis=1)];p=p[(np.linalg.norm(p,axis=1)>.5)&(np.linalg.norm(p,axis=1)<25.)]
            p=p[::max(1,len(p)//2000)]
            q=t.rotation;p=p@rotation([q.x,q.y,q.z,q.w]).T+np.array([t.translation.x,t.translation.y,t.translation.z])
            if self.z_offset is None:
                base=self.tf.lookup_transform('odom','base_footprint',rclpy.time.Time.from_msg(msg.header.stamp)).transform.translation
                self.z_offset=self.ground-base.z
            p[:,2]+=self.z_offset
            pose,rmse,overlap=icp_planar(p,self.tree,self.pose,max_distance=.6)
            accepted=overlap>=.45 and rmse<=.20 and np.linalg.norm(pose[:2]-self.pose[:2])<.5 and abs(pose[2]-self.pose[2])<.25
            if accepted:
                self.last_stamp=msg.header.stamp
                self.pose=pose;tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp=msg.header.stamp;tf.transform.translation.x,tf.transform.translation.y=pose[:2].tolist();tf.transform.translation.z=self.z_offset;tf.transform.rotation.z=math.sin(pose[2]/2);tf.transform.rotation.w=math.cos(pose[2]/2);self.broadcaster.sendTransform(tf)
            self.status.publish(String(data=json.dumps({'accepted':bool(accepted),'rmse_m':rmse,'overlap':overlap,'initial_pose_required':True})));self.last=stamp
        except (TransformException,ValueError):return

def main():
    rclpy.init();n=SavedLocalizer()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
