"""Publish base odometry and a conservative flat-site map from estimated 3D poses."""
import json,math,time,copy
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import PointCloud2,Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException,TransformBroadcaster
from niihan_description.point_cloud_utils import xyz_array
from .grid import Grid,rotation

class MapBridge(Node):
    def __init__(self):
        super().__init__('slam_map_bridge')
        def param(n,v):self.declare_parameter(n,v);return self.get_parameter(n).value
        self.directory=Path(param('output_directory','/tmp/niihan_map'))
        self.mapping=param('mapping',True);self.ground=param('ground_z',-.145)
        self.ground_initialized=False
        self.grid=Grid(param('resolution',.1));self.points={};self.last=None;self.received=0.;self.gyro=0.;self.cloud_stamp=None
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',latched)
        self.cloud_pub=self.create_publisher(PointCloud2,'/niihan/slam/map_cloud',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/niihan/slam/status',10)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.create_subscription(Odometry,'/glim_ros/odom',self.odom,10)
        self.create_subscription(Imu,'/niihan/imu/data',lambda m:setattr(self,'gyro',m.angular_velocity.z),qos_profile_sensor_data)
        if self.mapping:
            self.create_subscription(PointCloud2,'/glim_ros/points',self.cloud,10)
            self.create_subscription(PointCloud2,'/glim_ros/map',self.optimized_map,latched)
        self.create_service(Trigger,'/niihan/slam/save_map',self.save)
        self.create_timer(1.,self.publish)
    def odom(self,msg):
        try:
            tf=self.tf.lookup_transform(msg.child_frame_id,'base_footprint',rclpy.time.Time())
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;t=tf.transform.translation
            rot=rotation([q.x,q.y,q.z,q.w]);base=np.array([p.x,p.y,p.z])+rot@np.array([t.x,t.y,t.z])
            # Simulator IMU and base axes align; this bridge refuses other rotations.
            rq=tf.transform.rotation
            if abs(rq.w)<.9999:return
            out=Odometry();out.header=msg.header;out.child_frame_id='base_nav';out.pose.pose=copy.deepcopy(msg.pose.pose)
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=base.tolist()
            v=msg.twist.twist.linear;body=rot.T@np.array([v.x,v.y,v.z]);out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=body.tolist();out.twist.twist.angular.z=self.gyro
            out.pose.covariance=msg.pose.covariance;out.twist.covariance=msg.twist.covariance
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            out.pose.pose.position.z=0.;out.pose.pose.orientation.x=out.pose.pose.orientation.y=0.
            out.pose.pose.orientation.z=math.sin(yaw/2);out.pose.pose.orientation.w=math.cos(yaw/2)
            self.odom_pub.publish(out);self.last=out;self.received=time.monotonic()
        except (TransformException,ValueError):return
    def cloud(self,msg):
        try:
            p=xyz_array(msg);p=p[np.isfinite(p).all(axis=1)]
            alignment=self.tf.lookup_transform('map',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
            aq=alignment.rotation;at=alignment.translation;p=p@rotation([aq.x,aq.y,aq.z,aq.w]).T+np.array([at.x,at.y,at.z])
            tf=self.tf.lookup_transform('map','unitree_l2_link',rclpy.time.Time.from_msg(msg.header.stamp))
            t=tf.transform.translation
            if not self.ground_initialized:
                base=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time.from_msg(msg.header.stamp)).transform.translation
                self.ground=base.z;self.ground_initialized=True
            self.grid.update(p,[t.x,t.y,t.z],self.ground)
            self.cloud_stamp=copy.deepcopy(msg.header.stamp)
            for xyz in p:
                key=tuple(np.floor(xyz/.15).astype(int));self.points[key]=xyz
        except (TransformException,ValueError):return
    def optimized_map(self,msg):
        try:
            p=xyz_array(msg);p=p[np.isfinite(p).all(axis=1)]
            if not len(p):return
            self.points={tuple(np.floor(xyz/.15).astype(int)):xyz for xyz in p}
            self.grid.occupied[:]=False
            selected=p[(p[:,2]>self.ground+.15)&(p[:,2]<self.ground+1.8)]
            cells=self.grid.cells(selected[:,:2]);cells=cells[(cells>=0).all(axis=1)&(cells<self.grid.size).all(axis=1)]
            self.grid.occupied[cells[:,1],cells[:,0]]=True
        except ValueError:return

    def publish(self):
        if self.mapping and self.points and self.cloud_stamp is not None:
            m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.info.resolution=self.grid.resolution;m.info.width=m.info.height=self.grid.size;m.info.origin.position.x=m.info.origin.position.y=self.grid.origin;m.info.origin.orientation.w=1.;m.data=self.grid.data().ravel().tolist();self.map_pub.publish(m)
            from niihan_description.vortex_3d_mapper import _make_pointcloud2
            points=np.array(list(self.points.values()),dtype=np.float32)
            self.cloud_pub.publish(_make_pointcloud2(points[::max(1,len(points)//30000)],'map',self.cloud_stamp))
        status={'backend':'GLIM CPU','mode':'mapping' if self.mapping else 'saved-map localization','odometry_fresh':time.monotonic()-self.received<2.,'map_points':len(self.points),'simulation':True,'projection':'flat-ground; no slope/drop-off certification'}
        self.status.publish(String(data=json.dumps(status)))
    def save(self,request,response):
        if not self.mapping or not self.points:response.success=False;response.message='No estimated map to save';return response
        self.directory.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(self.directory/'map_3d.npz',points=np.array(list(self.points.values())),grid=self.grid.data(),resolution=self.grid.resolution,origin=self.grid.origin,ground_z=self.ground)
        import yaml
        data=self.grid.data();image=np.full(data.shape,205,dtype=np.uint8);image[data==0]=254;image[data==100]=0
        with open(self.directory/'map.pgm','wb') as f:f.write(f'P5\n{self.grid.size} {self.grid.size}\n255\n'.encode()+np.flipud(image).tobytes())
        (self.directory/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':self.grid.resolution,'origin':[self.grid.origin,self.grid.origin,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
        response.success=True;response.message=str(self.directory);return response

def main():
    rclpy.init();n=MapBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
