"""Contract adaptation, motion interlock, and independent GNSS monitoring."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, LaserScan, Imu, NavSatFix, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from niihan_description.point_cloud_utils import xyz_array
from .contracts import DriveState, project_scan, enu

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)


def parameter(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def spin(kind, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = kind()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        cloud = parameter(self, 'cloud_topic', '/niihan/raw/lidar/points')
        self.low = parameter(self, 'min_height', 0.10)
        self.high = parameter(self, 'max_height', 1.8)
        self.level_height = parameter(self, 'gravity_aligned_height', False)
        self.scan_frame = parameter(self, 'scan_frame', 'base_footprint')
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.cloud_pub = self.create_publisher(PointCloud2, '/niihan/sensors/lidar/points', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud, self.cloud, qos_profile_sensor_data)
        # Gazebo already bridges the sole forward camera onto the dashboard topic.
        if parameter(self, 'relay_camera', True):
            self.camera_pub = self.create_publisher(Image, '/niihan/sensors/panoramic/front/image_raw', qos_profile_sensor_data)
            self.create_subscription(Image, parameter(self, 'camera_topic', '/niihan/raw/camera/image_raw'), self.camera_pub.publish, qos_profile_sensor_data)

    def cloud(self, msg):
        try:
            tf = self.tf.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            p = xyz_array(msg)
            q = tf.transform.rotation
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if norm < 1e-9:
                return
            x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
            rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            t = tf.transform.translation
            transformed = p @ rotation.T + np.array([t.x,t.y,t.z])
            if self.level_height:
                level=self.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform.rotation
                from .contracts import level_points
                transformed=level_points(transformed, [level.x,level.y,level.z,level.w])
        except (TransformException, ValueError):
            return
        self.cloud_pub.publish(msg)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/360
        scan.angle_max = scan.angle_min+359*scan.angle_increment
        scan.scan_time = 0.1
        scan.range_min,scan.range_max = 0.3,25.0
        scan.ranges = project_scan(transformed, low=self.low, high=self.high)
        self.scan_pub.publish(scan)


class HealthSupervisor(Node):
    def __init__(self):
        super().__init__('health_supervisor')
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.last = {}
        self.start = time.monotonic()
        self.timeout = parameter(self, 'sensor_timeout', 1.0)
        self.require_gnss = parameter(self, 'require_gnss', False)
        self.require_cliff = parameter(self, 'require_cliff', False)
        self.quality = 'unknown'
        self.cliff = 'UNKNOWN'
        self.drive_ok = False
        self.gnss_ok = False
        self.gnss_accepted = False
        self.create_subscription(String,'/niihan/gnss/status',self.gnss_status,10)
        for topic, kind in [('/odom',Odometry),('/scan',LaserScan),('/niihan/imu/data',Imu),('/niihan/sensors/lidar/points',PointCloud2)]:
            self.create_subscription(kind,topic,lambda msg,t=topic:self.seen(t,msg),qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'/niihan/gnss/fix',self.gnss,qos_profile_sensor_data)
        self.create_subscription(String,'/niihan/gnss/quality',self.gnss_quality,10)
        self.create_subscription(String,'/cliff/status',self.cliff_status,10)
        self.create_subscription(Bool,'/niihan/drive/healthy',self.drive_health,10)
        self.allowed = self.create_publisher(Bool,'/niihan/motion_permitted',10)
        self.status = self.create_publisher(String,'/niihan/health',10)
        self.create_timer(0.1,self.check,clock=self.steady)

    def seen(self, topic, msg):
        if isinstance(msg,LaserScan) and not any(math.isfinite(v) and msg.range_min<=v<=msg.range_max for v in msg.ranges):return
        if isinstance(msg,PointCloud2) and (not msg.width or not {'x','y','z'}.issubset({f.name for f in msg.fields})):return
        if isinstance(msg,Imu) and not all(math.isfinite(v) for v in (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)):return
        if isinstance(msg,Odometry) and not all(math.isfinite(v) for v in (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z)):return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if stamp > 0 and -0.05 <= age <= self.timeout:
            self.last[topic] = time.monotonic()

    def gnss(self,msg):
        self.seen('/niihan/gnss/fix',msg)
        self.gnss_ok = msg.status.status >= 0 and all(math.isfinite(v) for v in (msg.latitude,msg.longitude,msg.altitude))

    def gnss_status(self,msg):
        try:self.gnss_accepted=json.loads(msg.data).get('accepted') is True
        except (ValueError,AttributeError):self.gnss_accepted=False
        self.last['gnss_status']=time.monotonic()

    def gnss_quality(self,msg):
        self.quality=msg.data
        self.last['quality']=time.monotonic()

    def cliff_status(self,msg):
        self.cliff=msg.data
        self.last['cliff']=time.monotonic()

    def drive_health(self,msg):
        self.drive_ok=msg.data
        self.last['drive']=time.monotonic()

    def check(self):
        now=time.monotonic()
        required=['/odom','/scan','/niihan/imu/data','/niihan/sensors/lidar/points','drive']
        faults=[t for t in required if now-self.last.get(t,-math.inf)>self.timeout]
        if not self.drive_ok:faults.append('drive_fault')
        if self.require_gnss and (not self.gnss_ok or not self.gnss_accepted or now-self.last.get('gnss_status',-math.inf)>self.timeout or self.quality!='rtk_fixed' or
                                 now-self.last.get('/niihan/gnss/fix',-math.inf)>self.timeout or
                                 now-self.last.get('quality',-math.inf)>self.timeout):
            faults.append('gnss_not_fixed_or_stale')
        if self.require_cliff and (self.cliff.startswith(('FAULT','UNKNOWN')) or now-self.last.get('cliff',-math.inf)>self.timeout):
            faults.append('cliff_unavailable')
        permission=Bool();permission.data=not faults;self.allowed.publish(permission)
        status=String();status.data=json.dumps({'ready':not faults,'faults':faults,'gnss_quality':self.quality,
                                               'require_gnss':self.require_gnss,'cliff':self.cliff})
        self.status.publish(status)


class MotionGateway(Node):
    def __init__(self):
        super().__init__('motion_gateway')
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.state=DriveState(enabled=parameter(self,'drive_enabled',False))
        self.permission_time=-math.inf
        self.max_linear=parameter(self,'max_linear_speed',0.3)
        self.max_angular=parameter(self,'max_angular_speed',0.6)
        self.pub=self.create_publisher(Twist,'/niihan/drive/cmd_vel',10)
        self.create_subscription(Twist,'/niihan/cmd_vel',self.command,10)
        self.create_subscription(Bool,'/niihan/motion_permitted',self.permission,10)
        self.create_subscription(Bool,'/niihan/e_stop',self.estop,LATCHED)
        self.create_timer(0.02,self.tick,clock=self.steady)

    def command(self,msg):
        self.state.receive(msg.linear.x,msg.angular.z,time.monotonic())

    def permission(self,msg):
        self.permission_time=time.monotonic()
        self.state.permitted=msg.data

    def estop(self,msg):
        self.state.estop=msg.data
        self.state.received=-math.inf
        self.tick()

    def tick(self):
        now=time.monotonic()
        if now-self.permission_time>0.3:self.state.permitted=False
        linear,angular=self.state.output(now)
        msg=Twist();msg.linear.x=max(-self.max_linear,min(self.max_linear,linear));msg.angular.z=max(-self.max_angular,min(self.max_angular,angular));self.pub.publish(msg)


class GnssMonitor(Node):
    def __init__(self):
        super().__init__('gnss_monitor')
        self.datum=tuple(parameter(self,n,v) for n,v in [('datum_latitude',12.9716),('datum_longitude',77.5946),('datum_altitude',0.0)])
        self.maximum=parameter(self,'maximum_horizontal_variance',25.0)
        self.pub=self.create_publisher(Odometry,'/niihan/gnss/odometry',10)
        self.status=self.create_publisher(String,'/niihan/gnss/status',10)
        self.create_subscription(NavSatFix,'/niihan/gnss/fix',self.fix,qos_profile_sensor_data)

    def fix(self,msg):
        accepted=msg.status.status>=0 and msg.position_covariance_type!=0
        cov=np.array(msg.position_covariance).reshape(3,3)
        accepted=accepted and np.isfinite(cov).all() and np.linalg.eigvalsh((cov+cov.T)/2).min()>=0 and 0<max(cov[0,0],cov[1,1])<=self.maximum
        age=self.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        accepted=accepted and -0.05<=age<=1.0
        try:xyz=enu(msg.latitude,msg.longitude,msg.altitude,self.datum)
        except ValueError:accepted=False;xyz=np.zeros(3)
        status=String();status.data=json.dumps({'accepted':bool(accepted),'latitude':msg.latitude if math.isfinite(msg.latitude) else None,
                                              'longitude':msg.longitude if math.isfinite(msg.longitude) else None,'frame':'gnss_local'})
        self.status.publish(status)
        if not accepted:return
        odom=Odometry();odom.header=msg.header;odom.header.frame_id='gnss_local';odom.child_frame_id='gnss_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
        odom.pose.pose.orientation.w=1.0
        for row in range(3):
            for col in range(3):odom.pose.covariance[row*6+col]=float(cov[row,col])
        odom.pose.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=1e6
        self.pub.publish(odom)


def sensor_adapter_main(args=None):spin(SensorAdapter,args)
def health_main(args=None):spin(HealthSupervisor,args)
def motion_main(args=None):spin(MotionGateway,args)
def gnss_monitor_main(args=None):spin(GnssMonitor,args)
