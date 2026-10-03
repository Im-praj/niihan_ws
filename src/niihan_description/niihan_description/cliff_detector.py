#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from niihan_description.point_cloud_utils import xyz_array, ground_observation_mask
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException

def _make_pointcloud2(points_xyz, frame_id, stamp):
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp)
    msg.height = 1
    msg.width = len(points_xyz) if len(points_xyz) > 0 else 0
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if msg.width > 0:
        msg.data = points_xyz.astype(np.float32).tobytes()
    else:
        msg.data = b''
    return msg

class CliffDetector(Node):
    def __init__(self):
        super().__init__('cliff_detector')
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('ground_height_threshold', 0.15)
        self.declare_parameter('obstacle_height_threshold', 0.3)
        self.declare_parameter('cliff_check_radius', 6.0)
        self.declare_parameter('cell_size', 0.5)
        self.declare_parameter('min_ground_points', 3)
        self.declare_parameter('min_obstacle_points', 3)
        self.declare_parameter('scan_range', 6.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('lidar_topic', '/niihan/sensors/lidar/points')
        self.declare_parameter('footprint_radius', 4.0)
        self.declare_parameter('stale_timeout', 1.0)
        self.declare_parameter('vertical_fov_min', -0.12217)
        self.declare_parameter('vertical_fov_max', 0.90757)
        self.declare_parameter('robot_exclusion_radius', 0.45)
        self.declare_parameter('cliff_persistence', 3) # frames

        self._ground_thresh = self.get_parameter('ground_height_threshold').value
        self._obstacle_thresh = self.get_parameter('obstacle_height_threshold').value
        self._check_radius = self.get_parameter('cliff_check_radius').value
        self._cell_size = self.get_parameter('cell_size').value
        self._min_ground = self.get_parameter('min_ground_points').value
        self._min_obstacle = self.get_parameter('min_obstacle_points').value
        self._scan_range = self.get_parameter('scan_range').value
        self._pub_rate = self.get_parameter('publish_rate').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._footprint_radius = self.get_parameter('footprint_radius').value
        self._stale_timeout = self.get_parameter('stale_timeout').value
        self._cliff_persistence = self.get_parameter('cliff_persistence').value

        self._vertical_min = self.get_parameter('vertical_fov_min').value
        self._vertical_max = self.get_parameter('vertical_fov_max').value
        self._robot_exclusion = self.get_parameter('robot_exclusion_radius').value
        self._observable_cells = 0

        self._n_rays = 360
        self._angle_min = -math.pi
        self._angle_max = math.pi
        self._angle_inc = (self._angle_max - self._angle_min) / self._n_rays
        self._n_cells = int(math.ceil(2 * self._check_radius / self._cell_size))

        self._cliff_ranges = [float('inf')] * self._n_rays
        self._cliff_points = np.empty((0, 3), dtype=np.float32)
        self._ground_pct = 100.0

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._scan_pub = self.create_publisher(LaserScan, '/cliff_scan', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/cliff/obstacles', 10)
        self._status_pub = self.create_publisher(String, '/cliff/status', 10)

        self.create_subscription(PointCloud2, self.get_parameter('lidar_topic').value, self._cb_pointcloud, qos_profile_sensor_data)
        self.create_timer(1.0 / self._pub_rate, self._publish_cliff)

        self.last_cloud_time = None
        self.last_tf_valid = False

        self.cliff_evidence = np.zeros((self._n_cells, self._n_cells), dtype=np.int32)

    def _cb_pointcloud(self, msg: PointCloud2):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        self.last_tf_valid = False
        if stamp.nanoseconds <= 0 or not 0 <= age <= self._stale_timeout:
            self.cliff_evidence.fill(0)
            return
        try:
            tf = self._tf_buffer.lookup_transform(self._robot_frame, msg.header.frame_id, stamp)
            points = xyz_array(msg)
        except (TransformException, ValueError):
            return
        if not len(points):
            return
        if self.last_cloud_time is not None:
            gap = (stamp - self.last_cloud_time).nanoseconds / 1e9
            if gap <= 0:
                self.cliff_evidence.fill(0)
                return
            if gap > self._stale_timeout:
                self.cliff_evidence.fill(0)
        xs, ys, zs = points.T
        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        if not len(xs):
            return
        self.last_cloud_time = stamp
        self.last_tf_valid = True

        tx, ty, tz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
        qx, qy, qz, qw = tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w

        r00 = 1 - 2*(qy*qy + qz*qz); r01 = 2*(qx*qy - qz*qw); r02 = 2*(qx*qz + qy*qw)
        r10 = 2*(qx*qy + qz*qw); r11 = 1 - 2*(qx*qx + qz*qz); r12 = 2*(qy*qz - qx*qw)
        r20 = 2*(qx*qz - qy*qw); r21 = 2*(qy*qz + qx*qw); r22 = 1 - 2*(qx*qx + qy*qy)

        bx = r00*xs + r01*ys + r02*zs + tx
        by = r10*xs + r11*ys + r12*zs + ty
        bz = r20*xs + r21*ys + r22*zs + tz

        half = self._check_radius
        cs = self._cell_size
        n = self._n_cells

        xy_dist = np.sqrt(bx*bx + by*by)
        in_range = xy_dist <= self._check_radius
        bx_r, by_r, bz_r = bx[in_range], by[in_range], bz[in_range]

        ci = np.clip(((bx_r + half) / cs).astype(np.int32), 0, n - 1)
        cj = np.clip(((by_r + half) / cs).astype(np.int32), 0, n - 1)

        ground_grid = np.zeros((n, n), dtype=np.int32)
        obstacle_grid = np.zeros((n, n), dtype=np.int32)

        is_ground = bz_r < self._ground_thresh
        is_obstacle = bz_r >= self._obstacle_thresh

        np.add.at(ground_grid, (ci[is_ground], cj[is_ground]), 1)
        np.add.at(obstacle_grid, (ci[is_obstacle], cj[is_obstacle]), 1)

        has_ground = ground_grid >= self._min_ground
        has_obstacle = obstacle_grid >= self._min_obstacle

        # Build valid footprint grid
        x, y = np.ogrid[0:n, 0:n]
        cx, cy = (x * cs - half + cs / 2), (y * cs - half + cs / 2)
        cell_dist = np.sqrt(cx*cx + cy*cy)
        rotation = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        observable = ground_observation_mask(
            cx, cy, (tx, ty, tz), rotation, self._vertical_min,
            self._vertical_max, self._robot_exclusion)
        in_footprint = (cell_dist <= self._footprint_radius) & observable
        self._observable_cells = int(np.sum(in_footprint))

        # Cliff: inside footprint, NO ground, NO obstacle
        is_cliff_instant = in_footprint & ~has_ground & ~has_obstacle

        self.cliff_evidence[is_cliff_instant] += 1
        self.cliff_evidence[~is_cliff_instant] = 0 # reset if observed ground/obstacle or outside footprint

        confirmed_cliff = self.cliff_evidence >= self._cliff_persistence

        self._ground_pct = 100.0 * np.sum(has_ground & in_footprint) / max(1, self._observable_cells)

        ranges = [float('inf')] * self._n_rays
        cliff_pts_list = []
        cliff_is, cliff_js = np.where(confirmed_cliff)

        for ci_val, cj_val in zip(cliff_is, cliff_js):
            cx_val = ci_val * cs - half + cs / 2
            cy_val = cj_val * cs - half + cs / 2
            dist = math.sqrt(cx_val*cx_val + cy_val*cy_val)
            if dist < 0.3 or dist > self._scan_range: continue

            angle = math.atan2(cy_val, cx_val)
            ray_idx = int((angle - self._angle_min) / self._angle_inc)
            ray_idx = max(0, min(self._n_rays - 1, ray_idx))
            if dist < ranges[ray_idx]:
                ranges[ray_idx] = dist
            cliff_pts_list.append([cx_val, cy_val, 0.0])

        self._cliff_ranges = ranges
        if cliff_pts_list:
            self._cliff_points = np.array(cliff_pts_list, dtype=np.float32)
        else:
            self._cliff_points = np.empty((0, 3), dtype=np.float32)

    def _publish_cliff(self):
        now = self.get_clock().now()

        # Freshness handling
        is_stale = False
        if self.last_cloud_time is None:
            is_stale = True
        else:
            dt = (now - self.last_cloud_time).nanoseconds / 1e9
            if not 0 <= dt <= self._stale_timeout:
                is_stale = True

        if not self.last_tf_valid:
            is_stale = True

        status = String()
        if is_stale:
            # Publish fault status and DO NOT publish stale cliff scans
            status.data = "FAULT: Stale pointcloud or TF lookup failure."
            self._status_pub.publish(status)
            return

        # Preserve acquisition time: old body-frame evidence must not move with the robot.
        stamp = self.last_cloud_time.to_msg()
        scan = LaserScan()
        scan.header = Header(frame_id=self._robot_frame, stamp=stamp)
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self._pub_rate
        scan.range_min = 0.3
        scan.range_max = self._scan_range
        scan.ranges = [r if r != float('inf') else float('inf') for r in self._cliff_ranges]

        self._scan_pub.publish(scan)

        if len(self._cliff_points) > 0:
            cloud = _make_pointcloud2(self._cliff_points, self._robot_frame, stamp)
            self._cloud_pub.publish(cloud)

        n_cliff = sum(1 for r in self._cliff_ranges if r < self._scan_range)
        status.data = (f'ground_coverage={self._ground_pct:.0f}% cliff_rays={n_cliff}/{self._n_rays}'
                       if self._observable_cells else
                       'UNKNOWN: Ground within cliff-check radius is outside lidar field of view.')
        self._status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = CliffDetector()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
