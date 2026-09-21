#!/usr/bin/env python3
"""Cliff / No-Land Detector for NIIHAN.

Analyses the 3D mast LiDAR point cloud to detect areas around the robot
that lack ground.  Areas without ground returns are treated as cliffs or
voids and are published as:

  * ``/cliff_scan``       – Synthetic LaserScan with cliff edges as obstacles
                            (consumed by Nav2 costmap as an obstacle source).
  * ``/cliff/obstacles``  – PointCloud2 visualisation of the cliff boundary.
  * ``/cliff/status``     – Human-readable ground-coverage status.
"""

import math
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException


def _make_pointcloud2(points_xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Create a PointCloud2 from an Nx3 float32 array."""
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
    """Detect cliff / no-ground areas and publish virtual obstacles."""

    def __init__(self):
        super().__init__('cliff_detector')

        # ── Parameters ───────────────────────────────────────────────
        try: self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException: pass
        except rclpy.exceptions.ParameterAlreadyDeclaredException: pass
        self.declare_parameter('ground_height_threshold', 0.15)
        self.declare_parameter('cliff_check_radius', 6.0)
        self.declare_parameter('cell_size', 0.5)
        self.declare_parameter('min_ground_points', 3)
        self.declare_parameter('scan_range', 6.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('lidar_topic', '/niihan/sensors/lidar/points')

        self._ground_thresh = self.get_parameter('ground_height_threshold').value
        self._check_radius = self.get_parameter('cliff_check_radius').value
        self._cell_size = self.get_parameter('cell_size').value
        self._min_ground = self.get_parameter('min_ground_points').value
        self._scan_range = self.get_parameter('scan_range').value
        self._pub_rate = self.get_parameter('publish_rate').value
        self._robot_frame = self.get_parameter('robot_frame').value

        # ── Precompute scan geometry ────────────────────────────────
        self._n_rays = 360
        self._angle_min = -math.pi
        self._angle_max = math.pi
        self._angle_inc = (self._angle_max - self._angle_min) / self._n_rays

        # ── Grid parameters ─────────────────────────────────────────
        self._n_cells = int(math.ceil(2 * self._check_radius / self._cell_size))

        # ── State ────────────────────────────────────────────────────
        self._cliff_ranges = [float('inf')] * self._n_rays
        self._cliff_points: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self._ground_pct: float = 100.0

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publishers ───────────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, '/cliff_scan', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/cliff/obstacles', 10)
        self._status_pub = self.create_publisher(String, '/cliff/status', 10)

        # ── Subscriber ───────────────────────────────────────────────
        self.create_subscription(
            PointCloud2,
            self.get_parameter('lidar_topic').value,
            self._cb_pointcloud,
            10,
        )

        # ── Timer ────────────────────────────────────────────────────
        self.create_timer(1.0 / self._pub_rate, self._publish_cliff)

        self.get_logger().info(
            f'CliffDetector started  radius={self._check_radius}m  '
            f'cell={self._cell_size}m  ground_thresh={self._ground_thresh}m')

    # ================================================================
    # Point cloud callback
    # ================================================================
    def _cb_pointcloud(self, msg: PointCloud2):
        """Analyse incoming point cloud for ground presence."""
        # Transform to robot base frame
        try:
            tf = self._tf_buffer.lookup_transform(
                self._robot_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        # ── Extract points ──────────────────────────────────────────
        field_map = {f.name: f.offset for f in msg.fields}
        if not all(k in field_map for k in ('x', 'y', 'z')):
            return

        x_off = field_map['x']
        y_off = field_map['y']
        z_off = field_map['z']
        step = msg.point_step
        data = bytes(msg.data)
        n_pts = len(data) // step
        if n_pts == 0:
            return

        raw = np.frombuffer(data, dtype=np.uint8).reshape(n_pts, step)
        xs = np.frombuffer(raw[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(raw[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(raw[:, z_off:z_off + 4].tobytes(), dtype=np.float32)

        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]

        # ── Transform to base_footprint ─────────────────────────────
        t = tf.transform.translation
        q = tf.transform.rotation
        tx, ty, tz = t.x, t.y, t.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)

        bx = r00 * xs + r01 * ys + r02 * zs + tx
        by = r10 * xs + r11 * ys + r12 * zs + ty
        bz = r20 * xs + r21 * ys + r22 * zs + tz

        # ── Build ground-presence grid ──────────────────────────────
        half = self._check_radius
        cs = self._cell_size
        n = self._n_cells

        # Filter to points within check radius on XY plane
        xy_dist = np.sqrt(bx * bx + by * by)
        in_range = xy_dist <= self._check_radius
        bx_r, by_r, bz_r = bx[in_range], by[in_range], bz[in_range]

        # Grid cell indices
        ci = ((bx_r + half) / cs).astype(np.int32)
        cj = ((by_r + half) / cs).astype(np.int32)
        ci = np.clip(ci, 0, n - 1)
        cj = np.clip(cj, 0, n - 1)

        # Count ground points per cell (z < threshold means ground)
        is_ground = bz_r < self._ground_thresh
        ground_grid = np.zeros((n, n), dtype=np.int32)
        total_grid = np.zeros((n, n), dtype=np.int32)

        # Use np.add.at for accumulation
        np.add.at(total_grid, (ci, cj), 1)
        np.add.at(ground_grid, (ci[is_ground], cj[is_ground]), 1)

        # ── Identify cliff cells ────────────────────────────────────
        # A cell is a cliff if it has been observed (total > 0) but lacks
        # enough ground points, OR if it hasn't been observed at all
        # and is within a reasonable range (we only mark observed no-ground)
        has_enough_obs = total_grid >= 2
        has_ground = ground_grid >= self._min_ground
        cliff_mask = has_enough_obs & ~has_ground

        # Compute ground coverage
        observed_cells = np.sum(has_enough_obs)
        ground_cells = np.sum(has_enough_obs & has_ground)
        if observed_cells > 0:
            self._ground_pct = 100.0 * ground_cells / observed_cells
        else:
            self._ground_pct = 100.0

        # ── Build cliff ranges for LaserScan ────────────────────────
        ranges = [float('inf')] * self._n_rays
        cliff_pts_list = []

        cliff_is, cliff_js = np.where(cliff_mask)
        for ci_val, cj_val in zip(cliff_is, cliff_js):
            # Cell center in robot frame
            cx = ci_val * cs - half + cs / 2
            cy = cj_val * cs - half + cs / 2
            dist = math.sqrt(cx * cx + cy * cy)
            if dist < 0.3 or dist > self._scan_range:
                continue

            angle = math.atan2(cy, cx)
            ray_idx = int((angle - self._angle_min) / self._angle_inc)
            ray_idx = max(0, min(self._n_rays - 1, ray_idx))

            # Set range to nearest cliff in this direction
            if dist < ranges[ray_idx]:
                ranges[ray_idx] = dist

            cliff_pts_list.append([cx, cy, 0.0])

        self._cliff_ranges = ranges
        if cliff_pts_list:
            self._cliff_points = np.array(cliff_pts_list, dtype=np.float32)
        else:
            self._cliff_points = np.empty((0, 3), dtype=np.float32)

    # ================================================================
    # Publish cliff obstacles
    # ================================================================
    def _publish_cliff(self):
        """Publish cliff scan and obstacle visualisation."""
        stamp = self.get_clock().now().to_msg()

        # ── LaserScan ────────────────────────────────────────────────
        scan = LaserScan()
        scan.header = Header(frame_id=self._robot_frame, stamp=stamp)
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self._pub_rate
        scan.range_min = 0.3
        scan.range_max = self._scan_range
        scan.ranges = [
            r if r != float('inf') else self._scan_range + 1.0
            for r in self._cliff_ranges
        ]
        # For Nav2: ranges > range_max are ignored (free space)
        # We set actual cliff distances within range
        self._scan_pub.publish(scan)

        # ── Obstacle PointCloud2 ─────────────────────────────────────
        if len(self._cliff_points) > 0:
            cloud = _make_pointcloud2(
                self._cliff_points, self._robot_frame, stamp)
            self._cloud_pub.publish(cloud)

        # ── Status ───────────────────────────────────────────────────
        status = String()
        n_cliff = sum(1 for r in self._cliff_ranges if r < self._scan_range)
        status.data = (
            f'ground_coverage={self._ground_pct:.0f}%  '
            f'cliff_rays={n_cliff}/{self._n_rays}'
        )
        self._status_pub.publish(status)


# ====================================================================
# Entry point
# ====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CliffDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CliffDetector.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
