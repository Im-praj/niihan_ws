#!/usr/bin/env python3
"""Vortex 3D Point Cloud Mapper for NIIHAN.

Subscribes to the 3D mast LiDAR point cloud, transforms each scan into
the ``map`` frame, accumulates points in a voxel grid, and publishes
the resulting 3D map as a PointCloud2 message.

This replaces the basic OctoMap voxel-grid approach with a full 3D
point cloud map that captures the environment geometry.
"""

import math
import os
import struct
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException


def _make_pointcloud2(points_xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Create a PointCloud2 from an Nx3 float32 array."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.height = 1
    msg.width = len(points_xyz)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12  # 3 × float32
    msg.row_step = msg.point_step * msg.width

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    msg.data = points_xyz.astype(np.float32).tobytes()
    return msg


class Vortex3DMapper(Node):
    """Accumulates 3D LiDAR scans into a voxel-filtered point cloud map."""

    def __init__(self):
        super().__init__('vortex_3d_mapper')

        # ── Parameters ───────────────────────────────────────────────
        try: self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException: pass
        except rclpy.exceptions.ParameterAlreadyDeclaredException: pass
        self.declare_parameter('voxel_resolution', 0.05)
        self.declare_parameter('max_range', 25.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('save_interval', 30.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('max_points', 5_000_000)
        self.declare_parameter('downsample_resolution', 0.15)

        self._voxel_res = self.get_parameter('voxel_resolution').value
        self._max_range = self.get_parameter('max_range').value
        self._min_range = self.get_parameter('min_range').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._save_interval = self.get_parameter('save_interval').value
        self._map_frame = self.get_parameter('map_frame').value
        self._max_points = self.get_parameter('max_points').value
        self._ds_res = self.get_parameter('downsample_resolution').value

        # ── Voxel storage ────────────────────────────────────────────
        # Store occupied voxel indices as a set of (ix, iy, iz) tuples
        self._voxels: set[tuple[int, int, int]] = set()
        self._scan_count: int = 0
        self._last_save_time: float = time.time()

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publishers ───────────────────────────────────────────────
        self._map_pub = self.create_publisher(
            PointCloud2, '/vortex/point_cloud_map', 10)
        self._ds_pub = self.create_publisher(
            PointCloud2, '/vortex/point_cloud_map_downsampled', 10)
        self._status_pub = self.create_publisher(
            String, '/vortex/status', 10)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            PointCloud2,
            '/niihan/sensors/lidar/points',
            self._cb_pointcloud,
            10,
        )

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(1.0 / self._publish_rate, self._publish_map)
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f'Vortex3DMapper started  voxel_res={self._voxel_res}m  '
            f'range=[{self._min_range}, {self._max_range}]m')

    # ================================================================
    # Point cloud callback
    # ================================================================
    def _cb_pointcloud(self, msg: PointCloud2):
        self.get_logger().info(f"Got cloud! frame_id={msg.header.frame_id}")
        """Transform incoming point cloud into map frame and add to voxel set."""
        # Look up transform from sensor frame to map
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),   # latest available
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed ({msg.header.frame_id} → '
                f'{self._map_frame}): {e}',
                throttle_duration_sec=5.0)
            self.get_logger().info("Returning early"); return

        # Extract x, y, z offsets from PointCloud2 fields
        field_map = {f.name: f.offset for f in msg.fields}
        if not all(k in field_map for k in ('x', 'y', 'z')):
            self.get_logger().info("Returning early"); return

        x_off = field_map['x']
        y_off = field_map['y']
        z_off = field_map['z']
        step = msg.point_step
        data = bytes(msg.data)
        n_pts = len(data) // step

        if n_pts == 0:
            self.get_logger().info("Returning early"); return

        # Fast extraction via numpy
        raw = np.frombuffer(data, dtype=np.uint8).reshape(n_pts, step)
        xs = np.frombuffer(raw[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(raw[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(raw[:, z_off:z_off + 4].tobytes(), dtype=np.float32)

        # Filter invalid / out-of-range points
        dist = np.sqrt(xs * xs + ys * ys + zs * zs)
        mask = (
            np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs) &
            (dist >= self._min_range) & (dist <= self._max_range)
        )
        xs, ys, zs = xs[mask], ys[mask], zs[mask]

        if len(xs) == 0:
            self.get_logger().info("Returning early"); return

        # ── Transform to map frame ──────────────────────────────────
        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion → rotation matrix (manual for no tf_transformations dep)
        tx, ty, tz = t.x, t.y, t.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Rotation matrix from quaternion
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)

        mx = r00 * xs + r01 * ys + r02 * zs + tx
        my = r10 * xs + r11 * ys + r12 * zs + ty
        mz = r20 * xs + r21 * ys + r22 * zs + tz

        # ── Voxelise and insert ──────────────────────────────────────
        inv = 1.0 / self._voxel_res
        ix = np.round(mx * inv).astype(np.int32)
        iy = np.round(my * inv).astype(np.int32)
        iz = np.round(mz * inv).astype(np.int32)

        new_voxels = set(zip(ix.tolist(), iy.tolist(), iz.tolist()))
        self._voxels |= new_voxels
        self._scan_count += 1

        # Trim if over budget
        if len(self._voxels) > self._max_points:
            self.get_logger().warn(
                f'Voxel count {len(self._voxels)} exceeds max {self._max_points}, '
                f'downsampling…')
            self._downsample_voxels()

        # Periodic save
        now = time.time()
        if now - self._last_save_time > self._save_interval:
            self._save_pcd()
            self._last_save_time = now

    # ================================================================
    # Publish accumulated map
    # ================================================================
    def _publish_map(self):
        """Publish the full and downsampled 3D point cloud map."""
        if not self._voxels:
            self.get_logger().info("Returning early"); return

        stamp = self.get_clock().now().to_msg()

        # Full map
        pts = self._voxels_to_array(self._voxels, self._voxel_res)
        pc_msg = _make_pointcloud2(pts, self._map_frame, stamp)
        self._map_pub.publish(pc_msg)

        # Downsampled
        ds_inv = 1.0 / self._ds_res
        ds_set: set[tuple[int, int, int]] = set()
        for v in self._voxels:
            ds_set.add((
                int(round(v[0] * self._voxel_res * ds_inv)),
                int(round(v[1] * self._voxel_res * ds_inv)),
                int(round(v[2] * self._voxel_res * ds_inv)),
            ))
        ds_pts = self._voxels_to_array(ds_set, self._ds_res)
        ds_msg = _make_pointcloud2(ds_pts, self._map_frame, stamp)
        self._ds_pub.publish(ds_msg)

    @staticmethod
    def _voxels_to_array(voxels: set, resolution: float) -> np.ndarray:
        """Convert voxel set to Nx3 float32 array."""
        if not voxels:
            return np.empty((0, 3), dtype=np.float32)
        arr = np.array(list(voxels), dtype=np.float32)
        arr *= resolution
        return arr

    def _downsample_voxels(self):
        """Reduce voxel count by increasing effective resolution."""
        factor = 2
        new_voxels: set[tuple[int, int, int]] = set()
        for vx, vy, vz in self._voxels:
            new_voxels.add((vx // factor, vy // factor, vz // factor))
        self._voxel_res *= factor
        self._voxels = new_voxels
        self.get_logger().info(
            f'Downsampled to {len(self._voxels)} voxels, '
            f'new resolution {self._voxel_res:.3f}m')

    # ================================================================
    # Save PCD
    # ================================================================
    def _save_pcd(self):
        """Save the current 3D map as an ASCII PCD file."""
        save_dir = os.path.expanduser('~/.ros/niihan_maps')
        os.makedirs(save_dir, exist_ok=True)
        pcd_path = os.path.join(save_dir, '3d_map.pcd')

        pts = self._voxels_to_array(self._voxels, self._voxel_res)
        n = len(pts)
        if n == 0:
            self.get_logger().info("Returning early"); return

        try:
            with open(pcd_path, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z\n')
                f.write('SIZE 4 4 4\n')
                f.write('TYPE F F F\n')
                f.write('COUNT 1 1 1\n')
                f.write(f'WIDTH {n}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {n}\n')
                f.write('DATA ascii\n')
                for p in pts:
                    f.write(f'{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n')
            self.get_logger().info(
                f'Saved 3D map ({n} points) to {pcd_path}')
        except OSError as e:
            self.get_logger().error(f'Failed to save PCD: {e}')

    # ================================================================
    # Logging
    # ================================================================
    def _log_stats(self):
        """Periodically log mapping statistics."""
        status = (
            f'voxels={len(self._voxels)}  '
            f'scans={self._scan_count}  '
            f'res={self._voxel_res:.3f}m'
        )
        self.get_logger().info(f'Vortex3D: {status}')

        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


# ====================================================================
# Entry point
# ====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = Vortex3DMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Vortex3DMapper.')
    finally:
        node._save_pcd()  # final save
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
