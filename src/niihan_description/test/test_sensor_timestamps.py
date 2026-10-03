"""Regression tests for acquisition-time TF, cloud layouts and command expiry."""
from collections import deque
from types import SimpleNamespace as NS
import struct
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import ClockType
from rclpy.time import Time as RclTime


def Time(*args, **kwargs):
    return RclTime(*args, clock_type=ClockType.ROS_TIME, **kwargs)
from sensor_msgs.msg import PointCloud2, PointField

from niihan_description.point_cloud_utils import xyz_array, ground_observation_mask
from niihan_description.vortex_3d_mapper import Vortex3DMapper
from niihan_description.command_arbiter import CommandArbiter
from niihan_description.cliff_detector import CliffDetector


def cloud(stamp=10.0):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar'
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.height = 1
    msg.width = 1
    msg.point_step = msg.row_step = 12
    msg.fields = [PointField(name=name, offset=4*i, datatype=7, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.data = struct.pack('<fff', 1, 2, 3)
    return msg


def test_cloud_reads_big_endian_padded_organized_rows():
    msg = cloud()
    msg.height = 2
    msg.row_step = 16
    msg.is_bigendian = True
    msg.data = struct.pack('>fff', 1, 2, 3) + b'pad!' + struct.pack('>fff', 4, 5, 6) + b'pad!'
    np.testing.assert_equal(xyz_array(msg), [[1, 2, 3], [4, 5, 6]])


@pytest.mark.parametrize('field', ['point_step', 'row_step', 'data'])
def test_malformed_cloud_rejected(field):
    msg = cloud()
    setattr(msg, field, b'' if field == 'data' else 0)
    with pytest.raises(ValueError):
        xyz_array(msg)


def mapper(now=10.1):
    return NS(_pending_clouds=deque(maxlen=5), _last_clock_stamp=None,
              _voxels={1}, _scan_count=1, _transform_timeout=0.5,
              get_clock=lambda: NS(now=lambda: Time(seconds=now)))


def test_mapper_uses_acquisition_timestamp_not_latest_tf():
    node = mapper()
    msg = cloud()
    Vortex3DMapper._cb_pointcloud(node, msg)
    calls = []
    node._map_frame = 'map'
    node._tf_buffer = NS(lookup_transform=lambda target, source, stamp: calls.append(stamp) or 'tf')
    node._integrate_cloud = lambda msg, tf: calls.append(tf)
    Vortex3DMapper._process_pending_clouds(node)
    assert calls[0].nanoseconds == 10_000_000_000
    assert calls[1] == 'tf'
    assert not node._pending_clouds


def test_mapper_drops_old_cloud_instead_of_relabeling_it_fresh():
    node = mapper(now=11.0)
    Vortex3DMapper._cb_pointcloud(node, cloud())
    Vortex3DMapper._process_pending_clouds(node)
    assert not node._pending_clouds


def test_out_of_order_cloud_does_not_erase_map_but_clock_reset_does():
    node = mapper()
    Vortex3DMapper._cb_pointcloud(node, cloud(10.0))
    Vortex3DMapper._cb_pointcloud(node, cloud(9.9))
    assert node._voxels == {1}
    node.get_clock = lambda: NS(now=lambda: Time(seconds=1.0))
    Vortex3DMapper._cb_pointcloud(node, cloud(1.0))
    assert not node._voxels
    assert len(node._pending_clouds) == 1


def test_cliff_invalid_cloud_does_not_refresh_safety_evidence():
    node = NS(get_clock=lambda: NS(now=lambda: Time(seconds=10.1)),
              _stale_timeout=1.0, last_tf_valid=True,
              last_cloud_time=Time(seconds=9.8), cliff_evidence=np.ones((2, 2)))
    CliffDetector._cb_pointcloud(node, cloud(2.0))
    assert not node.last_tf_valid
    assert node.last_cloud_time.nanoseconds == 9_800_000_000
    assert not node.cliff_evidence.any()


def test_arbiter_expires_command_on_steady_time_and_drops_pre_stop_commands():
    published = []
    cmd = Twist()
    cmd.linear.x = 0.5
    node = NS(estop_active=False, cmds={3: None, 2: None, 1: cmd},
              cmd_times={3: None, 2: None, 1: Time(seconds=5)},
              _watchdog_clock=NS(now=lambda: Time(seconds=6)),
              watchdog_timeout=0.5, cmd_pub=NS(publish=published.append))
    CommandArbiter.evaluate_commands(node)
    assert published[-1].linear.x == 0
    CommandArbiter.estop_cb(node, NS(data=True))
    CommandArbiter.estop_cb(node, NS(data=False))
    assert not any(node.cmds.values())


def test_mast_blind_ground_is_not_classified_as_a_cliff():
    x, y = np.meshgrid(np.linspace(-4, 4, 17), np.linspace(-4, 4, 17))
    observed = ground_observation_mask(x, y, (0, 0, 1.5), np.eye(3),
                                       -0.12217, 0.90757, 0.45)
    assert not observed.any()


def test_downward_lidar_observes_ground_but_excludes_robot_body():
    x = np.array([0.0, 1.0, 2.0])
    y = np.zeros(3)
    observed = ground_observation_mask(x, y, (0, 0, 1.5), np.eye(3),
                                       -1.57, 0.0, 0.45)
    np.testing.assert_equal(observed, [False, True, True])
