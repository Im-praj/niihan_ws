"""Exercise the real detector rather than placeholder safety assertions."""
from types import SimpleNamespace as NS
import numpy as np
from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from niihan_description.cliff_detector import CliffDetector


def cloud(points, stamp):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar'
    msg.header.stamp = Time(seconds=stamp, clock_type=ClockType.ROS_TIME).to_msg()
    msg.height, msg.width = 1, len(points)
    msg.point_step, msg.row_step = 12, 12*len(points)
    msg.fields = [PointField(name=n, offset=4*i, datatype=7, count=1)
                  for i,n in enumerate(('x','y','z'))]
    msg.data = np.asarray(points, dtype=np.float32).reshape(-1,3).tobytes()
    return msg


def detector():
    transform = NS(transform=NS(translation=NS(x=0,y=0,z=1), rotation=NS(x=0,y=0,z=0,w=1)))
    output = []
    node = NS(_stale_timeout=1, _robot_frame='base_footprint', last_tf_valid=False,
              last_cloud_time=None, _check_radius=2, _cell_size=0.5, _n_cells=8,
              _ground_thresh=0.15, _obstacle_thresh=0.3, _min_ground=1, _min_obstacle=1,
              _footprint_radius=1.5, _vertical_min=-1.57, _vertical_max=0,
              _robot_exclusion=0.45, _cliff_persistence=3, _n_rays=360,
              _angle_min=-np.pi, _angle_max=np.pi, _angle_inc=2*np.pi/360,
              _scan_range=2.0, _pub_rate=5.0, _cliff_ranges=[float('inf')]*360,
              _cliff_points=np.empty((0,3), dtype=np.float32), _ground_pct=0,
              cliff_evidence=np.zeros((8,8), dtype=np.int32), _observable_cells=0,
              _tf_buffer=NS(lookup_transform=lambda *args: transform),
              get_clock=lambda: NS(now=lambda: Time(seconds=10.3, clock_type=ClockType.ROS_TIME)),
              _status_pub=NS(publish=output.append), _scan_pub=NS(publish=output.append),
              _cloud_pub=NS(publish=output.append), output=output)
    return node


def floor():
    return [[x,y,-1] for x in np.arange(-1.75,2,0.5) for y in np.arange(-1.75,2,0.5)]


def test_flat_ground_has_no_cliff_and_all_observable_ground_is_covered():
    node = detector()
    for stamp in (10.0,10.1,10.2):
        CliffDetector._cb_pointcloud(node, cloud(floor(),stamp))
    assert not np.isfinite(node._cliff_ranges).any()
    assert node._ground_pct == 100


def test_drop_requires_multiple_consecutive_observations():
    node = detector()
    points = [p for p in floor() if p[:2] != [0.75,0.25]]
    for stamp in (10.0,10.1):
        CliffDetector._cb_pointcloud(node, cloud(points,stamp))
        assert not np.isfinite(node._cliff_ranges).any()
    CliffDetector._cb_pointcloud(node, cloud(points,10.2))
    assert np.isfinite(node._cliff_ranges).any()


def test_body_and_vertical_blind_zone_never_create_synthetic_obstacles():
    node = detector()
    node._vertical_min = -0.12217
    node._vertical_max = 0.90757
    for stamp in (10.0,10.1,10.2):
        CliffDetector._cb_pointcloud(node, cloud([[1,0,1]],stamp))
    assert node._observable_cells == 0
    assert not np.isfinite(node._cliff_ranges).any()
    CliffDetector._publish_cliff(node)
    assert 'UNKNOWN' in node.output[-1].data


def test_empty_cloud_invalidates_evidence_without_claiming_fresh_ground():
    node = detector()
    CliffDetector._cb_pointcloud(node, cloud(floor(),10.0))
    previous = node.last_cloud_time
    CliffDetector._cb_pointcloud(node, cloud([],10.2))
    assert not node.last_tf_valid
    assert node.last_cloud_time == previous
    CliffDetector._publish_cliff(node)
    assert len(node.output) == 1
    assert node.output[0].data.startswith('FAULT')


def test_scan_keeps_acquisition_time_and_stale_evidence_is_not_published():
    node = detector()
    CliffDetector._cb_pointcloud(node, cloud(floor(),10.0))
    CliffDetector._publish_cliff(node)
    assert node.output[0].header.stamp.sec == 10
    assert node.output[0].header.stamp.nanosec == 0
    node.output.clear()
    node.get_clock = lambda: NS(now=lambda: Time(seconds=12, clock_type=ClockType.ROS_TIME))
    CliffDetector._publish_cliff(node)
    assert len(node.output) == 1
    assert node.output[0].data.startswith('FAULT')
