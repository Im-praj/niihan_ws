from collections import deque
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
import niihan_description.scan_tf_gate as module


def gate(monkeypatch, now=10.1):
    node = module.ScanTFGate.__new__(module.ScanTFGate)
    node.pending = deque(maxlen=5)
    node.last_ros_time = None
    node.odom_frame, node.base_frame = 'odom', 'base_footprint'
    node.max_scan_age, node.max_wait_wall, node.tf_delivery_margin = .5, 2., .05
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(now * 1e9)))
    node.tf_buffer = NS(lookup_transform=lambda *args: None)
    published = []
    node.publisher = NS(publish=published.append)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    return node, published


def scan(stamp=10):
    msg = LaserScan()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'lidar'
    return msg


def test_waits_for_exact_tf_and_preserves_acquisition_stamp(monkeypatch):
    node, published = gate(monkeypatch)
    msg = scan()
    node.receive(msg)
    calls = []
    def unavailable(*args):
        calls.append(args)
        raise TransformException('future TF')
    node.tf_buffer.lookup_transform = unavailable
    node.flush()
    assert not published and len(node.pending) == 1
    node.tf_buffer.lookup_transform = lambda *args: calls.append(args)
    node.flush()
    assert published == [msg] and published[0].header.stamp.sec == 10
    assert calls[-2][2].nanoseconds == 10_000_000_000
    assert calls[-1][2].nanoseconds == 10_050_000_000


@pytest.mark.parametrize('now,received', [(10.6,100.), (10.1,97.), (10.1,100.)])
def test_expired_or_invalid_scans_are_dropped(monkeypatch, now, received):
    node, published = gate(monkeypatch,now)
    node.pending.append((scan(0 if now == 10.1 and received == 100. else 10), received))
    node.flush()
    assert not published and not node.pending


def test_clock_reset_flushes_previous_run_and_queue_is_bounded(monkeypatch):
    node, published = gate(monkeypatch)
    for _ in range(20):node.receive(scan())
    assert len(node.pending) == 5
    node.last_ros_time = 20.
    node.flush()
    assert not published and not node.pending
