"""Arrival verification, cancellation races and mission input regressions."""
from types import SimpleNamespace as NS
import threading
from concurrent.futures import Future

import pytest
from action_msgs.msg import GoalStatus
from niihan_dashboard.ros_bridge import ROSBridgeNode
from niihan_dashboard.mission_manager import MissionManager
from niihan_dashboard.geofence_manager import GeofenceManager


def future(value):
    f = Future()
    f.set_result(value)
    return f


def bridge():
    node = ROSBridgeNode.__new__(ROSBridgeNode)
    node._state_lock = threading.RLock()
    node.mission_manager = MissionManager()
    node.mission_manager.add_waypoint(2, 3, yaw=0)
    node.mission_manager.add_waypoint(4, 5, yaw=0)
    node.mission_manager.write_mission()
    node.mission_manager.start_mission()
    node.telemetry = {'pose': {'x': 2, 'y': 3, 'z': -0.6, 'yaw': 0}}
    node.goal_verify_position_tolerance = 0.2
    node.navigation = {'state': 'RUNNING', 'message': ''}
    node._update_pose_from_tf = lambda: True
    operation = {'kind': 'mission', 'targets': node.mission_manager.waypoints.copy(),
                 'index': 0, 'target': {'x': 2, 'y': 3, 'z': -0.6, 'yaw': 0},
                 'request_id': 1, 'cancelling': False}
    node._active_navigation = operation
    node.get_logger = lambda: NS(info=lambda message: None, error=lambda message: None)
    node.dispatched = []
    node._dispatch_target = lambda op: node.dispatched.append(op['index'])
    return node, operation


def test_success_far_from_target_cannot_complete_or_advance_mission():
    node, operation = bridge()
    node.telemetry['pose'].update(x=0, y=0)
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.mission_manager.state == 'FAILED'
    assert node.mission_manager.waypoints[0]['status'] == 'FAILED'
    assert node.mission_manager.waypoints[1]['status'] == 'PENDING'
    assert not node.dispatched


def test_success_at_one_target_only_advances_to_next_target():
    node, operation = bridge()
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.mission_manager.state == 'RUNNING'
    assert node.mission_manager.waypoints[0]['status'] == 'COMPLETED'
    assert node.mission_manager.waypoints[1]['status'] == 'ACTIVE'
    assert node.dispatched == [1]


def test_old_goal_result_cannot_overwrite_newer_mission():
    node, operation = bridge()
    operation['request_id'] = 2
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.mission_manager.waypoints[0]['status'] == 'ACTIVE'
    assert not node.dispatched


def test_aborted_goal_never_counts_as_reached():
    node, operation = bridge()
    node._goal_result(future(NS(status=GoalStatus.STATUS_ABORTED)), operation, 1)
    assert node.mission_manager.state == 'FAILED'
    assert not node.dispatched


def test_cancel_while_acceptance_pending_cancels_late_handle():
    node, operation = bridge()
    operation.update(cancelling=True, cancel_sent=False)
    cancels = []
    results = Future()
    handle = NS(accepted=True, get_result_async=lambda: results,
                cancel_goal_async=lambda: cancels.append(True))
    node._goal_response(future(handle), operation, 1)
    assert cancels == [True]
    assert node.navigation['state'] == 'RUNNING'  # no success/advance from acceptance
    assert not node.dispatched


def test_success_with_stale_pose_fails_verification():
    node, operation = bridge()
    node._update_pose_from_tf = lambda: False
    stops = []
    node._trigger_estop = stops.append
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert stops
    assert node.mission_manager.state == 'FAILED'
    assert not node.dispatched


@pytest.mark.parametrize('value', [float('nan'), float('inf'), None, 'invalid'])
def test_invalid_coordinates_rejected(value):
    manager = MissionManager()
    with pytest.raises(ValueError):
        manager.add_waypoint(value, 0)
    assert not manager.waypoints


def test_running_mission_cannot_be_edited():
    node, operation = bridge()
    with pytest.raises(ValueError):
        node.mission_manager.delete_waypoint(1)
    assert len(node.mission_manager.waypoints) == 2


def test_geofence_raster_blocks_outside_and_respects_rotated_origin():
    fence = GeofenceManager()
    assert fence.set_geofence([{'x': 0, 'y': 0}, {'x': 4, 'y': 0},
                              {'x': 4, 'y': 4}, {'x': 0, 'y': 4}])[0]
    mask = fence.keepout_mask(6, 6, 1, -1, -1, 0, 0)
    assert mask[3*6+3] == 0
    assert mask[0] == 100
    rotated = fence.keepout_mask(6, 6, 1, 5, -1, 1.5707963267948966, 0)
    assert rotated[3*6+3] == 0
    assert rotated[0] == 100
    fence.clear_geofence()
    assert fence.keepout_mask(6, 6, 1, 0, 0, 0, 0.45) == [0]*36


def test_concave_fence_allows_valid_waypoints_for_planner_detour():
    fence = GeofenceManager()
    assert fence.set_geofence([{'x': 0, 'y': 0}, {'x': 6, 'y': 0}, {'x': 6, 'y': 6},
                              {'x': 4, 'y': 6}, {'x': 4, 'y': 2}, {'x': 2, 'y': 2},
                              {'x': 2, 'y': 6}, {'x': 0, 'y': 6}])[0]
    assert fence.is_valid_mission([{'x': 1, 'y': 4}, {'x': 5, 'y': 4}])[0]
    mask = fence.keepout_mask(6, 6, 1, 0, 0, 0, 0)
    assert mask[4*6+3] == 100


def test_display_cloud_conversion_is_bounded_before_python_iteration():
    import numpy as np
    from sensor_msgs.msg import PointCloud2, PointField
    node = NS(last_pc_time=0.0, _state_lock=threading.RLock(), pc_generation=0,
              get_logger=lambda: NS(warning=lambda message: None))
    msg = PointCloud2()
    msg.header.frame_id='map';msg.header.stamp.sec=7
    requested=[]
    def lookup(parent,child,stamp):
        requested.append(stamp.nanoseconds)
        return NS(transform=NS(translation=NS(x=1.,y=2.,z=3.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    node.tf_buffer=NS(lookup_transform=lookup);node.global_frame='map';node.base_frame='base_footprint'
    msg.height, msg.width = 1, 100_000
    msg.point_step, msg.row_step = 12, 1_200_000
    msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                  for i,n in enumerate(('x','y','z'))]
    msg.data = np.zeros((100_000,3), dtype=np.float32).tobytes()
    ROSBridgeNode.pointcloud_callback(node, msg)
    assert len(node.pointcloud_data) <= 30_000
    assert node.pc_generation == 1
    assert requested == [7_000_000_000]
    assert node.pointcloud_metadata['stamp']==node.pointcloud_metadata['pose_stamp']==7.
    assert node.pointcloud_metadata['pose']['x']==1.

@pytest.mark.parametrize('ros_now,wall_now,expected', [(10.2, 101.5, True), (11.1, 101.5, False), (10.2, 103.1, False)])
def test_pose_age_and_paused_clock_have_separate_limits(monkeypatch, ros_now, wall_now, expected):
    """Slow simulation stays valid; old ROS data and a frozen clock still stop."""
    import niihan_dashboard.ros_bridge as module
    node = ROSBridgeNode.__new__(ROSBridgeNode)
    node.telemetry = {'pose': {}, 'localization': {}}
    node.pose_max_age = 1.0
    node.pose_freeze_timeout = 3.0
    node.pose_future_tolerance = 0.1
    node._last_ros_time = 10.0
    node._last_tf_stamp = 10.0
    node._last_tf_advance = 100.0
    node.global_frame, node.base_frame = 'map', 'base_footprint'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(ros_now * 1e9)))
    transform = NS(header=NS(stamp=NS(sec=10, nanosec=0)), transform=NS(
        translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    node.tf_buffer = NS(lookup_transform=lambda *args: transform)
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall_now)
    assert node._update_pose_from_tf() is expected


def test_matching_xyz_completes_regardless_of_yaw():
    node, operation = bridge()
    node.telemetry['pose']['yaw'] = 2.8
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.dispatched == [1]


def test_wrong_z_cannot_complete_even_when_xy_matches():
    node, operation = bridge()
    node.telemetry['pose']['z'] += 0.4
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.mission_manager.state == 'FAILED'
    assert not node.dispatched


def test_pause_does_not_complete_or_advance_when_late_success_arrives():
    node, operation = bridge()
    operation.update(pausing=True, cancelling=True)
    node._goal_result(future(NS(status=GoalStatus.STATUS_SUCCEEDED)), operation, 1)
    assert node.mission_manager.state == 'PAUSED'
    assert node._active_navigation is operation
    assert operation['paused'] and not node.dispatched


def test_cancel_paused_mission_clears_operation_without_waiting_for_result():
    node, operation = bridge()
    operation.update(paused=True, handle=None)
    node.cmd_vel_pub = NS(publish=lambda message: None)
    node._stop_navigation('Cancelled')
    assert node.mission_manager.state == 'CANCELLED'
    assert node._active_navigation is None


def test_action_endpoint_does_not_imply_active_or_fresh_nav2():
    import time
    node, _ = bridge()
    node.nav_to_pose_client = NS(server_is_ready=lambda: True)
    now = time.monotonic()
    node._nav2_lifecycle = {name:{'active':False,'received':now,'requested':now,'future':None}
                           for name in ('controller','planner','navigator','behavior','smoother')}
    assert node.check_nav2() is False
    for state in node._nav2_lifecycle.values():state['active'] = True
    assert node.check_nav2() is True
    node._nav2_lifecycle['controller']['received'] = now-5
    assert node.check_nav2() is False
