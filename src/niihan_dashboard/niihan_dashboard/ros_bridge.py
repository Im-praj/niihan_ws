import base64
import copy
from functools import wraps
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, Image, PointCloud2, NavSatFix
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

from niihan_dashboard.safety_manager import SafetyManager
from niihan_dashboard.mission_manager import MissionManager
from niihan_dashboard.geofence_manager import GeofenceManager


def synchronized(method):
    """Websocket commands and ROS callbacks share mission/action state."""
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._state_lock:
            return method(self, *args, **kwargs)
    return locked


def euler_from_quaternion(x, y, z, w):
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class ROSBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_ros_bridge')
        self._state_lock = threading.RLock()
        for name, value in (('global_frame', 'map'), ('robot_base_frame', 'base_footprint'),
                            ('pose_max_age', 1.0), ('pose_future_tolerance', 0.1),
                            ('pose_freeze_timeout', 3.0), ('require_health', False), ('slam_cloud_topic', '/vortex/point_cloud_map'),
                            ('goal_verify_xy_tolerance', 0.25), ('goal_verify_yaw_tolerance', 0.30),
                            ('geofence_margin', 0.45)):
            self.declare_parameter(name, value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.pose_max_age = float(self.get_parameter('pose_max_age').value)
        # ROS age bounds pose accuracy; wall time separately detects a paused clock.
        self.pose_freeze_timeout = float(self.get_parameter('pose_freeze_timeout').value)
        self.pose_future_tolerance = float(self.get_parameter('pose_future_tolerance').value)
        self.goal_verify_xy_tolerance = float(self.get_parameter('goal_verify_xy_tolerance').value)
        self.goal_verify_yaw_tolerance = float(self.get_parameter('goal_verify_yaw_tolerance').value)
        self.geofence_margin = float(self.get_parameter('geofence_margin').value)
        self.geofence_manager = GeofenceManager()
        self.safety_manager = SafetyManager(geofence_manager=self.geofence_manager)
        self.mission_manager = MissionManager(geofence_manager=self.geofence_manager)
        self.cv_bridge = CvBridge()
        self.telemetry = {
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'velocity': {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0},
            'imu': {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'wx': 0.0, 'wy': 0.0, 'wz': 0.0},
            'localization': {'source': 'TF', 'health': 'UNAVAILABLE', 'confidence': None, 'age': None},
        }
        self.pose_valid = False
        self._last_tf_stamp = None
        self._last_tf_advance = None
        self._last_ros_time = None
        self.map_data = None
        self.map_generation = 0
        self._map_info = None
        self._map_geometry = None
        self.latest_image = None
        self.image_generation = 0
        self.path_history = []
        self.pointcloud_data = []
        self.pc_generation = 0
        self.last_pc_time = 0.0
        self._active_navigation = None
        self._next_request_id = 0
        self.navigation = {'state': 'IDLE', 'message': '', 'target': None}
        self.nav2_ready = False
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_vel_pub = self.create_publisher(Twist, '/niihan/cmd_vel/manual', 10)
        self.estop_pub = self.create_publisher(Bool, '/niihan/e_stop', latched)
        self.keepout_pub = self.create_publisher(OccupancyGrid, '/niihan/geofence_mask', latched)
        self.filter_info_pub = self.create_publisher(CostmapFilterInfo, '/niihan/geofence_filter_info', latched)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.telemetry['gnss'] = {'available': False, 'quality': 'unknown'}
        self.telemetry['hardware_health'] = {'ready': False, 'faults': ['waiting_for_supervisor']}
        self.create_subscription(NavSatFix, '/niihan/gnss/fix', self.gnss_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/niihan/gnss/quality', self.gnss_quality_callback, 10)
        self.create_subscription(String, '/niihan/health', self.hardware_health_callback, 10)
        self._health_received = 0.0
        self.telemetry['slam'] = {'backend': 'unknown'}
        self.create_subscription(String, '/niihan/slam/status', self.slam_status_callback, 10)
        self.create_subscription(Imu, '/niihan/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched)
        self.create_subscription(PointCloud2, self.get_parameter('slam_cloud_topic').value, self.pointcloud_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/niihan/sensors/panoramic/front/image_raw', self.image_callback, qos_profile_sensor_data)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # A paused /clock must not freeze the manual watchdog or localization stop.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.safety_loop, clock=self._steady_clock)
        self.create_timer(1.0, self.update_path_history, clock=self._steady_clock)
        self.check_nav2()

    def check_nav2(self):
        self.nav2_ready = self.nav_to_pose_client.server_is_ready()
        return self.nav2_ready

    def pointcloud_callback(self, msg):
        now = time.monotonic()
        if now - self.last_pc_time < 0.2:
            return
        self.last_pc_time = now
        import sensor_msgs_py.point_cloud2 as pc2
        stride = max(3, math.ceil(msg.width * msg.height / 10000))
        try:
            # Select the display budget before Python conversion. Walking every
            # voxel here starves TF and the safety timer as the 3D map grows.
            selected = pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                       uvs=np.arange(0, msg.width * msg.height, stride))
            xyz = np.column_stack([selected[name] for name in ('x', 'y', 'z')])
            points = xyz[np.isfinite(xyz).all(axis=1)].ravel().tolist()
        except (ValueError, AssertionError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'Ignoring invalid display cloud: {exc}')
            return
        with self._state_lock:
            self.pointcloud_data = points
            self.pc_generation += 1

    @synchronized
    def odom_callback(self, msg):
        self.telemetry['velocity'].update(linear_x=msg.twist.twist.linear.x,
                                          linear_y=msg.twist.twist.linear.y,
                                          angular_z=msg.twist.twist.angular.z)

    def _update_pose_from_tf(self):
        now = self.get_clock().now().nanoseconds / 1e9
        steady_now = time.monotonic()
        self.pose_valid = False
        localization = self.telemetry['localization']
        localization.update(health='UNAVAILABLE', confidence=None, age=None)
        clock_reset = self._last_ros_time is not None and now < self._last_ros_time - self.pose_future_tolerance
        self._last_ros_time = now
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            age = now - stamp
            localization['age'] = age
            if stamp != self._last_tf_stamp:
                self._last_tf_stamp = stamp
                self._last_tf_advance = steady_now
            frozen = self._last_tf_advance is None or steady_now - self._last_tf_advance > self.pose_freeze_timeout
            if clock_reset or now <= 0 or stamp <= 0 or age > self.pose_max_age or age < -self.pose_future_tolerance or frozen:
                localization['health'] = 'STALE'
                return False
            translation = transform.transform.translation
            q = transform.transform.rotation
            values = (translation.x, translation.y, translation.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(value) for value in values) or sum(v * v for v in values[3:]) < 1e-12:
                return False
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.telemetry['pose'].update(x=translation.x, y=translation.y, z=translation.z, yaw=yaw)
            localization.update(health='OK', confidence=None)
            self.pose_valid = True
            return True
        except Exception:
            return False

    @synchronized
    def gnss_callback(self, msg):
        valid = msg.status.status >= 0 and all(math.isfinite(value) for value in (msg.latitude, msg.longitude, msg.altitude))
        self.telemetry['gnss'].update(available=valid, latitude=msg.latitude if valid else None,
                                      longitude=msg.longitude if valid else None, altitude=msg.altitude if valid else None,
                                      covariance=list(msg.position_covariance), stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)

    @synchronized
    def gnss_quality_callback(self, msg):
        self.telemetry['gnss']['quality'] = msg.data

    @synchronized
    def slam_status_callback(self, msg):
        try:
            self.telemetry['slam'] = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def hardware_health_callback(self, msg):
        self._health_received = time.monotonic()
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict):
            self.telemetry['hardware_health'] = status

    @synchronized
    def imu_callback(self, msg):
        self.telemetry['imu'].update(ax=msg.linear_acceleration.x, ay=msg.linear_acceleration.y,
                                     az=msg.linear_acceleration.z, wx=msg.angular_velocity.x,
                                     wy=msg.angular_velocity.y, wz=msg.angular_velocity.z)

    @synchronized
    def map_callback(self, msg):
        if msg.header.frame_id != self.global_frame or msg.info.resolution <= 0:
            self.get_logger().warning('Ignoring occupancy grid with invalid frame or resolution.')
            return
        if len(msg.data) != msg.info.width * msg.info.height:
            self.get_logger().warning('Ignoring occupancy grid with inconsistent dimensions.')
            return
        q = msg.info.origin.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.map_generation += 1
        self.map_data = {'type': 'map', 'frame_id': msg.header.frame_id,
                         'revision': self.map_generation,
                         'width': msg.info.width, 'height': msg.info.height,
                         'resolution': msg.info.resolution,
                         'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y, 'yaw': yaw},
                         'data': list(msg.data)}
        geometry = (msg.info.width, msg.info.height, msg.info.resolution,
                    msg.info.origin.position.x, msg.info.origin.position.y, yaw)
        self._map_info = copy.deepcopy(msg.info)
        if geometry != self._map_geometry:
            self._map_geometry = geometry
            self._publish_keepout()

    def _publish_keepout(self):
        if self._map_info is None:
            return
        mask = OccupancyGrid()
        mask.header.frame_id = self.global_frame
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = copy.deepcopy(self._map_info)
        mask.data = self.geofence_manager.keepout_mask(*self._map_geometry, self.geofence_margin)
        self.keepout_pub.publish(mask)
        info = CostmapFilterInfo()
        info.header = copy.deepcopy(mask.header)
        info.type = 0
        info.filter_mask_topic = '/niihan/geofence_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.filter_info_pub.publish(info)

    def image_callback(self, msg):
        try:
            cv_image = cv2.rotate(self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.ROTATE_180)
            result, encoded = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if result:
                with self._state_lock:
                    self.latest_image = base64.b64encode(encoded).decode('utf-8')
                    self.image_generation += 1
        except Exception as exc:
            self.get_logger().error(f'Image conversion error: {exc}')

    @synchronized
    def update_path_history(self):
        self.check_nav2()
        if self.pose_valid:
            self.path_history.append({key: self.telemetry['pose'][key] for key in ('x', 'y')})
            self.path_history = self.path_history[-300:]

    def _publish_estop(self, active):
        message = Bool()
        message.data = active
        self.estop_pub.publish(message)

    def _stop_navigation(self, message, failed=False):
        operation = self._active_navigation
        if operation is not None:
            operation['cancelling'] = True
            operation['failure'] = failed
            if operation['kind'] == 'mission':
                self.mission_manager.cancel_mission()
                if failed:
                    self.mission_manager.state = 'FAILED'
                    self.mission_manager.waypoints[operation['index']]['status'] = 'FAILED'
                self.mission_manager.message = message
            handle = operation.get('handle')
            if handle is not None and not operation.get('cancel_sent'):
                operation['cancel_sent'] = True
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error(f'Could not send action cancellation: {exc}')
            self.navigation.update(state='FAILED' if failed else 'CANCELLED', message=message)
        self.cmd_vel_pub.publish(Twist())

    def _trigger_estop(self, message):
        self.safety_manager.trigger_estop()
        self._publish_estop(True)
        self._stop_navigation(message, failed=True)
        self.get_logger().error(message)

    @synchronized
    def safety_loop(self):
        valid = self._update_pose_from_tf()
        pose = self.telemetry['pose']
        if not self.safety_manager.estop_active:
            if not valid and (self._active_navigation is not None or
                              (self.geofence_manager.enabled and self.safety_manager.can_move())):
                self._trigger_estop('Localization unavailable or stale. Navigation stopped.')
            elif valid and not self.geofence_manager.is_robot_inside(pose['x'], pose['y'], self.geofence_margin):
                self._trigger_estop('GEOFENCE BREACH: robot footprint reached the geofence boundary.')
        if self.safety_manager.estop_active:
            self.cmd_vel_pub.publish(Twist())
        elif self.safety_manager.mode == 'MANUAL' and not self.safety_manager.can_move():
            self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def _response(action, success, message):
        return {'type': 'mission_write_response' if action == 'write_mission' else 'command_response',
                'action': action, 'success': success, 'message': message}

    @synchronized
    def process_command(self, cmd_data):
        if not isinstance(cmd_data, dict):
            return self._response(None, False, 'Command must be a JSON object.')
        action = cmd_data.get('action')
        try:
            return self._process_command(action, cmd_data)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return self._response(action, False, str(exc))

    def _process_command(self, action, data):
        if action in ('joystick', 'nav_goal', 'start_mission') and self.get_parameter('require_health').value:
            if time.monotonic() - self._health_received > 2.0 or not self.telemetry.get('hardware_health', {}).get('ready', False):
                return self._response(action, False, 'Motion inhibited: required sensor/drive health is unavailable.')
        if action == 'estop':
            self._trigger_estop('E-stop activated by operator.')
        elif action == 'clear_estop':
            if self._active_navigation is not None:
                return self._response(action, False, 'Waiting for Nav2 cancellation; e-stop remains active.')
            if not self._update_pose_from_tf():
                return self._response(action, False, 'Fresh map-frame localization is required to clear e-stop.')
            pose = self.telemetry['pose']
            if not self.geofence_manager.is_robot_inside(pose['x'], pose['y'], self.geofence_margin):
                return self._response(action, False, 'Robot footprint is outside the usable geofence area.')
            self.safety_manager.clear_estop()
            self._publish_estop(False)
        elif action == 'set_mode':
            mode = data.get('mode')
            if mode not in ('MANUAL', 'AUTO'):
                raise ValueError('Mode must be MANUAL or AUTO.')
            if mode != self.safety_manager.mode:
                self._stop_navigation('Mode changed; navigation cancelled.')
            self.safety_manager.set_mode(mode)
        elif action == 'joystick':
            linear = float(data.get('linear_x', 0.0))
            angular = float(data.get('angular_z', 0.0))
            if not math.isfinite(linear) or not math.isfinite(angular):
                raise ValueError('Velocity must be finite.')
            if self.geofence_manager.enabled and not self._update_pose_from_tf():
                return self._response(action, False, 'Fresh localization is required while geofencing is enabled.')
            pose = self.telemetry['pose']
            if not self.geofence_manager.is_robot_inside(pose['x'], pose['y'], self.geofence_margin):
                self._trigger_estop('GEOFENCE BREACH: manual command rejected.')
            if not self.safety_manager.validate_manual_command():
                return self._response(action, False, 'Manual command blocked by mode or e-stop.')
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            self.cmd_vel_pub.publish(msg)
            return None
        elif action == 'nav_goal':
            coords = self.mission_manager._coordinates(data.get('x'), data.get('y'), yaw=data.get('yaw', 0.0))
            success, message = self.send_nav_goal(coords['x'], coords['y'], coords['yaw'])
            return self._response(action, success, message)
        elif action in ('add_waypoint', 'update_waypoint', 'delete_waypoint', 'reorder_waypoint', 'clear_mission', 'write_mission'):
            if self._active_navigation is not None:
                raise ValueError('Cancel navigation and wait for cancellation before editing a mission.')
            if action == 'add_waypoint':
                self.mission_manager.add_waypoint(data.get('x'), data.get('y'), data.get('z', 0.0), data.get('yaw'))
            elif action == 'update_waypoint':
                self.mission_manager.update_waypoint(data.get('id'), data.get('x'), data.get('y'), data.get('z', 0.0), data.get('yaw'))
            elif action == 'delete_waypoint':
                self.mission_manager.delete_waypoint(data.get('id'))
            elif action == 'reorder_waypoint':
                self.mission_manager.reorder_waypoint(data.get('id'), data.get('direction'))
            elif action == 'clear_mission':
                self.mission_manager.clear_mission()
            else:
                if not self.check_nav2():
                    return self._response(action, False, 'Nav2 NavigateToPose action unavailable.')
                valid, message = self.geofence_manager.is_valid_mission(self.mission_manager.waypoints, margin=self.geofence_margin)
                if not valid:
                    return self._response(action, False, message)
                return self._response(action, *self.mission_manager.write_mission())
        elif action == 'start_mission':
            return self._response(action, *self.send_waypoints())
        elif action == 'cancel_mission':
            self._stop_navigation('Navigation cancelled by operator.')
            if self._active_navigation is None:
                self.mission_manager.cancel_mission()
        elif action in ('set_geofence', 'clear_geofence'):
            if self._active_navigation is not None:
                raise ValueError('Cancel navigation before changing the geofence.')
            if action == 'set_geofence':
                success, message = self.geofence_manager.set_geofence(data.get('polygon', []))
                if not success:
                    return self._response(action, False, message)
            else:
                self.geofence_manager.clear_geofence()
            self._publish_keepout()
            if self.mission_manager.state == 'READY':
                self.mission_manager._edited()
        else:
            return self._response(action, False, 'Unknown command.')
        return self._response(action, True, action.replace('_', ' ').capitalize() + ' applied.')

    def _can_start_navigation(self, targets):
        if not self.safety_manager.validate_auto_command():
            return False, 'Navigation requires AUTO mode and a cleared e-stop.'
        if self._active_navigation is not None:
            return False, 'Navigation is already active or cancellation is pending.'
        if not self.check_nav2():
            return False, 'Nav2 NavigateToPose action unavailable.'
        if not self._update_pose_from_tf():
            return False, 'Fresh map-frame localization is required to navigate.'
        if self.geofence_manager.enabled and self._map_info is None:
            return False, 'The 2D map is required to publish the geofence keepout mask.'
        return self.geofence_manager.is_valid_mission(targets, self.telemetry['pose'], self.geofence_margin)

    @synchronized
    def send_nav_goal(self, x, y, yaw):
        target = self.mission_manager._coordinates(x, y, yaw=yaw)
        success, message = self._can_start_navigation([target])
        if not success:
            return success, message
        operation = {'kind': 'nav', 'targets': [target], 'index': 0, 'cancelling': False}
        self._active_navigation = operation
        return self._dispatch_target(operation)

    @synchronized
    def send_waypoints(self):
        targets = copy.deepcopy(self.mission_manager.waypoints)
        success, message = self._can_start_navigation(targets)
        if not success:
            return success, message
        if not self.mission_manager.start_mission():
            return False, 'Write a nonempty mission before starting it.'
        operation = {'kind': 'mission', 'targets': targets, 'index': 0, 'cancelling': False}
        self._active_navigation = operation
        return self._dispatch_target(operation)

    def _dispatch_target(self, operation):
        target = dict(operation['targets'][operation['index']])
        if target['yaw'] is None:
            pose = self.telemetry['pose']
            dx, dy = target['x'] - pose['x'], target['y'] - pose['y']
            target['yaw'] = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else pose['yaw']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        # Targets are fixed coordinates in the global frame. A zero stamp avoids
        # pinning long missions to a historical TF sample that leaves the cache.
        goal.pose.pose.position.x = target['x']
        goal.pose.pose.position.y = target['y']
        goal.pose.pose.orientation.z = math.sin(target['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(target['yaw'] / 2.0)
        self._next_request_id += 1
        request_id = self._next_request_id
        operation.update(request_id=request_id, target=target, handle=None, cancel_sent=False)
        self.navigation.update(state='PENDING', message='Waiting for Nav2 goal acceptance.', target=target)
        try:
            future = self.nav_to_pose_client.send_goal_async(goal)
            future.add_done_callback(lambda completed: self._goal_response(completed, operation, request_id))
        except Exception as exc:
            self._finish_navigation(operation, 'FAILED', f'Could not send goal: {exc}')
            return False, str(exc)
        return True, 'Navigation goal sent.'

    @synchronized
    def _goal_response(self, future, operation, request_id):
        try:
            handle = future.result()
            if self._active_navigation is not operation or operation['request_id'] != request_id:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                state = 'FAILED' if not operation['cancelling'] or operation.get('failure') else 'CANCELLED'
                self._finish_navigation(operation, state, 'Nav2 rejected the goal.')
                return
            operation['handle'] = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda completed: self._goal_result(completed, operation, request_id))
            if operation['cancelling']:
                # Stop requests can arrive while send_goal_async is awaiting acceptance.
                if not operation['cancel_sent']:
                    operation['cancel_sent'] = True
                    handle.cancel_goal_async()
            elif self._active_navigation is operation:
                self.navigation.update(state='RUNNING', message='Navigating to target.')
        except Exception as exc:
            if self._active_navigation is operation and operation['request_id'] == request_id:
                self._trigger_estop(f'Navigation action response failed: {exc}')
                self._finish_navigation(operation, 'FAILED', f'Navigation action response failed: {exc}')

    @synchronized
    def _goal_result(self, future, operation, request_id):
        if self._active_navigation is not operation or operation['request_id'] != request_id:
            return
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._trigger_estop(f'Navigation result unavailable: {exc}')
            self._finish_navigation(operation, 'FAILED', f'Navigation result unavailable: {exc}')
            return
        if operation['cancelling']:
            state = 'FAILED' if operation.get('failure') else 'CANCELLED'
            self._finish_navigation(operation, state, self.navigation['message'])
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            state = 'CANCELLED' if status == GoalStatus.STATUS_CANCELED else 'FAILED'
            self._finish_navigation(operation, state, f'Nav2 goal ended with status {status}.')
            return
        target = operation['target']
        if not self._update_pose_from_tf():
            self._trigger_estop('Nav2 reported success without fresh localization; arrival cannot be confirmed.')
            self._finish_navigation(operation, 'FAILED', 'Arrival cannot be confirmed: localization is stale.')
            return
        pose = self.telemetry['pose']
        distance = math.hypot(pose['x'] - target['x'], pose['y'] - target['y'])
        yaw_error = abs(math.atan2(math.sin(pose['yaw'] - target['yaw']), math.cos(pose['yaw'] - target['yaw'])))
        if distance > self.goal_verify_xy_tolerance or yaw_error > self.goal_verify_yaw_tolerance:
            self._finish_navigation(operation, 'FAILED',
                                    f'Nav2 reported success away from target ({distance:.2f} m, {yaw_error:.2f} rad); mission stopped.')
            return
        if operation['kind'] == 'mission':
            index = operation['index']
            self.mission_manager.waypoints[index]['status'] = 'COMPLETED'
            if index + 1 < len(operation['targets']):
                operation['index'] += 1
                self.mission_manager.current_waypoint_index = operation['index']
                self.mission_manager.waypoints[operation['index']]['status'] = 'ACTIVE'
                self.mission_manager.message = f"Navigating to waypoint {operation['index'] + 1}."
                self._dispatch_target(operation)
                return
        self._finish_navigation(operation, 'COMPLETED', 'Target reached and map-frame pose verified.')

    def _finish_navigation(self, operation, state, message):
        if self._active_navigation is not operation:
            return
        if operation['kind'] == 'mission':
            self.mission_manager.state = state
            self.mission_manager.message = message
            wp = self.mission_manager.waypoints[operation['index']]
            if state != 'COMPLETED' and wp['status'] != 'COMPLETED':
                wp['status'] = state
        self.navigation.update(state=state, message=message)
        self._active_navigation = None
        self.get_logger().info(message)

    @synchronized
    def get_telemetry_json(self):
        return json.dumps({'type': 'telemetry', 'timestamp': self.get_clock().now().nanoseconds / 1e9,
                           'pose': self.telemetry['pose'], 'pose_valid': self.pose_valid,
                           'velocity': self.telemetry['velocity'], 'mode': self.safety_manager.mode,
                           'estop': self.safety_manager.estop_active, 'mission': self.mission_manager.get_status(),
                           'geofence': self.geofence_manager.get_status(), 'localization': self.telemetry['localization'],
                           'gnss': self.telemetry['gnss'], 'hardware_health': self.telemetry['hardware_health'], 'slam': self.telemetry.get('slam', {}),
                           'path_history': self.path_history, 'nav2_ready': self.nav2_ready, 'navigation': self.navigation})

    @synchronized
    def get_map_snapshot(self):
        return self.map_generation, json.dumps(self.map_data) if self.map_data else None

    @synchronized
    def get_pointcloud_json(self):
        return json.dumps({'type': 'pointcloud', 'data': self.pointcloud_data})
