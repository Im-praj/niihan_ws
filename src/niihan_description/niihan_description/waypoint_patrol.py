#!/usr/bin/env python3
"""
User-configurable waypoint patrol node for niihan robot.

Replaces the old patrol_controller with a flexible system that allows
users to mark patrol waypoints via ROS2 topics / RViz and control
patrol behaviour at runtime.

Features:
  - Named waypoints (A, B, C, D, …)
  - Patrol modes: loop, pingpong, once
  - Progress tracking with JSON status
  - Arrival announcements
  - Auto-resume after obstacle clears
  - Cliff detection integration
  - Dynamic waypoint insertion
  - Skip waypoint command
  - Preset loading from YAML
"""

import json
import math
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped, Pose, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


class WaypointPatrol(Node):
    """Waypoint patrol node with user-configurable waypoints and safety checks."""

    # ------------------------------------------------------------------ init
    def __init__(self):
        super().__init__('waypoint_patrol')

        # ----- Parameters ------------------------------------------------
        self.declare_parameter('patrol_speed', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('wait_duration', 3.0)
        self.declare_parameter('obstacle_stop_distance', 0.6)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('patrol_mode', 'loop')        # loop | pingpong | once
        self.declare_parameter('preset_file', '')             # YAML file path
        self.declare_parameter('auto_resume_interval', 5.0)   # seconds

        self._patrol_speed = self.get_parameter('patrol_speed').value
        self._waypoint_tol = self.get_parameter('waypoint_tolerance').value
        self._wait_dur = self.get_parameter('wait_duration').value
        self._obstacle_dist = self.get_parameter('obstacle_stop_distance').value
        self._auto_start = self.get_parameter('auto_start').value
        self._patrol_mode = self.get_parameter('patrol_mode').value
        self._auto_resume_sec = self.get_parameter('auto_resume_interval').value

        # ----- State ------------------------------------------------------
        self._waypoints: list[Pose] = []
        self._wp_index: int = 0
        self._direction: int = 1          # 1 = forward, -1 = reverse
        self._state: str = 'idle'         # idle | patrolling | paused | waiting | navigating
        self._paused_index: int | None = None
        self._paused_reason: str = ''     # 'obstacle' | 'no ground' | 'user' | ''
        self._goal_handle = None
        self._wait_timer = None
        self._auto_resume_timer = None
        self._lap_count: int = 0
        self._completed_count: int = 0

        # Sensor state
        self._map_msg: OccupancyGrid | None = None
        self._scan_msg: LaserScan | None = None
        self._cliff_scan_msg: LaserScan | None = None
        self._ground_safe: bool = True    # assume safe until proven otherwise

        # ----- TF ---------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ----- Nav2 action client -----------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ----- Publishers -------------------------------------------------
        self._wp_pub = self.create_publisher(PoseArray, '/patrol/waypoints', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/patrol/markers', 10)
        self._status_pub = self.create_publisher(String, '/patrol/status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/niihan/cmd_vel', 10)
        self._arrival_pub = self.create_publisher(String, '/patrol/arrival', 10)
        self._progress_pub = self.create_publisher(String, '/patrol/progress', 10)

        # ----- Subscribers ------------------------------------------------
        self.create_subscription(
            PoseStamped, '/patrol/add_waypoint', self._cb_add_waypoint, 10)
        self.create_subscription(
            PoseArray, '/patrol/set_waypoints', self._cb_set_waypoints, 10)
        self.create_subscription(
            PointStamped, '/clicked_point', self._cb_clicked_point, 10)
        self.create_subscription(
            String, '/patrol/command', self._cb_command, 10)

        # Sensor subscriptions
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self.create_subscription(
            PointCloud2, '/niihan/sensors/lidar/points', self._cb_pointcloud, 10)
        # Cliff detection integration
        self.create_subscription(
            LaserScan, '/cliff_scan', self._cb_cliff_scan, 10)

        # ----- Timers -----------------------------------------------------
        self.create_timer(0.1, self._safety_tick)
        self.create_timer(1.0, self._publish_viz)
        self.create_timer(0.5, self._patrol_tick)

        self._publish_status('idle')
        self.get_logger().info(
            'WaypointPatrol node started.  Publish waypoints then send "start".')

        # ----- Load preset if configured ----------------------------------
        preset_file = self.get_parameter('preset_file').value
        if preset_file:
            self._load_preset_file(preset_file)

        if self._auto_start and self._waypoints:
            self.get_logger().info('auto_start=True – starting patrol.')
            self._cmd_start()

    # ================================================================ PARAM
    def _reload_params(self):
        self._patrol_speed = self.get_parameter('patrol_speed').value
        self._waypoint_tol = self.get_parameter('waypoint_tolerance').value
        self._wait_dur = self.get_parameter('wait_duration').value
        self._obstacle_dist = self.get_parameter('obstacle_stop_distance').value
        self._patrol_mode = self.get_parameter('patrol_mode').value
        self._auto_resume_sec = self.get_parameter('auto_resume_interval').value

    # ======================================================= PRESET LOADING
    def _load_preset_file(self, path: str):
        """Load waypoints from a YAML preset file."""
        try:
            import yaml
        except ImportError:
            self.get_logger().error('PyYAML not available, cannot load preset.')
            return

        if not os.path.isfile(path):
            self.get_logger().warn(f'Preset file not found: {path}')
            return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            wp_data = None
            if 'waypoints' in data:
                wp_data = data['waypoints']
            elif 'presets' in data:
                first_key = next(iter(data['presets']))
                wp_data = data['presets'][first_key]['waypoints']

            if not wp_data:
                self.get_logger().warn(f'No waypoints found in {path}')
                return

            self._waypoints.clear()
            for label in sorted(wp_data.keys()):
                coords = wp_data[label]
                pose = Pose()
                pose.position.x = float(coords['x'])
                pose.position.y = float(coords['y'])
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                self._waypoints.append(pose)

            self._wp_index = 0
            labels = ', '.join(self._wp_label(i) for i in range(len(self._waypoints)))
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoints from {path}: [{labels}]')
            self._publish_waypoints()

        except Exception as e:
            self.get_logger().error(f'Failed to load preset file: {e}')

    # ================================================== WAYPOINT MANAGEMENT
    def _cb_add_waypoint(self, msg: PoseStamped):
        """Add a single waypoint from a PoseStamped message."""
        pose = msg.pose
        if not self._validate_waypoint(pose.position.x, pose.position.y):
            return
        self._waypoints.append(pose)
        label = self._wp_label(len(self._waypoints) - 1)
        self.get_logger().info(
            f'Waypoint {label} added at '
            f'({pose.position.x:.2f}, {pose.position.y:.2f}).  '
            f'Total: {len(self._waypoints)}')
        self._publish_waypoints()

    def _cb_set_waypoints(self, msg: PoseArray):
        """Replace all waypoints with the given PoseArray."""
        valid: list[Pose] = []
        for i, pose in enumerate(msg.poses):
            if self._validate_waypoint(pose.position.x, pose.position.y):
                valid.append(pose)
            else:
                self.get_logger().warn(
                    f'Waypoint {i} at ({pose.position.x:.2f}, '
                    f'{pose.position.y:.2f}) rejected.')
        if not valid:
            self.get_logger().warn('No valid waypoints in PoseArray; keeping old set.')
            return
        self._waypoints = valid
        self._wp_index = 0
        self._completed_count = 0
        self._lap_count = 0
        labels = ', '.join(self._wp_label(i) for i in range(len(valid)))
        self.get_logger().info(
            f'Waypoints set: [{labels}]  ({len(valid)} total)')
        self._publish_waypoints()

    def _cb_clicked_point(self, msg: PointStamped):
        """Add a waypoint from an RViz2 /clicked_point click."""
        x, y = msg.point.x, msg.point.y
        if not self._validate_waypoint(x, y):
            return
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self._waypoints.append(pose)
        label = self._wp_label(len(self._waypoints) - 1)
        self.get_logger().info(
            f'Waypoint {label} added via RViz clicked_point at '
            f'({x:.2f}, {y:.2f}).  Total: {len(self._waypoints)}')
        self._publish_waypoints()

    # ===================================================== WAYPOINT CHECKS
    def _validate_waypoint(self, x: float, y: float) -> bool:
        """Return True if (x, y) falls in free, known map space."""
        # Hard boundary check (within fenced area with margin)
        if abs(x) > 19.0 or abs(y) > 14.0:
            self.get_logger().warn(
                f'Waypoint ({x:.2f}, {y:.2f}) is outside the safe boundary – rejected.')
            return False

        if self._map_msg is None:
            self.get_logger().warn(
                f'No map received yet – accepting waypoint ({x:.2f}, {y:.2f}) '
                f'provisionally.')
            return True
        info = self._map_msg.info
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        # Check the cell and a small neighbourhood (robot footprint)
        radius = max(1, int(0.3 / info.resolution))
        has_unknown = False
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ix, iy = cx + dx, cy + dy
                if not (0 <= ix < info.width and 0 <= iy < info.height):
                    self.get_logger().warn(
                        f'Waypoint ({x:.2f}, {y:.2f}) is outside the map bounds – rejected.')
                    return False
                val = self._map_msg.data[iy * info.width + ix]
                if val == -1:
                    has_unknown = True
                if val > 65:
                    self.get_logger().warn(
                        f'Waypoint ({x:.2f}, {y:.2f}) falls in occupied space '
                        f'(cell value {val}) – rejected.')
                    return False
        if has_unknown:
            self.get_logger().warn(
                f'Waypoint ({x:.2f}, {y:.2f}) is in currently unknown map space; '
                'accepting it provisionally.')
        return True

    @staticmethod
    def _wp_label(index: int) -> str:
        """Return a letter label: 0→A, 1→B, …, 25→Z, 26→AA, …"""
        label = ''
        n = index
        while True:
            label = chr(ord('A') + n % 26) + label
            n = n // 26 - 1
            if n < 0:
                break
        return label

    # ====================================================== PATROL COMMANDS
    def _cb_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received patrol command: "{cmd}"')

        if cmd == 'start':
            self._cmd_start()
        elif cmd == 'stop':
            self._cmd_stop()
        elif cmd == 'pause':
            self._cmd_pause()
        elif cmd == 'resume':
            self._cmd_resume()
        elif cmd == 'clear':
            self._cmd_clear()
        elif cmd == 'reverse':
            self._cmd_reverse()
        elif cmd == 'skip':
            self._cmd_skip()
        elif cmd.startswith('mode:'):
            self._cmd_set_mode(cmd.split(':', 1)[1].strip())
        else:
            self.get_logger().warn(
                f'Unknown patrol command "{cmd}". '
                f'Valid: start, stop, pause, resume, clear, reverse, skip, mode:<loop|pingpong|once>')

    def _cmd_start(self):
        if not self._waypoints:
            self.get_logger().warn('Cannot start patrol – no waypoints defined.')
            return
        self._reload_params()
        self._state = 'patrolling'
        self._wp_index = 0
        self._direction = 1 if self._direction == 0 else self._direction
        self._completed_count = 0
        self._lap_count = 0
        self._publish_status('patrolling')
        self.get_logger().info(
            f'Patrol started with {len(self._waypoints)} waypoints, '
            f'mode={self._patrol_mode}.')

    def _cmd_stop(self):
        self._cancel_current_goal()
        self._cancel_wait_timer()
        self._cancel_auto_resume_timer()
        self._state = 'idle'
        self._paused_index = None
        self._paused_reason = ''
        self._publish_status('stopped')
        self._publish_stop_vel()
        self.get_logger().info('Patrol stopped.')

    def _cmd_pause(self):
        if self._state not in ('patrolling', 'navigating', 'waiting'):
            self.get_logger().warn(f'Cannot pause from state "{self._state}".')
            return
        self._cancel_current_goal()
        self._cancel_wait_timer()
        self._paused_index = self._wp_index
        self._paused_reason = 'user'
        self._state = 'paused'
        self._publish_status('paused')
        self._publish_stop_vel()
        self.get_logger().info(
            f'Patrol paused at waypoint index {self._wp_index} '
            f'({self._wp_label(self._wp_index)}).')

    def _cmd_resume(self):
        if self._state != 'paused':
            self.get_logger().warn('Cannot resume – patrol is not paused.')
            return
        if self._paused_index is not None:
            self._wp_index = self._paused_index
        self._cancel_auto_resume_timer()
        self._paused_reason = ''
        self._state = 'patrolling'
        self._publish_status('resumed → patrolling')
        self.get_logger().info(
            f'Patrol resumed from waypoint {self._wp_label(self._wp_index)}.')

    def _cmd_clear(self):
        self._cancel_current_goal()
        self._cancel_wait_timer()
        self._cancel_auto_resume_timer()
        self._waypoints.clear()
        self._wp_index = 0
        self._state = 'idle'
        self._completed_count = 0
        self._lap_count = 0
        self._publish_status('cleared')
        self._publish_waypoints()
        self._publish_stop_vel()
        self.get_logger().info('All waypoints cleared.')

    def _cmd_reverse(self):
        self._direction *= -1
        direction_str = 'forward' if self._direction == 1 else 'reverse'
        self.get_logger().info(f'Patrol direction set to {direction_str}.')
        self._publish_status(f'direction: {direction_str}')

    def _cmd_skip(self):
        """Skip current waypoint and move to next."""
        if self._state not in ('navigating', 'waiting', 'patrolling', 'paused'):
            self.get_logger().warn(f'Cannot skip from state "{self._state}".')
            return
        self._cancel_current_goal()
        self._cancel_wait_timer()
        self._cancel_auto_resume_timer()
        old_label = self._wp_label(self._wp_index)
        self._advance_waypoint()
        new_label = self._wp_label(self._wp_index)
        self._paused_reason = ''
        self._state = 'patrolling'
        self.get_logger().info(f'Skipped {old_label}, moving to {new_label}.')
        self._publish_status(f'skipped → {new_label}')

    def _cmd_set_mode(self, mode: str):
        if mode in ('loop', 'pingpong', 'once'):
            self._patrol_mode = mode
            self.get_logger().info(f'Patrol mode set to: {mode}')
            self._publish_status(f'mode: {mode}')
        else:
            self.get_logger().warn(
                f'Invalid mode "{mode}". Valid: loop, pingpong, once')

    # ========================================================= PATROL LOOP
    def _patrol_tick(self):
        """Main patrol state machine – called at 2 Hz."""
        if self._state == 'idle' or self._state == 'paused':
            return
        if self._state == 'waiting':
            return  # wait_timer callback will advance
        if self._state == 'navigating':
            return  # waiting for Nav2 result callback

        # state == 'patrolling' → send next goal
        if not self._waypoints:
            self._state = 'idle'
            self._publish_status('idle (no waypoints)')
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready – waiting…',
                                   throttle_duration_sec=5.0)
            return

        self._send_next_goal()

    def _send_next_goal(self):
        """Send the current waypoint as a NavigateToPose goal."""
        wp = self._waypoints[self._wp_index]
        label = self._wp_label(self._wp_index)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = wp

        self._state = 'navigating'
        self._publish_status(f'navigating → {label}')
        self.get_logger().info(
            f'Navigating to waypoint {label} '
            f'({wp.position.x:.2f}, {wp.position.y:.2f})…')

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            label = self._wp_label(self._wp_index)
            self.get_logger().warn(f'Nav2 rejected goal for waypoint {label}.')
            self._advance_waypoint()
            self._state = 'patrolling'
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        status = future.result().status
        label = self._wp_label(self._wp_index)
        self._goal_handle = None

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached waypoint {label}.  Waiting {self._wait_dur}s…')
            self._completed_count += 1

            # Publish arrival announcement
            arrival = String()
            arrival.data = label
            self._arrival_pub.publish(arrival)

            self._state = 'waiting'
            self._publish_status(f'waiting at {label}')
            self._wait_timer = self.create_timer(
                self._wait_dur, self._wait_done, callback_group=None)
        elif status == 6:  # CANCELED
            self.get_logger().warn(f'Goal for waypoint {label} was canceled.')
            if self._state == 'navigating':
                self._state = 'patrolling'
        else:
            self.get_logger().warn(
                f'Goal for waypoint {label} ended with status {status}.')
            self._advance_waypoint()
            self._state = 'patrolling'

    def _wait_done(self):
        """Timer callback after dwelling at a waypoint."""
        self._cancel_wait_timer()
        self._advance_waypoint()

        # Check if patrol is complete (once mode)
        if self._patrol_mode == 'once' and self._completed_count >= len(self._waypoints):
            self._state = 'idle'
            self._publish_status('completed (once mode)')
            self.get_logger().info('Patrol complete (once mode).')
            return

        self._state = 'patrolling'
        self.get_logger().info(
            f'Moving to next waypoint {self._wp_label(self._wp_index)}.')

    def _advance_waypoint(self):
        """Move index to the next waypoint (cycling)."""
        if not self._waypoints:
            return

        new_index = self._wp_index + self._direction

        if self._patrol_mode == 'pingpong':
            # Reverse at boundaries
            if new_index >= len(self._waypoints):
                self._direction = -1
                new_index = self._wp_index + self._direction
                self._lap_count += 1
            elif new_index < 0:
                self._direction = 1
                new_index = self._wp_index + self._direction
                self._lap_count += 1
        elif self._patrol_mode == 'loop':
            if new_index >= len(self._waypoints):
                self._lap_count += 1
            new_index = new_index % len(self._waypoints)
        elif self._patrol_mode == 'once':
            if new_index >= len(self._waypoints) or new_index < 0:
                new_index = self._wp_index  # stay at last
                return  # don't advance further

        self._wp_index = max(0, min(len(self._waypoints) - 1, new_index))

    # ============================================================== SAFETY
    def _cb_map(self, msg: OccupancyGrid):
        self._map_msg = msg

    def _cb_scan(self, msg: LaserScan):
        self._scan_msg = msg

    def _cb_cliff_scan(self, msg: LaserScan):
        """Receive cliff detection scan for enhanced safety."""
        self._cliff_scan_msg = msg

    def _cb_pointcloud(self, msg: PointCloud2):
        """Check 3D point cloud for ground presence in front of the robot.

        If there are no returns below the robot's approximate height
        (z < 0.1 relative to base) in the forward region, we consider
        the ground absent (potential cliff / drop-off).
        """
        # We do a lightweight scan of the point cloud.
        # PointCloud2 field layout varies; we look for 'x','y','z'.
        field_names = [f.name for f in msg.fields]
        if 'x' not in field_names or 'z' not in field_names:
            return

        x_offset = next(f.offset for f in msg.fields if f.name == 'x')
        y_offset = next(f.offset for f in msg.fields if f.name == 'y')
        z_offset = next(f.offset for f in msg.fields if f.name == 'z')
        point_step = msg.point_step
        data = msg.data

        ground_count = 0
        front_count = 0
        # Sample every 8th point to keep CPU usage low
        sample_step = max(1, len(data) // (point_step * 500)) * point_step

        offset = 0
        while offset + point_step <= len(data):
            try:
                px = struct.unpack_from('f', data, offset + x_offset)[0]
                py = struct.unpack_from('f', data, offset + y_offset)[0]
                pz = struct.unpack_from('f', data, offset + z_offset)[0]
            except struct.error:
                break

            # Forward region: x in (0.2 … 1.5), |y| < 0.5
            if 0.2 < px < 1.5 and abs(py) < 0.5:
                front_count += 1
                if pz < -0.05:  # point below sensor → ground exists
                    ground_count += 1

            offset += sample_step

        if front_count > 10:
            self._ground_safe = (ground_count / front_count) > 0.15
        else:
            # Not enough data to decide – assume safe
            self._ground_safe = True

    def _safety_tick(self):
        """10 Hz safety monitor – obstacle proximity, cliff scan, and ground check."""
        if self._state not in ('navigating',):
            return

        # --- LiDAR front obstacle check ---
        if self._scan_msg is not None:
            front_min = self._front_distance()
            if front_min < self._obstacle_dist:
                self.get_logger().warn(
                    f'SAFETY: obstacle at {front_min:.2f} m – canceling goal.')
                self._cancel_current_goal()
                self._publish_stop_vel()
                self._state = 'paused'
                self._paused_index = self._wp_index
                self._paused_reason = 'obstacle'
                self._publish_status('paused (obstacle)')
                self._start_auto_resume()
                return

        # --- Cliff scan check (from cliff_detector node) ---
        if self._cliff_scan_msg is not None:
            cliff_front_min = self._cliff_front_distance()
            if cliff_front_min < 1.5:
                self.get_logger().error(
                    f'SAFETY: cliff detected at {cliff_front_min:.2f} m ahead – '
                    f'canceling goal!')
                self._cancel_current_goal()
                self._publish_stop_vel()
                self._state = 'paused'
                self._paused_index = self._wp_index
                self._paused_reason = 'cliff'
                self._publish_status('paused (cliff detected)')
                return

        # --- 3D ground check ---
        if not self._ground_safe:
            self.get_logger().error(
                'SAFETY: no ground detected ahead (cliff?) – canceling goal!')
            self._cancel_current_goal()
            self._publish_stop_vel()
            self._state = 'paused'
            self._paused_index = self._wp_index
            self._paused_reason = 'no ground'
            self._publish_status('paused (no ground)')
            return

    def _front_distance(self) -> float:
        """Minimum LiDAR distance within ±25° of straight ahead."""
        if self._scan_msg is None:
            return math.inf
        values = []
        for i, d in enumerate(self._scan_msg.ranges):
            angle = self._scan_msg.angle_min + i * self._scan_msg.angle_increment
            if abs(angle) <= math.radians(25.0) and math.isfinite(d):
                if self._scan_msg.range_min <= d <= self._scan_msg.range_max:
                    values.append(d)
        return min(values, default=math.inf)

    def _cliff_front_distance(self) -> float:
        """Minimum cliff scan distance within ±30° of straight ahead."""
        if self._cliff_scan_msg is None:
            return math.inf
        values = []
        for i, d in enumerate(self._cliff_scan_msg.ranges):
            angle = (self._cliff_scan_msg.angle_min +
                     i * self._cliff_scan_msg.angle_increment)
            if abs(angle) <= math.radians(30.0) and math.isfinite(d):
                if (self._cliff_scan_msg.range_min <= d <=
                        self._cliff_scan_msg.range_max):
                    values.append(d)
        return min(values, default=math.inf)

    # ===================================================== AUTO-RESUME
    def _start_auto_resume(self):
        """Start a periodic timer to check if obstacle has cleared."""
        if self._auto_resume_timer is not None:
            return  # already running
        self._auto_resume_timer = self.create_timer(
            self._auto_resume_sec, self._check_auto_resume)
        self.get_logger().info(
            f'Auto-resume check every {self._auto_resume_sec}s.')

    def _check_auto_resume(self):
        """Check if the obstacle has cleared and auto-resume patrol."""
        if self._state != 'paused' or self._paused_reason not in ('obstacle',):
            self._cancel_auto_resume_timer()
            return

        # Check if front is now clear
        front_min = self._front_distance()
        if front_min > self._obstacle_dist * 1.5:
            self.get_logger().info(
                f'Obstacle cleared (front distance={front_min:.2f}m). '
                f'Auto-resuming patrol.')
            self._cancel_auto_resume_timer()
            self._paused_reason = ''
            self._cmd_resume()
        else:
            self.get_logger().info(
                f'Obstacle still present (front={front_min:.2f}m). '
                f'Will retry in {self._auto_resume_sec}s.',
                throttle_duration_sec=10.0)

    def _cancel_auto_resume_timer(self):
        if self._auto_resume_timer is not None:
            self._auto_resume_timer.cancel()
            self.destroy_timer(self._auto_resume_timer)
            self._auto_resume_timer = None

    # ======================================================= NAV HELPERS
    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current Nav2 goal.')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self.destroy_timer(self._wait_timer)
            self._wait_timer = None

    def _publish_stop_vel(self):
        self._cmd_vel_pub.publish(Twist())

    # ======================================================= VISUALISATION
    def _publish_waypoints(self):
        """Publish current waypoints as a PoseArray."""
        pa = PoseArray()
        pa.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        pa.poses = list(self._waypoints)
        self._wp_pub.publish(pa)

    def _publish_viz(self):
        """Publish waypoint markers, status, and progress at 1 Hz."""
        self._publish_waypoints()
        self._publish_markers()
        self._publish_status(self._state)
        self._publish_progress()

    def _publish_markers(self):
        """Publish labeled sphere + text markers for each waypoint."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self._waypoints):
            label = self._wp_label(i)
            is_active = (self._state in ('navigating', 'waiting') and
                         i == self._wp_index)

            # Sphere marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'patrol_waypoints'
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = wp
            m.scale.x = m.scale.y = m.scale.z = 0.35
            if is_active:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.9
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = stamp
            t.ns = 'patrol_labels'
            t.id = i * 2 + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp.position.x
            t.pose.position.y = wp.position.y
            t.pose.position.z = wp.position.z + 0.5
            t.pose.orientation.w = 1.0
            t.scale.z = 0.4
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = label
            ma.markers.append(t)

        # Delete old markers beyond current count
        for j in range(len(self._waypoints), len(self._waypoints) + 20):
            for ns in ('patrol_waypoints', 'patrol_labels'):
                d = Marker()
                d.header.frame_id = 'map'
                d.header.stamp = stamp
                d.ns = ns
                d.id = j * 2 if ns == 'patrol_waypoints' else j * 2 + 1
                d.action = Marker.DELETE
                ma.markers.append(d)

        self._marker_pub.publish(ma)

    def _publish_status(self, status_text: str):
        msg = String()
        msg.data = status_text
        self._status_pub.publish(msg)

    def _publish_progress(self):
        """Publish JSON progress report."""
        if not self._waypoints:
            return

        current_label = self._wp_label(self._wp_index) if self._waypoints else '?'
        next_idx = (self._wp_index + self._direction) % len(self._waypoints) if self._waypoints else 0
        next_label = self._wp_label(next_idx) if self._waypoints else '?'

        progress = {
            'current': current_label,
            'next': next_label,
            'completed': self._completed_count,
            'total': len(self._waypoints),
            'mode': self._patrol_mode,
            'lap': self._lap_count,
            'state': self._state,
            'direction': 'forward' if self._direction == 1 else 'reverse',
        }

        msg = String()
        msg.data = json.dumps(progress)
        self._progress_pub.publish(msg)


# ================================================================== ENTRY
def waypoint_patrol_main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down WaypointPatrol.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    waypoint_patrol_main()
