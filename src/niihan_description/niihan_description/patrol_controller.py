#!/usr/bin/env python3

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from slam_toolbox.srv import DeserializePoseGraph, SaveMap, SerializePoseGraph


WAYPOINTS = [
    (0.0, 3.0, 1.57), (2.0, 6.0, 0.0), (8.0, 6.0, 0.0),
    (14.0, 6.0, 0.0), (16.0, 3.0, -1.57), (16.0, 0.0, -1.57),
    (16.0, -1.0, -1.57), (14.0, -6.0, 3.14), (8.0, -6.0, 3.14),
    (2.0, -6.0, 3.14), (0.0, -3.0, 1.57),
]

MAP_BOUNDARY = (-10.0, 26.0, -15.0, 15.0)
SAFE_BOUNDARY = (-9.0, 25.0, -14.0, 14.0)
ODOM_SAFE_BOUNDARY = (-11.0, 27.0, -14.0, 14.0)


class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/niihan/cmd_vel', 10)
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.deserialize_client = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.scan_msg = None
        self.map_version = 0
        self.goal_map_version = -1
        self.goal_handle = None
        self.goal_kind = None
        self.waypoint_idx = 0
        self.visited_exploration_goals = []
        self.blocked = False
        self.bootstrap_start = None
        self.bootstrap_map_version = -1
        self.bootstrap_waiting = False
        self.recovery_started = None
        self.recovery_phase = None
        self.recovery_inward = False
        self.recovery_cooldown_until = self.get_clock().now()
        self.mapping_complete = False
        self.last_mapping_log_version = -1
        self.last_decision_time = self.get_clock().now()
        self.persistence_dir = os.path.expanduser('~/.ros/niihan_maps')
        os.makedirs(self.persistence_dir, exist_ok=True)
        self.persistence_loaded = False
        self.last_persistence_time = self.get_clock().now()
        self.decision_timer = self.create_timer(0.5, self.decision_callback)
        self.create_timer(0.1, self.safety_callback)
        self.create_timer(15.0, self.persistence_callback)
        self.get_logger().info('Waiting for Nav2, map, and LiDAR data...')

    def map_callback(self, message):
        self.map_msg = message
        self.map_version += 1

    def scan_callback(self, message):
        self.scan_msg = message

    def front_distance(self):
        if self.scan_msg is None:
            return math.inf
        values = []
        for index, distance in enumerate(self.scan_msg.ranges):
            angle = self.scan_msg.angle_min + index * self.scan_msg.angle_increment
            if abs(angle) <= math.radians(25.0) and math.isfinite(distance):
                if self.scan_msg.range_min <= distance <= self.scan_msg.range_max:
                    values.append(distance)
        return min(values, default=math.inf)

    def safety_callback(self):
        if self.recovery_started is not None:
            return
        odom_position = self.odom_position()
        if odom_position is not None and not self.inside_odom_boundary(*odom_position):
            self.publish_stop()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.goal_kind = None
            if self.recovery_started is None:
                self.start_recovery(inward=True)
            return
        distance = self.front_distance()
        if distance < 0.8:
            if self.goal_handle is not None:
                self.publish_stop()
                self.get_logger().warn(
                    f'Obstacle at {distance:.2f} m: canceling goal and rescanning.'
                )
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.goal_kind = None
            self.blocked = True
            if self.recovery_started is None:
                self.get_logger().warn('Robot is too close to an obstacle; starting recovery.')
        elif self.blocked and distance > 1.1:
            self.get_logger().info('Front area clear; waiting for a fresh map update.')
            self.blocked = False

    def decision_callback(self):
        if self.goal_handle is not None:
            return
        if not self.nav_client.server_is_ready() or self.map_msg is None or self.scan_msg is None:
            return
        if self.recovery_started is not None:
            self.run_recovery()
            return
        if self.get_clock().now() < self.recovery_cooldown_until:
            self.publish_stop()
            return
        if self.blocked:
            self.start_recovery()
            self.run_recovery()
            return
        if self.map_version == self.goal_map_version:
            return
        if (self.get_clock().now() - self.last_decision_time).nanoseconds < 1_000_000_000:
            return
        if not self.mapping_complete:
            self.mapping_complete = self.map_boundary_complete()
            if self.mapping_complete:
                self.get_logger().info('Baseworld boundary is mapped; switching to patrol goals.')

        if self.mapping_complete:
            waypoint = self.next_free_waypoint()
            if waypoint is not None:
                self.send_goal(*waypoint, kind='waypoint')
                self.last_decision_time = self.get_clock().now()
                return
        else:
            goal = self.find_exploration_goal()
            if goal is not None:
                self.send_goal(*goal, kind='exploration')
            else:
                self.bootstrap_exploration()
        if self.mapping_complete and self.goal_handle is None:
            self.publish_stop()
            self.get_logger().info('No free patrol waypoint is mapped yet; holding position.')
        self.last_decision_time = self.get_clock().now()

    def map_boundary_complete(self):
        if self.map_msg is None:
            return False
        min_x, max_x, min_y, max_y = MAP_BOUNDARY
        samples = 80
        for index in range(samples + 1):
            fraction = index / samples
            perimeter_points = (
                (min_x + fraction * (max_x - min_x), min_y),
                (min_x + fraction * (max_x - min_x), max_y),
                (min_x, min_y + fraction * (max_y - min_y)),
                (max_x, min_y + fraction * (max_y - min_y)),
            )
            for x, y in perimeter_points:
                if not self.is_known_cell(x, y):
                    if self.last_mapping_log_version != self.map_version:
                        self.get_logger().info(
                            'Mapping in progress: baseworld boundary is not fully observed.'
                        )
                        self.last_mapping_log_version = self.map_version
                    return False
        return True

    def is_known_cell(self, x, y):
        info = self.map_msg.info
        cell_x = int((x - info.origin.position.x) / info.resolution)
        cell_y = int((y - info.origin.position.y) / info.resolution)
        radius = max(1, int(0.25 / info.resolution))
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                current_x = cell_x + offset_x
                current_y = cell_y + offset_y
                if not (0 <= current_x < info.width and 0 <= current_y < info.height):
                    return False
                if self.map_msg.data[current_y * info.width + current_x] < 0:
                    return False
        return True

    def bootstrap_exploration(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            self.publish_stop()
            return

        position = transform.transform.translation
        if not self.inside_safe_boundary(position.x, position.y) or self.near_safe_boundary(position.x, position.y):
            self.start_recovery(inward=True)
            self.run_recovery()
            return
        if self.bootstrap_waiting:
            self.publish_stop()
            if self.map_version > self.bootstrap_map_version:
                self.bootstrap_waiting = False
                self.bootstrap_start = (position.x, position.y)
                self.bootstrap_map_version = self.map_version
            return

        if self.bootstrap_start is None:
            self.bootstrap_start = (position.x, position.y)
            self.bootstrap_map_version = self.map_version

        traveled = math.hypot(
            position.x - self.bootstrap_start[0],
            position.y - self.bootstrap_start[1],
        )
        if traveled >= 1.2:
            self.publish_stop()
            self.get_logger().info('Bootstrap segment complete; waiting for map update.')
            self.bootstrap_start = None
            self.bootstrap_map_version = self.map_version
            self.bootstrap_waiting = True
            return

        best_angle, best_distance = self.best_scan_direction()
        if best_distance < 0.8 or self.front_distance() < 0.8:
            self.start_recovery()
            return

        command = Twist()
        if abs(best_angle) > math.radians(18.0):
            command.angular.z = 0.35 if best_angle > 0.0 else -0.35
        else:
            command.linear.x = 0.8
        self.cmd_pub.publish(command)
        self.get_logger().info(
            f'Bootstrap exploration: clearance {best_distance:.2f} m, '
            f'heading {math.degrees(best_angle):.0f} deg.'
        )

    def start_recovery(self, inward=False):
        self.recovery_started = self.get_clock().now()
        self.recovery_phase = 'reverse_inward' if inward else 'reverse'
        self.recovery_inward = inward
        self.blocked = True
        self.get_logger().warn('Recovery: reversing before turning toward the safe area.')

    def run_recovery(self):
        elapsed = (self.get_clock().now() - self.recovery_started).nanoseconds / 1e9
        command = Twist()
        if self.recovery_phase in ('reverse', 'reverse_inward') and elapsed < 1.0:
            command.linear.x = -0.24
        else:
            if self.recovery_phase in ('reverse', 'reverse_inward'):
                self.recovery_phase = 'turn'
                self.recovery_started = self.get_clock().now()
                elapsed = 0.0
                self.get_logger().warn('Recovery: rotating toward the clearest LiDAR direction.')
            angle, distance = self.best_scan_direction(prefer_front=False)
            if self.recovery_inward:
                angle = self.inward_scan_angle()
            if distance < 0.9:
                command.angular.z = 0.9 if angle >= 0.0 else -0.9
            elif self.front_distance() < 1.1 and elapsed < 5.0:
                command.angular.z = 0.9 if angle >= 0.0 else -0.9
            else:
                self.recovery_started = None
                self.recovery_phase = None
                self.recovery_inward = False
                self.blocked = self.front_distance() < 1.1
                self.recovery_cooldown_until = self.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
                self.bootstrap_start = None
                if self.blocked:
                    self.get_logger().warn('Recovery turn ended before full clearance; pausing before retry.')
                else:
                    self.get_logger().info('Recovery complete; resuming LiDAR mapping.')
        self.cmd_pub.publish(command)

    def inside_safe_boundary(self, x, y):
        min_x, max_x, min_y, max_y = SAFE_BOUNDARY
        return min_x <= x <= max_x and min_y <= y <= max_y

    def near_safe_boundary(self, x, y):
        min_x, max_x, min_y, max_y = SAFE_BOUNDARY
        margin = 0.8
        return (
            x < min_x + margin or x > max_x - margin or
            y < min_y + margin or y > max_y - margin
        )

    def inward_scan_angle(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            return 0.0
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        rotation = transform.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        odom = self.odom_position()
        if odom is not None:
            x, y = odom
            center_x = (ODOM_SAFE_BOUNDARY[0] + ODOM_SAFE_BOUNDARY[1]) / 2.0
            center_y = (ODOM_SAFE_BOUNDARY[2] + ODOM_SAFE_BOUNDARY[3]) / 2.0
        else:
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            center_x = (SAFE_BOUNDARY[0] + SAFE_BOUNDARY[1]) / 2.0
            center_y = (SAFE_BOUNDARY[2] + SAFE_BOUNDARY[3]) / 2.0
        target_angle = math.atan2(center_y - y, center_x - x)
        return math.atan2(
            math.sin(target_angle - robot_yaw),
            math.cos(target_angle - robot_yaw),
        )

    def odom_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return None

    def inside_odom_boundary(self, x, y):
        min_x, max_x, min_y, max_y = ODOM_SAFE_BOUNDARY
        return min_x <= x <= max_x and min_y <= y <= max_y

    def persistence_callback(self):
        if getattr(self, 'persistence_pending', 0) > 0:
            return
        posegraph_path = os.path.join(self.persistence_dir, 'baseworld')
        map_path = os.path.join(self.persistence_dir, 'baseworld')
        if not self.persistence_loaded and os.path.exists(posegraph_path + '.posegraph'):
            if self.deserialize_client.service_is_ready():
                request = DeserializePoseGraph.Request()
                request.filename = posegraph_path
                request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
                self.deserialize_client.call_async(request)
                self.persistence_loaded = True
                self.get_logger().info(f'Resuming mapping from {posegraph_path}.posegraph')
            return
        if not self.serialize_client.service_is_ready():
            self.get_logger().warn('SLAM serialize_map service is not ready; map not saved yet.')
            return
        serialize_request = SerializePoseGraph.Request()
        serialize_request.filename = posegraph_path
        self.persistence_pending = 1
        serialize_future = self.serialize_client.call_async(serialize_request)
        serialize_future.add_done_callback(self.persistence_result_callback)
        if self.map_msg is None or not self.save_map_client.service_is_ready():
            self.get_logger().warn('SLAM save_map service or /map is not ready; pose graph only.')
            return
        save_request = SaveMap.Request()
        save_request.name.data = os.path.abspath(map_path)
        self.persistence_pending += 1
        save_future = self.save_map_client.call_async(save_request)
        save_future.add_done_callback(self.persistence_result_callback)
        self.get_logger().info(f'Saving map and pose graph under {self.persistence_dir}.')

    def persistence_result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'SLAM persistence result: {result}')
        except Exception as error:
            self.get_logger().error(f'SLAM persistence failed: {error}')
        finally:
            self.persistence_pending = max(0, self.persistence_pending - 1)

    def best_scan_direction(self, prefer_front=True):
        front_distance = self.front_distance()
        if prefer_front and front_distance > 1.2:
            return 0.0, front_distance

        best_angle = 0.0
        best_score = -1.0
        best_distance = 0.0
        for center_degrees in range(-75, 76, 15):
            center = math.radians(center_degrees)
            samples = []
            for offset_degrees in range(-12, 13, 4):
                distance = self.scan_distance(center + math.radians(offset_degrees))
                samples.append(min(distance, 8.0))
            clearance = min(samples)
            average = sum(samples) / len(samples)
            score = clearance * 0.7 + average * 0.3
            if score > best_score:
                best_score = score
                best_angle = center
                best_distance = clearance
        return best_angle, best_distance

    def next_free_waypoint(self):
        for index in range(self.waypoint_idx, len(WAYPOINTS)):
            if self.is_free_goal(*WAYPOINTS[index][:2]):
                return WAYPOINTS[index]
        return None

    def is_free_goal(self, x, y):
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        center_x = int((x - info.origin.position.x) / info.resolution)
        center_y = int((y - info.origin.position.y) / info.resolution)
        radius = max(1, int(0.45 / info.resolution))
        for cell_y in range(center_y - radius, center_y + radius + 1):
            for cell_x in range(center_x - radius, center_x + radius + 1):
                if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                    return False
                value = self.map_msg.data[cell_y * info.width + cell_x]
                if value < 0 or value > 25:
                    return False
        return True

    def find_exploration_goal(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            return None
        rotation = transform.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        frontier_goal = self.find_frontier_goal(
            transform.transform.translation.x,
            transform.transform.translation.y,
            robot_yaw,
        )
        if frontier_goal is not None:
            return frontier_goal

        candidates = []
        for angle in range(-90, 91, 15):
            scan_angle = math.radians(angle)
            direction = robot_yaw + scan_angle
            distance = self.scan_distance(scan_angle)
            if distance < 0.8:
                continue
            origin_x = transform.transform.translation.x
            origin_y = transform.transform.translation.y
            for step in (0.6, 0.8, 1.0, 1.2, 1.5, 2.0):
                if step >= distance - 0.35:
                    break
                x = origin_x + math.cos(direction) * step
                y = origin_y + math.sin(direction) * step
                if any(math.hypot(x - old_x, y - old_y) < 0.7
                       for old_x, old_y in self.visited_exploration_goals):
                    continue
                if not self.inside_safe_boundary(x, y):
                    continue
                if self.is_free_goal(x, y) and self.is_free_corridor(origin_x, origin_y, x, y):
                    candidates.append((distance, x, y, direction))
        if not candidates:
            return None
        distance, x, y, yaw = max(candidates)
        self.visited_exploration_goals.append((x, y))
        self.get_logger().info(
            f'Exploring LiDAR-clear area at ({x:.2f}, {y:.2f}); clearance {distance:.2f} m.'
        )
        return x, y, yaw

    def find_frontier_goal(self, robot_x, robot_y, robot_yaw):
        info = self.map_msg.info
        robot_cell_x = int((robot_x - info.origin.position.x) / info.resolution)
        robot_cell_y = int((robot_y - info.origin.position.y) / info.resolution)
        search_radius = max(1, int(3.0 / info.resolution))
        candidates = []

        for cell_y in range(robot_cell_y - search_radius, robot_cell_y + search_radius + 1, 2):
            for cell_x in range(robot_cell_x - search_radius, robot_cell_x + search_radius + 1, 2):
                if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                    continue
                index = cell_y * info.width + cell_x
                if self.map_msg.data[index] < 0 or self.map_msg.data[index] > 25:
                    continue
                neighbor_unknown = False
                frontier_offset = max(2, int(0.8 / info.resolution))
                for offset_x, offset_y in (
                    (frontier_offset, 0), (-frontier_offset, 0),
                    (0, frontier_offset), (0, -frontier_offset),
                ):
                    neighbor_x = cell_x + offset_x
                    neighbor_y = cell_y + offset_y
                    if 0 <= neighbor_x < info.width and 0 <= neighbor_y < info.height:
                        if self.map_msg.data[neighbor_y * info.width + neighbor_x] < 0:
                            neighbor_unknown = True
                            break
                if not neighbor_unknown:
                    continue

                goal_x = info.origin.position.x + (cell_x + 0.5) * info.resolution
                goal_y = info.origin.position.y + (cell_y + 0.5) * info.resolution
                if not self.inside_safe_boundary(goal_x, goal_y):
                    continue
                distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
                if distance < 0.6 or distance > 2.5:
                    continue
                if not self.is_free_goal(goal_x, goal_y):
                    continue
                if not self.is_free_corridor(robot_x, robot_y, goal_x, goal_y):
                    continue

                direction = math.atan2(goal_y - robot_y, goal_x - robot_x)
                relative_angle = math.atan2(
                    math.sin(direction - robot_yaw),
                    math.cos(direction - robot_yaw),
                )
                if self.scan_distance(relative_angle) < distance + 0.35:
                    continue
                candidates.append((distance, goal_x, goal_y, direction))

        if not candidates:
            return None
        _, goal_x, goal_y, yaw = min(candidates)
        if any(math.hypot(goal_x - old_x, goal_y - old_y) < 0.7
               for old_x, old_y in self.visited_exploration_goals):
            return None
        self.visited_exploration_goals.append((goal_x, goal_y))
        self.get_logger().info(
            f'Frontier found at ({goal_x:.2f}, {goal_y:.2f}); moving to expand the map.'
        )
        return goal_x, goal_y, yaw

    def is_free_corridor(self, start_x, start_y, goal_x, goal_y):
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        samples = max(2, int(distance / 0.15))
        for index in range(samples + 1):
            fraction = index / samples
            x = start_x + (goal_x - start_x) * fraction
            y = start_y + (goal_y - start_y) * fraction
            if not self.is_free_goal(x, y):
                return False
        return True

    def scan_distance(self, angle):
        if self.scan_msg is None or self.scan_msg.angle_increment == 0.0:
            return 0.0
        index = round((angle - self.scan_msg.angle_min) / self.scan_msg.angle_increment)
        if not 0 <= index < len(self.scan_msg.ranges):
            return 0.0
        distance = self.scan_msg.ranges[index]
        return distance if math.isfinite(distance) else self.scan_msg.range_max

    def send_goal(self, x, y, yaw, kind):
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(x, y, yaw)
        self.goal_kind = kind
        self.goal_map_version = self.map_version
        self.get_logger().info(f'Sending {kind} goal: x={x:.2f}, y={y:.2f}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal; waiting for a map update.')
            self.goal_handle = None
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        kind = self.goal_kind
        self.goal_handle = None
        self.goal_kind = None
        if status == 4:
            if kind == 'waypoint':
                self.waypoint_idx += 1
            self.get_logger().info(f'{kind.capitalize()} goal reached; waiting for map update.')
        elif kind is not None:
            self.get_logger().warn(f'{kind.capitalize()} goal ended with status {status}.')

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PatrolController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
