import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, Image, PointCloud2
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math
import json
import base64
import cv2
from cv_bridge import CvBridge
import struct

from niihan_dashboard.safety_manager import SafetyManager
from niihan_dashboard.mission_manager import MissionManager
from niihan_dashboard.geofence_manager import GeofenceManager

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class ROSBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_ros_bridge')
        
        self.geofence_manager = GeofenceManager()
        self.safety_manager = SafetyManager(geofence_manager=self.geofence_manager)
        self.mission_manager = MissionManager(geofence_manager=self.geofence_manager)
        self.cv_bridge = CvBridge()
        
        self.telemetry = {
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "velocity": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            "imu": {"ax": 0.0, "ay": 0.0, "az": 0.0, "wx": 0.0, "wy": 0.0, "wz": 0.0},
            "localization": {"source": "RTAB-MAP", "health": "OK", "confidence": 1.0}
        }
        
        self.map_data = None
        self.latest_image = None
        self.path_history = []
        self.pointcloud_data = []
        
        self.last_pc_time = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/niihan/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/niihan/imu/data', self.imu_callback, 10)
        
        # Map requires transient local to get the latched map on startup
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # RTAB-Map Point Cloud
        self.create_subscription(PointCloud2, '/vortex/point_cloud_map', self.pointcloud_callback, map_qos)
        
        self.create_subscription(Image, '/niihan/sensors/panoramic/front/image_raw', self.image_callback, 10)
        
        # Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        
        # Timer for safety timeout
        self.create_timer(0.1, self.safety_loop)
        self.create_timer(1.0, self.update_path_history)
        
        self.nav2_ready = False
        self.check_nav2()

    def check_nav2(self):
        self.nav2_ready = self.follow_waypoints_client.server_is_ready()
        
    def pointcloud_callback(self, msg):
        import time
        now = time.time()
        if now - self.last_pc_time < 0.2: # 5 Hz max
            return
        self.last_pc_time = now

        import sensor_msgs_py.point_cloud2 as pc2
        points = []
        
        # Read x, y, z fields, skip some points for downsampling
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.extend([float(p[0]), float(p[1]), float(p[2])])
            
        # Decimate further if still too large
        if len(points) > 30000:
            points = points[::30]
        else:
            points = points[::3]
            
        self.pointcloud_data = points

    def odom_callback(self, msg):
        self.telemetry["pose"]["x"] = msg.pose.pose.position.x
        self.telemetry["pose"]["y"] = msg.pose.pose.position.y
        self.telemetry["pose"]["z"] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.telemetry["pose"]["yaw"] = yaw
        
        self.telemetry["velocity"]["linear_x"] = msg.twist.twist.linear.x
        self.telemetry["velocity"]["linear_y"] = msg.twist.twist.linear.y
        self.telemetry["velocity"]["angular_z"] = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        self.telemetry["imu"]["ax"] = msg.linear_acceleration.x
        self.telemetry["imu"]["ay"] = msg.linear_acceleration.y
        self.telemetry["imu"]["az"] = msg.linear_acceleration.z
        self.telemetry["imu"]["wx"] = msg.angular_velocity.x
        self.telemetry["imu"]["wy"] = msg.angular_velocity.y
        self.telemetry["imu"]["wz"] = msg.angular_velocity.z

    def map_callback(self, msg):
        q = msg.info.origin.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        self.map_data = {
            "type": "map",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "yaw": yaw
            },
            "data": list(msg.data)
        }

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            if result:
                self.latest_image = base64.b64encode(encimg).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def update_path_history(self):
        self.check_nav2()
        if self.telemetry["pose"]["x"] != 0.0 or self.telemetry["pose"]["y"] != 0.0:
            self.path_history.append({"x": self.telemetry["pose"]["x"], "y": self.telemetry["pose"]["y"]})
            if len(self.path_history) > 300:
                self.path_history.pop(0)

    def safety_loop(self):
        if self.safety_manager.mode == "MANUAL" and not self.safety_manager.can_move(self.telemetry["pose"]["x"], self.telemetry["pose"]["y"]):
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            
        if self.safety_manager.estop_active:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)

    def process_command(self, cmd_data):
        action = cmd_data.get("action")
        
        if action == "estop":
            self.safety_manager.trigger_estop()
            self.mission_manager.cancel_mission()
            self.nav_to_pose_client._cancel_goal_async(None)
            self.follow_waypoints_client._cancel_goal_async(None)
            self.get_logger().info("E-STOP ACTIVATED")
            
        elif action == "clear_estop":
            self.safety_manager.clear_estop()
            self.get_logger().info("E-STOP CLEARED")
            
        elif action == "set_mode":
            mode = cmd_data.get("mode", "MANUAL")
            self.safety_manager.set_mode(mode)
            self.get_logger().info(f"Mode set to {mode}")
            
        elif action == "joystick":
            if self.safety_manager.validate_manual_command():
                msg = Twist()
                msg.linear.x = float(cmd_data.get("linear_x", 0.0))
                msg.angular.z = float(cmd_data.get("angular_z", 0.0))
                self.cmd_vel_pub.publish(msg)
                
        elif action == "nav_goal":
            if self.safety_manager.mode == "AUTO" and not self.safety_manager.estop_active:
                x = float(cmd_data.get("x", 0.0))
                y = float(cmd_data.get("y", 0.0))
                yaw = float(cmd_data.get("yaw", 0.0))
                self.send_nav_goal(x, y, yaw)
                
        elif action == "add_waypoint":
            self.mission_manager.add_waypoint(cmd_data.get("x"), cmd_data.get("y"), cmd_data.get("z", 0.0), cmd_data.get("yaw"))
            
        elif action == "update_waypoint":
            self.mission_manager.update_waypoint(cmd_data.get("id"), cmd_data.get("x"), cmd_data.get("y"), cmd_data.get("z", 0.0), cmd_data.get("yaw"))
            
        elif action == "delete_waypoint":
            self.mission_manager.delete_waypoint(cmd_data.get("id"))
            
        elif action == "reorder_waypoint":
            self.mission_manager.reorder_waypoint(cmd_data.get("id"), cmd_data.get("direction"))
            
        elif action == "clear_mission":
            self.mission_manager.clear_mission()
            
        elif action == "write_mission":
            self.check_nav2()
            if not self.nav2_ready:
                return {"type": "mission_write_response", "success": False, "message": "MISSION NOT WRITTEN\nReason: Nav2 FollowWaypoints action unavailable."}
            success, msg = self.mission_manager.write_mission()
            return {"type": "mission_write_response", "success": success, "message": msg}
            
        elif action == "start_mission":
            if self.safety_manager.mode == "AUTO" and not self.safety_manager.estop_active:
                self.send_waypoints()
                
        elif action == "cancel_mission":
            self.mission_manager.cancel_mission()
            self.follow_waypoints_client._cancel_goal_async(None)
            
        elif action == "set_geofence":
            self.geofence_manager.set_geofence(cmd_data.get("polygon", []))
            
        elif action == "clear_geofence":
            self.geofence_manager.clear_geofence()

        return None

    def send_nav_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = cy
        goal_msg.pose.pose.orientation.z = sy
        
        self.nav_to_pose_client.wait_for_server(timeout_sec=1.0)
        self.nav_to_pose_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent nav goal: x={x}, y={y}")

    def send_waypoints(self):
        if not self.mission_manager.start_mission():
            return
            
        goal_msg = FollowWaypoints.Goal()
        for wp in self.mission_manager.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = wp.get("frame", "map")
            pose.pose.position.x = wp["x"]
            pose.pose.position.y = wp["y"]
            pose.pose.position.z = wp["z"]
            cy = math.cos(wp["yaw"] * 0.5)
            sy = math.sin(wp["yaw"] * 0.5)
            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy
            goal_msg.poses.append(pose)
            
        self.follow_waypoints_client.wait_for_server(timeout_sec=1.0)
        future = self.follow_waypoints_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("Sent waypoints mission")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.mission_manager.state = "FAILED"
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.mission_manager.state = "COMPLETED"
        for wp in self.mission_manager.waypoints:
            wp["status"] = "COMPLETED"

    def get_telemetry_json(self):
        state = {
            "type": "telemetry",
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "pose": self.telemetry["pose"],
            "velocity": self.telemetry["velocity"],
            "mode": self.safety_manager.mode,
            "estop": self.safety_manager.estop_active,
            "mission": self.mission_manager.get_status(),
            "geofence": self.geofence_manager.get_status(),
            "localization": self.telemetry["localization"],
            "path_history": self.path_history,
            "nav2_ready": self.nav2_ready
        }
        return json.dumps(state)

    def get_pointcloud_json(self):
        return json.dumps({
            "type": "pointcloud",
            "data": self.pointcloud_data
        })
