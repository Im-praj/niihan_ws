import re

with open('src/niihan_dashboard/niihan_dashboard/ros_bridge.py', 'r') as f:
    text = f.read()

# 1. Add active goal handles in __init__
text = text.replace("self.geofence_manager = GeofenceManager()", "self.geofence_manager = GeofenceManager()\n        self.active_nav_goal_handle = None\n        self.active_fw_goal_handle = None\n        self.nav_goal_status = 'IDLE'")

# 2. Fix safety_loop
old_safety_loop = """    def safety_loop(self):
        if self.safety_manager.mode == "MANUAL" and not self.safety_manager.can_move(self.telemetry["pose"]["x"], self.telemetry["pose"]["y"]):
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            
        if self.safety_manager.estop_active:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)"""

new_safety_loop = """    def safety_loop(self):
        # Geofence check runs every tick regardless of mode
        if not self.geofence_manager.is_robot_inside(self.telemetry["pose"]["x"], self.telemetry["pose"]["y"]):
            if not self.safety_manager.estop_active:
                self.safety_manager.trigger_estop()
                self.get_logger().error("GEOFENCE BREACH: E-STOP TRIGGERED")
                # Also cancel goals
                self.mission_manager.cancel_mission()
                if self.active_nav_goal_handle:
                    self.active_nav_goal_handle.cancel_goal_async()
                    self.active_nav_goal_handle = None
                if self.active_fw_goal_handle:
                    self.active_fw_goal_handle.cancel_goal_async()
                    self.active_fw_goal_handle = None

        if self.safety_manager.mode == "MANUAL" and not self.safety_manager.can_move(self.telemetry["pose"]["x"], self.telemetry["pose"]["y"]):
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            
        if self.safety_manager.estop_active:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)"""

text = text.replace(old_safety_loop, new_safety_loop)

# 3. Fix estop cancel
text = text.replace("self.nav_to_pose_client._cancel_goal_async(None)", "if self.active_nav_goal_handle:\n                self.active_nav_goal_handle.cancel_goal_async()\n                self.active_nav_goal_handle = None")
text = text.replace("self.follow_waypoints_client._cancel_goal_async(None)", "if self.active_fw_goal_handle:\n                self.active_fw_goal_handle.cancel_goal_async()\n                self.active_fw_goal_handle = None")

# 4. Fix nav_goal process command
old_nav_goal = """        elif action == "nav_goal":
            if self.safety_manager.mode == "AUTO" and not self.safety_manager.estop_active:
                x = float(cmd_data.get("x", 0.0))
                y = float(cmd_data.get("y", 0.0))
                yaw = float(cmd_data.get("yaw", 0.0))
                self.send_nav_goal(x, y, yaw)"""

new_nav_goal = """        elif action == "nav_goal":
            if self.safety_manager.mode == "AUTO" and not self.safety_manager.estop_active:
                x = float(cmd_data.get("x", 0.0))
                y = float(cmd_data.get("y", 0.0))
                yaw = float(cmd_data.get("yaw", 0.0))
                if self.geofence_manager.is_robot_inside(x, y):
                    self.send_nav_goal(x, y, yaw)
                else:
                    self.get_logger().error("Nav goal rejected: outside geofence.")
                    self.nav_goal_status = 'REJECTED'
                    return {"type": "nav_goal_response", "success": False, "message": "Goal outside geofence"}"""

text = text.replace(old_nav_goal, new_nav_goal)

# 5. Fix send_nav_goal and send_waypoints callbacks
# Wait, I need to rewrite send_nav_goal and send_waypoints to store handles properly and use unique callbacks.

with open('src/niihan_dashboard/niihan_dashboard/ros_bridge.py', 'w') as f:
    f.write(text)
