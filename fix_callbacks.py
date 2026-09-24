import re

with open('src/niihan_dashboard/niihan_dashboard/ros_bridge.py', 'r') as f:
    text = f.read()

# Replace send_nav_goal and send_waypoints and their callbacks
old_funcs = re.search(r'    def send_nav_goal\(self.*?def _get_result_callback\(self, future\):\n        result = future.result\(\)\n        self.get_logger\(\).info\(\'Result: \{0\}\'.format\(result.result\)\)\n        self.mission_manager.state = "IDLE"', text, re.DOTALL)

if old_funcs:
    new_funcs = """    def send_nav_goal(self, x, y, yaw):
        self.nav_goal_status = 'EXECUTING'
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
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)
        self.get_logger().info(f"Sent nav goal: x={x}, y={y}")

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav goal rejected')
            self.nav_goal_status = 'REJECTED'
            return
        
        self.get_logger().info('Nav goal accepted')
        self.active_nav_goal_handle = goal_handle
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
            self.nav_goal_status = 'SUCCEEDED'
        elif result.status == 5: # CANCELED
            self.nav_goal_status = 'CANCELED'
        else:
            self.nav_goal_status = 'ABORTED'
        self.get_logger().info(f'Nav goal result: {self.nav_goal_status}')
        self.active_nav_goal_handle = None

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
        future.add_done_callback(self.fw_goal_response_callback)
        self.get_logger().info("Sent waypoints mission")

    def fw_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.mission_manager.state = "FAILED"
            return
        
        self.get_logger().info('Goal accepted')
        self.active_fw_goal_handle = goal_handle
        self._get_fw_result_future = goal_handle.get_result_async()
        self._get_fw_result_future.add_done_callback(self.fw_result_callback)

    def fw_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Result: {0}'.format(result.result))
        self.mission_manager.state = "IDLE"
        self.active_fw_goal_handle = None"""

    text = text.replace(old_funcs.group(0), new_funcs)
    
    with open('src/niihan_dashboard/niihan_dashboard/ros_bridge.py', 'w') as f:
        f.write(text)
else:
    print("Match failed")
