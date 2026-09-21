class MissionManager:
    def __init__(self, geofence_manager=None):
        self.waypoints = []
        self.state = "IDLE"  # IDLE, READY, RUNNING, PAUSED, CANCELLED, COMPLETED, FAILED
        self.current_waypoint_index = 0
        self.mission_type = "WAYPOINT"
        self.geofence_manager = geofence_manager
        self.next_id = 1

    def clear_mission(self):
        self.waypoints = []
        self.state = "IDLE"
        self.current_waypoint_index = 0
        self.next_id = 1

    def add_waypoint(self, x, y, z, yaw):
        wp = {
            "id": self.next_id,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "frame": "map",
            "status": "PLANNED"
        }
        self.waypoints.append(wp)
        self.next_id += 1
        self.state = "EDITING"

    def update_waypoint(self, wp_id, x, y, z, yaw):
        for wp in self.waypoints:
            if wp["id"] == wp_id:
                wp["x"] = x
                wp["y"] = y
                wp["z"] = z
                wp["yaw"] = yaw
                break

    def delete_waypoint(self, wp_id):
        self.waypoints = [wp for wp in self.waypoints if wp["id"] != wp_id]

    def reorder_waypoint(self, wp_id, direction):
        idx = next((i for i, wp in enumerate(self.waypoints) if wp["id"] == wp_id), -1)
        if idx != -1:
            if direction == "up" and idx > 0:
                self.waypoints[idx], self.waypoints[idx-1] = self.waypoints[idx-1], self.waypoints[idx]
            elif direction == "down" and idx < len(self.waypoints) - 1:
                self.waypoints[idx], self.waypoints[idx+1] = self.waypoints[idx+1], self.waypoints[idx]

    def write_mission(self):
        if not self.waypoints:
            return False, "Mission is empty."
        if self.geofence_manager:
            valid, msg = self.geofence_manager.is_valid_mission(self.waypoints)
            if not valid:
                return False, msg
        self.state = "READY"
        return True, f"MISSION WRITTEN\n{len(self.waypoints)} WAYPOINTS\nFRAME: map"

    def start_mission(self):
        if not self.waypoints or self.state != "READY":
            return False
        self.state = "RUNNING"
        self.current_waypoint_index = 0
        for wp in self.waypoints:
            wp["status"] = "PLANNED"
        if self.waypoints:
            self.waypoints[0]["status"] = "ACTIVE"
        return True

    def cancel_mission(self):
        self.state = "CANCELLED"
        for wp in self.waypoints:
            if wp["status"] == "ACTIVE":
                wp["status"] = "CANCELLED"

    def get_status(self):
        return {
            "state": self.state,
            "waypoints_count": len(self.waypoints),
            "current_index": self.current_waypoint_index,
            "waypoints": self.waypoints
        }
