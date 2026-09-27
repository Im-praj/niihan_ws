import math


class MissionManager:
    def __init__(self, geofence_manager=None, frame="map"):
        self.waypoints = []
        self.state = "IDLE"  # IDLE, EDITING, READY, RUNNING, CANCELLED, COMPLETED, FAILED
        self.current_waypoint_index = 0
        self.mission_type = "WAYPOINT"
        self.geofence_manager = geofence_manager
        self.frame = frame
        self.next_id = 1
        self.message = ""

    def _require_editable(self):
        if self.state == "RUNNING":
            raise ValueError("Cancel the running mission before editing waypoints.")

    @staticmethod
    def validate_pose(x, y, z, yaw):
        values = tuple(float(value) for value in (x, y, z, yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Waypoint coordinates and yaw must be finite numbers.")
        x, y, z, yaw = values
        return x, y, z, math.atan2(math.sin(yaw), math.cos(yaw))

    def _mark_edited(self):
        self.state = "EDITING" if self.waypoints else "IDLE"
        self.current_waypoint_index = 0
        self.message = "Write the mission again after editing."
        for wp in self.waypoints:
            wp["status"] = "PLANNED"

    def clear_mission(self):
        self._require_editable()
        self.waypoints = []
        self.state = "IDLE"
        self.current_waypoint_index = 0
        self.next_id = 1
        self.message = ""

    def add_waypoint(self, x, y, z, yaw):
        self._require_editable()
        x, y, z, yaw = self.validate_pose(x, y, z, yaw)
        self.waypoints.append({
            "id": self.next_id, "x": x, "y": y, "z": z, "yaw": yaw,
            "frame": self.frame, "status": "PLANNED",
        })
        self.next_id += 1
        self._mark_edited()

    def update_waypoint(self, wp_id, x, y, z, yaw):
        self._require_editable()
        x, y, z, yaw = self.validate_pose(x, y, z, yaw)
        wp = next((wp for wp in self.waypoints if wp["id"] == wp_id), None)
        if wp is None:
            raise ValueError("Waypoint not found.")
        wp.update(x=x, y=y, z=z, yaw=yaw)
        self._mark_edited()

    def delete_waypoint(self, wp_id):
        self._require_editable()
        self.waypoints = [wp for wp in self.waypoints if wp["id"] != wp_id]
        self._mark_edited()

    def reorder_waypoint(self, wp_id, direction):
        self._require_editable()
        if direction not in ("up", "down"):
            raise ValueError("Direction must be up or down.")
        idx = next((i for i, wp in enumerate(self.waypoints) if wp["id"] == wp_id), -1)
        if idx == -1:
            raise ValueError("Waypoint not found.")
        other = idx - 1 if direction == "up" else idx + 1
        if 0 <= other < len(self.waypoints):
            self.waypoints[idx], self.waypoints[other] = self.waypoints[other], self.waypoints[idx]
            self._mark_edited()

    def write_mission(self):
        self._require_editable()
        if not self.waypoints:
            return False, "Mission is empty."
        if self.geofence_manager:
            valid, msg = self.geofence_manager.is_valid_mission(self.waypoints)
            if not valid:
                self.state = "EDITING"
                self.message = msg
                return False, msg
        self.state = "READY"
        self.message = "Mission ready."
        return True, f"MISSION WRITTEN\n{len(self.waypoints)} WAYPOINTS\nFRAME: {self.frame}"

    def start_mission(self):
        if not self.waypoints or self.state != "READY":
            return False
        if self.geofence_manager:
            valid, message = self.geofence_manager.is_valid_mission(self.waypoints)
            if not valid:
                self.state = "EDITING"
                self.message = message
                return False
        self.state = "RUNNING"
        self.message = "Waiting for Nav2 to accept the mission."
        self.current_waypoint_index = 0
        for wp in self.waypoints:
            wp["status"] = "PLANNED"
        self.waypoints[0]["status"] = "ACTIVE"
        return True

    def update_progress(self, index):
        if self.state != "RUNNING" or not self.current_waypoint_index <= index < len(self.waypoints):
            return False
        changed = index != self.current_waypoint_index
        self.current_waypoint_index = index
        self.message = f"Navigating to waypoint {index + 1} of {len(self.waypoints)}."
        for i, wp in enumerate(self.waypoints):
            # FollowWaypoints feedback only reports an index; skipped goals are
            # not known until the result's missed_waypoints list arrives.
            wp["status"] = "PASSED" if i < index else "ACTIVE" if i == index else "PLANNED"
        return changed

    def complete_mission(self):
        if self.state != "RUNNING":
            return
        self.state = "COMPLETED"
        self.message = "Nav2 confirmed every waypoint reached."
        self.current_waypoint_index = len(self.waypoints) - 1
        for wp in self.waypoints:
            wp["status"] = "COMPLETED"

    def fail_mission(self, message, missed_waypoints=()):
        if self.state != "RUNNING":
            return
        self.state = "FAILED"
        self.message = message
        missed = set(missed_waypoints)
        for index, wp in enumerate(self.waypoints):
            if index in missed or wp["status"] == "ACTIVE":
                wp["status"] = "FAILED"

    def cancel_mission(self, message="Mission cancelled."):
        if self.state not in ("READY", "RUNNING"):
            return
        self.state = "CANCELLED"
        self.message = message
        for wp in self.waypoints:
            if wp["status"] == "ACTIVE":
                wp["status"] = "CANCELLED"

    def get_status(self):
        return {
            "state": self.state,
            "message": self.message,
            "waypoints_count": len(self.waypoints),
            "current_index": self.current_waypoint_index,
            "waypoints": [dict(wp) for wp in self.waypoints],
        }
