"""Editable mission state. The ROS bridge owns action execution and serialization."""
import math


class MissionManager:
    def __init__(self, geofence_manager=None):
        self.waypoints = []
        self.state = 'IDLE'
        self.current_waypoint_index = 0
        self.mission_type = 'WAYPOINT'
        self.geofence_manager = geofence_manager
        self.next_id = 1
        self.message = ''

    def _ensure_editable(self):
        if self.state in ('RUNNING', 'PAUSING', 'PAUSED'):
            raise ValueError('Cancel the running mission before editing it.')

    @staticmethod
    def _coordinates(x, y, z=0.0, yaw=None):
        try:
            values = {'x': float(x), 'y': float(y), 'z': float(z)}
            values['yaw'] = None if yaw is None else float(yaw)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Waypoint coordinates must be finite numbers.') from exc
        if any(v is not None and not math.isfinite(v) for v in values.values()):
            raise ValueError('Waypoint coordinates must be finite numbers.')
        return values

    def _edited(self):
        self.state = 'EDITING' if self.waypoints else 'IDLE'
        self.current_waypoint_index = 0
        self.message = 'Write the mission after editing it.'
        for wp in self.waypoints:
            wp['status'] = 'PLANNED'

    def clear_mission(self):
        self._ensure_editable()
        self.waypoints = []
        self.next_id = 1
        self._edited()

    def add_waypoint(self, x, y, z=0.0, yaw=None):
        self._ensure_editable()
        wp = self._coordinates(x, y, z, yaw)
        wp.update(id=self.next_id, frame='map', status='PLANNED')
        self.waypoints.append(wp)
        self.next_id += 1
        self._edited()

    def update_waypoint(self, wp_id, x, y, z=0.0, yaw=None):
        self._ensure_editable()
        coordinates = self._coordinates(x, y, z, yaw)
        wp = next((wp for wp in self.waypoints if wp['id'] == wp_id), None)
        if wp is None:
            raise ValueError('Waypoint does not exist.')
        wp.update(coordinates)
        self._edited()

    def delete_waypoint(self, wp_id):
        self._ensure_editable()
        if not any(wp['id'] == wp_id for wp in self.waypoints):
            raise ValueError('Waypoint does not exist.')
        self.waypoints = [wp for wp in self.waypoints if wp['id'] != wp_id]
        self._edited()

    def reorder_waypoint(self, wp_id, direction):
        self._ensure_editable()
        idx = next((i for i, wp in enumerate(self.waypoints) if wp['id'] == wp_id), -1)
        if idx < 0 or direction not in ('up', 'down'):
            raise ValueError('Invalid waypoint or reorder direction.')
        other = idx + (-1 if direction == 'up' else 1)
        if 0 <= other < len(self.waypoints):
            self.waypoints[idx], self.waypoints[other] = self.waypoints[other], self.waypoints[idx]
        self._edited()

    def write_mission(self):
        self._ensure_editable()
        if not self.waypoints:
            return False, 'Mission is empty.'
        for wp in self.waypoints:
            self._coordinates(wp['x'], wp['y'], wp['z'], wp['yaw'])
        if self.geofence_manager:
            valid, message = self.geofence_manager.is_valid_mission(self.waypoints)
            if not valid:
                return False, message
        self.state = 'READY'
        self.message = 'Mission ready.'
        return True, f'MISSION WRITTEN\n{len(self.waypoints)} WAYPOINTS\nFRAME: map'

    def start_mission(self):
        if not self.waypoints or self.state != 'READY':
            return False
        self.state = 'RUNNING'
        self.current_waypoint_index = 0
        self.message = 'Navigating to waypoint 1.'
        for wp in self.waypoints:
            wp['status'] = 'PENDING'
        self.waypoints[0]['status'] = 'ACTIVE'
        return True

    def cancel_mission(self):
        self.state = 'CANCELLED'
        self.message = 'Mission cancelled.'
        for wp in self.waypoints:
            if wp['status'] == 'ACTIVE':
                wp['status'] = 'CANCELLED'

    def get_status(self):
        return {
            'state': self.state,
            'waypoints_count': len(self.waypoints),
            'current_index': self.current_waypoint_index,
            'waypoints': self.waypoints,
            'message': self.message,
        }
