import math

class GeofenceManager:
    def __init__(self):
        self.polygon = []
        self.enabled = False
        self.frame = "map"

    def set_geofence(self, polygon):
        """
        polygon: List of dictionaries [{'x': float, 'y': float}, ...]
        """
        self.polygon = polygon
        self.enabled = True

    def clear_geofence(self):
        self.polygon = []
        self.enabled = False

    def is_robot_inside(self, x, y):
        if not self.enabled or not self.polygon:
            return True # If no geofence is defined, everything is inside
        
        n = len(self.polygon)
        if n < 3:
            return True # Not a valid polygon
            
        inside = False
        p1x, p1y = self.polygon[0]['x'], self.polygon[0]['y']
        for i in range(1, n + 1):
            p2x, p2y = self.polygon[i % n]['x'], self.polygon[i % n]['y']
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def do_segments_intersect(self, p1, p2, p3, p4):
        def ccw(A, B, C):
            return (C['y'] - A['y']) * (B['x'] - A['x']) > (B['y'] - A['y']) * (C['x'] - A['x'])
        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def does_segment_cross_geofence(self, p1, p2):
        if not self.enabled or not self.polygon:
            return False
        n = len(self.polygon)
        if n < 3:
            return False
            
        for i in range(n):
            p3 = self.polygon[i]
            p4 = self.polygon[(i + 1) % n]
            if self.do_segments_intersect(p1, p2, p3, p4):
                return True
        return False

    def is_valid_mission(self, waypoints):
        if not self.enabled or not self.polygon:
            return True, "No geofence defined."
            
        if len(self.polygon) < 3:
            return True, "Geofence polygon is invalid."
        
        for wp in waypoints:
            if not self.is_robot_inside(wp['x'], wp['y']):
                return False, f"Waypoint {wp.get('id', '?')} is outside the geofence."
        
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i+1]
            if self.does_segment_cross_geofence(p1, p2):
                return False, f"Path from WP {p1.get('id', '?')} to WP {p2.get('id', '?')} crosses the geofence."
        
        return True, "Mission is valid."

    def get_status(self):
        return {
            "enabled": self.enabled,
            "vertices": len(self.polygon),
            "polygon": self.polygon
        }
