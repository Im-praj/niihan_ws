"""Map-frame geofences and a raster keepout mask for Nav2 replanning."""
import math


class GeofenceManager:
    def __init__(self):
        self.polygon = []
        self.enabled = False
        self.frame = 'map'

    @staticmethod
    def _is_finite(point):
        try:
            return math.isfinite(float(point['x'])) and math.isfinite(float(point['y']))
        except (KeyError, TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _dist(p1, p2):
        return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

    @staticmethod
    def _point_segment_distance(x, y, a, b):
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        squared = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - a['x']) * dx + (y - a['y']) * dy) / squared)) if squared else 0.0
        return math.hypot(x - a['x'] - t * dx, y - a['y'] - t * dy)

    def _do_segments_intersect(self, a, b, c, d):
        def cross(p, q, r):
            return (q['x'] - p['x']) * (r['y'] - p['y']) - (q['y'] - p['y']) * (r['x'] - p['x'])
        values = (cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b))
        if values[0] * values[1] < 0 and values[2] * values[3] < 0:
            return True
        # Nonadjacent touching or overlapping edges also make an invalid polygon.
        return any(abs(value) < 1e-9 and self._point_segment_distance(p['x'], p['y'], q, r) < 1e-9
                   for value, p, q, r in ((values[0], c, a, b), (values[1], d, a, b),
                                         (values[2], a, c, d), (values[3], b, c, d)))

    def set_geofence(self, polygon):
        if not isinstance(polygon, list):
            return False, 'Geofence must be a list of points.'
        cleaned = []
        for point in polygon:
            if not self._is_finite(point):
                return False, 'Geofence coordinates must be finite numbers.'
            point = {'x': float(point['x']), 'y': float(point['y'])}
            if not cleaned or self._dist(cleaned[-1], point) > 1e-5:
                cleaned.append(point)
        if len(cleaned) > 1 and self._dist(cleaned[-1], cleaned[0]) <= 1e-5:
            cleaned.pop()
        if len(cleaned) < 3:
            return False, 'Geofence must have at least three unique vertices.'
        n = len(cleaned)
        area2 = sum(cleaned[i]['x'] * cleaned[(i + 1) % n]['y'] -
                    cleaned[(i + 1) % n]['x'] * cleaned[i]['y'] for i in range(n))
        if abs(area2) < 1e-8:
            return False, 'Geofence must enclose a nonzero area.'
        for i in range(n):
            for j in range(i + 2, n):
                if i == 0 and j == n - 1:
                    continue
                if self._do_segments_intersect(cleaned[i], cleaned[(i + 1) % n], cleaned[j], cleaned[(j + 1) % n]):
                    return False, 'Geofence polygon is self-intersecting.'
        self.polygon = cleaned
        self.enabled = True
        return True, 'Geofence updated.'

    def clear_geofence(self):
        self.polygon = []
        self.enabled = False

    def is_robot_inside(self, x, y, margin=0.0):
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        if not self.enabled or not self.polygon:
            return True
        inside = False
        on_boundary = False
        min_distance = math.inf
        for i, a in enumerate(self.polygon):
            b = self.polygon[(i + 1) % len(self.polygon)]
            distance = self._point_segment_distance(x, y, a, b)
            min_distance = min(min_distance, distance)
            on_boundary |= distance < 1e-8
            if (a['y'] > y) != (b['y'] > y):
                intercept = a['x'] + (y - a['y']) * (b['x'] - a['x']) / (b['y'] - a['y'])
                if x < intercept:
                    inside = not inside
        return (inside or on_boundary) and min_distance + 1e-8 >= margin

    def is_valid_mission(self, waypoints, current_pose=None, margin=0.0):
        for wp in waypoints:
            if not self._is_finite(wp):
                return False, 'Invalid waypoint coordinates.'
            if not self.is_robot_inside(float(wp['x']), float(wp['y']), margin):
                return False, f"Waypoint {wp.get('id', '?')} is outside the usable geofence area."
        if current_pose is not None:
            if not self._is_finite(current_pose):
                return False, 'Invalid current pose coordinates.'
            if not self.is_robot_inside(float(current_pose['x']), float(current_pose['y']), margin):
                return False, 'Robot is outside the usable geofence area.'
        # A straight chord can leave a concave polygon even when Nav2 can find
        # a valid detour. The keepout filter constrains the actual planned path.
        return True, 'Mission is valid.'

    def keepout_mask(self, width, height, resolution, origin_x, origin_y, origin_yaw, margin):
        """Cell-center mask, inflated conservatively by half a cell diagonal."""
        import numpy as np
        if not self.enabled:
            return [0] * (width * height)
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        local = []
        for point in self.polygon:
            dx, dy = point['x'] - origin_x, point['y'] - origin_y
            local.append((c * dx + s * dy, -s * dx + c * dy))
        x = (np.arange(width, dtype=float)[None, :] + 0.5) * resolution
        clearance2 = (margin + resolution / math.sqrt(2.0)) ** 2
        result = np.empty((height, width), dtype=np.int8)
        for start in range(0, height, 128):
            y = (np.arange(start, min(height, start + 128), dtype=float)[:, None] + 0.5) * resolution
            inside = np.zeros((len(y), width), dtype=bool)
            boundary = np.zeros_like(inside)
            for index, (ax, ay) in enumerate(local):
                bx, by = local[(index + 1) % len(local)]
                dx, dy = bx - ax, by - ay
                if dy != 0.0:
                    inside ^= ((ay > y) != (by > y)) & (x < ax + (y - ay) * dx / dy)
                fraction = np.clip(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
                distance2 = (x - ax - fraction * dx) ** 2 + (y - ay - fraction * dy) ** 2
                boundary |= distance2 <= clearance2
            result[start:start + len(y)] = np.where(inside & ~boundary, 0, 100)
        return result.ravel().tolist()

    def get_status(self):
        return {'enabled': self.enabled, 'vertices': len(self.polygon), 'polygon': self.polygon}
