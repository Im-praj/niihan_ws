"""Read XYZ without assuming packed, native-endian PointCloud2 rows."""
import numpy as np


def xyz_array(msg):
    """Return Nx3 float32 XYZ; reject malformed/unsupported cloud layouts."""
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('Point cloud must contain x, y and z fields')
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid point cloud stride')
    if len(msg.data) < msg.height * msg.row_step:
        raise ValueError('Truncated point cloud data')
    endian = '>' if msg.is_bigendian else '<'
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        size = {7: 4, 8: 8}.get(field.datatype)
        if size is None or field.count != 1 or field.offset < 0 or field.offset + size > msg.point_step:
            raise ValueError('XYZ fields must be scalar FLOAT32 or FLOAT64 within point_step')
        formats.append(endian + ('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)
    if not msg.width or not msg.height:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step))
    return np.column_stack([rows[name].ravel() for name in ('x', 'y', 'z')]).astype(np.float32)


def ground_observation_mask(x, y, translation, rotation, min_elevation, max_elevation,
                            exclusion_radius):
    """Ground locations inside the lidar's vertical FOV, outside the robot body.

    Missing returns in an unobservable region are unknown, not cliff evidence.
    Rotation maps sensor-frame vectors into the robot frame.
    """
    points = np.stack(np.broadcast_arrays(x, y, np.zeros_like(x + y)), axis=-1)
    sensor_vectors = (points - np.asarray(translation)) @ np.asarray(rotation)
    horizontal_range = np.hypot(sensor_vectors[..., 0], sensor_vectors[..., 1])
    elevation = np.arctan2(sensor_vectors[..., 2], horizontal_range)
    return ((elevation >= min_elevation) & (elevation <= max_elevation) &
            (np.hypot(x, y) > exclusion_radius))
