"""Pure, tested geometry, GNSS, transport and watchdog functions."""
import math
import binascii
from dataclasses import dataclass
import numpy as np


def packet(payload):
    raw = payload.encode('ascii')
    return raw + f'*{binascii.crc_hqx(raw, 0xffff):04X}\n'.encode('ascii')


def unpack_packet(line):
    if len(line) > 256:
        raise ValueError('oversize packet')
    raw, checksum = line.strip().rsplit(b'*', 1)
    if len(checksum) != 4 or binascii.crc_hqx(raw, 0xffff) != int(checksum, 16):
        raise ValueError('packet checksum')
    return raw.decode('ascii').split(',')


def sequence_is_newer(current, previous):
    delta = (current - previous) & 0xffffffff
    return 0 < delta < 0x80000000


def ecef(lat, lon, alt):
    if not all(math.isfinite(v) for v in (lat, lon, alt)) or abs(lat) > 90 or abs(lon) > 180:
        raise ValueError('invalid GNSS coordinate')
    lat, lon = math.radians(lat), math.radians(lon)
    a, e2 = 6378137.0, 6.69437999014e-3
    n = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    return np.array([(n+alt)*math.cos(lat)*math.cos(lon),
                     (n+alt)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+alt)*math.sin(lat)])


def enu(lat, lon, alt, datum):
    lat0, lon0, alt0 = datum
    delta = ecef(lat, lon, alt) - ecef(lat0, lon0, alt0)
    p, l = math.radians(lat0), math.radians(lon0)
    rotation = np.array([[-math.sin(l), math.cos(l), 0],
                         [-math.sin(p)*math.cos(l), -math.sin(p)*math.sin(l), math.cos(p)],
                         [math.cos(p)*math.cos(l), math.cos(p)*math.sin(l), math.sin(p)]])
    return rotation @ delta


def project_scan(points, bins=360, low=0.10, high=1.8, minimum=0.3, maximum=25.0):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1) & (points[:,2] >= low) & (points[:,2] <= high)
    points = points[valid]
    ranges = np.full(bins, np.inf)
    distance = np.hypot(points[:,0], points[:,1])
    angle = np.arctan2(points[:,1], points[:,0])
    index = np.floor((angle + math.pi) * bins / (2*math.pi)).astype(int) % bins
    valid = (distance >= minimum) & (distance <= maximum)
    np.minimum.at(ranges, index[valid], distance[valid])
    return ranges.tolist()


@dataclass
class DriveState:
    timeout: float = 0.25
    enabled: bool = False
    estop: bool = False
    permitted: bool = False
    received: float = -math.inf
    linear: float = 0.0
    angular: float = 0.0

    def receive(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self.received = -math.inf
            return
        self.linear, self.angular, self.received = linear, angular, now

    def output(self, now):
        if not self.enabled or self.estop or not self.permitted or not 0 <= now-self.received <= self.timeout:
            return 0.0, 0.0
        return self.linear, self.angular


def ubx_checksum(data):
    a = b = 0
    for byte in data:
        a = (a+byte) & 255
        b = (b+a) & 255
    return bytes((a,b))


class UbxParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 8:
            pos = self.buffer.find(b'\xb5\x62')
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xb5' else b''
                break
            del self.buffer[:pos]
            if len(self.buffer) < 8:
                break
            size = int.from_bytes(self.buffer[4:6], 'little')
            if size > 4096:
                del self.buffer[0]
                continue
            if len(self.buffer) < size+8:
                break
            frame = bytes(self.buffer[:size+8])
            if ubx_checksum(frame[2:-2]) != frame[-2:]:
                del self.buffer[0]
                continue
            del self.buffer[:size+8]
            packets.append((frame[2],frame[3],frame[6:-2]))
        if len(self.buffer) > 8192:
            self.buffer.clear()
        return packets
