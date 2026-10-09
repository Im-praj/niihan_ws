import math
import struct
import pytest
import numpy as np
from niihan_bringup.contracts import packet,unpack_packet,sequence_is_newer,enu,project_scan,DriveState,UbxParser,ubx_checksum


def test_packet_corruption_and_bounds():
    assert unpack_packet(packet('N1,42,1,-1,0'))==['N1','42','1','-1','0']
    damaged=bytearray(packet('N1,42,1,-1,0'));damaged[4]^=1
    with pytest.raises(ValueError):unpack_packet(bytes(damaged))
    with pytest.raises(ValueError):unpack_packet(b'x'*300)


def test_wraparound_sequences():
    assert sequence_is_newer(0,0xffffffff)
    assert not sequence_is_newer(1,1)
    assert not sequence_is_newer(0xffffffff,0)


def test_motion_defaults_stale_permission_estop_and_invalid_command():
    state=DriveState();state.receive(.3,.2,1);assert state.output(1)==(0,0)
    state.enabled=state.permitted=True;assert state.output(1.1)==(.3,.2)
    assert state.output(1.26)==(0,0)
    state.estop=True;assert state.output(1.1)==(0,0)
    state.estop=False;state.receive(math.nan,0,1.2);assert state.output(1.2)==(0,0)
    assert state.output(.9)==(0,0)


def test_gnss_enu_metric_and_dateline():
    assert np.allclose(enu(0,0,0,(0,0,0)),[0,0,0])
    assert enu(0,.00001,0,(0,0,0))[0]==pytest.approx(1.1131949,rel=1e-5)
    assert enu(.00001,0,0,(0,0,0))[1]==pytest.approx(1.1057427,rel=1e-5)
    assert abs(enu(0,-179.99999,0,(0,179.99999,0))[0])<3
    with pytest.raises(ValueError):enu(math.nan,0,0,(0,0,0))


def test_scan_bins_ground_filter_and_nearest_obstacle():
    points=[[2,0,.5],[1,0,.5],[.2,0,.5],[3,0,0],[-2,0,.5],[math.nan,0,.5]]
    scan=project_scan(points)
    assert scan[180]==1
    assert scan[0]==2
    assert sum(math.isfinite(v) for v in scan)==2


def test_ubx_fragmented_corrupt_and_oversized():
    payload=bytes(92);data=b'\x01\x07'+struct.pack('<H',len(payload))+payload
    frame=b'\xb5\x62'+data+ubx_checksum(data)
    parser=UbxParser();assert parser.feed(b'noise'+frame[:30])==[]
    assert parser.feed(frame[30:])==[(1,7,payload)]
    bad=bytearray(frame);bad[-1]^=1
    assert parser.feed(bytes(bad)+frame)==[(1,7,payload)]
    assert parser.feed(b'\xb5\x62\x01\x07\xff\xff'+frame)==[(1,7,payload)]


def test_tilted_ground_rejection_preserves_yaw_coordinates():
    from niihan_bringup.contracts import level_points
    pitch=.19; yaw=.7
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    # Coordinates in the yaw-only frame: far ground and a real obstacle.
    level=np.array([[10,0,-.05],[2,0,.5]])
    body=level @ ry
    q=[-math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2),math.cos(yaw/2)*math.cos(pitch/2)]
    recovered=level_points(body,q)
    assert np.allclose(recovered,level,atol=1e-8)
    scan=project_scan(recovered)
    assert scan[180]==pytest.approx(2)
    assert sum(math.isfinite(v) for v in scan)==1
