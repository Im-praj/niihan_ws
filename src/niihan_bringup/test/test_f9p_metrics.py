import struct,json
import numpy as np
import pytest
from niihan_bringup.contracts import f9p_dop,f9p_metrics
from niihan_bringup.rtk_alignment import load_alignment,map_antenna

def test_f9p_dop_reports_receiver_values_not_position_covariance():
    p=bytearray(18);struct.pack_into('<I',p,0,12345);struct.pack_into('<HH',p,10,140,65)
    assert f9p_dop(p)==pytest.approx({'tow':12345,'hdop':.65,'vdop':1.4})
    assert f9p_dop(bytes(18))['hdop'] is None
    with pytest.raises(ValueError):f9p_dop(bytes(17))

@pytest.mark.parametrize('carrier,quality',[(0,'standalone'),(1,'rtk_float'),(2,'rtk_fixed'),(3,'unknown')])
def test_fixed_float_and_satellites_used(carrier,quality):
    p=bytearray(92);p[20]=3;p[21]=1|(carrier<<6);p[23]=19
    struct.pack_into('<II',p,40,20,35)
    d=f9p_metrics(p)
    assert d['quality']==quality and d['satellite_count']==19
    assert d['rtk_lock']==(carrier==2) and d['horizontal_accuracy_m']==.02
    p[21]&=~1
    assert f9p_metrics(p)['lock'] is False
    assert f9p_metrics(p)['rtk_lock'] is False

def test_surveyed_alignment_is_environment_specific(tmp_path):
    p=tmp_path/'alignment.json';d=dict(calibrated=True,environment_id='yard',datum=[0,0,0],map_translation=[10,20,30],map_yaw=np.pi/2)
    p.write_text(json.dumps(d));a=load_alignment(p,'yard')
    assert np.allclose(map_antenna(0,0,0,a),[10,20,30])
    assert map_antenna(0,.00001,0,a)[1]==pytest.approx(21.1131949,rel=1e-5)
    with pytest.raises(ValueError):load_alignment(p,'different-yard')
    d['calibrated']=False;p.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_alignment(p,'yard')

def test_receiver_utc_timestamp_is_not_replaced_with_serial_receive_time():
    from datetime import datetime,timezone
    from niihan_bringup.contracts import f9p_utc_nanoseconds
    p=bytearray(92);struct.pack_into('<H',p,4,2026);p[6:12]=bytes([10,14,12,30,20,7]);struct.pack_into('<i',p,16,-100000000)
    expected=int(datetime(2026,10,14,12,30,20,tzinfo=timezone.utc).timestamp())*1000000000-100000000
    assert f9p_utc_nanoseconds(p)==expected
    p[11]=3
    assert f9p_utc_nanoseconds(p) is None

def test_queued_fix_waits_for_its_tf_instead_of_being_overwritten():
    from collections import deque
    from types import SimpleNamespace as NS
    from sensor_msgs.msg import NavSatFix
    from builtin_interfaces.msg import Time
    from tf2_ros import TransformException
    from niihan_bringup.rtk_alignment import RtkAlignment
    import time
    node=RtkAlignment.__new__(RtkAlignment);node.alignment={'datum':[0,0,0],'map_yaw':0.,'map_translation':[0,0,0]}
    node.allow_simulated=False;node.maximum_horizontal=.05;node.maximum_vertical=.10
    node.received=time.monotonic();node.metrics={'rtk_lock':True};node.metrics_history={};node.pending=deque()
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10000000000));published=[];node.pose=NS(publish=published.append)
    available=[9.7]
    def lookup(parent,child,epoch):
        if epoch.nanoseconds/1e9>available[0]:raise TransformException('TF not ready')
        return NS(header=NS(stamp=Time(sec=9,nanosec=900000000)),transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    node.tf=NS(lookup_transform=lookup)
    for sec,nano in [(9,800000000),(10,0)]:
        msg=NavSatFix();msg.header.stamp=Time(sec=sec,nanosec=nano);msg.status.status=0;msg.position_covariance_type=2
        msg.position_covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0009]
        node.pending.append(msg);stamp=sec+nano/1e9
        node.metrics_history[stamp]={'quality':'rtk_fixed','rtk_lock':True,'simulation':False,'timestamp_source':'receiver_utc'}
    node.publish_pending();assert not published and len(node.pending)==2
    available[0]=9.9;node.publish_pending()
    assert len(published)==1 and len(node.pending)==1
    assert published[0].header.stamp.nanosec==800000000
