import copy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from niihan_slam.tf_filter import TfFilter

class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)

def transform(parent,child,stamp):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp.sec=stamp
    t.transform.rotation.w=1.;t.transform.translation.z=.5
    return t

def relay(mapping):
    n=object.__new__(TfFilter);n.mapping=mapping;n.pub=Publisher();n.base=n.global_tf=None
    return n

def test_mapping_requires_coherent_timestamps_and_preserves_body_pose():
    n=relay(True);base=transform('odom','base_footprint',10);original=copy.deepcopy(base)
    n.forward(TFMessage(transforms=[base]));assert not n.pub.messages
    n.forward(TFMessage(transforms=[transform('map','odom',9)]));assert not n.pub.messages
    n.forward(TFMessage(transforms=[transform('map','odom',10)]))
    out=n.pub.messages[-1].transforms
    assert len(out)==3 and base==original
    assert out[1].child_frame_id=='base_nav' and out[1].transform.translation.z==0
    assert all(t.header.stamp==base.header.stamp for t in out)
    assert sum(t.child_frame_id=='odom' for t in out)==1

def test_saved_localization_owns_the_only_global_odom_parent():
    n=relay(False)
    n.forward(TFMessage(transforms=[transform('lio_map','odom',10),transform('odom','base_footprint',10)]))
    assert {t.child_frame_id for t in n.pub.messages[-1].transforms}=={'base_footprint','base_nav'}
