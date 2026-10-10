import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
from niihan_dashboard.robot_model import visual_model

@pytest.mark.parametrize('minimal',['false','true'])
def test_actual_model_contains_one_camera_and_all_rover_visuals(minimal):
    source=Path(__file__).resolve().parents[2]/'niihan_description/urdf/niihan.urdf.xacro'
    xml=subprocess.check_output(['xacro',str(source),'minimal_sensors:='+minimal],text=True)
    urdf=ET.fromstring(xml);model=visual_model(xml)
    cameras=[s for s in urdf.findall('.//sensor') if 'camera' in s.get('type','')]
    assert len(cameras)==1
    assert cameras[0].get('name')=='camera_front_sensor'
    assert sum(len(l['visuals']) for l in model['links'])==len(urdf.findall('link/visual'))
    links={l['name'] for l in model['links']}
    assert {'base_link','mast_link','unitree_l2_link','camera_front_link'} <= links
    assert not any('ptz' in name or 'orbbec' in name or 'd435i' in name for name in links)
    assert all(j['parent'] in links and j['child'] in links for j in model['joints'])
