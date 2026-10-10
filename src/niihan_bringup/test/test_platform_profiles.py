"""Native platform selection and cross-distribution navigation contracts."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
import pytest

ROOT = Path(__file__).resolve().parents[3]

@pytest.mark.parametrize('version,distro,bridge', [
    ('22.04', 'humble', 'ros-humble-ros-gzharmonic'),
    ('24.04', 'jazzy', 'ros-jazzy-ros-gz'),
])
def test_native_selection(tmp_path, version, distro, bridge):
    release = tmp_path / 'os-release'
    release.write_text(f'ID=ubuntu\nVERSION_ID={version}\n')
    output = subprocess.check_output(['bash', '-c',
        'source "$1"; niihan_select_platform "$2" x86_64; printf "%s %s" "$NIIHAN_ROS_DISTRO" "$NIIHAN_GZ_BRIDGE"',
        'test', str(ROOT/'scripts/platform.sh'), str(release)], text=True)
    assert output == f'{distro} {bridge}'

@pytest.mark.parametrize('os,version,arch', [('ubuntu','20.04','x86_64'), ('debian','12','x86_64'), ('ubuntu','24.04','aarch64')])
def test_unsupported_host_fails(tmp_path, os, version, arch):
    release = tmp_path/'os-release'
    release.write_text(f'ID={os}\nVERSION_ID={version}\n')
    assert subprocess.run(['bash','-c','source "$1"; niihan_select_platform "$2" "$3"',
        'test',str(ROOT/'scripts/platform.sh'),str(release),arch],capture_output=True).returncode != 0

def test_jazzy_nav2_keeps_drive_contract():
    config=ROOT/'src/niihan_slam/config'
    d=yaml.safe_load((config/'nav2_3d_jazzy.yaml').read_text())
    for name in ('controller_server','behavior_server','velocity_smoother'):
        assert d[name]['ros__parameters']['enable_stamped_cmd_vel'] is False
    bt=d['bt_navigator']['ros__parameters']
    assert 'plugin_lib_names' not in bt  # built-ins auto-load in Jazzy
    assert bt['navigate_to_pose']['plugin']=='nav2_bt_navigator::NavigateToPoseNavigator'
    assert d['controller_server']['ros__parameters']['progress_checker_plugins']==['progress_checker']
    tree=ET.parse(config/'navigate_to_pose_position_jazzy.xml').getroot()
    assert tree.get('BTCPP_format')=='4'
    assert tree.find('.//GoalReached') is not None

@pytest.mark.parametrize('profile', ['nav2_3d.yaml', 'nav2_3d_jazzy.yaml'])
def test_planner_controller_arrival_budget(profile):
    # Smac ends at a cell center. Its cell quantization plus controller stop
    # allowance must fit inside the BT's original-goal arrival condition.
    import math
    d=yaml.safe_load((ROOT/'src/niihan_slam/config'/profile).read_text())
    controller=d['controller_server']['ros__parameters']
    checker=controller[controller['goal_checker_plugins'][0]]
    grid=d['global_costmap']['global_costmap']['ros__parameters']
    bt=d['bt_navigator']['ros__parameters']
    planner=d['planner_server']['ros__parameters']['GridBased']
    assert planner['tolerance']==0.0
    assert checker['stateful'] is False
    assert checker['yaw_goal_tolerance']>=math.pi
    assert math.sqrt(2)*grid['resolution']/2+checker['xy_goal_tolerance'] < bt['goal_reached_tol']
