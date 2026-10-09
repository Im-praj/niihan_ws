import importlib.util
from pathlib import Path
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,OpaqueFunction
from launch.utilities import perform_substitutions,normalize_to_list_of_substitutions
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]


def load(name):
    spec=importlib.util.spec_from_file_location('test_launch',ROOT/'launch'/name);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.generate_launch_description()


def application_args(name):
    ld=load(name);context=LaunchContext()
    for action in ld.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    include=next(a for a in ld.entities if isinstance(a,IncludeLaunchDescription))
    return {n:perform_substitutions(context,normalize_to_list_of_substitutions(v)) for n,v in include.launch_arguments}


def test_sim_and_hardware_include_same_application_with_correct_clock():
    for name,clock in [('niihan_simulation.launch.py','true'),('niihan_hardware.launch.py','false')]:
        ld=load(name);include=next(a for a in ld.entities if isinstance(a,IncludeLaunchDescription))
        include.launch_description_source.get_launch_description(LaunchContext())
        assert 'niihan_application.launch.py' in include.launch_description_source.location
        assert application_args(name)['use_sim_time']==clock
    assert application_args('niihan_hardware.launch.py')['drive_enabled']=='false'


def test_only_local_ekf_owns_odom_tf_and_slam_owns_map_tf():
    config=yaml.safe_load((ROOT/'config/localization.yaml').read_text())
    assert config['ekf_local']['ros__parameters']['world_frame']=='odom'
    assert config['ekf_local']['ros__parameters']['publish_tf'] is True
    assert config['ekf_gnss']['ros__parameters']['publish_tf'] is False
    assert config['ekf_gnss']['ros__parameters']['world_frame']=='gnss_local'


def test_hardware_never_starts_mock_or_gazebo():
    text=(ROOT/'launch/niihan_hardware.launch.py').read_text()
    assert 'mock_hardware' not in text and 'ros_gz_sim' not in text and 'gazebo' not in text


def test_common_application_does_not_publish_motor_transport():
    ld=load('niihan_application.launch.py')
    # Transport backends are selected only by the outer launch.
    text=(ROOT/'launch/niihan_application.launch.py').read_text()
    assert "'stm32_bridge'" not in text and "'mock_hardware'" not in text
    assert "'health_supervisor'" in text and "'motion_gateway'" in text
