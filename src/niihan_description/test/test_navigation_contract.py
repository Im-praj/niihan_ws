"""Regression checks for coherent goal, replan, geofence and launch contracts."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from test_launch_sim_time import PACKAGE, load_launch

CONFIG = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())


def test_planned_endpoint_cannot_be_half_a_meter_short():
    checker = CONFIG['controller_server']['ros__parameters']['general_goal_checker']
    planner = CONFIG['planner_server']['ros__parameters']['GridBased']
    assert planner['tolerance'] <= checker['xy_goal_tolerance'] <= 0.2
    assert checker['stateful'] is False


@pytest.mark.parametrize('tree', ['navigate_to_pose.xml', 'navigate_through_poses.xml'])
def test_replanning_runs_while_following_the_path(tree):
    root = ET.parse(PACKAGE / 'config' / tree).getroot()
    pipeline = root.find('.//PipelineSequence')
    assert pipeline is not None
    assert float(pipeline.find('RateController').attrib['hz']) == 2.0
    assert pipeline.find('.//FollowPath') is not None
    assert root.find('.//ClearEntireCostmap') is not None
    assert root.find('.//Wait') is not None
    remove_passed = root.find('.//RemovePassedGoals')
    if remove_passed is not None:
        assert float(remove_passed.attrib['radius']) <= 0.2


@pytest.mark.parametrize('name', ['global_costmap', 'local_costmap'])
def test_live_obstacles_and_geofence_reach_both_costmaps(name):
    costmap = CONFIG[name][name]['ros__parameters']
    assert 'keepout_filter' in costmap['filters']
    keepout = costmap['keepout_filter']
    assert keepout['plugin'] == 'nav2_costmap_2d::KeepoutFilter'
    assert keepout['filter_info_topic'] == '/niihan/geofence_filter_info'
    assert keepout['enabled'] is True
    scan = costmap['obstacle_layer']['scan']
    assert scan['marking'] and scan['clearing'] and scan['inf_is_valid']
    # Use sensor header frame IDs rather than one simulator's generated alias.
    assert not scan.get('sensor_frame')


@pytest.mark.parametrize('name', ['mapping.launch.py', 'niihan_mapping.launch.py'])
def test_mapping_wrapper_does_not_start_duplicate_3d_publishers(name):
    description = load_launch(PACKAGE / 'launch' / name)
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'true'
    gazebo = next(action for action in description.entities
                  if isinstance(action, IncludeLaunchDescription)
                  and 'niihan_gazebo.launch.py' in perform_substitutions(
                      context, action.launch_description_source._LaunchDescriptionSource__location))
    args = dict(gazebo.launch_arguments)
    assert args['mapping'] == 'false'
    assert args['vortex_3d'] == 'false'
    if name == 'niihan_mapping.launch.py':
        assert args['cliff_detect'] == 'false'
    assert perform_substitutions(context, [args['use_sim_time']]) == 'true'


def test_collision_horizon_uses_supported_humble_parameter():
    follow = CONFIG['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_allowed_time_to_collision_up_to_carrot'] > 0
    assert 'max_allowed_time_to_collision_up_to_carrots' not in follow


def test_upstream_nav2_boolean_conditions_are_evaluable():
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions
    from test_launch_sim_time import walk

    description = load_launch(PACKAGE / 'launch' / 'niihan_navigation.launch.py')
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    include = next(action for action in walk(description.entities)
                   if isinstance(action, IncludeLaunchDescription))
    for name, value in include.launch_arguments:
        context.launch_configurations[name] = perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
    upstream = include.launch_description_source.get_launch_description(context)
    for action in upstream.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for action in walk(upstream.entities):
        if action.condition is not None:
            assert isinstance(action.condition.evaluate(context), bool)
    params = yaml.safe_load(Path(context.launch_configurations['params_file']).read_text())
    for key in ['default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml']:
        assert Path(params['bt_navigator']['ros__parameters'][key]).is_file()
