"""Construction world geometry and launcher reproducibility contracts."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from test_launch_sim_time import PACKAGE, load_launch


def test_baseline_has_static_visible_collidable_construction_geometry():
    if not (PACKAGE / 'models/construction_site_chunk5/LICENSE').exists():
        pytest.skip('Default scene requires the supplied proprietary assets')
    world = ET.parse(PACKAGE / 'worlds/niihan_construction_site.sdf').getroot().find('world')
    models = world.findall('model')
    assert len(models) >= 50
    assert all(model.findtext('static') == 'true' for model in models)
    assert all(model.find('.//visual') is not None for model in models)
    assert world.find("model[@name='moving_obstacle']") is None
    for name in ['imported_site_static_collision', 'crane_tower', 'charging_dock', 'unfinished_floor', 'site_office']:
        assert world.find(f"model[@name='{name}']/.//collision") is not None
    assert float(world.findtext('physics/max_step_size')) == 0.001
    plugins = {p.attrib['name'] for p in world.findall('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::Sensors',
            'gz::sim::systems::Imu', 'gz::sim::systems::Contact',
            'gz::sim::systems::NavSat'} <= plugins
    for uri in world.findall('.//uri'):
        assert (PACKAGE / 'worlds' / uri.text).resolve().is_file()


def test_legacy_meshes_resolve_locally_with_license():
    path = PACKAGE / 'worlds/construction_site_chunk5.sdf'
    if not (PACKAGE / 'models/construction_site_chunk5/LICENSE').exists():
        pytest.skip('Optional proprietary assets are absent from public checkout')
    for uri in ET.parse(path).getroot().iter('uri'):
        assert (path.parent / uri.text).resolve().is_file()
    license_path = PACKAGE / 'models/construction_site_chunk5/LICENSE'
    assert 'Proprietary' in license_path.read_text()


@pytest.mark.parametrize('headless', ['true', 'false'])
def test_seed_and_headless_arguments_reach_gazebo(headless):
    description = load_launch(PACKAGE / 'launch/niihan_gazebo.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(headless=headless, seed='123')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    include = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription)
                   and 'gz_args' in dict(a.launch_arguments))
    args = perform_substitutions(context, normalize_to_list_of_substitutions(dict(include.launch_arguments)['gz_args']))
    assert 'niihan_construction_site.sdf' in args
    assert '--seed 123' in args
    assert ' -s ' in args
    timer = next(a for a in description.entities if isinstance(a, TimerAction) and any(isinstance(x, IncludeLaunchDescription) and dict(x.launch_arguments).get('gz_args')=='-g' for x in a.actions))
    gui = next(x for x in timer.actions if isinstance(x, IncludeLaunchDescription))
    assert gui.condition.evaluate(context) == (headless == 'false')
    assert ('--headless-rendering' in args) == (headless == 'true')


def test_construction_scene_is_distinct_from_legacy_default():
    current = ET.parse(PACKAGE / 'worlds/niihan_construction_site.sdf').getroot()
    legacy = ET.parse(PACKAGE / 'worlds/niihan_patrol_base.world').getroot()
    names = {m.get('name') for m in current.findall('world/model')}
    old_names = {m.get('name') for m in legacy.findall('world/model')}
    assert len(names - old_names) > 20
    assert 'crane_tower' in names - old_names
