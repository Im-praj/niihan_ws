"""Evaluate launch parameters without executing nodes or starting robot motion."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from launch.utilities import perform_substitutions
from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]


def load_launch(path):
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Resolve owned packages from source, so no build/old overlay can mask failures.
    def share(package):
        local = PACKAGE.parent / package
        return str(local) if local.is_dir() else get_package_share_directory(package)
    module.get_package_share_directory = share
    return module.generate_launch_description()


def walk(entities):
    for entity in entities:
        yield entity
        if not isinstance(entity, Node) and hasattr(entity, 'get_sub_entities'):
            yield from walk(entity.get_sub_entities())


@pytest.mark.parametrize('sim_time', ['true', 'false'])
@pytest.mark.parametrize('path', sorted((PACKAGE / 'launch').glob('*.launch.py')))
def test_nodes_honor_shared_clock(path, sim_time):
    description = load_launch(path)
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = sim_time
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for node in (action for action in walk(description.entities) if isinstance(action, Node)):
        clock = []
        for parameter in node._Node__parameters:
            if not isinstance(parameter, dict):
                continue
            for key, value in parameter.items():
                if perform_substitutions(context, key) == 'use_sim_time':
                    clock = evaluate_parameters(context, [{key: value}])
        assert list(clock) == [{'use_sim_time': sim_time == 'true'}], str(path)


@pytest.mark.parametrize('sim_time', ['true', 'false'])
def test_dashboard_honors_shared_clock(sim_time):
    path = PACKAGE.parent / 'niihan_dashboard' / 'launch' / 'dashboard.launch.py'
    test_nodes_honor_shared_clock(path, sim_time)
