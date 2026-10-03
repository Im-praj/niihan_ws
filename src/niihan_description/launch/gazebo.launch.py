"""Compatibility entrypoint for the canonical NIIHAN simulation launch."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location(
        'niihan_gazebo_launch', Path(__file__).with_name('niihan_gazebo.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
