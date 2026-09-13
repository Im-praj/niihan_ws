from setuptools import find_packages, setup
from glob import glob

package_name = 'niihan_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/worlds',
            glob('worlds/*')),
        ('share/' + package_name + '/scripts',
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='niihan_description ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'niihan_description_node = niihan_description.niihan_description_node:main',
            'patrol_controller = niihan_description.patrol_controller:main',
            'waypoint_patrol = niihan_description.waypoint_patrol:waypoint_patrol_main',
            'set_patrol_points = niihan_description.set_patrol_points:main',
            'vortex_3d_mapper = niihan_description.vortex_3d_mapper:main',
            'cliff_detector = niihan_description.cliff_detector:main',
            'teleop_mapping_keyboard = niihan_description.teleop_mapping_keyboard:main',
            'teleop_mapping_terminal = niihan_description.teleop_mapping_terminal:main',
            'differential_drive_controller = niihan_description.differential_drive_controller:main',
        ],
    },
)
