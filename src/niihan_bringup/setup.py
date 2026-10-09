from setuptools import setup, find_packages
from glob import glob
import os

name='niihan_bringup'
data=[('share/ament_index/resource_index/packages',['resource/'+name]),('share/'+name,['package.xml','README.md','COMMISSIONING.md'])]
for directory in ['launch','config','firmware']:
    for path,_,files in os.walk(directory):
        data.append(('share/'+name+'/'+path,[path+'/'+file for file in files]))
setup(name=name,version='0.1.0',packages=find_packages(exclude=['test']),data_files=data,
      install_requires=['setuptools','numpy','pyserial'],maintainer='NIIHAN maintainers',maintainer_email='dev@example.com',
      description='Shared NIIHAN application stack with simulation and physical hardware adapters',license='Apache-2.0',
      entry_points={'console_scripts':[
       'sensor_adapter=niihan_bringup.common_nodes:sensor_adapter_main',
       'health_supervisor=niihan_bringup.common_nodes:health_main',
       'motion_gateway=niihan_bringup.common_nodes:motion_main',
       'gnss_monitor=niihan_bringup.common_nodes:gnss_monitor_main',
       'mock_hardware=niihan_bringup.mock_hardware:main',
       'stm32_bridge=niihan_bringup.hardware_nodes:stm32_main',
       'f9p_driver=niihan_bringup.hardware_nodes:f9p_main',
       'bno085_driver=niihan_bringup.hardware_nodes:bno_main',
       'gazebo_drive=niihan_bringup.gazebo_adapter:main',
      ]})
