"""Visible portable simulation with estimated 3D SLAM, never truth-fed TF."""
import os,json,tempfile,shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def stack(context):
    share=get_package_share_directory('niihan_slam');desc=get_package_share_directory('niihan_description');dash=get_package_share_directory('niihan_dashboard')
    from niihan_slam.map_store import environment_directory,current_map
    output=LaunchConfiguration('output_directory').perform(context);mode=LaunchConfiguration('mode').perform(context)
    environment=LaunchConfiguration('environment_id').perform(context)
    world=LaunchConfiguration('world').perform(context)
    world_file=world if os.path.isabs(world) else os.path.join(desc,'worlds',world)
    directory=environment_directory(LaunchConfiguration('map_directory').perform(context),environment,world_file)
    output=output or str(directory)
    map_file=LaunchConfiguration('map_file').perform(context)
    if mode=='auto':
        map_file=map_file or current_map(directory)
        mode='localization' if map_file else 'mapping'
    print(f'Environment {environment}: {mode}; map={map_file or output}')
    if mode=='localization' and not map_file:
        map_file=current_map(directory)
        if not map_file:raise ValueError('No saved map for this environment; map it first')
    if mode not in ('mapping','localization'):raise ValueError('mode must be mapping or localization')
    cfg=tempfile.mkdtemp(prefix='niihan-glim-');shutil.copytree(os.path.join(share,'config'),cfg,dirs_exist_ok=True)
    if mode=='localization':
        p=os.path.join(cfg,'config_ros.json');d=json.load(open(p));d['glim_ros'].update(enable_local_mapping=False,enable_global_mapping=False,map_frame_id='lio_map');open(p,'w').write(json.dumps(d))
    alignment=LaunchConfiguration('rtk_alignment_file').perform(context)
    from niihan_bringup.rtk_alignment import load_alignment
    from niihan_slam.map_store import atomic_bytes
    saved_alignment=os.path.join(directory,'rtk_alignment.json')
    if alignment:
        load_alignment(alignment,environment)
        atomic_bytes(saved_alignment,open(alignment,'rb').read())
        alignment=saved_alignment
    elif os.path.isfile(saved_alignment):alignment=saved_alignment
    clock={'use_sim_time':True}
    def node(package,executable,name,params=None,**kw):return Node(package=package,executable=executable,name=name,output='screen',parameters=[clock]+(params or []),**kw)
    def include(path,args):return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments=args.items())
    nodes=[include(os.path.join(desc,'launch','niihan_gazebo.launch.py'),{'world':LaunchConfiguration('world'),'spawn_x':LaunchConfiguration('spawn_x'),'spawn_y':LaunchConfiguration('spawn_y'),'headless':LaunchConfiguration('headless'),'seed':LaunchConfiguration('seed'),'external_stack':'true','minimal_sensors':'true','mapping':'false','launch_nav2':'false','vortex_3d':'false','cliff_detect':'false','rqt_cam':'false','patrol':'false'}),
        node('glim_ros','glim_rosnode','glim_ros',[{'config_path':cfg,'dump_on_unload':False}],remappings=[('/tf','/niihan/lio/tf')]),
        node('niihan_slam','tf_filter','lio_tf_filter',[{'mapping':mode=='mapping'}]),
        node('niihan_slam','map_bridge','slam_map_bridge',[{'mapping':mode=='mapping','output_directory':output,'map_file':map_file or '', 'environment_id':environment}]),
        node('niihan_description','command_arbiter','command_arbiter'),
        node('niihan_bringup','gazebo_drive','gazebo_drive',[{'relay_gnss':LaunchConfiguration('gnss_source').perform(context)=='gazebo'}]),
        node('niihan_bringup','rtk_alignment','rtk_alignment',[{'alignment_file':alignment,'environment_id':environment,'allow_simulated_rtk':LaunchConfiguration('allow_simulated_rtk').perform(context)=='true'}]),
        node('niihan_bringup','sensor_adapter','sensor_adapter',[{'gravity_aligned_height':True,'scan_frame':'base_nav','relay_camera':False}]),
        node('niihan_bringup','health_supervisor','health_supervisor',[{'sensor_timeout':2.0}]),
        node('niihan_bringup','motion_gateway','motion_gateway',[{'drive_enabled':True}]),
        include(os.path.join(desc,'launch','niihan_navigation.launch.py'),{'use_sim_time':'true','params_file':os.path.join(share,'config','nav2_3d_jazzy.yaml' if os.environ.get('ROS_DISTRO')=='jazzy' else 'nav2_3d.yaml'),'bt_xml':os.path.join(share,'config','navigate_to_pose_position_jazzy.xml' if os.environ.get('ROS_DISTRO')=='jazzy' else 'navigate_to_pose_position.xml')}),
        node('rviz2','rviz2','rviz2',arguments=['-d',os.path.join(share,'config','slam.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
        node('niihan_dashboard','dashboard_node','niihan_dashboard_node',[{'require_health':True,'slam_cloud_topic':'/niihan/slam/map_cloud','rtk_goal_verification':bool(alignment),'arrival_dwell_seconds':1.0}])]
    # Only the common adapter publishes the navigation scan in this profile.
    if mode=='localization':
        nodes.append(node('niihan_slam','saved_localizer','saved_map_localizer',[{'map_file':map_file,'initial_x':float(LaunchConfiguration('initial_x').perform(context)),'initial_y':float(LaunchConfiguration('initial_y').perform(context)),'initial_yaw':float(LaunchConfiguration('initial_yaw').perform(context))}]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n,default_value=v) for n,v in [('world','niihan_portable_site.sdf'),('headless','false'),('rviz','true'),('seed','42'),('spawn_x','-10.0'),('spawn_y','0.0'),('mode','auto'),('environment_id','portable-site'),('map_directory',os.path.expanduser('~/.local/share/niihan/maps')),('map_file',''),('rtk_alignment_file',''),('gnss_source','gazebo'),('allow_simulated_rtk','false'),('initial_x','0.0'),('initial_y','0.0'),('initial_yaw','0.0'),('output_directory','')]]+[OpaqueFunction(function=stack)])
