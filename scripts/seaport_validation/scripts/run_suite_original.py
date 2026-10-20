import subprocess,os,signal,time,json,sys,psutil
from pathlib import Path
root=Path(__file__).resolve().parents[2];os.chdir(root);base=root/'outputs/seaport';work=root/'work/seaport';env=os.environ.copy();env.update(ROS_DOMAIN_ID='73',GZ_PARTITION='niihan_seaport_73',ROS_HOME=str(work/'ros_home'))
def stop(p):
 try:children=psutil.Process(p.pid).children(recursive=True)
 except psutil.Error:children=[]
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for child in children:
  try:
   if child.is_running():child.terminate()
  except psutil.Error:pass
 _,alive=psutil.wait_procs(children,timeout=3)
 for child in alive:
  try:child.kill()
  except psutil.Error:pass
for k in range(int(sys.argv[1]) if len(sys.argv)>1 else 1,int(sys.argv[2])+1 if len(sys.argv)>2 else 8):
 dest=base/'results'/f'world_{k:02}';dest.mkdir(parents=True,exist_ok=True);env['ROS_LOG_DIR']=str(work/f'logs_{k:02}');env['ROS_DOMAIN_ID']=str(80+k);env['GZ_PARTITION']=f'niihan_seaport_{os.getpid()}_{k}';env['ROS_HOME']=str(dest/'ros_home')
 processes=[];logs=[]
 try:
  commands=[['ros2','launch',str(work/'seaport_3d.launch.py'),f'world:={base}/worlds/seaport_{k:02}.sdf','headless:=true','rviz:=false','mode:=mapping','spawn_x:=-12.0','gnss_source:=external',f'environment_id:=seaport_{k:02}',f'map_directory:={dest}/maps',f'output_directory:={dest}/maps'],['ros2','run','ros_gz_bridge','parameter_bridge','/seaport/overview@sensor_msgs/msg/Image[gz.msgs.Image'],['python3',str(work/'dashboard_launch.py'),'--ros-args','-p','use_sim_time:=true','-p','slam_cloud_topic:=/niihan/slam/map_cloud','-p','require_health:=true','-r','/niihan/cmd_vel/manual:=/seaport/observer/manual','-r','/niihan/e_stop:=/seaport/observer/estop']]
  for i,c in enumerate(commands):
   f=(dest/f'process_{i}.log').open('w');logs.append(f);processes.append(subprocess.Popen(c,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
  f=(dest/'probe.log').open('w');logs.append(f)
  p=subprocess.Popen(['python3',str(work/'probe.py'),str(dest),str(k)],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
  try:p.wait(timeout=650)
  except subprocess.TimeoutExpired:stop(p);(dest/'result.json').write_text(json.dumps({'world':k,'pass':False,'error':'suite timeout'}))
  print('FINISHED',k,(dest/'result.json').read_text() if (dest/'result.json').exists() else 'no result',flush=True)
 finally:
  for p in reversed(processes):stop(p)
  for f in logs:f.close()
 time.sleep(3)
