#!/usr/bin/env python3
import os,sys,time,subprocess,signal,json,urllib.request
from pathlib import Path
workspace=Path(sys.argv[1]);root=Path(sys.argv[2]);root.mkdir(parents=True,exist_ok=False)
env=os.environ.copy();env.update(GZ_PARTITION='niihan_'+root.name,ROS_DOMAIN_ID='61',ROS_HOME=str(root/'ros_home'),ROS_LOG_DIR=str(root/'ros'))
(root/'source.txt').write_text(subprocess.check_output(['git','-C',str(workspace),'rev-parse','HEAD'],text=True)+subprocess.check_output(['git','-C',str(workspace),'submodule','status'],text=True)+subprocess.check_output(['git','-C',str(workspace),'status','--short'],text=True))
log=open(root/'launch.txt','w');record=open(root/'recorder.txt','w')
launch=subprocess.Popen(['ros2','launch','niihan_slam','slam_simulation.launch.py','headless:=false','seed:=42','output_directory:='+str(root/'map')]+sys.argv[3:],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
video=subprocess.Popen(['gst-launch-1.0','-e','ximagesrc','display-name='+env.get('DISPLAY',':1'),'use-damage=false','show-pointer=true','do-timestamp=true','!','video/x-raw,framerate=15/1','!','videoconvert','!','vp8enc','deadline=1','cpu-used=8','threads=2','target-bitrate=3000000','!','webmmux','!','filesink','location='+str(root/'desktop.webm')],env=env,stdout=record,stderr=subprocess.STDOUT,start_new_session=True)
exit_code=1
try:
 end=time.monotonic()+45
 while time.monotonic()<end:
  if launch.poll() is not None:raise RuntimeError('launch exited before dashboard became ready')
  try:
   urllib.request.urlopen('http://127.0.0.1:8080',timeout=1).close();break
  except OSError:time.sleep(.5)
 with open(root/'acceptance_stdout.txt','w') as out:
  test=subprocess.run(['python3',str(Path(__file__).parent/'acceptance_probe.py'),str(root)],env=env,stdout=out,stderr=subprocess.STDOUT,timeout=420)
 with open(root/'manifest_stdout.txt','w') as manifest_log:
  subprocess.run(['python3',str(Path(__file__).parent/'runtime_manifest.py'),str(root/'runtime_manifest.json')],env=env,stdout=manifest_log,stderr=subprocess.STDOUT,timeout=30)
 print('probe exit',test.returncode,flush=True)
 if (root/'result.json').exists():
  result=json.loads((root/'result.json').read_text());print(json.dumps(result,indent=2),flush=True)
  exit_code=0 if test.returncode==0 and result.get('acceptance_pass') else 1
 (root/'run_status.json').write_text(json.dumps({'probe_exit':test.returncode,'acceptance_pass':exit_code==0}))
finally:
 for p in [launch,video]:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in [launch,video]:
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
 # Gazebo can orphan its server. Terminate only this run's transport partition.
 for d in Path('/proc').iterdir():
  if not d.name.isdigit():continue
  try:
   command=(d/'cmdline').read_bytes()
   if b'gz sim' not in command:continue
   values=(d/'environ').read_bytes().split(b'\0')
   if ('GZ_PARTITION='+env['GZ_PARTITION']).encode() in values:os.kill(int(d.name),signal.SIGTERM)
  except (OSError,ProcessLookupError):pass
 log.close();record.close()

sys.exit(exit_code)
