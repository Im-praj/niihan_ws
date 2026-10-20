#!/usr/bin/env python3
"""Visible RTK control test using an explicitly labelled emulated F9P."""
import os,pty,sys,subprocess,signal,json
from pathlib import Path
os.umask(0o077)
repo=Path(__file__).resolve().parents[2];root=Path(sys.argv[1]).resolve();root.mkdir(parents=True,exist_ok=True)
alignment=Path(sys.argv[2]).resolve();environment=json.loads(alignment.read_text())['environment_id'];mode=sys.argv[3] if len(sys.argv)>3 else 'auto'
master,slave=pty.openpty();port=os.ttyname(slave)
env=os.environ.copy();env.update(ROS_DOMAIN_ID='61',ROS_LOG_DIR=str(root/'device-logs'))
log=(root/'fixture.log').open('w');procs=[]
try:
    procs.append(subprocess.Popen(['python3',str(repo/'scripts/testing/f9p_fixture.py'),str(master)],pass_fds=(master,),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    procs.append(subprocess.Popen(['ros2','run','niihan_bringup','f9p_driver','--ros-args','-p','use_sim_time:=true','-p','simulation_fixture:=true','-p','port:='+port],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    (root/'fixture.json').write_text(json.dumps({'source':'emulated F9P, Gazebo NavSat position','real_receiver':False,'injected_satellites_used':24,'injected_hdop':.65,'injected_vdop':1.10,'alignment':str(alignment)},indent=2))
    evidence=root/'trial'
    code=subprocess.call(['python3',str(repo/'scripts/record_acceptance.py'),str(repo),str(evidence),'mode:='+mode,'environment_id:='+environment,'map_directory:='+str(root.parent/'maps'),'output_directory:='+str(root.parent/'maps'/environment),'gnss_source:=external','allow_simulated_rtk:=true',*([] if '--reuse-alignment' in sys.argv else ['rtk_alignment_file:='+str(alignment)])],env=env)
finally:
    for p in procs:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in procs:
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
    os.close(master);os.close(slave);log.close()
sys.exit(code)
