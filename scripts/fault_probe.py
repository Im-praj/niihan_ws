"""Visible-simulation fault and dynamic-obstacle evidence, with real drive scoring."""
import asyncio,json,time,sys,os,signal,subprocess,math
from pathlib import Path
import rclpy,websockets
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String

async def run():
    root=Path(sys.argv[1]); rows=[]; tele={}; responses=[]; truth=[]; paths=[]
    f=open(root/'faults.jsonl','w')
    def log(kind,**data):
        row={'time':time.time(),'kind':kind,**data};rows.append(row);f.write(json.dumps(row)+'\n');f.flush()
    rclpy.init(); n=Node('release_fault_probe')
    n.create_subscription(Twist,'/niihan/drive/cmd_vel',lambda m:log('drive',v=m.linear.x,w=m.angular.z),10)
    n.create_subscription(String,'/niihan/health',lambda m:log('health',data=json.loads(m.data)),10)
    def ground(m):
        p=m.pose.pose.position;truth.append((time.time(),p.x,p.y,p.z));log('truth',x=p.x,y=p.y,z=p.z)
    def path(m):
        points=[[p.pose.position.x,p.pose.position.y] for p in m.poses];paths.append(points);log('path',points=points)
    n.create_subscription(Odometry,'/niihan/ground_truth',ground,qos_profile_sensor_data)
    n.create_subscription(NavPath,'/plan',path,10)
    async def spin():
        while rclpy.ok():
            for _ in range(20):rclpy.spin_once(n,timeout_sec=0)
            await asyncio.sleep(.01)
    st=asyncio.create_task(spin())
    def zero_after(start,delay,end):
        samples=[r for r in rows if r['kind']=='drive' and start+delay<r['time']<end]
        return bool(samples) and all(abs(r['v'])<1e-9 and abs(r['w'])<1e-9 for r in samples)
    def moved(start,end):return any(r['kind']=='drive' and start<r['time']<end and abs(r['v'])>.04 for r in rows)
    async def service(name,typ,request):
        p=await asyncio.create_subprocess_exec('gz','service','-s','/world/niihan_construction_site/'+name,'--reqtype',typ,'--reptype','gz.msgs.Boolean','--timeout','3000','--req',request,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.STDOUT)
        out=await p.communicate();log('service',name=name,request=request,output=out[0].decode(),exit=p.returncode)
        return p.returncode==0 and 'true' in out[0].decode()
    async with websockets.connect('ws://127.0.0.1:8081',max_size=30000000) as ws:
        async def read():
            nonlocal tele
            async for raw in ws:
                d=json.loads(raw)
                if d.get('type')=='telemetry':tele=d;log('telemetry',data={k:v for k,v in d.items() if k!='path_history'})
                if d.get('type')=='command_response':responses.append(d);log('response',data=d)
        rt=asyncio.create_task(read())
        async def send(action,**kw):
            log('sent',action=action,**kw);await ws.send(json.dumps({'action':action,**kw}));await asyncio.sleep(.2)
        async def ready():
            end=time.monotonic()+60
            while time.monotonic()<end:
                if tele.get('pose_valid') and tele.get('hardware_health',{}).get('ready'):return True
                await asyncio.sleep(.2)
            return False
        assert await ready(),'startup health unavailable'
        await send('clear_estop');await send('set_mode',mode='MANUAL')
        start=time.time();log('phase',name='bounded_manual_and_deadman')
        for _ in range(10):await send('joystick',linear_x=.10,angular_z=0)
        ceased=time.time();await asyncio.sleep(2)
        result={'manual_movement_observed':moved(start,ceased),'manual_deadman_zero':zero_after(ceased,.8,time.time())}
        # Suspend only this simulation's LiDAR/drive adapter, never unrelated nodes.
        partition=os.environ['GZ_PARTITION'];pids=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                args=(entry/'cmdline').read_bytes();env=(entry/'environ').read_bytes().split(b'\0')
                if b'/gazebo_drive' in args and ('GZ_PARTITION='+partition).encode() in env:pids.append(int(entry.name))
            except OSError:pass
        assert len(pids)==1,('adapter PID count',pids)
        log('phase',name='sensor_adapter_loss',pid=pids[0]);start=time.time();os.kill(pids[0],signal.SIGSTOP)
        try:
            for _ in range(25):await send('joystick',linear_x=.1,angular_z=0)
            result['sensor_loss_health_fault']=not tele.get('hardware_health',{}).get('ready',True)
            result['sensor_loss_drive_zero']=zero_after(start,2.8,time.time())
            result['sensor_loss_command_rejected']=any(r.get('success') is False for r in responses[-10:])
        finally:os.kill(pids[0],signal.SIGCONT)
        await ready();await send('clear_estop')
        log('phase',name='paused_simulation_clock');start=time.time()
        result['clock_pause_service']=await service('control','gz.msgs.WorldControl','pause: true')
        try:
            for _ in range(25):await send('joystick',linear_x=.1,angular_z=0)
            result['clock_loss_drive_zero']=zero_after(start,3.6,time.time())
            result['clock_loss_health_fault']=not tele.get('hardware_health',{}).get('ready',True)
        finally:await service('control','gz.msgs.WorldControl','pause: false')
        await ready();await send('clear_estop');await send('set_mode',mode='AUTO')
        await send('estop');await asyncio.sleep(1)
        result['acceptance_pass']=all(result.values())
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True);rt.cancel()
    st.cancel();n.destroy_node();rclpy.try_shutdown();f.close()
asyncio.run(run())
