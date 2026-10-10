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
                if tele.get('pose_valid') and tele.get('nav2_ready') and tele.get('hardware_health',{}).get('ready'):return True
                await asyncio.sleep(.2)
            return False
        assert await ready(),'startup health unavailable'
        await asyncio.sleep(4)
        await send('clear_estop');await send('set_mode',mode='MANUAL')
        result={}
        await send('set_mode',mode='AUTO')
        log('phase',name='dynamic_obstacle_navigation');await send('nav_goal',x=3.,y=0.,z=tele['pose']['z'])
        await asyncio.sleep(2)
        # World X=-8.5 corresponds to map X~1.5. Insert after goal starts.
        obstacle=root/'obstacle.sdf';obstacle.write_text('<sdf version="1.9"><model name="release_obstacle"><static>true</static><pose>-8.5 0 .5 0 0 0</pose><link name="box"><collision name="collision"><geometry><box><size>.4 .6 1</size></box></geometry></collision><visual name="visual"><geometry><box><size>.4 .6 1</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>')
        inserted=time.time();result['obstacle_inserted']=await service('create','gz.msgs.EntityFactory','sdf_filename: "'+str(obstacle)+'" allow_renaming: false')
        end=time.monotonic()+100
        while time.monotonic()<end:
            if tele.get('navigation',{}).get('state') in ('COMPLETED','FAILED','CANCELLED'):break
            await asyncio.sleep(.2)
        result['obstacle_navigation_state']=tele.get('navigation',{}).get('state')
        samples=[p for p in truth if p[0]>inserted]
        result['obstacle_min_center_distance_m']=min((math.hypot(p[1]+8.5,p[2]) for p in samples),default=None)
        result['obstacle_max_lateral_detour_m']=max((abs(p[2]) for p in samples),default=0)
        result['obstacle_path_detour_observed']=any(any(abs(p[1])>.45 for p in path) for path in paths)
        result['obstacle_pass']=result['obstacle_inserted'] and result['obstacle_navigation_state']=='COMPLETED' and result['obstacle_max_lateral_detour_m']>.45 and result['obstacle_min_center_distance_m']>.45
        await send('cancel_mission');await send('estop');await asyncio.sleep(1)
        result['acceptance_pass']=result['obstacle_pass']
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True);rt.cancel()
    st.cancel();n.destroy_node();rclpy.try_shutdown();f.close()
asyncio.run(run())
