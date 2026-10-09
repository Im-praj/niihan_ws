#!/usr/bin/env python3
"""Live simulation acceptance; truth is only subscribed for scoring, never published."""
import asyncio,json,time,sys,math
from pathlib import Path
import websockets,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
async def run():
    root=Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True);f=open(root/'acceptance.jsonl','w');rows=[];tele={}
    def log(d):d={'wall_time':time.time(),**d};f.write(json.dumps(d)+'\n');f.flush();rows.append(d)
    rclpy.init();n=Node('acceptance_probe');tf=Buffer(node=n);listener=TransformListener(tf,n)
    def truth(m):
        try:
            tr=tf.lookup_transform('map','base_footprint',rclpy.time.Time());t=tr.transform;p=m.pose.pose.position
            log({'kind':'pose_pair','stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'estimate_stamp':tr.header.stamp.sec+tr.header.stamp.nanosec/1e9-.1,'truth_yaw':math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2)),'estimate_yaw':math.atan2(2*(t.rotation.w*t.rotation.z+t.rotation.x*t.rotation.y),1-2*(t.rotation.y**2+t.rotation.z**2)),'truth':[p.x,p.y,p.z],'estimate':[t.translation.x,t.translation.y,t.translation.z]})
        except TransformException:pass
    n.create_subscription(Odometry,'/niihan/ground_truth',truth,qos_profile_sensor_data)
    n.create_subscription(Twist,'/niihan/drive/cmd_vel',lambda m:log({'kind':'drive','v':m.linear.x,'w':m.angular.z}),10)
    n.create_subscription(String,'/niihan/slam/status',lambda m:log({'kind':'slam','data':json.loads(m.data)}),10)
    async def spin():
        while rclpy.ok():rclpy.spin_once(n,timeout_sec=0.);await asyncio.sleep(.01)
    st=asyncio.create_task(spin())
    async with websockets.connect('ws://127.0.0.1:8081',max_size=30_000_000) as w:
        async def read():
            nonlocal tele
            async for raw in w:
                d=json.loads(raw)
                if d.get('type')=='telemetry':tele=d;log({'kind':'telemetry','data':{k:v for k,v in d.items() if k!='path_history'}})
                elif d.get('type') in ('command_response','mission_write_response'):log({'kind':'response','data':d})
                else:log({'kind':'payload','type':d.get('type'),'bytes':len(raw)})
        rt=asyncio.create_task(read())
        async def send(action,**kw):d={'action':action,**kw};log({'kind':'sent','data':d});await w.send(json.dumps(d));await asyncio.sleep(.4)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if tele.get('pose_valid') and tele.get('hardware_health',{}).get('ready'):break
            await asyncio.sleep(.2)
        log({'kind':'phase','name':'three_waypoint_route'})
        await send('clear_estop');await send('set_mode',mode='AUTO');await send('clear_mission')
        target_z=tele['pose']['z']
        for x,y in [(1.,0.),(1.,1.),(.1,.1)]:await send('add_waypoint',x=x,y=y,z=target_z)
        await send('write_mission');route_started=time.monotonic();await send('start_mission')
        deadline=time.monotonic()+180;completed=False
        while time.monotonic()<deadline:
            state=tele.get('mission',{}).get('state')
            if state=='COMPLETED':completed=True;break
            if state in ('FAILED','CANCELLED'):break
            await asyncio.sleep(.2)
        result={'mission_completed':completed,'terminal':tele.get('mission'),'pose':tele.get('pose'),'criteria':{'final_xyz_error_m':.25,'max_relative_position_error_m':.20,'run_count':3},'route_elapsed_wall_s':0.}
        result['route_elapsed_wall_s']=time.monotonic()-route_started
        pose=tele.get('pose',{});result['final_xy_error_m']=math.hypot(pose.get('x',math.inf)-.1,pose.get('y',math.inf)-.1);result['final_xyz_error_m']=math.dist([pose.get('x',math.inf),pose.get('y',math.inf),pose.get('z',math.inf)],[.1,.1,target_z])
        if not completed:await send('cancel_mission')
        await asyncio.sleep(2)
        # Low-speed manual pulse and asserted e-stop with continuing requests.
        log({'kind':'phase','name':'safety_override'})
        await send('set_mode',mode='MANUAL')
        for _ in range(5):await send('joystick',linear_x=.1,angular_z=0.)
        estop_wall=time.time();await send('estop')
        for _ in range(5):await send('joystick',linear_x=.1,angular_z=0.)
        await asyncio.sleep(2);await send('clear_estop');await asyncio.sleep(2)
        client=n.create_client(Trigger,'/niihan/slam/save_map')
        if client.service_is_ready():
            future=client.call_async(Trigger.Request());deadline=time.monotonic()+20
            while not future.done() and time.monotonic()<deadline:await asyncio.sleep(.1)
            if future.done():result['map_save']={'success':future.result().success,'message':future.result().message}
        pairs=[r for r in rows if r.get('kind')=='pose_pair']
        if pairs:
            import numpy as np
            times=np.array([r['stamp'] for r in pairs]);et=np.array([r['estimate_stamp'] for r in pairs]);truth=np.array([r['truth'][:2] for r in pairs]);valid=(et>=times[0])&(et<=times[-1]);a=np.column_stack([np.interp(et[valid],times,truth[:,i]) for i in range(2)]);b=np.array([r['estimate'][:2] for r in pairs])[valid];error=np.linalg.norm((b-b[0])-(a-a[0]),axis=1)
            result['relative_position_rmse_m']=float(np.sqrt(np.mean(error**2)));result['relative_position_max_m']=float(error.max());result['samples']=len(error);result['evaluation']='time-aligned relative XY displacement; initial translation removed; no trajectory rotation fitted'
            ty=np.unwrap([r['truth_yaw'] for r in pairs]);ay=np.interp(et[valid],times,ty);by=np.unwrap([r['estimate_yaw'] for r in pairs])[valid]
            vt=et[valid];later=np.searchsorted(vt,vt+1.0);indices=np.flatnonzero(later<len(vt));later=later[indices]
            if len(indices):
                da=a[later]-a[indices];db=b[later]-b[indices]
                def local(delta,yaw):return np.column_stack([delta[:,0]*np.cos(yaw)+delta[:,1]*np.sin(yaw),-delta[:,0]*np.sin(yaw)+delta[:,1]*np.cos(yaw)])
                re=np.linalg.norm(local(db,by[indices])-local(da,ay[indices]),axis=1)
                he=np.arctan2(np.sin((by[later]-by[indices])-(ay[later]-ay[indices])),np.cos((by[later]-by[indices])-(ay[later]-ay[indices])))
                result['rpe_1s_translation_rmse_m']=float(np.sqrt(np.mean(re**2)));result['rpe_1s_heading_rmse_rad']=float(np.sqrt(np.mean(he**2)))
        stops=[r for r in rows if r.get('kind')=='drive' and estop_wall+.5<r['wall_time']<estop_wall+3.5]
        result['estop_drive_zero']=bool(stops) and all(abs(r['v'])<1e-9 and abs(r['w'])<1e-9 for r in stops)
        result['acceptance_pass']=completed and result['final_xyz_error_m']<=.25 and result.get('relative_position_max_m',math.inf)<=.20 and result['estop_drive_zero']
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        rt.cancel()
    st.cancel();n.destroy_node();rclpy.try_shutdown();f.close()
asyncio.run(run())
