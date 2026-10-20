from pathlib import Path
import xml.etree.ElementTree as E, json, random
out=Path(__file__).resolve().parent.parent; worlds=out/'worlds';worlds.mkdir(parents=True,exist_ok=True)
def sub(p,t,v=None,**kw):
 e=E.SubElement(p,t,kw)
 if v is not None:e.text=str(v)
 return e
def box(w,name,x,y,z,sx,sy,sz,color,collision=True):
 m=sub(w,'model',name=name);sub(m,'static','true');sub(m,'pose',f'{x} {y} {z} 0 0 0');l=sub(m,'link',name='body')
 for kind in (['collision','visual'] if collision else ['visual']):
  v=sub(l,kind,name=kind);sub(sub(sub(v,'geometry'),'box'),'size',f'{sx} {sy} {sz}')
  if kind=='visual':
   mat=sub(v,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
 return m
layouts=[]
for k in range(7):
 root=E.Element('sdf',version='1.9');w=sub(root,'world',name='niihan_seaport')
 for fn,cl in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('imu','Imu'),('navsat','NavSat'),('contact','Contact'),('sensors','Sensors')]:
  p=sub(w,'plugin',filename=f'gz-sim-{fn}-system',name=f'gz::sim::systems::{cl}')
  if fn=='sensors':sub(p,'render_engine','ogre2')
 ph=sub(w,'physics',name='physics',type='ignored');sub(ph,'max_step_size','.001');sub(ph,'real_time_factor','1')
 sub(w,'gravity','0 0 -9.81');sc=sub(w,'spherical_coordinates')
 for t,v in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg','13.0827'),('longitude_deg','80.2707'),('elevation','0'),('heading_deg','0')]:sub(sc,t,v)
 sc=sub(w,'scene');sub(sc,'ambient','.65 .65 .65 1');sub(sc,'background','.64 .79 .89 1');sub(sc,'shadows','true')
 light=sub(w,'light',name='sun',type='directional');sub(light,'pose','0 0 30 0 0 0');sub(light,'diffuse','.95 .93 .86 1');sub(light,'direction','-.4 .2 -.9');sub(light,'cast_shadows','true')
 box(w,'quay',0,0,-.25,50,36,.5,'.30 .32 .34 1')
 box(w,'sea',0,37,-.5,100,38,.1,'.04 .28 .42 1',False)
 box(w,'quay_edge',0,17.8,.15,50,.35,.3,'.75 .72 .59 1')
 for x in range(-22,24,4):
  box(w,f'lane_{x}',x,0,.006,2,.12,.012,'.94 .77 .24 1',False)
  box(w,f'edge_{x}',x,16.8,.01,1.8,.3,.02,'.95 .8 .12 1',False)
 for y in [-16,16]:box(w,f'fence_{y}',0,y,.7,48,.15,1.4,'.45 .48 .50 1')
 box(w,'warehouse',-17,-11,3.5,12,7,7,'.58 .61 .63 1')
 box(w,'warehouse_roof',-17,-11,7.1,12.5,7.5,.3,'.24 .29 .34 1')
 for x in [-20,-16,-12]:box(w,f'warehouse_door{x}',x,-7.46,1.6,2,.05,3.2,'.2 .25 .3 1',False)
 # Fixed dock crane and moored ship establish the same port in every layout.
 for x in [13,21]:
  for y in [11,15]:box(w,f'crane_leg{x}_{y}',x,y,6,.5,.5,12,'.83 .61 .12 1')
 box(w,'crane_beam',17,13,12,9,5,.6,'.83 .61 .12 1')
 box(w,'crane_boom',17,20,12.6,.7,22,.7,'.83 .61 .12 1')
 box(w,'ship_hull',8,29,1,29,7,3,'.13 .20 .26 1')
 box(w,'ship_deck',8,29,2.8,27,6,.4,'.59 .58 .52 1')
 box(w,'ship_bridge',-3,29,5,4,5,4,'.83 .85 .84 1')
 rng=random.Random(701+k); cargo=[]
 for i,(x,y) in enumerate([(-5,-10),(3,-10),(11,-10),(19,-10),(-9,8),(-1,8),(7,8)]):
  dx=0 if k==0 else round(rng.uniform(-.8,.8),3);dy=0 if k==0 else round(rng.uniform(-.6,.6),3);x+=dx;y+=dy
  col=['.61 .18 .12 1','.13 .32 .48 1','.19 .40 .30 1','.67 .45 .17 1'][i%4]
  box(w,f'cargo_{i}',x,y,1.3,6.06,2.44,2.6,col);cargo.append({'id':i,'x':x,'y':y,'dx':dx,'dy':dy})
  for rib in range(15):box(w,f'cargo_{i}_rib_{rib}',x-2.9+rib*.41,y-1.235,1.3,.065,.04,2.48,col,False)
  if i%2==0:box(w,f'cargo_{i}_stack',x,y,3.9,6.06,2.44,2.6,col)
 # Actual simulated overview camera, not an illustration.
 m=sub(w,'model',name='overview');sub(m,'static','true');sub(m,'pose','0 -30 36 0 .86 1.57079632679');l=sub(m,'link',name='camera_link');s=sub(l,'sensor',name='overview',type='camera');sub(s,'always_on','true');sub(s,'update_rate','1');sub(s,'topic','/seaport/overview');c=sub(s,'camera');sub(c,'horizontal_fov','1.25');im=sub(c,'image');sub(im,'width','1440');sub(im,'height','1000');sub(im,'format','R8G8B8');cl=sub(c,'clip');sub(cl,'near','.1');sub(cl,'far','200')
 E.indent(root);E.ElementTree(root).write(worlds/f'seaport_{k+1:02}.sdf',encoding='unicode',xml_declaration=True)
 layouts.append({'world':k+1,'seed':42,'cargo':cargo})
(out/'layouts.json').write_text(json.dumps(layouts,indent=2))
if not (out/'protocol.json').exists():
 (out/'protocol.json').write_text(json.dumps({'spawn_world':[-12,0,.15],'datum':[13.0827,80.2707,0],'route_map':[[3,0],[6,0],[9,0]],'position_tolerance_m':.10,'gps_model':'Gazebo NavSat ideal fix; no constellation or RF propagation','layout_count':7,'seed':42},indent=2))
