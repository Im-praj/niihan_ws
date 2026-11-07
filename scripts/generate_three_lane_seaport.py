#!/usr/bin/env python3
"""Generate a self-contained three-lane seaport with 24 uniquely tagged cargos."""
from pathlib import Path
import json, xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
WORLDS=ROOT/'src/niihan_description/worlds'

def sub(p,t,text=None,**attrs):
 e=E.SubElement(p,t,attrs)
 if text is not None:e.text=str(text)
 return e

def visual(link,name,x,y,z,sx,sy,sz,color):
 v=sub(link,'visual',name=name);sub(v,'pose',f'{x} {y} {z} 0 0 0')
 sub(sub(sub(v,'geometry'),'box'),'size',f'{sx} {sy} {sz}')
 mat=sub(v,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
 return v

def box(w,name,x,y,z,sx,sy,sz,color,collision=False):
 m=sub(w,'model',name=name);sub(m,'static','true');sub(m,'pose',f'{x} {y} {z} 0 0 0');l=sub(m,'link',name='body')
 visual(l,'surface',0,0,0,sx,sy,sz,color)
 if collision:sub(sub(sub(sub(l,'collision',name='solid'),'geometry'),'box'),'size',f'{sx} {sy} {sz}')
 return l

SEGMENTS={'0':'abcdef','1':'bc','2':'abged','3':'abgcd','4':'fgbc','5':'afgcd','6':'afgecd','7':'abc','8':'abcdefg','9':'abfgcd'}
# Seven-segment geometry is portable across Gazebo renderers: no font/plugin assets.
BARS={'a':(0,.48,.34,.07),'g':(0,0,.34,.07),'d':(0,-.48,.34,.07),'f':(-.20,.25,.07,.40),'b':(.20,.25,.07,.40),'e':(-.20,-.25,.07,.40),'c':(.20,-.25,.07,.40)}
def tag(l,number):
 text=f'{number:03d}'
 for side in [-1,1]:
  y=side*1.265
  visual(l,f'tag_plate_{side}',0,y,.12,2.1,.035,1.35,'.035 .045 .055 1')
  for i,d in enumerate(text):
   for seg in SEGMENTS[d]:
    x,z,sx,sz=BARS[seg]
    visual(l,f'tag_{side}_{i}_{seg}',(i-1)*.59+x,y+side*.026,.12+z,sx,.018,sz,'.98 .98 .92 1')
 # Roof tag allows identification from the overview camera too.
 visual(l,'roof_plate',0,0,1.322,2.1,1.35,.025,'.035 .045 .055 1')
 for i,d in enumerate(text):
  for seg in SEGMENTS[d]:
   x,y,sx,sy=BARS[seg]
   visual(l,f'roof_{i}_{seg}',(i-1)*.59+x,y,1.343,sx,sy,.018,'.98 .98 .92 1')

def generate():
 tree=E.parse(WORLDS/'seaport_01.sdf');w=tree.getroot().find('world');w.set('name','niihan_seaport_three_lane')
 for m in list(w.findall('model')):
  name=m.get('name','')
  if name.startswith(('cargo_','lane_','warehouse','fence_')):w.remove(m)
  elif name=='quay':m.find('.//collision/geometry/box/size').text='76 44 .5';m.find('.//visual/geometry/box/size').text='76 44 .5'
  elif name.startswith('crane_'):
   pose=m.find('pose');a=pose.text.split();a[0]=str(float(a[0])+13);pose.text=' '.join(a)
  elif name=='quay_edge':m.find('pose').text='0 21.8 .15 0 0 0';[setattr(v,'text','76 .35 .3') for v in m.findall('.//box/size')]
  elif name.startswith('edge_'):w.remove(m)
 # Three parallel 4.4 m lanes between four cargo rows; turning space at both ends.
 for lane,y in enumerate([-10,0,10],1):
  for x in range(-24,26,4):box(w,f'lane_{lane}_dash_{x}',x,y,.012,1.9,.13,.016,'.96 .78 .18 1')
  for dy in [-2.2,2.2]:box(w,f'lane_{lane}_edge_{dy}',0,y+dy,.009,52,.09,.012,'.90 .91 .87 1')
 for y in [-21.5,21.5]:box(w,f'fence_{y}',0,y,.65,75,.10,1.3,'.40 .46 .49 1',True)
 box(w,'warehouse',-31,0,4,10,17,8,'.57 .62 .66 1',True)
 box(w,'warehouse_roof',-31,0,8.15,10.6,17.6,.3,'.19 .26 .31 1',True)
 for y in [-5,0,5]:box(w,f'warehouse_door_{y}',-25.98,y,2,.035,3.1,4,'.25 .31 .36 1')
 colors=[('red','.66 .16 .12 1'),('blue','.08 .30 .61 1'),('green','.12 .43 .26 1'),('yellow','.91 .65 .10 1'),('orange','.91 .34 .08 1')]
 manifest=[]
 for row,y in enumerate([-15,-5,5,15]):
  for col,x in enumerate([-20,-12,-4,4,12,20]):
   number=row*6+col+1;color,rgba=colors[(number-1)%5]
   l=box(w,f'cargo_{number:03d}_{color}',x,y,1.32,6.06,2.44,2.6,rgba,True)
   for rib in range(16):
    for side in [-1,1]:visual(l,f'rib_{side}_{rib}',-2.9+rib*.387,side*1.238,0,.055,.035,2.48,rgba)
   for end in [-1,1]:
    for yy in [-.82,.82]:visual(l,f'door_bar_{end}_{yy}',end*3.045,yy,0,.035,.045,2.25,'.66 .69 .70 1')
   tag(l,number);manifest.append(dict(id=f'{number:03d}',color=color,x=x,y=y,size=[6.06,2.44,2.6]))
 # Dock details, rubber fenders, mooring bollards and floodlight poles.
 for x in range(-24,37,6):
  box(w,f'fender_{x}',x,22.04,.3,1.0,.30,.8,'.055 .065 .07 1')
  box(w,f'bollard_base_{x}',x,20.8,.12,.65,.65,.24,'.10 .12 .14 1',True)
  box(w,f'bollard_{x}',x,20.8,.43,.18,.18,.5,'.85 .64 .12 1',True)
 for x in [-27,27]:
  for y in [-19,19]:
   box(w,f'light_pole_{x}_{y}',x,y,5,.15,.15,10,'.47 .51 .54 1',True)
   box(w,f'light_head_{x}_{y}',x,y,10,1.1,.5,.2,'.92 .94 .91 1')
 # Same simulated datum and camera topic; wider overview covers the complete yard.
 overview=w.find("model[@name='overview']");overview.find('pose').text='0 -45 55 0 .87 1.57079632679'
 overview.find('.//horizontal_fov').text='1.35';overview.find('.//image/width').text='1800';overview.find('.//image/height').text='1200'
 E.indent(tree);tree.write(WORLDS/'seaport_three_lane.sdf',encoding='unicode',xml_declaration=True)
 (WORLDS/'seaport_three_lane_layout.json').write_text(json.dumps({'lane_centres_y':[-10,0,10],'lane_width_m':4.4,'cargo_count':24,'colors':[c[0] for c in colors],'cargo':manifest},indent=2)+'\n')
 print('Generated three lanes and 24 cargos tagged 001 through 024 in five colors')
if __name__=='__main__':generate()
