from pathlib import Path
import json,math,csv,statistics,hashlib,sys
import numpy as np
base=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parent.parent;layouts=json.loads((base/'layouts.json').read_text());rows=[];waypoints=[]
for i in range(1,8):
 root=base/'results'/f'world_{i:02}';p=root/'result.json'
 if not p.exists():continue
 d=json.loads(p.read_text());saved_points=0
 if d.get('slam_3d',{}).get('map_save',{}).get('success'):
  checkpoint=Path(d['slam_3d']['map_save']['message'])
  if not checkpoint.exists():checkpoint=root/'maps'/'checkpoints'/checkpoint.name
  with np.load(checkpoint) as saved:saved_points=len(saved['points'])
 w=d.get('waypoints',[]);trace=json.loads((root/'trace.json').read_text());layout=layouts[i-1]
 obstacles=[(-17,-11,12,7)]+[(c['x'],c['y'],6.06,2.44) for c in layout['cargo']]
 clearance=min((math.hypot(max(abs(t['truth'][0]-x)-sx/2,0),max(abs(t['truth'][1]-y)-sy/2,0))-.45 for t in trace for x,y,sx,sy in obstacles),default=None)
 row={'world':i,'pass':d.get('pass',False),'waypoints_reached':sum(t.get('pass',False) for t in w),'baseline_mean_m':statistics.mean([t['baseline_truth_error_m'] for t in w if 'baseline_truth_error_m' in t]) if w else None,'baseline_max_m':max((t.get('baseline_truth_error_m',0) for t in w),default=None),'final_mean_m':statistics.mean([t['final_truth_error_m'] for t in w if 'final_truth_error_m' in t]) if w and all('final_truth_error_m' in t for t in w) else None,'final_max_m':max((t.get('final_truth_error_m',0) for t in w),default=None),'elapsed_wall_s':d.get('elapsed_wall_s'),'min_sampled_obstacle_clearance_m':clearance,'max_cross_track_m':max((abs(t['truth'][1]) for t in trace),default=None),'saved_map_points':saved_points,'cloud_points':d.get('slam_3d',{}).get('last_cloud_points',0),'map_saved':d.get('slam_3d',{}).get('map_save',{}).get('success',False),'trace_samples':len(trace),'gps_approach_min_scan_m':min((t.get('gps_approach',{}).get('min_scan_range_m',math.inf) for t in w),default=None)}
 rows.append(row)
 for t in w:waypoints.append({'world':i,**t})
 for image in ['world.png','front.png']:
  if (root/image).exists():pass
summary={'worlds_completed':len(rows),'worlds_passed':sum(r['pass'] for r in rows),'waypoints_passed':sum(r['waypoints_reached'] for r in rows),'maps_3d_saved':sum(r['map_saved'] for r in rows),'rows':rows,'waypoints':waypoints,'criteria_m':.10,'evaluation':'Ground truth only used for scoring; GPS is ideal; fresh online map per world; identical procedure; fixed seed 42'}
final=[t['final_truth_error_m'] for t in waypoints if 'final_truth_error_m' in t];baseline=[t['baseline_truth_error_m'] for t in waypoints if 'baseline_truth_error_m' in t]
if final:summary.update(mean_final_error_m=statistics.mean(final),max_final_error_m=max(final),mean_baseline_error_m=statistics.mean(baseline),mean_error_reduction_percent=100*(1-statistics.mean(final)/statistics.mean(baseline)))
(base/'summary.json').write_text(json.dumps(summary,indent=2))
with (base/'results.csv').open('w') as f:
 if rows:
  c=csv.DictWriter(f,fieldnames=rows[0].keys());c.writeheader();c.writerows(rows)
print(json.dumps({k:v for k,v in summary.items() if k not in ('rows','waypoints')},indent=2))
