import rclpy,time,json,sys,collections,hashlib,subprocess
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2,Imu,LaserScan,Image
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
rclpy.init();n=Node('release_manifest');counts=collections.Counter();headers={};parents=collections.defaultdict(set)
start=time.monotonic()
def sensor(topic,msg):
 counts[topic]+=1;headers[topic]={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9}
def tf(msg):
 for t in msg.transforms:parents[t.child_frame_id].add(t.header.frame_id)
n.create_subscription(TFMessage,'/tf',tf,100)
for topic,typ in [('/niihan/sensors/unitree_lidar/points',PointCloud2),('/niihan/sensors/lidar/points',PointCloud2),('/scan',LaserScan),('/niihan/imu/data',Imu),('/odom',Odometry),('/niihan/ground_truth',Odometry),('/niihan/sensors/panoramic/front/image_raw',Image)]:n.create_subscription(typ,topic,lambda m,t=topic:sensor(t,m),qos_profile_sensor_data)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
node_counts=collections.Counter('/'+ns.strip('/')+'/'+name if ns!='/' else '/'+name for name,ns in n.get_node_names_and_namespaces())
result={'nodes':dict(node_counts),'duplicate_nodes':[k for k,v in node_counts.items() if v>1],'clock_publishers':[{'name':i.node_name,'namespace':i.node_namespace} for i in n.get_publishers_info_by_topic('/clock')],'tf_parents':{k:sorted(v) for k,v in parents.items()},'sensor_headers':headers,'rates_wall_hz':{k:v/(time.monotonic()-start) for k,v in counts.items()},'lifecycle':{}}
for name in ['controller_server','planner_server','bt_navigator','behavior_server','velocity_smoother']:
 client=n.create_client(GetState,'/'+name+'/get_state')
 if not client.wait_for_service(timeout_sec=.3):continue
 future=client.call_async(GetState.Request());rclpy.spin_until_future_complete(n,future,timeout_sec=1.)
 if future.done() and future.result():result['lifecycle'][name]=future.result().current_state.label
Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps(result));n.destroy_node();rclpy.shutdown()
