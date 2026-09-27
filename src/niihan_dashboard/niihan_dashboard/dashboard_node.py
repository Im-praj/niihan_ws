import rclpy
import threading
import asyncio
import websockets
import http.server
import socketserver
import os
import json
import signal
import sys
from ament_index_python.packages import get_package_share_directory
from niihan_dashboard.ros_bridge import ROSBridgeNode

class ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

class DashboardServer:
    def __init__(self):
        self.ros_node = None
        
    async def ws_handler(self, websocket):
        # Background task to send telemetry
        async def send_telemetry():
            last_map_id = None
            while True:
                if self.ros_node:
                    try:
                        telemetry_str = self.ros_node.get_telemetry_json()
                        await websocket.send(telemetry_str)
                        
                        # Send map if updated
                        if self.ros_node.map_data and id(self.ros_node.map_data) != last_map_id:
                            map_json = json.dumps(self.ros_node.map_data)
                            await websocket.send(map_json)
                            last_map_id = id(self.ros_node.map_data)
                        
                        # Send pointcloud
                        if self.ros_node.pointcloud_data:
                            pc_str = self.ros_node.get_pointcloud_json()
                            await websocket.send(pc_str)
                            self.ros_node.pointcloud_data = [] # clear after sending
                        
                        # Send camera if available
                        if self.ros_node.latest_image:
                            cam_msg = json.dumps({"type": "camera", "image": self.ros_node.latest_image})
                            await websocket.send(cam_msg)
                            self.ros_node.latest_image = None
                            
                    except websockets.exceptions.ConnectionClosed:
                        break
                await asyncio.sleep(0.1) # 10Hz
                
        # Background task to receive commands
        async def receive_commands():
            try:
                async for message in websocket:
                    if self.ros_node:
                        try:
                            cmd = json.loads(message)
                            response = self.ros_node.process_command(cmd)
                            if response:
                                await websocket.send(json.dumps(response))
                        except json.JSONDecodeError:
                            pass
            except websockets.exceptions.ConnectionClosed:
                pass
                
        # Run both tasks concurrently
        await asyncio.gather(
            send_telemetry(),
            receive_commands()
        )

# Global reference to httpd so we can shut it down
httpd_server = None

def serve_static(directory, port=8080):
    global httpd_server
    os.chdir(directory)
    Handler = http.server.SimpleHTTPRequestHandler
    httpd_server = ReusableTCPServer(("0.0.0.0", port), Handler)
    with httpd_server:
        print(f"Serving at http://0.0.0.0:{port}")
        httpd_server.serve_forever()

async def run_websocket_server(dashboard_server, stop_event):
    print("Starting WebSocket server on ws://0.0.0.0:8081")
    async with websockets.serve(dashboard_server.ws_handler, "0.0.0.0", 8081):
        await stop_event.wait()

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize ROS Node
    ros_node = ROSBridgeNode()
    
    # Start ROS spinning in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # Start HTTP server in a background thread
    pkg_dir = get_package_share_directory('niihan_dashboard')
    web_dir = os.path.join(pkg_dir, 'web')
    http_thread = threading.Thread(target=serve_static, args=(web_dir, 8080), daemon=True)
    http_thread.start()
    
    # Run WebSocket server in main thread (asyncio)
    dashboard_server = DashboardServer()
    dashboard_server.ros_node = ros_node
    
    loop = asyncio.get_event_loop()
    stop_event = asyncio.Event()

    def shutdown_handler(signum, frame):
        print("Shutdown signal received...")
        if httpd_server:
            # Run in a separate thread to not block the signal handler
            threading.Thread(target=httpd_server.shutdown).start()
        # Schedule the stop event to be set in the asyncio loop
        loop.call_soon_threadsafe(stop_event.set)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    
    try:
        loop.run_until_complete(run_websocket_server(dashboard_server, stop_event))
    except KeyboardInterrupt:
        pass
    finally:
        if httpd_server:
            httpd_server.server_close()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
