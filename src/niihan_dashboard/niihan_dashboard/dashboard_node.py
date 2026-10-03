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
        ws_lock = asyncio.Lock()

        async def safe_send(msg):
            async with ws_lock:
                await websocket.send(msg)

        async def send_telemetry():
            last_map_generation = -1
            last_pc_gen = -1
            last_img_gen = -1
            while True:
                if self.ros_node:
                    try:
                        telemetry_str = self.ros_node.get_telemetry_json()
                        await safe_send(telemetry_str)

                        generation, map_json = self.ros_node.get_map_snapshot()
                        if map_json and generation != last_map_generation:
                            await safe_send(map_json)
                            last_map_generation = generation

                        if self.ros_node.pointcloud_data and self.ros_node.pc_generation != last_pc_gen:
                            pc_str = self.ros_node.get_pointcloud_json()
                            await safe_send(pc_str)
                            last_pc_gen = self.ros_node.pc_generation

                        if self.ros_node.latest_image and self.ros_node.image_generation != last_img_gen:
                            cam_msg = json.dumps({"type": "camera", "image": self.ros_node.latest_image})
                            await safe_send(cam_msg)
                            last_img_gen = self.ros_node.image_generation

                    except websockets.exceptions.ConnectionClosed:
                        break
                await asyncio.sleep(0.1)

        async def receive_commands():
            try:
                async for message in websocket:
                    if self.ros_node:
                        try:
                            cmd = json.loads(message)
                            response = self.ros_node.process_command(cmd)
                            if response:
                                await safe_send(json.dumps(response))
                        except json.JSONDecodeError:
                            self.ros_node.get_logger().error(f"Malformed JSON command received: {message}")
                            await safe_send(json.dumps({"error": "Malformed JSON command"}))
                        except Exception as e:
                            self.ros_node.get_logger().error(f"Error processing command: {e}")
                            await safe_send(json.dumps({"error": str(e)}))
            except websockets.exceptions.ConnectionClosed:
                pass

        # Run tasks and cancel the other if one finishes
        t1 = asyncio.create_task(send_telemetry())
        t2 = asyncio.create_task(receive_commands())
        done, pending = await asyncio.wait([t1, t2], return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()

# Global reference to httpd so we can shut it down
httpd_server = None

def serve_static(directory, port=8080, bind_host="127.0.0.1"):
    global httpd_server
    from functools import partial
    class DashboardStaticHandler(http.server.SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header("Cache-Control", "no-store, max-age=0")
            super().end_headers()

    Handler = partial(DashboardStaticHandler, directory=directory)
    httpd_server = ReusableTCPServer((bind_host, port), Handler)
    with httpd_server:
        print(f"Serving at http://{bind_host}:{port}")
        httpd_server.serve_forever()

async def run_websocket_server(dashboard_server, stop_event, bind_host="127.0.0.1"):
    print(f"Starting WebSocket server on ws://{bind_host}:8081")
    async with websockets.serve(dashboard_server.ws_handler, bind_host, 8081):
        await stop_event.wait()

def main(args=None):
    rclpy.init(args=args)
    ros_node = ROSBridgeNode()
    ros_node.declare_parameter("bind_host", "127.0.0.1")
    bind_host = ros_node.get_parameter("bind_host").value
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    pkg_dir = get_package_share_directory('niihan_dashboard')
    web_dir = os.path.join(pkg_dir, 'web')
    http_thread = threading.Thread(target=serve_static, args=(web_dir, 8080, bind_host), daemon=True)
    http_thread.start()

    dashboard_server = DashboardServer()
    dashboard_server.ros_node = ros_node

    loop = asyncio.get_event_loop()
    stop_event = asyncio.Event()

    def shutdown_handler(signum, frame):
        print("Shutdown signal received...")
        if httpd_server:
            threading.Thread(target=httpd_server.shutdown).start()
        loop.call_soon_threadsafe(stop_event.set)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        loop.run_until_complete(run_websocket_server(dashboard_server, stop_event, bind_host))
    except KeyboardInterrupt:
        pass
    finally:
        if httpd_server:
            httpd_server.server_close()
        ros_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
