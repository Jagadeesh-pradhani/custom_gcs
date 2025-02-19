#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Import Float32 for battery percentage
from dronekit import connect, VehicleMode, LocationGlobalRelative
import websockets
from custom_gcs.mission import AutonomousMission
import asyncio
import json
import threading
import time

class DronekitBridge(Node):
    def __init__(self):
        # Initialize the ROS2 node first
        super().__init__('dronekit_bridge')
        
        # Initialize protected attributes
        self._websocket_clients = set()
        self._loop = None
        self._server = None
        self._vehicle = None
        self.mission = False
        self._battery_percent = 0.0  # Holds latest battery percentage from /percent topic
        self._battery_warning_sent = False  # Flag to prevent repeated warnings
        self._current_state = {
            'connected': False,
            'armed': False,
            'mode': 'UNKNOWN',
            'battery': 0.0,  # This will be updated from /percent topic
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'heading': 0.0,
            'groundspeed': 0.0,
            'airspeed': 0.0,
            'attitude': {
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0
            }
        }
        
        # Create an AutonomousMission instance and pass this node as parent
        self.autoMission = AutonomousMission(self)

        # Log initialization
        self.send_log("DronekitBridge initialized")

        # Connect to vehicle
        self.get_logger().info("Connecting to vehicle...")
        self.send_log("Connecting to vehicle...")
        try:
            self._vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
            self.get_logger().info("Vehicle connected!")
            self.send_log("Vehicle connected!")
        except Exception as e:
            error_msg = f"Failed to connect to vehicle: {str(e)}"
            self.get_logger().error(error_msg)
            self.send_log(error_msg, level="error")
        
        # Subscribe to /percent topic for battery updates
        self.create_subscription(Float32, '/percent', self.percent_callback, 10)
        
        # Create timer for vehicle state updates
        self.create_timer(0.1, self.update_vehicle_state)
        
        # Start WebSocket server
        self._start_websocket_server()

    def percent_callback(self, msg: Float32):
        """Callback to update battery percentage from the /percent topic."""
        self._battery_percent = msg.data
        self._current_state["battery"] = msg.data
        if msg.data < 10 and not self._battery_warning_sent:
            self.send_log("Warning: Battery below 10%!", level="warning")
            self._battery_warning_sent = True
        elif msg.data >= 10:
            self._battery_warning_sent = False

    def update_vehicle_state(self):
        if not self._vehicle:
            return
            
        try:
            self._current_state.update({
                'connected': True,
                'armed': self._vehicle.armed,
                'mode': self._vehicle.mode.name,
                # Use the battery percentage from the /percent topic instead of vehicle's own value
                'battery': self._battery_percent if self._battery_percent > 10.0 else 10.0,
                'lat': self._vehicle.location.global_relative_frame.lat,
                'lon': self._vehicle.location.global_relative_frame.lon,
                'alt': self._vehicle.location.global_relative_frame.alt,
                'heading': self._vehicle.heading,
                'groundspeed': self._vehicle.groundspeed,
                'airspeed': self._vehicle.airspeed,
                'attitude': {
                    'roll': self._vehicle.attitude.roll,
                    'pitch': self._vehicle.attitude.pitch,
                    'yaw': self._vehicle.attitude.yaw
                }
            })
            if self._loop and self._loop.is_running():
                asyncio.run_coroutine_threadsafe(self._broadcast_state(), self._loop)
        except Exception as e:
            error_msg = f"Error updating state: {str(e)}"
            self.get_logger().error(error_msg)
            self.send_log(error_msg, level="error")

    async def _websocket_handler(self, websocket):
        """Handle WebSocket connections, receiving commands and sending state updates."""
        self._websocket_clients.add(websocket)
        try:
            async for message in websocket:
                await self._handle_command(websocket, message)
        except Exception as e:
            err_msg = f"WebSocket error: {str(e)}"
            self.get_logger().error(err_msg)
            self.send_log(err_msg, level="error")
        finally:
            self._websocket_clients.remove(websocket)
        
    async def _handle_command(self, websocket, message):
        if not self._vehicle:
            await websocket.send(json.dumps({
                'status': 'error',
                'message': 'Vehicle not connected'
            }))
            return

        try:
            cmd = json.loads(message)
            response = {'status': 'success', 'message': ''}
            
            if cmd['type'] == 'arm':
                self._vehicle.armed = cmd['value']
                response['message'] = f"Vehicle armed: {cmd['value']}"
                self.send_log(response['message'])
                
            elif cmd['type'] == 'mode':
                self._vehicle.mode = VehicleMode(cmd['value'])
                response['message'] = f"Mode changed to: {cmd['value']}"
                self.send_log(response['message'])
                
            elif cmd['type'] == 'takeoff':
                await self._takeoff(cmd.get('altitude', 10))
                response['message'] = f"Taking off to {cmd.get('altitude', 10)}m"
                self.send_log(response['message'])
                
            elif cmd['type'] == 'goto':
                await self._goto(cmd['lat'], cmd['lon'], cmd['alt'])
                response['message'] = "Moving to location"
                self.send_log(response['message'])
                
            elif cmd['type'] == 'rtl':
                self._vehicle.mode = VehicleMode("RTL")
                response['message'] = "Returning to launch"
                self.send_log(response['message'])
            
            elif cmd['type'] == 'land':
                self._vehicle.mode = VehicleMode("LAND")
                response['message'] = "Landing initiated"
                self.send_log(response['message'])
            
            elif cmd['type'] == 'guided':
                self._vehicle.mode = VehicleMode("GUIDED")
                response['message'] = "Guided mode activated"
                self.send_log(response['message'])
            
            elif cmd['type'] == 'mission':
                if not self.mission:
                    self.mission = True
                    threading.Thread(target=self.run_autonomous_mission, daemon=True).start()
                    response['message'] = "Autonomous mission started"
                    self.send_log(response['message'])
                else:
                    response['message'] = "Mission already in progress"
                    self.send_log(response['message'])
                
            await websocket.send(json.dumps(response))
            
        except Exception as e:
            err_msg = str(e)
            await websocket.send(json.dumps({
                'status': 'error',
                'message': err_msg
            }))
            self.send_log(err_msg, level="error")

    def run_autonomous_mission(self):
        """Run the autonomous mission in a separate thread."""
        self.autoMission.mission()
        self.mission = False
        self.send_log("Autonomous mission completed")

    async def _takeoff(self, altitude):
        self._vehicle.armed = True
        while not self._vehicle.armed:
            await asyncio.sleep(1)
        
        self._vehicle.simple_takeoff(altitude)
        
        while True:
            if self._vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                break
            await asyncio.sleep(1)

    async def _goto(self, lat, lon, alt):
        point = LocationGlobalRelative(lat, lon, alt)
        self._vehicle.simple_goto(point)

    async def _broadcast_state(self):
        if self._websocket_clients:
            message = json.dumps(self._current_state)
            websockets.broadcast(self._websocket_clients, message)

    def _start_websocket_server(self):
        def run_server():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            
            async def serve():
                self._server = await websockets.serve(
                    self._websocket_handler,
                    "localhost",
                    8765
                )
                await self._server.wait_closed()
            
            try:
                self._loop.run_until_complete(serve())
                self._loop.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebSocket server error: {str(e)}")
                self.send_log(f"WebSocket server error: {str(e)}", level="error")
            finally:
                self._loop.close()
        
        self._ws_thread = threading.Thread(target=run_server, daemon=True)
        self._ws_thread.start()

    def cleanup(self):
        if self._vehicle:
            self._vehicle.close()
        
        if self._server:
            self._loop.call_soon_threadsafe(self._server.close)
            time.sleep(0.5)
        
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        
        if hasattr(self, '_ws_thread') and self._ws_thread.is_alive():
            self._ws_thread.join(timeout=1.0)

    # ===== New Methods for Sending Log Messages =====
    def send_log(self, message, level="info"):
        """
        Send a log message to all connected WebSocket clients.
        The message is sent as a JSON object with a type "log".
        """
        log_data = {"type": "log", "level": level, "message": message}
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast_log(log_data), self._loop)

    async def _broadcast_log(self, log_data):
        if self._websocket_clients:
            message = json.dumps(log_data)
            websockets.broadcast(self._websocket_clients, message)

def main():
    rclpy.init()
    try:
        bridge = DronekitBridge()
        rclpy.spin(bridge)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'bridge' in locals():
            bridge.cleanup()
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
