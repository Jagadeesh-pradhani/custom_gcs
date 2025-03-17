#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # For battery percentage
from dronekit import connect, VehicleMode, LocationGlobalRelative
import websockets
from custom_gcs.mission import AutonomousMission
import asyncio
import json
import threading
import time

class DronekitBridge(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('dronekit_bridge')
        
        # Declare and read parameter for number of drones
        self.declare_parameter("num_drones", 1)
        self.num_drones = self.get_parameter("num_drones").value
        
        # Containers for multi-drone support
        self._websocket_clients = set()
        self._loop = None
        self._server = None
        self.vehicles = {}             # drone_id -> vehicle instance
        self._current_states = {}      # drone_id -> state dict
        self.mission_flags = {}        # drone_id -> bool flag for autonomous mission
        self._battery_percents = {}    # drone_id -> battery percentage from topic
        self._battery_warning_sent = {}  # drone_id -> bool flag
        self.autoMissions = {}         # drone_id -> AutonomousMission instance
        
        # Initialize state containers for each drone
        for drone_id in range(self.num_drones):
            self._current_states[drone_id] = {
                'connected': False,
                'armed': False,
                'mode': 'UNKNOWN',
                'battery': 0.0,
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
            self.mission_flags[drone_id] = False
            self._battery_percents[drone_id] = 0.0
            self._battery_warning_sent[drone_id] = False

        self.send_log(f"DronekitBridge initialized with {self.num_drones} drone(s)")
        
        # Connect to each vehicle using unique UDP ports (14550, 14551, ...)
        for drone_id in range(self.num_drones):
            connection_string = f'udp:127.0.0.1:{14550 + drone_id*10}'
            self.get_logger().info(f"Connecting to vehicle {drone_id} on {connection_string}...")
            self.send_log(f"Connecting to vehicle {drone_id} on {connection_string}...")
            try:
                vehicle = connect(connection_string, wait_ready=True)
                self.vehicles[drone_id] = vehicle
                self.get_logger().info(f"Vehicle {drone_id} connected!")
                self.send_log(f"Vehicle {drone_id} connected!")
            except Exception as e:
                error_msg = f"Failed to connect to vehicle {drone_id}: {str(e)}"
                self.get_logger().error(error_msg)
                self.send_log(error_msg, level="error")
        
        # Create a battery subscription for each drone on a unique topic (e.g., /drone0/percent)
        for drone_id in range(self.num_drones):
            topic = f'/drone{drone_id}/percent'
            self.create_subscription(Float32, topic, self.create_percent_callback(drone_id), 10)
        
        # Create a timer to update the state for each vehicle
        self.create_timer(0.1, self.update_vehicle_states)
        
        # Initialize an autonomous mission instance per drone
        for drone_id in range(self.num_drones):
            self.autoMissions[drone_id] = AutonomousMission(self)
        
        # Start the WebSocket server
        self._start_websocket_server()

    def create_percent_callback(self, drone_id):
        # Returns a callback function that updates battery info for the given drone_id
        def percent_callback(msg: Float32):
            self._battery_percents[drone_id] = round(msg.data, 2)
            self._current_states[drone_id]["battery"] = self._battery_percents[drone_id]
            if msg.data < 10 and not self._battery_warning_sent[drone_id]:
                self.send_log(f"Warning: Battery for drone {drone_id} below 10%!", level="warning")
                self._battery_warning_sent[drone_id] = True
            elif msg.data >= 10:
                self._battery_warning_sent[drone_id] = False
        return percent_callback

    def update_vehicle_states(self):
        # Update state for each connected vehicle
        for drone_id, vehicle in self.vehicles.items():
            try:
                self._current_states[drone_id].update({
                    'connected': True,
                    'armed': vehicle.armed,
                    'mode': vehicle.mode.name,
                    'battery': self._battery_percents[drone_id] if self._battery_percents[drone_id] > 10.0 else 10.0,
                    'lat': vehicle.location.global_relative_frame.lat,
                    'lon': vehicle.location.global_relative_frame.lon,
                    'alt': round(vehicle.location.global_relative_frame.alt, 2),
                    'heading': vehicle.heading,
                    'groundspeed': round(vehicle.groundspeed, 2),
                    'airspeed': round(vehicle.airspeed, 2),
                    'attitude': {
                        'roll': vehicle.attitude.roll,
                        'pitch': vehicle.attitude.pitch,
                        'yaw': vehicle.attitude.yaw
                    }
                })
            except Exception as e:
                error_msg = f"Error updating state for drone {drone_id}: {str(e)}"
                self.get_logger().error(error_msg)
                self.send_log(error_msg, level="error")
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast_state(), self._loop)

    async def _websocket_handler(self, websocket):
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
        try:
            cmd = json.loads(message)
            # Determine target(s): if "target" equals "all", then send command to all drones;
            # otherwise if "target" is specified, use that drone id. Default is drone 0.
            targets = []
            if 'target' in cmd:
                if cmd['target'] == "all":
                    targets = list(self.vehicles.keys())
                else:
                    try:
                        target_id = int(cmd['target'])
                        if target_id in self.vehicles:
                            targets = [target_id]
                        else:
                            raise ValueError("Invalid drone id")
                    except Exception:
                        await websocket.send(json.dumps({
                            'status': 'error',
                            'message': f'Invalid target: {cmd["target"]}'
                        }))
                        return
            else:
                targets = [0]  # default target is the first drone
            
            responses = []
            for drone_id in targets:
                response = await self.process_command_for_drone(drone_id, cmd)
                responses.append({ "drone_id": drone_id, "response": response })
            await websocket.send(json.dumps({
                'status': 'success',
                'responses': responses
            }))
        except Exception as e:
            err_msg = str(e)
            await websocket.send(json.dumps({
                'status': 'error',
                'message': err_msg
            }))
            self.send_log(err_msg, level="error")
    
    async def process_command_for_drone(self, drone_id, cmd):
        vehicle = self.vehicles.get(drone_id)
        if not vehicle:
            return {'message': f'Vehicle {drone_id} not connected'}
        try:
            if cmd['type'] == 'arm':
                vehicle.armed = cmd['value']
                message = f"Drone {drone_id} armed: {cmd['value']}"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'mode':
                vehicle.mode = VehicleMode(cmd['value'])
                message = f"Drone {drone_id} mode changed to: {cmd['value']}"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'takeoff':
                altitude = cmd.get('altitude', 10)
                await self._takeoff(vehicle, altitude, drone_id)
                message = f"Drone {drone_id} taking off to {altitude}m"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'goto':
                # Retrieve target coordinates from the command
                lat = cmd['lat']
                lon = cmd['lon']
                alt = cmd['alt']
                # If targeting all drones, let the drone with the smallest id go exactly
                # and assign an offset for the others.
                if cmd.get('target') == 'all':
                    if drone_id != min(self.vehicles.keys()):
                        offset = 0.0001  # Offset value (adjust as needed)
                        # A simple offset pattern: alternate between adding to latitude or longitude
                        if drone_id % 2 == 0:
                            lat = lat + offset
                        else:
                            lon = lon + offset
                await self._goto(vehicle, lat, lon, alt)
                message = f"Drone {drone_id} moving to location ({lat}, {lon}, {alt})"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'rtl':
                vehicle.mode = VehicleMode("RTL")
                message = f"Drone {drone_id} returning to launch"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'land':
                vehicle.mode = VehicleMode("LAND")
                message = f"Drone {drone_id} landing initiated"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'guided':
                vehicle.mode = VehicleMode("GUIDED")
                message = f"Drone {drone_id} guided mode activated"
                self.send_log(message)
                return {'message': message}
            elif cmd['type'] == 'mission':
                if not self.mission_flags[drone_id]:
                    self.mission_flags[drone_id] = True
                    threading.Thread(target=self.run_autonomous_mission, args=(drone_id,), daemon=True).start()
                    message = f"Autonomous mission started for drone {drone_id}"
                    self.send_log(message)
                    return {'message': message}
                else:
                    message = f"Mission already in progress for drone {drone_id}"
                    self.send_log(message)
                    return {'message': message}
            else:
                return {'message': f"Unknown command type: {cmd['type']}"}
        except Exception as e:
            error_msg = f"Error processing command for drone {drone_id}: {str(e)}"
            self.send_log(error_msg, level="error")
            return {'message': error_msg}
    
    def run_autonomous_mission(self, drone_id):
        try:
            self.autoMissions[drone_id].mission()
            self.send_log(f"Autonomous mission completed for drone {drone_id}")
        except Exception as e:
            self.send_log(f"Error in autonomous mission for drone {drone_id}: {str(e)}", level="error")
        finally:
            self.mission_flags[drone_id] = False

    async def _takeoff(self, vehicle, altitude, drone_id):
        vehicle.armed = True
        while not vehicle.armed:
            await asyncio.sleep(1)
        vehicle.simple_takeoff(altitude)
        while True:
            if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                break
            await asyncio.sleep(1)

    async def _goto(self, vehicle, lat, lon, alt):
        point = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point)

    async def _broadcast_state(self):
        # Broadcast an object with the states for all drones
        message = json.dumps({"drones": self._current_states})
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
        for drone_id, vehicle in self.vehicles.items():
            vehicle.close()
        if self._server:
            self._loop.call_soon_threadsafe(self._server.close)
            time.sleep(0.5)
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if hasattr(self, '_ws_thread') and self._ws_thread.is_alive():
            self._ws_thread.join(timeout=1.0)

    # ===== New Methods for Sending Log Messages =====
    def send_log(self, message, level="info"):
        log_data = {"type": "log", "level": level, "message": message}
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast_log(log_data), self._loop)

    async def _broadcast_log(self, log_data):
        if self._websocket_clients:
            message = json.dumps(log_data)
            websockets.broadcast(self._websocket_clients, message)

def main(args=None):
    rclpy.init(args=args)  # Pass command-line args to allow parameter overrides
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
