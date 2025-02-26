#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from dronekit import connect, VehicleMode, LocationGlobalRelative
import websockets
import asyncio
import threading
import time
import json

class DronekitBridge(Node):
    def __init__(self):
        super().__init__('dronekit_bridge')
        # WebSocket configuration
        self._websocket_clients = set()
        self._loop = None
        self._server = None

        # State management
        self._battery_percent = 0.0
        self._battery_warning_sent = False
        self._current_state = {
            'connected': False,
            'armed': False,
            'mode': 'UNKNOWN',
            'battery': 0.0,
            'lat': 0.0,
            'lon': 0.0,
            'altitude': 0.0,  # Changed key from "alt" to "altitude"
            'heading': 0.0,
            'groundspeed': 0.0,
            'airspeed': 0.0,
            'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }

        # Vehicle connection
        self._vehicle = None
        self._connect_vehicle()

        # ROS2 setup
        self.create_subscription(Float32, '/percent', self.percent_callback, 10)
        self.create_timer(0.1, self.update_vehicle_state)

        # Start WebSocket server
        self._start_websocket_server()

        self.get_logger().info("Dronekit Bridge initialized")

    def _connect_vehicle(self):
        """Connect to the vehicle."""
        try:
            self._vehicle = connect('udp:127.0.0.1:14550', wait_ready=True, baud=57600)
            self._current_state['connected'] = True
            self.send_log("Vehicle connected successfully")
        except Exception as e:
            self.get_logger().error(f"Connection failed: {str(e)}")
            self.send_log(f"Connection error: {str(e)}", level="error")

    def percent_callback(self, msg):
        """Handle battery percentage updates."""
        self._battery_percent = msg.data
        self._current_state['battery'] = round(msg.data, 2)

        if msg.data < 20.0 and not self._battery_warning_sent:
            self.send_log(f"Low battery warning: {msg.data}%", level="warning")
            self._battery_warning_sent = True
        elif msg.data >= 20.0:
            self._battery_warning_sent = False

    def update_vehicle_state(self):
        """Update vehicle state parameters."""
        if not self._vehicle:
            return

        try:
            self._current_state.update({
                'armed': self._vehicle.armed,
                'mode': self._vehicle.mode.name,
                'lat': self._vehicle.location.global_relative_frame.lat,
                'lon': self._vehicle.location.global_relative_frame.lon,
                'altitude': round(self._vehicle.location.global_relative_frame.alt, 2),  # Updated key here
                'heading': self._vehicle.heading,
                'groundspeed': round(self._vehicle.groundspeed, 2),
                'airspeed': round(self._vehicle.airspeed, 2),
                'attitude': {
                    'roll': self._vehicle.attitude.roll,
                    'pitch': self._vehicle.attitude.pitch,
                    'yaw': self._vehicle.attitude.yaw
                }
            })

            if self._loop and self._loop.is_running():
                asyncio.run_coroutine_threadsafe(self._broadcast_state(), self._loop)

        except Exception as e:
            self.get_logger().error(f"State update error: {str(e)}")
            self.send_log(f"State update failed: {str(e)}", level="error")

    async def _websocket_handler(self, websocket):
        """Handle WebSocket connections."""
        self._websocket_clients.add(websocket)
        try:
            async for message in websocket:
                await self._handle_command(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._websocket_clients.remove(websocket)

    async def _handle_command(self, websocket, message):
        """Process incoming commands."""
        response = {'status': 'error', 'message': 'Unknown command'}
        try:
            cmd = json.loads(message)
            command_type = cmd.get('type')

            if not self._vehicle:
                response['message'] = 'Vehicle not connected'
                await websocket.send(json.dumps(response))
                return

            # Command processing
            if command_type == 'arm':
                success = await self._handle_arm_command(cmd.get('value'))
                response = self._format_response(success,
                                                 f"Armed: {cmd.get('value')}",
                                                 "Arming failed")
            elif command_type == 'mode':
                success = await self._change_mode(cmd.get('value'))
                response = self._format_response(success,
                                                 f"Mode changed to {cmd.get('value')}",
                                                 "Mode change failed")
            elif command_type == 'takeoff':
                altitude = cmd.get('altitude', 10)
                success = await self._execute_takeoff(altitude)
                response = self._format_response(success,
                                                 f"Taking off to {altitude}m",
                                                 "Takeoff failed")
            elif command_type == 'goto':
                success = await self._execute_goto(
                    cmd.get('lat'),
                    cmd.get('lon'),
                    cmd.get('alt')
                )
                response = self._format_response(success,
                                                 f"Navigating to {cmd.get('lat')}, {cmd.get('lon')}",
                                                 "GoTo command failed")
            elif command_type == 'rtl':
                success = await self._change_mode('RTL')
                response = self._format_response(success,
                                                 "Returning to launch",
                                                 "RTL failed")
            elif command_type == 'land':
                success = await self._change_mode('LAND')
                response = self._format_response(success,
                                                 "Landing initiated",
                                                 "Landing failed")
            else:
                response['message'] = 'Invalid command type'

            await websocket.send(json.dumps(response))

        except Exception as e:
            error_msg = f"Command error: {str(e)}"
            self.get_logger().error(error_msg)
            await websocket.send(json.dumps({
                'status': 'error',
                'message': error_msg
            }))

    async def _handle_arm_command(self, value):
        """Handle arming/disarming the vehicle."""
        try:
            if value:
                self.send_log("Arming vehicle")
                self._vehicle.mode = VehicleMode("GUIDED")
                self._vehicle.armed = True
                # Wait for arming confirmation
                timeout = time.time() + 10  # 10-second timeout
                while not self._vehicle.armed:
                    if time.time() > timeout:
                        raise Exception("Arming timeout")
                    await asyncio.sleep(0.5)
                return True
            else:
                self.send_log("Disarming vehicle")
                self._vehicle.armed = False
                # Wait for disarming confirmation
                timeout = time.time() + 10
                while self._vehicle.armed:
                    if time.time() > timeout:
                        raise Exception("Disarming timeout")
                    await asyncio.sleep(0.5)
                return True
        except Exception as e:
            self.send_log(f"Arming command failed: {str(e)}", level="error")
            return False

    async def _change_mode(self, mode):
        """Change the vehicle's mode."""
        try:
            self.send_log(f"Changing mode to {mode}")
            self._vehicle.mode = VehicleMode(mode)
            # Wait for mode change confirmation
            timeout = time.time() + 10
            while self._vehicle.mode.name != mode:
                if time.time() > timeout:
                    raise Exception("Mode change timeout")
                await asyncio.sleep(0.5)
            return True
        except Exception as e:
            self.send_log(f"Mode change failed: {str(e)}", level="error")
            return False

    async def _execute_goto(self, lat, lon, alt):
        """Execute GoTo command with safety checks."""
        if None in [lat, lon, alt]:
            raise ValueError("Missing coordinates")

        if not await self._ensure_guided_mode():
            return False

        try:
            target = LocationGlobalRelative(lat, lon, alt)
            self._vehicle.simple_goto(target)
            self.send_log(f"Going to: Lat {lat}, Lon {lon}, Alt {alt}m")
            return True
        except Exception as e:
            self.send_log(f"GoTo failed: {str(e)}", level="error")
            return False

    async def _ensure_guided_mode(self):
        """Ensure the vehicle is in GUIDED mode."""
        if self._vehicle.mode.name == "GUIDED":
            return True

        try:
            self.send_log("Switching to GUIDED mode")
            self._vehicle.mode = VehicleMode("GUIDED")
            # Wait for mode change
            for _ in range(10):
                if self._vehicle.mode.name == "GUIDED":
                    return True
                await asyncio.sleep(0.5)
            raise Exception("Failed to switch to GUIDED mode")
        except Exception as e:
            self.send_log(str(e), level="error")
            return False

    async def _execute_takeoff(self, altitude):
        """Handle the takeoff procedure."""
        if not self._vehicle.armed:
            await self._handle_arm_command(True)

        self._vehicle.simple_takeoff(altitude)

        # Wait for takeoff completion
        timeout = time.time() + 60  # 1-minute timeout
        while True:
            current_alt = self._vehicle.location.global_relative_frame.alt
            if current_alt >= altitude * 0.95:
                return True
            if time.time() > timeout:
                raise Exception("Takeoff timeout")
            await asyncio.sleep(1)

    def _format_response(self, success, success_msg, error_msg):
        """Format a WebSocket response."""
        return {
            'status': 'success' if success else 'error',
            'message': success_msg if success else error_msg
        }

    async def _broadcast_state(self):
        """Send the current state to all connected clients."""
        if self._websocket_clients:
            message = json.dumps({
                **self._current_state,
                'timestamp': time.time()
            })
            websockets.broadcast(self._websocket_clients, message)

    def send_log(self, message, level="info"):
        """Send a log message to connected clients."""
        log_data = json.dumps({
            'type': 'log',
            'level': level,
            'message': message,
            'timestamp': time.time()
        })

        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._broadcast_log(log_data),
                self._loop
            )

    async def _broadcast_log(self, log_data):
        """Broadcast a log message to clients."""
        if self._websocket_clients:
            websockets.broadcast(self._websocket_clients, log_data)

    def _start_websocket_server(self):
        """Start the WebSocket server in a background thread."""
        def run_server():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            async def server_main():
                self._server = await websockets.serve(
                    self._websocket_handler,
                    "0.0.0.0",
                    8765,
                    ping_interval=None
                )
                await self._server.wait_closed()

            try:
                self._loop.run_until_complete(server_main())
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {str(e)}")
            finally:
                self._loop.close()

        self._ws_thread = threading.Thread(target=run_server, daemon=True)
        self._ws_thread.start()

    def cleanup(self):
        """Cleanup resources on shutdown."""
        self.get_logger().info("Shutting down bridge...")

        # Close vehicle connection
        if self._vehicle:
            self._vehicle.close()
            self.get_logger().info("Vehicle connection closed")

        # Stop WebSocket server
        if self._server:
            self._loop.call_soon_threadsafe(self._server.close())

        # Stop ROS2 node
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = DronekitBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
