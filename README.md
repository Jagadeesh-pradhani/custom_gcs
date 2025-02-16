# Custom Ground Control Station (GCS)

ROS2-based Ground Control Station with web interface for drone management.

## Features
- Real-time telemetry display
- Drone arm/disarm control
- Flight mode switching
- Takeoff/Landing/RTL commands
- Cross-platform desktop app

## Prerequisites
- ROS2 Humble/Hawksbill
- Python 3.8+
- Node.js 18+
- Dronekit-Python 2.4.4

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/YOUR_USERNAME/custom_gcs.git
cd custom_gcs
```

### 2. Install ROS2 Dependencies
```bash
sudo apt install python3-pip
pip install dronekit websockets

# In your ROS2 workspace
colcon build --packages-select custom_gcs
source install/setup.bash
```

### 3. Install Web Dependencies
```bash
cd custom_gcs/web
npm install
npm install cross-env --save-dev
```

### 4. Build Web Interface
```bash
npm run build
```

## Usage

### 1. Start ROS2 Bridge
```bash
source install/setup.bash
ros2 launch custom_gcs gcs.launch.py
```

### 2. Run GCS Application

#### Development Mode:
```bash
cd custom_gcs/web
npm run electron:dev
```

#### Production Mode:
```bash
cd custom_gcs/web
npm run electron:build
# App will be in release/ directory
```

## Configuration
Edit `custom_gcs/custom_gcs/bridge_node.py` to modify:
- Drone connection parameters
- WebSocket port (default: 8765)
- Telemetry update rate

## Troubleshooting

### Connection Issues
- Ensure drone is connected via UDP (default: 127.0.0.1:14550)
- Check firewall settings for WebSocket port 8765

### Build Errors
```bash
rm -rf node_modules package-lock.json
npm install
```

## License
MIT License
