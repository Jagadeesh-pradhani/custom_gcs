// src/App.jsx
import React, { useState, useEffect } from 'react';
import { Wifi } from 'lucide-react';

const DroneStatus = ({ connected, battery, mode, altitude }) => {
  return (
    <div className="status-panel">
      <div className="status-item">
        <Wifi className={connected ? 'icon-green' : 'icon-red'} />
        <span>{connected ? 'Connected' : 'Disconnected'}</span>
      </div>
      <div className="status-item">
        <div>Battery: {battery}%</div>
        <div className="mode-display">Mode: {mode}</div>
        <div>Alt: {altitude}m</div>
      </div>
    </div>
  );
};

const MapView = () => {
  return (
    <div className="map-view">
      <div>Map View Placeholder</div>
    </div>
  );
};

const TelemetryData = ({ data }) => {
  return (
    <div className="telemetry-panel">
      <h2>Telemetry Data</h2>
      <div className="telemetry-grid">
        <div>
          <strong>Heading:</strong> {data.heading}°
        </div>
        <div>
          <strong>Groundspeed:</strong> {data.groundspeed} m/s
        </div>
        <div>
          <strong>Airspeed:</strong> {data.airspeed} m/s
        </div>
      </div>
    </div>
  );
};

const ControlPanel = ({ onCommand }) => {
  return (
    <div className="control-panel">
      <button 
        onClick={() => onCommand('arm')}
        className="btn btn-blue"
      >
        Arm
      </button>
      <button 
        onClick={() => onCommand('takeoff')}
        className="btn btn-green"
      >
        Takeoff
      </button>
      <button 
        onClick={() => onCommand('rtl')}
        className="btn btn-yellow"
      >
        Return to Launch
      </button>
      <button 
        onClick={() => onCommand('land')}
        className="btn btn-red"
      >
        Land
      </button>
    </div>
  );
};

const App = () => {
  const [websocket, setWebsocket] = useState(null);
  const [droneState, setDroneState] = useState({
    connected: false,
    battery: 0,
    mode: 'STABILIZE',
    altitude: 0,
    telemetry: {
      heading: 0,
      groundspeed: 0,
      airspeed: 0
    }
  });

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8765');
    setWebsocket(ws);

    ws.onopen = () => {
      console.log('Connected to WebSocket server');
      setDroneState(prev => ({ ...prev, connected: true }));
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setDroneState(prev => ({
        ...prev,
        battery: data.battery,
        mode: data.mode,
        altitude: data.alt,
        telemetry: {
          heading: data.heading,
          groundspeed: data.groundspeed,
          airspeed: data.airspeed
        }
      }));
    };

    ws.onclose = () => {
      console.log('Disconnected from WebSocket server');
      setDroneState(prev => ({ ...prev, connected: false }));
    };

    return () => {
      if (ws) ws.close();
    };
  }, []);

  const handleCommand = (command) => {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      console.error('WebSocket not connected');
      return;
    }

    const commandMap = {
      'arm': { type: 'arm', value: true },
      'takeoff': { type: 'takeoff', altitude: 10 },
      'rtl': { type: 'mode', value: 'RTL' },
      'land': { type: 'mode', value: 'LAND' }
    };

    const commandObj = commandMap[command];
    if (commandObj) {
      websocket.send(JSON.stringify(commandObj));
    } else {
      console.error('Unknown command:', command);
    }
  };

  return (
    <div className="container">
      <h1>Custom Ground Control Station</h1>
      
      <DroneStatus 
        connected={droneState.connected}
        battery={droneState.battery}
        mode={droneState.mode}
        altitude={droneState.altitude}
      />
      
      <div className="main-content">
        <div className="map-container">
          <MapView />
        </div>
        <div className="side-panel">
          <TelemetryData data={droneState.telemetry} />
          <ControlPanel onCommand={handleCommand} />
        </div>
      </div>
    </div>
  );
};

export default App;