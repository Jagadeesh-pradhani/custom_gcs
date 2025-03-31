// src/App.jsx
import React, { useState, useEffect, useRef } from 'react';
import { Wifi } from 'lucide-react';
import { loadModules } from 'esri-loader';
import logo from './assets/GCS.png';

// =================== Drone Status Component =================== //
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

// =================== Telemetry Data Component =================== //
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

// =================== Control Panel Component =================== //
const ControlPanel = ({ onCommand }) => {
  const [altitude, setAltitude] = useState(10);
  
  // New state variables for goto coordinates and groundspeed
  const [gotoLat, setGotoLat] = useState('');
  const [gotoLon, setGotoLon] = useState('');
  const [gotoAlt, setGotoAlt] = useState(10);
  const [gotoGroundSpeed, setGotoGroundSpeed] = useState(5); // default ground speed

  return (
    <div className="control-panel">
      <div className="altitude-control">
        <label>Takeoff Altitude (m): </label>
        <input
          type="number"
          min="1"
          value={altitude}
          onChange={(e) =>
            setAltitude(Math.max(1, parseInt(e.target.value) || 10))
          }
        />
      </div>
      <button onClick={() => onCommand('arm')} className="btn btn-brown">
        Arm
      </button>
      <button onClick={() => onCommand('takeoff', altitude)} className="btn btn-brown">
        Takeoff
      </button>
      <button onClick={() => onCommand('rtl')} className="btn btn-brown">
        Return to Launch
      </button>
      <button onClick={() => onCommand('land')} className="btn btn-brown">
        Land
      </button>
      <button onClick={() => onCommand('guided')} className="btn btn-brown">
        Guided
      </button>
      <button onClick={() => onCommand('mission')} className="btn btn-brown">
        Mission
      </button>

      {/* New Goto Controls */}
      <div className="goto-control" style={{ marginTop: '1rem' }}>
        <h3>Go To Coordinates</h3>
        <div>
          <label>Target Latitude: </label>
          <input
            type="number"
            value={gotoLat}
            onChange={(e) => setGotoLat(e.target.value)}
            placeholder="e.g., -35.363262"
          />
        </div>
        <div>
          <label>Target Longitude: </label>
          <input
            type="number"
            value={gotoLon}
            onChange={(e) => setGotoLon(e.target.value)}
            placeholder="e.g., 149.165237"
          />
        </div>
        <div>
          <label>Target Altitude (m): </label>
          <input
            type="number"
            value={gotoAlt}
            onChange={(e) => setGotoAlt(e.target.value)}
          />
        </div>
        <div>
          <label>Ground Speed (m/s): </label>
          <input
            type="number"
            value={gotoGroundSpeed}
            onChange={(e) => setGotoGroundSpeed(e.target.value)}
          />
        </div>
        <button
          onClick={() => onCommand('goto', gotoLat, gotoLon, gotoAlt, gotoGroundSpeed)}
          className="btn btn-brown"
        >
          Go To
        </button>
      </div>
    </div>
  );
};

// =================== Log Panel Component =================== //
const LogPanel = ({ logs }) => {
  return (
    <div className="log-panel" style={{ marginTop: '1rem' }}>
      <h2>System Log</h2>
      <div
        style={{
          maxHeight: '200px',
          overflowY: 'scroll',
          background: '#f5f5f5',
          padding: '0.5rem',
          border: '1px solid #ccc'
        }}
      >
        {logs.map((log, index) => (
          <div key={index} className="log-entry">
            {log}
          </div>
        ))}
      </div>
    </div>
  );
};

// =================== ArcGIS Map Component with Path Tracking =================== //
const ArcgisMap = ({ lat, lon }) => {
  const mapRef = useRef(null);
  const viewRef = useRef(null);
  const markerGraphicRef = useRef(null);
  const pathGraphicRef = useRef(null);
  const graphicsLayerRef = useRef(null);
  const polylineCoordsRef = useRef([]);
  const GraphicModuleRef = useRef(null);

  useEffect(() => {
    loadModules(
      [
        'esri/Map',
        'esri/views/MapView',
        'esri/Graphic',
        'esri/layers/GraphicsLayer'
      ],
      { css: true }
    )
      .then(([Map, MapView, Graphic, GraphicsLayer]) => {
        GraphicModuleRef.current = Graphic;
        const map = new Map({
          basemap: 'satellite'
        });
        const view = new MapView({
          container: mapRef.current,
          map: map,
          center: [lon || 0, lat || 0],
          zoom: 15
        });
        viewRef.current = view;
        const graphicsLayer = new GraphicsLayer();
        map.add(graphicsLayer);
        graphicsLayerRef.current = graphicsLayer;
        const markerGraphic = new Graphic({
          geometry: {
            type: 'point',
            longitude: lon || 0,
            latitude: lat || 0
          },
          symbol: {
            type: 'simple-marker',
            color: [226, 119, 40],
            outline: { color: [255, 255, 255], width: 2 }
          }
        });
        graphicsLayer.add(markerGraphic);
        markerGraphicRef.current = markerGraphic;
        polylineCoordsRef.current = [[lon, lat]];
        const polylineGraphic = new Graphic({
          geometry: {
            type: 'polyline',
            paths: polylineCoordsRef.current
          },
          symbol: {
            type: 'simple-line',
            color: [0, 0, 255],
            width: 2
          }
        });
        graphicsLayer.add(polylineGraphic);
        pathGraphicRef.current = polylineGraphic;
      })
      .catch((err) => console.error('ArcGIS loadModules error: ', err));

    return () => {
      if (viewRef.current) {
        viewRef.current.destroy();
      }
    };
  }, []);

  useEffect(() => {
    if (
      markerGraphicRef.current &&
      viewRef.current &&
      GraphicModuleRef.current &&
      graphicsLayerRef.current
    ) {
      markerGraphicRef.current.geometry = {
        type: 'point',
        longitude: lon,
        latitude: lat
      };

      const lastCoords =
        polylineCoordsRef.current[polylineCoordsRef.current.length - 1];
      const toRadians = (deg) => (deg * Math.PI) / 180;
      const latDiff = lat - lastCoords[1];
      const lonDiff = lon - lastCoords[0];
      const avgLat = (lat + lastCoords[1]) / 2;
      const metersPerDegLat = 111320;
      const metersPerDegLon = 111320 * Math.cos(toRadians(avgLat));
      const distance = Math.sqrt(
        (latDiff * metersPerDegLat) ** 2 + (lonDiff * metersPerDegLon) ** 2
      );

      const threshold = 2;
      if (distance >= threshold) {
        polylineCoordsRef.current.push([lon, lat]);
        pathGraphicRef.current.geometry = {
          type: 'polyline',
          paths: polylineCoordsRef.current
        };
      }
      viewRef.current.center = [lon, lat];
    }
  }, [lat, lon]);

  return <div style={{ height: '100%', width: '100%' }} ref={mapRef}></div>;
};

// =================== Main App Component =================== //
const App = () => {
  const [websocket, setWebsocket] = useState(null);
  const [droneState, setDroneState] = useState({
    connected: false,
    battery: 0,
    mode: 'STABILIZE',
    altitude: 0,
    lat: 39.925533,
    lon: 32.866287,
    telemetry: {
      heading: 0,
      groundspeed: 0,
      airspeed: 0
    }
  });
  const [logs, setLogs] = useState([]);

  const addLog = (message) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs((prev) => [...prev, `[${timestamp}] ${message}`]);
  };

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8765');
    setWebsocket(ws);

    ws.onopen = () => {
      console.log('Connected to WebSocket server');
      addLog('Connected to WebSocket server');
      setDroneState((prev) => ({ ...prev, connected: true }));
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "log" && data.message) {
        addLog(`[${data.level.toUpperCase()}] ${data.message}`);
      }
      setDroneState((prev) => ({
        ...prev,
        battery: data.battery,
        mode: data.mode,
        altitude: data.alt,
        lat: data.lat,
        lon: data.lon,
        telemetry: {
          heading: data.heading,
          groundspeed: data.groundspeed,
          airspeed: data.airspeed
        }
      }));
    };

    ws.onclose = () => {
      console.log('Disconnected from WebSocket server');
      addLog('Disconnected from WebSocket server');
      setDroneState((prev) => ({ ...prev, connected: false }));
    };

    return () => {
      if (ws) ws.close();
    };
  }, []);

  // Update handleCommand to support the "goto" command with groundspeed.
  const handleCommand = (commandType, ...params) => {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      console.error('WebSocket not connected');
      addLog('WebSocket not connected');
      return;
    }

    let commandObj;
    switch (commandType) {
      case 'arm':
        commandObj = { type: 'arm', value: true };
        break;
      case 'takeoff':
        const takeoffAltitude = params[0] || 10;
        commandObj = { type: 'takeoff', altitude: takeoffAltitude };
        break;
      case 'rtl':
        commandObj = { type: 'mode', value: 'RTL' };
        break;
      case 'guided':
        commandObj = { type: 'mode', value: 'GUIDED' };
        break;
      case 'land':
        commandObj = { type: 'mode', value: 'LAND' };
        break;
      case 'mission':
        commandObj = { type: 'mission' };
        break;
      case 'goto': {
        const targetLat = params[0];
        const targetLon = params[1];
        const targetAlt = params[2];
        const groundSpeed = params[3] ? parseFloat(params[3]) : 5;
        commandObj = { 
          type: 'goto', 
          lat: parseFloat(targetLat), 
          lon: parseFloat(targetLon), 
          alt: parseFloat(targetAlt),
          groundspeed: groundSpeed
        };
        break;
      }
      default:
        console.error('Unknown command:', commandType);
        addLog(`Unknown command: ${commandType}`);
        return;
    }

    addLog(`Sending command: ${JSON.stringify(commandObj)}`);
    websocket.send(JSON.stringify(commandObj));
  };

  return (
    <div className="container" style={{ height: '100vh', width: '100vw' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', marginBottom: '1rem' }}>
        <img src={logo} alt="Logo" style={{ height: '50px', marginRight: '1rem' }} />
        <h1>Custom Ground Control Station</h1>
      </div>
      <DroneStatus
        connected={droneState.connected}
        battery={droneState.battery}
        mode={droneState.mode}
        altitude={droneState.altitude}
      />

      <div
        className="main-content"
        style={{ display: 'flex', gap: '1rem', height: 'calc(100% - 100px)' }}
      >
        <div className="map-container" style={{ height: '100%', flex: 2 }}>
          <ArcgisMap lat={droneState.lat} lon={droneState.lon} />
        </div>
        <div className="side-panel" style={{ flex: 1 }}>
          <TelemetryData data={droneState.telemetry} />
          <ControlPanel onCommand={handleCommand} />
          <LogPanel logs={logs} />
        </div>
      </div>
    </div>
  );
};

export default App;
