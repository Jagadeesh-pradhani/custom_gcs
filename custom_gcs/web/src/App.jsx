// src/App.jsx
import React, { useState, useEffect, useRef } from 'react';
import { Wifi } from 'lucide-react';
import { loadModules } from 'esri-loader';

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
      <button
        onClick={() => onCommand('takeoff', altitude)}
        className="btn btn-brown"
      >
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
  const mapRef = useRef(null); // Reference to the map container div
  const viewRef = useRef(null); // Reference to the MapView instance
  const markerGraphicRef = useRef(null); // Reference to the drone marker graphic
  const pathGraphicRef = useRef(null); // Reference to the polyline graphic
  const graphicsLayerRef = useRef(null); // Reference to the GraphicsLayer
  const polylineCoordsRef = useRef([]); // Holds an array of [lon, lat] points
  const GraphicModuleRef = useRef(null); // Will store the loaded Graphic module

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
        // Save the Graphic module for later updates
        GraphicModuleRef.current = Graphic;

        // Create the map with a "topo" basemap.
        const map = new Map({
          basemap: 'topo'
        });

        // Initialize the MapView.
        const view = new MapView({
          container: mapRef.current,
          map: map,
          center: [lon || 0, lat || 0],
          zoom: 15
        });
        viewRef.current = view;

        // Create and add a GraphicsLayer.
        const graphicsLayer = new GraphicsLayer();
        map.add(graphicsLayer);
        graphicsLayerRef.current = graphicsLayer;

        // Create the drone marker graphic.
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

        // Initialize the polyline coordinates with the starting point.
        polylineCoordsRef.current = [[lon, lat]];

        // Create the polyline graphic to show the drone’s path.
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

    // Cleanup: Destroy the MapView when the component unmounts.
    return () => {
      if (viewRef.current) {
        viewRef.current.destroy();
      }
    };
  }, []); // Run once on mount

  // Update the drone marker and polyline path whenever lat/lon changes.
  useEffect(() => {
    if (
      markerGraphicRef.current &&
      viewRef.current &&
      GraphicModuleRef.current &&
      graphicsLayerRef.current
    ) {
      // Update the drone marker position.
      markerGraphicRef.current.geometry = {
        type: 'point',
        longitude: lon,
        latitude: lat
      };

      // Calculate the distance from the last recorded point.
      const lastCoords =
        polylineCoordsRef.current[polylineCoordsRef.current.length - 1];
      const toRadians = (deg) => (deg * Math.PI) / 180;
      const latDiff = lat - lastCoords[1];
      const lonDiff = lon - lastCoords[0];
      // Approximate conversion: 1° latitude ~ 111320 meters, and for longitude scale with cos(latitude)
      const avgLat = (lat + lastCoords[1]) / 2;
      const metersPerDegLat = 111320;
      const metersPerDegLon = 111320 * Math.cos(toRadians(avgLat));
      const distance = Math.sqrt(
        (latDiff * metersPerDegLat) ** 2 + (lonDiff * metersPerDegLon) ** 2
      );

      // Set a threshold (e.g., 2 meters) to record a new point.
      const threshold = 2;
      if (distance >= threshold) {
        // Add the new coordinate to the polyline path.
        polylineCoordsRef.current.push([lon, lat]);
        // Update the polyline graphic geometry.
        pathGraphicRef.current.geometry = {
          type: 'polyline',
          paths: polylineCoordsRef.current
        };
      }

      // Optionally recenter the view.
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
    lat: -35.363262, // Drone latitude
    lon: 149.165237, // Drone longitude
    telemetry: {
      heading: 0,
      groundspeed: 0,
      airspeed: 0
    }
  });
  const [logs, setLogs] = useState([]);

  // Utility to add a log message with timestamp.
  const addLog = (message) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs((prev) => [...prev, `[${timestamp}] ${message}`]);
  };

  // Set up a WebSocket connection to receive drone telemetry.
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
      // Only log if a message is present (e.g., command responses)
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

  // Send control commands via WebSocket.
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
        const altitude = params[0] || 10;
        commandObj = { type: 'takeoff', altitude: altitude };
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
      <h1>Custom Ground Control Station</h1>
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
