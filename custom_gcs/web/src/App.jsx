// src/App.jsx
import React, { useState, useEffect, useRef } from 'react';
import { Wifi } from 'lucide-react';
import { loadModules } from 'esri-loader';
import logo from './assets/GCS.png';

// =============== Component to Display Status for Multiple Drones =============== //
const DronesStatus = ({ drones }) => {
  return (
    <div className="status-panel">
      {Object.entries(drones).map(([droneId, state]) => (
        <div key={droneId} className="status-item">
          <Wifi className={state.connected ? 'icon-green' : 'icon-red'} />
          <span>Drone {droneId}: {state.connected ? 'Connected' : 'Disconnected'}</span>
          <div>Battery: {state.battery}%</div>
          <div className="mode-display">Mode: {state.mode}</div>
          <div>Alt: {state.alt}m</div>
        </div>
      ))}
    </div>
  );
};

// =============== Telemetry Data Component for a Selected Drone =============== //
const TelemetryData = ({ data, droneId }) => {
  return (
    <div className="telemetry-panel">
      <h2>Drone {droneId} Telemetry Data</h2>
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

// =============== Control Panel Component for Individual Drone =============== //
const IndividualControlPanel = ({ onCommand, selectedDrone, setSelectedDrone }) => {
  const [altitude, setAltitude] = useState(10);
  const [gotoLat, setGotoLat] = useState('');
  const [gotoLon, setGotoLon] = useState('');
  const [gotoAlt, setGotoAlt] = useState(10);

  return (
    <div className="control-panel">
      <h3>Control Individual Drone</h3>
      <div>
        <label>Select Drone: </label>
        <select value={selectedDrone} onChange={(e) => setSelectedDrone(e.target.value)}>
          {/* Assume drone IDs from 0 to 9 for simplicity */}
          {[...Array(10).keys()].map(id => (
            <option key={id} value={id}>{id}</option>
          ))}
        </select>
      </div>
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
      <button onClick={() => onCommand('arm', { target: selectedDrone, value: true })} className="btn btn-brown">
        Arm Drone {selectedDrone}
      </button>
      <button onClick={() => onCommand('takeoff', { target: selectedDrone, altitude })} className="btn btn-brown">
        Takeoff Drone {selectedDrone}
      </button>
      <button onClick={() => onCommand('rtl', { target: selectedDrone })} className="btn btn-brown">
        RTL Drone {selectedDrone}
      </button>
      <button onClick={() => onCommand('land', { target: selectedDrone })} className="btn btn-brown">
        Land Drone {selectedDrone}
      </button>
      <button onClick={() => onCommand('guided', { target: selectedDrone })} className="btn btn-brown">
        Guided Drone {selectedDrone}
      </button>
      <button onClick={() => onCommand('mission', { target: selectedDrone })} className="btn btn-brown">
        Mission Drone {selectedDrone}
      </button>
      <div className="goto-control" style={{ marginTop: '1rem' }}>
        <h4>Go To Coordinates</h4>
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
        <button
          onClick={() => onCommand('goto', { target: selectedDrone, lat: parseFloat(gotoLat), lon: parseFloat(gotoLon), alt: parseFloat(gotoAlt) })}
          className="btn btn-brown"
        >
          Go To (Drone {selectedDrone})
        </button>
      </div>
    </div>
  );
};

// =============== Control Panel Component for All Drones =============== //
const AllDronesControlPanel = ({ onCommand }) => {
  const [altitude, setAltitude] = useState(10);
  const [gotoLat, setGotoLat] = useState('');
  const [gotoLon, setGotoLon] = useState('');
  const [gotoAlt, setGotoAlt] = useState(10);

  return (
    <div className="control-panel" style={{ marginTop: '2rem' }}>
      <h3>Control All Drones</h3>
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
      <button onClick={() => onCommand('arm', { target: 'all', value: true })} className="btn btn-brown">
        Arm All
      </button>
      <button onClick={() => onCommand('takeoff', { target: 'all', altitude })} className="btn btn-brown">
        Takeoff All
      </button>
      <button onClick={() => onCommand('rtl', { target: 'all' })} className="btn btn-brown">
        RTL All
      </button>
      <button onClick={() => onCommand('land', { target: 'all' })} className="btn btn-brown">
        Land All
      </button>
      <button onClick={() => onCommand('guided', { target: 'all' })} className="btn btn-brown">
        Guided All
      </button>
      <button onClick={() => onCommand('mission', { target: 'all' })} className="btn btn-brown">
        Mission All
      </button>
      <div className="goto-control" style={{ marginTop: '1rem' }}>
        <h4>Go To Coordinates</h4>
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
        <button
          onClick={() => onCommand('goto', { target: 'all', lat: parseFloat(gotoLat), lon: parseFloat(gotoLon), alt: parseFloat(gotoAlt) })}
          className="btn btn-brown"
        >
          Go To (All Drones)
        </button>
      </div>
    </div>
  );
};

// =============== ArcGIS Map Component for Multiple Drones =============== //
const ArcgisMap = ({ drones }) => {
  const mapRef = useRef(null);
  const viewRef = useRef(null);
  const graphicsLayerRef = useRef(null);
  // Store marker and polyline graphics per drone
  const markersRef = useRef({});
  const polylinesRef = useRef({});
  const polylineCoordsRef = useRef({}); // drone_id -> array of [lon, lat]
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
        const map = new Map({ basemap: 'topo' });
        const view = new MapView({
          container: mapRef.current,
          map: map,
          center: [0, 0],
          zoom: 15
        });
        viewRef.current = view;
        const graphicsLayer = new GraphicsLayer();
        map.add(graphicsLayer);
        graphicsLayerRef.current = graphicsLayer;
      })
      .catch((err) => console.error('ArcGIS loadModules error: ', err));

    return () => {
      if (viewRef.current) {
        viewRef.current.destroy();
      }
    };
  }, []);

  useEffect(() => {
    if (!graphicsLayerRef.current || !GraphicModuleRef.current) return;
    const Graphic = GraphicModuleRef.current;
    // For each drone, update or create its marker and polyline.
    Object.entries(drones).forEach(([droneId, state]) => {
      const { lat, lon } = state;
      // Create or update marker for this drone
      if (!markersRef.current[droneId]) {
        const marker = new Graphic({
          geometry: { type: 'point', longitude: lon, latitude: lat },
          symbol: {
            type: 'simple-marker',
            color: [226, 119, 40],
            outline: { color: [255, 255, 255], width: 2 }
          }
        });
        graphicsLayerRef.current.add(marker);
        markersRef.current[droneId] = marker;
        // Initialize polyline coordinates for this drone
        polylineCoordsRef.current[droneId] = [[lon, lat]];
        const polyline = new Graphic({
          geometry: { type: 'polyline', paths: polylineCoordsRef.current[droneId] },
          symbol: { type: 'simple-line', color: [0, 0, 255], width: 2 }
        });
        graphicsLayerRef.current.add(polyline);
        polylinesRef.current[droneId] = polyline;
      } else {
        // Update marker position
        markersRef.current[droneId].geometry = { type: 'point', longitude: lon, latitude: lat };
        // Append new coordinate if moved enough
        const coords = polylineCoordsRef.current[droneId];
        const lastCoords = coords[coords.length - 1];
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
          coords.push([lon, lat]);
          polylinesRef.current[droneId].geometry = { type: 'polyline', paths: coords };
        }
      }
    });
    // Optionally, center the view on the first drone
    const firstDrone = Object.values(drones)[0];
    if (firstDrone && viewRef.current) {
      viewRef.current.center = [firstDrone.lon, firstDrone.lat];
    }
  }, [drones]);

  return <div style={{ height: '100%', width: '100%' }} ref={mapRef}></div>;
};

// =============== Main App Component =============== //
const App = () => {
  const [websocket, setWebsocket] = useState(null);
  // droneStates will be an object keyed by drone id
  const [droneStates, setDroneStates] = useState({});
  const [logs, setLogs] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState("0");

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
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "log" && data.message) {
        addLog(`[${data.level.toUpperCase()}] ${data.message}`);
      }
      // If message contains "drones", update droneStates; else assume single drone
      if (data.drones) {
        setDroneStates(data.drones);
      } else {
        setDroneStates({ 0: {
          connected: data.connected,
          battery: data.battery,
          mode: data.mode,
          alt: data.alt,
          lat: data.lat,
          lon: data.lon,
          heading: data.heading,
          groundspeed: data.groundspeed,
          airspeed: data.airspeed
        }});
      }
    };

    ws.onclose = () => {
      console.log('Disconnected from WebSocket server');
      addLog('Disconnected from WebSocket server');
    };

    return () => {
      if (ws) ws.close();
    };
  }, []);

  // Generalized command handler: accepts a command type and a payload (which includes a target)
  const handleCommand = (commandType, payload = {}) => {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      console.error('WebSocket not connected');
      addLog('WebSocket not connected');
      return;
    }
    let commandObj = { type: commandType, ...payload };
    addLog(`Sending command: ${JSON.stringify(commandObj)}`);
    websocket.send(JSON.stringify(commandObj));
  };

  return (
    <div className="container" style={{ height: '100vh', width: '100vw' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', marginBottom: '1rem' }}>
        <img src={logo} alt="Logo" style={{ height: '50px', marginRight: '1rem' }} />
        <h1>Custom Ground Control Station</h1>
      </div>
      <DronesStatus drones={droneStates} />

      <div className="main-content" style={{ display: 'flex', gap: '1rem', height: 'calc(100% - 100px)' }}>
        <div className="map-container" style={{ height: '100%', flex: 2 }}>
          <ArcgisMap drones={droneStates} />
        </div>
        <div className="side-panel" style={{ flex: 1, overflowY: 'auto' }}>
          {/* Show telemetry for the selected individual drone */}
          {droneStates[selectedDrone] && (
            <TelemetryData data={droneStates[selectedDrone]} droneId={selectedDrone} />
          )}
          <IndividualControlPanel onCommand={handleCommand} selectedDrone={selectedDrone} setSelectedDrone={setSelectedDrone} />
          <AllDronesControlPanel onCommand={handleCommand} />
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
        </div>
      </div>
    </div>
  );
};

export default App;
