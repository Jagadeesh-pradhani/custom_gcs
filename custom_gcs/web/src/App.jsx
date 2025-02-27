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
const ControlPanel = ({ onCommand, gotoCoordinates, setGotoCoordinates }) => {
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

      {/* New Goto Controls */}
      <div className="goto-control" style={{ marginTop: '1rem' }}>
        <h3>Go To Coordinates</h3>
        <div>
          <label>Target Latitude: </label>
          <input
            type="number"
            value={gotoCoordinates.lat}
            onChange={(e) =>
              setGotoCoordinates({ ...gotoCoordinates, lat: e.target.value })
            }
            placeholder="e.g., -35.363262"
          />
        </div>
        <div>
          <label>Target Longitude: </label>
          <input
            type="number"
            value={gotoCoordinates.lon}
            onChange={(e) =>
              setGotoCoordinates({ ...gotoCoordinates, lon: e.target.value })
            }
            placeholder="e.g., 149.165237"
          />
        </div>
        <div>
          <label>Target Altitude (m): </label>
          <input
            type="number"
            value={gotoCoordinates.alt}
            onChange={(e) =>
              setGotoCoordinates({ ...gotoCoordinates, alt: e.target.value })
            }
          />
        </div>
        <button
          onClick={() =>
            onCommand(
              'goto',
              gotoCoordinates.lat,
              gotoCoordinates.lon,
              gotoCoordinates.alt
            )
          }
          className="btn btn-brown"
        >
          Go To
        </button>
      </div>
    </div>
  );
};

// =================== ArcGIS Map Component with Path Tracking =================== //
const ArcgisMap = ({ lat, lon, onMapRightClick }) => {
  const mapRef = useRef(null);           // Map container reference
  const viewRef = useRef(null);          // MapView instance reference
  const markerGraphicRef = useRef(null); // Drone marker graphic reference
  const pathGraphicRef = useRef(null);   // Polyline graphic reference
  const graphicsLayerRef = useRef(null); // GraphicsLayer reference
  const polylineCoordsRef = useRef([]);  // Holds an array of [lon, lat] points
  const GraphicModuleRef = useRef(null); // Loaded Graphic module

  // State for custom context menu
  const [contextMenu, setContextMenu] = useState({
    visible: false,
    x: 0,
    y: 0,
    lat: 0,
    lon: 0
  });

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

        // Add a GraphicsLayer for the drone marker and path
        const graphicsLayer = new GraphicsLayer();
        map.add(graphicsLayer);
        graphicsLayerRef.current = graphicsLayer;

        // Create the drone marker using a picture marker symbol.
        const markerGraphic = new Graphic({
          geometry: {
            type: 'point',
            longitude: lon || 0,
            latitude: lat || 0
          },
          // symbol: {
          //   type: 'picture-marker',
          //   // Replace the URL below with any drone icon image of your choice.
          //   url: 'https://img.icons8.com/color/48/000000/drone.png',
          //   width: '32px',
          //   height: '32px'
          // }
          symbol: {
            type: "simple-marker",
            color: [50, 205, 50],  // Lime green
            outline: {
              color: [255, 255, 255],
              width: 2
            },
            size: "20px",
            style: "triangle"
          }
        });
        graphicsLayer.add(markerGraphic);
        markerGraphicRef.current = markerGraphic;

        // Initialize the polyline with the starting point.
        polylineCoordsRef.current = [[lon, lat]];

        // Create the polyline graphic to display the drone’s path.
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

        // Add a right-click (contextmenu) listener on the view container.
        view.when(() => {
          view.container.addEventListener('contextmenu', (event) => {
            event.preventDefault(); // Prevent default browser menu

            // Convert the screen point to map coordinates.
            const screenPoint = { x: event.clientX, y: event.clientY };
            const mapPoint = view.toMap(screenPoint);

            // Set context menu state.
            setContextMenu({
              visible: true,
              x: event.clientX,
              y: event.clientY,
              lat: mapPoint.latitude,
              lon: mapPoint.longitude
            });
          });
        });
      })
      .catch(err => console.error('ArcGIS loadModules error: ', err));

    // Cleanup: Destroy the MapView on unmount.
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
      // Update marker position.
      markerGraphicRef.current.geometry = {
        type: 'point',
        longitude: lon,
        latitude: lat
      };

      // Update the path if the movement exceeds a threshold.
      const lastCoords = polylineCoordsRef.current[polylineCoordsRef.current.length - 1];
      const toRadians = (deg) => (deg * Math.PI) / 180;
      const latDiff = lat - lastCoords[1];
      const lonDiff = lon - lastCoords[0];
      const avgLat = (lat + lastCoords[1]) / 2;
      const metersPerDegLat = 111320;
      const metersPerDegLon = 111320 * Math.cos(toRadians(avgLat));
      const distance = Math.sqrt(
        (latDiff * metersPerDegLat) ** 2 + (lonDiff * metersPerDegLon) ** 2
      );

      const threshold = 2; // in meters
      if (distance >= threshold) {
        polylineCoordsRef.current.push([lon, lat]);
        pathGraphicRef.current.geometry = {
          type: 'polyline',
          paths: polylineCoordsRef.current
        };
      }

      // Optionally recenter the view.
      viewRef.current.center = [lon, lat];
    }
  }, [lat, lon]);

  return (
    <div style={{ height: '100%', width: '100%', position: 'relative' }} ref={mapRef}>
      {/* Render the custom context menu if visible */}
      {contextMenu.visible && (
        <div
          style={{
            position: 'absolute',
            top: contextMenu.y,
            left: contextMenu.x,
            background: 'white',
            border: '1px solid #ccc',
            padding: '5px',
            zIndex: 1000,
            cursor: 'pointer'
          }}
          onClick={() => {
            // Call the parent's callback with the selected coordinates.
            if (onMapRightClick) {
              onMapRightClick(contextMenu.lat, contextMenu.lon);
            }
            setContextMenu({ ...contextMenu, visible: false });
          }}
        >
          <button style={{ cursor: 'pointer' }}>Set goto coordinates</button>
        </div>
      )}
    </div>
  );
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

  // State to hold goto coordinates.
  const [gotoCoordinates, setGotoCoordinates] = useState({
    lat: '',
    lon: '',
    alt: 10
  });

  // Callback to update goto coordinates from the map's right-click.
  const handleMapRightClick = (lat, lon) => {
    setGotoCoordinates({ ...gotoCoordinates, lat, lon });
  };

  // WebSocket connection for drone telemetry.
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
      setDroneState(prev => ({ ...prev, connected: false }));
    };

    return () => {
      if (ws) ws.close();
    };
  }, []);

  // Send control commands via WebSocket.
  const handleCommand = (commandType, ...params) => {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      console.error('WebSocket not connected');
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
      case 'goto': {
          const targetLat = params[0];
          const targetLon = params[1];
          const targetAlt = params[2];
          commandObj = {
            type: 'goto',
            lat: parseFloat(targetLat),
            lon: parseFloat(targetLon),
            alt: parseFloat(targetAlt)
          };
          break;
        }
      default:
        console.error('Unknown command:', commandType);
        return;
    }

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

      <div className="main-content" style={{ display: 'flex', gap: '1rem', height: 'calc(100% - 100px)' }}>
        <div className="map-container" style={{ height: '100%', flex: 2 }}>
          <ArcgisMap
            lat={droneState.lat}
            lon={droneState.lon}
            onMapRightClick={handleMapRightClick}
          />
        </div>
        <div className="side-panel" style={{ flex: 1 }}>
          <TelemetryData data={droneState.telemetry} />
          <ControlPanel
            onCommand={handleCommand}
            gotoCoordinates={gotoCoordinates}
            setGotoCoordinates={setGotoCoordinates}
          />
        </div>
      </div>
    </div>
  );
};

export default App;
