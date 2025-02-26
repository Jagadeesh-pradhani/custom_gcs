import React, { useState, useEffect, useRef } from 'react';
import { Wifi, MapPin, Navigation } from 'lucide-react';
import { loadModules } from 'esri-loader';

// =================== Component Styles =================== //
const styles = {
  container: {
    height: '100vh',
    width: '100vw',
    padding: '1rem',
    backgroundColor: '#f0f2f5',
  },
  statusPanel: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '1rem',
    backgroundColor: 'white',
    borderRadius: '8px',
    marginBottom: '1rem',
    boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
  },
  controlPanel: {
    backgroundColor: 'white',
    padding: '1.5rem',
    borderRadius: '8px',
    boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
  },
  telemetryPanel: {
    backgroundColor: 'white',
    padding: '1.5rem',
    borderRadius: '8px',
    marginBottom: '1rem',
    boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
  },
  gotoSection: {
    margin: '1.5rem 0',
    padding: '1rem',
    border: '1px solid #e8e8e8',
    borderRadius: '8px',
  },
  inputGroup: {
    marginBottom: '1rem',
  },
  logPanel: {
    maxHeight: '200px',
    overflowY: 'auto',
    backgroundColor: '#f8f9fa',
    padding: '1rem',
    borderRadius: '8px',
    border: '1px solid #dee2e6',
  },
};

const inputStyle = {
  width: '100%',
  padding: '0.5rem',
  border: '1px solid #d9d9d9',
  borderRadius: '4px',
  marginTop: '0.25rem',
};

const Button = ({ children, ...props }) => (
  <button
    {...props}
    style={{
      padding: '0.5rem 1rem',
      backgroundColor: '#1890ff',
      color: 'white',
      border: 'none',
      borderRadius: '4px',
      cursor: 'pointer',
      transition: 'background-color 0.3s',
      ...props.style,
    }}
    onMouseOver={(e) => (e.target.style.backgroundColor = '#40a9ff')}
    onMouseOut={(e) => (e.target.style.backgroundColor = '#1890ff')}
  >
    {children}
  </button>
);

// =================== Drone Status Component =================== //
const DroneStatus = ({ connected, battery, mode, altitude }) => (
  <div style={styles.statusPanel}>
    <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
      <Wifi size={24} color={connected ? '#52c41a' : '#ff4d4f'} />
      <span style={{ fontWeight: 500 }}>
        {connected ? 'CONNECTED' : 'DISCONNECTED'}
      </span>
    </div>
    <div style={{ display: 'flex', gap: '2rem' }}>
      <div>
        <span style={{ color: '#595959' }}>Battery</span>
        <div style={{ fontSize: '1.25rem', fontWeight: 600 }}>{battery}%</div>
      </div>
      <div>
        <span style={{ color: '#595959' }}>Mode</span>
        <div style={{ fontSize: '1.25rem', fontWeight: 600 }}>{mode}</div>
      </div>
      <div>
        <span style={{ color: '#595959' }}>Altitude</span>
        <div style={{ fontSize: '1.25rem', fontWeight: 600 }}>{altitude}m</div>
      </div>
    </div>
  </div>
);

// =================== Telemetry Display Component =================== //
const TelemetryData = ({ data }) => (
  <div style={styles.telemetryPanel}>
    <h2 style={{ marginBottom: '1rem', color: '#262626' }}>Flight Telemetry</h2>
    <div
      style={{
        display: 'grid',
        gridTemplateColumns: 'repeat(3, 1fr)',
        gap: '1rem',
      }}
    >
      <MetricCard
        icon={<Navigation />}
        title="Heading"
        value={`${data.heading}°`}
      />
      <MetricCard
        icon={<Navigation />}
        title="Ground Speed"
        value={`${data.groundspeed} m/s`}
      />
      <MetricCard
        icon={<Navigation />}
        title="Air Speed"
        value={`${data.airspeed} m/s`}
      />
    </div>
  </div>
);

const MetricCard = ({ icon, title, value }) => (
  <div
    style={{
      display: 'flex',
      alignItems: 'center',
      gap: '0.5rem',
      padding: '1rem',
      backgroundColor: '#f8f9fa',
      borderRadius: '8px',
    }}
  >
    <div style={{ color: '#1890ff' }}>{icon}</div>
    <div>
      <div style={{ color: '#595959', fontSize: '0.9rem' }}>{title}</div>
      <div style={{ fontWeight: 600 }}>{value}</div>
    </div>
  </div>
);

// =================== Control Panel Component =================== //
const ControlPanel = ({ onCommand }) => {
  const [altitude, setAltitude] = useState(10);
  const [gotoCoords, setGotoCoords] = useState({ lat: '', lon: '', alt: 10 });

  const handleGoto = () => {
    const lat = parseFloat(gotoCoords.lat);
    const lon = parseFloat(gotoCoords.lon);
    const alt = parseFloat(gotoCoords.alt);

    if (!validateCoordinates(lat, lon, alt)) return;

    // Pass an object for the 'goto' command
    onCommand('goto', { lat, lon, alt });
    setGotoCoords({ lat: '', lon: '', alt: 10 });
  };

  const validateCoordinates = (lat, lon, alt) => {
    if ([lat, lon, alt].some(isNaN)) {
      alert('Please enter valid numbers in all fields');
      return false;
    }
    if (lat < -90 || lat > 90) {
      alert('Latitude must be between -90 and 90 degrees');
      return false;
    }
    if (lon < -180 || lon > 180) {
      alert('Longitude must be between -180 and 180 degrees');
      return false;
    }
    if (alt < 0) {
      alert('Altitude cannot be negative');
      return false;
    }
    return true;
  };

  return (
    <div style={styles.controlPanel}>
      <h2 style={{ marginBottom: '1.5rem', color: '#262626' }}>
        Flight Controls
      </h2>

      <div style={{ marginBottom: '1.5rem' }}>
        <div style={styles.inputGroup}>
          <label>Takeoff Altitude (meters)</label>
          <input
            type="number"
            value={altitude}
            onChange={(e) =>
              setAltitude(Math.max(1, parseInt(e.target.value)))
            }
            style={inputStyle}
          />
        </div>
        <div
          style={{
            display: 'grid',
            gridTemplateColumns: 'repeat(2, 1fr)',
            gap: '1rem',
          }}
        >
          <Button onClick={() => onCommand('arm')}>Arm Drone</Button>
          <Button onClick={() => onCommand('takeoff', altitude)}>
            Takeoff
          </Button>
        </div>
      </div>

      <div style={styles.gotoSection}>
        <h3 style={{ marginBottom: '1rem', color: '#262626' }}>
          <MapPin size={18} style={{ marginRight: '0.5rem' }} />
          Navigate To Position
        </h3>

        <div style={styles.inputGroup}>
          <label>Latitude</label>
          <input
            type="number"
            step="any"
            value={gotoCoords.lat}
            onChange={(e) =>
              setGotoCoords((p) => ({ ...p, lat: e.target.value }))
            }
            placeholder="Enter latitude (-90 to 90)"
            style={inputStyle}
          />
        </div>

        <div style={styles.inputGroup}>
          <label>Longitude</label>
          <input
            type="number"
            step="any"
            value={gotoCoords.lon}
            onChange={(e) =>
              setGotoCoords((p) => ({ ...p, lon: e.target.value }))
            }
            placeholder="Enter longitude (-180 to 180)"
            style={inputStyle}
          />
        </div>

        <div style={styles.inputGroup}>
          <label>Altitude (meters)</label>
          <input
            type="number"
            value={gotoCoords.alt}
            onChange={(e) =>
              setGotoCoords((p) => ({ ...p, alt: e.target.value }))
            }
            style={inputStyle}
          />
        </div>

        <Button onClick={handleGoto} style={{ width: '100%' }}>
          Execute GoTo
        </Button>
      </div>

      <div
        style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(3, 1fr)',
          gap: '1rem',
        }}
      >
        <Button onClick={() => onCommand('rtl')}>Return Home</Button>
        <Button onClick={() => onCommand('land')}>Land Now</Button>
        <Button onClick={() => onCommand('guided')}>Guided Mode</Button>
      </div>
    </div>
  );
};

// =================== Map Component =================== //
const ArcgisMap = ({ lat, lon }) => {
  const mapRef = useRef(null);
  const viewRef = useRef(null);
  const markerRef = useRef(null);
  const pathLayerRef = useRef(null);
  const pathCoords = useRef([]);
  const graphicRef = useRef(null);

  useEffect(() => {
    let cleanup;

    loadModules([
      'esri/Map',
      'esri/views/MapView',
      'esri/Graphic',
      'esri/layers/GraphicsLayer',
    ]).then(([Map, MapView, Graphic, GraphicsLayer]) => {
      const map = new Map({ basemap: 'satellite' });
      const view = new MapView({
        container: mapRef.current,
        map: map,
        center: [lon || 0, lat || 0],
        zoom: 16,
      });

      // Setup graphics layers
      const graphicsLayer = new GraphicsLayer();
      map.add(graphicsLayer);

      // Initial marker
      const marker = new Graphic({
        geometry: { type: 'point', longitude: lon, latitude: lat },
        symbol: {
          type: 'simple-marker',
          color: [255, 0, 0],
          size: '12px',
          outline: { color: [255, 255, 255], width: 2 },
        },
      });
      graphicsLayer.add(marker);

      // Path layer
      const pathLayer = new GraphicsLayer();
      map.add(pathLayer);
      pathCoords.current = [[lon, lat]];

      // Store references
      viewRef.current = view;
      markerRef.current = marker;
      pathLayerRef.current = pathLayer;
      graphicRef.current = Graphic;

      cleanup = () => view.destroy();
    });

    return () => cleanup && cleanup();
  }, []);

  useEffect(() => {
    if (!viewRef.current || !markerRef.current || !graphicRef.current) return;

    // Update marker position
    markerRef.current.geometry = {
      type: 'point',
      longitude: lon,
      latitude: lat,
    };

    // Update path
    pathCoords.current.push([lon, lat]);
    pathLayerRef.current.removeAll();
    pathLayerRef.current.add(
      new graphicRef.current({
        geometry: { type: 'polyline', paths: pathCoords.current },
        symbol: {
          type: 'simple-line',
          color: [3, 133, 255],
          width: 3,
        },
      })
    );

    // Update view center
    viewRef.current.center = [lon, lat];
  }, [lat, lon]);

  return (
    <div
      style={{ height: '100%', borderRadius: '8px', overflow: 'hidden' }}
      ref={mapRef}
    />
  );
};

// =================== Main App Component =================== //
const App = () => {
  const [websocket, setWebsocket] = useState(null);
  const [droneState, setDroneState] = useState({
    connected: false,
    battery: 0,
    mode: 'INIT',
    altitude: 0,
    lat: -35.363262,
    lon: 149.165237,
    telemetry: { heading: 0, groundspeed: 0, airspeed: 0 },
  });
  const [logs, setLogs] = useState([]);

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8765');
    setWebsocket(ws);

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'log') {
        setLogs((prev) => [
          ...prev.slice(-50),
          `${new Date().toLocaleTimeString()}: ${data.message}`,
        ]);
      } else {
        setDroneState((prev) => ({
          ...prev,
          ...data,
          telemetry: {
            heading: data.heading || prev.telemetry.heading,
            groundspeed: data.groundspeed || prev.telemetry.groundspeed,
            airspeed: data.airspeed || prev.telemetry.airspeed,
          },
        }));
      }
    };

    return () => ws.close();
  }, []);

  const handleCommand = (command, ...params) => {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) return;

    const commands = {
      arm: { type: 'arm', value: true },
      takeoff: { type: 'takeoff', altitude: params[0] },
      rtl: { type: 'mode', value: 'RTL' },
      land: { type: 'mode', value: 'LAND' },
      guided: { type: 'mode', value: 'GUIDED' },
      goto: { type: 'goto', ...params[0] },
    };

    websocket.send(JSON.stringify(commands[command]));
  };

  return (
    <div style={styles.container}>
      <h1 style={{ marginBottom: '1rem', color: '#262626' }}>
        Drone Mission Control
      </h1>

      <DroneStatus
        connected={droneState.connected}
        battery={droneState.battery}
        mode={droneState.mode}
        altitude={droneState.altitude}
      />

      <div
        style={{
          display: 'grid',
          gridTemplateColumns: '2fr 1fr',
          gap: '1rem',
          height: 'calc(100vh - 160px)',
        }}
      >
        <ArcgisMap lat={droneState.lat} lon={droneState.lon} />

        <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
          <TelemetryData data={droneState.telemetry} />
          <ControlPanel onCommand={handleCommand} />

          <div style={styles.logPanel}>
            <h3 style={{ marginBottom: '0.5rem' }}>System Logs</h3>
            {logs.map((log, i) => (
              <div
                key={i}
                style={{
                  padding: '0.25rem 0',
                  borderBottom: '1px solid #eee',
                  fontSize: '0.9rem',
                }}
              >
                {log}
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default App;
