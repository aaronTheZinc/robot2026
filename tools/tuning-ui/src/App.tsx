import { useMemo, useRef, useState } from 'react';
import './App.css';
import { connectPoseSubscription } from './lib/nt4Client';
import type { Pose2dSample } from './lib/nt4Client';
import { interpolateKNN } from './lib/interpolate';
import type { KNNConfig, SamplePoint } from './lib/interpolate';

const STORAGE_KEY = 'tuning-ui-samples-v1';

const parseNumber = (value: string) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
};

const loadSamples = (): SamplePoint[] => {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed.filter((item) => item && typeof item.id === 'string');
  } catch {
    return [];
  }
};

const formatNumber = (value: number, digits = 3) =>
  Number.isFinite(value) ? value.toFixed(digits) : '—';

function App() {
  const [ntUri, setNtUri] = useState('localhost');
  const [ntPort, setNtPort] = useState(5810);
  const [connectionState, setConnectionState] = useState<'disconnected' | 'connecting' | 'connected'>(
    'disconnected'
  );
  const [pose, setPose] = useState<Pose2dSample>({
    x: 0,
    y: 0,
    rotation: 0,
    timestampMs: 0,
  });
  const [manualValues, setManualValues] = useState({
    turret: 0,
    shooter: 0,
    hood: 0,
  });
  const [samples, setSamples] = useState<SamplePoint[]>(() => loadSamples());
  const [target, setTarget] = useState({ x: 0, y: 0, rotation: 0 });
  const [knnConfig, setKnnConfig] = useState<KNNConfig>({
    k: 3,
    weightX: 1,
    weightY: 1,
    weightRotation: 1,
    power: 1,
  });

  const disconnectRef = useRef<null | (() => void)>(null);

  const persistSamples = (nextSamples: SamplePoint[]) => {
    setSamples(nextSamples);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(nextSamples));
  };

  const handleConnect = () => {
    try {
      disconnectRef.current?.();
      setConnectionState('connecting');
      const subscription = connectPoseSubscription(ntUri, ntPort, {
        onPose: setPose,
        onConnection: (connected) =>
          setConnectionState(connected ? 'connected' : 'disconnected'),
      });
      disconnectRef.current = subscription.disconnect;
    } catch {
      setConnectionState('disconnected');
    }
  };

  const handleDisconnect = () => {
    disconnectRef.current?.();
    disconnectRef.current = null;
    setConnectionState('disconnected');
  };

  const recordSample = () => {
    const now = Date.now();
    const newSample: SamplePoint = {
      id: `${now}-${Math.random().toString(16).slice(2)}`,
      timestampMs: now,
      x: pose.x,
      y: pose.y,
      rotation: pose.rotation,
      turret: manualValues.turret,
      shooter: manualValues.shooter,
      hood: manualValues.hood,
    };
    persistSamples([newSample, ...samples]);
  };

  const updateSample = (id: string, field: keyof SamplePoint, value: number) => {
    persistSamples(
      samples.map((sample) => (sample.id === id ? { ...sample, [field]: value } : sample))
    );
  };

  const removeSample = (id: string) => {
    persistSamples(samples.filter((sample) => sample.id !== id));
  };

  const handleImport = (file: File | null) => {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const parsed = JSON.parse(String(reader.result));
        if (!Array.isArray(parsed)) return;
        const cleaned = parsed
          .filter((item) => item && typeof item.id === 'string')
          .map((item) => ({
            id: item.id,
            timestampMs: Number(item.timestampMs) || Date.now(),
            x: Number(item.x) || 0,
            y: Number(item.y) || 0,
            rotation: Number(item.rotation) || 0,
            turret: Number(item.turret) || 0,
            shooter: Number(item.shooter) || 0,
            hood: Number(item.hood) || 0,
          }));
        persistSamples(cleaned);
      } catch {
        return;
      }
    };
    reader.readAsText(file);
  };

  const handleExport = () => {
    const blob = new Blob([JSON.stringify(samples, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = 'tuning-samples.json';
    anchor.click();
    URL.revokeObjectURL(url);
  };

  const interpolated = useMemo(
    () => interpolateKNN(target, samples, knnConfig),
    [target, samples, knnConfig]
  );

  return (
    <div className="app">
      <header className="header">
        <div>
          <h1>Robot Tuning UI</h1>
          <p>Live pose from NT4, manual turret/shooter/hood logging, KNN interpolation.</p>
        </div>
        <div className={`status status-${connectionState}`}>
          {connectionState === 'connected' && 'Connected'}
          {connectionState === 'connecting' && 'Connecting'}
          {connectionState === 'disconnected' && 'Disconnected'}
        </div>
      </header>

      <section className="panel">
        <h2>NT4 Connection</h2>
        <div className="grid">
          <label>
            Robot URI
            <input
              value={ntUri}
              onChange={(event) => setNtUri(event.target.value)}
              placeholder="roborio-####-frc.local"
            />
          </label>
          <label>
            Port
            <input
              type="number"
              value={ntPort}
              onChange={(event) => setNtPort(parseNumber(event.target.value))}
            />
          </label>
          <div className="button-row">
            <button onClick={handleConnect}>Connect</button>
            <button className="secondary" onClick={handleDisconnect}>
              Disconnect
            </button>
          </div>
        </div>
      </section>

      <section className="panel">
        <h2>Live Pose (Pose/robotPose)</h2>
        <div className="grid">
          <div className="stat">
            <span>X</span>
            <strong>{formatNumber(pose.x)}</strong>
          </div>
          <div className="stat">
            <span>Y</span>
            <strong>{formatNumber(pose.y)}</strong>
          </div>
          <div className="stat">
            <span>Rotation (deg)</span>
            <strong>{formatNumber(pose.rotation)}</strong>
          </div>
          <div className="stat">
            <span>Timestamp</span>
            <strong>{pose.timestampMs ? new Date(pose.timestampMs).toLocaleTimeString() : '—'}</strong>
          </div>
        </div>
      </section>

      <section className="panel">
        <h2>Manual Shot Parameters</h2>
        <div className="grid">
          <label>
            Turret Rotation
            <input
              type="number"
              value={manualValues.turret}
              onChange={(event) =>
                setManualValues((prev) => ({ ...prev, turret: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Shooter Velocity
            <input
              type="number"
              value={manualValues.shooter}
              onChange={(event) =>
                setManualValues((prev) => ({ ...prev, shooter: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Hood Angle
            <input
              type="number"
              value={manualValues.hood}
              onChange={(event) =>
                setManualValues((prev) => ({ ...prev, hood: parseNumber(event.target.value) }))
              }
            />
          </label>
        </div>
        <div className="button-row">
          <button onClick={recordSample}>Record Sample</button>
          <button className="secondary" onClick={() => persistSamples([])}>
            Clear Samples
          </button>
        </div>
      </section>

      <section className="panel">
        <div className="panel-header">
          <h2>Samples</h2>
          <div className="panel-actions">
            <button className="secondary" onClick={handleExport}>
              Export JSON
            </button>
            <label className="file-input">
              Import JSON
              <input type="file" accept="application/json" onChange={(e) => handleImport(e.target.files?.[0] ?? null)} />
            </label>
          </div>
        </div>
        <div className="table">
          <div className="table-row table-header">
            <span>X</span>
            <span>Y</span>
            <span>Rot</span>
            <span>Turret</span>
            <span>Shooter</span>
            <span>Hood</span>
            <span>Action</span>
          </div>
          {samples.map((sample) => (
            <div className="table-row" key={sample.id}>
              <input
                type="number"
                value={sample.x}
                onChange={(event) => updateSample(sample.id, 'x', parseNumber(event.target.value))}
              />
              <input
                type="number"
                value={sample.y}
                onChange={(event) => updateSample(sample.id, 'y', parseNumber(event.target.value))}
              />
              <input
                type="number"
                value={sample.rotation}
                onChange={(event) =>
                  updateSample(sample.id, 'rotation', parseNumber(event.target.value))
                }
              />
              <input
                type="number"
                value={sample.turret}
                onChange={(event) =>
                  updateSample(sample.id, 'turret', parseNumber(event.target.value))
                }
              />
              <input
                type="number"
                value={sample.shooter}
                onChange={(event) =>
                  updateSample(sample.id, 'shooter', parseNumber(event.target.value))
                }
              />
              <input
                type="number"
                value={sample.hood}
                onChange={(event) =>
                  updateSample(sample.id, 'hood', parseNumber(event.target.value))
                }
              />
              <button className="danger" onClick={() => removeSample(sample.id)}>
                Delete
              </button>
            </div>
          ))}
          {samples.length === 0 && <div className="empty">No samples yet.</div>}
        </div>
      </section>

      <section className="panel">
        <h2>KNN Interpolation</h2>
        <div className="grid">
          <label>
            Target X
            <input
              type="number"
              value={target.x}
              onChange={(event) => setTarget((prev) => ({ ...prev, x: parseNumber(event.target.value) }))}
            />
          </label>
          <label>
            Target Y
            <input
              type="number"
              value={target.y}
              onChange={(event) => setTarget((prev) => ({ ...prev, y: parseNumber(event.target.value) }))}
            />
          </label>
          <label>
            Target Rotation
            <input
              type="number"
              value={target.rotation}
              onChange={(event) =>
                setTarget((prev) => ({ ...prev, rotation: parseNumber(event.target.value) }))
              }
            />
          </label>
        </div>
        <div className="grid">
          <label>
            K
            <input
              type="number"
              min={1}
              value={knnConfig.k}
              onChange={(event) =>
                setKnnConfig((prev) => ({ ...prev, k: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Weight X
            <input
              type="number"
              value={knnConfig.weightX}
              onChange={(event) =>
                setKnnConfig((prev) => ({ ...prev, weightX: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Weight Y
            <input
              type="number"
              value={knnConfig.weightY}
              onChange={(event) =>
                setKnnConfig((prev) => ({ ...prev, weightY: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Weight Rotation
            <input
              type="number"
              value={knnConfig.weightRotation}
              onChange={(event) =>
                setKnnConfig((prev) => ({ ...prev, weightRotation: parseNumber(event.target.value) }))
              }
            />
          </label>
          <label>
            Power
            <input
              type="number"
              value={knnConfig.power}
              onChange={(event) =>
                setKnnConfig((prev) => ({ ...prev, power: parseNumber(event.target.value) }))
              }
            />
          </label>
        </div>
        <div className="result">
          {interpolated ? (
            <>
              <div>
                <span>Turret</span>
                <strong>{formatNumber(interpolated.turret)}</strong>
              </div>
              <div>
                <span>Shooter</span>
                <strong>{formatNumber(interpolated.shooter)}</strong>
              </div>
              <div>
                <span>Hood</span>
                <strong>{formatNumber(interpolated.hood)}</strong>
              </div>
            </>
          ) : (
            <span className="empty">Add samples to see interpolation results.</span>
          )}
        </div>
      </section>
    </div>
  );
}

export default App;
