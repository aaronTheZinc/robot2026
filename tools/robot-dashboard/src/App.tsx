import { useEffect, useMemo, useRef, useState } from 'react';

import './App.css';
import CameraView from './components/CameraView';
import ChassisView from './components/ChassisView';
import KnnGridView from './components/KnnGridView';
import MotorTestPanel from './components/MotorTestPanel';
import MotorTestView from './components/MotorTestView';
import AutoDebugPanel from './components/AutoDebugPanel';
import NetworkTableView from './components/NetworkTableView';
import SimulationView from './components/SimulationView';
import TurretView from './components/TurretView';
import ConnectionScreen from './screens/ConnectionScreen';
import type { ConnectionConfig, DashboardMode } from './screens/ConnectionScreen';
import {
  connectRobotStateSubscription,
  createMotorTestPublisher,
  createShooterSetpointPublisher,
} from './lib/nt4Client';
import type { MotorTestPublisher, ShooterSetpointPublisher } from './lib/nt4Client';
import {
  applyRobotStateUpdate,
  createInitialRobotState,
  nextMockState,
} from './lib/robotState';
import type { RobotMode, RobotStateUpdate } from './lib/robotState';
import type { KnnPoint } from './lib/knnInference';

const STORAGE_KEY_URI = 'robot-dashboard-uri';
const STORAGE_KEY_PORT = 'robot-dashboard-port';
const STORAGE_KEY_DASHBOARD_MODE = 'robot-dashboard-mode';
const STORAGE_KEY_KNN_MAP = 'robot-dashboard-knn-map-v1';

export type ViewTab =
  | 'dashboard'
  | 'turret'
  | 'chassis'
  | 'motortest'
  | 'networktables'
  | 'autodebug'
  | 'knngrid'
  | 'simulation';

const DEFAULT_URI = 'localhost';
const DEFAULT_PORT = 5810;

const DEBUG_TABS: ViewTab[] = [
  'dashboard',
  'turret',
  'chassis',
  'motortest',
  'simulation',
  'networktables',
  'autodebug',
  'knngrid',
];
const COMPETITION_TABS: ViewTab[] = ['dashboard', 'turret', 'simulation', 'motortest'];
const FIELD_LENGTH_M = 16.46;
const FIELD_WIDTH_M = 8.23;
const CURRENT_HISTORY_LIMIT = 90;

type CurrentHistory = {
  hood: number[];
  left: number[];
  right: number[];
};

function pushHistory(values: number[], nextValue: number): number[] {
  const clampedValue = Number.isFinite(nextValue) ? Math.max(0, nextValue) : 0;
  const next = [...values, clampedValue];
  return next.length > CURRENT_HISTORY_LIMIT ? next.slice(next.length - CURRENT_HISTORY_LIMIT) : next;
}

function buildSparklinePoints(values: number[], width: number, height: number, maxValue: number): string {
  if (values.length === 0) {
    return '';
  }
  const denominator = Math.max(values.length - 1, 1);
  return values
    .map((value, index) => {
      const x = (index / denominator) * width;
      const y = height - (Math.max(0, value) / Math.max(maxValue, 1)) * height;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(' ');
}

function formatNumber(value: number, digits = 1): string {
  return value.toFixed(digits);
}

function formatSeconds(value: number): string {
  return `${Math.max(0, Math.round(value))}s`;
}

function normalizeKnnPoints(value: unknown): KnnPoint[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .filter(
      (item: unknown): item is { x: number; y: number } =>
        item != null &&
        typeof item === 'object' &&
        typeof (item as { x?: unknown }).x === 'number' &&
        typeof (item as { y?: unknown }).y === 'number'
    )
    .map((item) => {
      const record = item as Record<string, unknown>;
      return {
        x: item.x,
        y: item.y,
        headingDeg:
          typeof record.headingDeg === 'number'
            ? record.headingDeg
            : typeof record.rotation === 'number'
              ? record.rotation
              : 0,
        shooterRpm:
          typeof record.shooterRpm === 'number'
            ? record.shooterRpm
            : typeof record.shooter === 'number'
              ? record.shooter
              : 0,
        hoodDeg:
          typeof record.hoodDeg === 'number'
            ? record.hoodDeg
            : typeof record.hood === 'number'
              ? record.hood
              : 0,
      };
    });
}

function loadStoredKnnMap(): KnnPoint[] {
  try {
    const raw = localStorage.getItem(STORAGE_KEY_KNN_MAP);
    if (!raw) {
      return [];
    }
    return normalizeKnnPoints(JSON.parse(raw));
  } catch {
    return [];
  }
}

function downloadJson(filename: string, data: unknown) {
  const blob = new Blob([JSON.stringify(data, null, 2)], {
    type: 'application/json',
  });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function CurrentTrace({
  label,
  color,
  value,
  values,
  maxValue,
}: {
  label: string;
  color: string;
  value: number;
  values: number[];
  maxValue: number;
}) {
  const width = 220;
  const height = 54;
  const points = buildSparklinePoints(values, width, height, maxValue);

  return (
    <div className="current-trace">
      <div className="current-trace-header">
        <span className="current-trace-label">
          <span className="current-trace-swatch" style={{ backgroundColor: color }} />
          {label}
        </span>
        <span className="current-trace-value">{formatNumber(value, 1)} A</span>
      </div>
      <svg className="current-trace-chart" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
        <polyline className="current-trace-grid" points={`0,${height - 1} ${width},${height - 1}`} />
        {points && (
          <polyline
            points={points}
            fill="none"
            stroke={color}
            strokeWidth="2.5"
            strokeLinejoin="round"
            strokeLinecap="round"
          />
        )}
      </svg>
    </div>
  );
}

function getStoredUri(): string {
  try {
    const v = localStorage.getItem(STORAGE_KEY_URI);
    return v ?? DEFAULT_URI;
  } catch {
    return DEFAULT_URI;
  }
}

function getStoredPort(): number {
  try {
    const v = localStorage.getItem(STORAGE_KEY_PORT);
    return v ? Number(v) || DEFAULT_PORT : DEFAULT_PORT;
  } catch {
    return DEFAULT_PORT;
  }
}

function getStoredDashboardMode(): DashboardMode {
  try {
    const v = localStorage.getItem(STORAGE_KEY_DASHBOARD_MODE);
    return v === 'competition' || v === 'debug' ? v : 'debug';
  } catch {
    return 'debug';
  }
}

function App() {
  const [connectionScreenDone, setConnectionScreenDone] = useState(false);
  const [dashboardMode, setDashboardMode] = useState<DashboardMode>(() => getStoredDashboardMode());
  const [mode, setMode] = useState<RobotMode>('mock');
  const [nt4Enabled, setNt4Enabled] = useState(false);
  const [uri, setUri] = useState(DEFAULT_URI);
  const [port, setPort] = useState(DEFAULT_PORT);
  const [viewTab, setViewTab] = useState<ViewTab>('dashboard');
  const [state, setState] = useState(() => createInitialRobotState('mock'));
  const robotDims = { lengthM: 0.9, widthM: 0.8 };
  const [loggedPoints, setLoggedPoints] = useState<KnnPoint[]>(() => loadStoredKnnMap());
  const [knnDraftPoint, setKnnDraftPoint] = useState<KnnPoint>({
    x: 0,
    y: 0,
    headingDeg: 0,
    shooterRpm: 0,
    hoodDeg: 0,
  });
  const [shooterCurrentHistory, setShooterCurrentHistory] = useState<CurrentHistory>({
    hood: [0],
    left: [0],
    right: [0],
  });
  const [milestoneFlash, setMilestoneFlash] = useState<'flash-30' | 'flash-15' | 'flash-10' | ''>('');
  const lastRemainingRef = useRef(state.match.timeRemainingSec);
  const flashTimeoutRef = useRef<number | null>(null);
  const knnLogInputRef = useRef<HTMLInputElement | null>(null);
  const motorTestPublisherRef = useRef<MotorTestPublisher | null>(null);
  const shooterSetpointPublisherRef = useRef<ShooterSetpointPublisher | null>(null);

  const visibleTabs =
    dashboardMode === 'competition' ? COMPETITION_TABS : DEBUG_TABS;
  const isDebug = dashboardMode === 'debug';

  useEffect(() => {
    if (connectionScreenDone && !visibleTabs.includes(viewTab)) {
      setViewTab('dashboard');
    }
  }, [connectionScreenDone, visibleTabs, viewTab]);

  useEffect(() => {
    if (connectionScreenDone) {
      try {
        localStorage.setItem(STORAGE_KEY_DASHBOARD_MODE, dashboardMode);
      } catch {
        /* ignore */
      }
    }
  }, [connectionScreenDone, dashboardMode]);

  const handleEnterDashboard = (config: ConnectionConfig, nextMode: DashboardMode) => {
    setMode(config.mode);
    setUri(config.uri);
    setPort(config.port);
    setNt4Enabled(config.nt4Enabled);
    setDashboardMode(nextMode);
    setConnectionScreenDone(true);
    try {
      localStorage.setItem(STORAGE_KEY_URI, config.uri);
      localStorage.setItem(STORAGE_KEY_PORT, String(config.port));
      localStorage.setItem(STORAGE_KEY_DASHBOARD_MODE, nextMode);
    } catch {
      /* ignore */
    }
  };

  const handleBackToConnection = () => {
    setConnectionScreenDone(false);
  };

  const lastUpdateLabel = useMemo(() => {
    const date = new Date(state.lastUpdateMs);
    return date.toLocaleTimeString();
  }, [state.lastUpdateMs]);

  const applyUpdate = (update: RobotStateUpdate) => {
    setState((prev) => applyRobotStateUpdate(prev, update, Date.now()));
  };

  const persistKnnMap = (points: KnnPoint[]) => {
    setLoggedPoints(points);
    try {
      localStorage.setItem(STORAGE_KEY_KNN_MAP, JSON.stringify(points));
    } catch {
      /* ignore */
    }
  };

  const loadKnnLog = (file: File | null) => {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const points = normalizeKnnPoints(JSON.parse(String(reader.result)));
        persistKnnMap(points);
      } catch {
        persistKnnMap([]);
      }
    };
    reader.readAsText(file);
  };

  const addKnnPoint = (point: KnnPoint) => {
    persistKnnMap([
      ...loggedPoints,
      {
        x: Number.isFinite(point.x) ? point.x : 0,
        y: Number.isFinite(point.y) ? point.y : 0,
        headingDeg: Number.isFinite(point.headingDeg) ? point.headingDeg : 0,
        shooterRpm: Number.isFinite(point.shooterRpm) ? point.shooterRpm : 0,
        hoodDeg: Number.isFinite(point.hoodDeg) ? point.hoodDeg : 0,
      },
    ]);
  };

  const updateKnnPoint = (index: number, field: keyof KnnPoint, value: number) => {
    persistKnnMap(
      loggedPoints.map((point, pointIndex) =>
        pointIndex === index ? { ...point, [field]: Number.isFinite(value) ? value : 0 } : point
      )
    );
  };

  const removeKnnPoint = (index: number) => {
    persistKnnMap(loggedPoints.filter((_, pointIndex) => pointIndex !== index));
  };

  const clearKnnPoints = () => {
    persistKnnMap([]);
  };

  const addCurrentPoseToKnnMap = () => {
    addKnnPoint({
      x: state.swerve.x,
      y: state.swerve.y,
      headingDeg: state.swerve.headingDeg,
      shooterRpm: state.shooter.rpmSetpoint,
      hoodDeg: state.shooter.hoodSetpoint,
    });
  };

  const copyCurrentStateToDraft = () => {
    setKnnDraftPoint({
      x: state.swerve.x,
      y: state.swerve.y,
      headingDeg: state.swerve.headingDeg,
      shooterRpm: state.shooter.rpmSetpoint,
      hoodDeg: state.shooter.hoodSetpoint,
    });
  };

  const exportKnnMap = () => {
    downloadJson('knn_map.json', loggedPoints);
  };

  useEffect(() => {
    setState((prev) => applyRobotStateUpdate(prev, { mode }, Date.now()));
    if (mode === 'mock') {
      setState((prev) => applyRobotStateUpdate(prev, { connected: false }, Date.now()));
      setNt4Enabled(false);
    }
  }, [mode]);

  useEffect(() => {
    if (mode !== 'mock') {
      return;
    }
    const interval = window.setInterval(() => {
      setState((prev) => nextMockState(prev));
    }, 250);
    return () => window.clearInterval(interval);
  }, [mode]);

  useEffect(() => {
    if (mode !== 'nt4' || !nt4Enabled) {
      return;
    }
    const subscription = connectRobotStateSubscription(uri, port, {
      onConnection: (connected) => {
        setState((prev) => applyRobotStateUpdate(prev, { connected }, Date.now()));
      },
      onState: (update, timestampMs) => {
        setState((prev) => applyRobotStateUpdate(prev, update, timestampMs));
      },
    });
    return () => subscription.disconnect();
  }, [mode, nt4Enabled, port, uri]);

  useEffect(() => {
    if (mode !== 'nt4' || !nt4Enabled || !state.connected) {
      if (motorTestPublisherRef.current) {
        motorTestPublisherRef.current.runMotor('', 0, false);
      }
      motorTestPublisherRef.current = null;
      shooterSetpointPublisherRef.current = null;
      return;
    }
    let cancelled = false;
    Promise.all([
      createMotorTestPublisher(uri, port),
      createShooterSetpointPublisher(uri, port),
    ])
      .then(([motorPub, shooterPub]) => {
        if (!cancelled) {
          motorTestPublisherRef.current = motorPub;
          shooterSetpointPublisherRef.current = shooterPub;
        }
      })
      .catch(() => {
        if (!cancelled) {
          motorTestPublisherRef.current = null;
          shooterSetpointPublisherRef.current = null;
        }
      });
    return () => {
      cancelled = true;
      if (motorTestPublisherRef.current) {
        motorTestPublisherRef.current.runMotor('', 0, false);
      }
      motorTestPublisherRef.current = null;
      shooterSetpointPublisherRef.current = null;
    };
  }, [mode, nt4Enabled, state.connected, uri, port]);

  useEffect(() => {
    const remaining = state.match.timeRemainingSec;
    const last = lastRemainingRef.current;
    lastRemainingRef.current = remaining;

    const crossed = (threshold: number) => last > threshold && remaining <= threshold;
    if (!crossed(30) && !crossed(15) && !crossed(10)) {
      return;
    }

    const flashClass = crossed(10) ? 'flash-10' : crossed(15) ? 'flash-15' : 'flash-30';
    setMilestoneFlash(flashClass);
    if (flashTimeoutRef.current) {
      window.clearTimeout(flashTimeoutRef.current);
    }
    flashTimeoutRef.current = window.setTimeout(() => {
      setMilestoneFlash('');
    }, 1600);
    return () => {
      if (flashTimeoutRef.current) {
        window.clearTimeout(flashTimeoutRef.current);
      }
    };
  }, [state.match.timeRemainingSec]);

  useEffect(() => {
    setShooterCurrentHistory((prev) => ({
      hood: pushHistory(prev.hood, state.shooter.hoodCurrentAmps),
      left: pushHistory(prev.left, state.shooter.leftCurrentAmps),
      right: pushHistory(prev.right, state.shooter.rightCurrentAmps),
    }));
  }, [
    state.shooter.hoodCurrentAmps,
    state.shooter.leftCurrentAmps,
    state.shooter.rightCurrentAmps,
  ]);

  const shooterCurrentMax = useMemo(
    () =>
      Math.max(
        5,
        ...shooterCurrentHistory.hood,
        ...shooterCurrentHistory.left,
        ...shooterCurrentHistory.right
      ),
    [shooterCurrentHistory]
  );

  if (!connectionScreenDone) {
    return (
      <ConnectionScreen
        onEnterDashboard={handleEnterDashboard}
        initialUri={getStoredUri()}
        initialPort={getStoredPort()}
        initialDashboardMode={getStoredDashboardMode()}
      />
    );
  }

  return (
    <div className={`app ${milestoneFlash}`}>
      <header className="status-bar">
        <div className="status-card">
          <div className="status-label">Match Phase</div>
          <div className="status-value">{state.match.phase.toUpperCase()}</div>
          <div className="status-sub">
            {formatSeconds(state.match.timeRemainingSec)} remaining
          </div>
        </div>
        <div className="status-card">
          <div className="status-label">Battery</div>
          <div className="status-value">{formatNumber(state.match.batteryVolts, 1)} V</div>
          <div className="status-sub">Last update {lastUpdateLabel}</div>
        </div>
        <div className={`status-card lock-card ${state.lock.hasLock ? 'locked' : 'unlocked'}`}>
          <div className="status-label">Target Lock</div>
          <div className="status-value">
            {state.lock.hasLock ? 'LOCKED' : 'NO LOCK'}
          </div>
          <div className="status-sub">
            {state.lock.targetLabel} • {Math.round(state.lock.confidence * 100)}%
          </div>
        </div>
        <div className="status-card">
          <div className="status-label">Connection</div>
          <div className="status-value">{state.connected ? 'ONLINE' : 'OFFLINE'}</div>
          <div className="status-sub">
            {mode === 'mock'
              ? 'Mock telemetry'
              : nt4Enabled
              ? 'NT4 streaming'
              : 'NT4 idle'}
          </div>
        </div>
        {isDebug && (
          <>
            <div className="status-card status-card-actions">
              <div className="status-label">Dashboard mode</div>
              <div className="segmented segmented-sm">
                <button
                  type="button"
                  className={(dashboardMode as DashboardMode) === 'competition' ? 'active' : ''}
                  onClick={() => setDashboardMode('competition')}
                >
                  Competition
                </button>
                <button
                  type="button"
                  className={(dashboardMode as DashboardMode) === 'debug' ? 'active' : ''}
                  onClick={() => setDashboardMode('debug')}
                >
                  Debug
                </button>
              </div>
            </div>
            <div className="status-card status-card-actions">
              <button
                type="button"
                className="status-back-link"
                onClick={handleBackToConnection}
              >
                Back to connection
              </button>
            </div>
          </>
        )}
      </header>

      <nav className="view-tabs">
        <div className="segmented view-tabs-segmented">
          {visibleTabs.map((tab) => (
            <button
              key={tab}
              type="button"
              className={viewTab === tab ? 'active' : ''}
              onClick={() => setViewTab(tab)}
            >
              {tab === 'knngrid'
                ? 'KNN Grid'
                : tab === 'networktables'
                  ? 'Network Tables'
                  : tab === 'autodebug'
                    ? 'Auto debug'
                  : tab === 'motortest'
                    ? 'Motor Test'
                    : tab === 'simulation'
                    ? 'Simulation'
                    : tab.charAt(0).toUpperCase() + tab.slice(1)}
            </button>
          ))}
        </div>
      </nav>

      {viewTab === 'turret' && (
        <section className="subsystem-view">
          <TurretView
            turretAngleDeg={state.turret.angleDeg}
            hoodPitchDeg={state.shooter.hoodPitchDeg}
            velocityMps={state.shooter.velocityMps}
          />
        </section>
      )}

      {viewTab === 'motortest' && (
        <MotorTestView
          uri={uri}
          port={port}
          connected={state.connected}
          nt4Enabled={nt4Enabled}
          mode={mode}
          publisherRef={motorTestPublisherRef}
        />
      )}

      {viewTab === 'networktables' && (
        <NetworkTableView
          uri={uri}
          port={port}
          connected={state.connected}
          nt4Enabled={nt4Enabled}
          mode={mode}
        />
      )}

      {viewTab === 'autodebug' && (
        <AutoDebugPanel
          connected={state.connected}
          nt4Enabled={nt4Enabled}
          mode={mode}
          state={state.autoDebug}
        />
      )}

      {viewTab === 'knngrid' && (
        <>
          {isDebug && (
            <section className="subsystem-view">
              <div className="panel">
                <div className="panel-header">
                  <h2>KNN Map Builder</h2>
                  {loggedPoints.length > 0 && (
                    <span className="status-pill neutral">{loggedPoints.length} points</span>
                  )}
                </div>
                <div className="control-row">
                  <label>Map file</label>
                  <input
                    ref={knnLogInputRef}
                    type="file"
                    accept=".json,application/json"
                    style={{ display: 'none' }}
                    onChange={(e) => {
                      loadKnnLog(e.target.files?.[0] ?? null);
                      e.target.value = '';
                    }}
                  />
                  <button
                    type="button"
                    onClick={() => knnLogInputRef.current?.click()}
                  >
                    Load JSON
                  </button>
                  <button type="button" onClick={exportKnnMap} disabled={loggedPoints.length === 0}>
                    Export JSON
                  </button>
                  <button type="button" onClick={addCurrentPoseToKnnMap}>
                    Add current pose
                  </button>
                  <button type="button" onClick={clearKnnPoints} disabled={loggedPoints.length === 0}>
                    Clear map
                  </button>
                </div>
                <div className="control-row">
                  <label>Manual point</label>
                  <input
                    type="number"
                    value={knnDraftPoint.x}
                    onChange={(event) =>
                      setKnnDraftPoint((prev) => ({
                        ...prev,
                        x: Number(event.target.value) || 0,
                      }))
                    }
                  />
                  <input
                    type="number"
                    value={knnDraftPoint.y}
                    onChange={(event) =>
                      setKnnDraftPoint((prev) => ({
                        ...prev,
                        y: Number(event.target.value) || 0,
                      }))
                    }
                  />
                  <input
                    type="number"
                    value={knnDraftPoint.headingDeg ?? 0}
                    onChange={(event) =>
                      setKnnDraftPoint((prev) => ({
                        ...prev,
                        headingDeg: Number(event.target.value) || 0,
                      }))
                    }
                  />
                  <input
                    type="number"
                    value={knnDraftPoint.shooterRpm ?? 0}
                    onChange={(event) =>
                      setKnnDraftPoint((prev) => ({
                        ...prev,
                        shooterRpm: Number(event.target.value) || 0,
                      }))
                    }
                  />
                  <input
                    type="number"
                    value={knnDraftPoint.hoodDeg ?? 0}
                    onChange={(event) =>
                      setKnnDraftPoint((prev) => ({
                        ...prev,
                        hoodDeg: Number(event.target.value) || 0,
                      }))
                    }
                  />
                  <button type="button" onClick={copyCurrentStateToDraft}>
                    Use current state
                  </button>
                  <button type="button" onClick={() => addKnnPoint(knnDraftPoint)}>
                    Add point
                  </button>
                </div>
                <div className="hint">
                  Build the same `knn_map.json` format the robot reads from deploy, and keep it visible
                  here as a field reference while you set up positions. Each point can also store shooter
                  RPM and hood angle for later interpretation.
                </div>
              </div>
            </section>
          )}
          <KnnGridView
            fieldLengthM={FIELD_LENGTH_M}
            fieldWidthM={FIELD_WIDTH_M}
            loggedPoints={loggedPoints}
            poseX={state.swerve.x}
            poseY={state.swerve.y}
            headingDeg={state.swerve.headingDeg}
            onPointChange={updateKnnPoint}
            onPointRemove={removeKnnPoint}
            robotSelectedIndex={
              state.knnSelectedIndex >= 0 ? state.knnSelectedIndex : null
            }
          />
        </>
      )}

      {viewTab === 'chassis' && (
        <section className="subsystem-view">
          <ChassisView
            fieldLengthM={FIELD_LENGTH_M}
            fieldWidthM={FIELD_WIDTH_M}
            robotLengthM={robotDims.lengthM}
            robotWidthM={robotDims.widthM}
            poseX={state.swerve.x}
            poseY={state.swerve.y}
            headingDeg={state.swerve.headingDeg}
            visionPoseX={state.visionPose.x}
            visionPoseY={state.visionPose.y}
            visionHeadingDeg={state.visionPose.headingDeg}
            visionPoseVisible={state.visionPose.valid}
            speedMps={state.swerve.speedMps}
            fieldRelative={state.swerve.fieldRelative}
            targets={state.targets}
            loggedPoints={loggedPoints}
          />
        </section>
      )}

      {viewTab === 'simulation' && (
        <SimulationView
          fieldLengthM={FIELD_LENGTH_M}
          fieldWidthM={FIELD_WIDTH_M}
          robotLengthM={robotDims.lengthM}
          robotWidthM={robotDims.widthM}
          poseX={state.swerve.x}
          poseY={state.swerve.y}
          headingDeg={state.swerve.headingDeg}
          turretAngleDeg={state.turret.angleDeg}
          hoodPitchDeg={state.shooter.hoodPitchDeg}
          velocityMps={state.shooter.velocityMps}
        />
      )}

      {viewTab === 'dashboard' && (
      isDebug ? (
      <section className="dashboard-grid">
        <div className="panel connection-panel">
          <div className="panel-header">
            <h2>Connection</h2>
            <span className={`status-pill ${state.connected ? 'connected' : 'disconnected'}`}>
              {state.connected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
          <div className="control-row">
            <label>Mode</label>
            <div className="segmented">
              <button
                className={mode === 'mock' ? 'active' : ''}
                onClick={() => setMode('mock')}
              >
                Mock
              </button>
              <button
                className={mode === 'nt4' ? 'active' : ''}
                onClick={() => setMode('nt4')}
              >
                NT4
              </button>
            </div>
          </div>
          <div className="control-row">
            <label>URI</label>
            <input
              value={uri}
              onChange={(event) => setUri(event.target.value)}
              disabled={mode === 'mock'}
            />
          </div>
          <div className="control-row">
            <label>Port</label>
            <input
              type="number"
              value={port}
              onChange={(event) => setPort(Number(event.target.value) || 0)}
              disabled={mode === 'mock'}
            />
          </div>
          <div className="control-row">
            <label>NT4</label>
            <button
              className={nt4Enabled ? 'danger' : ''}
              onClick={() => setNt4Enabled((prev) => !prev)}
              disabled={mode === 'mock'}
            >
              {nt4Enabled ? 'Disconnect' : 'Connect'}
            </button>
          </div>
          <div className="hint">
            Configure the NT4 host for match ops. Mock mode simulates telemetry while
            still allowing manual overrides.
          </div>
        </div>

        <div className="panel field-panel">
          <div className="panel-header">
            <h2>Camera</h2>
            <span className="status-pill neutral">
              USB Camera
            </span>
          </div>
          <CameraView uri={uri} enabled={mode === 'nt4' && nt4Enabled} />
          <div className="hint">
            Live MJPEG stream from the roboRIO camera server on port 1181.
          </div>
        </div>

        <div className="panel">
          <div className="panel-header">
            <h2>Swerve Drive</h2>
            <span className="status-pill neutral">
              {state.swerve.fieldRelative ? 'Field Relative' : 'Robot Relative'}
            </span>
          </div>
          <div className="metric-grid">
            <div>
              <div className="metric-label">Pose X</div>
              <div className="metric-value">{formatNumber(state.swerve.x, 2)} m</div>
            </div>
            <div>
              <div className="metric-label">Pose Y</div>
              <div className="metric-value">{formatNumber(state.swerve.y, 2)} m</div>
            </div>
            <div>
              <div className="metric-label">Heading</div>
              <div className="metric-value">
                {formatNumber(state.swerve.headingDeg, 1)}°
              </div>
            </div>
            <div>
              <div className="metric-label">Speed</div>
              <div className="metric-value">
                {formatNumber(state.swerve.speedMps, 1)} m/s
              </div>
            </div>
            <div>
              <div className="metric-label">LL Pose</div>
              <div className="metric-value">
                {state.visionPose.valid
                  ? `${formatNumber(state.visionPose.x, 2)}, ${formatNumber(state.visionPose.y, 2)}`
                  : 'No target'}
              </div>
            </div>
            <div>
              <div className="metric-label">LL Heading</div>
              <div className="metric-value">
                {state.visionPose.valid
                  ? `${formatNumber(state.visionPose.headingDeg, 1)}°`
                  : '--'}
              </div>
            </div>
          </div>
          <div className="button-row">
            <button onClick={() => applyUpdate({ swerve: { headingDeg: 0 } })}>
              Zero Heading
            </button>
            <button onClick={() => applyUpdate({ swerve: { x: 0, y: 0 } })}>
              Zero Pose
            </button>
            <button
              onClick={() =>
                applyUpdate({ swerve: { fieldRelative: !state.swerve.fieldRelative } })
              }
            >
              Toggle Frame
            </button>
          </div>
        </div>

        <div className="panel">
          <div className="panel-header">
            <h2>Intake</h2>
            <span className={`status-pill ${state.intake.enabled ? 'connected' : 'disconnected'}`}>
              {state.intake.enabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          <div className="metric-grid">
            <div>
              <div className="metric-label">Roller RPM</div>
              <div className="metric-value">{formatNumber(state.intake.rollerRpm, 0)}</div>
            </div>
            <div>
              <div className="metric-label">Has Piece</div>
              <div className="metric-value">{state.intake.hasPiece ? 'Yes' : 'No'}</div>
            </div>
            <div>
              <div className="metric-label">Deployed</div>
              <div className="metric-value">{state.intake.deployed ? 'Down' : 'Up'}</div>
            </div>
          </div>
          <div className="button-row">
            <button onClick={() => applyUpdate({ intake: { enabled: !state.intake.enabled } })}>
              {state.intake.enabled ? 'Disable' : 'Enable'}
            </button>
            <button onClick={() => applyUpdate({ intake: { deployed: !state.intake.deployed } })}>
              {state.intake.deployed ? 'Stow' : 'Deploy'}
            </button>
            <button onClick={() => applyUpdate({ intake: { hasPiece: !state.intake.hasPiece } })}>
              Toggle Piece
            </button>
          </div>
        </div>

        <div className="panel">
          <div className="panel-header">
            <h2>Shooter + Hood</h2>
            <span className={`status-pill ${state.shooter.enabled ? 'connected' : 'disconnected'}`}>
              {state.shooter.enabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          <div className="metric-grid">
            <div>
              <div className="metric-label">RPM</div>
              <div className="metric-value">{formatNumber(state.shooter.rpm, 0)}</div>
            </div>
            <div>
              <div className="metric-label">RPM Setpoint</div>
              <div className="metric-value">{formatNumber(state.shooter.rpmSetpoint, 0)}</div>
            </div>
            <div>
              <div className="metric-label">Hood</div>
              <div className="metric-value">{formatNumber(state.shooter.hoodDeg, 1)}°</div>
            </div>
            <div>
              <div className="metric-label">Hood Setpoint</div>
              <div className="metric-value">
                {formatNumber(state.shooter.hoodSetpoint, 1)}°
              </div>
            </div>
            <div>
              <div className="metric-label">Hood Current</div>
              <div className="metric-value">
                {formatNumber(state.shooter.hoodCurrentAmps, 1)} A
              </div>
            </div>
            <div>
              <div className="metric-label">Left Motor Current</div>
              <div className="metric-value">
                {formatNumber(state.shooter.leftCurrentAmps, 1)} A
              </div>
            </div>
            <div>
              <div className="metric-label">Right Motor Current</div>
              <div className="metric-value">
                {formatNumber(state.shooter.rightCurrentAmps, 1)} A
              </div>
            </div>
          </div>
          <div className="current-plot-card">
            <div className="current-plot-header">
              <div className="metric-label">Current Plot</div>
              <div className="current-plot-scale">0-{formatNumber(shooterCurrentMax, 0)} A</div>
            </div>
            <div className="current-trace-list">
              <CurrentTrace
                label="Hood"
                color="#f59e0b"
                value={state.shooter.hoodCurrentAmps}
                values={shooterCurrentHistory.hood}
                maxValue={shooterCurrentMax}
              />
              <CurrentTrace
                label="Left Motor"
                color="#22c55e"
                value={state.shooter.leftCurrentAmps}
                values={shooterCurrentHistory.left}
                maxValue={shooterCurrentMax}
              />
              <CurrentTrace
                label="Right Motor"
                color="#3b82f6"
                value={state.shooter.rightCurrentAmps}
                values={shooterCurrentHistory.right}
                maxValue={shooterCurrentMax}
              />
            </div>
          </div>
          <div className="control-row control-row-slider">
            <label>RPM Setpoint</label>
            <div className="slider-with-value">
              <input
                type="range"
                min={0}
                max={6000}
                step={50}
                value={state.shooter.rpmSetpoint}
                onChange={(event) => {
                  const val = Number(event.target.value) || 0;
                  applyUpdate({ shooter: { rpmSetpoint: val } });
                  shooterSetpointPublisherRef.current?.setRpmSetpoint(val);
                }}
              />
              <span className="metric-value">{formatNumber(state.shooter.rpmSetpoint, 0)}</span>
            </div>
          </div>
          <div className="control-row control-row-slider">
            <label>Hood Setpoint</label>
            <div className="slider-with-value">
              <input
                type="range"
                min={0}
                max={90}
                step={1}
                value={state.shooter.hoodSetpoint}
                onChange={(event) => {
                  const val = Number(event.target.value) || 0;
                  applyUpdate({ shooter: { hoodSetpoint: val } });
                  shooterSetpointPublisherRef.current?.setHoodSetpoint(val);
                }}
              />
              <span className="metric-value">{formatNumber(state.shooter.hoodSetpoint, 1)}°</span>
            </div>
          </div>
          <div className="button-row">
            <button onClick={() => applyUpdate({ shooter: { enabled: !state.shooter.enabled } })}>
              {state.shooter.enabled ? 'Disable' : 'Enable'}
            </button>
            <button onClick={() => applyUpdate({ shooter: { rpmSetpoint: 4100 } })}>
              High Goal
            </button>
            <button onClick={() => applyUpdate({ shooter: { rpmSetpoint: 2900 } })}>
              Low Goal
            </button>
          </div>
        </div>

        <div className="panel">
          <div className="panel-header">
            <h2>Turret</h2>
            <span className={`status-pill ${state.turret.enabled ? 'connected' : 'disconnected'}`}>
              {state.turret.enabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          <div className="metric-grid">
            <div>
              <div className="metric-label">Angle</div>
              <div className="metric-value">{formatNumber(state.turret.angleDeg, 1)}°</div>
            </div>
            <div>
              <div className="metric-label">Setpoint</div>
              <div className="metric-value">{formatNumber(state.turret.angleSetpoint, 1)}°</div>
            </div>
            <div>
              <div className="metric-label">Tracking</div>
              <div className="metric-value">{state.turret.tracking ? 'Active' : 'Idle'}</div>
            </div>
          </div>
          <div className="control-row">
            <label>Angle Setpoint</label>
            <input
              type="number"
              value={state.turret.angleSetpoint}
              onChange={(event) =>
                applyUpdate({ turret: { angleSetpoint: Number(event.target.value) || 0 } })
              }
            />
          </div>
          <div className="button-row">
            <button onClick={() => applyUpdate({ turret: { enabled: !state.turret.enabled } })}>
              {state.turret.enabled ? 'Disable' : 'Enable'}
            </button>
            <button
              onClick={() => applyUpdate({ turret: { tracking: !state.turret.tracking } })}
            >
              {state.turret.tracking ? 'Tracking Off' : 'Tracking On'}
            </button>
            <button onClick={() => applyUpdate({ turret: { angleSetpoint: 0 } })}>
              Center
            </button>
          </div>
        </div>

        <MotorTestPanel
          publisherRef={motorTestPublisherRef}
          connected={state.connected}
          nt4Enabled={nt4Enabled}
          mode={mode}
          compact
        />
      </section>
      ) : (
      <section className="competition-dashboard">
        <div className="panel competition-camera-panel">
          <CameraView uri={uri} enabled={mode === 'nt4' && nt4Enabled} />
        </div>
      </section>
      )
      )}
    </div>
  );
}

export default App;
