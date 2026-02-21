import { useState } from 'react';

const DEFAULT_URI = 'localhost';
const DEFAULT_PORT = 5810;

export type ConnectionConfig = {
  mode: 'mock' | 'nt4';
  uri: string;
  port: number;
  nt4Enabled: boolean;
};

export type DashboardMode = 'competition' | 'debug';

type Props = {
  onEnterDashboard: (
    config: ConnectionConfig,
    dashboardMode: DashboardMode
  ) => void;
  initialUri?: string;
  initialPort?: number;
  initialDashboardMode?: DashboardMode;
};

export default function ConnectionScreen({
  onEnterDashboard,
  initialUri = DEFAULT_URI,
  initialPort = DEFAULT_PORT,
  initialDashboardMode = 'debug',
}: Props) {
  const [mode, setMode] = useState<'mock' | 'nt4'>('nt4');
  const [uri, setUri] = useState(initialUri);
  const [port, setPort] = useState(initialPort);
  const [nt4Enabled, setNt4Enabled] = useState(false);
  const [dashboardMode, setDashboardMode] = useState<DashboardMode>(
    initialDashboardMode
  );

  const handleEnter = () => {
    onEnterDashboard(
      {
        mode,
        uri,
        port,
        nt4Enabled: mode === 'nt4' ? nt4Enabled : false,
      },
      dashboardMode
    );
  };

  return (
    <div className="connection-screen">
      <div className="connection-screen-card">
        <h1 className="connection-screen-title">Robot Dashboard</h1>

        <div className="connection-screen-section">
          <div className="control-row">
            <label>Data source</label>
            <div className="segmented">
              <button
                type="button"
                className={mode === 'mock' ? 'active' : ''}
                onClick={() => setMode('mock')}
              >
                Mock
              </button>
              <button
                type="button"
                className={mode === 'nt4' ? 'active' : ''}
                onClick={() => setMode('nt4')}
              >
                NT4
              </button>
            </div>
          </div>

          {mode === 'nt4' && (
            <>
              <div className="control-row">
                <label>URI</label>
                <input
                  type="text"
                  value={uri}
                  onChange={(e) => setUri(e.target.value)}
                  placeholder="e.g. localhost or 10.x.x.x"
                />
              </div>
              <div className="control-row">
                <label>Port</label>
                <input
                  type="number"
                  value={port}
                  onChange={(e) => setPort(Number(e.target.value) || 0)}
                />
              </div>
              <div className="control-row">
                <label>Connect now</label>
                <button
                  type="button"
                  className={nt4Enabled ? 'danger' : ''}
                  onClick={() => setNt4Enabled((prev) => !prev)}
                >
                  {nt4Enabled ? 'Disconnect' : 'Connect'}
                </button>
              </div>
              <span className="connection-screen-hint">
                Optional — you can connect from the dashboard in debug mode.
              </span>
            </>
          )}
        </div>

        <div className="connection-screen-section">
          <div className="control-row">
            <label>Dashboard mode</label>
            <div className="segmented">
              <button
                type="button"
                className={dashboardMode === 'competition' ? 'active' : ''}
                onClick={() => setDashboardMode('competition')}
              >
                Competition
              </button>
              <button
                type="button"
                className={dashboardMode === 'debug' ? 'active' : ''}
                onClick={() => setDashboardMode('debug')}
              >
                Debug
              </button>
            </div>
          </div>
          <div className="connection-screen-hint connection-screen-mode-hint">
            {dashboardMode === 'competition'
              ? 'Minimal UI: Dashboard and Turret tabs only.'
              : 'Full UI: all tabs and connection/field controls.'}
          </div>
        </div>

        <div className="connection-screen-actions">
          <button
            type="button"
            className="connection-screen-enter"
            onClick={handleEnter}
          >
            Enter dashboard
          </button>
        </div>
      </div>
    </div>
  );
}
