import type { RobotState } from '../lib/robotState';

type Props = {
  connected: boolean;
  nt4Enabled: boolean;
  mode: 'mock' | 'nt4';
  state: RobotState['autoDebug'];
  onSelectAuto: (autoName: string) => void;
};

function Row({ label, value }: { label: string; value: string }) {
  return (
    <tr>
      <td className="nt-key">{label}</td>
      <td className="nt-value">{value}</td>
    </tr>
  );
}

export default function AutoDebugPanel({ connected, nt4Enabled, mode, state, onSelectAuto }: Props) {
  const show = mode === 'nt4' && nt4Enabled && connected;

  return (
    <section className="subsystem-view network-table-view">
      <div className="panel nt-panel">
        <div className="panel-header">
          <h2>Auto / PathPlanner debug</h2>
          <span className={`status-pill ${show ? 'connected' : 'disconnected'}`}>
            {show ? 'Streaming NT' : 'Not connected'}
          </span>
        </div>
        {!show && (
          <p className="nt-hint">
            Connect in <strong>NT4</strong> mode to stream <code>Auto/*</code> and{' '}
            <code>PathFollower/*</code> from the robot.
          </p>
        )}
        {show && (
          <>
            <div className="control-row" style={{ marginBottom: '0.75rem' }}>
              <label>Auto chooser</label>
              <select
                value={state.chooserSelected}
                onChange={(event) => onSelectAuto(event.target.value)}
                disabled={state.availableAutos.length === 0}
              >
                {state.availableAutos.length === 0 ? (
                  <option value="">No autos published</option>
                ) : (
                  state.availableAutos.map((autoName) => (
                    <option key={autoName} value={autoName}>
                      {autoName}
                    </option>
                  ))
                )}
              </select>
              <span className="nt-value">
                Active: {state.chooserActive || '—'} | Default: {state.chooserDefault || '—'}
              </span>
            </div>
            <p className="nt-hint" style={{ marginBottom: '0.75rem' }}>
              Mirrors robot diagnostics for autonomous scheduling, PathPlanner outputs, and tracking
              error. If <code>outputInvokeCount</code> stays at 0 during auto, the PathPlanner output
              callback never ran (command not following a path). If <code>defaultDriveScheduled</code> is
              true while a path should run, teleop default drive may still be claiming the drivetrain.
              Full keys: <strong>Network Tables</strong> (filter <code>Auto</code> or{' '}
              <code>PathFollower</code>).
            </p>
            <div className="nt-table-wrap">
              <table className="nt-table">
                <tbody>
                  <Row
                    label="SmartDashboard / Auto Mode/options"
                    value={state.availableAutos.length > 0 ? state.availableAutos.join(', ') : '—'}
                  />
                  <Row
                    label="SmartDashboard / Auto Mode/selected"
                    value={state.chooserSelected || '—'}
                  />
                  <Row label="SmartDashboard / Auto Mode/active" value={state.chooserActive || '—'} />
                  <Row
                    label="SmartDashboard / Auto Mode/default"
                    value={state.chooserDefault || '—'}
                  />
                  <Row label="Auto / lastEvent" value={state.lastEvent || '—'} />
                  <Row label="Auto / eventIndex" value={String(state.eventIndex)} />
                  <Row label="Auto / chooserCommandName" value={state.chooserCommandName || '—'} />
                  <Row label="Auto / chooserCommandClass" value={state.chooserCommandClass || '—'} />
                  <Row label="Auto / resolvedAutoCommandName" value={state.resolvedAutoCommandName || '—'} />
                  <Row label="Auto / resolvedAutoCommandClass" value={state.resolvedAutoCommandClass || '—'} />
                  <Row
                    label="Auto / autoUsedChooserFallback"
                    value={String(state.autoUsedChooserFallback)}
                  />
                  <Row
                    label="Auto / defaultDriveCanceledForAuto"
                    value={String(state.defaultDriveCanceledForAuto)}
                  />
                  <Row label="Auto / selectedCommandName (alias)" value={state.selectedCommandName || '—'} />
                  <Row label="Auto / selectedCommandClass (alias)" value={state.selectedCommandClass || '—'} />
                  <Row label="Auto / driveSubsystemCommand" value={state.driveSubsystemCommand || '—'} />
                  <Row
                    label="Auto / registeredNamedCommands"
                    value={state.registeredNamedCommands || '—'}
                  />
                  <Row
                    label="Auto / autonomousCommandScheduled"
                    value={String(state.autonomousCommandScheduled)}
                  />
                  <Row
                    label="Auto / defaultDriveScheduled"
                    value={String(state.defaultDriveScheduled)}
                  />
                  <Row
                    label="Auto / debugTelemetryEnabled"
                    value={String(state.debugTelemetryEnabled)}
                  />
                  <Row
                    label="PathFollower / outputInvokeCount"
                    value={String(state.outputInvokeCount)}
                  />
                  <Row
                    label="PathFollower / pathOutputRecent"
                    value={String(state.pathOutputRecent)}
                  />
                  <Row
                    label="PathFollower / alliancePresent"
                    value={String(state.alliancePresent)}
                  />
                  <Row label="PathFollower / flipPathForRed" value={String(state.flipPathForRed)} />
                  <Row label="PathFollower / commandedVx (m/s)" value={state.commandedVx.toFixed(3)} />
                  <Row label="PathFollower / commandedVy (m/s)" value={state.commandedVy.toFixed(3)} />
                  <Row
                    label="PathFollower / commandedOmega (rad/s)"
                    value={state.commandedOmega.toFixed(3)}
                  />
                  <Row
                    label="PathFollower / feedforwardForceSumN"
                    value={state.feedforwardForceSumN.toFixed(1)}
                  />
                  <Row label="PathFollower / vxError (m/s)" value={state.vxError.toFixed(3)} />
                  <Row label="PathFollower / vyError (m/s)" value={state.vyError.toFixed(3)} />
                  <Row label="PathFollower / omegaError (rad/s)" value={state.omegaError.toFixed(3)} />
                  <Row label="PathFollower / configMassKg" value={state.configMassKg.toFixed(1)} />
                  <Row label="PathFollower / configMoiKgM2" value={state.configMoiKgM2.toFixed(2)} />
                  <Row label="PathFollower / configWheelCOF" value={state.configWheelCOF.toFixed(2)} />
                  <Row label="PathFollower / configTransKp" value={state.configTransKp.toFixed(2)} />
                  <Row label="PathFollower / configRotKp" value={state.configRotKp.toFixed(2)} />
                </tbody>
              </table>
            </div>
          </>
        )}
      </div>
    </section>
  );
}
