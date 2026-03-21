import type { RobotState } from '../lib/robotState';

type Props = {
  connected: boolean;
  nt4Enabled: boolean;
  mode: 'mock' | 'nt4';
  state: RobotState['autoDebug'];
};

function Row({ label, value }: { label: string; value: string }) {
  return (
    <tr>
      <td className="nt-key">{label}</td>
      <td className="nt-value">{value}</td>
    </tr>
  );
}

export default function AutoDebugPanel({ connected, nt4Enabled, mode, state }: Props) {
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
            <p className="nt-hint" style={{ marginBottom: '0.75rem' }}>
              Mirrors robot diagnostics for autonomous scheduling, PathPlanner outputs, and tracking
              error. Full raw keys also appear under <strong>Network Tables</strong> (filter{' '}
              <code>Auto</code> or <code>PathFollower</code>).
            </p>
            <div className="nt-table-wrap">
              <table className="nt-table">
                <tbody>
                  <Row label="Auto / lastEvent" value={state.lastEvent || '—'} />
                  <Row label="Auto / eventIndex" value={String(state.eventIndex)} />
                  <Row label="Auto / selectedCommandName" value={state.selectedCommandName || '—'} />
                  <Row label="Auto / selectedCommandClass" value={state.selectedCommandClass || '—'} />
                  <Row label="Auto / driveSubsystemCommand" value={state.driveSubsystemCommand || '—'} />
                  <Row
                    label="Auto / registeredNamedCommands"
                    value={state.registeredNamedCommands || '—'}
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
