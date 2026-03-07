import { useState } from 'react';
import type { MotorTestPublisher } from '../lib/nt4Client';

const DEFAULT_SPEED = 0.15;

type MotorRow = { label: string; id: string };

const SHOOTER_MOTORS: MotorRow[] = [
  { label: 'Shooter L', id: 'shooterLeft' },
  { label: 'Shooter R', id: 'shooterRight' },
  { label: 'Hood', id: 'hood' },
];

const INTAKE_MOTORS: MotorRow[] = [
  { label: 'Roller', id: 'intakeRoller' },
  { label: 'Pivot', id: 'intakePivot' },
];

const DRIVE_MOTORS: MotorRow[] = [
  { label: 'FL', id: 'drive0' },
  { label: 'FR', id: 'drive1' },
  { label: 'BL', id: 'drive2' },
  { label: 'BR', id: 'drive3' },
];

const STEER_MOTORS: MotorRow[] = [
  { label: 'FL', id: 'steer0' },
  { label: 'FR', id: 'steer1' },
  { label: 'BL', id: 'steer2' },
  { label: 'BR', id: 'steer3' },
];

function MotorRowControls({
  motorId,
  speed,
  publisherRef,
}: {
  motorId: string;
  speed: number;
  publisherRef: React.MutableRefObject<MotorTestPublisher | null>;
}) {
  const run = (direction: 1 | -1) => {
    const pub = publisherRef.current;
    if (!pub) return;
    pub.runMotor(motorId, speed * direction, true);
  };
  const stop = () => {
    const pub = publisherRef.current;
    if (!pub) return;
    pub.runMotor(motorId, 0, false);
  };
  return (
    <div className="motor-test-row">
      <button type="button" className="motor-test-btn fwd" onClick={() => run(1)}>
        Fwd
      </button>
      <button type="button" className="motor-test-btn rev" onClick={() => run(-1)}>
        Back
      </button>
      <button type="button" className="motor-test-btn stop" onClick={stop}>
        Stop
      </button>
    </div>
  );
}

type Props = {
  publisherRef: React.MutableRefObject<MotorTestPublisher | null>;
  connected: boolean;
  nt4Enabled: boolean;
  mode: 'mock' | 'nt4';
  /** If true, show compact single-column layout (for dashboard grid). */
  compact?: boolean;
};

export default function MotorTestPanel({
  publisherRef,
  connected,
  nt4Enabled,
  mode,
  compact = false,
}: Props) {
  const [speed, setSpeed] = useState(DEFAULT_SPEED);
  const showContent = mode === 'nt4' && nt4Enabled && connected;
  const clampedSpeed = Math.max(0.05, Math.min(0.5, speed));

  return (
    <div className={`panel motor-test-panel ${compact ? 'motor-test-panel-compact' : ''}`}>
      <div className="panel-header">
        <h2>Motor Test (NT)</h2>
        <span className={`status-pill ${showContent ? 'connected' : 'disconnected'}`}>
          {showContent ? 'Ready' : 'Off'}
        </span>
      </div>
      {!showContent && (
        <p className="motor-test-hint">
          Use <strong>NT4</strong> and <strong>Connect</strong>. Robot must be <strong>disabled</strong>.
        </p>
      )}
      {showContent && (
        <>
          <div className="motor-test-global">
            <label>Speed (0.05–0.5)</label>
            <input
              type="number"
              min={0.05}
              max={0.5}
              step={0.05}
              value={speed}
              onChange={(e) => setSpeed(Number(e.target.value) || DEFAULT_SPEED)}
              className="motor-test-speed-input"
            />
          </div>
          <div className="motor-test-section">
            <h3>Shooter</h3>
            {SHOOTER_MOTORS.map(({ label, id }) => (
              <div key={id} className="motor-test-motor">
                <span className="motor-test-label">{label}</span>
                <MotorRowControls motorId={id} speed={clampedSpeed} publisherRef={publisherRef} />
              </div>
            ))}
          </div>
          <div className="motor-test-section">
            <h3>Intake</h3>
            {INTAKE_MOTORS.map(({ label, id }) => (
              <div key={id} className="motor-test-motor">
                <span className="motor-test-label">{label}</span>
                <MotorRowControls motorId={id} speed={clampedSpeed} publisherRef={publisherRef} />
              </div>
            ))}
          </div>
          <div className="motor-test-section">
            <h3>Drivetrain</h3>
            <h4 className="motor-test-subheading">Drive</h4>
            {DRIVE_MOTORS.map(({ label, id }) => (
              <div key={id} className="motor-test-motor">
                <span className="motor-test-label">{label}</span>
                <MotorRowControls motorId={id} speed={clampedSpeed} publisherRef={publisherRef} />
              </div>
            ))}
            <h4 className="motor-test-subheading">Steer</h4>
            {STEER_MOTORS.map(({ label, id }) => (
              <div key={id} className="motor-test-motor">
                <span className="motor-test-label">{label}</span>
                <MotorRowControls motorId={id} speed={clampedSpeed} publisherRef={publisherRef} />
              </div>
            ))}
          </div>
        </>
      )}
    </div>
  );
}
