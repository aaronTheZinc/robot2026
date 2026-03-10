export type RobotMode = 'mock' | 'nt4';
export type MatchPhase = 'disabled' | 'auto' | 'teleop' | 'endgame';

export type RobotState = {
  mode: RobotMode;
  connected: boolean;
  lastUpdateMs: number;
  match: {
    phase: MatchPhase;
    timeRemainingSec: number;
    batteryVolts: number;
  };
  lock: {
    hasLock: boolean;
    confidence: number;
    targetLabel: string;
  };
  swerve: {
    x: number;
    y: number;
    headingDeg: number;
    speedMps: number;
    fieldRelative: boolean;
  };
  intake: {
    enabled: boolean;
    deployed: boolean;
    hasPiece: boolean;
    rollerRpm: number;
  };
  shooter: {
    enabled: boolean;
    rpm: number;
    rpmSetpoint: number;
    hoodDeg: number;
    hoodSetpoint: number;
    hoodPitchDeg: number;
    velocityMps: number;
    hoodCurrentAmps: number;
    leftCurrentAmps: number;
    rightCurrentAmps: number;
  };
  turret: {
    enabled: boolean;
    angleDeg: number;
    angleSetpoint: number;
    tracking: boolean;
  };
  visionPose: {
    valid: boolean;
    x: number;
    y: number;
    headingDeg: number;
  };
  targets: { x: number; y: number }[];
  /** Robot KNN selected point index from NT /KNN/selectedIndex, or -1 if not set */
  knnSelectedIndex: number;
};

export type RobotStateUpdate = {
  mode?: RobotMode;
  connected?: boolean;
  lastUpdateMs?: number;
  match?: Partial<RobotState['match']>;
  lock?: Partial<RobotState['lock']>;
  swerve?: Partial<RobotState['swerve']>;
  intake?: Partial<RobotState['intake']>;
  shooter?: Partial<RobotState['shooter']>;
  turret?: Partial<RobotState['turret']>;
  visionPose?: Partial<RobotState['visionPose']>;
  targets?: RobotState['targets'];
  knnSelectedIndex?: number;
};

const DEFAULT_MATCH_TIME: Record<MatchPhase, number> = {
  disabled: 10,
  auto: 15,
  teleop: 135,
  endgame: 30,
};

export function createInitialRobotState(mode: RobotMode): RobotState {
  return {
    mode,
    connected: false,
    lastUpdateMs: Date.now(),
    match: {
      phase: 'disabled',
      timeRemainingSec: DEFAULT_MATCH_TIME.disabled,
      batteryVolts: 12.6,
    },
    lock: {
      hasLock: false,
      confidence: 0,
      targetLabel: 'None',
    },
    swerve: {
      x: 0,
      y: 0,
      headingDeg: 0,
      speedMps: 0,
      fieldRelative: true,
    },
    intake: {
      enabled: false,
      deployed: false,
      hasPiece: false,
      rollerRpm: 0,
    },
    shooter: {
      enabled: false,
      rpm: 0,
      rpmSetpoint: 3200,
      hoodDeg: 18,
      hoodSetpoint: 20,
      hoodPitchDeg: 0,
      velocityMps: 0,
      hoodCurrentAmps: 0,
      leftCurrentAmps: 0,
      rightCurrentAmps: 0,
    },
    turret: {
      enabled: false,
      angleDeg: 0,
      angleSetpoint: 0,
      tracking: false,
    },
    visionPose: {
      valid: false,
      x: 0,
      y: 0,
      headingDeg: 0,
    },
    targets: [],
    knnSelectedIndex: -1,
  };
}

export function applyRobotStateUpdate(
  prev: RobotState,
  update: RobotStateUpdate,
  timestampMs = Date.now()
): RobotState {
  return {
    ...prev,
    ...update,
    match: { ...prev.match, ...update.match },
    lock: { ...prev.lock, ...update.lock },
    swerve: { ...prev.swerve, ...update.swerve },
    intake: { ...prev.intake, ...update.intake },
    shooter: { ...prev.shooter, ...update.shooter },
    turret: { ...prev.turret, ...update.turret },
    visionPose: { ...prev.visionPose, ...update.visionPose },
    targets: update.targets !== undefined ? update.targets : prev.targets,
    knnSelectedIndex:
      update.knnSelectedIndex !== undefined
        ? update.knnSelectedIndex
        : prev.knnSelectedIndex,
    lastUpdateMs: timestampMs,
  };
}

function approachValue(current: number, target: number, delta: number): number {
  if (current < target) {
    return Math.min(current + delta, target);
  }
  if (current > target) {
    return Math.max(current - delta, target);
  }
  return current;
}

function nextPhase(phase: MatchPhase): MatchPhase {
  switch (phase) {
    case 'disabled':
      return 'auto';
    case 'auto':
      return 'teleop';
    case 'teleop':
      return 'endgame';
    case 'endgame':
    default:
      return 'disabled';
  }
}

export function nextMockState(prev: RobotState, nowMs = Date.now()): RobotState {
  const lastTime = prev.lastUpdateMs || nowMs;
  const deltaSec = Math.max((nowMs - lastTime) / 1000, 0.05);

  const headingDeg = (prev.swerve.headingDeg + 25 * deltaSec) % 360;
  const speedMps = 1.2 + Math.sin(nowMs / 1200) * 0.8;
  const headingRad = (headingDeg * Math.PI) / 180;
  const x = prev.swerve.x + Math.cos(headingRad) * speedMps * deltaSec;
  const y = prev.swerve.y + Math.sin(headingRad) * speedMps * deltaSec;

  let timeRemaining = prev.match.timeRemainingSec - deltaSec;
  let phase = prev.match.phase;
  if (timeRemaining <= 0) {
    phase = nextPhase(phase);
    timeRemaining = DEFAULT_MATCH_TIME[phase];
  }

  const lockPulse = Math.sin(nowMs / 900) > 0.45;
  const lockConfidence = lockPulse ? 0.78 + Math.sin(nowMs / 400) * 0.15 : 0.12;

  const shooterRpmTarget = prev.shooter.enabled ? prev.shooter.rpmSetpoint : 0;
  const shooterRpm = approachValue(prev.shooter.rpm, shooterRpmTarget, 850 * deltaSec);
  const hoodDeg = approachValue(prev.shooter.hoodDeg, prev.shooter.hoodSetpoint, 12 * deltaSec);
  const hoodPitchDeg = approachValue(prev.shooter.hoodPitchDeg, hoodDeg, 12 * deltaSec);
  const velocityMps = prev.shooter.enabled ? 12 + Math.sin(nowMs / 800) * 2 : 0;

  const turretTarget = prev.turret.enabled
    ? prev.turret.angleSetpoint + Math.sin(nowMs / 1300) * 4
    : 0;
  const turretAngle = approachValue(prev.turret.angleDeg, turretTarget, 18 * deltaSec);

  const intakeRpmTarget = prev.intake.enabled ? 2800 : 0;
  const intakeRpm = approachValue(prev.intake.rollerRpm, intakeRpmTarget, 1200 * deltaSec);
  const hasPiece = prev.intake.enabled ? Math.sin(nowMs / 1500) > 0.2 : prev.intake.hasPiece;

  const batteryVolts = 12.1 + Math.sin(nowMs / 5000) * 0.4;
  const targets: { x: number; y: number }[] = [
    { x: 5.5, y: 2 },
    { x: 5.5, y: -2 },
    { x: 2, y: 0 },
  ];

  return {
    ...prev,
    lastUpdateMs: nowMs,
    match: {
      ...prev.match,
      phase,
      timeRemainingSec: Math.max(0, timeRemaining),
      batteryVolts,
    },
    lock: {
      ...prev.lock,
      hasLock: lockPulse,
      confidence: Math.max(0, Math.min(1, lockConfidence)),
      targetLabel: lockPulse ? 'Speaker' : 'None',
    },
    swerve: {
      ...prev.swerve,
      x,
      y,
      headingDeg,
      speedMps,
    },
    intake: {
      ...prev.intake,
      hasPiece,
      rollerRpm: intakeRpm,
    },
    shooter: {
      ...prev.shooter,
      rpm: shooterRpm,
      hoodDeg,
      hoodPitchDeg,
      velocityMps,
      hoodCurrentAmps: prev.shooter.enabled ? 6 + Math.sin(nowMs / 400) * 1.5 : 0.8,
      leftCurrentAmps: prev.shooter.enabled ? 18 + Math.sin(nowMs / 350) * 3 : 0,
      rightCurrentAmps: prev.shooter.enabled ? 17 + Math.cos(nowMs / 370) * 3 : 0,
    },
    turret: {
      ...prev.turret,
      angleDeg: turretAngle,
      tracking: lockPulse,
    },
    targets,
  };
}
