import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';
import type { NetworkTablesTypeInfo } from 'ntcore-ts-client';

import type { MatchPhase, RobotStateUpdate } from './robotState';

export type RobotSubscription = {
  disconnect: () => void;
};

type RobotCallbacks = {
  onState: (update: RobotStateUpdate, timestampMs?: number) => void;
  onConnection: (connected: boolean) => void;
};

type TopicConfig = {
  path: string;
  type: NetworkTablesTypeInfo;
  defaultValue: unknown;
  map: (value: unknown) => RobotStateUpdate | null;
};

const MATCH_PHASES: MatchPhase[] = ['disabled', 'auto', 'teleop', 'endgame'];

const DEFAULT_TOPICS: TopicConfig[] = [
  {
    path: '/Dashboard/matchPhase',
    type: NetworkTablesTypeInfos.kString,
    defaultValue: 'disabled',
    map: (value) =>
      typeof value === 'string' && MATCH_PHASES.includes(value as MatchPhase)
        ? { match: { phase: value as MatchPhase } }
        : null,
  },
  {
    path: '/Dashboard/timeRemainingSec',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { match: { timeRemainingSec: value } } : null,
  },
  {
    path: '/Dashboard/batteryVolts',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 12,
    map: (value) =>
      typeof value === 'number' ? { match: { batteryVolts: value } } : null,
  },
  {
    path: '/Dashboard/hasLock',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { lock: { hasLock: value } } : null,
  },
  {
    path: '/Dashboard/lockConfidence',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { lock: { confidence: value } } : null,
  },
  {
    path: '/Dashboard/lockTarget',
    type: NetworkTablesTypeInfos.kString,
    defaultValue: 'None',
    map: (value) =>
      typeof value === 'string' ? { lock: { targetLabel: value } } : null,
  },
  {
    path: '/Pose/robotPose',
    type: NetworkTablesTypeInfos.kDoubleArray,
    defaultValue: [0, 0, 0],
    map: (value) => {
      if (!Array.isArray(value) || value.length < 3) {
        return null;
      }
      return {
        swerve: {
          x: Number(value[0]) || 0,
          y: Number(value[1]) || 0,
          headingDeg: Number(value[2]) || 0,
        },
      };
    },
  },
  {
    path: '/Pose/limelightEstimatedPose',
    type: NetworkTablesTypeInfos.kDoubleArray,
    defaultValue: [0, 0, 0],
    map: (value) => {
      if (!Array.isArray(value) || value.length < 3) {
        return null;
      }
      return {
        visionPose: {
          x: Number(value[0]) || 0,
          y: Number(value[1]) || 0,
          headingDeg: Number(value[2]) || 0,
        },
      };
    },
  },
  {
    path: '/Pose/limelightEstimatedPoseValid',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { visionPose: { valid: value } } : null,
  },
  {
    path: '/Swerve/fieldRelative',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: true,
    map: (value) =>
      typeof value === 'boolean' ? { swerve: { fieldRelative: value } } : null,
  },
  {
    path: '/Swerve/speedMps',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { swerve: { speedMps: value } } : null,
  },
  {
    path: '/Intake/enabled',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { intake: { enabled: value } } : null,
  },
  {
    path: '/Intake/deployed',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { intake: { deployed: value } } : null,
  },
  {
    path: '/Intake/hasPiece',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { intake: { hasPiece: value } } : null,
  },
  {
    path: '/Intake/rollerRpm',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { intake: { rollerRpm: value } } : null,
  },
  {
    path: '/Shooter/enabled',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { shooter: { enabled: value } } : null,
  },
  {
    path: '/Shooter/rpm',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { rpm: value } } : null,
  },
  {
    path: '/Shooter/rpmSetpoint',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { rpmSetpoint: value } } : null,
  },
  {
    path: '/Shooter/hoodDeg',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { hoodDeg: value } } : null,
  },
  {
    path: '/Shooter/hoodSetpoint',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { hoodSetpoint: value } } : null,
  },
  {
    path: '/Shooter/hoodPitchDeg',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { hoodPitchDeg: value } } : null,
  },
  {
    path: '/Shooter/velocityMps',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { velocityMps: value } } : null,
  },
  {
    path: '/Shooter/hoodCurrentAmps',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { hoodCurrentAmps: value } } : null,
  },
  {
    path: '/Shooter/leftCurrentAmps',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { leftCurrentAmps: value } } : null,
  },
  {
    path: '/Shooter/rightCurrentAmps',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { shooter: { rightCurrentAmps: value } } : null,
  },
  {
    path: '/Localization/targets',
    type: NetworkTablesTypeInfos.kDoubleArray,
    defaultValue: [],
    map: (value) => {
      if (!Array.isArray(value) || value.length < 2) {
        return null;
      }
      const targets: { x: number; y: number }[] = [];
      for (let i = 0; i + 1 < value.length; i += 2) {
        targets.push({ x: Number(value[i]) || 0, y: Number(value[i + 1]) || 0 });
      }
      return { targets };
    },
  },
  {
    path: '/Turret/enabled',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { turret: { enabled: value } } : null,
  },
  {
    path: '/Turret/angleDeg',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { turret: { angleDeg: value } } : null,
  },
  {
    path: '/Turret/angleSetpoint',
    type: NetworkTablesTypeInfos.kDouble,
    defaultValue: 0,
    map: (value) =>
      typeof value === 'number' ? { turret: { angleSetpoint: value } } : null,
  },
  {
    path: '/Turret/tracking',
    type: NetworkTablesTypeInfos.kBoolean,
    defaultValue: false,
    map: (value) =>
      typeof value === 'boolean' ? { turret: { tracking: value } } : null,
  },
  {
    path: '/KNN/selectedIndex',
    type: NetworkTablesTypeInfos.kInteger,
    defaultValue: -1,
    map: (value) =>
      typeof value === 'number'
        ? { knnSelectedIndex: Math.floor(value) }
        : null,
  },
];

export function connectRobotStateSubscription(
  uri: string,
  port: number,
  { onConnection, onState }: RobotCallbacks,
  topics: TopicConfig[] = DEFAULT_TOPICS
): RobotSubscription {
  const nt = NetworkTables.getInstanceByURI(uri, port);
  const removeConnectionListener = nt.addRobotConnectionListener(onConnection, true);

  const unsubscribers = topics.map((topic) => {
    const ntTopic = nt.createTopic(topic.path, topic.type, topic.defaultValue as string | number | boolean | number[] | string[] | ArrayBuffer | boolean[] | undefined);
    const subId = ntTopic.subscribe((value) => {
      const update = topic.map(value);
      if (update) {
        onState(update, Date.now());
      }
    });
    return () => ntTopic.unsubscribe(subId);
  });

  return {
    disconnect: () => {
      unsubscribers.forEach((unsubscribe) => unsubscribe());
      removeConnectionListener();
      nt.client.cleanup();
    },
  };
}

export const DEFAULT_NT4_TOPICS = DEFAULT_TOPICS;

// --- Full Network Table view ---

export type NetworkTableEntry = { type: string; value: unknown };

export type FullNetworkTableSubscription = {
  disconnect: () => void;
};

/**
 * Subscribes to all NetworkTables topics (prefix "") and calls onUpdate with
 * the current map of topic name -> { type, value } whenever any topic changes.
 * Use the same uri/port as the main dashboard connection so the same NT client is used.
 */
export function connectFullNetworkTableSubscription(
  uri: string,
  port: number,
  onUpdate: (entries: Map<string, NetworkTableEntry>) => void
): FullNetworkTableSubscription {
  const nt = NetworkTables.getInstanceByURI(uri, port);
  const prefixTopic = nt.createPrefixTopic('');
  const entries = new Map<string, NetworkTableEntry>();

  const subId = prefixTopic.subscribe((value, params) => {
    entries.set(params.name, { type: params.type, value });
    onUpdate(new Map(entries));
  });

  return {
    disconnect: () => {
      prefixTopic.unsubscribe(subId);
      // Do not call nt.client.cleanup() — other subscriptions (dashboard) use the same client
    },
  };
}

// --- Motor Test publisher (dashboard writes, robot reads) ---

const MOTOR_TEST_ENABLE_PATH = 'MotorTest/Enable';
const MOTOR_TEST_MOTOR_PATH = 'MotorTest/Motor';
const MOTOR_TEST_SPEED_PATH = 'MotorTest/Speed';

export type MotorTestPublisher = {
  setEnable: (value: boolean) => void;
  setMotor: (value: string) => void;
  setSpeed: (value: number) => void;
  runMotor: (motorId: string, speed: number, enable: boolean) => void;
};

/**
 * Creates MotorTest NT topics, publishes so this client can set values, and returns setters.
 * Call when connected (NT4 mode). Robot reads MotorTest/Enable, MotorTest/Motor, MotorTest/Speed.
 */
export async function createMotorTestPublisher(
  uri: string,
  port: number
): Promise<MotorTestPublisher> {
  const nt = NetworkTables.getInstanceByURI(uri, port);
  const enableTopic = nt.createTopic(MOTOR_TEST_ENABLE_PATH, NetworkTablesTypeInfos.kBoolean, false as boolean);
  const motorTopic = nt.createTopic(MOTOR_TEST_MOTOR_PATH, NetworkTablesTypeInfos.kString, '' as string);
  const speedTopic = nt.createTopic(MOTOR_TEST_SPEED_PATH, NetworkTablesTypeInfos.kDouble, 0.15 as number);

  await Promise.all([enableTopic.publish(), motorTopic.publish(), speedTopic.publish()]);

  return {
    setEnable: (value: boolean) => enableTopic.setValue(value),
    setMotor: (value: string) => motorTopic.setValue(value),
    setSpeed: (value: number) => speedTopic.setValue(value),
    runMotor: (motorId: string, speed: number, enable: boolean) => {
      motorTopic.setValue(motorId);
      speedTopic.setValue(speed);
      enableTopic.setValue(enable);
    },
  };
}

// --- Shooter setpoint publisher (dashboard sliders -> robot reads hoodSetpointInput / rpmSetpointInput) ---

const SHOOTER_HOOD_SETPOINT_INPUT_PATH = 'Shooter/hoodSetpointInput';
const SHOOTER_RPM_SETPOINT_INPUT_PATH = 'Shooter/rpmSetpointInput';

export type ShooterSetpointPublisher = {
  setHoodSetpoint: (degrees: number) => void;
  setRpmSetpoint: (rpm: number) => void;
};

/**
 * Creates NT topics for shooter/hood setpoint input. Robot reads these in periodic() and applies.
 * Call when connected (NT4 mode).
 */
export async function createShooterSetpointPublisher(
  uri: string,
  port: number
): Promise<ShooterSetpointPublisher> {
  const nt = NetworkTables.getInstanceByURI(uri, port);
  const hoodTopic = nt.createTopic(SHOOTER_HOOD_SETPOINT_INPUT_PATH, NetworkTablesTypeInfos.kDouble, 0 as number);
  const rpmTopic = nt.createTopic(SHOOTER_RPM_SETPOINT_INPUT_PATH, NetworkTablesTypeInfos.kDouble, 0 as number);

  await Promise.all([hoodTopic.publish(), rpmTopic.publish()]);

  return {
    setHoodSetpoint: (degrees: number) => hoodTopic.setValue(degrees),
    setRpmSetpoint: (rpm: number) => rpmTopic.setValue(rpm),
  };
}
