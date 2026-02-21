import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';

export type Pose2dSample = {
  x: number;
  y: number;
  rotation: number;
  timestampMs: number;
};

export type PoseSubscription = {
  disconnect: () => void;
};

type PoseCallbacks = {
  onPose: (pose: Pose2dSample) => void;
  onConnection: (connected: boolean) => void;
};

export function connectPoseSubscription(
  uri: string,
  port: number,
  { onPose, onConnection }: PoseCallbacks
): PoseSubscription {
  const nt = NetworkTables.getInstanceByURI(uri, port);
  const removeConnectionListener = nt.addRobotConnectionListener(onConnection, true);

  const poseTopic = nt.createTopic(
    '/Pose/robotPose',
    NetworkTablesTypeInfos.kDoubleArray,
    [0, 0, 0]
  );

  const subId = poseTopic.subscribe((value) => {
    if (!value || !Array.isArray(value) || value.length < 3) {
      return;
    }
    onPose({
      x: Number(value[0]) || 0,
      y: Number(value[1]) || 0,
      rotation: Number(value[2]) || 0,
      timestampMs: Date.now(),
    });
  });

  return {
    disconnect: () => {
      poseTopic.unsubscribe(subId);
      removeConnectionListener();
      nt.client.cleanup();
    },
  };
}
