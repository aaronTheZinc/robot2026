import { useEffect, useMemo, useState } from 'react';

type CameraViewProps = {
  uri: string;
  enabled: boolean;
};

function buildCameraStreamUrl(uri: string): string | null {
  const trimmed = uri.trim();
  if (!trimmed) {
    return null;
  }

  try {
    const parsed = new URL(trimmed.includes('://') ? trimmed : `http://${trimmed}`);
    return `http://${parsed.hostname}:1181/?action=stream`;
  } catch {
    return null;
  }
}

export default function CameraView({ uri, enabled }: CameraViewProps) {
  const [streamState, setStreamState] = useState<'idle' | 'loading' | 'live' | 'error'>('idle');
  const streamUrl = useMemo(() => buildCameraStreamUrl(uri), [uri]);

  useEffect(() => {
    if (!enabled || !streamUrl) {
      setStreamState('idle');
      return;
    }
    setStreamState('loading');
  }, [enabled, streamUrl]);

  if (!enabled) {
    return (
      <div className="camera-view camera-view-placeholder">
        <div className="camera-view-message">
          Connect to NT4 to view the robot camera stream.
        </div>
      </div>
    );
  }

  if (!streamUrl) {
    return (
      <div className="camera-view camera-view-placeholder">
        <div className="camera-view-message">Enter a valid robot address to load the camera.</div>
      </div>
    );
  }

  return (
    <div className="camera-view">
      <img
        key={streamUrl}
        className="camera-stream"
        src={streamUrl}
        alt="Robot camera stream"
        onLoad={() => setStreamState('live')}
        onError={() => setStreamState('error')}
      />
      <div className="camera-overlay">
        <span
          className={`status-pill ${
            streamState === 'live'
              ? 'connected'
              : streamState === 'error'
                ? 'disconnected'
                : 'neutral'
          }`}
        >
          {streamState === 'live'
            ? 'Live'
            : streamState === 'error'
              ? 'Stream unavailable'
              : 'Connecting'}
        </span>
        <span className="camera-overlay-url">{streamUrl}</span>
      </div>
    </div>
  );
}
