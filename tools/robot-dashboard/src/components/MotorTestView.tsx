import { useEffect, useRef } from 'react';
import { createMotorTestPublisher, type MotorTestPublisher } from '../lib/nt4Client';
import MotorTestPanel from './MotorTestPanel';

type Props = {
  uri: string;
  port: number;
  connected: boolean;
  nt4Enabled: boolean;
  mode: 'mock' | 'nt4';
  /** When provided, use this publisher instead of creating one (e.g. from App). */
  publisherRef?: React.MutableRefObject<MotorTestPublisher | null>;
};

export default function MotorTestView({
  uri,
  port,
  connected,
  nt4Enabled,
  mode,
  publisherRef: externalPublisherRef,
}: Props) {
  const internalPublisherRef = useRef<MotorTestPublisher | null>(null);
  const publisherRef = externalPublisherRef ?? internalPublisherRef;

  useEffect(() => {
    if (externalPublisherRef) return;
    if (mode !== 'nt4' || !nt4Enabled || !connected) {
      internalPublisherRef.current = null;
      return;
    }
    let cancelled = false;
    createMotorTestPublisher(uri, port)
      .then((pub) => {
        if (!cancelled) internalPublisherRef.current = pub;
      })
      .catch(() => {
        if (!cancelled) internalPublisherRef.current = null;
      });
    return () => {
      cancelled = true;
      if (internalPublisherRef.current) {
        internalPublisherRef.current.runMotor('', 0, false);
      }
      internalPublisherRef.current = null;
    };
  }, [externalPublisherRef, mode, nt4Enabled, connected, uri, port]);

  return (
    <section className="subsystem-view motor-test-view">
      <MotorTestPanel
        publisherRef={publisherRef}
        connected={connected}
        nt4Enabled={nt4Enabled}
        mode={mode}
        compact={false}
      />
    </section>
  );
}
