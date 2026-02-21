import { useEffect, useMemo, useState } from 'react';
import {
  connectFullNetworkTableSubscription,
  type NetworkTableEntry,
} from '../lib/nt4Client';

function formatValue(value: unknown, _type: string): string {
  if (value === null || value === undefined) return '—';
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'number') return String(value);
  if (typeof value === 'string') return value;
  if (Array.isArray(value)) {
    const str = JSON.stringify(value);
    return str.length > 80 ? str.slice(0, 77) + '…' : str;
  }
  if (value instanceof ArrayBuffer)
    return `<binary ${value.byteLength} bytes>`;
  try {
    const str = JSON.stringify(value);
    return str.length > 80 ? str.slice(0, 77) + '…' : str;
  } catch {
    return String(value);
  }
}

type Props = {
  uri: string;
  port: number;
  connected: boolean;
  nt4Enabled: boolean;
  mode: 'mock' | 'nt4';
};

export default function NetworkTableView({
  uri,
  port,
  connected,
  nt4Enabled,
  mode,
}: Props) {
  const [entries, setEntries] = useState<Map<string, NetworkTableEntry>>(
    new Map()
  );
  const [filter, setFilter] = useState('');

  useEffect(() => {
    if (mode !== 'nt4' || !nt4Enabled) {
      setEntries(new Map());
      return;
    }
    const sub = connectFullNetworkTableSubscription(uri, port, setEntries);
    return () => sub.disconnect();
  }, [mode, nt4Enabled, uri, port]);

  const sortedEntries = useMemo(() => {
    const list = Array.from(entries.entries());
    const f = filter.trim().toLowerCase();
    const filtered = f
      ? list.filter(([name]) => name.toLowerCase().includes(f))
      : list;
    filtered.sort(([a], [b]) => a.localeCompare(b));
    return filtered;
  }, [entries, filter]);

  const showTable = mode === 'nt4' && nt4Enabled && connected;

  return (
    <section className="subsystem-view network-table-view">
      <div className="panel nt-panel">
        <div className="panel-header">
          <h2>Network Tables</h2>
          <span
            className={`status-pill ${showTable ? 'connected' : 'disconnected'}`}
          >
            {showTable
              ? `${entries.size} topic${entries.size === 1 ? '' : 's'}`
              : 'Not connected'}
          </span>
        </div>
        {!showTable && (
          <p className="nt-hint">
            Switch to <strong>NT4</strong> mode and <strong>Connect</strong> to
            stream the robot’s Network Tables.
          </p>
        )}
        {showTable && (
          <>
            <div className="control-row">
              <label>Filter</label>
              <input
                type="text"
                placeholder="Filter by key (e.g. /Swerve)"
                value={filter}
                onChange={(e) => setFilter(e.target.value)}
                className="nt-filter"
              />
            </div>
            <div className="nt-table-wrap">
              <table className="nt-table">
                <thead>
                  <tr>
                    <th>Key</th>
                    <th>Type</th>
                    <th>Value</th>
                  </tr>
                </thead>
                <tbody>
                  {sortedEntries.map(([name, { type, value }]) => (
                    <tr key={name}>
                      <td className="nt-key">{name}</td>
                      <td className="nt-type">{type}</td>
                      <td className="nt-value">
                        {formatValue(value, type)}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
              {sortedEntries.length === 0 && (
                <p className="nt-empty">
                  {entries.size === 0
                    ? 'No topics yet. Publish from the robot to see entries here.'
                    : 'No keys match the filter.'}
                </p>
              )}
            </div>
          </>
        )}
      </div>
    </section>
  );
}
