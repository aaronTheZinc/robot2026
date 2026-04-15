import { useMemo, useState } from 'react';

import { FIELD_LENGTH_M, FIELD_WIDTH_M } from '../lib/fieldDimensions';
import { defaultFieldSize, type FieldSize } from '../lib/pathPlannerFlip';
import { flipPolylineRotational, samplePathPolyline, type PathPoint } from '../lib/pathPlannerBezier';
import {
  collectPathNamesFromAutoJson,
  getDeployAutoRaw,
  getDeployPathRaw,
  listDeployAutoNames,
  listDeployPathNames,
  parsePathJson,
} from '../lib/deployPathPlanner';

type PreviewMode = 'path' | 'auto';

function fieldToSvg(x: number, y: number, L: number, W: number, svgW: number, svgH: number) {
  const px = (x / L) * svgW;
  const py = svgH - (y / W) * svgH;
  return { px, py };
}

function polylineToSvgPoints(
  pts: PathPoint[],
  L: number,
  W: number,
  svgW: number,
  svgH: number
): string {
  return pts
    .map((p) => {
      const { px, py } = fieldToSvg(p.x, p.y, L, W, svgW, svgH);
      return `${px.toFixed(2)},${py.toFixed(2)}`;
    })
    .join(' ');
}

export default function AutoPathPreview() {
  const pathNames = useMemo(() => listDeployPathNames(), []);
  const autoNames = useMemo(() => listDeployAutoNames(), []);

  const [mode, setMode] = useState<PreviewMode>('path');
  const [pathName, setPathName] = useState(pathNames[0] ?? '');
  const [autoName, setAutoName] = useState(autoNames[0] ?? '');
  const [showFlipped, setShowFlipped] = useState(true);
  const [flipSize, setFlipSize] = useState<FieldSize>(defaultFieldSize());

  const { blueLines, orangeLines, error } = useMemo(() => {
    const err = (msg: string) => ({ blueLines: [] as PathPoint[][], orangeLines: [] as PathPoint[][], error: msg });

    if (mode === 'path') {
      const raw = getDeployPathRaw(pathName);
      if (!raw) {
        return err(`Path "${pathName}" not found in deploy.`);
      }
      const parsed = parsePathJson(raw);
      if (!parsed) {
        return err('Invalid path JSON.');
      }
      const line = samplePathPolyline(parsed);
      const flipped = flipPolylineRotational(line, flipSize);
      return { blueLines: [line], orangeLines: showFlipped ? [flipped] : [], error: null as string | null };
    }

    const autoRaw = getDeployAutoRaw(autoName);
    if (!autoRaw) {
      return err(`Auto "${autoName}" not found in deploy.`);
    }
    const pathNameList = collectPathNamesFromAutoJson(autoRaw);
    if (pathNameList.length === 0) {
      return err('No path commands in this auto.');
    }
    const blueSegs: PathPoint[][] = [];
    const orangeSegs: PathPoint[][] = [];
    for (const pn of pathNameList) {
      const pr = getDeployPathRaw(pn);
      if (!pr) {
        return err(`Missing path file for "${pn}".`);
      }
      const parsed = parsePathJson(pr);
      if (!parsed) {
        return err(`Invalid path JSON: ${pn}`);
      }
      const line = samplePathPolyline(parsed);
      blueSegs.push(line);
      if (showFlipped) {
        orangeSegs.push(flipPolylineRotational(line, flipSize));
      }
    }
    return { blueLines: blueSegs, orangeLines: orangeSegs, error: null as string | null };
  }, [mode, pathName, autoName, showFlipped, flipSize]);

  const svgW = 720;
  const svgH = Math.round(svgW * (FIELD_WIDTH_M / FIELD_LENGTH_M));

  return (
    <div className="panel" style={{ marginTop: '1rem' }}>
      <div className="panel-header">
        <h3>Path preview (deploy JSON)</h3>
      </div>
      <p className="nt-hint" style={{ marginBottom: '0.75rem' }}>
        Blue: coordinates as stored in <code>paths/*.path</code>. Orange: PathPlanner{' '}
        <strong>rotational</strong> flip — same as <code>FlippingUtil</code> default (field center 180°):{' '}
        <code>
          x&apos; = L − x, y&apos; = W − y
        </code>
        . Match <code>fieldSizeX/Y</code> to your PathPlanner GUI if the preview is offset.
      </p>
      <div className="control-row" style={{ flexWrap: 'wrap', gap: '0.5rem', marginBottom: '0.5rem' }}>
        <label>Source</label>
        <select value={mode} onChange={(e) => setMode(e.target.value as PreviewMode)}>
          <option value="path">Single path</option>
          <option value="auto">Auto (chained paths)</option>
        </select>
        {mode === 'path' ? (
          <select value={pathName} onChange={(e) => setPathName(e.target.value)} disabled={pathNames.length === 0}>
            {pathNames.length === 0 ? (
              <option value="">No .path files in glob</option>
            ) : (
              pathNames.map((n) => (
                <option key={n} value={n}>
                  {n}
                </option>
              ))
            )}
          </select>
        ) : (
          <select value={autoName} onChange={(e) => setAutoName(e.target.value)} disabled={autoNames.length === 0}>
            {autoNames.length === 0 ? (
              <option value="">No .auto files in glob</option>
            ) : (
              autoNames.map((n) => (
                <option key={n} value={n}>
                  {n}
                </option>
              ))
            )}
          </select>
        )}
        <label>
          <input
            type="checkbox"
            checked={showFlipped}
            onChange={(e) => setShowFlipped(e.target.checked)}
          />{' '}
          Show red flip
        </label>
      </div>
      <div className="control-row" style={{ marginBottom: '0.75rem' }}>
        <label>Flip L (m)</label>
        <input
          type="number"
          step={0.01}
          value={flipSize.sizeXM}
          onChange={(e) =>
            setFlipSize((s) => ({ ...s, sizeXM: Number(e.target.value) || FIELD_LENGTH_M }))
          }
        />
        <label>W (m)</label>
        <input
          type="number"
          step={0.01}
          value={flipSize.sizeYM}
          onChange={(e) =>
            setFlipSize((s) => ({ ...s, sizeYM: Number(e.target.value) || FIELD_WIDTH_M }))
          }
        />
        <button type="button" onClick={() => setFlipSize(defaultFieldSize())}>
          Reset to dashboard field
        </button>
      </div>
      {error && <p className="nt-hint" style={{ color: '#f87171' }}>{error}</p>}
      {!error && (
        <svg
          width="100%"
          viewBox={`0 0 ${svgW} ${svgH}`}
          style={{ maxHeight: 420, background: '#0f172a', borderRadius: 8, border: '1px solid #334155' }}
          aria-label="Path preview field"
        >
          <rect x={0} y={0} width={svgW} height={svgH} fill="#0b1222" stroke="#334155" strokeWidth={1} />
          <text x={8} y={18} fill="#94a3b8" fontSize={12}>
            X→ {FIELD_LENGTH_M} m · Y↑ {FIELD_WIDTH_M} m (blue origin corner)
          </text>
          {showFlipped &&
            orangeLines.map((seg, i) => (
              <polyline
                key={`o-${i}`}
                fill="none"
                stroke="#f97316"
                strokeWidth={3}
                strokeLinejoin="round"
                strokeLinecap="round"
                opacity={0.95}
                points={polylineToSvgPoints(seg, FIELD_LENGTH_M, FIELD_WIDTH_M, svgW, svgH)}
              />
            ))}
          {blueLines.map((seg, i) => (
            <polyline
              key={`b-${i}`}
              fill="none"
              stroke="#38bdf8"
              strokeWidth={2.5}
              strokeLinejoin="round"
              strokeLinecap="round"
              points={polylineToSvgPoints(seg, FIELD_LENGTH_M, FIELD_WIDTH_M, svgW, svgH)}
            />
          ))}
        </svg>
      )}
    </div>
  );
}
