import { useMemo, useState } from 'react';
import {
  formatKnnShootTargetSummary,
  inferKnn,
  KNN_GRID_CELL_SIZE_M,
  knnMapPointCellIndex,
  knnMapPointFieldYForAlliance,
} from '../lib/knnInference';
import type { KnnPoint } from '../lib/knnInference';

type Props = {
  fieldLengthM: number;
  fieldWidthM: number;
  loggedPoints: KnnPoint[];
  poseX: number;
  poseY: number;
  headingDeg?: number;
  /** Red alliance: flips map marker Y for display only (WPIBlue lookup is unchanged). */
  isRedAlliance?: boolean;
  /** If set, highlight this index as "robot-selected" (from NT) in addition to local inference */
  robotSelectedIndex?: number | null;
  /** From NT /KNN/nearestIndexBlue when connected */
  knnNearestIndexBlue?: number;
  /** From NT /KNN/nearestIndexRed when connected */
  knnNearestIndexRed?: number;
  connected?: boolean;
  onPointChange?: (index: number, field: keyof KnnPoint, value: number) => void;
  onPointRemove?: (index: number) => void;
  /** Add {@code deltaRpm} to {@code shooterRpm} for every map row whose (x,y) falls in one of the cells. */
  onApplyRpmDeltaToCells?: (
    cells: { ci: number; cj: number }[],
    deltaRpm: number
  ) => void;
};

export default function KnnGridView({
  fieldLengthM,
  fieldWidthM,
  loggedPoints,
  poseX,
  poseY,
  headingDeg,
  isRedAlliance = false,
  robotSelectedIndex = null,
  knnNearestIndexBlue = -1,
  knnNearestIndexRed = -1,
  connected = false,
  onPointChange,
  onPointRemove,
  onApplyRpmDeltaToCells,
}: Props) {
  const [selectedCellKeys, setSelectedCellKeys] = useState<Set<string>>(
    () => new Set()
  );
  const [rpmDeltaStr, setRpmDeltaStr] = useState('50');

  const inference = useMemo(
    () =>
      inferKnn({ x: poseX, y: poseY, headingDeg }, loggedPoints, {
        k: 1,
        fieldWidthM,
      }),
    [poseX, poseY, headingDeg, loggedPoints, fieldWidthM]
  );

  const nearestBlueDisplay = useMemo(() => {
    if (connected && knnNearestIndexBlue >= 0) {
      return knnNearestIndexBlue;
    }
    return inference?.inferredIndex ?? -1;
  }, [connected, knnNearestIndexBlue, inference?.inferredIndex]);

  const nearestRedDisplay = useMemo(() => {
    if (connected && knnNearestIndexRed >= 0) {
      return knnNearestIndexRed;
    }
    return nearestBlueDisplay;
  }, [connected, knnNearestIndexRed, nearestBlueDisplay]);

  const gridState = useMemo(() => {
    const cols = Math.ceil(fieldLengthM / KNN_GRID_CELL_SIZE_M);
    const rows = Math.ceil(fieldWidthM / KNN_GRID_CELL_SIZE_M);
    const inferredIndex = inference?.inferredIndex ?? -1;

    const poseCellCol = Math.floor(
      Math.max(0, Math.min(poseX, fieldLengthM - 1e-6)) / KNN_GRID_CELL_SIZE_M
    );
    const poseCellRow = Math.floor(
      Math.max(0, Math.min(poseY, fieldWidthM - 1e-6)) / KNN_GRID_CELL_SIZE_M
    );

    const inferredPoint =
      inferredIndex >= 0 && inferredIndex < loggedPoints.length
        ? loggedPoints[inferredIndex]
        : null;
    const inferredYField =
      inferredPoint != null
        ? knnMapPointFieldYForAlliance(inferredPoint.y, isRedAlliance, fieldWidthM)
        : 0;
    const inferredCellCol =
      inferredPoint != null
        ? Math.floor(
            Math.max(0, Math.min(inferredPoint.x, fieldLengthM - 1e-6)) /
              KNN_GRID_CELL_SIZE_M
          )
        : -1;
    const inferredCellRow =
      inferredPoint != null
        ? Math.floor(
            Math.max(0, Math.min(inferredYField, fieldWidthM - 1e-6)) /
              KNN_GRID_CELL_SIZE_M
          )
        : -1;

    const robotPoint =
      robotSelectedIndex != null &&
      robotSelectedIndex >= 0 &&
      robotSelectedIndex < loggedPoints.length
        ? loggedPoints[robotSelectedIndex]
        : null;
    const robotYField =
      robotPoint != null
        ? knnMapPointFieldYForAlliance(robotPoint.y, isRedAlliance, fieldWidthM)
        : 0;
    const robotCellCol =
      robotPoint != null
        ? Math.floor(
            Math.max(0, Math.min(robotPoint.x, fieldLengthM - 1e-6)) /
              KNN_GRID_CELL_SIZE_M
          )
        : -1;
    const robotCellRow =
      robotPoint != null
        ? Math.floor(
            Math.max(0, Math.min(robotYField, fieldWidthM - 1e-6)) /
              KNN_GRID_CELL_SIZE_M
          )
        : -1;

    return {
      cols,
      rows,
      inferredIndex,
      poseCellCol,
      poseCellRow,
      inferredCellCol,
      inferredCellRow,
      robotCellCol,
      robotCellRow,
    };
  }, [
    fieldLengthM,
    fieldWidthM,
    poseX,
    poseY,
    loggedPoints,
    inference?.inferredIndex,
    isRedAlliance,
    robotSelectedIndex,
  ]);

  const selectedRegionPointCount = useMemo(() => {
    if (selectedCellKeys.size === 0) {
      return 0;
    }
    return loggedPoints.reduce((n, pt) => {
      const { ci, cj } = knnMapPointCellIndex(
        pt.x,
        pt.y,
        fieldLengthM,
        fieldWidthM
      );
      return n + (selectedCellKeys.has(`${ci},${cj}`) ? 1 : 0);
    }, 0);
  }, [loggedPoints, selectedCellKeys, fieldLengthM, fieldWidthM]);

  const toggleCellSelection = (ci: number, cj: number) => {
    if (!onApplyRpmDeltaToCells) {
      return;
    }
    const key = `${ci},${cj}`;
    setSelectedCellKeys((prev) => {
      const next = new Set(prev);
      if (next.has(key)) {
        next.delete(key);
      } else {
        next.add(key);
      }
      return next;
    });
  };

  const handleApplyRpmDelta = () => {
    if (!onApplyRpmDeltaToCells || selectedCellKeys.size === 0) {
      return;
    }
    const delta = Number(rpmDeltaStr);
    if (!Number.isFinite(delta)) {
      return;
    }
    const cells = Array.from(selectedCellKeys).map((key) => {
      const [ci, cj] = key.split(',').map(Number);
      return { ci, cj };
    });
    onApplyRpmDeltaToCells(cells, delta);
    setSelectedCellKeys(new Set());
  };

  const {
    cols,
    rows,
    inferredIndex,
    poseCellCol,
    poseCellRow,
    inferredCellCol,
    inferredCellRow,
    robotCellCol,
    robotCellRow,
  } = gridState;

  if (loggedPoints.length === 0) {
    return (
      <section className="subsystem-view knn-grid-view">
        <div className="panel knn-panel">
          <div className="panel-header">
            <h2>KNN Grid</h2>
          </div>
          <p className="nt-hint">
            Load a KNN log to see the grid and inferred point for the current
            chassis pose.
          </p>
        </div>
      </section>
    );
  }

  return (
    <section className="subsystem-view knn-grid-view">
      <div className="panel knn-panel">
        <div className="panel-header">
          <h2>KNN Grid</h2>
          <span className="status-pill neutral">
            {inferredIndex >= 0 ? `Inferred: #${inferredIndex}` : 'No points'}
            {nearestBlueDisplay >= 0 ? (
              <span title="Geometric nearest map row, raw WPIBlue pose">
                {' '}
                · B{nearestBlueDisplay}
              </span>
            ) : null}
            {nearestRedDisplay >= 0 && nearestRedDisplay !== nearestBlueDisplay ? (
              <span title="NT nearestIndexRed (mirrored lookup pose on red when enabled)">
                {' '}
                · R{nearestRedDisplay}
              </span>
            ) : null}
          </span>
        </div>

        <div className="knn-grid-layout">
          <div className="knn-spatial-grid-wrap">
            {onApplyRpmDeltaToCells ? (
              <div className="knn-region-rpm-bar control-row">
                <label htmlFor="knn-rpm-delta">Δ RPM (selected cells)</label>
                <input
                  id="knn-rpm-delta"
                  className="knn-rpm-delta-input"
                  type="number"
                  step="10"
                  value={rpmDeltaStr}
                  onChange={(e) => setRpmDeltaStr(e.target.value)}
                  title="Added to shooterRpm for every map row whose (x, y) lies in a highlighted cell"
                />
                <button
                  type="button"
                  className="knn-apply-rpm-btn"
                  disabled={
                    selectedCellKeys.size === 0 ||
                    !Number.isFinite(Number(rpmDeltaStr))
                  }
                  onClick={handleApplyRpmDelta}
                  title="Persist to map (same as table edits); export JSON to update deploy file"
                >
                  Apply ΔRPM
                </button>
                <button
                  type="button"
                  onClick={() => setSelectedCellKeys(new Set())}
                  disabled={selectedCellKeys.size === 0}
                >
                  Clear selection
                </button>
                <span className="knn-region-rpm-meta">
                  {selectedCellKeys.size} cell
                  {selectedCellKeys.size === 1 ? '' : 's'} · affects{' '}
                  {selectedRegionPointCount} point
                  {selectedRegionPointCount === 1 ? '' : 's'}
                </span>
              </div>
            ) : null}
            <div
              className="knn-spatial-grid knn-spatial-grid--wpilib-blue-origin"
              style={{
                display: 'grid',
                gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))`,
                gridTemplateRows: `repeat(${rows}, minmax(0, 1fr))`,
                aspectRatio: `${fieldLengthM} / ${fieldWidthM}`,
                maxHeight: 360,
              }}
            >
              {Array.from({ length: rows * cols }, (_, idx) => {
                const cj = Math.floor(idx / cols);
                const ci = idx % cols;
                const isPose =
                  ci === poseCellCol && cj === poseCellRow;
                const isInferred =
                  inferredIndex >= 0 &&
                  ci === inferredCellCol &&
                  cj === inferredCellRow;
                const isRobotNtCell =
                  robotCellCol >= 0 &&
                  robotCellRow >= 0 &&
                  ci === robotCellCol &&
                  cj === robotCellRow;
                const cellKey = `${ci},${cj}`;
                const isRegionSelected = selectedCellKeys.has(cellKey);
                const classes = [
                  'knn-cell',
                  onApplyRpmDeltaToCells && 'knn-cell-selectable',
                  isPose && 'knn-cell-pose',
                  isInferred && 'knn-cell-inferred',
                  isRobotNtCell && 'knn-cell-robot-selected',
                  isRegionSelected && 'knn-cell-region-selected',
                ]
                  .filter(Boolean)
                  .join(' ');
                return (
                  <div
                    key={idx}
                    role={onApplyRpmDeltaToCells ? 'button' : undefined}
                    tabIndex={onApplyRpmDeltaToCells ? 0 : undefined}
                    className={classes}
                    title={
                      onApplyRpmDeltaToCells
                        ? `Click to highlight · Cell (${ci}, ${cj}) · WPIBlue 1 m tiles`
                        : `Cell (${ci}, ${cj})`
                    }
                    onClick={
                      onApplyRpmDeltaToCells
                        ? () => toggleCellSelection(ci, cj)
                        : undefined
                    }
                    onKeyDown={
                      onApplyRpmDeltaToCells
                        ? (e) => {
                            if (e.key === 'Enter' || e.key === ' ') {
                              e.preventDefault();
                              toggleCellSelection(ci, cj);
                            }
                          }
                        : undefined
                    }
                  />
                );
              })}
            </div>
            <p className="knn-grid-hint">
              Field (0,0) bottom-left — WPIBlue · Blue = pose · Green = inferred cell · Gold ={' '}
              <code>/KNN/selectedIndex</code> · B = geometric nearest (fused WPIBlue pose,{' '}
              <code>/KNN/nearestIndexBlue</code>) · R differs when red uses mirrored KNN lookup
              {onApplyRpmDeltaToCells
                ? ' · Click cells to highlight a region, set ΔRPM, Apply (updates map / export JSON).'
                : ''}
            </p>
          </div>

          <div className="knn-table-wrap">
            <table className="knn-points-table">
              <thead>
                <tr>
                  <th>Index</th>
                  <th>X (m)</th>
                  <th>Y (m)</th>
                  <th>Heading (deg)</th>
                  <th>Shooter RPM</th>
                  <th>Hood (deg)</th>
                  <th>Shoot target</th>
                  <th>Inferred</th>
                  <th>NT idx</th>
                  <th>Action</th>
                </tr>
              </thead>
              <tbody>
                {loggedPoints.map((pt, idx) => {
                  const { ci: rowCi, cj: rowCj } = knnMapPointCellIndex(
                    pt.x,
                    pt.y,
                    fieldLengthM,
                    fieldWidthM
                  );
                  const inSelectedRegion =
                    selectedCellKeys.size > 0 &&
                    selectedCellKeys.has(`${rowCi},${rowCj}`);
                  return (
                  <tr
                    key={idx}
                    className={[
                      idx === inferredIndex ? 'knn-row-inferred' : '',
                      robotSelectedIndex != null && idx === robotSelectedIndex
                        ? 'knn-row-nt-selected'
                        : '',
                      inSelectedRegion ? 'knn-row-region-selected' : '',
                    ]
                      .filter(Boolean)
                      .join(' ')}
                  >
                    <td>{idx}</td>
                    <td>
                      {onPointChange ? (
                        <input
                          className="knn-point-input"
                          type="number"
                          value={pt.x}
                          onChange={(event) =>
                            onPointChange(idx, 'x', Number(event.target.value) || 0)
                          }
                        />
                      ) : (
                        pt.x.toFixed(2)
                      )}
                    </td>
                    <td>
                      {onPointChange ? (
                        <input
                          className="knn-point-input"
                          type="number"
                          value={pt.y}
                          onChange={(event) =>
                            onPointChange(idx, 'y', Number(event.target.value) || 0)
                          }
                        />
                      ) : (
                        pt.y.toFixed(2)
                      )}
                    </td>
                    <td>
                      {onPointChange ? (
                        <input
                          className="knn-point-input"
                          type="number"
                          value={pt.headingDeg ?? 0}
                          onChange={(event) =>
                            onPointChange(idx, 'headingDeg', Number(event.target.value) || 0)
                          }
                        />
                      ) : (
                        (pt.headingDeg ?? 0).toFixed(1)
                      )}
                    </td>
                    <td>
                      {onPointChange ? (
                        <input
                          className="knn-point-input"
                          type="number"
                          value={pt.shooterRpm ?? 0}
                          onChange={(event) =>
                            onPointChange(idx, 'shooterRpm', Number(event.target.value) || 0)
                          }
                        />
                      ) : (
                        (pt.shooterRpm ?? 0).toFixed(0)
                      )}
                    </td>
                    <td>
                      {onPointChange ? (
                        <input
                          className="knn-point-input"
                          type="number"
                          value={pt.hoodDeg ?? 0}
                          onChange={(event) =>
                            onPointChange(idx, 'hoodDeg', Number(event.target.value) || 0)
                          }
                        />
                      ) : (
                        (pt.hoodDeg ?? 0).toFixed(1)
                      )}
                    </td>
                    <td className="knn-shoot-target-cell" title="Saved in map; export includes shootTarget">
                      {formatKnnShootTargetSummary(pt)}
                    </td>
                    <td>{idx === inferredIndex ? 'Yes' : '—'}</td>
                    <td title="Robot /KNN/selectedIndex (same index as knn_map.json row)">
                      {robotSelectedIndex != null && idx === robotSelectedIndex ? '✓' : '—'}
                    </td>
                    <td>
                      {onPointRemove ? (
                        <button
                          type="button"
                          className="knn-row-delete"
                          onClick={() => onPointRemove(idx)}
                        >
                          Delete
                        </button>
                      ) : (
                        '—'
                      )}
                    </td>
                  </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </section>
  );
}

