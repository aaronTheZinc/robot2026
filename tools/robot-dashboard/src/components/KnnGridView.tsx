import { useMemo } from 'react';
import {
  formatKnnShootTargetSummary,
  inferKnn,
  knnMapPointFieldYForAlliance,
} from '../lib/knnInference';
import type { KnnPoint } from '../lib/knnInference';

const CELL_SIZE_M = 1.0;

type Props = {
  fieldLengthM: number;
  fieldWidthM: number;
  loggedPoints: KnnPoint[];
  poseX: number;
  poseY: number;
  headingDeg?: number;
  /** Match robot KNN red-side Y mirror ({@code FMSInfo/IsRedAlliance}). */
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
}: Props) {
  const inference = useMemo(
    () =>
      inferKnn(
        { x: poseX, y: poseY, headingDeg },
        loggedPoints,
        { k: 1, fieldWidthM: fieldWidthM, isRedAlliance }
      ),
    [poseX, poseY, headingDeg, loggedPoints, fieldWidthM, isRedAlliance]
  );

  const nearestBlueDisplay = useMemo(() => {
    if (connected && knnNearestIndexBlue >= 0) {
      return knnNearestIndexBlue;
    }
    const r = inferKnn(
      { x: poseX, y: poseY, headingDeg },
      loggedPoints,
      { k: 1, fieldWidthM, isRedAlliance: false, mirrorRedAllianceLookup: true }
    );
    return r?.inferredIndex ?? -1;
  }, [connected, knnNearestIndexBlue, poseX, poseY, headingDeg, loggedPoints, fieldWidthM]);

  const nearestRedDisplay = useMemo(() => {
    if (connected && knnNearestIndexRed >= 0) {
      return knnNearestIndexRed;
    }
    const r = inferKnn(
      { x: poseX, y: poseY, headingDeg },
      loggedPoints,
      { k: 1, fieldWidthM, isRedAlliance: true, mirrorRedAllianceLookup: true }
    );
    return r?.inferredIndex ?? -1;
  }, [connected, knnNearestIndexRed, poseX, poseY, headingDeg, loggedPoints, fieldWidthM]);

  const gridState = useMemo(() => {
    const cols = Math.ceil(fieldLengthM / CELL_SIZE_M);
    const rows = Math.ceil(fieldWidthM / CELL_SIZE_M);
    const inferredIndex = inference?.inferredIndex ?? -1;

    const poseCellCol = Math.floor(
      Math.max(0, Math.min(poseX, fieldLengthM - 1e-6)) / CELL_SIZE_M
    );
    const poseCellRow = Math.floor(
      Math.max(0, Math.min(poseY, fieldWidthM - 1e-6)) / CELL_SIZE_M
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
              CELL_SIZE_M
          )
        : -1;
    const inferredCellRow =
      inferredPoint != null
        ? Math.floor(
            Math.max(0, Math.min(inferredYField, fieldWidthM - 1e-6)) /
              CELL_SIZE_M
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
              CELL_SIZE_M
          )
        : -1;
    const robotCellRow =
      robotPoint != null
        ? Math.floor(
            Math.max(0, Math.min(robotYField, fieldWidthM - 1e-6)) /
              CELL_SIZE_M
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
            {nearestRedDisplay >= 0 ? (
              <span title="Geometric nearest map row, Y-mirrored pose (red lookup)">
                {' '}
                · R{nearestRedDisplay}
              </span>
            ) : null}
          </span>
        </div>

        <div className="knn-grid-layout">
          <div className="knn-spatial-grid-wrap">
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
                const classes = [
                  'knn-cell',
                  isPose && 'knn-cell-pose',
                  isInferred && 'knn-cell-inferred',
                  isRobotNtCell && 'knn-cell-robot-selected',
                ]
                  .filter(Boolean)
                  .join(' ');
                return (
                  <div key={idx} className={classes} title={`Cell (${ci}, ${cj})`} />
                );
              })}
            </div>
            <p className="knn-grid-hint">
              Field (0,0) bottom-left — WPIBlue · Blue = pose · Green = alliance-inferred cell · Gold ={' '}
              <code>/KNN/selectedIndex</code> · B/R = nearest row for raw vs Y-mirror pose (
              <code>/KNN/nearestIndexBlue</code>, <code>/KNN/nearestIndexRed</code>)
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
                {loggedPoints.map((pt, idx) => (
                  <tr
                    key={idx}
                    className={[
                      idx === inferredIndex ? 'knn-row-inferred' : '',
                      robotSelectedIndex != null && idx === robotSelectedIndex
                        ? 'knn-row-nt-selected'
                        : '',
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
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </section>
  );
}

