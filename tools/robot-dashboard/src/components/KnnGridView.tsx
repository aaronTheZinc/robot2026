import { useMemo } from 'react';
import { inferKnn } from '../lib/knnInference';
import type { KnnPoint } from '../lib/knnInference';

const CELL_SIZE_M = 1.0;

type Props = {
  fieldLengthM: number;
  fieldWidthM: number;
  loggedPoints: KnnPoint[];
  poseX: number;
  poseY: number;
  headingDeg?: number;
  /** If set, highlight this index as "robot-selected" (from NT) in addition to local inference */
  robotSelectedIndex?: number | null;
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
  robotSelectedIndex = null,
  onPointChange,
  onPointRemove,
}: Props) {
  const inference = useMemo(
    () =>
      inferKnn(
        { x: poseX, y: poseY, headingDeg },
        loggedPoints,
        {},
        1
      ),
    [poseX, poseY, headingDeg, loggedPoints]
  );

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
            Math.max(0, Math.min(inferredPoint.y, fieldWidthM - 1e-6)) /
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
    };
  }, [
    fieldLengthM,
    fieldWidthM,
    poseX,
    poseY,
    loggedPoints,
    inference?.inferredIndex,
  ]);

  const {
    cols,
    rows,
    inferredIndex,
    poseCellCol,
    poseCellRow,
    inferredCellCol,
    inferredCellRow,
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
          </span>
        </div>

        <div className="knn-grid-layout">
          <div className="knn-spatial-grid-wrap">
            <div
              className="knn-spatial-grid"
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
                  ci === inferredCellCol && cj === inferredCellRow;
                const isRobotSelected =
                  robotSelectedIndex != null &&
                  inferredIndex >= 0 &&
                  robotSelectedIndex === inferredIndex;
                const classes = [
                  'knn-cell',
                  isPose && 'knn-cell-pose',
                  isInferred && 'knn-cell-inferred',
                  isRobotSelected && isInferred && 'knn-cell-robot-selected',
                ]
                  .filter(Boolean)
                  .join(' ');
                return (
                  <div key={idx} className={classes} title={`Cell (${ci}, ${cj})`} />
                );
              })}
            </div>
            <p className="knn-grid-hint">
              Blue = robot pose cell · Green = inferred point cell
              {robotSelectedIndex != null && ' · Robot selection matches'}
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
                  <th>Inferred</th>
                  <th>Action</th>
                </tr>
              </thead>
              <tbody>
                {loggedPoints.map((pt, idx) => (
                  <tr
                    key={idx}
                    className={
                      idx === inferredIndex ? 'knn-row-inferred' : ''
                    }
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
                    <td>{idx === inferredIndex ? 'Yes' : '—'}</td>
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

