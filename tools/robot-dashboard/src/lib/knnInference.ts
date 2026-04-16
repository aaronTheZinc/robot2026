/**
 * KNN inference for the dashboard: given chassis pose (WPIBlue, same as fused /Pose/robotPose) and logged
 * points, returns which point index is nearest. Matches {@code KnnInterpreter} when
 * {@code kMirrorPoseAcrossFieldForRedAllianceKnnLookup} is false: compare pose to map rows directly.
 */

import { FIELD_LENGTH_M, FIELD_WIDTH_M } from './fieldDimensions';

/** Scoring / inspection aim for this map sample. Omitted or `hub` = hub shot (default). */
export type KnnShootTarget =
  | { kind: 'hub' }
  | { kind: 'field'; x: number; y: number; label?: string };

export type KnnPoint = {
  x: number;
  y: number;
  headingDeg?: number;
  shooterRpm?: number;
  hoodDeg?: number;
  /** Persisted aim for simulation / export; default is hub when absent. */
  shootTarget?: KnnShootTarget;
};

export function normalizeKnnShootTarget(raw: unknown): KnnShootTarget | undefined {
  if (raw === undefined || raw === null) {
    return undefined;
  }
  if (typeof raw !== 'object') {
    return undefined;
  }
  const o = raw as Record<string, unknown>;
  if (o.kind === 'hub') {
    return { kind: 'hub' };
  }
  if (
    o.kind === 'field' &&
    typeof o.x === 'number' &&
    typeof o.y === 'number' &&
    Number.isFinite(o.x) &&
    Number.isFinite(o.y)
  ) {
    const label = typeof o.label === 'string' && o.label.trim().length > 0 ? o.label.trim() : undefined;
    return { kind: 'field', x: o.x, y: o.y, ...(label ? { label } : {}) };
  }
  return undefined;
}

/** Default aim for new or legacy points (not stored on disk until export if you omit the key). */
export function defaultKnnShootTarget(): KnnShootTarget {
  return { kind: 'hub' };
}

export function formatKnnShootTargetSummary(point: KnnPoint): string {
  const st = point.shootTarget;
  if (st?.kind === 'field') {
    const coord = `(${st.x.toFixed(2)}, ${st.y.toFixed(2)})`;
    return st.label ? `${st.label} ${coord}` : coord;
  }
  return 'Hub';
}

export type KnnInferenceConfig = {
  /** Weight for x distance (default 1) */
  weightX?: number;
  /** Weight for y distance (default 1) */
  weightY?: number;
  /** Weight for rotation delta in degrees (default 0 = ignore rotation) */
  weightRotation?: number;
  /**
   * When true, on red alliance use {@code (L-x, W-y)} for distance (matches robot
   * {@code kMirrorPoseAcrossFieldForRedAllianceKnnLookup}). Default true.
   */
  mirrorRedAllianceLookup?: boolean;
  /** Field width W (m); default {@link FIELD_WIDTH_M} (must match {@code KnnConstants.kFieldWidthYMeters}). */
  fieldWidthM?: number;
  /** Field length L (m); default {@link FIELD_LENGTH_M} (must match {@code KnnConstants.kFieldLengthXMeters}). */
  fieldLengthM?: number;
  /** When true, apply red-alliance field mirror for lookup (with {@code mirrorRedAllianceLookup}). */
  isRedAlliance?: boolean;
};

const DEFAULT_CONFIG: Required<
  Pick<KnnInferenceConfig, 'weightX' | 'weightY' | 'weightRotation' | 'mirrorRedAllianceLookup'>
> & { fieldWidthM: number; fieldLengthM: number; isRedAlliance: boolean } = {
  weightX: 1,
  weightY: 1,
  weightRotation: 0,
  mirrorRedAllianceLookup: true,
  fieldWidthM: FIELD_WIDTH_M,
  fieldLengthM: FIELD_LENGTH_M,
  isRedAlliance: false,
};

/**
 * Optional field mirror for red alliance. When {@code mirrorRedAllianceLookup} is false, returns pose unchanged.
 */
export function poseForKnnLookup(
  pose: { x: number; y: number; headingDeg?: number },
  options: {
    mirrorRedAllianceLookup?: boolean;
    fieldWidthM?: number;
    fieldLengthM?: number;
    isRedAlliance?: boolean;
  } = {}
): { x: number; y: number; headingDeg?: number } {
  const mirror = options.mirrorRedAllianceLookup !== false;
  const w = options.fieldWidthM ?? FIELD_WIDTH_M;
  const L = options.fieldLengthM ?? FIELD_LENGTH_M;
  const red = options.isRedAlliance === true;
  if (!mirror || !red) {
    return pose;
  }
  return { ...pose, x: L - pose.x, y: w - pose.y };
}

/**
 * Y coordinate (m) for placing a {@code knn_map.json} row on the field in the UI when the view is flipped for red
 * alliance display. Map rows stay in WPIBlue storage; this only affects drawing.
 */
export function knnMapPointFieldYForAlliance(
  mapY: number,
  isRedAlliance: boolean,
  fieldWidthM: number = FIELD_WIDTH_M
): number {
  return isRedAlliance ? fieldWidthM - mapY : mapY;
}

/** Grid cell size (m) for the dashboard KNN spatial grid (1 m tiles over the field). */
export const KNN_GRID_CELL_SIZE_M = 1.0;

/**
 * Which grid cell (WPIBlue) a map sample occupies — same indexing as {@code KnnGridView} cells.
 * {@code ci} = column along field length X, {@code cj} = row along field width Y.
 */
export function knnMapPointCellIndex(
  x: number,
  y: number,
  fieldLengthM: number,
  fieldWidthM: number,
  cellSizeM: number = KNN_GRID_CELL_SIZE_M
): { ci: number; cj: number } {
  const ci = Math.floor(
    Math.max(0, Math.min(x, fieldLengthM - 1e-6)) / cellSizeM
  );
  const cj = Math.floor(
    Math.max(0, Math.min(y, fieldWidthM - 1e-6)) / cellSizeM
  );
  return { ci, cj };
}

function distance(
  pose: { x: number; y: number },
  point: KnnPoint,
  config: typeof DEFAULT_CONFIG
): number {
  const dx = (pose.x - point.x) * config.weightX;
  const dy = (pose.y - point.y) * config.weightY;
  return Math.sqrt(dx * dx + dy * dy);
}

export type KnnInferenceResult = {
  /** Index of the nearest point in the points array */
  inferredIndex: number;
  /** Indices of the k nearest points (nearest first) */
  inferredIndices: number[];
  /** Distance to each of the k nearest points */
  distances: number[];
};

export type InferKnnOptions = KnnInferenceConfig & {
  /** How many nearest neighbors to return (default 1). */
  k?: number;
};

/**
 * Given current pose and logged points, returns the inferred (nearest) point
 * index and optionally top-k indices and distances.
 *
 * @param legacyK — optional 4th argument: nearest neighbor count (same as {@code options.k}).
 */
export function inferKnn(
  pose: { x: number; y: number; headingDeg?: number },
  points: KnnPoint[],
  config: InferKnnOptions = {},
  legacyK?: number
): KnnInferenceResult | null {
  if (points.length === 0) {
    return null;
  }

  const cfg: typeof DEFAULT_CONFIG = {
    ...DEFAULT_CONFIG,
    ...config,
    fieldWidthM: config.fieldWidthM ?? DEFAULT_CONFIG.fieldWidthM,
    fieldLengthM: config.fieldLengthM ?? DEFAULT_CONFIG.fieldLengthM,
    mirrorRedAllianceLookup: config.mirrorRedAllianceLookup ?? DEFAULT_CONFIG.mirrorRedAllianceLookup,
    isRedAlliance: config.isRedAlliance ?? DEFAULT_CONFIG.isRedAlliance,
  };
  const kWant = legacyK ?? config.k ?? 1;
  const effectiveK = Math.max(1, Math.min(kWant, points.length));

  const lookupPose = poseForKnnLookup(pose, {
    mirrorRedAllianceLookup: cfg.mirrorRedAllianceLookup,
    fieldWidthM: cfg.fieldWidthM,
    fieldLengthM: cfg.fieldLengthM,
    isRedAlliance: cfg.isRedAlliance,
  });

  const scored = points
    .map((point, index) => ({
      index,
      d: distance(lookupPose, point, cfg),
    }))
    .sort((a, b) => a.d - b.d)
    .slice(0, effectiveK);

  return {
    inferredIndex: scored[0].index,
    inferredIndices: scored.map((s) => s.index),
    distances: scored.map((s) => s.d),
  };
}
