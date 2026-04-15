/**
 * KNN inference for the dashboard: given chassis pose and logged points,
 * returns which point index is "inferred" (nearest) so the grid view and
 * robot interpreter can align on the same selection.
 *
 * Red alliance: matches {@code KnnInterpreter} / {@code KnnConstants#kMirrorPoseYForRedAllianceKnnLookup}
 * — distance uses pose {@code (x, W - y)} in the WPIBlue frame so blue-recorded map rows match.
 */

import { FIELD_WIDTH_M } from './fieldDimensions';

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
   * When true (default), match on-robot KNN: on red alliance use {@code y' = fieldWidthM - y} for distance.
   * Set false only for debugging.
   */
  mirrorRedAllianceLookup?: boolean;
  /** Field width W (m) for Y mirror; default {@link FIELD_WIDTH_M} (must match {@code KnnConstants.kFieldWidthYMeters}). */
  fieldWidthM?: number;
  /** When true, apply Y mirror (red alliance in WPIBlue frame). */
  isRedAlliance?: boolean;
};

const DEFAULT_CONFIG: Required<
  Pick<KnnInferenceConfig, 'weightX' | 'weightY' | 'weightRotation' | 'mirrorRedAllianceLookup'>
> & { fieldWidthM: number; isRedAlliance: boolean } = {
  weightX: 1,
  weightY: 1,
  weightRotation: 0,
  mirrorRedAllianceLookup: true,
  fieldWidthM: FIELD_WIDTH_M,
  isRedAlliance: false,
};

/**
 * Same transform as {@code KnnInterpreter.poseForKnnLookup}: WPIBlue pose, red alliance uses {@code y' = W - y}.
 */
export function poseForKnnLookup(
  pose: { x: number; y: number; headingDeg?: number },
  options: {
    mirrorRedAllianceLookup?: boolean;
    fieldWidthM?: number;
    isRedAlliance?: boolean;
  } = {}
): { x: number; y: number; headingDeg?: number } {
  const mirror = options.mirrorRedAllianceLookup !== false;
  const w = options.fieldWidthM ?? FIELD_WIDTH_M;
  const red = options.isRedAlliance === true;
  if (!mirror || !red) {
    return pose;
  }
  return { ...pose, y: w - pose.y };
}

/**
 * Y coordinate (m) for placing a {@code knn_map.json} row on the WPIBlue field in the UI. Map rows are stored in
 * blue-side coordinates; on red alliance the robot compares using {@link poseForKnnLookup}, so markers and grid
 * cells should use the symmetric {@code W - y} so the selected sample appears on the correct half-field.
 */
export function knnMapPointFieldYForAlliance(
  mapY: number,
  isRedAlliance: boolean,
  fieldWidthM: number = FIELD_WIDTH_M
): number {
  return isRedAlliance ? fieldWidthM - mapY : mapY;
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
    mirrorRedAllianceLookup: config.mirrorRedAllianceLookup ?? DEFAULT_CONFIG.mirrorRedAllianceLookup,
    isRedAlliance: config.isRedAlliance ?? DEFAULT_CONFIG.isRedAlliance,
  };
  const kWant = legacyK ?? config.k ?? 1;
  const effectiveK = Math.max(1, Math.min(kWant, points.length));

  const lookupPose = poseForKnnLookup(pose, {
    mirrorRedAllianceLookup: cfg.mirrorRedAllianceLookup,
    fieldWidthM: cfg.fieldWidthM,
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
