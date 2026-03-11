/**
 * KNN inference for the dashboard: given chassis pose and logged points,
 * returns which point index is "inferred" (nearest) so the grid view and
 * robot interpreter can align on the same selection.
 */

export type KnnPoint = {
  x: number;
  y: number;
  headingDeg?: number;
  shooterRpm?: number;
  hoodDeg?: number;
};

export type KnnInferenceConfig = {
  /** Weight for x distance (default 1) */
  weightX?: number;
  /** Weight for y distance (default 1) */
  weightY?: number;
  /** Weight for rotation delta in degrees (default 0 = ignore rotation) */
  weightRotation?: number;
};

const DEFAULT_CONFIG: Required<KnnInferenceConfig> = {
  weightX: 1,
  weightY: 1,
  weightRotation: 0,
};

function distance(
  pose: { x: number; y: number },
  point: KnnPoint,
  config: Required<KnnInferenceConfig>
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

/**
 * Given current pose and logged points, returns the inferred (nearest) point
 * index and optionally top-k indices and distances.
 */
export function inferKnn(
  pose: { x: number; y: number; headingDeg?: number },
  points: KnnPoint[],
  config: KnnInferenceConfig = {},
  k: number = 1
): KnnInferenceResult | null {
  if (points.length === 0) {
    return null;
  }

  const cfg = { ...DEFAULT_CONFIG, ...config };
  const effectiveK = Math.max(1, Math.min(k, points.length));

  const scored = points
    .map((point, index) => ({
      index,
      d: distance(
        pose,
        point,
        cfg as Required<KnnInferenceConfig>
      ),
    }))
    .sort((a, b) => a.d - b.d)
    .slice(0, effectiveK);

  return {
    inferredIndex: scored[0].index,
    inferredIndices: scored.map((s) => s.index),
    distances: scored.map((s) => s.d),
  };
}
