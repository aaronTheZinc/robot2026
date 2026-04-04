/**
 * Mirror KNN map samples across the field width midline (y = W/2).
 * WPILib field coords: X = length (m), Y = width (m), origin at a corner — same as PathPlanner.
 *
 * Reflection across the line y = W/2:
 *   x' = x
 *   y' = W − y
 *   headingDeg' = wrap180(−headingDeg)   // CCW-from-+X heading; mirrors like vector (vx, vy) → (vx, −vy)
 * Shooter RPM / hood: unchanged (symmetric robot).
 * shootTarget: hub unchanged; field aim point mirrors the same as pose: (fx, fy)' = (fx, W − fy).
 */

import type { KnnPoint, KnnShootTarget } from './knnInference';

/** Wrap degrees to (−180, 180]. */
export function wrapHeadingDeg180(deg: number): number {
  let d = deg % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return d;
}

function mirrorShootTarget(st: KnnShootTarget | undefined, fieldWidthM: number): KnnShootTarget | undefined {
  if (st === undefined) {
    return undefined;
  }
  if (st.kind === 'hub') {
    return { kind: 'hub' };
  }
  return {
    kind: 'field',
    x: st.x,
    y: fieldWidthM - st.y,
    ...(st.label !== undefined ? { label: st.label } : {}),
  };
}

/** One mirrored sample; does not mutate the input. */
export function mirrorKnnPointAcrossFieldWidth(point: KnnPoint, fieldWidthM: number): KnnPoint {
  const heading = point.headingDeg ?? 0;
  const next: KnnPoint = {
    x: point.x,
    y: fieldWidthM - point.y,
    headingDeg: wrapHeadingDeg180(-heading),
    shooterRpm: point.shooterRpm,
    hoodDeg: point.hoodDeg,
  };
  const st = mirrorShootTarget(point.shootTarget, fieldWidthM);
  if (st !== undefined) {
    next.shootTarget = st;
  }
  return next;
}

export type MirrorFilter = 'all' | 'yLow' | 'yHigh';

/** Whether this point should be mirrored for the given half-field filter (midline y = W/2). */
export function shouldMirrorPoint(y: number, fieldWidthM: number, filter: MirrorFilter): boolean {
  const mid = fieldWidthM / 2;
  if (filter === 'all') {
    return true;
  }
  if (filter === 'yLow') {
    return y < mid;
  }
  return y > mid;
}

/** Mirror each selected point and return new array (for append or clipboard). */
export function mirrorKnnPoints(
  points: KnnPoint[],
  fieldWidthM: number,
  filter: MirrorFilter
): KnnPoint[] {
  const out: KnnPoint[] = [];
  for (const p of points) {
    if (!shouldMirrorPoint(p.y, fieldWidthM, filter)) {
      continue;
    }
    out.push(mirrorKnnPointAcrossFieldWidth(p, fieldWidthM));
  }
  return out;
}
