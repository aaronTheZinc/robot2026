import { flipFieldPositionRotational, type FieldSize } from './pathPlannerFlip';

export type PathPoint = { x: number; y: number };

type WaypointJson = {
  anchor: { x: number; y: number };
  prevControl: { x: number; y: number } | null;
  nextControl: { x: number; y: number } | null;
};

export type PathPlannerPathJson = {
  version?: string;
  waypoints: WaypointJson[];
};

function cubicBezierPoint(
  p0: PathPoint,
  p1: PathPoint,
  p2: PathPoint,
  p3: PathPoint,
  t: number
): PathPoint {
  const u = 1 - t;
  const u2 = u * u;
  const u3 = u2 * u;
  const t2 = t * t;
  const t3 = t2 * t;
  return {
    x: u3 * p0.x + 3 * u2 * t * p1.x + 3 * u * t2 * p2.x + t3 * p3.x,
    y: u3 * p0.y + 3 * u2 * t * p1.y + 3 * u * t2 * p2.y + t3 * p3.y,
  };
}

/** Samples cubic segments between PathPlanner waypoints (same structure as GUI export). */
export function samplePathPolyline(path: PathPlannerPathJson, samplesPerSegment = 24): PathPoint[] {
  const wps = path.waypoints;
  if (!wps || wps.length === 0) {
    return [];
  }
  if (wps.length === 1) {
    return [{ x: wps[0].anchor.x, y: wps[0].anchor.y }];
  }
  const out: PathPoint[] = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const w0 = wps[i];
    const w1 = wps[i + 1];
    const p0 = { x: w0.anchor.x, y: w0.anchor.y };
    const p3 = { x: w1.anchor.x, y: w1.anchor.y };
    const p1 = w0.nextControl != null ? { x: w0.nextControl.x, y: w0.nextControl.y } : p0;
    const p2 = w1.prevControl != null ? { x: w1.prevControl.x, y: w1.prevControl.y } : p3;
    const n = samplesPerSegment;
    for (let s = 0; s < n; s++) {
      const t = s / n;
      out.push(cubicBezierPoint(p0, p1, p2, p3, t));
    }
  }
  const last = wps[wps.length - 1];
  out.push({ x: last.anchor.x, y: last.anchor.y });
  return out;
}

export function flipPolylineRotational(points: PathPoint[], size: FieldSize): PathPoint[] {
  return points.map((p) => flipFieldPositionRotational(p.x, p.y, size));
}
