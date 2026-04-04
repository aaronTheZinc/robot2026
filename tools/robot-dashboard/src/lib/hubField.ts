/**
 * Hub pose and hub-facing heading — mirrors {@code DriveConstants} on the robot
 * (PathPlanner / fused odometry field frame).
 */
export const HUB_FIELD_X_M = 4.62;
export const HUB_FIELD_Y_M = 4.0;
/** Degrees added to bearing toward hub (0 when scorer aligns with chassis +X; 180 if on the back). */
export const HUB_FACING_OFFSET_DEG = 0;

/** Normalize to (-180, 180]. */
export function normalizeHeadingDeg(deg: number): number {
  let d = ((deg + 180) % 360) - 180;
  if (d <= -180) d += 360;
  return d;
}

/**
 * Chassis heading (deg, field frame) toward an arbitrary field point, with {@link HUB_FACING_OFFSET_DEG}.
 */
export function optimalHeadingTowardFieldPointDeg(
  robotFieldX: number,
  robotFieldY: number,
  targetFieldX: number,
  targetFieldY: number
): number {
  const dx = targetFieldX - robotFieldX;
  const dy = targetFieldY - robotFieldY;
  if (dx * dx + dy * dy < 1e-12) {
    return 0;
  }
  const towardDeg = (Math.atan2(dy, dx) * 180) / Math.PI;
  return normalizeHeadingDeg(towardDeg + HUB_FACING_OFFSET_DEG);
}

/**
 * Chassis heading (deg, field frame) so the robot faces the hub per robot code
 * ({@code rotationToFaceHub}).
 */
export function optimalHeadingToFaceHubDeg(robotFieldX: number, robotFieldY: number): number {
  return optimalHeadingTowardFieldPointDeg(
    robotFieldX,
    robotFieldY,
    HUB_FIELD_X_M,
    HUB_FIELD_Y_M
  );
}

/**
 * Mirrors {@code DriveConstants.kHubShotMapHeadingOffset*} — LSQ fit from {@code knn_map.json}.
 * Keep in sync with Java when refitting.
 */
export const HUB_SHOT_MAP_HEADING_OFFSET_A = -0.024117514387849173;
export const HUB_SHOT_MAP_HEADING_OFFSET_PER_M_DX = 0.015164153162676098;
export const HUB_SHOT_MAP_HEADING_OFFSET_PER_M_DY = 0.03551452819037583;

/** Same degenerate-distance threshold as {@code DriveConstants.rotationToFaceHubFromShotMap}. */
export const HUB_SHOT_MAP_MIN_DIST_SQ_M2 = 1e-8;

/**
 * Additive heading correction (rad): {@code a + b·dx + c·dy} with hub−robot {@code dx, dy} (m) —
 * mirrors the {@code offsetRad} block in {@code DriveConstants.rotationToFaceHubFromShotMap}.
 */
export function hubShotMapHeadingOffsetRad(
  hubMinusRobotDxM: number,
  hubMinusRobotDyM: number
): number {
  return (
    HUB_SHOT_MAP_HEADING_OFFSET_A +
    HUB_SHOT_MAP_HEADING_OFFSET_PER_M_DX * hubMinusRobotDxM +
    HUB_SHOT_MAP_HEADING_OFFSET_PER_M_DY * hubMinusRobotDyM
  );
}

/**
 * Hub-facing heading (deg) using the continuous shot-map correction — mirrors
 * {@code DriveConstants.rotationToFaceHubFromShotMap}.
 */
export function hubShotMapHeadingTowardHubDeg(robotFieldX: number, robotFieldY: number): number {
  const dx = HUB_FIELD_X_M - robotFieldX;
  const dy = HUB_FIELD_Y_M - robotFieldY;
  if (dx * dx + dy * dy < HUB_SHOT_MAP_MIN_DIST_SQ_M2) {
    return 0;
  }
  const thetaRad = Math.atan2(dy, dx) + hubShotMapHeadingOffsetRad(dx, dy);
  const deg = (thetaRad * 180) / Math.PI + HUB_FACING_OFFSET_DEG;
  return normalizeHeadingDeg(deg);
}

export function distanceBetweenFieldPointsM(
  ax: number,
  ay: number,
  bx: number,
  by: number
): number {
  return Math.hypot(bx - ax, by - ay);
}

export function distanceToHubM(robotFieldX: number, robotFieldY: number): number {
  return distanceBetweenFieldPointsM(robotFieldX, robotFieldY, HUB_FIELD_X_M, HUB_FIELD_Y_M);
}

/** Shortest signed delta from {@code fromDeg} to {@code toDeg} (degrees). */
export function shortestAngleDeltaDeg(fromDeg: number, toDeg: number): number {
  let d = toDeg - fromDeg;
  d = ((d + 180) % 360) - 180;
  if (d <= -180) d += 360;
  return d;
}
