/**
 * PathPlanner {@link https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/FlippingUtil.java FlippingUtil}
 * rotational symmetry (default): red-side geometry is blue rotated 180° about the field center.
 */
import { FIELD_LENGTH_M, FIELD_WIDTH_M } from './fieldDimensions';

export type FieldSize = { sizeXM: number; sizeYM: number };

export const defaultFieldSize = (): FieldSize => ({
  sizeXM: FIELD_LENGTH_M,
  sizeYM: FIELD_WIDTH_M,
});

/** Rotational flip: (x, y) → (L − x, W − y). */
export function flipFieldPositionRotational(
  x: number,
  y: number,
  size: FieldSize = defaultFieldSize()
): { x: number; y: number } {
  return {
    x: size.sizeXM - x,
    y: size.sizeYM - y,
  };
}

/** Match {@code FlippingUtil.flipFieldRotation} for kRotational: θ' = θ − π. */
export function flipFieldHeadingDegRotational(headingDeg: number): number {
  return normalizeHeadingDeg180(headingDeg - 180);
}

function normalizeHeadingDeg180(deg: number): number {
  let d = deg % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return d;
}
