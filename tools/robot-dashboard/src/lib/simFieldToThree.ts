import * as THREE from 'three';

/**
 * Maps PathPlanner / WPILib field coordinates (X = length, Y = width, origin at corner)
 * into the simulation's Three.js XZ plane (Y up): centered origin, field X → world X, field Y → world Z.
 */
export function fieldPoseToWorldXZ(
  fieldX: number,
  fieldY: number,
  fieldLengthM: number,
  fieldWidthM: number
): { wx: number; wz: number } {
  const cx = THREE.MathUtils.clamp(fieldX, 0, fieldLengthM) - fieldLengthM / 2;
  const cz = THREE.MathUtils.clamp(fieldY, 0, fieldWidthM) - fieldWidthM / 2;
  return { wx: cx, wz: cz };
}

/** Inverse of {@link fieldPoseToWorldXZ}: intersection with field floor plane y=0, clamped to field bounds. */
export function worldXZToFieldXY(
  worldX: number,
  worldZ: number,
  fieldLengthM: number,
  fieldWidthM: number
): { x: number; y: number } {
  const x = THREE.MathUtils.clamp(worldX + fieldLengthM / 2, 0, fieldLengthM);
  const y = THREE.MathUtils.clamp(worldZ + fieldWidthM / 2, 0, fieldWidthM);
  return { x, y };
}

/**
 * Ray from NDC through camera intersects horizontal plane y=0; returns field X/Y or null.
 */
export function rayNdcToFieldXY(
  ndcX: number,
  ndcY: number,
  camera: THREE.PerspectiveCamera,
  fieldLengthM: number,
  fieldWidthM: number
): { x: number; y: number } | null {
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
  const origin = raycaster.ray.origin;
  const dir = raycaster.ray.direction;
  if (Math.abs(dir.y) < 1e-8) {
    return null;
  }
  const t = -origin.y / dir.y;
  if (t < 0) {
    return null;
  }
  const wx = origin.x + t * dir.x;
  const wz = origin.z + t * dir.z;
  return worldXZToFieldXY(wx, wz, fieldLengthM, fieldWidthM);
}

/** rotation.y (rad) for the robot / markers — same sign as NT / odometry heading (CCW positive from +X). */
export function fieldHeadingDegToSceneYawRad(headingDeg: number): number {
  return THREE.MathUtils.degToRad(headingDeg);
}

export function fieldHeadingDegToSceneForwardXZ(headingDeg: number): THREE.Vector3 {
  const r = fieldHeadingDegToSceneYawRad(headingDeg);
  return new THREE.Vector3(Math.cos(r), 0, Math.sin(r));
}
