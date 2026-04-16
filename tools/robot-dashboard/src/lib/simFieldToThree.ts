import * as THREE from 'three';

/**
 * Maps PathPlanner / WPILib field coordinates (X = length, Y = width) into the simulation's
 * Three.js XZ plane (Y up): centered origin, field X → world X.
 * Blue alliance station is anchored at field (0,0) (WPIBlue / PathPlanner corner); +X runs toward red.
 * Field Y maps to world Z (mirrored about field center) so the sim floor matches PathPlanner/WPILib Y.
 */
export function fieldPoseToWorldXZ(
  fieldX: number,
  fieldY: number,
  fieldLengthM: number,
  fieldWidthM: number
): { wx: number; wz: number } {
  const cx = THREE.MathUtils.clamp(fieldX, 0, fieldLengthM) - fieldLengthM / 2;
  const fy = THREE.MathUtils.clamp(fieldY, 0, fieldWidthM);
  const cz = fieldWidthM / 2 - fy;
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
  const fy = fieldWidthM / 2 - worldZ;
  const y = THREE.MathUtils.clamp(fy, 0, fieldWidthM);
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

/**
 * Same as {@link rayNdcToFieldXY}, but transforms the floor hit through {@code fieldRoot} with
 * {@link THREE.Object3D#worldToLocal} first so picks match robot pose when the field group is yawed
 * (e.g. red driver perspective).
 */
export function worldPointToFieldXY(
  worldPoint: THREE.Vector3,
  fieldLengthM: number,
  fieldWidthM: number,
  fieldRoot: THREE.Object3D | null
): { x: number; y: number } {
  const p = worldPoint.clone();
  if (fieldRoot) {
    fieldRoot.worldToLocal(p);
  }
  return worldXZToFieldXY(p.x, p.z, fieldLengthM, fieldWidthM);
}

export function rayNdcToFieldXYWithFieldRoot(
  ndcX: number,
  ndcY: number,
  camera: THREE.PerspectiveCamera,
  fieldLengthM: number,
  fieldWidthM: number,
  fieldRoot: THREE.Object3D | null
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
  const hit = new THREE.Vector3(
    origin.x + t * dir.x,
    origin.y + t * dir.y,
    origin.z + t * dir.z
  );
  return worldPointToFieldXY(hit, fieldLengthM, fieldWidthM, fieldRoot);
}

/** rotation.y (rad): WPILib heading (CCW from +X) matches Three.js Y-up yaw on the XZ floor. */
export function fieldHeadingDegToSceneYawRad(headingDeg: number): number {
  return THREE.MathUtils.degToRad(headingDeg);
}

/**
 * Unit forward on the XZ floor in scene space for a WPILib field heading (deg).
 * Matches {@link fieldHeadingDegToSceneYawRad}: Three.js local +X after Y yaw is
 * (cos θ, 0, −sin θ), not (cos θ, 0, +sin θ).
 */
export function fieldHeadingDegToSceneForwardXZ(headingDeg: number): THREE.Vector3 {
  const r = fieldHeadingDegToSceneYawRad(headingDeg);
  return new THREE.Vector3(Math.cos(r), 0, -Math.sin(r));
}
