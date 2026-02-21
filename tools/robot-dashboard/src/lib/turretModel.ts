import * as THREE from 'three';

export const FLYWHEEL_R = 0.085;
export const RACK_TEETH = 20;
export const PINION_TEETH = 10;
export const GEAR_RATIO = RACK_TEETH / PINION_TEETH;

export function makeBox(
  w: number,
  h: number,
  d: number,
  mat: THREE.Material
): THREE.Mesh {
  const g = new THREE.BoxGeometry(w, h, d);
  return new THREE.Mesh(g, mat);
}

export function makeCyl(
  rt: number,
  rb: number,
  h: number,
  seg: number,
  mat: THREE.Material
): THREE.Mesh {
  const g = new THREE.CylinderGeometry(rt, rb, h, seg);
  return new THREE.Mesh(g, mat);
}

export function createGearMesh(
  teeth: number,
  moduleSize: number,
  faceWidth: number,
  mat: THREE.Material
): THREE.Mesh {
  const pitchR = (teeth * moduleSize) / 2;
  const addendum = moduleSize;
  const dedendum = 1.25 * moduleSize;
  const outerR = pitchR + addendum;
  const rootR = pitchR - dedendum;
  const toothAngle = (2 * Math.PI) / teeth;
  const shape = new THREE.Shape();

  for (let i = 0; i < teeth; i++) {
    const base = i * toothAngle;
    const t0 = base - toothAngle * 0.2;
    const t1 = base - toothAngle * 0.08;
    const t2 = base + toothAngle * 0.08;
    const t3 = base + toothAngle * 0.2;
    if (i === 0) {
      shape.moveTo(rootR * Math.cos(t0), rootR * Math.sin(t0));
    } else {
      shape.lineTo(rootR * Math.cos(t0), rootR * Math.sin(t0));
    }
    shape.lineTo(outerR * Math.cos(t1), outerR * Math.sin(t1));
    shape.lineTo(outerR * Math.cos(t2), outerR * Math.sin(t2));
    shape.lineTo(rootR * Math.cos(t3), rootR * Math.sin(t3));
  }
  shape.closePath();
  const hole = new THREE.Path();
  hole.absarc(0, 0, rootR * 0.45, 0, Math.PI * 2, true);
  shape.holes.push(hole);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: faceWidth,
    bevelEnabled: false,
    steps: 1,
  });
  geo.center();
  return new THREE.Mesh(geo, mat);
}

export function createQuarterRack(
  _teeth: number,
  moduleSize: number,
  faceWidth: number,
  mat: THREE.Material
): THREE.Mesh {
  const totalTeeth = 20;
  const pitchR = (totalTeeth * moduleSize) / (Math.PI / 2);
  const addendum = moduleSize;
  const dedendum = 1.25 * moduleSize;
  const outerR = pitchR + addendum;
  const rootR = pitchR - dedendum;
  const arcSpan = Math.PI / 2;
  const toothAngle = arcSpan / totalTeeth;
  const shape = new THREE.Shape();
  const startAngle = -arcSpan / 2;
  shape.moveTo(rootR * Math.cos(startAngle), rootR * Math.sin(startAngle));
  for (let i = 0; i < totalTeeth; i++) {
    const base = startAngle + i * toothAngle;
    const t0 = base;
    const t1 = base + toothAngle * 0.3;
    const t2 = base + toothAngle * 0.7;
    const t3 = base + toothAngle;
    shape.lineTo(rootR * Math.cos(t0), rootR * Math.sin(t0));
    shape.lineTo(outerR * Math.cos(t1), outerR * Math.sin(t1));
    shape.lineTo(outerR * Math.cos(t2), outerR * Math.sin(t2));
    shape.lineTo(rootR * Math.cos(t3), rootR * Math.sin(t3));
  }
  for (let a = arcSpan / 2; a >= -arcSpan / 2; a -= 0.15) {
    shape.lineTo(
      (rootR - faceWidth * 0.8) * Math.cos(a),
      (rootR - faceWidth * 0.8) * Math.sin(a)
    );
  }
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: faceWidth * 0.6,
    bevelEnabled: false,
  });
  return new THREE.Mesh(geo, mat);
}

export type TurretMeshRefs = {
  turretRef: THREE.Group;
  hoodPivotRef: THREE.Group;
  flywheelGroupRef: THREE.Group;
  pinionMeshRef: THREE.Mesh;
  matWheelRef: THREE.MeshStandardMaterial;
  speedLightRef: THREE.PointLight;
  turretRingRef: THREE.Mesh;
};

/**
 * Builds the exact turret/shooter mechanism and adds it to `parent` at y = baseY.
 * Use the returned refs to drive turret.rotation.y, hoodPivot.rotation.x, flywheel rotation, etc.
 */
export function createTurretMesh(
  parent: THREE.Object3D,
  baseY: number
): TurretMeshRefs {
  const matAlum = new THREE.MeshStandardMaterial({
    color: 0x8899bb,
    metalness: 0.85,
    roughness: 0.25,
  });
  const matSteel = new THREE.MeshStandardMaterial({
    color: 0x445566,
    metalness: 0.95,
    roughness: 0.15,
  });
  const matWheelRed = new THREE.MeshStandardMaterial({
    color: 0xcc2233,
    metalness: 0.3,
    roughness: 0.7,
  });
  const matAccent = new THREE.MeshStandardMaterial({
    color: 0x4fc3f7,
    metalness: 0.5,
    roughness: 0.4,
    emissive: 0x1a4060,
    emissiveIntensity: 0.3,
  });
  const matGear = new THREE.MeshStandardMaterial({
    color: 0x334455,
    metalness: 0.9,
    roughness: 0.2,
  });
  const matPlate = new THREE.MeshStandardMaterial({
    color: 0x667799,
    metalness: 0.7,
    roughness: 0.35,
  });
  const matHub = new THREE.MeshStandardMaterial({
    color: 0x222233,
    metalness: 0.8,
    roughness: 0.3,
  });

  const turretBase = new THREE.Group();
  turretBase.position.y = baseY;
  parent.add(turretBase);

  const basePlate = makeBox(0.5, 0.04, 0.5, matPlate);
  basePlate.position.y = 0.02;
  basePlate.castShadow = true;
  turretBase.add(basePlate);

  for (const dx of [-0.18, 0.18]) {
    for (const dz of [-0.18, 0.18]) {
      const post = makeCyl(0.015, 0.015, 0.12, 8, matSteel);
      post.position.set(dx, 0.08, dz);
      post.castShadow = true;
      turretBase.add(post);
    }
  }

  const bearing = makeCyl(0.04, 0.04, 0.05, 16, matSteel);
  bearing.position.y = 0.045;
  turretBase.add(bearing);

  const turret = new THREE.Group();
  turret.position.y = 0.07;
  turretBase.add(turret);
  turret.rotation.y = -Math.PI / 2;

  const turretRing = makeCyl(0.07, 0.07, 0.03, 32, matAccent);
  turret.add(turretRing);

  const turretTop = makeBox(0.38, 0.025, 0.3, matAlum);
  turretTop.position.y = 0.025;
  turret.add(turretTop);

  for (const side of [-1, 1]) {
    const sp = makeBox(0.025, 0.18, 0.3, matAlum);
    sp.position.set(side * 0.18, 0.12, 0);
    sp.castShadow = true;
    turret.add(sp);
  }

  const FLYWHEEL_Y = 0.25;
  const FLYWHEEL_Z = 0.06;

  const flywheelGroup = new THREE.Group();
  flywheelGroup.position.set(0, FLYWHEEL_Y, FLYWHEEL_Z);
  turret.add(flywheelGroup);

  for (const side of [-0.06, 0.06]) {
    const wheel = makeCyl(FLYWHEEL_R, FLYWHEEL_R, 0.03, 32, matWheelRed);
    wheel.rotation.z = Math.PI / 2;
    wheel.position.x = side;
    wheel.castShadow = true;
    flywheelGroup.add(wheel);
    const hub = makeCyl(0.025, 0.025, 0.035, 16, matHub);
    hub.rotation.z = Math.PI / 2;
    hub.position.x = side;
    flywheelGroup.add(hub);
  }

  const axle = makeCyl(0.01, 0.01, 0.2, 12, matSteel);
  axle.rotation.z = Math.PI / 2;
  flywheelGroup.add(axle);

  const motor = makeCyl(0.038, 0.038, 0.09, 16, matHub);
  motor.rotation.z = Math.PI / 2;
  motor.position.set(-0.12, 0, 0);
  flywheelGroup.add(motor);

  const hoodPivotGroup = new THREE.Group();
  hoodPivotGroup.position.set(0, FLYWHEEL_Y, FLYWHEEL_Z);
  turret.add(hoodPivotGroup);

  const MODULE = 0.018;
  const PINION_PITCH_R = (PINION_TEETH * MODULE) / 2;
  const RACK_PITCH_R = (RACK_TEETH * MODULE) / (Math.PI / 2);
  const CENTER_DIST = RACK_PITCH_R + PINION_PITCH_R;

  const pinionGroup = new THREE.Group();
  pinionGroup.position.set(0.12, FLYWHEEL_Y - CENTER_DIST, FLYWHEEL_Z);
  turret.add(pinionGroup);

  const pinionMesh = createGearMesh(PINION_TEETH, MODULE, 0.04, matGear);
  pinionMesh.rotation.x = Math.PI / 2;
  pinionGroup.add(pinionMesh);

  const pinionShaft = makeCyl(0.008, 0.008, 0.1, 12, matSteel);
  pinionShaft.rotation.x = Math.PI / 2;
  pinionGroup.add(pinionShaft);

  const rackGroup = new THREE.Group();
  rackGroup.position.set(0.12, 0, 0);
  hoodPivotGroup.add(rackGroup);

  const rackMesh = createQuarterRack(RACK_TEETH, MODULE, 0.04, matGear);
  rackMesh.rotation.x = Math.PI / 2;
  rackMesh.rotation.z = -Math.PI / 2;
  rackGroup.add(rackMesh);

  const rackPitchRing = new THREE.Mesh(
    new THREE.TorusGeometry(RACK_PITCH_R, 0.003, 8, 40, Math.PI / 2),
    new THREE.MeshStandardMaterial({
      color: 0x4fc3f7,
      emissive: 0x224466,
      emissiveIntensity: 0.5,
    })
  );
  rackPitchRing.rotation.x = Math.PI / 2;
  rackGroup.add(rackPitchRing);

  const hoodAssembly = new THREE.Group();
  hoodPivotGroup.add(hoodAssembly);

  for (const side of [-0.09, 0.09]) {
    const arm = makeBox(0.025, 0.22, 0.025, matAlum);
    arm.position.set(side, 0.11, -0.12);
    arm.castShadow = true;
    hoodAssembly.add(arm);
  }

  const hoodPlate = makeBox(0.22, 0.012, 0.18, matAlum);
  hoodPlate.position.set(0, 0.22, -0.24);
  hoodPlate.castShadow = true;
  hoodAssembly.add(hoodPlate);

  const hoodLip = makeBox(0.22, 0.04, 0.012, matAccent);
  hoodLip.position.set(0, 0.22, -0.16);
  hoodAssembly.add(hoodLip);

  const sensorBox = makeBox(0.1, 0.08, 0.06, matHub);
  sensorBox.position.set(0, 0.3, -0.2);
  hoodAssembly.add(sensorBox);

  const channelGroup = new THREE.Group();
  channelGroup.position.set(0, 0.15, 0);
  turret.add(channelGroup);
  for (const s of [-0.075, 0.075]) {
    const ch = makeBox(0.012, 0.06, 0.22, matPlate);
    ch.position.set(s, 0, 0.05);
    ch.castShadow = true;
    channelGroup.add(ch);
  }

  const pivotMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.008, 16, 16),
    new THREE.MeshStandardMaterial({
      color: 0xffdd44,
      emissive: 0x664400,
      emissiveIntensity: 1,
    })
  );
  pivotMarker.position.set(0, FLYWHEEL_Y, FLYWHEEL_Z);
  turret.add(pivotMarker);

  const speedLight = new THREE.PointLight(0x4444ff, 0, 0.5);
  speedLight.position.set(0, FLYWHEEL_Y, FLYWHEEL_Z);
  turret.add(speedLight);

  return {
    turretRef: turret,
    hoodPivotRef: hoodPivotGroup,
    flywheelGroupRef: flywheelGroup,
    pinionMeshRef: pinionMesh,
    matWheelRef: matWheelRed,
    speedLightRef: speedLight,
    turretRingRef: turretRing,
  };
}
