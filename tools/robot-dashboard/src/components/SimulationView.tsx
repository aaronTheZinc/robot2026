import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import {
  createTurretMesh,
  FLYWHEEL_R,
  GEAR_RATIO,
} from '../lib/turretModel';
import type { TurretMeshRefs } from '../lib/turretModel';
import {
  inferKnn,
  knnMapPointFieldYForAlliance,
  type KnnPoint,
  type KnnShootTarget,
} from '../lib/knnInference';
import {
  distanceBetweenFieldPointsM,
  HUB_FIELD_X_M,
  HUB_FIELD_Y_M,
  HUB_FACING_OFFSET_DEG,
  HUB_SHOT_MAP_MIN_DIST_SQ_M2,
  hubShotMapHeadingOffsetRad,
  hubShotMapHeadingTowardHubDeg,
  normalizeHeadingDeg,
  optimalHeadingTowardFieldPointDeg,
  shortestAngleDeltaDeg,
} from '../lib/hubField';
import type { SimDropTarget } from '../lib/simDropTargets';
import {
  resolveSimAlignmentOrigin,
  SIM_HUB_ORIGIN_ID,
  type SimNamedOrigin,
} from '../lib/simFieldOrigins';
import {
  fieldHeadingDegToSceneForwardXZ,
  fieldHeadingDegToSceneYawRad,
  fieldPoseToWorldXZ,
  rayNdcToFieldXYWithFieldRoot,
} from '../lib/simFieldToThree';
import {
  flipFieldHeadingDegRotational,
  flipFieldPositionRotational,
  type FieldSize,
} from '../lib/pathPlannerFlip';
import { useDraggablePanel } from '../lib/useDraggablePanel';

type SimulationViewProps = {
  fieldLengthM: number;
  fieldWidthM: number;
  robotLengthM: number;
  robotWidthM: number;
  /**
   * When true, the field (and all field-space visuals) are rotated 180° so the sim matches the red
   * driver station perspective; driven from NT {@code FMSInfo/IsRedAlliance} when connected.
   */
  viewAsRedAlliance: boolean;
  /** {@code FMSInfo/StationNumber} (1–3), or null */
  driverStationStationNumber: number | null;
  poseX: number;
  poseY: number;
  headingDeg: number;
  limelightPoseX?: number;
  limelightPoseY?: number;
  limelightHeadingDeg?: number;
  limelightPoseVisible?: boolean;
  limelightHasLock?: boolean;
  limelightFrontPoseX?: number;
  limelightFrontPoseY?: number;
  limelightFrontHeadingDeg?: number;
  limelightFrontPoseVisible?: boolean;
  limelightFrontHasLock?: boolean;
  idealShooterPoseX?: number;
  idealShooterPoseY?: number;
  idealShooterHeadingDeg?: number;
  idealShooterPoseVisible?: boolean;
  turretAngleDeg: number;
  hoodPitchDeg: number;
  velocityMps: number;
  /** Logged / deploy KNN shot samples (same indices as robot `knn_map.json` when lists match). */
  shotMapPoints: KnnPoint[];
  /** Red alliance: flips KNN marker Y for display; inference uses WPIBlue pose only. */
  isRedAlliance: boolean;
  /** `/KNN/selectedIndex` from robot, or -1 when unknown */
  knnSelectedIndex: number;
  /** `/KNN/nearestIndexBlue` — nearest map row to fused WPIBlue pose */
  knnNearestIndexBlue: number;
  /** `/KNN/nearestIndexRed` — duplicate of blue when mirror lookup off (legacy NT) */
  knnNearestIndexRed: number;
  connected: boolean;
  swerveSpeedMps: number;
  shooterRpm: number;
  shooterRpmSetpoint: number;
  hoodSetpointDeg: number;
  turretTracking: boolean;
  aimTargetKind: 'hub' | 'drop';
  dropTargets: SimDropTarget[];
  namedOrigins: SimNamedOrigin[];
  selectedDropTargetId: string | null;
  onAimTargetKindChange: (kind: 'hub' | 'drop') => void;
  onAddDropTarget: (fieldX: number, fieldY: number) => void;
  onAddNamedOrigin: (fieldX: number, fieldY: number) => void;
  /** Append a map row at field coordinates (WPIBlue); returns the new row index. */
  onAddShotSample: (fieldX: number, fieldY: number) => number;
  onRemoveNamedOrigin: (id: string) => void;
  onRenameNamedOrigin: (id: string, name: string) => void;
  onUpdateDropTarget: (
    id: string,
    patch: Partial<Pick<SimDropTarget, 'label' | 'alignmentOriginId' | 'x' | 'y'>>
  ) => void;
  onRemoveDropTarget: (id: string) => void;
  onSelectDropForAim: (id: string) => void;
  mockMode: boolean;
  /** When mock, matches dashboard swerve field-relative toggle (W/S = ±field X, A/D = ±field Y). */
  fieldRelative: boolean;
  /** Mock mode only: field-frame deltas (m) and heading change (deg) from WASD + J/L. */
  onMockSimKeyboardDrive: (payload: {
    dx: number;
    dy: number;
    dHeadingDeg: number;
    speedMps: number;
  }) => void;
  /**
   * Mock mode only: set field pose / heading (m, °). Used for click-drag on the robot and the
   * mock pose panel.
   */
  onMockSimPatchSwerve: (patch: { x?: number; y?: number; headingDeg?: number }) => void;
  onApplyMockShotFromSim: (
    headingDeg: number,
    shooterRpmSetpoint: number,
    hoodDegSetpoint: number
  ) => void;
  /** Persist hub vs current field aim for the selected map point index. */
  onSaveKnnShootTarget: (index: number, target: KnnShootTarget) => void;
};

const ROBOT_HEIGHT_M = 0.18;
const VECTOR_OFFSET = 2.2;
/** Mock sim: keyboard translation / rotation rates (WPILib field X/Y). */
const MOCK_SIM_DRIVE_MPS = 3.0;
const MOCK_SIM_ROT_DEG_PER_SEC = 80;

function isPlaceholderKnnPoint(p: KnnPoint): boolean {
  return (
    Math.abs(p.x) < 1e-6 &&
    Math.abs(p.y) < 1e-6 &&
    Math.abs(p.shooterRpm ?? 0) < 1
  );
}

function disposeObject3DSubtree(root: THREE.Object3D) {
  root.traverse((obj) => {
    const maybeWithGeometry = obj as THREE.Object3D & {
      geometry?: THREE.BufferGeometry;
      material?: THREE.Material | THREE.Material[];
    };
    maybeWithGeometry.geometry?.dispose();
    const mat = maybeWithGeometry.material;
    if (mat) {
      if (Array.isArray(mat)) {
        mat.forEach((m) => m.dispose());
      } else {
        mat.dispose();
      }
    }
  });
}

function createPhantomPoseGhost(
  robotLengthM: number,
  robotWidthM: number,
  color: number,
  opacity: number
): THREE.Group {
  const group = new THREE.Group();
  const bodyHeightM = ROBOT_HEIGHT_M * 0.82;
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(robotLengthM, bodyHeightM, robotWidthM),
    new THREE.MeshStandardMaterial({
      color,
      transparent: true,
      opacity,
      metalness: 0.12,
      roughness: 0.52,
      emissive: color,
      emissiveIntensity: 0.08,
      depthWrite: false,
    })
  );
  body.position.y = bodyHeightM / 2 + 0.015;
  group.add(body);

  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, bodyHeightM / 2 + 0.015, 0),
    Math.max(robotLengthM, robotWidthM) * 0.7,
    color
  );
  group.add(arrow);
  group.visible = false;
  return group;
}

/**
 * Speaker / processor hub (hex funnel) — two independent instances for blue- and red-side field
 * symmetry (PathPlanner rotational flip).
 */
function buildSpeakerHubAssembly(): THREE.Group {
  const hubRoot = new THREE.Group();
  const funnelH = 1.55;
  const funnelRTop = 0.62;
  const funnelRBot = 0.19;
  const HEX = 6;

  const funnelMat = new THREE.MeshStandardMaterial({
    color: 0xb8c0cc,
    metalness: 0.78,
    roughness: 0.32,
    side: THREE.DoubleSide,
    emissive: 0xff6b2d,
    emissiveIntensity: 0.06,
  });
  const funnelGeom = new THREE.CylinderGeometry(
    funnelRTop,
    funnelRBot,
    funnelH,
    HEX,
    1,
    true
  );
  const hubFunnel = new THREE.Mesh(funnelGeom, funnelMat);
  hubFunnel.position.y = funnelH / 2 + 0.01;
  hubFunnel.rotation.y = Math.PI / 6;
  hubFunnel.castShadow = true;
  hubFunnel.receiveShadow = true;
  hubRoot.add(hubFunnel);

  const innerMat = new THREE.MeshStandardMaterial({
    color: 0x4a5568,
    metalness: 0.55,
    roughness: 0.52,
    side: THREE.DoubleSide,
  });
  const innerGeom = new THREE.CylinderGeometry(
    funnelRTop - 0.05,
    funnelRBot - 0.035,
    funnelH - 0.06,
    HEX,
    1,
    true
  );
  const hubInner = new THREE.Mesh(innerGeom, innerMat);
  hubInner.position.y = funnelH / 2 + 0.01;
  hubInner.rotation.y = Math.PI / 6;
  hubRoot.add(hubInner);

  const rimMat = new THREE.MeshStandardMaterial({
    color: 0xff9a4d,
    metalness: 0.82,
    roughness: 0.22,
    emissive: 0xff6600,
    emissiveIntensity: 0.12,
  });
  const rimGeom = new THREE.TorusGeometry(funnelRTop + 0.04, 0.04, 8, HEX);
  const hubRim = new THREE.Mesh(rimGeom, rimMat);
  hubRim.rotation.x = Math.PI / 2;
  hubRim.rotation.z = Math.PI / 6;
  hubRim.position.y = funnelH + 0.04;
  hubRim.castShadow = true;
  hubRoot.add(hubRim);

  const capMat = new THREE.MeshStandardMaterial({
    color: 0xa0a8b5,
    metalness: 0.8,
    roughness: 0.35,
  });
  const capThick = 0.055;
  const apothemTop = funnelRTop * Math.cos(Math.PI / HEX);
  const hexEdgeLen = 2 * funnelRTop * Math.sin(Math.PI / HEX);
  for (let i = 0; i < HEX; i++) {
    const ang = (i + 0.5) * ((2 * Math.PI) / HEX) + Math.PI / 6;
    const capGeom = new THREE.BoxGeometry(capThick, 0.075, hexEdgeLen * 1.04);
    const cap = new THREE.Mesh(capGeom, capMat);
    const r = apothemTop + capThick / 2 + 0.02;
    cap.position.set(Math.cos(ang) * r, funnelH + 0.038, Math.sin(ang) * r);
    cap.rotation.y = -ang;
    cap.castShadow = true;
    hubRoot.add(cap);
  }

  const strutMat = new THREE.MeshStandardMaterial({
    color: 0x8b95a5,
    metalness: 0.75,
    roughness: 0.38,
  });
  for (let i = 0; i < HEX; i++) {
    const ang = (i * Math.PI) / 3 + Math.PI / 6;
    const strutGeom = new THREE.BoxGeometry(0.065, funnelH * 0.94, 0.065);
    const strut = new THREE.Mesh(strutGeom, strutMat);
    const r = funnelRTop - 0.02;
    strut.position.set(Math.cos(ang) * r, funnelH / 2 + 0.02, Math.sin(ang) * r);
    strut.rotation.y = -ang;
    strut.castShadow = true;
    hubRoot.add(strut);
  }

  const throatCollarGeom = new THREE.CylinderGeometry(
    funnelRBot + 0.045,
    funnelRBot + 0.02,
    0.11,
    HEX,
    1,
    false
  );
  const throatCollarMat = new THREE.MeshStandardMaterial({
    color: 0x9ca3af,
    metalness: 0.7,
    roughness: 0.4,
  });
  const throatCollar = new THREE.Mesh(throatCollarGeom, throatCollarMat);
  throatCollar.position.y = 0.055;
  throatCollar.rotation.y = Math.PI / 6;
  throatCollar.castShadow = true;
  hubRoot.add(throatCollar);

  const mouthHintGeom = new THREE.RingGeometry(
    funnelRTop * 0.74,
    funnelRTop * 1.06,
    HEX
  );
  const mouthHintMat = new THREE.MeshBasicMaterial({
    color: 0xffb84d,
    transparent: true,
    opacity: 0.28,
    side: THREE.DoubleSide,
  });
  const mouthHint = new THREE.Mesh(mouthHintGeom, mouthHintMat);
  mouthHint.rotation.x = -Math.PI / 2;
  mouthHint.rotation.z = Math.PI / 6;
  mouthHint.position.y = funnelH + 0.045;
  hubRoot.add(mouthHint);

  const intakeRingGeom = new THREE.RingGeometry(funnelRBot * 0.42, funnelRBot * 1.38, HEX);
  const intakeRingMat = new THREE.MeshBasicMaterial({
    color: 0xffaa55,
    transparent: true,
    opacity: 0.38,
    side: THREE.DoubleSide,
  });
  const intakeRing = new THREE.Mesh(intakeRingGeom, intakeRingMat);
  intakeRing.rotation.x = -Math.PI / 2;
  intakeRing.rotation.z = Math.PI / 6;
  intakeRing.position.y = 0.012;
  hubRoot.add(intakeRing);

  const hubLight = new THREE.PointLight(0xffaa77, 2.4, 7, 2);
  hubLight.position.set(0, funnelH * 0.55, 0);
  hubRoot.add(hubLight);

  hubRoot.userData.hubFunnel = hubFunnel;
  hubRoot.userData.hubRim = hubRim;
  hubRoot.userData.hubMouthHint = mouthHint;
  hubRoot.userData.hubIntakeRing = intakeRing;

  return hubRoot;
}

function duplicateOffsetsForShots(points: KnnPoint[]): { dx: number; dy: number; lift: number }[] {
  const keyCounts = new Map<string, number>();
  return points.map((p) => {
    const key = `${p.x.toFixed(2)},${p.y.toFixed(2)}`;
    const n = keyCounts.get(key) ?? 0;
    keyCounts.set(key, n + 1);
    const angle = n * 2.1;
    const r = 0.07 * n;
    return {
      dx: r * Math.cos(angle),
      dy: r * Math.sin(angle),
      lift: n * 0.028,
    };
  });
}

export default function SimulationView({
  fieldLengthM,
  fieldWidthM,
  robotLengthM,
  robotWidthM,
  viewAsRedAlliance,
  driverStationStationNumber,
  poseX,
  poseY,
  headingDeg,
  limelightPoseX,
  limelightPoseY,
  limelightHeadingDeg,
  limelightPoseVisible = false,
  limelightHasLock = false,
  limelightFrontPoseX,
  limelightFrontPoseY,
  limelightFrontHeadingDeg,
  limelightFrontPoseVisible = false,
  limelightFrontHasLock = false,
  idealShooterPoseX,
  idealShooterPoseY,
  idealShooterHeadingDeg,
  idealShooterPoseVisible = false,
  turretAngleDeg,
  hoodPitchDeg,
  velocityMps,
  shotMapPoints,
  isRedAlliance,
  knnSelectedIndex,
  knnNearestIndexBlue,
  knnNearestIndexRed,
  connected,
  swerveSpeedMps,
  shooterRpm,
  shooterRpmSetpoint,
  hoodSetpointDeg,
  turretTracking,
  aimTargetKind,
  dropTargets,
  namedOrigins,
  selectedDropTargetId,
  onAimTargetKindChange,
  onAddDropTarget,
  onAddNamedOrigin,
  onAddShotSample,
  onRemoveNamedOrigin,
  onRenameNamedOrigin,
  onUpdateDropTarget,
  onRemoveDropTarget,
  onSelectDropForAim,
  mockMode,
  fieldRelative,
  onMockSimKeyboardDrive,
  onMockSimPatchSwerve,
  onApplyMockShotFromSim,
  onSaveKnnShootTarget,
}: SimulationViewProps) {
  /**
   * Nearest map row index: when connected, {@code /KNN/selectedIndex} from the robot (same list order as
   * {@code knn_map.json}); otherwise local {@link inferKnn} on fused WPIBlue pose (no Y mirror).
   */
  const effectiveKnnSelectedIndex = useMemo(() => {
    const inferred = inferKnn(
      { x: poseX, y: poseY, headingDeg },
      shotMapPoints,
      { k: 1, fieldWidthM }
    );
    const localIdx = inferred?.inferredIndex ?? -1;
    if (connected && knnSelectedIndex >= 0) {
      return knnSelectedIndex;
    }
    return localIdx;
  }, [poseX, poseY, headingDeg, shotMapPoints, fieldWidthM, connected, knnSelectedIndex]);

  /** Geometric nearest map index: fused WPIBlue pose vs map rows (matches {@code /KNN/nearestIndexBlue}). */
  const effectiveKnnNearestBlue = useMemo(() => {
    if (connected && knnNearestIndexBlue >= 0) {
      return knnNearestIndexBlue;
    }
    const inferred = inferKnn(
      { x: poseX, y: poseY, headingDeg },
      shotMapPoints,
      { k: 1, fieldWidthM }
    );
    return inferred?.inferredIndex ?? -1;
  }, [connected, knnNearestIndexBlue, poseX, poseY, headingDeg, shotMapPoints, fieldWidthM]);

  /** Same as blue when mirror lookup is off; else legacy NT {@code /KNN/nearestIndexRed}. */
  const effectiveKnnNearestRed = useMemo(() => {
    if (connected && knnNearestIndexRed >= 0) {
      return knnNearestIndexRed;
    }
    return effectiveKnnNearestBlue;
  }, [connected, knnNearestIndexRed, effectiveKnnNearestBlue]);

  type PlaceFieldMode = 'none' | 'drop' | 'origin' | 'sample';
  const [clickedShotIndex, setClickedShotIndex] = useState<number | null>(null);
  const [placeFieldMode, setPlaceFieldMode] = useState<PlaceFieldMode>('none');
  const [mockPoseXStr, setMockPoseXStr] = useState(() => String(poseX));
  const [mockPoseYStr, setMockPoseYStr] = useState(() => String(poseY));
  const [mockPoseHStr, setMockPoseHStr] = useState(() => String(headingDeg));
  const mockPoseFormRef = useRef<HTMLDivElement | null>(null);
  /** Second arrow on each KNN marker: {@code rotationToFaceHubFromShotMap} heading (linear fit). */
  const [showShotMapHubHeading, setShowShotMapHubHeading] = useState(false);
  const setClickedShotIndexRef = useRef(setClickedShotIndex);
  setClickedShotIndexRef.current = setClickedShotIndex;

  const placeFieldModeRef = useRef(placeFieldMode);
  placeFieldModeRef.current = placeFieldMode;
  const fieldMetricsRef = useRef({ fieldLengthM, fieldWidthM });
  fieldMetricsRef.current = { fieldLengthM, fieldWidthM };
  const onAddDropTargetRef = useRef(onAddDropTarget);
  onAddDropTargetRef.current = onAddDropTarget;
  const onAddNamedOriginRef = useRef(onAddNamedOrigin);
  onAddNamedOriginRef.current = onAddNamedOrigin;
  const onAddShotSampleRef = useRef(onAddShotSample);
  onAddShotSampleRef.current = onAddShotSample;
  const onSelectDropForAimRef = useRef(onSelectDropForAim);
  onSelectDropForAimRef.current = onSelectDropForAim;
  const setPlaceFieldModeRef = useRef(setPlaceFieldMode);
  setPlaceFieldModeRef.current = setPlaceFieldMode;

  const mockModeRef = useRef(mockMode);
  mockModeRef.current = mockMode;
  const fieldRelativeRef = useRef(fieldRelative);
  fieldRelativeRef.current = fieldRelative;
  const headingDegRef = useRef(headingDeg);
  headingDegRef.current = headingDeg;
  const onMockSimKeyboardDriveRef = useRef(onMockSimKeyboardDrive);
  onMockSimKeyboardDriveRef.current = onMockSimKeyboardDrive;
  const onMockSimPatchSwerveRef = useRef(onMockSimPatchSwerve);
  onMockSimPatchSwerveRef.current = onMockSimPatchSwerve;
  const wasMockKeyboardDrivingRef = useRef(false);
  const robotDragActiveRef = useRef(false);
  const robotDragOffsetRef = useRef({ x: 0, y: 0 });
  const poseXRef = useRef(poseX);
  poseXRef.current = poseX;
  const poseYRef = useRef(poseY);
  poseYRef.current = poseY;

  const simulationSectionRef = useRef<HTMLElement | null>(null);
  const aimPanelDrag = useDraggablePanel(simulationSectionRef);
  const inspectPanelDrag = useDraggablePanel(simulationSectionRef);
  const vectorsPanelDrag = useDraggablePanel(simulationSectionRef);

  const containerRef = useRef<HTMLDivElement | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
  const limelightGhostRef = useRef<THREE.Group | null>(null);
  const limelightFrontGhostRef = useRef<THREE.Group | null>(null);
  const turretMeshRefsRef = useRef<TurretMeshRefs | null>(null);
  const turretRef = useRef<THREE.Group | null>(null);
  const hoodPivotRef = useRef<THREE.Group | null>(null);
  const pinionMeshRef = useRef<THREE.Mesh | null>(null);
  const velocityMpsRef = useRef(velocityMps);
  const vectorArrowsGroupRef = useRef<THREE.Group | null>(null);
  const headingVectorArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const shooterVectorArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const velocityVectorArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const shotMarkersGroupRef = useRef<THREE.Group | null>(null);
  const dropMarkersGroupRef = useRef<THREE.Group | null>(null);
  const originMarkersGroupRef = useRef<THREE.Group | null>(null);
  const aimLineGroupRef = useRef<THREE.Group | null>(null);
  const idealAimLineGroupRef = useRef<THREE.Group | null>(null);
  const hubVfxRootsRef = useRef<THREE.Group[]>([]);
  /** All field-space content (floor, hub, robot, markers); yaw for red vs blue driver perspective. */
  const fieldRootRef = useRef<THREE.Group | null>(null);
  const animationRef = useRef<number | null>(null);

  /** Default orbit: look from blue alliance (field x≈0) toward +X / red; matches WPIBlue (0,0) corner. */
  const sphericalRef = useRef({ theta: -Math.PI / 2 + 0.2, phi: 0.92, radius: 14 });
  const panOffsetRef = useRef(new THREE.Vector3(0, 0, 0));
  const isDraggingRef = useRef(false);
  const pointerDownOnCanvasRef = useRef(false);
  const dragStartRef = useRef({ x: 0, y: 0 });
  const hasDraggedRef = useRef(false);
  const lastMouseRef = useRef({ x: 0, y: 0 });
  const keysRef = useRef({
    w: false,
    a: false,
    s: false,
    d: false,
    q: false,
    e: false,
    j: false,
    l: false,
  });
  const clockRef = useRef(new THREE.Clock());

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x141a28);
    scene.fog = new THREE.FogExp2(0x141a28, 0.012);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 2.0;
    container.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / container.clientHeight,
      0.1,
      200
    );
    panOffsetRef.current.set(0, 0, 0);
    const s = sphericalRef.current;
    camera.position.set(
      s.radius * Math.sin(s.phi) * Math.sin(s.theta),
      s.radius * Math.cos(s.phi),
      s.radius * Math.sin(s.phi) * Math.cos(s.theta)
    );
    camera.lookAt(0, 0, 0);

    const ambient = new THREE.AmbientLight(0x8899bb, 1.4);
    const directional = new THREE.DirectionalLight(0xccddff, 2.2);
    directional.position.set(10, 22, 10);
    directional.castShadow = true;
    directional.shadow.mapSize.set(1024, 1024);
    directional.shadow.camera.near = 0.5;
    directional.shadow.camera.far = 80;
    directional.shadow.camera.left = -25;
    directional.shadow.camera.right = 25;
    directional.shadow.camera.top = 25;
    directional.shadow.camera.bottom = -25;
    scene.add(ambient, directional);

    const fieldGeometry = new THREE.PlaneGeometry(fieldLengthM, fieldWidthM);
    const fieldMaterial = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.08,
      roughness: 0.85,
    });
    const fieldMesh = new THREE.Mesh(fieldGeometry, fieldMaterial);
    fieldMesh.rotation.x = -Math.PI / 2;
    fieldMesh.receiveShadow = true;

    const borderGeometry = new THREE.EdgesGeometry(fieldGeometry);
    const borderMaterial = new THREE.LineBasicMaterial({ color: 0x2f3a52 });
    const border = new THREE.LineSegments(borderGeometry, borderMaterial);
    border.rotation.x = -Math.PI / 2;

    const gridSize = Math.max(fieldLengthM, fieldWidthM);
    const grid = new THREE.GridHelper(gridSize, 24, 0x1e3a5f, 0x0f1e30);
    grid.position.y = 0.002;

    const halfLen = fieldLengthM / 2;
    const zoneBlueGeom = new THREE.PlaneGeometry(halfLen, fieldWidthM);
    const zoneBlueMat = new THREE.MeshBasicMaterial({
      color: 0x2563eb,
      transparent: true,
      opacity: 0.16,
      depthWrite: false,
    });
    const zoneBlue = new THREE.Mesh(zoneBlueGeom, zoneBlueMat);
    zoneBlue.rotation.x = -Math.PI / 2;
    zoneBlue.position.set(-halfLen / 2, 0.006, 0);

    const zoneRedGeom = new THREE.PlaneGeometry(halfLen, fieldWidthM);
    const zoneRedMat = new THREE.MeshBasicMaterial({
      color: 0xdc2626,
      transparent: true,
      opacity: 0.16,
      depthWrite: false,
    });
    const zoneRed = new THREE.Mesh(zoneRedGeom, zoneRedMat);
    zoneRed.rotation.x = -Math.PI / 2;
    zoneRed.position.set(halfLen / 2, 0.006, 0);

    const fieldRoot = new THREE.Group();
    fieldRootRef.current = fieldRoot;
    fieldRoot.add(fieldMesh, border, grid, zoneBlue, zoneRed);
    scene.add(fieldRoot);

    const hubPrimary = buildSpeakerHubAssembly();
    const { wx: hubCx, wz: hubCz } = fieldPoseToWorldXZ(
      HUB_FIELD_X_M,
      HUB_FIELD_Y_M,
      fieldLengthM,
      fieldWidthM
    );
    hubPrimary.position.set(hubCx, 0, hubCz);
    fieldRoot.add(hubPrimary);

    const simFieldSize: FieldSize = { sizeXM: fieldLengthM, sizeYM: fieldWidthM };
    const hubFlippedField = flipFieldPositionRotational(
      HUB_FIELD_X_M,
      HUB_FIELD_Y_M,
      simFieldSize
    );
    const hubSecondary = buildSpeakerHubAssembly();
    const { wx: hfx, wz: hfz } = fieldPoseToWorldXZ(
      hubFlippedField.x,
      hubFlippedField.y,
      fieldLengthM,
      fieldWidthM
    );
    hubSecondary.position.set(hfx, 0, hfz);
    hubSecondary.rotation.y = Math.PI;
    fieldRoot.add(hubSecondary);
    hubVfxRootsRef.current = [hubPrimary, hubSecondary];

    const robotGroup = new THREE.Group();
    fieldRoot.add(robotGroup);
    robotRef.current = robotGroup;

    const limelightGhost = createPhantomPoseGhost(robotLengthM, robotWidthM, 0xf97316, 0.34);
    fieldRoot.add(limelightGhost);
    limelightGhostRef.current = limelightGhost;

    const limelightFrontGhost = createPhantomPoseGhost(robotLengthM, robotWidthM, 0xec4899, 0.34);
    fieldRoot.add(limelightFrontGhost);
    limelightFrontGhostRef.current = limelightFrontGhost;

    const robotGeometry = new THREE.BoxGeometry(
      robotLengthM,
      ROBOT_HEIGHT_M,
      robotWidthM
    );
    const robotMaterial = new THREE.MeshStandardMaterial({
      color: 0x3b82f6,
      metalness: 0.25,
      roughness: 0.4,
    });
    const robot = new THREE.Mesh(robotGeometry, robotMaterial);
    robot.position.y = ROBOT_HEIGHT_M / 2;
    robot.castShadow = true;
    robot.receiveShadow = true;
    robotGroup.add(robot);

    const headingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, ROBOT_HEIGHT_M / 2 + 0.01, 0),
      Math.max(robotLengthM, robotWidthM) * 0.55,
      0xffd166
    );
    robotGroup.add(headingArrow);

    const refs = createTurretMesh(robotGroup, ROBOT_HEIGHT_M);
    turretMeshRefsRef.current = refs;
    turretRef.current = refs.turretRef;
    hoodPivotRef.current = refs.hoodPivotRef;
    pinionMeshRef.current = refs.pinionMeshRef;

    const vectorArrowsGroup = new THREE.Group();
    fieldRoot.add(vectorArrowsGroup);
    vectorArrowsGroupRef.current = vectorArrowsGroup;

    const headingVectorArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      0.9,
      0xffd166
    );
    vectorArrowsGroup.add(headingVectorArrow);
    headingVectorArrowRef.current = headingVectorArrow;

    const shooterVectorArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.35, 0),
      0.9,
      0xf97316
    );
    vectorArrowsGroup.add(shooterVectorArrow);
    shooterVectorArrowRef.current = shooterVectorArrow;

    const velocityVectorArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, -0.35, 0),
      0.3,
      0x22c55e
    );
    vectorArrowsGroup.add(velocityVectorArrow);
    velocityVectorArrowRef.current = velocityVectorArrow;

    const shotMarkersGroup = new THREE.Group();
    fieldRoot.add(shotMarkersGroup);
    shotMarkersGroupRef.current = shotMarkersGroup;

    const dropMarkersGroup = new THREE.Group();
    fieldRoot.add(dropMarkersGroup);
    dropMarkersGroupRef.current = dropMarkersGroup;

    const originMarkersGroup = new THREE.Group();
    fieldRoot.add(originMarkersGroup);
    originMarkersGroupRef.current = originMarkersGroup;

    const updateCamera = () => {
      const s = sphericalRef.current;
      const p = panOffsetRef.current;
      camera.position.set(
        p.x + s.radius * Math.sin(s.phi) * Math.sin(s.theta),
        p.y + s.radius * Math.cos(s.phi),
        p.z + s.radius * Math.sin(s.phi) * Math.cos(s.theta)
      );
      camera.lookAt(p);
    };

    const onMouseDown = (e: MouseEvent) => {
      pointerDownOnCanvasRef.current = true;
      hasDraggedRef.current = false;
      dragStartRef.current = { x: e.clientX, y: e.clientY };
      lastMouseRef.current = { x: e.clientX, y: e.clientY };

      const rect = renderer.domElement.getBoundingClientRect();
      const ndcX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      const ndcY = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      const cam = cameraRef.current;
      const { fieldLengthM: fL, fieldWidthM: fW } = fieldMetricsRef.current;
      const fr = fieldRootRef.current;

      if (mockModeRef.current && cam && robotRef.current) {
        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), cam);
        const robotHits = raycaster.intersectObject(robotRef.current, true);
        if (robotHits.length > 0) {
          const fieldHit = rayNdcToFieldXYWithFieldRoot(ndcX, ndcY, cam, fL, fW, fr);
          if (fieldHit) {
            robotDragActiveRef.current = true;
            robotDragOffsetRef.current = {
              x: fieldHit.x - poseXRef.current,
              y: fieldHit.y - poseYRef.current,
            };
            isDraggingRef.current = false;
            return;
          }
        }
      }

      robotDragActiveRef.current = false;
      isDraggingRef.current = true;
    };
    const onWindowMouseUp = (e: MouseEvent) => {
      robotDragActiveRef.current = false;
      if (
        pointerDownOnCanvasRef.current &&
        isDraggingRef.current &&
        !hasDraggedRef.current
      ) {
        const rect = renderer.domElement.getBoundingClientRect();
        if (
          e.clientX >= rect.left &&
          e.clientX <= rect.right &&
          e.clientY >= rect.top &&
          e.clientY <= rect.bottom
        ) {
          const ndcX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
          const ndcY = -((e.clientY - rect.top) / rect.height) * 2 + 1;
          const cam = cameraRef.current;
          const { fieldLengthM: fL, fieldWidthM: fW } = fieldMetricsRef.current;
          const fr = fieldRootRef.current;

          const pf = placeFieldModeRef.current;
          if ((pf === 'drop' || pf === 'origin' || pf === 'sample') && cam) {
            const fieldHit = rayNdcToFieldXYWithFieldRoot(ndcX, ndcY, cam, fL, fW, fr);
            if (fieldHit) {
              if (pf === 'drop') {
                onAddDropTargetRef.current(fieldHit.x, fieldHit.y);
              } else if (pf === 'origin') {
                onAddNamedOriginRef.current(fieldHit.x, fieldHit.y);
              } else {
                const idx = onAddShotSampleRef.current(fieldHit.x, fieldHit.y);
                setClickedShotIndexRef.current(idx);
              }
              setPlaceFieldModeRef.current('none');
            }
          } else if (cam) {
            const raycaster = new THREE.Raycaster();
            raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), cam);
            let foundShot: number | null = null;
            const shotGroup = shotMarkersGroupRef.current;
            if (shotGroup && shotGroup.children.length > 0) {
              const hits = raycaster.intersectObjects(shotGroup.children, true);
              for (const hit of hits) {
                let el: THREE.Object3D | null = hit.object;
                let isGhost = false;
                while (el) {
                  if (el.userData.ghostDuplicate) {
                    isGhost = true;
                    break;
                  }
                  el = el.parent;
                }
                if (isGhost) {
                  continue;
                }
                let o: THREE.Object3D | null = hit.object;
                while (o) {
                  if (typeof o.userData.mapIndex === 'number') {
                    foundShot = o.userData.mapIndex as number;
                    break;
                  }
                  o = o.parent;
                }
                if (foundShot !== null) break;
              }
            }
            if (foundShot !== null) {
              setClickedShotIndexRef.current(foundShot);
            } else {
              const dropGroup = dropMarkersGroupRef.current;
              if (dropGroup && dropGroup.children.length > 0) {
                const dHits = raycaster.intersectObjects(dropGroup.children, true);
                outer: for (const hit of dHits) {
                  let el: THREE.Object3D | null = hit.object;
                  while (el) {
                    if (el.userData.ghostDuplicate) {
                      continue outer;
                    }
                    el = el.parent;
                  }
                  let o: THREE.Object3D | null = hit.object;
                  while (o) {
                    if (typeof o.userData.dropTargetId === 'string') {
                      onSelectDropForAimRef.current(o.userData.dropTargetId as string);
                      break outer;
                    }
                    o = o.parent;
                  }
                }
              }
            }
          }
        }
      }
      pointerDownOnCanvasRef.current = false;
      isDraggingRef.current = false;
    };
    const onMouseMove = (e: MouseEvent) => {
      const rect = renderer.domElement.getBoundingClientRect();
      const ndcX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      const ndcY = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      const cam = cameraRef.current;
      const { fieldLengthM: fL, fieldWidthM: fW } = fieldMetricsRef.current;
      const fr = fieldRootRef.current;

      if (robotDragActiveRef.current && mockModeRef.current && cam) {
        const fieldHit = rayNdcToFieldXYWithFieldRoot(ndcX, ndcY, cam, fL, fW, fr);
        if (fieldHit) {
          const { x: ox, y: oy } = robotDragOffsetRef.current;
          const nx = THREE.MathUtils.clamp(fieldHit.x - ox, 0, fL);
          const ny = THREE.MathUtils.clamp(fieldHit.y - oy, 0, fW);
          onMockSimPatchSwerveRef.current({ x: nx, y: ny });
        }
        if (
          Math.hypot(
            e.clientX - dragStartRef.current.x,
            e.clientY - dragStartRef.current.y
          ) > 3
        ) {
          hasDraggedRef.current = true;
        }
        lastMouseRef.current = { x: e.clientX, y: e.clientY };
        return;
      }

      if (!isDraggingRef.current) return;
      if (
        Math.hypot(
          e.clientX - dragStartRef.current.x,
          e.clientY - dragStartRef.current.y
        ) > 5
      ) {
        hasDraggedRef.current = true;
      }
      const dx = (e.clientX - lastMouseRef.current.x) * 0.008;
      const dy = (e.clientY - lastMouseRef.current.y) * 0.008;
      lastMouseRef.current = { x: e.clientX, y: e.clientY };
      sphericalRef.current.theta -= dx;
      sphericalRef.current.phi = Math.max(
        0.15,
        Math.min(Math.PI - 0.15, sphericalRef.current.phi + dy)
      );
      updateCamera();
    };
    const onWheel = (e: WheelEvent) => {
      sphericalRef.current.radius = Math.max(
        3,
        Math.min(50, sphericalRef.current.radius + e.deltaY * 0.08)
      );
      updateCamera();
    };
    const PAN_SPEED = 8;
    const isTypingTarget = (t: EventTarget | null) => {
      if (!(t instanceof HTMLElement)) return false;
      if (t.isContentEditable) return true;
      const tag = t.tagName;
      return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT';
    };
    const onKeyDown = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return;
      const k = e.key.toLowerCase();
      if (k === 'w') keysRef.current.w = true;
      if (k === 's') keysRef.current.s = true;
      if (k === 'a') keysRef.current.a = true;
      if (k === 'd') keysRef.current.d = true;
      if (k === 'q') keysRef.current.q = true;
      if (k === 'e') keysRef.current.e = true;
      if (k === 'j') keysRef.current.j = true;
      if (k === 'l') keysRef.current.l = true;
    };
    const onKeyUp = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return;
      const k = e.key.toLowerCase();
      if (k === 'w') keysRef.current.w = false;
      if (k === 's') keysRef.current.s = false;
      if (k === 'a') keysRef.current.a = false;
      if (k === 'd') keysRef.current.d = false;
      if (k === 'q') keysRef.current.q = false;
      if (k === 'e') keysRef.current.e = false;
      if (k === 'j') keysRef.current.j = false;
      if (k === 'l') keysRef.current.l = false;
    };

    container.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mouseup', onWindowMouseUp);
    window.addEventListener('mousemove', onMouseMove);
    container.addEventListener('wheel', onWheel, { passive: true });
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);

    const handleResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    window.addEventListener('resize', handleResize);

    let flywheelRotation = 0;
    const animate = () => {
      const dt = clockRef.current.getDelta();
      const keys = keysRef.current;
      const mock = mockModeRef.current;

      if (mock) {
        let fwd = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
        let str = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
        const mag = Math.hypot(fwd, str);
        if (mag > 1e-6) {
          fwd /= mag;
          str /= mag;
        }
        const step = MOCK_SIM_DRIVE_MPS * dt;
        let dx = 0;
        let dy = 0;
        if (fieldRelativeRef.current) {
          dx = fwd * step;
          dy = str * step;
        } else {
          const th = THREE.MathUtils.degToRad(headingDegRef.current);
          const cos = Math.cos(th);
          const sin = Math.sin(th);
          dx = (fwd * cos + str * sin) * step;
          dy = (fwd * sin - str * cos) * step;
        }
        const rot =
          ((keys.j ? 1 : 0) - (keys.l ? 1 : 0)) * MOCK_SIM_ROT_DEG_PER_SEC * dt;
        const trans = keys.w || keys.s || keys.a || keys.d;
        const active = trans || keys.j || keys.l;
        if (active) {
          onMockSimKeyboardDriveRef.current({
            dx,
            dy,
            dHeadingDeg: rot,
            speedMps: trans ? MOCK_SIM_DRIVE_MPS * mag : 0,
          });
          wasMockKeyboardDrivingRef.current = true;
        } else if (wasMockKeyboardDrivingRef.current) {
          onMockSimKeyboardDriveRef.current({
            dx: 0,
            dy: 0,
            dHeadingDeg: 0,
            speedMps: 0,
          });
          wasMockKeyboardDrivingRef.current = false;
        }
        if (keys.q || keys.e) {
          const p = panOffsetRef.current;
          const move = PAN_SPEED * dt;
          if (keys.q) p.y += move;
          if (keys.e) p.y -= move;
          updateCamera();
        }
      } else if (keys.w || keys.s || keys.a || keys.d || keys.q || keys.e) {
        const p = panOffsetRef.current;
        const forward = new THREE.Vector3()
          .subVectors(p, camera.position)
          .setY(0)
          .normalize();
        if (forward.lengthSq() < 1e-6) forward.set(0, 0, 1);
        const right = new THREE.Vector3(-forward.z, 0, forward.x);
        const move = PAN_SPEED * dt;
        if (keys.w) p.addScaledVector(forward, move);
        if (keys.s) p.addScaledVector(forward, -move);
        if (keys.a) p.addScaledVector(right, -move);
        if (keys.d) p.addScaledVector(right, move);
        if (keys.q) p.y += move;
        if (keys.e) p.y -= move;
        updateCamera();
      }
      const refs = turretMeshRefsRef.current;
      if (refs) {
        const v = velocityMpsRef.current;
        const omega = v / FLYWHEEL_R;
        flywheelRotation += omega * dt;
        refs.flywheelGroupRef.rotation.x = flywheelRotation;
        const speedNorm = Math.min(1, Math.max(0, v / 30));
        refs.speedLightRef.intensity = speedNorm * 2.5;
        if (!refs.matWheelRef.emissive) refs.matWheelRef.emissive = new THREE.Color();
        refs.matWheelRef.emissive.setScalar(speedNorm * 0.4);
      }
      const t = clockRef.current.elapsedTime;
      for (const hubRoot of hubVfxRootsRef.current) {
        const funnel = hubRoot.userData.hubFunnel as THREE.Mesh | undefined;
        if (funnel?.material instanceof THREE.MeshStandardMaterial) {
          funnel.material.emissiveIntensity = 0.05 + 0.06 * Math.sin(t * 2.4);
        }
        const rim = hubRoot.userData.hubRim as THREE.Mesh | undefined;
        if (rim?.material instanceof THREE.MeshStandardMaterial) {
          rim.material.emissiveIntensity = 0.1 + 0.08 * Math.sin(t * 2.1 + 0.5);
        }
        const mouth = hubRoot.userData.hubMouthHint as THREE.Mesh | undefined;
        if (mouth?.material instanceof THREE.MeshBasicMaterial) {
          mouth.material.opacity = 0.22 + 0.1 * Math.sin(t * 1.8);
        }
        const intake = hubRoot.userData.hubIntakeRing as THREE.Mesh | undefined;
        if (intake?.material instanceof THREE.MeshBasicMaterial) {
          intake.material.opacity = 0.32 + 0.12 * Math.sin(t * 2.3 + 1.2);
        }
      }
      renderer.render(scene, camera);
      animationRef.current = requestAnimationFrame(animate);
    };
    animate();

    rendererRef.current = renderer;
    sceneRef.current = scene;
    cameraRef.current = camera;

    return () => {
      if (animationRef.current) cancelAnimationFrame(animationRef.current);
      window.removeEventListener('resize', handleResize);
      window.removeEventListener('mouseup', onWindowMouseUp);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      container.removeEventListener('mousedown', onMouseDown);
      container.removeEventListener('wheel', onWheel);
      for (const hr of hubVfxRootsRef.current) {
        disposeObject3DSubtree(hr);
      }
      hubVfxRootsRef.current = [];
      fieldRootRef.current = null;
      renderer.dispose();
      fieldGeometry.dispose();
      fieldMaterial.dispose();
      borderGeometry.dispose();
      borderMaterial.dispose();
      zoneBlueGeom.dispose();
      zoneBlueMat.dispose();
      zoneRedGeom.dispose();
      zoneRedMat.dispose();
      robotGeometry.dispose();
      robotMaterial.dispose();
      scene.clear();
      limelightGhostRef.current = null;
      limelightFrontGhostRef.current = null;
      turretMeshRefsRef.current = null;
      turretRef.current = null;
      hoodPivotRef.current = null;
      pinionMeshRef.current = null;
      vectorArrowsGroupRef.current = null;
      headingVectorArrowRef.current = null;
      shooterVectorArrowRef.current = null;
      velocityVectorArrowRef.current = null;
      shotMarkersGroupRef.current = null;
      dropMarkersGroupRef.current = null;
      originMarkersGroupRef.current = null;
      idealAimLineGroupRef.current = null;
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

  useEffect(() => {
    const fr = fieldRootRef.current;
    if (fr) {
      fr.rotation.y = viewAsRedAlliance ? Math.PI : 0;
    }
  }, [viewAsRedAlliance]);

  useEffect(() => {
    const robotGroup = robotRef.current;
    const vectorGroup = vectorArrowsGroupRef.current;
    if (!robotGroup) return;
    const { wx: worldX, wz: worldZ } = fieldPoseToWorldXZ(
      poseX,
      poseY,
      fieldLengthM,
      fieldWidthM
    );
    robotGroup.position.x = worldX;
    robotGroup.position.z = worldZ;
    robotGroup.position.y = 0;
    robotGroup.rotation.y = fieldHeadingDegToSceneYawRad(headingDeg);
    if (vectorGroup) {
      const hRad = fieldHeadingDegToSceneYawRad(headingDeg);
      vectorGroup.position.set(
        worldX + VECTOR_OFFSET * Math.cos(hRad),
        0.5,
        worldZ - VECTOR_OFFSET * Math.sin(hRad)
      );
      vectorGroup.rotation.y = hRad;
    }
  }, [fieldLengthM, fieldWidthM, headingDeg, poseX, poseY]);

  useEffect(() => {
    const ghost = limelightGhostRef.current;
    if (!ghost) return;
    ghost.visible = limelightPoseVisible;
    if (!limelightPoseVisible) return;
    const { wx, wz } = fieldPoseToWorldXZ(
      limelightPoseX ?? 0,
      limelightPoseY ?? 0,
      fieldLengthM,
      fieldWidthM
    );
    ghost.position.set(wx, 0, wz);
    ghost.rotation.y = fieldHeadingDegToSceneYawRad(limelightHeadingDeg ?? 0);
  }, [
    fieldLengthM,
    fieldWidthM,
    limelightHeadingDeg,
    limelightPoseVisible,
    limelightPoseX,
    limelightPoseY,
  ]);

  useEffect(() => {
    const ghost = limelightFrontGhostRef.current;
    if (!ghost) return;
    ghost.visible = limelightFrontPoseVisible;
    if (!limelightFrontPoseVisible) return;
    const { wx, wz } = fieldPoseToWorldXZ(
      limelightFrontPoseX ?? 0,
      limelightFrontPoseY ?? 0,
      fieldLengthM,
      fieldWidthM
    );
    ghost.position.set(wx, 0, wz);
    ghost.rotation.y = fieldHeadingDegToSceneYawRad(limelightFrontHeadingDeg ?? 0);
  }, [
    fieldLengthM,
    fieldWidthM,
    limelightFrontHeadingDeg,
    limelightFrontPoseVisible,
    limelightFrontPoseX,
    limelightFrontPoseY,
  ]);

  useEffect(() => {
    velocityMpsRef.current = velocityMps;
  }, [velocityMps]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const disposeIdealAimLine = () => {
      const existing = idealAimLineGroupRef.current;
      if (!existing) return;
      const parent = existing.parent;
      if (parent) parent.remove(existing);
      existing.traverse((child) => {
        if (child instanceof THREE.Line) {
          child.geometry.dispose();
          const mat = child.material;
          if (!Array.isArray(mat)) mat.dispose();
        }
      });
      idealAimLineGroupRef.current = null;
    };

    disposeIdealAimLine();

    if (!idealShooterPoseVisible) return;

    const startFieldX = THREE.MathUtils.clamp(idealShooterPoseX ?? poseX, 0, fieldLengthM);
    const startFieldY = THREE.MathUtils.clamp(idealShooterPoseY ?? poseY, 0, fieldWidthM);
    const headingRad = THREE.MathUtils.degToRad(idealShooterHeadingDeg ?? 0);
    const dirX = Math.cos(headingRad);
    const dirY = Math.sin(headingRad);
    const tCandidates: number[] = [];
    const EPS = 1e-6;

    if (Math.abs(dirX) > EPS) {
      tCandidates.push((0 - startFieldX) / dirX, (fieldLengthM - startFieldX) / dirX);
    }
    if (Math.abs(dirY) > EPS) {
      tCandidates.push((0 - startFieldY) / dirY, (fieldWidthM - startFieldY) / dirY);
    }

    const t = tCandidates
      .filter((candidate) => Number.isFinite(candidate) && candidate > 0)
      .reduce((best, candidate) => Math.min(best, candidate), Number.POSITIVE_INFINITY);

    const travelM = Number.isFinite(t) ? Math.max(0.6, t) : 3;
    const endFieldX = THREE.MathUtils.clamp(startFieldX + dirX * travelM, 0, fieldLengthM);
    const endFieldY = THREE.MathUtils.clamp(startFieldY + dirY * travelM, 0, fieldWidthM);
    const { wx: sx, wz: sz } = fieldPoseToWorldXZ(
      startFieldX,
      startFieldY,
      fieldLengthM,
      fieldWidthM
    );
    const { wx: ex, wz: ez } = fieldPoseToWorldXZ(
      endFieldX,
      endFieldY,
      fieldLengthM,
      fieldWidthM
    );

    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(sx, ROBOT_HEIGHT_M + 0.06, sz),
        new THREE.Vector3(ex, ROBOT_HEIGHT_M + 0.06, ez),
      ]),
      new THREE.LineDashedMaterial({
        color: 0x14b8a6,
        transparent: true,
        opacity: 0.82,
        dashSize: 0.25,
        gapSize: 0.16,
      })
    );
    line.computeLineDistances();

    const grp = new THREE.Group();
    grp.add(line);
    const parent = fieldRootRef.current ?? scene;
    parent.add(grp);
    idealAimLineGroupRef.current = grp;

    return disposeIdealAimLine;
  }, [
    fieldLengthM,
    fieldWidthM,
    idealShooterHeadingDeg,
    idealShooterPoseVisible,
    idealShooterPoseX,
    idealShooterPoseY,
    poseX,
    poseY,
  ]);

  useEffect(() => {
    const turret = turretRef.current;
    if (!turret) return;
    turret.rotation.y = -Math.PI / 2 + (turretAngleDeg * Math.PI) / 180;
  }, [turretAngleDeg]);

  useEffect(() => {
    const hoodPivot = hoodPivotRef.current;
    if (!hoodPivot) return;
    hoodPivot.rotation.x = (hoodPitchDeg * Math.PI) / 180;
  }, [hoodPitchDeg]);

  useEffect(() => {
    const pinionMesh = pinionMeshRef.current;
    if (!pinionMesh) return;
    const pinionAngleDeg = hoodPitchDeg * GEAR_RATIO;
    pinionMesh.rotation.x = Math.PI / 2 + (pinionAngleDeg * Math.PI) / 180;
  }, [hoodPitchDeg]);

  useEffect(() => {
    const shooterArrow = shooterVectorArrowRef.current;
    if (!shooterArrow) return;
    const turretRad = (turretAngleDeg * Math.PI) / 180;
    const hoodRad = (hoodPitchDeg * Math.PI) / 180;
    const dx = Math.cos(turretRad) * Math.cos(hoodRad);
    const dy = Math.sin(hoodRad);
    const dz = Math.sin(turretRad) * Math.cos(hoodRad);
    shooterArrow.setDirection(new THREE.Vector3(dx, dy, dz).normalize());
    shooterArrow.setLength(0.9);
  }, [turretAngleDeg, hoodPitchDeg]);

  useEffect(() => {
    const velocityArrow = velocityVectorArrowRef.current;
    if (!velocityArrow) return;
    const len = Math.max(0.08, Math.min(1.2, velocityMps * 0.06));
    velocityArrow.setLength(len);
  }, [velocityMps]);

  useEffect(() => {
    const group = shotMarkersGroupRef.current;
    if (!group) return;

    const disposeMarkerGeometries = (root: THREE.Object3D) => {
      root.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose();
        }
      });
    };

    while (group.children.length > 0) {
      const ch = group.children[0];
      group.remove(ch);
      disposeMarkerGeometries(ch);
    }

    const offsets = duplicateOffsetsForShots(shotMapPoints);
    const matDefault = new THREE.MeshStandardMaterial({
      color: 0x34d399,
      metalness: 0.15,
      roughness: 0.55,
    });
    const matSelected = new THREE.MeshStandardMaterial({
      color: 0xf472b6,
      emissive: 0x9d174d,
      emissiveIntensity: 0.4,
      metalness: 0.1,
      roughness: 0.45,
    });
    const matInspect = new THREE.MeshStandardMaterial({
      color: 0xfbbf24,
      emissive: 0xb45309,
      emissiveIntensity: 0.48,
      metalness: 0.2,
      roughness: 0.4,
    });

    const fieldSize: FieldSize = { sizeXM: fieldLengthM, sizeYM: fieldWidthM };
    const hubFieldFlipped = flipFieldPositionRotational(HUB_FIELD_X_M, HUB_FIELD_Y_M, fieldSize);

    const addKnnMarkerAtField = (
      fieldPx: number,
      fieldPy: number,
      lift: number,
      headingDeg: number,
      mapIndex: number | undefined,
      mapFitHubX: number,
      mapFitHubY: number
    ) => {
      const px = THREE.MathUtils.clamp(fieldPx, 0, fieldLengthM);
      const py = THREE.MathUtils.clamp(fieldPy, 0, fieldWidthM);
      const { wx, wz } = fieldPoseToWorldXZ(px, py, fieldLengthM, fieldWidthM);

      const markerRoot = new THREE.Group();
      markerRoot.position.set(wx, 0, wz);
      if (mapIndex !== undefined) {
        markerRoot.userData.mapIndex = mapIndex;
      } else {
        markerRoot.userData.ghostDuplicate = true;
      }

      const isPrimary = mapIndex !== undefined;
      const isClick = isPrimary && clickedShotIndex === mapIndex;
      const isNt =
        isPrimary && effectiveKnnSelectedIndex >= 0 && effectiveKnnSelectedIndex === mapIndex;
      const isInfBlue =
        isPrimary && effectiveKnnNearestBlue >= 0 && effectiveKnnNearestBlue === mapIndex;
      const isInfRed =
        isPrimary && effectiveKnnNearestRed >= 0 && effectiveKnnNearestRed === mapIndex;
      const sphereMat = isClick ? matInspect : isNt ? matSelected : matDefault;
      const arrowColor = isClick ? 0xfde68a : isNt ? 0xf9a8d4 : 0x6ee7b7;

      const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.11, 10, 8), sphereMat);
      sphere.position.y = 0.12 + lift;
      sphere.castShadow = true;
      markerRoot.add(sphere);

      if (isInfBlue) {
        const ringB = new THREE.Mesh(
          new THREE.RingGeometry(0.13, 0.2, 28),
          new THREE.MeshBasicMaterial({
            color: 0x22d3ee,
            transparent: true,
            opacity: 0.95,
            side: THREE.DoubleSide,
          })
        );
        ringB.rotation.x = -Math.PI / 2;
        ringB.position.y = 0.025 + lift;
        ringB.renderOrder = 1;
        markerRoot.add(ringB);
      }
      if (isInfRed) {
        const ringR = new THREE.Mesh(
          new THREE.RingGeometry(0.21, 0.3, 28),
          new THREE.MeshBasicMaterial({
            color: 0xfb923c,
            transparent: true,
            opacity: 0.92,
            side: THREE.DoubleSide,
          })
        );
        ringR.rotation.x = -Math.PI / 2;
        ringR.position.y = 0.028 + lift;
        ringR.renderOrder = 1;
        markerRoot.add(ringR);
      }

      const dir = fieldHeadingDegToSceneForwardXZ(headingDeg);
      const arrow = new THREE.ArrowHelper(
        dir,
        new THREE.Vector3(0, 0.11 + lift, 0),
        0.52,
        arrowColor,
        0.11,
        0.07
      );
      markerRoot.add(arrow);

      if (showShotMapHubHeading) {
        const hdx = mapFitHubX - px;
        const hdy = mapFitHubY - py;
        const mapFitDeg =
          hdx * hdx + hdy * hdy < HUB_SHOT_MAP_MIN_DIST_SQ_M2
            ? 0
            : normalizeHeadingDeg(
                (Math.atan2(hdy, hdx) + hubShotMapHeadingOffsetRad(hdx, hdy)) * (180 / Math.PI) +
                  HUB_FACING_OFFSET_DEG
              );
        const dirMap = fieldHeadingDegToSceneForwardXZ(mapFitDeg);
        const arrowMap = new THREE.ArrowHelper(
          dirMap,
          new THREE.Vector3(0, 0.22 + lift, 0),
          0.48,
          0xc4b5fd,
          0.1,
          0.065
        );
        markerRoot.add(arrowMap);
      }

      group.add(markerRoot);
    };

    for (let index = 0; index < shotMapPoints.length; index++) {
      const p = shotMapPoints[index];
      if (isPlaceholderKnnPoint(p)) {
        continue;
      }
      const { dx, dy, lift } = offsets[index] ?? { dx: 0, dy: 0, lift: 0 };
      const px = THREE.MathUtils.clamp(p.x + dx, 0, fieldLengthM);
      const py = THREE.MathUtils.clamp(p.y + dy, 0, fieldWidthM);
      const pyField = knnMapPointFieldYForAlliance(py, isRedAlliance, fieldWidthM);

      addKnnMarkerAtField(px, pyField, lift, p.headingDeg ?? 0, index, HUB_FIELD_X_M, HUB_FIELD_Y_M);

      const flipped = flipFieldPositionRotational(px, py, fieldSize);
      addKnnMarkerAtField(
        flipped.x,
        flipped.y,
        lift,
        flipFieldHeadingDegRotational(p.headingDeg ?? 0),
        undefined,
        hubFieldFlipped.x,
        hubFieldFlipped.y
      );
    }

    return () => {
      while (group.children.length > 0) {
        const ch = group.children[0];
        group.remove(ch);
        disposeMarkerGeometries(ch);
      }
      matDefault.dispose();
      matSelected.dispose();
      matInspect.dispose();
    };
  }, [
    clickedShotIndex,
    fieldLengthM,
    fieldWidthM,
    effectiveKnnSelectedIndex,
    effectiveKnnNearestBlue,
    effectiveKnnNearestRed,
    shotMapPoints,
    showShotMapHubHeading,
    isRedAlliance,
  ]);

  useEffect(() => {
    const group = dropMarkersGroupRef.current;
    if (!group) return;

    const disposeSubtree = (root: THREE.Object3D) => {
      root.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose();
          const mat = child.material;
          if (Array.isArray(mat)) {
            mat.forEach((m) => m.dispose());
          } else {
            mat?.dispose();
          }
        }
      });
    };

    while (group.children.length > 0) {
      const ch = group.children[0];
      group.remove(ch);
      disposeSubtree(ch);
    }

    const fieldSize: FieldSize = { sizeXM: fieldLengthM, sizeYM: fieldWidthM };

    const addDropCone = (
      fieldX: number,
      fieldY: number,
      id: string | undefined,
      ghost: boolean
    ) => {
      const { wx, wz } = fieldPoseToWorldXZ(fieldX, fieldY, fieldLengthM, fieldWidthM);
      const markerRoot = new THREE.Group();
      markerRoot.position.set(wx, 0, wz);
      if (ghost) {
        markerRoot.userData.ghostDuplicate = true;
      } else if (id) {
        markerRoot.userData.dropTargetId = id;
      }

      const isSel = id != null && selectedDropTargetId === id;
      const coneMat = new THREE.MeshStandardMaterial({
        color: isSel ? 0xfbbf24 : 0x38bdf8,
        emissive: isSel ? 0xb45309 : 0x000000,
        emissiveIntensity: isSel ? 0.35 : 0,
        metalness: 0.2,
        roughness: 0.45,
      });
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.36, 8), coneMat);
      cone.position.y = 0.2;
      cone.castShadow = true;
      markerRoot.add(cone);

      const ringMat = new THREE.MeshBasicMaterial({
        color: isSel ? 0xfde68a : 0x7dd3fc,
        transparent: true,
        opacity: ghost ? 0.5 : 0.85,
        side: THREE.DoubleSide,
      });
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.08, 0.2, 24), ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.02;
      markerRoot.add(ring);

      group.add(markerRoot);
    };

    for (const t of dropTargets) {
      addDropCone(t.x, t.y, t.id, false);
      const f = flipFieldPositionRotational(t.x, t.y, fieldSize);
      addDropCone(f.x, f.y, undefined, true);
    }

    return () => {
      while (group.children.length > 0) {
        const ch = group.children[0];
        group.remove(ch);
        disposeSubtree(ch);
      }
    };
  }, [dropTargets, fieldLengthM, fieldWidthM, selectedDropTargetId]);

  useEffect(() => {
    const group = originMarkersGroupRef.current;
    if (!group) return;

    const disposeSubtree = (root: THREE.Object3D) => {
      root.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose();
          const mat = child.material;
          if (Array.isArray(mat)) {
            mat.forEach((m) => m.dispose());
          } else {
            mat?.dispose();
          }
        }
      });
    };

    while (group.children.length > 0) {
      const ch = group.children[0];
      group.remove(ch);
      disposeSubtree(ch);
    }

    const fieldSize: FieldSize = { sizeXM: fieldLengthM, sizeYM: fieldWidthM };

    const addOriginPillar = (
      fieldX: number,
      fieldY: number,
      originId: string | undefined,
      ghost: boolean
    ) => {
      const { wx, wz } = fieldPoseToWorldXZ(fieldX, fieldY, fieldLengthM, fieldWidthM);
      const markerRoot = new THREE.Group();
      markerRoot.position.set(wx, 0, wz);
      if (ghost) {
        markerRoot.userData.ghostDuplicate = true;
      } else if (originId) {
        markerRoot.userData.namedOriginId = originId;
      }

      const pillarMat = new THREE.MeshStandardMaterial({
        color: 0xc084fc,
        emissive: 0x6b21a8,
        emissiveIntensity: ghost ? 0.12 : 0.2,
        metalness: 0.25,
        roughness: 0.5,
        transparent: ghost,
        opacity: ghost ? 0.65 : 1,
      });
      const pillar = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.32, 10), pillarMat);
      pillar.position.y = 0.18;
      pillar.castShadow = true;
      markerRoot.add(pillar);

      const capMat = new THREE.MeshStandardMaterial({
        color: 0xe9d5ff,
        metalness: 0.35,
        roughness: 0.4,
        transparent: ghost,
        opacity: ghost ? 0.65 : 1,
      });
      const cap = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 10), capMat);
      cap.position.y = 0.38;
      cap.castShadow = true;
      markerRoot.add(cap);

      group.add(markerRoot);
    };

    for (const o of namedOrigins) {
      addOriginPillar(o.x, o.y, o.id, false);
      const f = flipFieldPositionRotational(o.x, o.y, fieldSize);
      addOriginPillar(f.x, f.y, undefined, true);
    }

    return () => {
      while (group.children.length > 0) {
        const ch = group.children[0];
        group.remove(ch);
        disposeSubtree(ch);
      }
    };
  }, [namedOrigins, fieldLengthM, fieldWidthM]);

  useEffect(() => {
    if (!mockMode) return;
    const ae = document.activeElement;
    if (
      mockPoseFormRef.current &&
      ae instanceof Node &&
      mockPoseFormRef.current.contains(ae)
    ) {
      return;
    }
    setMockPoseXStr(String(poseX));
    setMockPoseYStr(String(poseY));
    setMockPoseHStr(String(headingDeg));
  }, [mockMode, poseX, poseY, headingDeg]);

  const applyMockPoseFromPanel = useCallback(() => {
    const x = parseFloat(mockPoseXStr);
    const y = parseFloat(mockPoseYStr);
    const h = parseFloat(mockPoseHStr);
    if (![x, y, h].every(Number.isFinite)) return;
    onMockSimPatchSwerve({
      x: THREE.MathUtils.clamp(x, 0, fieldLengthM),
      y: THREE.MathUtils.clamp(y, 0, fieldWidthM),
      headingDeg: normalizeHeadingDeg(h),
    });
  }, [
    mockPoseXStr,
    mockPoseYStr,
    mockPoseHStr,
    fieldLengthM,
    fieldWidthM,
    onMockSimPatchSwerve,
  ]);

  const resetMockPoseFormFromSim = useCallback(() => {
    setMockPoseXStr(String(poseX));
    setMockPoseYStr(String(poseY));
    setMockPoseHStr(String(headingDeg));
  }, [poseX, poseY, headingDeg]);

  const globalAimFieldTarget = useMemo(() => {
    if (aimTargetKind === 'hub') {
      return {
        x: HUB_FIELD_X_M,
        y: HUB_FIELD_Y_M,
        label: 'Hub',
      };
    }
    const d = dropTargets.find((p) => p.id === selectedDropTargetId);
    if (d) {
      return { x: d.x, y: d.y, label: d.label };
    }
    return {
      x: HUB_FIELD_X_M,
      y: HUB_FIELD_Y_M,
      label: 'Hub',
    };
  }, [aimTargetKind, dropTargets, selectedDropTargetId]);

  /** Persisted per-map-point target overrides the live hub / aim-point picker while inspecting that shot. */
  const effectiveAimFieldTarget = useMemo(() => {
    if (clickedShotIndex === null) {
      return globalAimFieldTarget;
    }
    const p = shotMapPoints[clickedShotIndex];
    if (!p || isPlaceholderKnnPoint(p)) {
      return globalAimFieldTarget;
    }
    if (p.shootTarget?.kind === 'field') {
      return {
        x: p.shootTarget.x,
        y: p.shootTarget.y,
        label: p.shootTarget.label ?? 'Saved aim',
      };
    }
    return globalAimFieldTarget;
  }, [clickedShotIndex, shotMapPoints, globalAimFieldTarget]);

  const selectedDropForAim = useMemo(() => {
    if (aimTargetKind !== 'drop') {
      return null;
    }
    return dropTargets.find((d) => d.id === selectedDropTargetId) ?? null;
  }, [aimTargetKind, dropTargets, selectedDropTargetId]);

  const alignmentOriginResolved = useMemo(() => {
    if (!selectedDropForAim) {
      return null;
    }
    return resolveSimAlignmentOrigin(selectedDropForAim.alignmentOriginId, namedOrigins);
  }, [selectedDropForAim, namedOrigins]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const existing = aimLineGroupRef.current;
    if (existing) {
      const parent = existing.parent;
      if (parent) parent.remove(existing);
      existing.traverse((child) => {
        if (child instanceof THREE.Line) {
          child.geometry.dispose();
          const mat = child.material;
          if (!Array.isArray(mat)) mat.dispose();
        }
      });
      aimLineGroupRef.current = null;
    }

    if (clickedShotIndex === null) return;
    const p = shotMapPoints[clickedShotIndex];
    if (!p || isPlaceholderKnnPoint(p)) return;

    const offsets = duplicateOffsetsForShots(shotMapPoints);
    const { dx, dy } = offsets[clickedShotIndex] ?? { dx: 0, dy: 0 };
    const px = THREE.MathUtils.clamp(p.x + dx, 0, fieldLengthM);
    const py = THREE.MathUtils.clamp(p.y + dy, 0, fieldWidthM);
    const { wx, wz } = fieldPoseToWorldXZ(px, py, fieldLengthM, fieldWidthM);
    const { wx: tx, wz: tz } = fieldPoseToWorldXZ(
      effectiveAimFieldTarget.x,
      effectiveAimFieldTarget.y,
      fieldLengthM,
      fieldWidthM
    );

    const lineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(wx, 0.09, wz),
      new THREE.Vector3(tx, 0.09, tz),
    ]);
    const lineMat = new THREE.LineBasicMaterial({
      color: 0xfbbf24,
      transparent: true,
      opacity: 0.9,
    });
    const line = new THREE.Line(lineGeom, lineMat);
    const grp = new THREE.Group();
    grp.add(line);
    const parent = fieldRootRef.current ?? scene;
    parent.add(grp);
    aimLineGroupRef.current = grp;

    return () => {
      const g = aimLineGroupRef.current;
      if (!g) return;
      const parent = g.parent;
      if (parent) parent.remove(g);
      g.traverse((child) => {
        if (child instanceof THREE.Line) {
          child.geometry.dispose();
          const mat = child.material;
          if (!Array.isArray(mat)) mat.dispose();
        }
      });
      aimLineGroupRef.current = null;
    };
  }, [
    effectiveAimFieldTarget.x,
    effectiveAimFieldTarget.y,
    clickedShotIndex,
    fieldLengthM,
    fieldWidthM,
    shotMapPoints,
  ]);

  useEffect(() => {
    if (clickedShotIndex === null) return;
    if (clickedShotIndex >= shotMapPoints.length) {
      setClickedShotIndex(null);
    }
  }, [clickedShotIndex, shotMapPoints.length]);

  /** Drop mode + shot selected: ensure an aim point is chosen so the inspect panel can compute heading. */
  useEffect(() => {
    if (clickedShotIndex === null) return;
    const p = shotMapPoints[clickedShotIndex];
    if (p?.shootTarget?.kind === 'field') {
      return;
    }
    if (aimTargetKind !== 'drop' || dropTargets.length === 0) return;
    if (
      selectedDropTargetId &&
      dropTargets.some((d) => d.id === selectedDropTargetId)
    ) {
      return;
    }
    onSelectDropForAim(dropTargets[0].id);
  }, [
    aimTargetKind,
    clickedShotIndex,
    dropTargets,
    onSelectDropForAim,
    selectedDropTargetId,
    shotMapPoints,
  ]);

  const shotPlacesCount = shotMapPoints.filter((p) => !isPlaceholderKnnPoint(p)).length;

  const driverStationSummary = useMemo(() => {
    const side = viewAsRedAlliance ? 'Red' : 'Blue';
    const pos =
      driverStationStationNumber != null ? ` · station ${driverStationStationNumber}` : '';
    const mirror = viewAsRedAlliance ? ' · mirrored view' : '';
    return `Field ${side}${pos}${mirror}`;
  }, [viewAsRedAlliance, driverStationStationNumber]);

  const inspected =
    clickedShotIndex !== null ? shotMapPoints[clickedShotIndex] ?? null : null;
  const inspectedOk = inspected != null && !isPlaceholderKnnPoint(inspected);
  const offsetsInspect =
    clickedShotIndex !== null ? duplicateOffsetsForShots(shotMapPoints) : [];
  const io = clickedShotIndex !== null ? offsetsInspect[clickedShotIndex] : undefined;
  const aimFromX =
    inspectedOk && inspected
      ? THREE.MathUtils.clamp(inspected.x + (io?.dx ?? 0), 0, fieldLengthM)
      : 0;
  const aimFromY =
    inspectedOk && inspected
      ? THREE.MathUtils.clamp(inspected.y + (io?.dy ?? 0), 0, fieldWidthM)
      : 0;
  const shotUsesPersistedFieldAim =
    inspectedOk && inspected?.shootTarget?.kind === 'field';

  const optimalAimHeadingDeg =
    inspectedOk
      ? shotUsesPersistedFieldAim
        ? optimalHeadingTowardFieldPointDeg(
            aimFromX,
            aimFromY,
            effectiveAimFieldTarget.x,
            effectiveAimFieldTarget.y
          )
        : aimTargetKind === 'hub'
          ? hubShotMapHeadingTowardHubDeg(aimFromX, aimFromY)
          : selectedDropForAim && alignmentOriginResolved
            ? optimalHeadingTowardFieldPointDeg(
                alignmentOriginResolved.x,
                alignmentOriginResolved.y,
                effectiveAimFieldTarget.x,
                effectiveAimFieldTarget.y
              )
            : null
      : null;
  const loggedHeadingDeg = inspected?.headingDeg ?? 0;
  const aimHeadingDeltaDeg =
    optimalAimHeadingDeg !== null
      ? shortestAngleDeltaDeg(loggedHeadingDeg, optimalAimHeadingDeg)
      : null;
  const aimDistanceM =
    inspectedOk
      ? distanceBetweenFieldPointsM(
          aimFromX,
          aimFromY,
          effectiveAimFieldTarget.x,
          effectiveAimFieldTarget.y
        )
      : null;

  const alignmentRayLabel =
    shotUsesPersistedFieldAim
      ? `Sample pose → ${effectiveAimFieldTarget.label}`
      : aimTargetKind === 'hub'
        ? 'Sample pose → Hub'
        : selectedDropForAim && alignmentOriginResolved
          ? `${alignmentOriginResolved.displayName} → ${selectedDropForAim.label}`
          : '—';

  const showHeadingAnalysis =
    inspectedOk &&
    optimalAimHeadingDeg !== null &&
    aimHeadingDeltaDeg !== null;

  const putAimPointOnSamplePose = () => {
    if (
      aimTargetKind === 'drop' &&
      selectedDropTargetId &&
      dropTargets.some((d) => d.id === selectedDropTargetId)
    ) {
      onUpdateDropTarget(selectedDropTargetId, { x: aimFromX, y: aimFromY });
    } else {
      onAddDropTarget(aimFromX, aimFromY);
    }
  };

  const persistShootTargetFromPicker = () => {
    if (clickedShotIndex === null || !inspectedOk) return;
    const target: KnnShootTarget =
      aimTargetKind === 'hub'
        ? { kind: 'hub' }
        : {
            kind: 'field',
            x: globalAimFieldTarget.x,
            y: globalAimFieldTarget.y,
            label: globalAimFieldTarget.label,
          };
    onSaveKnnShootTarget(clickedShotIndex, target);
  };

  return (
    <section className="simulation-view" ref={simulationSectionRef}>
      <div className="simulation-canvas" ref={containerRef} />
      <p className="simulation-nav-hint" aria-hidden="true">
        WASD — pan · Q/E — up/down · Mouse — orbit · Wheel — zoom · Hex funnel — hubs (both alliances) ·
        Blue/red floor tint · low field X = blue · high X = red · Symmetric KNN/drop/origin ghosts (decorative
        only) · Teal dashed line — ideal shooting angle · Click KNN shot — aim line · Violet pillar — origins
        · Cyan — drops · Teal — map · Gold — selected shot · Pink — NT nearest · Optional lavender — map-fit
        hub heading (checkbox)
      </p>
      <div
        className="simulation-vectors-panel"
        aria-hidden="true"
        {...vectorsPanelDrag.panelProps}
      >
        <div
          className="simulation-vectors-title simulation-panel-drag-handle"
          title="Drag panel"
          {...vectorsPanelDrag.handleProps}
        >
          Vectors
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#ffd166' }} />
          <span className="simulation-vector-label">Heading</span>
          <span className="simulation-vector-value">{headingDeg.toFixed(1)}°</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#f97316' }} />
          <span className="simulation-vector-label">Turret</span>
          <span className="simulation-vector-value">{turretAngleDeg.toFixed(1)}°</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#f97316' }} />
          <span className="simulation-vector-label">Hood</span>
          <span className="simulation-vector-value">{hoodPitchDeg.toFixed(1)}°</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#22c55e' }} />
          <span className="simulation-vector-label">Velocity</span>
          <span className="simulation-vector-value">{velocityMps.toFixed(1)} m/s</span>
        </div>
        <div className="simulation-vector-row simulation-vector-pose">
          <span className="simulation-vector-label">Pose</span>
          <span className="simulation-vector-value">
            ({poseX.toFixed(2)}, {poseY.toFixed(2)})
          </span>
        </div>
        <div className="simulation-vector-row simulation-vector-pose">
          <span className="simulation-vector-label">DS / FMS</span>
          <span className="simulation-vector-value" title="FMSInfo when connected; mock defaults to blue">
            {driverStationSummary}
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#14b8a6' }} />
          <span className="simulation-vector-label">Ideal aim</span>
          <span className="simulation-vector-value">
            {idealShooterPoseVisible ? `${(idealShooterHeadingDeg ?? 0).toFixed(1)}°` : '—'}
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#f97316' }} />
          <span className="simulation-vector-label">LL pose</span>
          <span className="simulation-vector-value">
            {limelightPoseVisible
              ? `${(limelightPoseX ?? 0).toFixed(2)}, ${(limelightPoseY ?? 0).toFixed(2)}`
              : limelightHasLock
                ? 'Lock only'
                : '—'}
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#ec4899' }} />
          <span className="simulation-vector-label">LL front</span>
          <span className="simulation-vector-value">
            {limelightFrontPoseVisible
              ? `${(limelightFrontPoseX ?? 0).toFixed(2)}, ${(limelightFrontPoseY ?? 0).toFixed(2)}`
              : limelightFrontHasLock
                ? 'Lock only'
                : '—'}
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#94a3b8' }} />
          <span className="simulation-vector-label">NT</span>
          <span className="simulation-vector-value">{connected ? 'Live' : 'Mock / off'}</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#a78bfa' }} />
          <span className="simulation-vector-label">Drive ω</span>
          <span className="simulation-vector-value">{swerveSpeedMps.toFixed(2)} m/s</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#fb7185' }} />
          <span className="simulation-vector-label">Shooter</span>
          <span className="simulation-vector-value">
            {Math.round(shooterRpm)} / {Math.round(shooterRpmSetpoint)} rpm
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#fdba74' }} />
          <span className="simulation-vector-label">Hood SP</span>
          <span className="simulation-vector-value">{hoodSetpointDeg.toFixed(1)}°</span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#38bdf8' }} />
          <span className="simulation-vector-label">Turret track</span>
          <span className="simulation-vector-value">{turretTracking ? 'On' : 'Off'}</span>
        </div>
        <div className="simulation-vector-row simulation-vector-pose">
          <span className="simulation-vector-label">KNN map</span>
          <span className="simulation-vector-value">
            {shotPlacesCount} shots
            {effectiveKnnSelectedIndex >= 0 ? ` · sel #${effectiveKnnSelectedIndex}` : ''}
            {effectiveKnnNearestBlue >= 0 ? (
              <span title="Nearest row, raw WPIBlue pose">
                {' '}
                · <span style={{ color: '#22d3ee' }}>B{effectiveKnnNearestBlue}</span>
              </span>
            ) : null}
            {effectiveKnnNearestRed >= 0 &&
            effectiveKnnNearestRed !== effectiveKnnNearestBlue ? (
              <span title="Legacy NT nearestIndexRed (Y-mirror lookup)">
                {' '}
                · <span style={{ color: '#fb923c' }}>R{effectiveKnnNearestRed}</span>
              </span>
            ) : null}
          </span>
        </div>
        <div className="simulation-vector-row">
          <span className="simulation-vector-swatch" style={{ background: '#86efac' }} />
          <span className="simulation-vector-label">KNN hood / RPM SP</span>
          <span className="simulation-vector-value" title="Smoothed IDW targets while driving (robot)">
            {hoodSetpointDeg.toFixed(1)}° · {Math.round(shooterRpmSetpoint)} rpm
          </span>
        </div>
        <label className="simulation-vector-row simulation-vector-checkbox-row">
          <input
            type="checkbox"
            checked={showShotMapHubHeading}
            onChange={(e) => setShowShotMapHubHeading(e.target.checked)}
          />
          <span className="simulation-vector-label">Map-fit hub heading</span>
          <span
            className="simulation-vector-swatch"
            style={{ background: '#c4b5fd' }}
            title="Violet arrow: DriveConstants.rotationToFaceHubFromShotMap (linear fit)"
          />
        </label>
      </div>
      <div className="simulation-overlay" aria-hidden="true">
        <div className="simulation-overlay-left">
          <span className="simulation-badge">Simulation</span>
          {mockMode ? (
            <span
              className="simulation-mock-keys-hint"
              title="Uses the same field vs robot-relative mode as the Dashboard tab"
            >
              Mock: Aim panel — pose · WASD move · J/L rotate · Q/E camera height
              {fieldRelative ? ' · field-relative' : ' · robot-relative'}
            </span>
          ) : null}
        </div>
        <span className="simulation-stats">
          {driverStationSummary} · Pose ({poseX.toFixed(1)}, {poseY.toFixed(1)}) · Heading{' '}
          {headingDeg.toFixed(0)}° · Turret {turretAngleDeg.toFixed(0)}° · Hood {hoodPitchDeg.toFixed(0)}° ·
          Flywheel {velocityMps.toFixed(1)} m/s · Shooter {Math.round(shooterRpm)}/
          {Math.round(shooterRpmSetpoint)} rpm · KNN sel{' '}
          {effectiveKnnSelectedIndex >= 0 ? `#${effectiveKnnSelectedIndex}` : '—'} · B
          {effectiveKnnNearestBlue >= 0 ? effectiveKnnNearestBlue : '—'}
          {effectiveKnnNearestRed >= 0 && effectiveKnnNearestRed !== effectiveKnnNearestBlue
            ? ` · R${effectiveKnnNearestRed}`
            : ''}{' '}
          · {shotPlacesCount} map shots
        </span>
      </div>

      <aside
        className="simulation-aim-panel"
        aria-label="Aim target for KNN inspection"
        {...aimPanelDrag.panelProps}
      >
        <h3 className="simulation-panel-drag-handle" title="Drag panel" {...aimPanelDrag.handleProps}>
          Aim target
        </h3>
        <div className="simulation-aim-row">
          <button
            type="button"
            className={aimTargetKind === 'hub' ? 'active' : ''}
            onClick={() => onAimTargetKindChange('hub')}
          >
            Hub (default)
          </button>
          <button
            type="button"
            className={aimTargetKind === 'drop' ? 'active' : ''}
            onClick={() => onAimTargetKindChange('drop')}
            disabled={dropTargets.length === 0}
            title={dropTargets.length === 0 ? 'Add an aim point first' : undefined}
          >
            Aim point
          </button>
        </div>

        {mockMode ? (
          <>
            <h4 className="simulation-aim-sub">Mock robot pose</h4>
            <p className="simulation-aim-hint simulation-mock-pose-hint">
              Field X / Y (m) and heading (°). The robot often starts under this panel at the blue
              corner, so use these fields instead of clicking the 3D model.
            </p>
            <div ref={mockPoseFormRef} className="simulation-mock-pose-form">
              <label className="simulation-mock-pose-field">
                <span>X</span>
                <input
                  type="number"
                  inputMode="decimal"
                  step={0.01}
                  min={0}
                  max={fieldLengthM}
                  value={mockPoseXStr}
                  onChange={(e) => setMockPoseXStr(e.target.value)}
                  aria-label="Mock pose field X meters"
                />
              </label>
              <label className="simulation-mock-pose-field">
                <span>Y</span>
                <input
                  type="number"
                  inputMode="decimal"
                  step={0.01}
                  min={0}
                  max={fieldWidthM}
                  value={mockPoseYStr}
                  onChange={(e) => setMockPoseYStr(e.target.value)}
                  aria-label="Mock pose field Y meters"
                />
              </label>
              <label className="simulation-mock-pose-field">
                <span>Heading</span>
                <input
                  type="number"
                  inputMode="decimal"
                  step={1}
                  value={mockPoseHStr}
                  onChange={(e) => setMockPoseHStr(e.target.value)}
                  aria-label="Mock pose heading degrees"
                />
              </label>
              <div className="simulation-mock-pose-actions">
                <button type="button" className="simulation-mock-pose-apply" onClick={applyMockPoseFromPanel}>
                  Apply pose
                </button>
                <button type="button" className="simulation-mock-pose-reset" onClick={resetMockPoseFormFromSim}>
                  Reset fields
                </button>
              </div>
            </div>
          </>
        ) : null}

        <h4 className="simulation-aim-sub">Alignment origins</h4>
        <p className="simulation-aim-hint">
          <strong>Hub</strong> is the default origin. Optimal heading for each aim point uses the field
          ray from its assigned origin to that point (named origins are optional).
        </p>
        <ul className="simulation-aim-origin-list">
          <li className="simulation-aim-origin-built-in">Hub — built-in (default)</li>
          {namedOrigins.map((o) => (
            <li key={o.id} className="simulation-aim-origin-item">
              <input
                className="simulation-aim-name-input"
                value={o.name}
                onChange={(e) => onRenameNamedOrigin(o.id, e.target.value)}
                aria-label={`Name for origin ${o.id}`}
              />
              <button
                type="button"
                className="simulation-aim-remove"
                onClick={() => onRemoveNamedOrigin(o.id)}
                aria-label={`Remove origin ${o.name}`}
              >
                ×
              </button>
            </li>
          ))}
        </ul>

        <div className="simulation-aim-place-row">
          <button
            type="button"
            className={placeFieldMode === 'drop' ? 'simulation-place-active' : ''}
            onClick={() => setPlaceFieldMode((m) => (m === 'drop' ? 'none' : 'drop'))}
          >
            {placeFieldMode === 'drop' ? 'Place aim point (on)' : 'Place aim point'}
          </button>
          <button
            type="button"
            className={placeFieldMode === 'origin' ? 'simulation-place-active' : ''}
            onClick={() => setPlaceFieldMode((m) => (m === 'origin' ? 'none' : 'origin'))}
          >
            {placeFieldMode === 'origin' ? 'Place origin (on)' : 'Place named origin'}
          </button>
          <button
            type="button"
            className={placeFieldMode === 'sample' ? 'simulation-place-active' : ''}
            onClick={() => setPlaceFieldMode((m) => (m === 'sample' ? 'none' : 'sample'))}
            title="Click the field to add a KNN map shot at that pose (WPIBlue). Opens inspect so you can set Aim at and save."
          >
            {placeFieldMode === 'sample' ? 'Place shot sample (on)' : 'Place shot sample'}
          </button>
        </div>

        {aimTargetKind === 'drop' && dropTargets.length > 0 && (
          <>
            <h4 className="simulation-aim-sub">Aim points</h4>
            <ul className="simulation-aim-drop-list">
              {dropTargets.map((t) => (
                <li key={t.id}>
                  <input
                    className="simulation-aim-name-input simulation-aim-drop-name"
                    value={t.label}
                    onChange={(e) => onUpdateDropTarget(t.id, { label: e.target.value })}
                    aria-label="Aim point name"
                  />
                  <label className="simulation-aim-select-label">
                    Align from
                    <select
                      value={t.alignmentOriginId}
                      onChange={(e) =>
                        onUpdateDropTarget(t.id, { alignmentOriginId: e.target.value })
                      }
                    >
                      <option value={SIM_HUB_ORIGIN_ID}>Hub</option>
                      {namedOrigins.map((o) => (
                        <option key={o.id} value={o.id}>
                          {o.name}
                        </option>
                      ))}
                    </select>
                  </label>
                  <div className="simulation-aim-drop-actions">
                    <button
                      type="button"
                      className={selectedDropTargetId === t.id ? 'active' : ''}
                      onClick={() => onSelectDropForAim(t.id)}
                    >
                      Aim here
                    </button>
                    <button
                      type="button"
                      className="simulation-aim-remove"
                      onClick={() => onRemoveDropTarget(t.id)}
                      aria-label={`Remove ${t.label}`}
                    >
                      ×
                    </button>
                  </div>
                </li>
              ))}
            </ul>
          </>
        )}
      </aside>

      {inspectedOk && inspected && (
        <aside
          className="simulation-inspect-panel"
          aria-label="Selected KNN shot and aim"
          {...inspectPanelDrag.panelProps}
        >
          <h3
            className="simulation-panel-drag-handle"
            title="Drag panel"
            {...inspectPanelDrag.handleProps}
          >
            Shot #{clickedShotIndex} → {effectiveAimFieldTarget.label}
          </h3>

          {dropTargets.length > 0 ? (
            <div className="simulation-inspect-aim-picker">
              <span className="simulation-inspect-aim-picker-label">Aim at</span>
              <div className="simulation-inspect-aim-chips" role="group" aria-label="Aim target">
                <button
                  type="button"
                  className={aimTargetKind === 'hub' ? 'active' : ''}
                  onClick={() => onAimTargetKindChange('hub')}
                >
                  Hub
                </button>
                {dropTargets.map((t) => (
                  <button
                    key={t.id}
                    type="button"
                    className={
                      aimTargetKind === 'drop' && selectedDropTargetId === t.id ? 'active' : ''
                    }
                    onClick={() => {
                      onSelectDropForAim(t.id);
                      onAimTargetKindChange('drop');
                    }}
                  >
                    {t.label}
                  </button>
                ))}
              </div>
              {shotUsesPersistedFieldAim && (
                <p className="simulation-inspect-aim-picker-saved-note">
                  Gold line uses the <strong>saved</strong> target. Adjust chips, then{' '}
                  <strong>Save shoot target…</strong> to update this map point.
                </p>
              )}
            </div>
          ) : (
            <p className="simulation-inspect-note simulation-inspect-note-compact">
              Add an <strong>aim point</strong> in the Aim target panel (or use the button below) to compare
              against a field target.
            </p>
          )}

          <div className="simulation-inspect-persist-target">
            <button
              type="button"
              className="simulation-inspect-persist-target-btn"
              onClick={persistShootTargetFromPicker}
              title="Writes the current Aim at choice into this map point for local storage and JSON export"
            >
              Save shoot target for map point #{clickedShotIndex}
            </button>
            <p className="simulation-inspect-persist-hint">
              Default is hub. After choosing <strong>Hub</strong> or an aim point above, save here, then use{' '}
              <strong>Export JSON</strong> on KNN Grid. Robot code ignores <code>shootTarget</code>.
            </p>
          </div>

          <div className="simulation-inspect-place-sample">
            <button
              type="button"
              className="simulation-inspect-place-sample-btn"
              onClick={putAimPointOnSamplePose}
              title={
                aimTargetKind === 'drop' && selectedDropTargetId
                  ? 'Move the selected aim point to this map shot’s field position'
                  : 'Create an aim point at this map shot’s field position and select Aim point mode'
              }
            >
              {aimTargetKind === 'drop' && selectedDropTargetId
                ? 'Move aim point to this sample'
                : 'Aim point at this sample'}
            </button>
          </div>

          {showHeadingAnalysis ? (
            <>
              <p className="simulation-inspect-note">
                {shotUsesPersistedFieldAim ? (
                  <>
                    This point uses a <strong>saved field target</strong>. Heading is sample pose toward
                    that target. Change <strong>Aim at</strong> and <strong>Save shoot target…</strong> to
                    replace it, or pick <strong>Hub</strong> and save to clear the field target.
                  </>
                ) : aimTargetKind === 'hub' ? (
                  <>
                    Hub optimal heading uses <code>DriveConstants.rotationToFaceHubFromShotMap</code>{' '}
                    (same linear offset constants as the robot).
                  </>
                ) : (
                  <>
                    Optimal heading matches the field ray <strong>{alignmentRayLabel}</strong> (chassis +X
                    along that direction).
                  </>
                )}
              </p>
              <dl className="simulation-inspect-grid">
                <dt>Alignment ray</dt>
                <dd>{alignmentRayLabel}</dd>
                <dt>Sample pose</dt>
                <dd>
                  ({aimFromX.toFixed(2)}, {aimFromY.toFixed(2)}) m
                </dd>
                <dt>Distance to target</dt>
                <dd>{aimDistanceM !== null ? `${aimDistanceM.toFixed(2)} m` : '—'}</dd>
                <dt>Optimal heading</dt>
                <dd
                  title={
                    aimTargetKind === 'hub'
                      ? 'DriveConstants.rotationToFaceHubFromShotMap at sample pose'
                      : 'Chassis +X along alignment ray toward selected aim'
                  }
                >
                  {optimalAimHeadingDeg!.toFixed(1)}°
                </dd>
                <dt>Logged heading</dt>
                <dd>{loggedHeadingDeg.toFixed(1)}° (arrow on field)</dd>
                <dt>Turn to optimal</dt>
                <dd
                  title="Shortest rotation from logged heading to aim heading; negative = turn the other way"
                >
                  {aimHeadingDeltaDeg! >= 0 ? '+' : ''}
                  {aimHeadingDeltaDeg!.toFixed(1)}°
                </dd>
                <dt>Map shooter</dt>
                <dd>
                  {Math.round(inspected.shooterRpm ?? 0)} rpm · hood{' '}
                  {inspected.hoodDeg?.toFixed(0) ?? 0}°
                </dd>
              </dl>
            </>
          ) : (
            <p className="simulation-inspect-note simulation-inspect-note-wait">
              Pick <strong>Hub</strong> or an aim point above to see distance, optimal heading, and turn
              delta for this shot.
            </p>
          )}

          <div className="simulation-inspect-actions">
            <button type="button" onClick={() => setClickedShotIndex(null)}>
              Clear selection
            </button>
            <button
              type="button"
              disabled={!mockMode || !showHeadingAnalysis}
              title={
                !mockMode
                  ? 'Switch to Mock mode'
                  : !showHeadingAnalysis
                    ? 'Choose an aim target first'
                    : 'Set mock pose heading and shooter to this shot / aim'
              }
              onClick={() =>
                onApplyMockShotFromSim(
                  optimalAimHeadingDeg!,
                  inspected.shooterRpm ?? 0,
                  inspected.hoodDeg ?? 0
                )
              }
            >
              Apply to mock robot
            </button>
          </div>
        </aside>
      )}
    </section>
  );
}
