import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import {
  createTurretMesh,
  FLYWHEEL_R,
  GEAR_RATIO,
} from '../lib/turretModel';
import type { TurretMeshRefs } from '../lib/turretModel';
import type { KnnPoint, KnnShootTarget } from '../lib/knnInference';
import {
  distanceBetweenFieldPointsM,
  HUB_FIELD_X_M,
  HUB_FIELD_Y_M,
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
  rayNdcToFieldXY,
} from '../lib/simFieldToThree';
import { useDraggablePanel } from '../lib/useDraggablePanel';

type SimulationViewProps = {
  fieldLengthM: number;
  fieldWidthM: number;
  robotLengthM: number;
  robotWidthM: number;
  poseX: number;
  poseY: number;
  headingDeg: number;
  turretAngleDeg: number;
  hoodPitchDeg: number;
  velocityMps: number;
  /** Logged / deploy KNN shot samples (same indices as robot `knn_map.json` when lists match). */
  shotMapPoints: KnnPoint[];
  /** `/KNN/selectedIndex` from robot, or -1 when unknown */
  knnSelectedIndex: number;
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
  onRemoveNamedOrigin: (id: string) => void;
  onRenameNamedOrigin: (id: string, name: string) => void;
  onUpdateDropTarget: (
    id: string,
    patch: Partial<Pick<SimDropTarget, 'label' | 'alignmentOriginId' | 'x' | 'y'>>
  ) => void;
  onRemoveDropTarget: (id: string) => void;
  onSelectDropForAim: (id: string) => void;
  mockMode: boolean;
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

function isPlaceholderKnnPoint(p: KnnPoint): boolean {
  return (
    Math.abs(p.x) < 1e-6 &&
    Math.abs(p.y) < 1e-6 &&
    Math.abs(p.shooterRpm ?? 0) < 1
  );
}

function disposeObject3DSubtree(root: THREE.Object3D) {
  root.traverse((obj) => {
    if (obj instanceof THREE.Mesh) {
      obj.geometry?.dispose();
      const mat = obj.material;
      if (Array.isArray(mat)) {
        mat.forEach((m) => m.dispose());
      } else {
        mat?.dispose();
      }
    }
  });
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
  poseX,
  poseY,
  headingDeg,
  turretAngleDeg,
  hoodPitchDeg,
  velocityMps,
  shotMapPoints,
  knnSelectedIndex,
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
  onRemoveNamedOrigin,
  onRenameNamedOrigin,
  onUpdateDropTarget,
  onRemoveDropTarget,
  onSelectDropForAim,
  mockMode,
  onApplyMockShotFromSim,
  onSaveKnnShootTarget,
}: SimulationViewProps) {
  type PlaceFieldMode = 'none' | 'drop' | 'origin';
  const [clickedShotIndex, setClickedShotIndex] = useState<number | null>(null);
  const [placeFieldMode, setPlaceFieldMode] = useState<PlaceFieldMode>('none');
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
  const onSelectDropForAimRef = useRef(onSelectDropForAim);
  onSelectDropForAimRef.current = onSelectDropForAim;
  const setPlaceFieldModeRef = useRef(setPlaceFieldMode);
  setPlaceFieldModeRef.current = setPlaceFieldMode;

  const simulationSectionRef = useRef<HTMLElement | null>(null);
  const aimPanelDrag = useDraggablePanel(simulationSectionRef);
  const inspectPanelDrag = useDraggablePanel(simulationSectionRef);
  const vectorsPanelDrag = useDraggablePanel(simulationSectionRef);

  const containerRef = useRef<HTMLDivElement | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
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
  const hubVfxRef = useRef<THREE.Group | null>(null);
  const animationRef = useRef<number | null>(null);

  const sphericalRef = useRef({ theta: 0.6, phi: 1.0, radius: 14 });
  const panOffsetRef = useRef(new THREE.Vector3(0, 0, 0));
  const isDraggingRef = useRef(false);
  const pointerDownOnCanvasRef = useRef(false);
  const dragStartRef = useRef({ x: 0, y: 0 });
  const hasDraggedRef = useRef(false);
  const lastMouseRef = useRef({ x: 0, y: 0 });
  const keysRef = useRef({ w: false, a: false, s: false, d: false, q: false, e: false });
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
    scene.add(fieldMesh);

    const borderGeometry = new THREE.EdgesGeometry(fieldGeometry);
    const borderMaterial = new THREE.LineBasicMaterial({ color: 0x2f3a52 });
    const border = new THREE.LineSegments(borderGeometry, borderMaterial);
    border.rotation.x = -Math.PI / 2;
    scene.add(border);

    const gridSize = Math.max(fieldLengthM, fieldWidthM);
    const grid = new THREE.GridHelper(gridSize, 24, 0x1e3a5f, 0x0f1e30);
    grid.position.y = 0.002;
    scene.add(grid);

    const { wx: hubCx, wz: hubCz } = fieldPoseToWorldXZ(
      HUB_FIELD_X_M,
      HUB_FIELD_Y_M,
      fieldLengthM,
      fieldWidthM
    );

    const hubRoot = new THREE.Group();
    hubRoot.position.set(hubCx, 0, hubCz);
    scene.add(hubRoot);
    hubVfxRef.current = hubRoot;

    /** Hexagonal open funnel: flat panels, wide hex mouth up, smaller hex throat down. */
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

    const robotGroup = new THREE.Group();
    scene.add(robotGroup);
    robotRef.current = robotGroup;

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
    scene.add(vectorArrowsGroup);
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
    scene.add(shotMarkersGroup);
    shotMarkersGroupRef.current = shotMarkersGroup;

    const dropMarkersGroup = new THREE.Group();
    scene.add(dropMarkersGroup);
    dropMarkersGroupRef.current = dropMarkersGroup;

    const originMarkersGroup = new THREE.Group();
    scene.add(originMarkersGroup);
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
      isDraggingRef.current = true;
      hasDraggedRef.current = false;
      dragStartRef.current = { x: e.clientX, y: e.clientY };
      lastMouseRef.current = { x: e.clientX, y: e.clientY };
    };
    const onWindowMouseUp = (e: MouseEvent) => {
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

          const pf = placeFieldModeRef.current;
          if ((pf === 'drop' || pf === 'origin') && cam) {
            const fieldHit = rayNdcToFieldXY(ndcX, ndcY, cam, fL, fW);
            if (fieldHit) {
              if (pf === 'drop') {
                onAddDropTargetRef.current(fieldHit.x, fieldHit.y);
              } else {
                onAddNamedOriginRef.current(fieldHit.x, fieldHit.y);
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
    const onKeyDown = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (k === 'w') keysRef.current.w = true;
      if (k === 's') keysRef.current.s = true;
      if (k === 'a') keysRef.current.a = true;
      if (k === 'd') keysRef.current.d = true;
      if (k === 'q') keysRef.current.q = true;
      if (k === 'e') keysRef.current.e = true;
    };
    const onKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (k === 'w') keysRef.current.w = false;
      if (k === 's') keysRef.current.s = false;
      if (k === 'a') keysRef.current.a = false;
      if (k === 'd') keysRef.current.d = false;
      if (k === 'q') keysRef.current.q = false;
      if (k === 'e') keysRef.current.e = false;
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
      if (keys.w || keys.s || keys.a || keys.d || keys.q || keys.e) {
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
      const hubRoot = hubVfxRef.current;
      if (hubRoot) {
        const t = clockRef.current.elapsedTime;
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
      const hubRootCleanup = hubVfxRef.current;
      if (hubRootCleanup) {
        disposeObject3DSubtree(hubRootCleanup);
        scene.remove(hubRootCleanup);
      }
      hubVfxRef.current = null;
      renderer.dispose();
      fieldGeometry.dispose();
      fieldMaterial.dispose();
      borderGeometry.dispose();
      borderMaterial.dispose();
      robotGeometry.dispose();
      robotMaterial.dispose();
      scene.clear();
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
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

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
        worldZ + VECTOR_OFFSET * Math.sin(hRad)
      );
      vectorGroup.rotation.y = hRad;
    }
  }, [fieldLengthM, fieldWidthM, headingDeg, poseX, poseY]);

  useEffect(() => {
    velocityMpsRef.current = velocityMps;
  }, [velocityMps]);

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

    for (let index = 0; index < shotMapPoints.length; index++) {
      const p = shotMapPoints[index];
      if (isPlaceholderKnnPoint(p)) {
        continue;
      }
      const { dx, dy, lift } = offsets[index] ?? { dx: 0, dy: 0, lift: 0 };
      const px = THREE.MathUtils.clamp(p.x + dx, 0, fieldLengthM);
      const py = THREE.MathUtils.clamp(p.y + dy, 0, fieldWidthM);
      const { wx, wz } = fieldPoseToWorldXZ(px, py, fieldLengthM, fieldWidthM);

      const markerRoot = new THREE.Group();
      markerRoot.userData.mapIndex = index;
      markerRoot.position.set(wx, 0, wz);

      const isClick = clickedShotIndex === index;
      const isNt = knnSelectedIndex >= 0 && knnSelectedIndex === index;
      const sphereMat = isClick ? matInspect : isNt ? matSelected : matDefault;
      const arrowColor = isClick ? 0xfde68a : isNt ? 0xf9a8d4 : 0x6ee7b7;

      const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.11, 10, 8), sphereMat);
      sphere.position.y = 0.12 + lift;
      sphere.castShadow = true;
      markerRoot.add(sphere);

      const dir = fieldHeadingDegToSceneForwardXZ(p.headingDeg ?? 0);
      const arrow = new THREE.ArrowHelper(
        dir,
        new THREE.Vector3(0, 0.11 + lift, 0),
        0.52,
        arrowColor,
        0.11,
        0.07
      );
      markerRoot.add(arrow);

      group.add(markerRoot);
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
    knnSelectedIndex,
    shotMapPoints,
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

    for (const t of dropTargets) {
      const { wx, wz } = fieldPoseToWorldXZ(t.x, t.y, fieldLengthM, fieldWidthM);
      const markerRoot = new THREE.Group();
      markerRoot.userData.dropTargetId = t.id;
      markerRoot.position.set(wx, 0, wz);

      const isSel = selectedDropTargetId === t.id;
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
        opacity: 0.85,
        side: THREE.DoubleSide,
      });
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.08, 0.2, 24), ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.02;
      markerRoot.add(ring);

      group.add(markerRoot);
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

    for (const o of namedOrigins) {
      const { wx, wz } = fieldPoseToWorldXZ(o.x, o.y, fieldLengthM, fieldWidthM);
      const markerRoot = new THREE.Group();
      markerRoot.userData.namedOriginId = o.id;
      markerRoot.position.set(wx, 0, wz);

      const pillarMat = new THREE.MeshStandardMaterial({
        color: 0xc084fc,
        emissive: 0x6b21a8,
        emissiveIntensity: 0.2,
        metalness: 0.25,
        roughness: 0.5,
      });
      const pillar = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.32, 10), pillarMat);
      pillar.position.y = 0.18;
      pillar.castShadow = true;
      markerRoot.add(pillar);

      const capMat = new THREE.MeshStandardMaterial({
        color: 0xe9d5ff,
        metalness: 0.35,
        roughness: 0.4,
      });
      const cap = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 10), capMat);
      cap.position.y = 0.38;
      cap.castShadow = true;
      markerRoot.add(cap);

      group.add(markerRoot);
    }

    return () => {
      while (group.children.length > 0) {
        const ch = group.children[0];
        group.remove(ch);
        disposeSubtree(ch);
      }
    };
  }, [namedOrigins, fieldLengthM, fieldWidthM]);

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
    scene.add(grp);
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
          ? optimalHeadingTowardFieldPointDeg(
              aimFromX,
              aimFromY,
              HUB_FIELD_X_M,
              HUB_FIELD_Y_M
            )
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
        WASD — pan · Q/E — up/down · Mouse — orbit · Wheel — zoom · Hex funnel — hub · Click KNN shot —
        aim line · Violet — named alignment origins · Cyan — aim points (drops) · Teal — map · Gold —
        selected shot · Pink — NT nearest
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
            {knnSelectedIndex >= 0 ? ` · #${knnSelectedIndex}` : ''}
          </span>
        </div>
      </div>
      <div className="simulation-overlay" aria-hidden="true">
        <span className="simulation-badge">Simulation</span>
        <span className="simulation-stats">
          Pose ({poseX.toFixed(1)}, {poseY.toFixed(1)}) · Heading {headingDeg.toFixed(0)}° · Turret{' '}
          {turretAngleDeg.toFixed(0)}° · Hood {hoodPitchDeg.toFixed(0)}° · Flywheel {velocityMps.toFixed(1)}{' '}
          m/s · Shooter {Math.round(shooterRpm)}/{Math.round(shooterRpmSetpoint)} rpm · KNN{' '}
          {knnSelectedIndex >= 0 ? `#${knnSelectedIndex}` : '—'} · {shotPlacesCount} map shots
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
                    Bearing from the <strong>sample pose</strong> toward <strong>Hub</strong> (same
                    convention as <code>DriveConstants.rotationToFaceHub</code>).
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
                <dd title="Chassis +X toward selected aim point (same convention as hub helper)">
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
