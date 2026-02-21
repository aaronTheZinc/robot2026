import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import {
  createTurretMesh,
  FLYWHEEL_R,
  GEAR_RATIO,
} from '../lib/turretModel';
import type { TurretMeshRefs } from '../lib/turretModel';

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
};

const ROBOT_HEIGHT_M = 0.18;
const VECTOR_OFFSET = 2.2;

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
}: SimulationViewProps) {
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
  const animationRef = useRef<number | null>(null);

  const sphericalRef = useRef({ theta: 0.6, phi: 1.0, radius: 14 });
  const panOffsetRef = useRef(new THREE.Vector3(0, 0, 0));
  const isDraggingRef = useRef(false);
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
      isDraggingRef.current = true;
      lastMouseRef.current = { x: e.clientX, y: e.clientY };
    };
    const onMouseUp = () => {
      isDraggingRef.current = false;
    };
    const onMouseMove = (e: MouseEvent) => {
      if (!isDraggingRef.current) return;
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
    window.addEventListener('mouseup', onMouseUp);
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
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      container.removeEventListener('mousedown', onMouseDown);
      container.removeEventListener('wheel', onWheel);
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
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

  useEffect(() => {
    const robotGroup = robotRef.current;
    const vectorGroup = vectorArrowsGroupRef.current;
    if (!robotGroup) return;
    const clampedX = THREE.MathUtils.clamp(poseX, 0, fieldLengthM);
    const clampedY = THREE.MathUtils.clamp(poseY, 0, fieldWidthM);
    const worldX = clampedX - fieldLengthM / 2;
    const worldZ = clampedY - fieldWidthM / 2;
    robotGroup.position.x = worldX;
    robotGroup.position.z = worldZ;
    robotGroup.position.y = 0;
    robotGroup.rotation.y = THREE.MathUtils.degToRad(headingDeg);
    if (vectorGroup) {
      const hRad = THREE.MathUtils.degToRad(headingDeg);
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

  return (
    <section className="simulation-view">
      <div className="simulation-canvas" ref={containerRef} />
      <p className="simulation-nav-hint" aria-hidden="true">
        WASD — pan · Q/E — up/down · Mouse — orbit · Wheel — zoom
      </p>
      <div className="simulation-vectors-panel" aria-hidden="true">
        <div className="simulation-vectors-title">Vectors</div>
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
      </div>
      <div className="simulation-overlay" aria-hidden="true">
        <span className="simulation-badge">Simulation</span>
        <span className="simulation-stats">
          Pose ({poseX.toFixed(1)}, {poseY.toFixed(1)}) · Heading {headingDeg.toFixed(0)}° · Turret{' '}
          {turretAngleDeg.toFixed(0)}° · Hood {hoodPitchDeg.toFixed(0)}° · {velocityMps.toFixed(1)}{' '}
          m/s
        </span>
      </div>
    </section>
  );
}
