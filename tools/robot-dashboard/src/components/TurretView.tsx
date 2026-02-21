import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import {
  createTurretMesh,
  FLYWHEEL_R,
  GEAR_RATIO,
  makeBox,
  PINION_TEETH,
  RACK_TEETH,
} from '../lib/turretModel';

type TurretViewProps = {
  turretAngleDeg: number;
  hoodPitchDeg: number;
  velocityMps: number;
};

const CHASSIS_L = 0.5;
const CHASSIS_H = 0.18;
const CHASSIS_W = 0.5;

export default function TurretView({
  turretAngleDeg,
  hoodPitchDeg,
  velocityMps,
}: TurretViewProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const turretRef = useRef<THREE.Group | null>(null);
  const hoodPivotRef = useRef<THREE.Group | null>(null);
  const pinionMeshRef = useRef<THREE.Mesh | null>(null);
  const flywheelGroupRef = useRef<THREE.Group | null>(null);
  const matWheelRef = useRef<THREE.MeshStandardMaterial | null>(null);
  const speedLightRef = useRef<THREE.PointLight | null>(null);
  const animationRef = useRef<number | null>(null);
  const clockRef = useRef<THREE.Clock>(new THREE.Clock());
  const velocityMpsRef = useRef(velocityMps);

  const sphericalRef = useRef({ theta: 0.4, phi: 1.0, radius: 2.8 });
  const panOffsetRef = useRef(new THREE.Vector3(0, 0.3, 0));
  const isDraggingRef = useRef(false);
  const lastMouseRef = useRef({ x: 0, y: 0 });
  const keysRef = useRef({ w: false, a: false, s: false, d: false, q: false, e: false });

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0c12);
    scene.fog = new THREE.FogExp2(0x0a0c12, 0.025);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.6;
    container.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      50,
      container.clientWidth / container.clientHeight,
      0.01,
      100
    );
    camera.position.set(1.2, 1.0, 2.2);

    const ambient = new THREE.AmbientLight(0x446688, 1.0);
    scene.add(ambient);

    const keyLightPos = new THREE.Vector3(2.5, 2.5, 2.5);
    const key = new THREE.DirectionalLight(0xaaccff, 3.5);
    key.position.copy(keyLightPos);
    key.target.position.set(0, 0.25, 0);
    scene.add(key.target);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.5;
    key.shadow.camera.far = 15;
    key.shadow.camera.left = -2;
    key.shadow.camera.right = 2;
    key.shadow.camera.top = 2;
    key.shadow.camera.bottom = -2;
    scene.add(key);

    const mainPointLight = new THREE.PointLight(0xffeedd, 2.5, 12, 1.5);
    mainPointLight.position.copy(keyLightPos);
    scene.add(mainPointLight);

    const lightBulbGeo = new THREE.SphereGeometry(0.12, 20, 20);
    const lightBulbMat = new THREE.MeshStandardMaterial({
      color: 0xffdd88,
      emissive: 0xffaa44,
      emissiveIntensity: 1.2,
      metalness: 0.1,
      roughness: 0.4,
    });
    const lightBulb = new THREE.Mesh(lightBulbGeo, lightBulbMat);
    lightBulb.position.copy(keyLightPos);
    scene.add(lightBulb);

    const fill = new THREE.DirectionalLight(0x6688aa, 1.0);
    fill.position.set(-2, 1.5, 2);
    fill.target.position.set(0, 0.2, 0);
    scene.add(fill.target);
    scene.add(fill);
    const rim = new THREE.DirectionalLight(0x4488cc, 0.8);
    rim.position.set(0, 1, -2);
    rim.target.position.set(0, 0.2, 0);
    scene.add(rim.target);
    scene.add(rim);

    const grid = new THREE.GridHelper(6, 30, 0x1e3a5f, 0x0f1e30);
    grid.position.y = -0.01;
    scene.add(grid);

    const matChassis = new THREE.MeshStandardMaterial({
      color: 0x3b82f6,
      metalness: 0.25,
      roughness: 0.4,
    });
    const chassisGroup = new THREE.Group();
    scene.add(chassisGroup);
    const chassisBox = makeBox(CHASSIS_L, CHASSIS_H, CHASSIS_W, matChassis);
    chassisBox.position.y = CHASSIS_H / 2;
    chassisBox.castShadow = true;
    chassisBox.receiveShadow = true;
    chassisGroup.add(chassisBox);

    const refs = createTurretMesh(chassisGroup, CHASSIS_H);
    turretRef.current = refs.turretRef;
    hoodPivotRef.current = refs.hoodPivotRef;
    flywheelGroupRef.current = refs.flywheelGroupRef;
    pinionMeshRef.current = refs.pinionMeshRef;
    matWheelRef.current = refs.matWheelRef;
    speedLightRef.current = refs.speedLightRef;
    const turretRing = refs.turretRingRef;

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
    updateCamera();

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
        0.1,
        Math.min(Math.PI - 0.1, sphericalRef.current.phi + dy)
      );
      updateCamera();
    };
    const onWheel = (e: WheelEvent) => {
      sphericalRef.current.radius = Math.max(
        0.5,
        Math.min(8, sphericalRef.current.radius + e.deltaY * 0.003)
      );
      updateCamera();
    };

    const PAN_SPEED = 1.2;
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
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);

    renderer.domElement.addEventListener('mousedown', onMouseDown);
    renderer.domElement.addEventListener('mouseup', onMouseUp);
    renderer.domElement.addEventListener('mousemove', onMouseMove);
    renderer.domElement.addEventListener('wheel', onWheel, { passive: true });

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
      const t = clockRef.current.getElapsedTime();
      const v = velocityMpsRef.current;

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

      const flywheelGroup = flywheelGroupRef.current;
      if (flywheelGroup) {
        const omega = v / FLYWHEEL_R;
        flywheelRotation += omega * dt;
        flywheelGroup.rotation.x = flywheelRotation;
      }

      const speedLight = speedLightRef.current;
      const matWheel = matWheelRef.current;
      if (speedLight && matWheel) {
        const speedNorm = Math.min(1, Math.max(0, v / 30));
        speedLight.intensity = speedNorm * 2.5;
        if (!matWheel.emissive) matWheel.emissive = new THREE.Color();
        matWheel.emissive.setScalar(speedNorm * 0.4);
      }

      if (turretRing.material instanceof THREE.MeshStandardMaterial) {
        turretRing.material.emissiveIntensity = 0.2 + 0.1 * Math.sin(t * 2);
      }

      renderer.render(scene, camera);
      animationRef.current = requestAnimationFrame(animate);
    };
    animate();

    sceneRef.current = scene;
    cameraRef.current = camera;
    rendererRef.current = renderer;

    return () => {
      if (animationRef.current) cancelAnimationFrame(animationRef.current);
      window.removeEventListener('resize', handleResize);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      renderer.domElement.removeEventListener('mousedown', onMouseDown);
      renderer.domElement.removeEventListener('mouseup', onMouseUp);
      renderer.domElement.removeEventListener('mousemove', onMouseMove);
      renderer.domElement.removeEventListener('wheel', onWheel);
      renderer.dispose();
      scene.clear();
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, []);

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
    velocityMpsRef.current = velocityMps;
  }, [velocityMps]);

  useEffect(() => {
    const pinionMesh = pinionMeshRef.current;
    if (!pinionMesh) return;
    const pinionAngleDeg = hoodPitchDeg * GEAR_RATIO;
    pinionMesh.rotation.x =
      Math.PI / 2 + (pinionAngleDeg * Math.PI) / 180;
  }, [hoodPitchDeg]);

  const pinionAngleDeg = hoodPitchDeg * GEAR_RATIO;

  return (
    <div className="subsystem-view-inner">
      <div className="field-canvas turret-canvas" ref={containerRef} />
      <p className="turret-nav-hint">WASD — pan · Q/E — up/down · Mouse — orbit · Wheel — zoom</p>
      <div className="turret-metrics">
        <div className="metric-row">
          <span className="metric-label">Turret</span>
          <span className="metric-value">{turretAngleDeg.toFixed(1)}°</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Hood pitch</span>
          <span className="metric-value">{hoodPitchDeg.toFixed(1)}°</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Velocity</span>
          <span className="metric-value">{velocityMps.toFixed(1)} m/s</span>
        </div>
        <div className="turret-gear-info">
          <div className="metric-row">
            <span className="metric-label">Gear ratio</span>
            <span className="metric-value">{RACK_TEETH}:{PINION_TEETH}</span>
          </div>
          <div className="metric-row">
            <span className="metric-label">Pinion rotation</span>
            <span className="metric-value">{pinionAngleDeg.toFixed(1)}°</span>
          </div>
        </div>
      </div>
    </div>
  );
}
