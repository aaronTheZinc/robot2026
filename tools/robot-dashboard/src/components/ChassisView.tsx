import { useEffect, useRef } from 'react';
import * as THREE from 'three';

const ROBOT_HEIGHT_M = 0.18;

type ChassisViewProps = {
  fieldLengthM: number;
  fieldWidthM: number;
  robotLengthM: number;
  robotWidthM: number;
  poseX: number;
  poseY: number;
  headingDeg: number;
  speedMps: number;
  fieldRelative: boolean;
  targets: { x: number; y: number }[];
  loggedPoints?: { x: number; y: number }[];
};

export default function ChassisView({
  fieldLengthM,
  fieldWidthM,
  robotLengthM,
  robotWidthM,
  poseX,
  poseY,
  headingDeg,
  speedMps,
  fieldRelative,
  targets,
  loggedPoints,
}: ChassisViewProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Mesh | null>(null);
  const targetsRef = useRef<THREE.Group | null>(null);
  const loggedPointsRef = useRef<THREE.Group | null>(null);
  const animationRef = useRef<number | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#0b0f1a');

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      50,
      container.clientWidth / container.clientHeight,
      0.1,
      100
    );
    camera.position.set(0, 12, 14);
    camera.lookAt(0, 0, 0);

    const ambient = new THREE.AmbientLight(0xffffff, 0.7);
    const directional = new THREE.DirectionalLight(0xffffff, 0.6);
    directional.position.set(10, 12, 6);
    scene.add(ambient, directional);

    const fieldGeometry = new THREE.PlaneGeometry(fieldLengthM, fieldWidthM);
    const fieldMaterial = new THREE.MeshStandardMaterial({
      color: '#0f172a',
      metalness: 0.1,
      roughness: 0.8,
    });
    const fieldMesh = new THREE.Mesh(fieldGeometry, fieldMaterial);
    fieldMesh.rotation.x = -Math.PI / 2;
    scene.add(fieldMesh);

    const borderGeometry = new THREE.EdgesGeometry(fieldGeometry);
    const borderMaterial = new THREE.LineBasicMaterial({ color: '#4f5d7a' });
    const border = new THREE.LineSegments(borderGeometry, borderMaterial);
    border.rotation.x = -Math.PI / 2;
    scene.add(border);

    const gridSize = Math.max(fieldLengthM, fieldWidthM);
    const grid = new THREE.GridHelper(gridSize, 18, '#2f3a52', '#1f293b');
    scene.add(grid);

    const robotGeometry = new THREE.BoxGeometry(
      robotLengthM,
      ROBOT_HEIGHT_M,
      robotWidthM
    );
    const robotMaterial = new THREE.MeshStandardMaterial({
      color: '#3b82f6',
      metalness: 0.2,
      roughness: 0.4,
    });
    const robot = new THREE.Mesh(robotGeometry, robotMaterial);
    robot.position.y = ROBOT_HEIGHT_M / 2;
    scene.add(robot);

    const headingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, ROBOT_HEIGHT_M / 2 + 0.01, 0),
      Math.max(robotLengthM, robotWidthM) * 0.6,
      0xffd166
    );
    robot.add(headingArrow);

    const targetsGroup = new THREE.Group();
    scene.add(targetsGroup);

    const loggedPointsGroup = new THREE.Group();
    scene.add(loggedPointsGroup);

    const handleResize = () => {
      const width = container.clientWidth;
      const height = container.clientHeight;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };

    const animate = () => {
      renderer.render(scene, camera);
      animationRef.current = requestAnimationFrame(animate);
    };
    animate();

    window.addEventListener('resize', handleResize);

    sceneRef.current = scene;
    cameraRef.current = camera;
    robotRef.current = robot;
    targetsRef.current = targetsGroup;
    loggedPointsRef.current = loggedPointsGroup;

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
      window.removeEventListener('resize', handleResize);
      renderer.dispose();
      fieldGeometry.dispose();
      fieldMaterial.dispose();
      borderGeometry.dispose();
      borderMaterial.dispose();
      robotGeometry.dispose();
      robotMaterial.dispose();
      scene.clear();
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;
    const clampedX = THREE.MathUtils.clamp(poseX, 0, fieldLengthM);
    const clampedY = THREE.MathUtils.clamp(poseY, 0, fieldWidthM);
    robot.position.x = clampedX - fieldLengthM / 2;
    robot.position.z = clampedY - fieldWidthM / 2;
    robot.rotation.y = THREE.MathUtils.degToRad(headingDeg);
  }, [fieldLengthM, fieldWidthM, headingDeg, poseX, poseY]);

  useEffect(() => {
    const group = targetsRef.current;
    if (!group) return;

    while (group.children.length > 0) {
      const child = group.children[0];
      group.remove(child);
    }

    const markerGeometry = new THREE.ConeGeometry(0.15, 0.4, 8);
    const markerMaterial = new THREE.MeshStandardMaterial({
      color: '#ef4444',
      metalness: 0.2,
      roughness: 0.6,
    });

    for (const t of targets) {
      const marker = new THREE.Mesh(markerGeometry, markerMaterial);
      marker.position.x = t.x - fieldLengthM / 2;
      marker.position.y = 0.2;
      marker.position.z = t.y - fieldWidthM / 2;
      marker.rotation.x = Math.PI; // point up
      group.add(marker);
    }

    return () => {
      while (group.children.length > 0) {
        group.remove(group.children[0]);
      }
      markerGeometry.dispose();
      markerMaterial.dispose();
    };
  }, [fieldLengthM, fieldWidthM, targets]);

  useEffect(() => {
    const group = loggedPointsRef.current;
    if (!group) return;

    while (group.children.length > 0) {
      group.remove(group.children[0]);
    }

    const points = loggedPoints ?? [];
    if (points.length === 0) return;

    const sphereGeometry = new THREE.SphereGeometry(0.12, 8, 6);
    const sphereMaterial = new THREE.MeshStandardMaterial({
      color: 0x22c55e,
      metalness: 0.2,
      roughness: 0.6,
    });

    for (const p of points) {
      const marker = new THREE.Mesh(sphereGeometry, sphereMaterial);
      marker.position.x = p.x - fieldLengthM / 2;
      marker.position.y = 0.12;
      marker.position.z = p.y - fieldWidthM / 2;
      group.add(marker);
    }

    return () => {
      while (group.children.length > 0) {
        group.remove(group.children[0]);
      }
      sphereGeometry.dispose();
      sphereMaterial.dispose();
    };
  }, [fieldLengthM, fieldWidthM, loggedPoints]);

  return (
    <div className="subsystem-view-inner">
      <div className="field-canvas chassis-canvas" ref={containerRef} />
      <div className="chassis-metrics">
        <div className="metric-row">
          <span className="metric-label">Pose X</span>
          <span className="metric-value">{poseX.toFixed(2)} m</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Pose Y</span>
          <span className="metric-value">{poseY.toFixed(2)} m</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Heading</span>
          <span className="metric-value">{headingDeg.toFixed(1)}°</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Speed</span>
          <span className="metric-value">{speedMps.toFixed(1)} m/s</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Frame</span>
          <span className="metric-value">
            {fieldRelative ? 'Field' : 'Robot'}
          </span>
        </div>
        <div className="metric-row">
          <span className="metric-label">Targets</span>
          <span className="metric-value">{targets.length}</span>
        </div>
        <div className="metric-row">
          <span className="metric-label">KNN points</span>
          <span className="metric-value">{(loggedPoints ?? []).length}</span>
        </div>
      </div>
    </div>
  );
}
