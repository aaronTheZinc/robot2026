import { useEffect, useRef } from 'react';
import * as THREE from 'three';

import {
  fieldHeadingDegToSceneYawRad,
  fieldPoseToWorldXZ,
} from '../lib/simFieldToThree';

const ROBOT_HEIGHT_M = 0.18;

/** Shortest signed angle from current heading to hub-facing ideal (degrees). */
function hubHeadingErrorDeg(currentDeg: number, idealDeg: number): number {
  let d = idealDeg - currentDeg;
  d = ((d % 360) + 360) % 360;
  if (d > 180) d -= 360;
  return d;
}

type ChassisViewProps = {
  fieldLengthM: number;
  fieldWidthM: number;
  robotLengthM: number;
  robotWidthM: number;
  poseX: number;
  poseY: number;
  headingDeg: number;
  visionPoseX?: number;
  visionPoseY?: number;
  visionHeadingDeg?: number;
  visionPoseVisible?: boolean;
  /** Hub-facing heading debug (same XY as fused pose; heading = shot-map hub aim from robot). */
  idealShooterPoseX?: number;
  idealShooterPoseY?: number;
  idealShooterHeadingDeg?: number;
  idealShooterPoseVisible?: boolean;
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
  visionPoseX,
  visionPoseY,
  visionHeadingDeg,
  visionPoseVisible = false,
  idealShooterPoseX,
  idealShooterPoseY,
  idealShooterHeadingDeg,
  idealShooterPoseVisible = true,
  speedMps,
  fieldRelative,
  targets,
  loggedPoints,
}: ChassisViewProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Mesh | null>(null);
  const visionRobotRef = useRef<THREE.Mesh | null>(null);
  const idealShooterRobotRef = useRef<THREE.Mesh | null>(null);
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
    /* Blue alliance / field (0,0) on −X; view from outside that wall toward +X (red). */
    camera.position.set(-14, 12, 3.5);
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

    const visionRobotMaterial = new THREE.MeshStandardMaterial({
      color: '#f97316',
      transparent: true,
      opacity: 0.45,
      metalness: 0.1,
      roughness: 0.5,
    });
    const visionRobot = new THREE.Mesh(robotGeometry.clone(), visionRobotMaterial);
    visionRobot.position.y = ROBOT_HEIGHT_M / 2 + 0.01;
    visionRobot.visible = false;
    scene.add(visionRobot);

    const idealShooterMaterial = new THREE.MeshStandardMaterial({
      color: '#14b8a6',
      transparent: true,
      opacity: 0.55,
      metalness: 0.15,
      roughness: 0.45,
    });
    const idealShooterRobot = new THREE.Mesh(robotGeometry.clone(), idealShooterMaterial);
    idealShooterRobot.position.y = ROBOT_HEIGHT_M / 2 + 0.03;
    idealShooterRobot.visible = false;
    scene.add(idealShooterRobot);

    const idealHeadingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, ROBOT_HEIGHT_M / 2 + 0.01, 0),
      Math.max(robotLengthM, robotWidthM) * 0.55,
      0x14b8a6
    );
    idealShooterRobot.add(idealHeadingArrow);

    const headingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, ROBOT_HEIGHT_M / 2 + 0.01, 0),
      Math.max(robotLengthM, robotWidthM) * 0.6,
      0xffd166
    );
    robot.add(headingArrow);

    const visionHeadingArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, ROBOT_HEIGHT_M / 2 + 0.01, 0),
      Math.max(robotLengthM, robotWidthM) * 0.6,
      0xf97316
    );
    visionRobot.add(visionHeadingArrow);

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
    visionRobotRef.current = visionRobot;
    idealShooterRobotRef.current = idealShooterRobot;
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
      visionRobot.geometry.dispose();
      visionRobotMaterial.dispose();
      idealShooterRobot.geometry.dispose();
      idealShooterMaterial.dispose();
      scene.clear();
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;
    const { wx, wz } = fieldPoseToWorldXZ(
      poseX,
      poseY,
      fieldLengthM,
      fieldWidthM
    );
    robot.position.x = wx;
    robot.position.z = wz;
    robot.rotation.y = fieldHeadingDegToSceneYawRad(headingDeg);
  }, [fieldLengthM, fieldWidthM, headingDeg, poseX, poseY]);

  useEffect(() => {
    const visionRobot = visionRobotRef.current;
    if (!visionRobot) return;
    visionRobot.visible = visionPoseVisible;
    if (!visionPoseVisible) return;
    const { wx, wz } = fieldPoseToWorldXZ(
      visionPoseX ?? 0,
      visionPoseY ?? 0,
      fieldLengthM,
      fieldWidthM
    );
    visionRobot.position.x = wx;
    visionRobot.position.z = wz;
    visionRobot.rotation.y = fieldHeadingDegToSceneYawRad(visionHeadingDeg ?? 0);
  }, [
    fieldLengthM,
    fieldWidthM,
    visionHeadingDeg,
    visionPoseVisible,
    visionPoseX,
    visionPoseY,
  ]);

  useEffect(() => {
    const ideal = idealShooterRobotRef.current;
    if (!ideal) return;
    ideal.visible = idealShooterPoseVisible ?? true;
    if (!ideal.visible) return;
    const { wx, wz } = fieldPoseToWorldXZ(
      idealShooterPoseX ?? 0,
      idealShooterPoseY ?? 0,
      fieldLengthM,
      fieldWidthM
    );
    ideal.position.x = wx;
    ideal.position.z = wz;
    ideal.rotation.y = fieldHeadingDegToSceneYawRad(idealShooterHeadingDeg ?? 0);
  }, [
    fieldLengthM,
    fieldWidthM,
    idealShooterHeadingDeg,
    idealShooterPoseVisible,
    idealShooterPoseX,
    idealShooterPoseY,
  ]);

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
      const { wx, wz } = fieldPoseToWorldXZ(
        t.x,
        t.y,
        fieldLengthM,
        fieldWidthM
      );
      const marker = new THREE.Mesh(markerGeometry, markerMaterial);
      marker.position.x = wx;
      marker.position.y = 0.2;
      marker.position.z = wz;
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
      const { wx, wz } = fieldPoseToWorldXZ(
        p.x,
        p.y,
        fieldLengthM,
        fieldWidthM
      );
      const marker = new THREE.Mesh(sphereGeometry, sphereMaterial);
      marker.position.x = wx;
      marker.position.y = 0.12;
      marker.position.z = wz;
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
        {idealShooterPoseVisible && (
          <>
            <div className="metric-row">
              <span className="metric-label">Hub aim (map)</span>
              <span className="metric-value">
                {(idealShooterHeadingDeg ?? 0).toFixed(1)}°
              </span>
            </div>
            <div className="metric-row">
              <span className="metric-label">Hub aim Δ</span>
              <span className="metric-value">
                {hubHeadingErrorDeg(headingDeg, idealShooterHeadingDeg ?? 0).toFixed(1)}°
              </span>
            </div>
          </>
        )}
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
        <div className="metric-row">
          <span className="metric-label">LL pose</span>
          <span className="metric-value">
            {visionPoseVisible
              ? `${(visionPoseX ?? 0).toFixed(2)}, ${(visionPoseY ?? 0).toFixed(2)}`
              : 'No target'}
          </span>
        </div>
      </div>
    </div>
  );
}
