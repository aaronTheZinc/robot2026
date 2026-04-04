import { useEffect, useRef } from 'react';
import * as THREE from 'three';

import {
  fieldHeadingDegToSceneYawRad,
  fieldPoseToWorldXZ,
} from '../lib/simFieldToThree';

type FieldViewProps = {
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
  loggedPoints?: { x: number; y: number }[];
};

const ROBOT_HEIGHT_M = 0.18;

export default function FieldView({
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
  loggedPoints,
}: FieldViewProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Mesh | null>(null);
  const visionRobotRef = useRef<THREE.Mesh | null>(null);
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

    rendererRef.current = renderer;
    sceneRef.current = scene;
    cameraRef.current = camera;
    robotRef.current = robot;
    visionRobotRef.current = visionRobot;
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
      scene.clear();
      if (renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, [fieldLengthM, fieldWidthM, robotLengthM, robotWidthM]);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) {
      return;
    }
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
    if (!visionRobot) {
      return;
    }
    visionRobot.visible = visionPoseVisible;
    if (!visionPoseVisible) {
      return;
    }
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

  return <div className="field-canvas" ref={containerRef} />;
}
