#!/usr/bin/env node
/**
 * export-glb.mjs — Export CleanWalker CW-1 robot to GLB
 *
 * Builds the complete robot (body + arm + bag system) using Three.js
 * and exports to public/models/cleanwalker-cw1.glb
 *
 * Usage: node scripts/export-glb.mjs
 */

// Polyfill FileReader for Node.js (required by Three.js GLTFExporter)
if (typeof globalThis.FileReader === "undefined") {
  globalThis.FileReader = class FileReader extends EventTarget {
    constructor() {
      super();
      this.result = null;
      this.onloadend = null;
    }
    readAsArrayBuffer(blob) {
      blob.arrayBuffer().then((buf) => {
        this.result = buf;
        if (this.onloadend) this.onloadend();
      });
    }
  };
}

import * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";
import { writeFileSync, mkdirSync } from "node:fs";
import { resolve, dirname } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(__dirname, "..");
const OUTPUT = resolve(ROOT, "public/models/cleanwalker-cw1.glb");

// ──────────────────────────────────────────────
// Materials (V2.3 spec)
// ──────────────────────────────────────────────

function makeMaterials() {
  const oliveGreen = new THREE.MeshStandardMaterial({
    color: 0x3b4a3f,
    roughness: 0.75,
    metalness: 0.1,
  });
  const darkGrey = new THREE.MeshStandardMaterial({
    color: 0x2a2a2a,
    roughness: 0.6,
    metalness: 0.25,
  });
  const rubberBlack = new THREE.MeshStandardMaterial({
    color: 0x1a1a1a,
    roughness: 0.95,
    metalness: 0.02,
  });
  const ledGreen = new THREE.MeshStandardMaterial({
    color: 0x22c55e,
    roughness: 0.3,
    metalness: 0.1,
    emissive: new THREE.Color(0x22c55e),
    emissiveIntensity: 0.5,
  });
  const frameMat = new THREE.MeshStandardMaterial({
    color: 0x2a2a2a,
    roughness: 0.4,
    metalness: 0.5,
  });
  const rollMat = new THREE.MeshStandardMaterial({
    color: 0x1a1a1a,
    roughness: 0.3,
    metalness: 0.6,
  });
  const bagMat = new THREE.MeshStandardMaterial({
    color: 0x1a1a1a,
    roughness: 0.9,
    metalness: 0.0,
    transparent: true,
    opacity: 0.7,
    side: THREE.DoubleSide,
  });

  return { oliveGreen, darkGrey, rubberBlack, ledGreen, frameMat, rollMat, bagMat };
}

// ──────────────────────────────────────────────
// Arm builder (ported from build-robot-body.ts)
// ──────────────────────────────────────────────

function buildArm(oliveGreen, darkGrey, ledGreen, parentGroup) {
  const turretBase = new THREE.Mesh(
    new THREE.CylinderGeometry(0.04, 0.04, 0.05, 16),
    darkGrey
  );
  turretBase.rotation.x = Math.PI / 2;
  turretBase.position.set(0.18, 0, 0.085);
  turretBase.castShadow = true;
  parentGroup.add(turretBase);

  const shoulderBase = new THREE.Group();
  shoulderBase.position.set(0.18, 0, 0.11);

  const shoulderJoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 16, 16),
    darkGrey
  );
  shoulderJoint.castShadow = true;
  shoulderBase.add(shoulderJoint);

  const shoulderPitch = new THREE.Group();
  const baseShoulderPitch = Math.PI / 3;
  shoulderPitch.rotation.y = baseShoulderPitch;

  const upperArm = new THREE.Mesh(
    new THREE.BoxGeometry(0.035, 0.03, 0.18),
    oliveGreen
  );
  upperArm.position.set(0, 0, 0.09);
  upperArm.castShadow = true;
  shoulderPitch.add(upperArm);

  const elbowGroup = new THREE.Group();
  elbowGroup.position.set(0, 0, 0.18);

  const elbowJoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 12, 12),
    darkGrey
  );
  elbowJoint.castShadow = true;
  elbowGroup.add(elbowJoint);

  const elbowPitch = new THREE.Group();
  const baseElbowPitch = Math.PI / 2;
  elbowPitch.rotation.y = baseElbowPitch;

  const forearm = new THREE.Mesh(
    new THREE.BoxGeometry(0.03, 0.025, 0.18),
    oliveGreen
  );
  forearm.position.set(0, 0, 0.09);
  forearm.castShadow = true;
  elbowPitch.add(forearm);

  const wristGroup = new THREE.Group();
  wristGroup.position.set(0, 0, 0.18);

  const wristJoint = new THREE.Mesh(
    new THREE.CylinderGeometry(0.015, 0.015, 0.03, 12),
    darkGrey
  );
  wristJoint.rotation.x = Math.PI / 2;
  wristJoint.castShadow = true;
  wristGroup.add(wristJoint);

  const wristCam = new THREE.Mesh(
    new THREE.SphereGeometry(0.01, 8, 8),
    ledGreen
  );
  wristCam.position.set(0.02, 0, 0);
  wristGroup.add(wristCam);

  const fingerGeom = new THREE.BoxGeometry(0.008, 0.015, 0.06);
  const fingerL = new THREE.Mesh(fingerGeom, darkGrey);
  fingerL.position.set(0, 0.015, 0.035);
  fingerL.rotation.y = 0.1;
  fingerL.castShadow = true;
  wristGroup.add(fingerL);

  const fingerR = new THREE.Mesh(fingerGeom, darkGrey);
  fingerR.position.set(0, -0.015, 0.035);
  fingerR.rotation.y = -0.1;
  fingerR.castShadow = true;
  wristGroup.add(fingerR);

  elbowPitch.add(wristGroup);
  elbowGroup.add(elbowPitch);
  shoulderPitch.add(elbowGroup);
  shoulderBase.add(shoulderPitch);
  parentGroup.add(shoulderBase);

  return { shoulderPitch, elbowPitch, baseShoulderPitch, baseElbowPitch };
}

// ──────────────────────────────────────────────
// Robot body builder (ported from build-robot-body.ts)
// ──────────────────────────────────────────────

function buildRobotBody(mats) {
  const { oliveGreen, darkGrey, rubberBlack, ledGreen } = mats;
  const group = new THREE.Group();

  // Body (V2.3: 600×150×120mm)
  const bodyMesh = new THREE.Mesh(new THREE.BoxGeometry(0.60, 0.15, 0.12), oliveGreen);
  bodyMesh.castShadow = true;
  group.add(bodyMesh);

  const bodyTop = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.13, 0.02), oliveGreen);
  bodyTop.position.set(0, 0, 0.07);
  bodyTop.castShadow = true;
  group.add(bodyTop);

  // LiDAR puck
  const lidar = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.015, 16), darkGrey);
  lidar.rotation.x = Math.PI / 2;
  lidar.position.set(0.05, 0, 0.085);
  group.add(lidar);

  // Head (sensor housing)
  const head = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.15, 0.08), oliveGreen);
  head.position.set(0.30, 0, 0.0);
  head.castShadow = true;
  group.add(head);

  // Eyes
  const eyeGeom = new THREE.BoxGeometry(0.012, 0.038, 0.038);
  const eyeL = new THREE.Mesh(eyeGeom, ledGreen);
  eyeL.position.set(0.346, 0.03, 0.01);
  group.add(eyeL);
  const eyeR = new THREE.Mesh(eyeGeom, ledGreen);
  eyeR.position.set(0.346, -0.03, 0.01);
  group.add(eyeR);

  // Arm
  const arm = buildArm(oliveGreen, darkGrey, ledGreen, group);

  // Legs (×4, mammalian stance)
  const legConfigs = [
    { origin: [0.24, 0.12, -0.06], pitchOffset: [0, 0.04, 0], isRear: false, phase: 0, outwardY: 1 },
    { origin: [0.24, -0.12, -0.06], pitchOffset: [0, -0.04, 0], isRear: false, phase: Math.PI / 2, outwardY: -1 },
    { origin: [-0.24, 0.12, -0.06], pitchOffset: [0, 0.04, 0], isRear: true, phase: Math.PI, outwardY: 1 },
    { origin: [-0.24, -0.12, -0.06], pitchOffset: [0, -0.04, 0], isRear: true, phase: Math.PI * 1.5, outwardY: -1 },
  ];

  for (const cfg of legConfigs) {
    const legGroup = new THREE.Group();
    legGroup.position.set(...cfg.origin);

    const hipYaw = new THREE.Group();
    const hipMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.035, 0.035, 0.07, 16),
      darkGrey
    );
    hipMesh.rotation.x = Math.PI / 2;
    hipMesh.castShadow = true;
    hipYaw.add(hipMesh);

    const hipPitch = new THREE.Group();
    hipPitch.position.set(...cfg.pitchOffset);

    const upperLeg = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.045, 0.20), oliveGreen);
    upperLeg.position.set(0, 0, -0.10);
    upperLeg.castShadow = true;
    hipPitch.add(upperLeg);

    const ledStrip = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.008, 0.15), ledGreen);
    ledStrip.position.set(0, cfg.outwardY * 0.027, -0.10);
    hipPitch.add(ledStrip);

    const kneeVis = new THREE.Mesh(new THREE.SphereGeometry(0.032, 12, 12), darkGrey);
    kneeVis.position.set(0, 0, -0.20);
    kneeVis.castShadow = true;
    hipPitch.add(kneeVis);

    const knee = new THREE.Group();
    knee.position.set(0, 0, -0.20);

    const lowerLeg = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.035, 0.20), oliveGreen);
    lowerLeg.position.set(0, 0, -0.10);
    lowerLeg.castShadow = true;
    knee.add(lowerLeg);

    const foot = new THREE.Mesh(new THREE.SphereGeometry(0.025, 16, 16), rubberBlack);
    foot.position.set(0, 0, -0.20);
    foot.castShadow = true;
    knee.add(foot);

    const baseHipPitch = cfg.isRear ? -0.15 : 0.15;
    const baseKneePitch = cfg.isRear ? -0.30 : 0.30;
    hipPitch.rotation.y = baseHipPitch;
    knee.rotation.y = baseKneePitch;

    hipPitch.add(knee);
    hipYaw.add(hipPitch);
    legGroup.add(hipYaw);
    group.add(legGroup);
  }

  return group;
}

// ──────────────────────────────────────────────
// Bag system builder (ported from build-bag-system.ts)
// ──────────────────────────────────────────────

function buildBagSystem(mats) {
  const { frameMat, rollMat, bagMat } = mats;
  const group = new THREE.Group();

  // Bag roll dispenser
  const roll = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.14, 16), rollMat);
  roll.position.set(-0.06, 0, 0.10);
  roll.castShadow = true;
  group.add(roll);

  const cradle = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.16, 0.015), frameMat);
  cradle.position.set(-0.06, 0, 0.065);
  group.add(cradle);

  const rollBag = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.12, 16), bagMat);
  rollBag.position.set(-0.06, 0, 0.10);
  group.add(rollBag);

  // Hinge servo
  const servo = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.04, 0.03), frameMat);
  servo.position.set(-0.30, 0, 0.065);
  servo.castShadow = true;
  group.add(servo);

  // Folding bag frame
  const hingeGroup = new THREE.Group();
  hingeGroup.position.set(-0.30, 0, 0.065);
  hingeGroup.rotation.y = -(3 * Math.PI) / 4;

  const tubeR = 0.005;
  const frameW = 0.15;
  const frameD = 0.22;

  const farEdge = new THREE.Mesh(new THREE.CylinderGeometry(tubeR, tubeR, frameW, 8), frameMat);
  farEdge.position.set(frameD, 0, 0);
  farEdge.castShadow = true;
  hingeGroup.add(farEdge);

  const nearEdge = new THREE.Mesh(new THREE.CylinderGeometry(tubeR, tubeR, frameW, 8), frameMat);
  nearEdge.castShadow = true;
  hingeGroup.add(nearEdge);

  const sideGeom = new THREE.CylinderGeometry(tubeR, tubeR, frameD, 8);
  const leftEdge = new THREE.Mesh(sideGeom, frameMat);
  leftEdge.rotation.z = Math.PI / 2;
  leftEdge.position.set(frameD / 2, -frameW / 2, 0);
  leftEdge.castShadow = true;
  hingeGroup.add(leftEdge);

  const rightEdge = new THREE.Mesh(sideGeom, frameMat);
  rightEdge.rotation.z = Math.PI / 2;
  rightEdge.position.set(frameD / 2, frameW / 2, 0);
  rightEdge.castShadow = true;
  hingeGroup.add(rightEdge);

  group.add(hingeGroup);

  // Hanging bag mesh
  const bagMesh = createBagMesh(bagMat);
  group.add(bagMesh);

  return group;
}

function createBagMesh(material) {
  const hw = 0.065;
  const profile = [
    [-0.02, 0.10],
    [-0.10, 0.00],
    [-0.24, -0.10],
    [-0.36, 0.00],
    [-0.455, 0.22],
  ];

  const n = profile.length;
  const verts = [];
  for (const [x, z] of profile) verts.push(x, -hw, z);
  for (const [x, z] of profile) verts.push(x, hw, z);

  const indices = [];
  for (let i = 0; i < n - 1; i++) {
    const l0 = i, l1 = i + 1, r0 = i + n, r1 = i + 1 + n;
    indices.push(l0, r0, r1, l0, r1, l1);
  }
  for (let i = 1; i < n - 1; i++) indices.push(0, i + 1, i);
  for (let i = 1; i < n - 1; i++) indices.push(n, n + i, n + i + 1);

  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(verts), 3));
  geom.setIndex(indices);
  geom.computeVertexNormals();

  const mesh = new THREE.Mesh(geom, material);
  mesh.castShadow = true;
  return mesh;
}

// ──────────────────────────────────────────────
// Main: build scene → export GLB
// ──────────────────────────────────────────────

async function main() {
  console.log("Building CleanWalker CW-1 robot...");

  const mats = makeMaterials();

  // Build full robot
  const robot = new THREE.Group();
  robot.name = "CleanWalker_CW1";

  const body = buildRobotBody(mats);
  body.name = "Body";
  robot.add(body);

  const bag = buildBagSystem(mats);
  bag.name = "BagSystem";
  robot.add(bag);

  // Rotate to Y-up convention (Three.js model uses Z-up internally)
  robot.rotation.x = -Math.PI / 2;

  // Create scene for export
  const scene = new THREE.Scene();
  scene.name = "CW1_Scene";
  scene.add(robot);

  console.log("Exporting to GLB...");

  const exporter = new GLTFExporter();

  const glb = await exporter.parseAsync(scene, {
    binary: true,
    includeCustomExtensions: false,
  });

  // glb is an ArrayBuffer for binary mode
  mkdirSync(resolve(ROOT, "public/models"), { recursive: true });
  writeFileSync(OUTPUT, Buffer.from(glb));

  const sizeMB = (Buffer.from(glb).length / 1024 / 1024).toFixed(2);
  console.log(`Exported: ${OUTPUT} (${sizeMB} MB)`);
}

main().catch((err) => {
  console.error("Export failed:", err);
  process.exit(1);
});
