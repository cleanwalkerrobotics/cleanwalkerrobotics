import type { BodyResult, LegRef, ArmRef } from "./types";

export function buildRobotBody(THREE: typeof import("three")): BodyResult {
	// -- Materials (V2.2 spec) --
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

	const group = new THREE.Group();
	const legs: LegRef[] = [];

	// ---------- Body (V2.2: 450×300×150mm, 3:2:1 ratio) ----------
	const bodyMesh = new THREE.Mesh(
		new THREE.BoxGeometry(0.45, 0.30, 0.15),
		oliveGreen,
	);
	bodyMesh.castShadow = true;
	group.add(bodyMesh);

	// Raised top panel for visual depth
	const bodyTop = new THREE.Mesh(
		new THREE.BoxGeometry(0.40, 0.26, 0.02),
		oliveGreen,
	);
	bodyTop.position.set(0, 0, 0.085);
	bodyTop.castShadow = true;
	group.add(bodyTop);

	// LiDAR puck on top
	const lidar = new THREE.Mesh(
		new THREE.CylinderGeometry(0.02, 0.02, 0.015, 16),
		darkGrey,
	);
	lidar.rotation.x = Math.PI / 2;
	lidar.position.set(0.02, 0, 0.10);
	group.add(lidar);

	// ---------- Head module (flat-face sensor housing) ----------
	const head = new THREE.Mesh(
		new THREE.BoxGeometry(0.10, 0.12, 0.10),
		oliveGreen,
	);
	head.position.set(0.275, 0, 0.03);
	head.castShadow = true;
	group.add(head);

	// Eyes — two square green LED panels on flat front face
	const eyeGeom = new THREE.BoxGeometry(0.012, 0.038, 0.038);
	const eyeL = new THREE.Mesh(eyeGeom, ledGreen);
	eyeL.position.set(0.331, 0.03, 0.04);
	group.add(eyeL);
	const eyeR = new THREE.Mesh(eyeGeom, ledGreen);
	eyeR.position.set(0.331, -0.03, 0.04);
	group.add(eyeR);

	// ---------- Arm (multi-joint, front-center) ----------
	const arm = buildArm(THREE, oliveGreen, darkGrey, ledGreen, group);

	// ---------- Legs (×4, mammalian stance) ----------
	const legConfigs: {
		origin: [number, number, number];
		pitchOffset: [number, number, number];
		isRear: boolean;
		phase: number;
		outwardY: number;
	}[] = [
		{
			origin: [0.17, 0.16, -0.075],
			pitchOffset: [0, 0.03, 0],
			isRear: false,
			phase: 0,
			outwardY: 1,
		},
		{
			origin: [0.17, -0.16, -0.075],
			pitchOffset: [0, -0.03, 0],
			isRear: false,
			phase: Math.PI / 2,
			outwardY: -1,
		},
		{
			origin: [-0.17, 0.16, -0.075],
			pitchOffset: [0, 0.03, 0],
			isRear: true,
			phase: Math.PI,
			outwardY: 1,
		},
		{
			origin: [-0.17, -0.16, -0.075],
			pitchOffset: [0, -0.03, 0],
			isRear: true,
			phase: Math.PI * 1.5,
			outwardY: -1,
		},
	];

	for (const cfg of legConfigs) {
		const legGroup = new THREE.Group();
		legGroup.position.set(...cfg.origin);

		// Hip yaw joint (dark grey cylindrical housing)
		const hipYaw = new THREE.Group();
		const hipMesh = new THREE.Mesh(
			new THREE.CylinderGeometry(0.035, 0.035, 0.07, 16),
			darkGrey,
		);
		hipMesh.rotation.x = Math.PI / 2;
		hipMesh.castShadow = true;
		hipYaw.add(hipMesh);

		// Hip pitch group — upper leg
		const hipPitch = new THREE.Group();
		hipPitch.position.set(...cfg.pitchOffset);

		// Upper leg — box shape for flat panel surface
		const upperLeg = new THREE.Mesh(
			new THREE.BoxGeometry(0.055, 0.045, 0.20),
			oliveGreen,
		);
		upperLeg.position.set(0, 0, -0.10);
		upperLeg.castShadow = true;
		hipPitch.add(upperLeg);

		// LED strip on outward-facing surface of upper leg
		const ledStrip = new THREE.Mesh(
			new THREE.BoxGeometry(0.008, 0.008, 0.15),
			ledGreen,
		);
		ledStrip.position.set(0, cfg.outwardY * 0.027, -0.10);
		hipPitch.add(ledStrip);

		// Knee joint (dark grey sphere)
		const kneeVis = new THREE.Mesh(
			new THREE.SphereGeometry(0.032, 12, 12),
			darkGrey,
		);
		kneeVis.position.set(0, 0, -0.20);
		kneeVis.castShadow = true;
		hipPitch.add(kneeVis);

		// Knee group — lower leg + foot
		const knee = new THREE.Group();
		knee.position.set(0, 0, -0.20);

		const lowerLeg = new THREE.Mesh(
			new THREE.BoxGeometry(0.045, 0.035, 0.20),
			oliveGreen,
		);
		lowerLeg.position.set(0, 0, -0.10);
		lowerLeg.castShadow = true;
		knee.add(lowerLeg);

		const foot = new THREE.Mesh(
			new THREE.SphereGeometry(0.025, 16, 16),
			rubberBlack,
		);
		foot.position.set(0, 0, -0.20);
		foot.castShadow = true;
		knee.add(foot);

		// Standing pose — mammalian stance
		const baseHipPitch = cfg.isRear ? -0.15 : 0.15;
		const baseKneePitch = cfg.isRear ? -0.30 : 0.30;
		hipPitch.rotation.y = baseHipPitch;
		knee.rotation.y = baseKneePitch;

		hipPitch.add(knee);
		hipYaw.add(hipPitch);
		legGroup.add(hipYaw);
		group.add(legGroup);

		legs.push({
			hipPitch,
			knee,
			baseHipPitch,
			baseKneePitch,
			phase: cfg.phase,
		});
	}

	return { group, legs, arm, ledMaterial: ledGreen };
}

function buildArm(
	THREE: typeof import("three"),
	oliveGreen: import("three").MeshStandardMaterial,
	darkGrey: import("three").MeshStandardMaterial,
	ledGreen: import("three").MeshStandardMaterial,
	parentGroup: import("three").Group,
): ArmRef {
	// Shoulder mount on body top, front-center, behind head
	const shoulderBase = new THREE.Group();
	shoulderBase.position.set(0.12, 0, 0.095);

	// Shoulder joint housing
	const shoulderJoint = new THREE.Mesh(
		new THREE.SphereGeometry(0.03, 16, 16),
		darkGrey,
	);
	shoulderJoint.castShadow = true;
	shoulderBase.add(shoulderJoint);

	// Shoulder pitch group (arm starts going upward, rotated forward)
	const shoulderPitch = new THREE.Group();
	const baseShoulderPitch = Math.PI / 3; // 60° forward from vertical
	shoulderPitch.rotation.y = baseShoulderPitch;

	// Upper arm segment
	const upperArm = new THREE.Mesh(
		new THREE.BoxGeometry(0.035, 0.03, 0.18),
		oliveGreen,
	);
	upperArm.position.set(0, 0, 0.09);
	upperArm.castShadow = true;
	shoulderPitch.add(upperArm);

	// Elbow joint
	const elbowGroup = new THREE.Group();
	elbowGroup.position.set(0, 0, 0.18);

	const elbowJoint = new THREE.Mesh(
		new THREE.SphereGeometry(0.025, 12, 12),
		darkGrey,
	);
	elbowJoint.castShadow = true;
	elbowGroup.add(elbowJoint);

	// Elbow pitch group
	const elbowPitch = new THREE.Group();
	const baseElbowPitch = Math.PI / 2; // 90° additional bend
	elbowPitch.rotation.y = baseElbowPitch;

	// Forearm segment
	const forearm = new THREE.Mesh(
		new THREE.BoxGeometry(0.03, 0.025, 0.18),
		oliveGreen,
	);
	forearm.position.set(0, 0, 0.09);
	forearm.castShadow = true;
	elbowPitch.add(forearm);

	// Wrist
	const wristGroup = new THREE.Group();
	wristGroup.position.set(0, 0, 0.18);

	const wristJoint = new THREE.Mesh(
		new THREE.CylinderGeometry(0.015, 0.015, 0.03, 12),
		darkGrey,
	);
	wristJoint.rotation.x = Math.PI / 2;
	wristJoint.castShadow = true;
	wristGroup.add(wristJoint);

	// Wrist camera
	const wristCam = new THREE.Mesh(
		new THREE.SphereGeometry(0.01, 8, 8),
		ledGreen,
	);
	wristCam.position.set(0.02, 0, 0);
	wristGroup.add(wristCam);

	// Gripper — two mechanical fingers
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

	// Assemble arm chain
	elbowPitch.add(wristGroup);
	elbowGroup.add(elbowPitch);
	shoulderPitch.add(elbowGroup);
	shoulderBase.add(shoulderPitch);
	parentGroup.add(shoulderBase);

	return { shoulderPitch, elbowPitch, baseShoulderPitch, baseElbowPitch };
}
