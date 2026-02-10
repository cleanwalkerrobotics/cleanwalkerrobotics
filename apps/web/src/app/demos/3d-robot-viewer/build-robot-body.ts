import type { BodyResult, LegRef, ArmRef } from "./types";

export function buildRobotBody(THREE: typeof import("three")): BodyResult {
	// -- Materials (V2.3 spec) --
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

	// Panel seam material — slightly darker than body
	const seamDark = new THREE.MeshStandardMaterial({
		color: 0x2e3b32,
		roughness: 0.8,
		metalness: 0.1,
	});

	const group = new THREE.Group();
	const legs: LegRef[] = [];

	// ---------- Body Shell (V2.3: 600×150×120mm, chamfered edges) ----------
	// Rounded rectangle shape for extrusion — creates injection-molded ABS look
	const bodyW = 0.15; // Y dimension (width)
	const bodyH = 0.60; // X dimension (length) — extruded along Z, rotated later
	const bodyD = 0.12; // Z dimension (height) — extrusion depth
	const bevelR = 0.012; // corner radius

	const bodyShape = new THREE.Shape();
	const bHW = bodyW / 2 - bevelR;
	const bHH = bodyH / 2 - bevelR;
	bodyShape.moveTo(-bHW, -bHH - bevelR);
	bodyShape.lineTo(bHW, -bHH - bevelR);
	bodyShape.quadraticCurveTo(bHW + bevelR, -bHH - bevelR, bHW + bevelR, -bHH);
	bodyShape.lineTo(bHW + bevelR, bHH);
	bodyShape.quadraticCurveTo(bHW + bevelR, bHH + bevelR, bHW, bHH + bevelR);
	bodyShape.lineTo(-bHW, bHH + bevelR);
	bodyShape.quadraticCurveTo(-bHW - bevelR, bHH + bevelR, -bHW - bevelR, bHH);
	bodyShape.lineTo(-bHW - bevelR, -bHH);
	bodyShape.quadraticCurveTo(-bHW - bevelR, -bHH - bevelR, -bHW, -bHH - bevelR);

	const bodyExtrudeSettings = {
		depth: bodyD,
		bevelEnabled: true,
		bevelThickness: 0.004,
		bevelSize: 0.004,
		bevelSegments: 3,
	};
	const bodyGeom = new THREE.ExtrudeGeometry(bodyShape, bodyExtrudeSettings);
	// Rotate so extrusion goes along Z, then orient: shape XY → body YX, extrude Z → body Z
	const bodyMesh = new THREE.Mesh(bodyGeom, oliveGreen);
	// Shape is in XY plane, extruded along +Z. We want: shape X→body Y, shape Y→body X, extrude Z→body Z
	// After extrusion, geometry spans: X=[-W/2, W/2], Y=[-H/2, H/2], Z=[0, bodyD]
	// We need body centered at origin: rotate and translate
	bodyMesh.rotation.set(0, 0, Math.PI / 2); // swap X↔Y so length is along X
	bodyMesh.position.set(0, 0, -bodyD / 2); // center Z
	bodyMesh.castShadow = true;
	group.add(bodyMesh);

	// Raised top panel with bevel (slightly inset from body edges)
	const topW = 0.13;
	const topH = 0.55;
	const topD = 0.015;
	const topR = 0.008;
	const topShape = new THREE.Shape();
	const tHW = topW / 2 - topR;
	const tHH = topH / 2 - topR;
	topShape.moveTo(-tHW, -tHH - topR);
	topShape.lineTo(tHW, -tHH - topR);
	topShape.quadraticCurveTo(tHW + topR, -tHH - topR, tHW + topR, -tHH);
	topShape.lineTo(tHW + topR, tHH);
	topShape.quadraticCurveTo(tHW + topR, tHH + topR, tHW, tHH + topR);
	topShape.lineTo(-tHW, tHH + topR);
	topShape.quadraticCurveTo(-tHW - topR, tHH + topR, -tHW - topR, tHH);
	topShape.lineTo(-tHW - topR, -tHH);
	topShape.quadraticCurveTo(-tHW - topR, -tHH - topR, -tHW, -tHH - topR);
	const topGeom = new THREE.ExtrudeGeometry(topShape, {
		depth: topD,
		bevelEnabled: true,
		bevelThickness: 0.002,
		bevelSize: 0.002,
		bevelSegments: 2,
	});
	const bodyTop = new THREE.Mesh(topGeom, oliveGreen);
	bodyTop.rotation.set(0, 0, Math.PI / 2);
	bodyTop.position.set(0, 0, 0.06);
	bodyTop.castShadow = true;
	group.add(bodyTop);

	// ---------- Panel Seam Lines (subtle recessed lines on body sides) ----------
	// Long side seam lines (visible on left and right body sides)
	for (const side of [1, -1]) {
		// Mid-height horizontal line on each long side
		const seamMid = new THREE.Mesh(
			new THREE.BoxGeometry(0.54, 0.001, 0.003),
			seamDark,
		);
		seamMid.position.set(0, side * 0.076, 0.0);
		group.add(seamMid);

		// Lower horizontal line
		const seamLow = new THREE.Mesh(
			new THREE.BoxGeometry(0.54, 0.001, 0.003),
			seamDark,
		);
		seamLow.position.set(0, side * 0.076, -0.03);
		group.add(seamLow);
	}

	// Transverse seam separating rear bag section from main body (at ~X = -0.10)
	const seamRear = new THREE.Mesh(
		new THREE.BoxGeometry(0.001, 0.15, 0.10),
		seamDark,
	);
	seamRear.position.set(-0.10, 0, 0.0);
	group.add(seamRear);

	// ---------- Rear Mounting Points (hinge brackets for bag frame) ----------
	// Two L-shaped brackets at X = -0.30, Y = ±0.05, Z = 0.065
	for (const side of [0.05, -0.05]) {
		// Vertical plate of bracket
		const bracketVert = new THREE.Mesh(
			new THREE.BoxGeometry(0.003, 0.015, 0.018),
			darkGrey,
		);
		bracketVert.position.set(-0.30, side, 0.065);
		bracketVert.castShadow = true;
		group.add(bracketVert);

		// Horizontal plate of bracket (top flange)
		const bracketHoriz = new THREE.Mesh(
			new THREE.BoxGeometry(0.015, 0.015, 0.003),
			darkGrey,
		);
		bracketHoriz.position.set(-0.3075, side, 0.075);
		bracketHoriz.castShadow = true;
		group.add(bracketHoriz);
	}

	// ---------- Roll Cradle / Recess (where bag roll sits) ----------
	// Two raised guide rails running across body width at X = -0.06
	for (const offset of [0.02, -0.02]) {
		const rail = new THREE.Mesh(
			new THREE.BoxGeometry(0.008, 0.12, 0.005),
			darkGrey,
		);
		rail.position.set(-0.06 + offset, 0, 0.065);
		rail.castShadow = true;
		group.add(rail);
	}
	// Shallow channel between rails (slightly recessed visual — thin dark strip)
	const cradleChannel = new THREE.Mesh(
		new THREE.BoxGeometry(0.032, 0.11, 0.001),
		seamDark,
	);
	cradleChannel.position.set(-0.06, 0, 0.061);
	group.add(cradleChannel);

	// LiDAR puck on top (between arm mount and bag roll)
	const lidar = new THREE.Mesh(
		new THREE.CylinderGeometry(0.02, 0.02, 0.015, 16),
		darkGrey,
	);
	lidar.rotation.x = Math.PI / 2;
	lidar.position.set(0.05, 0, 0.085);
	group.add(lidar);

	// ---------- Head (integrated tapered sensor housing) ----------
	// Custom geometry: body tapers from full width to a narrower head at the front
	// Use a tapered extrusion — narrowing the last ~8cm of the body
	const headShape = new THREE.Shape();
	const headFrontW = 0.13; // slightly narrower at front face
	const headRearW = 0.15; // full body width where it meets body
	const headLen = 0.08;
	const headH = 0.08;
	const hR = 0.006; // small corner radius for head

	// Head profile in XY plane (X = width, Y = length), extruded along Z (height)
	// Trapezoidal profile: wider at back (Y=0), narrower at front (Y=headLen)
	const hFHW = headFrontW / 2 - hR;
	const hRHW = headRearW / 2 - hR;
	headShape.moveTo(-hRHW, -hR);
	headShape.lineTo(-hFHW, headLen - hR);
	headShape.quadraticCurveTo(-hFHW, headLen, -hFHW + hR, headLen);
	headShape.lineTo(hFHW - hR, headLen);
	headShape.quadraticCurveTo(hFHW, headLen, hFHW, headLen - hR);
	headShape.lineTo(hRHW, -hR);
	headShape.quadraticCurveTo(hRHW, 0, hRHW - hR, 0);
	headShape.lineTo(-hRHW + hR, 0);
	headShape.quadraticCurveTo(-hRHW, 0, -hRHW, -hR);

	const headGeom = new THREE.ExtrudeGeometry(headShape, {
		depth: headH,
		bevelEnabled: true,
		bevelThickness: 0.003,
		bevelSize: 0.003,
		bevelSegments: 2,
	});
	const head = new THREE.Mesh(headGeom, oliveGreen);
	// Orient: shape X → body Y (width), shape Y → body +X (forward), extrude Z → body Z (height)
	// Rotation -π/2 around Z: shape Y → world +X, shape X → world -Y (centered, symmetric)
	head.rotation.set(0, 0, -Math.PI / 2);
	// Position: rear edge at X=0.26 (4cm overlap with body), front face at X=0.34, Z centered
	head.position.set(0.26, 0, -headH / 2);
	head.castShadow = true;
	group.add(head);

	// Eyes — two square green LED panels on flat front face
	const eyeGeom = new THREE.BoxGeometry(0.012, 0.038, 0.038);
	const eyeL = new THREE.Mesh(eyeGeom, ledGreen);
	eyeL.position.set(0.346, 0.03, 0.01);
	group.add(eyeL);
	const eyeR = new THREE.Mesh(eyeGeom, ledGreen);
	eyeR.position.set(0.346, -0.03, 0.01);
	group.add(eyeR);

	// ---------- Arm turret mount flange + arm ----------
	// Mounting flange ring at base of turret (wider than turret, thin disc)
	const turretFlange = new THREE.Mesh(
		new THREE.CylinderGeometry(0.05, 0.05, 0.005, 20),
		darkGrey,
	);
	turretFlange.rotation.x = Math.PI / 2;
	turretFlange.position.set(0.18, 0, 0.0625);
	turretFlange.castShadow = true;
	group.add(turretFlange);

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
			origin: [0.24, 0.12, -0.06],
			pitchOffset: [0, 0.04, 0],
			isRear: false,
			phase: 0,
			outwardY: 1,
		},
		{
			origin: [0.24, -0.12, -0.06],
			pitchOffset: [0, -0.04, 0],
			isRear: false,
			phase: Math.PI / 2,
			outwardY: -1,
		},
		{
			origin: [-0.24, 0.12, -0.06],
			pitchOffset: [0, 0.04, 0],
			isRear: true,
			phase: Math.PI,
			outwardY: 1,
		},
		{
			origin: [-0.24, -0.12, -0.06],
			pitchOffset: [0, -0.04, 0],
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
	// Cylindrical turret base on body top, front-center, behind head (~8cm dia, ~5cm tall)
	const turretBase = new THREE.Mesh(
		new THREE.CylinderGeometry(0.04, 0.04, 0.05, 16),
		darkGrey,
	);
	turretBase.rotation.x = Math.PI / 2;
	turretBase.position.set(0.18, 0, 0.085);
	turretBase.castShadow = true;
	parentGroup.add(turretBase);

	// Shoulder mount on top of turret
	const shoulderBase = new THREE.Group();
	shoulderBase.position.set(0.18, 0, 0.11);

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

	// Gripper — two mechanical fingers with silicone-tipped pads
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
