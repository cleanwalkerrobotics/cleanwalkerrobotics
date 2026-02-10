import type { BagResult } from "./types";

export function buildBagSystem(
	THREE: typeof import("three"),
): BagResult {
	const group = new THREE.Group();

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

	// ---------- Bag Roll Dispenser ----------
	// Cylinder axis left-to-right (Y), center of back
	const roll = new THREE.Mesh(
		new THREE.CylinderGeometry(0.04, 0.04, 0.28, 16),
		rollMat,
	);
	// CylinderGeometry is Y-aligned, which matches our left-right Y axis
	roll.position.set(-0.08, 0, 0.115);
	roll.castShadow = true;
	group.add(roll);

	// Roll cradle mount
	const cradle = new THREE.Mesh(
		new THREE.BoxGeometry(0.10, 0.30, 0.015),
		frameMat,
	);
	cradle.position.set(-0.08, 0, 0.082);
	group.add(cradle);

	// Visible bag material on roll
	const rollBag = new THREE.Mesh(
		new THREE.CylinderGeometry(0.035, 0.035, 0.26, 16),
		bagMat,
	);
	rollBag.position.set(-0.08, 0, 0.115);
	group.add(rollBag);

	// ---------- Hinge servo at rear edge ----------
	const servo = new THREE.Mesh(
		new THREE.BoxGeometry(0.03, 0.04, 0.03),
		frameMat,
	);
	servo.position.set(-0.225, 0, 0.08);
	servo.castShadow = true;
	group.add(servo);

	// ---------- Folding Bag Frame ----------
	const hingeGroup = new THREE.Group();
	hingeGroup.position.set(-0.225, 0, 0.08);
	// 135° open: extends backward and upward, 45° past vertical
	hingeGroup.rotation.y = -(3 * Math.PI) / 4;

	const tubeR = 0.005;
	const frameW = 0.30;
	const frameD = 0.22;

	// Far edge (rim) — runs along Y
	const farEdge = new THREE.Mesh(
		new THREE.CylinderGeometry(tubeR, tubeR, frameW, 8),
		frameMat,
	);
	farEdge.position.set(frameD, 0, 0);
	farEdge.castShadow = true;
	hingeGroup.add(farEdge);

	// Near edge (at hinge) — runs along Y
	const nearEdge = new THREE.Mesh(
		new THREE.CylinderGeometry(tubeR, tubeR, frameW, 8),
		frameMat,
	);
	nearEdge.castShadow = true;
	hingeGroup.add(nearEdge);

	// Left edge — rotated to run along X
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

	// ---------- Hanging Bag ----------
	const bagMesh = createBagMesh(THREE, bagMat);
	group.add(bagMesh);

	return { group, hingeGroup };
}

function createBagMesh(
	THREE: typeof import("three"),
	material: import("three").Material,
): import("three").Mesh {
	const hw = 0.13;

	// Side profile points (X, Z) in URDF coords
	// Inner clip at roll → sag bottom → outer clip at frame rim
	const profile: [number, number][] = [
		[-0.04, 0.115], // inner clip (at roll)
		[-0.10, 0.01], // inner curve
		[-0.21, -0.08], // bottom sag
		[-0.32, 0.01], // outer curve
		[-0.381, 0.236], // outer clip (at frame rim)
	];

	const n = profile.length;
	const verts: number[] = [];

	// Left side (y = -hw), then right side (y = +hw)
	for (const [x, z] of profile) verts.push(x, -hw, z);
	for (const [x, z] of profile) verts.push(x, hw, z);

	const indices: number[] = [];

	// Main bag surface — quad strip between left and right
	for (let i = 0; i < n - 1; i++) {
		const l0 = i,
			l1 = i + 1,
			r0 = i + n,
			r1 = i + 1 + n;
		indices.push(l0, r0, r1, l0, r1, l1);
	}

	// Left side panel
	for (let i = 1; i < n - 1; i++) {
		indices.push(0, i + 1, i);
	}

	// Right side panel
	for (let i = 1; i < n - 1; i++) {
		indices.push(n, n + i, n + i + 1);
	}

	const geom = new THREE.BufferGeometry();
	geom.setAttribute(
		"position",
		new THREE.BufferAttribute(new Float32Array(verts), 3),
	);
	geom.setIndex(indices);
	geom.computeVertexNormals();

	const mesh = new THREE.Mesh(geom, material);
	mesh.castShadow = true;
	return mesh;
}

export function buildGhostBody(
	THREE: typeof import("three"),
): import("three").Mesh {
	const mat = new THREE.MeshStandardMaterial({
		color: 0x3b4a3f,
		roughness: 0.8,
		metalness: 0.1,
		transparent: true,
		opacity: 0.15,
	});
	return new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.3, 0.15), mat);
}
