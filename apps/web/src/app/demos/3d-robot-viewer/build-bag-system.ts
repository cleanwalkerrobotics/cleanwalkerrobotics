import type { BagResult } from "./types";

export function buildBagSystem(
	THREE: typeof import("three"),
): BagResult {
	const group = new THREE.Group();

	/* ── Materials ────────────────────────────────────────────── */
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
		opacity: 0.65,
		side: THREE.DoubleSide,
	});
	const clipMat = new THREE.MeshStandardMaterial({
		color: 0x888888,
		roughness: 0.3,
		metalness: 0.8,
	});
	const servoMat = new THREE.MeshStandardMaterial({
		color: 0x222222,
		roughness: 0.5,
		metalness: 0.4,
	});
	const cordMat = new THREE.MeshStandardMaterial({
		color: 0x444444,
		roughness: 0.8,
	});

	/* ── Constants ────────────────────────────────────────────── */
	const ROLL_X = -0.06;
	const ROLL_Z = 0.1;
	const ROLL_R = 0.04;
	const ROLL_W = 0.13;
	const CRADLE_Z = 0.065;
	const HINGE_X = -0.3;
	const HINGE_Z = 0.065;
	const FRAME_W = 0.15;
	const FRAME_D = 0.22;
	const TUBE_R = 0.005;
	const HW = FRAME_W / 2;
	const OPEN_ANGLE = -(3 * Math.PI) / 4;
	const CLOSED_ANGLE = 0;
	const BAG_HW = 0.065;
	const BAG_ARC = 0.55;
	const INNER_CLIP_X = -0.02;
	const INNER_CLIP_Z = 0.08;
	const PROFILE_N = 12;
	const LATERAL_N = 8;

	/* ── Helper: oriented tube between two 3D points ─────────── */
	function tube(
		a: [number, number, number],
		b: [number, number, number],
		r: number,
		mat: import("three").Material,
	) {
		const dx = b[0] - a[0];
		const dy = b[1] - a[1];
		const dz = b[2] - a[2];
		const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
		const geom = new THREE.CylinderGeometry(r, r, len, 8);
		const mesh = new THREE.Mesh(geom, mat);
		mesh.position.set(
			(a[0] + b[0]) / 2,
			(a[1] + b[1]) / 2,
			(a[2] + b[2]) / 2,
		);
		const dir = new THREE.Vector3(dx, dy, dz).normalize();
		const up = new THREE.Vector3(0, 1, 0);
		if (Math.abs(dir.dot(up)) < 0.9999) {
			mesh.quaternion.setFromUnitVectors(up, dir);
		}
		mesh.castShadow = true;
		return mesh;
	}

	/* ═══════════════════════════════════════════════════════════
	   1. BAG ROLL DISPENSER
	   ═══════════════════════════════════════════════════════════ */

	// Main roll cylinder (black anodized aluminum)
	const roll = new THREE.Mesh(
		new THREE.CylinderGeometry(ROLL_R, ROLL_R, ROLL_W, 24),
		rollMat,
	);
	roll.position.set(ROLL_X, 0, ROLL_Z);
	roll.castShadow = true;
	group.add(roll);

	// Visible bag material layer on the roll
	const bagLayer = new THREE.Mesh(
		new THREE.CylinderGeometry(
			ROLL_R - 0.005,
			ROLL_R - 0.005,
			ROLL_W - 0.01,
			24,
		),
		bagMat,
	);
	bagLayer.position.set(ROLL_X, 0, ROLL_Z);
	group.add(bagLayer);

	// Axle end caps (small discs at both Y ends)
	const capGeom = new THREE.CylinderGeometry(0.006, 0.006, 0.005, 12);
	for (const s of [-1, 1]) {
		const cap = new THREE.Mesh(capGeom, frameMat);
		cap.position.set(ROLL_X, s * (ROLL_W / 2 + 0.002), ROLL_Z);
		group.add(cap);
	}

	// Mounting brackets (L-shaped cradle supports at each side)
	for (const s of [-1, 1]) {
		// Horizontal base
		const base = new THREE.Mesh(
			new THREE.BoxGeometry(0.06, 0.008, 0.035),
			frameMat,
		);
		base.position.set(ROLL_X, s * 0.07, CRADLE_Z);
		base.castShadow = true;
		group.add(base);

		// Vertical arm (U-shape to cradle the roll)
		const arm = new THREE.Mesh(
			new THREE.BoxGeometry(0.008, 0.008, 0.04),
			frameMat,
		);
		arm.position.set(ROLL_X, s * 0.07, CRADLE_Z + 0.032);
		group.add(arm);
	}

	// Cradle base plate
	const cradle = new THREE.Mesh(
		new THREE.BoxGeometry(0.1, 0.15, 0.008),
		frameMat,
	);
	cradle.position.set(ROLL_X, 0, CRADLE_Z - 0.004);
	group.add(cradle);

	/* ═══════════════════════════════════════════════════════════
	   2. HINGE SERVO
	   ═══════════════════════════════════════════════════════════ */

	const servo = new THREE.Mesh(
		new THREE.BoxGeometry(0.03, 0.04, 0.03),
		servoMat,
	);
	servo.position.set(HINGE_X, 0, HINGE_Z);
	servo.castShadow = true;
	group.add(servo);

	// Servo shaft (visible axle running left-to-right)
	const shaft = new THREE.Mesh(
		new THREE.CylinderGeometry(0.004, 0.004, 0.05, 8),
		frameMat,
	);
	shaft.position.set(HINGE_X, 0, HINGE_Z + 0.015);
	group.add(shaft);

	/* ═══════════════════════════════════════════════════════════
	   3. FOLDING BAG FRAME (tube construction)
	   ═══════════════════════════════════════════════════════════ */

	const hingeGroup = new THREE.Group();
	hingeGroup.position.set(HINGE_X, 0, HINGE_Z);
	hingeGroup.rotation.y = OPEN_ANGLE;

	// Frame rectangle corners (hingeGroup local coords)
	const NL: [number, number, number] = [0, -HW, 0];
	const NR: [number, number, number] = [0, HW, 0];
	const FL: [number, number, number] = [FRAME_D, -HW, 0];
	const FR: [number, number, number] = [FRAME_D, HW, 0];

	// Four edges of the rectangular frame
	hingeGroup.add(tube(NL, NR, TUBE_R, frameMat)); // near edge
	hingeGroup.add(tube(FL, FR, TUBE_R, frameMat)); // far edge (rim)
	hingeGroup.add(tube(NL, FL, TUBE_R, frameMat)); // left side
	hingeGroup.add(tube(NR, FR, TUBE_R, frameMat)); // right side

	// Cross-brace at mid-depth for rigidity
	hingeGroup.add(
		tube(
			[FRAME_D / 2, -HW, 0],
			[FRAME_D / 2, HW, 0],
			TUBE_R * 0.7,
			frameMat,
		),
	);

	// Two support bars from hinge area to far corners
	const SY = 0.05;
	hingeGroup.add(tube([0, -SY, 0], FL, TUBE_R * 0.8, frameMat));
	hingeGroup.add(tube([0, SY, 0], FR, TUBE_R * 0.8, frameMat));

	// Corner reinforcement gussets at far edge
	const gussetGeom = new THREE.BoxGeometry(0.015, 0.012, 0.003);
	for (const ys of [-1, 1]) {
		const g = new THREE.Mesh(gussetGeom, frameMat);
		g.position.set(FRAME_D - 0.008, ys * HW, 0);
		hingeGroup.add(g);
	}

	group.add(hingeGroup);

	/* ═══════════════════════════════════════════════════════════
	   4. CLIP MECHANISMS
	   ═══════════════════════════════════════════════════════════ */

	const clipGeom = new THREE.BoxGeometry(0.008, 0.012, 0.005);
	const springGeom = new THREE.BoxGeometry(0.006, 0.008, 0.002);

	// Inner clips (fixed on body, near roll front edge)
	for (const ys of [-1, 1]) {
		const clip = new THREE.Mesh(clipGeom, clipMat);
		clip.position.set(INNER_CLIP_X, ys * 0.05, INNER_CLIP_Z);
		clip.castShadow = true;
		group.add(clip);

		const spr = new THREE.Mesh(springGeom, clipMat);
		spr.position.set(INNER_CLIP_X + 0.005, ys * 0.05, INNER_CLIP_Z + 0.003);
		group.add(spr);
	}

	// Outer clips (on frame, at far rim)
	for (const ys of [-1, 1]) {
		const clip = new THREE.Mesh(clipGeom, clipMat);
		clip.position.set(FRAME_D, ys * 0.05, 0);
		clip.castShadow = true;
		hingeGroup.add(clip);

		const spr = new THREE.Mesh(springGeom, clipMat);
		spr.position.set(FRAME_D - 0.005, ys * 0.05, 0.003);
		hingeGroup.add(spr);
	}

	/* ═══════════════════════════════════════════════════════════
	   5. HANGING BAG — high-detail catenary mesh
	   ═══════════════════════════════════════════════════════════ */

	const totalVerts = (PROFILE_N + 1) * (LATERAL_N + 1);
	const positions = new Float32Array(totalVerts * 3);
	const bagGeom = new THREE.BufferGeometry();
	const posAttr = new THREE.BufferAttribute(positions, 3);
	bagGeom.setAttribute("position", posAttr);

	// Build index buffer: main surface + side panels
	const indices: number[] = [];
	const stride = LATERAL_N + 1;

	// Main surface quads (two triangles each)
	for (let i = 0; i < PROFILE_N; i++) {
		for (let j = 0; j < LATERAL_N; j++) {
			const a = i * stride + j;
			const b = a + 1;
			const c = (i + 1) * stride + j;
			const d = c + 1;
			indices.push(a, c, d, a, d, b);
		}
	}

	// Left side panel (j = 0 column, fan from first vertex)
	for (let i = 1; i < PROFILE_N; i++) {
		indices.push(0, (i + 1) * stride, i * stride);
	}

	// Right side panel (j = LATERAL_N column, fan from first vertex)
	for (let i = 1; i < PROFILE_N; i++) {
		indices.push(LATERAL_N, i * stride + LATERAL_N, (i + 1) * stride + LATERAL_N);
	}

	bagGeom.setIndex(indices);

	const bagMesh = new THREE.Mesh(bagGeom, bagMat);
	bagMesh.castShadow = true;
	group.add(bagMesh);

	/* ═══════════════════════════════════════════════════════════
	   6. DRAWSTRINGS
	   ═══════════════════════════════════════════════════════════ */

	// Inner drawstring cord (fixed on body, along inner clip line)
	const innerCord = new THREE.Mesh(
		new THREE.CylinderGeometry(0.0015, 0.0015, BAG_HW * 2, 6),
		cordMat,
	);
	innerCord.position.set(INNER_CLIP_X, 0, INNER_CLIP_Z);
	group.add(innerCord);

	// Outer drawstring cord (on frame, along far rim)
	const outerCord = new THREE.Mesh(
		new THREE.CylinderGeometry(0.0015, 0.0015, BAG_HW * 2, 6),
		cordMat,
	);
	outerCord.position.set(FRAME_D, 0, 0);
	hingeGroup.add(outerCord);

	// Retention nubs (drawstring anchor points on frame corners)
	const nubGeom = new THREE.CylinderGeometry(0.003, 0.003, 0.006, 6);
	for (const ys of [-1, 1]) {
		const nub = new THREE.Mesh(nubGeom, frameMat);
		nub.position.set(FRAME_D, ys * BAG_HW, 0.005);
		hingeGroup.add(nub);
	}

	/* ═══════════════════════════════════════════════════════════
	   7. ANIMATION — Fold Cycle
	   ═══════════════════════════════════════════════════════════
	   Phase 1: Open hold   (0   – 3.0s)
	   Phase 2: Fold in     (3.0 – 4.5s)
	   Phase 3: Closed hold (4.5 – 5.5s)
	   Phase 4: Fold out    (5.5 – 7.0s)
	   Total cycle: 7 seconds                                    */

	const CYCLE = 7;
	const T1 = 3;
	const T2 = 4.5;
	const T3 = 5.5;

	function ease(t: number) {
		return t * t * (3 - 2 * t);
	}

	/** Compute outer clip position in world (group) coords for a given hinge angle */
	function outerClipWorld(angle: number) {
		return {
			x: HINGE_X + FRAME_D * Math.cos(angle),
			z: HINGE_Z - FRAME_D * Math.sin(angle),
		};
	}

	/** Recompute all bag mesh vertices for the current hinge angle */
	function refreshBag(angle: number) {
		const o = outerClipWorld(angle);
		const dx = o.x - INNER_CLIP_X;
		const dz = o.z - INNER_CLIP_Z;
		const chord = Math.sqrt(dx * dx + dz * dz);

		// Fixed bag arc length → sag via parabolic approximation
		const sagSq = (3 / 8) * (BAG_ARC * BAG_ARC - chord * chord);
		const sag = Math.min(sagSq > 0 ? Math.sqrt(sagSq) : 0.02, 0.3);

		for (let i = 0; i <= PROFILE_N; i++) {
			const t = i / PROFILE_N;
			const profileSag = 4 * t * (1 - t); // parabolic: max at t=0.5

			const px = INNER_CLIP_X + dx * t;
			const baseZ = INNER_CLIP_Z + dz * t;

			for (let j = 0; j <= LATERAL_N; j++) {
				const s = j / LATERAL_N;
				const y = -BAG_HW + 2 * BAG_HW * s;

				// Lateral sag gives the bag pouch-like volume
				const lateralSag = 4 * s * (1 - s);
				const pz =
					baseZ - sag * profileSag - sag * 0.25 * lateralSag * profileSag;

				const vi = (i * stride + j) * 3;
				positions[vi] = px;
				positions[vi + 1] = y;
				positions[vi + 2] = pz;
			}
		}

		posAttr.needsUpdate = true;
		bagGeom.computeVertexNormals();
		bagGeom.computeBoundingSphere();
	}

	// Initialize bag at the open position
	refreshBag(OPEN_ANGLE);

	/** Called each frame with elapsed time (seconds) from THREE.Clock */
	function update(time: number) {
		const ct = time % CYCLE;
		let angle: number;

		if (ct < T1) {
			// Phase 1: hold open
			angle = OPEN_ANGLE;
		} else if (ct < T2) {
			// Phase 2: fold in (open → closed)
			const t = ease((ct - T1) / (T2 - T1));
			angle = OPEN_ANGLE + (CLOSED_ANGLE - OPEN_ANGLE) * t;
		} else if (ct < T3) {
			// Phase 3: hold closed
			angle = CLOSED_ANGLE;
		} else {
			// Phase 4: fold out (closed → open)
			const t = ease((ct - T3) / (CYCLE - T3));
			angle = CLOSED_ANGLE + (OPEN_ANGLE - CLOSED_ANGLE) * t;
		}

		hingeGroup.rotation.y = angle;
		refreshBag(angle);
	}

	return { group, hingeGroup, update };
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
	return new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.15, 0.12), mat);
}
