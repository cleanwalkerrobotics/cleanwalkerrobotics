// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useEffect, useRef, useState } from "react";

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export default function ThreeDRobotViewerPage() {
	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Header bar */}
			<div className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-7xl items-center justify-between px-6 py-3">
					<div className="flex items-center gap-4">
						<a
							href="/demos"
							className="text-sm text-gray-400 transition-colors hover:text-white"
						>
							&larr; Demos
						</a>
						<div className="h-4 w-px bg-white/20" />
						<h1 className="font-mono text-sm font-medium text-white">
							3D Robot Viewer
						</h1>
					</div>
					<div className="flex items-center gap-2">
						<div className="h-2 w-2 animate-pulse rounded-full bg-cw-green" />
						<span className="font-mono text-xs text-cw-green">Interactive</span>
					</div>
				</div>
			</div>

			{/* Content */}
			<div className="lg:grid lg:grid-cols-4">
				<div className="lg:col-span-3">
					<RobotViewer />
				</div>
				<div className="border-t border-white/10 bg-black/20 lg:border-l lg:border-t-0">
					<RobotSpecs />
				</div>
			</div>
		</div>
	);
}

// ---------------------------------------------------------------------------
// 3D Viewer
// ---------------------------------------------------------------------------

function RobotViewer() {
	const containerRef = useRef<HTMLDivElement>(null);
	const [loading, setLoading] = useState(true);

	useEffect(() => {
		const container = containerRef.current;
		if (!container) return;

		let disposed = false;
		let animationId: number;
		let renderer: import("three").WebGLRenderer | null = null;
		let resizeObserver: ResizeObserver | null = null;

		(async () => {
			const THREE = await import("three");
			const { OrbitControls } = await import(
				"three/examples/jsm/controls/OrbitControls.js"
			);

			if (disposed) return;

			// Scene
			const scene = new THREE.Scene();
			scene.background = new THREE.Color(0x0a0f15);
			scene.fog = new THREE.FogExp2(0x0a0f15, 0.25);

			// Camera
			const camera = new THREE.PerspectiveCamera(
				45,
				container.clientWidth / Math.max(container.clientHeight, 1),
				0.01,
				50,
			);
			camera.position.set(1.4, 0.9, 1.4);

			// Renderer
			renderer = new THREE.WebGLRenderer({ antialias: true });
			renderer.setSize(container.clientWidth, container.clientHeight);
			renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
			renderer.shadowMap.enabled = true;
			renderer.shadowMap.type = THREE.PCFSoftShadowMap;
			renderer.toneMapping = THREE.ACESFilmicToneMapping;
			renderer.toneMappingExposure = 1.2;
			container.appendChild(renderer.domElement);

			// Controls
			const controls = new OrbitControls(camera, renderer.domElement);
			controls.enableDamping = true;
			controls.dampingFactor = 0.05;
			controls.autoRotate = true;
			controls.autoRotateSpeed = 0.5;
			controls.minDistance = 0.5;
			controls.maxDistance = 4;
			controls.target.set(0, 0.25, 0);
			controls.maxPolarAngle = Math.PI * 0.85;
			controls.update();

			// ------ Lighting ------
			const hemiLight = new THREE.HemisphereLight(0x1a2030, 0x080c08, 0.7);
			scene.add(hemiLight);

			const dirLight = new THREE.DirectionalLight(0xffffff, 1.4);
			dirLight.position.set(3, 5, 2);
			dirLight.castShadow = true;
			dirLight.shadow.mapSize.width = 1024;
			dirLight.shadow.mapSize.height = 1024;
			dirLight.shadow.camera.near = 0.5;
			dirLight.shadow.camera.far = 12;
			dirLight.shadow.camera.left = -2;
			dirLight.shadow.camera.right = 2;
			dirLight.shadow.camera.top = 2;
			dirLight.shadow.camera.bottom = -2;
			scene.add(dirLight);

			const fillLight = new THREE.DirectionalLight(0x4488ff, 0.3);
			fillLight.position.set(-3, 2, -3);
			scene.add(fillLight);

			const rimLight = new THREE.PointLight(0x22c55e, 0.5, 4);
			rimLight.position.set(0, 0.6, -1.2);
			scene.add(rimLight);

			// ------ Ground ------
			const groundGeom = new THREE.PlaneGeometry(12, 12);
			const groundMat = new THREE.MeshStandardMaterial({
				color: 0x0d1117,
				roughness: 0.95,
				metalness: 0.05,
			});
			const ground = new THREE.Mesh(groundGeom, groundMat);
			ground.rotation.x = -Math.PI / 2;
			ground.receiveShadow = true;
			scene.add(ground);

			const grid = new THREE.GridHelper(5, 30, 0x1a2a1a, 0x111a11);
			grid.position.y = 0.001;
			scene.add(grid);

			// ------ Robot ------
			const robot = buildRobot(THREE);
			scene.add(robot.group);

			if (disposed) return;
			setLoading(false);

			// ------ Animation ------
			const clock = new THREE.Clock();

			const animate = () => {
				if (disposed) return;
				animationId = requestAnimationFrame(animate);

				const t = clock.getElapsedTime();

				// Subtle idle breathing
				robot.group.position.y =
					robot.baseY + Math.sin(t * 1.5) * 0.004;

				// Micro leg movements
				for (const leg of robot.legs) {
					leg.hipPitch.rotation.y =
						leg.baseHipPitch +
						Math.sin(t * 1.2 + leg.phase) * 0.02;
					leg.knee.rotation.y =
						leg.baseKneePitch +
						Math.sin(t * 1.5 + leg.phase + 0.5) * 0.015;
				}

				// LED glow pulse
				robot.ledMaterial.emissiveIntensity =
					0.35 + Math.sin(t * 2) * 0.2;

				controls.update();
				renderer!.render(scene, camera);
			};

			animate();

			// ------ Resize ------
			resizeObserver = new ResizeObserver(() => {
				if (disposed || !renderer) return;
				const w = container.clientWidth;
				const h = container.clientHeight;
				if (w === 0 || h === 0) return;
				camera.aspect = w / h;
				camera.updateProjectionMatrix();
				renderer.setSize(w, h);
			});
			resizeObserver.observe(container);
		})();

		return () => {
			disposed = true;
			if (animationId!) cancelAnimationFrame(animationId);
			resizeObserver?.disconnect();
			if (renderer) {
				renderer.dispose();
				if (container.contains(renderer.domElement)) {
					container.removeChild(renderer.domElement);
				}
			}
		};
	}, []);

	return (
		<div className="relative">
			<div
				ref={containerRef}
				className="aspect-square w-full lg:aspect-auto lg:h-[calc(100vh-7rem)]"
			/>
			{loading && (
				<div className="absolute inset-0 flex items-center justify-center bg-cw-dark">
					<div className="text-center">
						<div className="mx-auto mb-4 h-12 w-12 animate-spin rounded-full border-2 border-white/10 border-t-cw-green" />
						<p className="font-mono text-sm text-gray-400">
							Loading 3D model&hellip;
						</p>
					</div>
				</div>
			)}
			{!loading && (
				<div className="pointer-events-none absolute bottom-4 left-4 rounded-lg bg-black/60 px-3 py-2 backdrop-blur-sm">
					<p className="font-mono text-xs text-gray-400">
						Drag to rotate &bull; Scroll to zoom &bull; Right-drag to pan
					</p>
				</div>
			)}
		</div>
	);
}

// ---------------------------------------------------------------------------
// Specs sidebar
// ---------------------------------------------------------------------------

function RobotSpecs() {
	const specs = [
		{ label: "Type", value: "Quadruped" },
		{ label: "Degrees of Freedom", value: "12 DOF" },
		{ label: "Target Mass", value: "~46 kg" },
		{ label: "Body Dimensions", value: "600 x 250 x 120 mm" },
		{ label: "Upper Leg", value: "250 mm" },
		{ label: "Lower Leg", value: "250 mm" },
		{ label: "Standing Height", value: "~500 mm" },
		{ label: "Joints per Leg", value: "3 (yaw, pitch, knee)" },
	];

	return (
		<div className="space-y-6 p-6">
			<div>
				<h2 className="text-lg font-semibold text-white">
					CleanWalker CW-1
				</h2>
				<p className="mt-1 text-sm text-gray-400">
					12-DOF Autonomous Litter-Collecting Quadruped
				</p>
			</div>

			{/* Spec list */}
			<div className="space-y-3">
				{specs.map((s) => (
					<div
						key={s.label}
						className="flex items-center justify-between border-b border-white/5 pb-2"
					>
						<span className="text-xs uppercase tracking-wider text-gray-500">
							{s.label}
						</span>
						<span className="font-mono text-sm text-white">{s.value}</span>
					</div>
				))}
			</div>

			{/* Joint ranges */}
			<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
				<h3 className="mb-2 text-sm font-medium text-white">
					Joint Ranges
				</h3>
				<div className="grid grid-cols-2 gap-y-2 text-xs">
					<span className="text-gray-400">Hip Yaw</span>
					<span className="font-mono text-white">&plusmn;28.6&deg;</span>
					<span className="text-gray-400">Hip Pitch</span>
					<span className="font-mono text-white">&plusmn;90.0&deg;</span>
					<span className="text-gray-400">Knee (Front)</span>
					<span className="font-mono text-white">
						&minus;5.7&deg; to 149&deg;
					</span>
					<span className="text-gray-400">Knee (Rear)</span>
					<span className="font-mono text-white">
						&minus;149&deg; to 5.7&deg;
					</span>
				</div>
			</div>

			{/* Materials legend */}
			<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
				<h3 className="mb-3 text-sm font-medium text-white">
					Materials
				</h3>
				<div className="space-y-2">
					{[
						{ color: "#3B4A3F", label: "Body & Legs — Olive Green" },
						{ color: "#D98C26", label: "Joint Housings — Orange Amber" },
						{ color: "#333333", label: "Mounts — Dark Grey" },
						{ color: "#22C55E", label: "LEDs & Eyes — Status Green" },
						{ color: "#1A1A1A", label: "Feet — Rubber Black" },
					].map((m) => (
						<div key={m.color} className="flex items-center gap-2">
							<div
								className="h-3 w-3 rounded-sm"
								style={{ backgroundColor: m.color }}
							/>
							<span className="text-xs text-gray-400">{m.label}</span>
						</div>
					))}
				</div>
			</div>

			{/* Description */}
			<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
				<h3 className="mb-2 text-sm font-medium text-white">
					About This Model
				</h3>
				<p className="text-xs leading-relaxed text-gray-400">
					This interactive 3D model is rendered directly from our engineering
					URDF specification &mdash; 23 links and 22 joints, with accurate
					dimensions and materials. The robot features a mammalian quadruped
					stance with front knees bending forward and rear knees bending
					backward for optimal stability across all terrain types.
				</p>
			</div>

			<p className="text-xs text-gray-600">
				Drag to rotate &bull; Scroll to zoom &bull; Right-click to pan
			</p>
		</div>
	);
}

// ---------------------------------------------------------------------------
// Robot builder — creates Three.js scene graph from URDF specifications
// ---------------------------------------------------------------------------

interface LegRef {
	hipPitch: import("three").Group;
	knee: import("three").Group;
	baseHipPitch: number;
	baseKneePitch: number;
	phase: number;
}

function buildRobot(THREE: typeof import("three")) {
	// -- Materials --
	const oliveGreen = new THREE.MeshStandardMaterial({
		color: 0x3b4a3f,
		roughness: 0.8,
		metalness: 0.15,
	});
	const orangeAmber = new THREE.MeshStandardMaterial({
		color: 0xd98c26,
		roughness: 0.5,
		metalness: 0.3,
	});
	const darkGrey = new THREE.MeshStandardMaterial({
		color: 0x333333,
		roughness: 0.7,
		metalness: 0.2,
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

	// Root group — built in URDF coordinates (Z-up), then rotated to Y-up
	const robot = new THREE.Group();
	const legs: LegRef[] = [];

	// ---------- Body (base_link) ----------
	const bodyMesh = new THREE.Mesh(
		new THREE.BoxGeometry(0.60, 0.25, 0.12),
		oliveGreen,
	);
	bodyMesh.castShadow = true;
	robot.add(bodyMesh);

	// Slight raised panel on top for visual interest
	const bodyTop = new THREE.Mesh(
		new THREE.BoxGeometry(0.54, 0.22, 0.02),
		oliveGreen,
	);
	bodyTop.position.set(0, 0, 0.07);
	bodyTop.castShadow = true;
	robot.add(bodyTop);

	// ---------- LED strips ----------
	const ledGeom = new THREE.BoxGeometry(0.40, 0.012, 0.016);
	const ledLeft = new THREE.Mesh(ledGeom, ledGreen);
	ledLeft.position.set(0, 0.131, 0);
	robot.add(ledLeft);
	const ledRight = new THREE.Mesh(ledGeom, ledGreen);
	ledRight.position.set(0, -0.131, 0);
	robot.add(ledRight);

	// ---------- Eyes (camera windows) ----------
	const eyeGeom = new THREE.BoxGeometry(0.012, 0.04, 0.04);
	const eyeLeft = new THREE.Mesh(eyeGeom, ledGreen);
	eyeLeft.position.set(0.301, 0.06, 0.02);
	robot.add(eyeLeft);
	const eyeRight = new THREE.Mesh(eyeGeom, ledGreen);
	eyeRight.position.set(0.301, -0.06, 0.02);
	robot.add(eyeRight);

	// ---------- Arm mount (top-front) ----------
	const armMount = new THREE.Mesh(
		new THREE.CylinderGeometry(0.03, 0.03, 0.04, 16),
		darkGrey,
	);
	// URDF cylinders are Z-aligned; Three.js cylinders are Y-aligned — rotate
	armMount.rotation.x = Math.PI / 2;
	armMount.position.set(0.20, 0, 0.08);
	armMount.castShadow = true;
	robot.add(armMount);

	// ---------- Bag frame mount (top-rear) ----------
	const bagMount = new THREE.Mesh(
		new THREE.BoxGeometry(0.10, 0.10, 0.02),
		darkGrey,
	);
	bagMount.position.set(-0.15, 0, 0.07);
	bagMount.castShadow = true;
	robot.add(bagMount);

	// ---------- Legs ----------
	const legConfigs: {
		origin: [number, number, number];
		pitchOffset: [number, number, number];
		isRear: boolean;
		phase: number;
	}[] = [
		{
			origin: [0.25, 0.125, -0.03],
			pitchOffset: [0, 0.04, 0],
			isRear: false,
			phase: 0,
		},
		{
			origin: [0.25, -0.125, -0.03],
			pitchOffset: [0, -0.04, 0],
			isRear: false,
			phase: Math.PI / 2,
		},
		{
			origin: [-0.25, 0.125, -0.03],
			pitchOffset: [0, 0.04, 0],
			isRear: true,
			phase: Math.PI,
		},
		{
			origin: [-0.25, -0.125, -0.03],
			pitchOffset: [0, -0.04, 0],
			isRear: true,
			phase: Math.PI * 1.5,
		},
	];

	for (const cfg of legConfigs) {
		const legGroup = new THREE.Group();
		legGroup.position.set(...cfg.origin);

		// Hip yaw link (orange joint housing cylinder)
		const hipYaw = new THREE.Group();
		const hipMesh = new THREE.Mesh(
			new THREE.CylinderGeometry(0.04, 0.04, 0.08, 16),
			orangeAmber,
		);
		hipMesh.rotation.x = Math.PI / 2;
		hipMesh.castShadow = true;
		hipYaw.add(hipMesh);

		// Hip pitch group — contains upper leg
		const hipPitch = new THREE.Group();
		hipPitch.position.set(...cfg.pitchOffset);

		const upperLeg = new THREE.Mesh(
			new THREE.CylinderGeometry(0.035, 0.035, 0.25, 16),
			oliveGreen,
		);
		upperLeg.rotation.x = Math.PI / 2;
		upperLeg.position.set(0, 0, -0.125);
		upperLeg.castShadow = true;
		hipPitch.add(upperLeg);

		// Small orange sphere at the knee joint for visual detail
		const kneeJointVis = new THREE.Mesh(
			new THREE.SphereGeometry(0.038, 12, 12),
			orangeAmber,
		);
		kneeJointVis.position.set(0, 0, -0.25);
		kneeJointVis.castShadow = true;
		hipPitch.add(kneeJointVis);

		// Knee group — contains lower leg + foot
		const knee = new THREE.Group();
		knee.position.set(0, 0, -0.25);

		const lowerLeg = new THREE.Mesh(
			new THREE.CylinderGeometry(0.03, 0.03, 0.25, 16),
			oliveGreen,
		);
		lowerLeg.rotation.x = Math.PI / 2;
		lowerLeg.position.set(0, 0, -0.125);
		lowerLeg.castShadow = true;
		knee.add(lowerLeg);

		const foot = new THREE.Mesh(
			new THREE.SphereGeometry(0.025, 16, 16),
			rubberBlack,
		);
		foot.position.set(0, 0, -0.25);
		foot.castShadow = true;
		knee.add(foot);

		// Standing pose — mammalian stance
		const baseHipPitch = cfg.isRear ? -0.12 : 0.12;
		const baseKneePitch = cfg.isRear ? -0.24 : 0.24;
		hipPitch.rotation.y = baseHipPitch;
		knee.rotation.y = baseKneePitch;

		// Assemble
		hipPitch.add(knee);
		hipYaw.add(hipPitch);
		legGroup.add(hipYaw);
		robot.add(legGroup);

		legs.push({ hipPitch, knee, baseHipPitch, baseKneePitch, phase: cfg.phase });
	}

	// Convert URDF Z-up to Three.js Y-up
	robot.rotation.x = -Math.PI / 2;

	// Position so feet touch ground plane at y = 0
	// With pose angles ~0.12/0.24 rad, foot bottom ≈ 0.545 below body center
	const baseY = 0.545;
	robot.position.y = baseY;

	return { group: robot, legs, ledMaterial: ledGreen, baseY };
}
