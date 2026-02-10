// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useEffect, useRef, useState } from "react";
import type { ViewMode } from "./types";

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export default function ThreeDRobotViewerPage() {
	const [viewMode, setViewMode] = useState<ViewMode>("full");

	return (
		<div className="min-h-screen bg-cw-dark">
			<div className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-7xl flex-wrap items-center justify-between gap-2 px-6 py-3">
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
						<ViewToggle mode={viewMode} onChange={setViewMode} />
						<div className="ml-2 h-4 w-px bg-white/20" />
						<div className="h-2 w-2 animate-pulse rounded-full bg-cw-green" />
						<span className="font-mono text-xs text-cw-green">
							Interactive
						</span>
					</div>
				</div>
			</div>

			<div className="lg:grid lg:grid-cols-4">
				<div className="lg:col-span-3">
					<RobotViewer viewMode={viewMode} />
				</div>
				<div className="border-t border-white/10 bg-black/20 lg:border-l lg:border-t-0">
					<RobotSpecs viewMode={viewMode} />
				</div>
			</div>
		</div>
	);
}

// ---------------------------------------------------------------------------
// View Toggle
// ---------------------------------------------------------------------------

const VIEW_MODES: { key: ViewMode; label: string }[] = [
	{ key: "full", label: "Full Robot" },
	{ key: "body", label: "Robot Body" },
	{ key: "bag", label: "Bag System" },
];

function ViewToggle({
	mode,
	onChange,
}: {
	mode: ViewMode;
	onChange: (m: ViewMode) => void;
}) {
	return (
		<div className="flex rounded-lg border border-white/10 bg-black/40 p-0.5">
			{VIEW_MODES.map((m) => (
				<button
					key={m.key}
					onClick={() => onChange(m.key)}
					className={`rounded-md px-2.5 py-1 font-mono text-xs transition-all ${
						mode === m.key
							? "bg-cw-green/20 text-cw-green"
							: "text-gray-400 hover:text-white"
					}`}
				>
					{m.label}
				</button>
			))}
		</div>
	);
}

// ---------------------------------------------------------------------------
// 3D Viewer
// ---------------------------------------------------------------------------

interface SceneParts {
	bodyGroup: import("three").Group;
	bagGroup: import("three").Group;
	ghostBody: import("three").Mesh;
}

function RobotViewer({ viewMode }: { viewMode: ViewMode }) {
	const containerRef = useRef<HTMLDivElement>(null);
	const [loading, setLoading] = useState(true);
	const partsRef = useRef<SceneParts | null>(null);
	const viewModeRef = useRef(viewMode);
	viewModeRef.current = viewMode;

	// Update visibility on mode change
	useEffect(() => {
		const p = partsRef.current;
		if (!p) return;
		p.bodyGroup.visible = viewMode !== "bag";
		p.bagGroup.visible = viewMode !== "body";
		p.ghostBody.visible = viewMode === "bag";
	}, [viewMode]);

	// Initialize scene
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
			const { buildRobotBody } = await import("./build-robot-body");
			const { buildBagSystem, buildGhostBody } = await import(
				"./build-bag-system"
			);

			if (disposed) return;

			// Scene
			const scene = new THREE.Scene();
			scene.background = new THREE.Color(0xd4d4d4);
			scene.fog = new THREE.FogExp2(0xd4d4d4, 0.15);

			// Camera
			const camera = new THREE.PerspectiveCamera(
				45,
				container.clientWidth / Math.max(container.clientHeight, 1),
				0.01,
				50,
			);
			camera.position.set(1.2, 0.8, 1.2);

			// Renderer
			renderer = new THREE.WebGLRenderer({ antialias: true });
			renderer.setSize(container.clientWidth, container.clientHeight);
			renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
			renderer.shadowMap.enabled = true;
			renderer.shadowMap.type = THREE.PCFSoftShadowMap;
			renderer.toneMapping = THREE.ACESFilmicToneMapping;
			renderer.toneMappingExposure = 1.6;
			container.appendChild(renderer.domElement);

			// Controls
			const controls = new OrbitControls(camera, renderer.domElement);
			controls.enableDamping = true;
			controls.dampingFactor = 0.05;
			controls.autoRotate = true;
			controls.autoRotateSpeed = 0.5;
			controls.minDistance = 0.5;
			controls.maxDistance = 4;
			controls.target.set(0, 0.35, 0);
			controls.maxPolarAngle = Math.PI * 0.85;
			controls.update();

			// Lighting
			scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.5));

			const dirLight = new THREE.DirectionalLight(0xffffff, 2.0);
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

			const fillLight = new THREE.DirectionalLight(0xddeeff, 0.8);
			fillLight.position.set(-3, 2, -3);
			scene.add(fillLight);

			const rimLight = new THREE.PointLight(0x22c55e, 0.6, 4);
			rimLight.position.set(0, 0.6, -1.2);
			scene.add(rimLight);

			const bottomFill = new THREE.DirectionalLight(0xffffff, 0.4);
			bottomFill.position.set(0, -1, 2);
			scene.add(bottomFill);

			// Ground
			const ground = new THREE.Mesh(
				new THREE.PlaneGeometry(12, 12),
				new THREE.MeshStandardMaterial({
					color: 0xbcbcbc,
					roughness: 0.9,
					metalness: 0.05,
				}),
			);
			ground.rotation.x = -Math.PI / 2;
			ground.receiveShadow = true;
			scene.add(ground);

			const grid = new THREE.GridHelper(5, 30, 0x999999, 0xaaaaaa);
			grid.position.y = 0.001;
			scene.add(grid);

			// Build robot
			const bodyResult = buildRobotBody(THREE);
			const bagResult = buildBagSystem(THREE);
			const ghostBody = buildGhostBody(THREE);

			const robot = new THREE.Group();
			robot.add(bodyResult.group);
			robot.add(bagResult.group);
			robot.add(ghostBody);

			// Z-up to Y-up
			robot.rotation.x = -Math.PI / 2;
			const baseY = 0.48;
			robot.position.y = baseY;
			scene.add(robot);

			// Store refs
			partsRef.current = {
				bodyGroup: bodyResult.group,
				bagGroup: bagResult.group,
				ghostBody,
			};

			// Apply initial visibility
			const mode = viewModeRef.current;
			bodyResult.group.visible = mode !== "bag";
			bagResult.group.visible = mode !== "body";
			ghostBody.visible = mode === "bag";

			if (disposed) return;
			setLoading(false);

			// Animation
			const clock = new THREE.Clock();
			const animate = () => {
				if (disposed) return;
				animationId = requestAnimationFrame(animate);
				const t = clock.getElapsedTime();

				// Idle breathing
				robot.position.y = baseY + Math.sin(t * 1.5) * 0.004;

				// Leg micro-movements
				for (const leg of bodyResult.legs) {
					leg.hipPitch.rotation.y =
						leg.baseHipPitch +
						Math.sin(t * 1.2 + leg.phase) * 0.02;
					leg.knee.rotation.y =
						leg.baseKneePitch +
						Math.sin(t * 1.5 + leg.phase + 0.5) * 0.015;
				}

				// Arm subtle sway
				bodyResult.arm.shoulderPitch.rotation.y =
					bodyResult.arm.baseShoulderPitch +
					Math.sin(t * 0.8) * 0.03;
				bodyResult.arm.elbowPitch.rotation.y =
					bodyResult.arm.baseElbowPitch +
					Math.sin(t * 0.6 + 0.5) * 0.02;

				// LED glow pulse
				bodyResult.ledMaterial.emissiveIntensity =
					0.35 + Math.sin(t * 2) * 0.2;

				controls.update();
				renderer!.render(scene, camera);
			};
			animate();

			// Resize
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
				if (container.contains(renderer.domElement))
					container.removeChild(renderer.domElement);
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
						Drag to rotate &bull; Scroll to zoom &bull; Right-drag
						to pan
					</p>
				</div>
			)}
		</div>
	);
}

// ---------------------------------------------------------------------------
// Specs sidebar
// ---------------------------------------------------------------------------

function RobotSpecs({ viewMode }: { viewMode: ViewMode }) {
	return (
		<div className="space-y-6 p-6">
			<div>
				<h2 className="text-lg font-semibold text-white">
					CleanWalker CW-1
				</h2>
				<p className="mt-1 text-sm text-gray-400">
					{viewMode === "bag"
						? "V2 Bag System — Folding Frame + Roll Dispenser"
						: viewMode === "body"
							? "Robot Body — Quadruped Platform + Arm"
							: "Autonomous Litter-Collecting Quadruped"}
				</p>
			</div>

			{(viewMode === "full" || viewMode === "body") && (
				<>
					<SpecList
						items={[
							{ label: "Body (L×W×H)", value: "450 × 300 × 150 mm" },
							{ label: "Standing Height", value: "~550-600 mm" },
							{ label: "Upper Leg", value: "200 mm" },
							{ label: "Lower Leg", value: "200 mm" },
							{ label: "Ground Clearance", value: "~350 mm" },
							{ label: "DOF", value: "12 (legs) + 5 (arm)" },
							{ label: "Arm Reach", value: "~500 mm" },
						]}
					/>

					<InfoCard title="Joint Ranges">
						<div className="grid grid-cols-2 gap-y-2 text-xs">
							<span className="text-gray-400">Hip Yaw</span>
							<span className="font-mono text-white">
								&plusmn;28.6&deg;
							</span>
							<span className="text-gray-400">Hip Pitch</span>
							<span className="font-mono text-white">
								&plusmn;90.0&deg;
							</span>
							<span className="text-gray-400">Knee (Front)</span>
							<span className="font-mono text-white">
								&minus;5.7&deg; to 149&deg;
							</span>
							<span className="text-gray-400">Knee (Rear)</span>
							<span className="font-mono text-white">
								&minus;149&deg; to 5.7&deg;
							</span>
						</div>
					</InfoCard>
				</>
			)}

			{(viewMode === "full" || viewMode === "bag") && (
				<>
					<SpecList
						items={[
							{ label: "Bag Capacity", value: "30-50 L" },
							{ label: "Roll Width", value: "280 mm" },
							{ label: "Roll Diameter", value: "80 mm" },
							{ label: "Frame Width", value: "300 mm" },
							{ label: "Frame Depth", value: "220 mm" },
							{ label: "Frame Angle", value: "135° open" },
							{ label: "Moving Parts", value: "1 (frame servo)" },
						]}
					/>

					<InfoCard title="Bag Swap Cycle">
						<ol className="list-inside list-decimal space-y-1.5 text-xs text-gray-400">
							<li>
								<strong className="text-white">Fold In</strong>{" "}
								— Frame closes, full bag drops by gravity
							</li>
							<li>
								<strong className="text-white">Clip</strong> —
								Frame clips new bag edge from roll
							</li>
							<li>
								<strong className="text-white">
									Fold Out
								</strong>{" "}
								— Frame opens to 135°, new bag ready
							</li>
						</ol>
					</InfoCard>

					<InfoCard title="Gravity Seal">
						<p className="text-xs leading-relaxed text-gray-400">
							Drawstring bags cinch closed under their own weight
							when released — no extra actuator needed. The sealed
							bag drops behind the robot for curbside collection.
						</p>
					</InfoCard>
				</>
			)}

			<InfoCard title="Materials">
				<div className="space-y-2">
					{[
						{ color: "#3B4A3F", label: "Body & Legs — Olive Green" },
						{ color: "#2A2A2A", label: "Joint Housings — Dark Grey" },
						{ color: "#1A1A1A", label: "Frame & Roll — Black Anodized" },
						{ color: "#22C55E", label: "LEDs & Eyes — Status Green" },
						{ color: "#1A1A1A", label: "Feet — Rubber Black" },
					].map((m) => (
						<div key={m.label} className="flex items-center gap-2">
							<div
								className="h-3 w-3 rounded-sm border border-white/10"
								style={{ backgroundColor: m.color }}
							/>
							<span className="text-xs text-gray-400">
								{m.label}
							</span>
						</div>
					))}
				</div>
			</InfoCard>

			<p className="text-xs text-gray-600">
				Drag to rotate &bull; Scroll to zoom &bull; Right-click to pan
			</p>
		</div>
	);
}

function SpecList({ items }: { items: { label: string; value: string }[] }) {
	return (
		<div className="space-y-3">
			{items.map((s) => (
				<div
					key={s.label}
					className="flex items-center justify-between border-b border-white/5 pb-2"
				>
					<span className="text-xs uppercase tracking-wider text-gray-500">
						{s.label}
					</span>
					<span className="font-mono text-sm text-white">
						{s.value}
					</span>
				</div>
			))}
		</div>
	);
}

function InfoCard({
	title,
	children,
}: {
	title: string;
	children: React.ReactNode;
}) {
	return (
		<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
			<h3 className="mb-2 text-sm font-medium text-white">{title}</h3>
			{children}
		</div>
	);
}
