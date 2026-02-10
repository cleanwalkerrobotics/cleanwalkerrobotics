// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// --- Terrain definitions ---

interface Terrain {
	id: string;
	label: string;
	gaitMode: string;
	color: string;
	speed: number;
	stability: number;
	incline: number;
	stanceWidth: number; // relative, 1 = normal
	stepHeight: number; // relative, 1 = normal
	gaitPeriod: number; // ms per full gait cycle
	widthPx: number;
}

const TERRAINS: Terrain[] = [
	{
		id: "grass",
		label: "Grass",
		gaitMode: "Trot",
		color: "#2d5a3a",
		speed: 1.2,
		stability: 96,
		incline: 0,
		stanceWidth: 1,
		stepHeight: 1,
		gaitPeriod: 600,
		widthPx: 400,
	},
	{
		id: "gravel",
		label: "Gravel",
		gaitMode: "Crawl",
		color: "#6b5b4f",
		speed: 0.8,
		stability: 89,
		incline: 0,
		stanceWidth: 1.3,
		stepHeight: 0.7,
		gaitPeriod: 900,
		widthPx: 350,
	},
	{
		id: "curb",
		label: "Curb Step-Up",
		gaitMode: "Step-Over",
		color: "#555",
		speed: 0.3,
		stability: 78,
		incline: 45,
		stanceWidth: 1.1,
		stepHeight: 2.0,
		gaitPeriod: 1400,
		widthPx: 200,
	},
	{
		id: "mud",
		label: "Mud",
		gaitMode: "High-Step",
		color: "#5a4632",
		speed: 0.5,
		stability: 82,
		incline: 0,
		stanceWidth: 1.2,
		stepHeight: 1.5,
		gaitPeriod: 1000,
		widthPx: 350,
	},
	{
		id: "stairs",
		label: "Stairs",
		gaitMode: "Stair-Climb",
		color: "#4a4a4a",
		speed: 0.2,
		stability: 72,
		incline: 35,
		stanceWidth: 1.0,
		stepHeight: 2.5,
		gaitPeriod: 1800,
		widthPx: 300,
	},
	{
		id: "grass2",
		label: "Grass",
		gaitMode: "Trot",
		color: "#2d5a3a",
		speed: 1.2,
		stability: 96,
		incline: 0,
		stanceWidth: 1,
		stepHeight: 1,
		gaitPeriod: 600,
		widthPx: 400,
	},
];

const TOTAL_TERRAIN_WIDTH = TERRAINS.reduce((sum, t) => sum + t.widthPx, 0);

function getTerrainAtOffset(offset: number): Terrain {
	const wrapped = ((offset % TOTAL_TERRAIN_WIDTH) + TOTAL_TERRAIN_WIDTH) % TOTAL_TERRAIN_WIDTH;
	let cumulative = 0;
	for (const terrain of TERRAINS) {
		cumulative += terrain.widthPx;
		if (wrapped < cumulative) return terrain;
	}
	return TERRAINS[0];
}

// --- Component ---

export default function QuadrupedalLocomotionPage() {
	const [terrainOffset, setTerrainOffset] = useState(0);
	const [currentTerrain, setCurrentTerrain] = useState<Terrain>(TERRAINS[0]);
	const [gaitPhase, setGaitPhase] = useState(0);
	const [isRunning, setIsRunning] = useState(true);
	const [telemetry, setTelemetry] = useState({
		speed: 1.2,
		stability: 96,
		incline: 0,
		pitch: 0,
		roll: 0,
	});

	const animRef = useRef<number>(0);
	const lastTimeRef = useRef(0);

	useEffect(() => {
		if (!isRunning) return;

		lastTimeRef.current = 0;

		const animate = (timestamp: number) => {
			if (!lastTimeRef.current) lastTimeRef.current = timestamp;
			const delta = (timestamp - lastTimeRef.current) / 1000;
			lastTimeRef.current = timestamp;

			setTerrainOffset((prev) => {
				const newOffset = prev + currentTerrain.speed * 60 * delta;
				const terrain = getTerrainAtOffset(newOffset + 300);
				setCurrentTerrain(terrain);
				return newOffset;
			});

			setGaitPhase((prev) => {
				const increment = (delta * 1000) / currentTerrain.gaitPeriod;
				return (prev + increment) % 1;
			});

			setTelemetry({
				speed: currentTerrain.speed + (Math.random() - 0.5) * 0.1,
				stability: currentTerrain.stability + (Math.random() - 0.5) * 3,
				incline: currentTerrain.incline + (Math.random() - 0.5) * 2,
				pitch: (Math.random() - 0.5) * 4,
				roll: (Math.random() - 0.5) * 2,
			});

			animRef.current = requestAnimationFrame(animate);
		};

		animRef.current = requestAnimationFrame(animate);
		return () => cancelAnimationFrame(animRef.current);
	}, [isRunning, currentTerrain]);

	const toggleRunning = useCallback(() => {
		setIsRunning((prev) => {
			if (prev) lastTimeRef.current = 0;
			return !prev;
		});
	}, []);

	return (
		<div className="min-h-screen bg-cw-dark">
			<style>{`
				@keyframes body-bob {
					0%, 100% { transform: translateY(0px); }
					50% { transform: translateY(-3px); }
				}
			`}</style>

			{/* Header */}
			<header className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-[1600px] items-center justify-between px-6 py-3">
					<div className="flex items-center gap-3">
						<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
							<svg className="h-4 w-4 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
								<path d="M4 18l4-4M20 18l-4-4M8 14V8M16 14V8M8 8h8M6 8l2-4h8l2 4" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">Quadrupedal Locomotion</h1>
							<p className="font-mono text-[10px] text-gray-500">Adaptive terrain traversal simulation</p>
						</div>
					</div>
					<div className="flex items-center gap-4">
						<button
							onClick={toggleRunning}
							className="rounded-lg border border-white/10 bg-white/5 px-3 py-1.5 font-mono text-xs text-gray-300 transition-colors hover:bg-white/10"
						>
							{isRunning ? "Pause" : "Resume"}
						</button>
						<div className="hidden items-center gap-2 sm:flex">
							<span className="relative flex h-2 w-2">
								{isRunning && <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />}
								<span className={`relative inline-flex h-2 w-2 rounded-full ${isRunning ? "bg-cw-green" : "bg-gray-500"}`} />
							</span>
							<span className={`font-mono text-xs ${isRunning ? "text-cw-green" : "text-gray-500"}`}>
								{isRunning ? "LIVE" : "PAUSED"}
							</span>
						</div>
					</div>
				</div>
			</header>

			<div className="mx-auto max-w-[1600px] px-6 py-6">
				{/* Back link */}
				<a
					href="/demos"
					className="mb-6 inline-flex items-center gap-2 font-mono text-xs text-gray-500 transition-colors hover:text-cw-green"
				>
					<svg className="h-3 w-3" fill="none" viewBox="0 0 24 24" stroke="currentColor">
						<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
					</svg>
					Back to Demos
				</a>

				{/* Main Layout */}
				<div className="mb-8 grid gap-6 lg:grid-cols-4">
					{/* Visualization */}
					<div className="lg:col-span-3">
						<div
							className="relative w-full overflow-hidden rounded-xl border border-white/10"
							style={{ height: 500, backgroundColor: "#0d1117" }}
						>
							{/* Sky gradient */}
							<div className="absolute inset-x-0 top-0 h-[55%] bg-gradient-to-b from-[#0a1628] to-[#0d1117]" />

							{/* Terrain label */}
							<div className="absolute left-4 top-4 z-10 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] uppercase tracking-widest text-gray-400">
									Terrain: <span className="text-cw-green">{currentTerrain.label}</span>
								</span>
							</div>

							{/* Gait mode badge */}
							<div className="absolute right-4 top-4 z-10 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] uppercase tracking-widest text-gray-400">
									Gait: <span className="text-cw-green">{currentTerrain.gaitMode}</span>
								</span>
							</div>

							{/* Terrain strip */}
							<div className="absolute inset-x-0 bottom-0 h-[45%] overflow-hidden">
								<TerrainStrip offset={terrainOffset} />
							</div>

							{/* Robot (fixed at center) */}
							<div className="absolute left-1/2 -translate-x-1/2" style={{ bottom: "45%", transform: "translateX(-50%) translateY(50%)" }}>
								<RobotSilhouette
									gaitPhase={gaitPhase}
									stanceWidth={currentTerrain.stanceWidth}
									stepHeight={currentTerrain.stepHeight}
									isRunning={isRunning}
								/>
							</div>

							{/* Horizon line */}
							<div className="absolute inset-x-0" style={{ top: "55%" }}>
								<div className="h-px w-full bg-white/5" />
							</div>

							{/* Speed indicator */}
							<div className="absolute bottom-4 left-4 z-10 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] text-gray-400">
									{telemetry.speed.toFixed(1)} m/s
								</span>
							</div>

							{/* IMU readout */}
							<div className="absolute bottom-4 right-4 z-10 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] text-gray-500">
									P:{telemetry.pitch.toFixed(1)}&deg; R:{telemetry.roll.toFixed(1)}&deg;
								</span>
							</div>
						</div>
					</div>

					{/* Telemetry Panel */}
					<div className="lg:col-span-1">
						<div className="flex flex-col gap-4">
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-4 flex items-center gap-2">
									<span className="relative flex h-2 w-2">
										{isRunning && <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />}
										<span className={`relative inline-flex h-2 w-2 rounded-full ${isRunning ? "bg-cw-green" : "bg-gray-500"}`} />
									</span>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
										CW-001 Locomotion
									</span>
								</div>

								{/* Terrain Type */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Terrain</p>
									<p className="font-mono text-xl font-bold text-cw-green">{currentTerrain.label}</p>
								</div>

								{/* Gait Mode */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Gait Mode</p>
									<p className="font-mono text-xl font-bold text-white">{currentTerrain.gaitMode}</p>
								</div>

								{/* Stability */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Stability</p>
									<div className="mt-1 flex items-center gap-3">
										<p
											className="font-mono text-2xl font-bold"
											style={{
												color: telemetry.stability > 90 ? "#22c55e" : telemetry.stability > 80 ? "#eab308" : "#ef4444",
											}}
										>
											{telemetry.stability.toFixed(0)}%
										</p>
										<div className="h-2 flex-1 overflow-hidden rounded-full bg-white/10">
											<div
												className="h-full rounded-full transition-all duration-300"
												style={{
													width: `${telemetry.stability}%`,
													backgroundColor:
														telemetry.stability > 90 ? "#22c55e" : telemetry.stability > 80 ? "#eab308" : "#ef4444",
												}}
											/>
										</div>
									</div>
								</div>

								{/* Speed */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Speed</p>
									<p className="font-mono text-2xl font-bold text-white">
										{telemetry.speed.toFixed(1)} <span className="text-sm text-gray-500">m/s</span>
									</p>
								</div>

								{/* Incline */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Incline</p>
									<p className="font-mono text-2xl font-bold text-white">
										{telemetry.incline.toFixed(0)}&deg;
									</p>
								</div>

								{/* IMU */}
								<div>
									<p className="text-[10px] uppercase tracking-wider text-gray-500">IMU</p>
									<div className="mt-1 grid grid-cols-2 gap-2">
										<div className="rounded bg-white/5 px-2 py-1.5 text-center">
											<p className="font-mono text-[9px] text-gray-500">Pitch</p>
											<p className="font-mono text-sm font-bold text-white">{telemetry.pitch.toFixed(1)}&deg;</p>
										</div>
										<div className="rounded bg-white/5 px-2 py-1.5 text-center">
											<p className="font-mono text-[9px] text-gray-500">Roll</p>
											<p className="font-mono text-sm font-bold text-white">{telemetry.roll.toFixed(1)}&deg;</p>
										</div>
									</div>
								</div>
							</div>

							{/* Terrain Legend */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-3 flex items-center gap-2">
									<svg className="h-3.5 w-3.5 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
										<path strokeLinecap="round" strokeLinejoin="round" d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" />
									</svg>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">Terrain Types</span>
								</div>
								<div className="space-y-2">
									{[
										{ label: "Grass", color: "#2d5a3a" },
										{ label: "Gravel", color: "#6b5b4f" },
										{ label: "Curb", color: "#555" },
										{ label: "Mud", color: "#5a4632" },
										{ label: "Stairs", color: "#4a4a4a" },
									].map((t) => (
										<div key={t.label} className="flex items-center gap-2">
											<div className="h-3 w-6 rounded-sm" style={{ backgroundColor: t.color }} />
											<span
												className={`font-mono text-[11px] ${
													currentTerrain.label === t.label ? "font-bold text-cw-green" : "text-gray-500"
												}`}
											>
												{t.label}
												{currentTerrain.label === t.label && " (active)"}
											</span>
										</div>
									))}
								</div>
							</div>
						</div>
					</div>
				</div>

				{/* How It Works */}
				<section className="mb-8">
					<h2 className="mb-6 text-2xl font-bold text-white">Adaptive Gait Control</h2>
					<div className="grid gap-6 md:grid-cols-3">
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">Terrain Classification</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								Stereo depth cameras and contact-force sensors classify terrain type in real-time.
								The system identifies grass, gravel, mud, concrete, stairs, and curbs — adjusting
								gait parameters before the robot steps onto new surfaces. LiDAR point cloud analysis
								detects elevation changes and step heights up to 3m ahead.
							</p>
						</div>

						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M13 10V3L4 14h7v7l9-11h-7z" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">Gait Generation</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								A Central Pattern Generator (CPG) produces five gait modes: trot for flat terrain,
								crawl for unstable surfaces, high-step for mud, stair-climb for steps, and step-over
								for curbs. Each mode adjusts stance width, step height, leg timing, and body posture.
								Transitions between gaits are blended over 2-3 steps for smooth switching.
							</p>
						</div>

						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M3 12h4l3-9 4 18 3-9h4" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">IMU Feedback Loop</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								A 9-axis IMU (accelerometer, gyroscope, magnetometer) at 1kHz feeds a real-time
								balance controller. Pitch and roll corrections are applied within 5ms to maintain
								stability on uneven ground. The system maintains a stability margin above 72% even
								on stairs — if stability drops below threshold, the robot slows or stops to rebalance.
							</p>
						</div>
					</div>
				</section>

				{/* Specs Table */}
				<section className="mb-8">
					<div className="overflow-hidden rounded-xl border border-white/10">
						<table className="w-full">
							<thead>
								<tr className="border-b border-white/10 bg-white/[0.03]">
									<th className="px-6 py-3 text-left font-mono text-xs uppercase tracking-wider text-gray-400">Spec</th>
									<th className="px-6 py-3 text-left font-mono text-xs uppercase tracking-wider text-gray-400">Value</th>
								</tr>
							</thead>
							<tbody className="divide-y divide-white/5">
								{[
									["Leg configuration", "4-leg (mammalian), 3 DOF per leg"],
									["Max step height", "18cm (curbs, stairs)"],
									["Max incline", "35 degrees (stairs), 45 degrees (ramps)"],
									["Gait modes", "5 (trot, crawl, high-step, stair-climb, step-over)"],
									["Terrain types", "Grass, gravel, mud, concrete, stairs, curbs"],
									["IMU rate", "1kHz (9-axis)"],
									["Balance response", "< 5ms correction latency"],
									["Speed range", "0.2 m/s (stairs) — 1.4 m/s (flat grass)"],
									["Stability margin", "> 72% on all supported terrain"],
								].map(([spec, value]) => (
									<tr key={spec} className="transition-colors hover:bg-white/[0.02]">
										<td className="px-6 py-3 text-sm font-medium text-gray-300">{spec}</td>
										<td className="px-6 py-3 font-mono text-sm text-gray-400">{value}</td>
									</tr>
								))}
							</tbody>
						</table>
					</div>
				</section>

				{/* Footer */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						This is a simulated demo. In production, locomotion data streams from deployed
						CleanWalker robots via onboard IMU sensors, joint encoders, and ROS2 control nodes.
					</p>
					<a
						href="/contact"
						className="mt-3 inline-block rounded-lg bg-cw-green/10 px-6 py-2 font-mono text-xs font-medium text-cw-green transition-colors hover:bg-cw-green/20"
					>
						Schedule a Live Demo
					</a>
				</div>
			</div>
		</div>
	);
}

// --- Terrain Strip ---

function TerrainStrip({ offset }: { offset: number }) {
	const segments: { terrain: Terrain; x: number }[] = [];
	let x = -offset % TOTAL_TERRAIN_WIDTH;
	if (x > 0) x -= TOTAL_TERRAIN_WIDTH;

	// Render enough repetitions to fill the viewport
	while (x < 1600) {
		for (const terrain of TERRAINS) {
			segments.push({ terrain, x });
			x += terrain.widthPx;
		}
	}

	return (
		<div className="relative h-full w-full">
			{segments.map((seg, i) => (
				<div
					key={i}
					className="absolute top-0 h-full"
					style={{ left: seg.x, width: seg.terrain.widthPx }}
				>
					{/* Main terrain fill */}
					<div className="h-full w-full" style={{ backgroundColor: seg.terrain.color }}>
						{/* Terrain texture details */}
						{seg.terrain.id === "grass" || seg.terrain.id === "grass2" ? (
							<GrassTexture width={seg.terrain.widthPx} />
						) : seg.terrain.id === "gravel" ? (
							<GravelTexture width={seg.terrain.widthPx} />
						) : seg.terrain.id === "curb" ? (
							<CurbTexture />
						) : seg.terrain.id === "mud" ? (
							<MudTexture width={seg.terrain.widthPx} />
						) : seg.terrain.id === "stairs" ? (
							<StairsTexture width={seg.terrain.widthPx} />
						) : null}
					</div>
					{/* Terrain label */}
					<div className="absolute left-1/2 top-3 -translate-x-1/2">
						<span className="rounded bg-black/40 px-2 py-0.5 font-mono text-[9px] uppercase tracking-widest text-white/40 backdrop-blur-sm">
							{seg.terrain.label}
						</span>
					</div>
				</div>
			))}
		</div>
	);
}

function GrassTexture({ width }: { width: number }) {
	const blades = Array.from({ length: Math.floor(width / 8) }, (_, i) => i);
	return (
		<div className="absolute inset-0 overflow-hidden">
			{blades.map((i) => (
				<div
					key={i}
					className="absolute bottom-0"
					style={{
						left: i * 8 + Math.random() * 4,
						width: 2,
						height: 8 + Math.random() * 12,
						backgroundColor: `hsl(${130 + Math.random() * 20}, ${40 + Math.random() * 20}%, ${20 + Math.random() * 15}%)`,
						borderRadius: "1px 1px 0 0",
						transform: `rotate(${(Math.random() - 0.5) * 20}deg)`,
						transformOrigin: "bottom",
					}}
				/>
			))}
		</div>
	);
}

function GravelTexture({ width }: { width: number }) {
	const stones = Array.from({ length: Math.floor(width / 12) }, (_, i) => i);
	return (
		<div className="absolute inset-0 overflow-hidden">
			{stones.map((i) => (
				<div
					key={i}
					className="absolute rounded-full"
					style={{
						left: i * 12 + Math.random() * 6,
						top: 10 + Math.random() * 80 + "%",
						width: 4 + Math.random() * 6,
						height: 3 + Math.random() * 5,
						backgroundColor: `hsl(30, ${10 + Math.random() * 15}%, ${35 + Math.random() * 25}%)`,
					}}
				/>
			))}
		</div>
	);
}

function CurbTexture() {
	return (
		<div className="absolute inset-0 overflow-hidden">
			{/* Concrete curb surface */}
			<div className="absolute inset-x-0 top-0 h-[40%] bg-[#666]" />
			{/* Step face */}
			<div className="absolute inset-x-0 top-[40%] h-[15%] bg-[#555] shadow-inner" />
			{/* Lower surface */}
			<div className="absolute inset-x-0 top-[55%] h-[45%] bg-[#777]" />
			{/* Edge line */}
			<div className="absolute inset-x-0 top-[40%] h-px bg-white/10" />
		</div>
	);
}

function MudTexture({ width }: { width: number }) {
	const puddles = Array.from({ length: Math.floor(width / 40) }, (_, i) => i);
	return (
		<div className="absolute inset-0 overflow-hidden">
			{puddles.map((i) => (
				<div
					key={i}
					className="absolute rounded-full"
					style={{
						left: i * 40 + Math.random() * 20,
						top: 20 + Math.random() * 60 + "%",
						width: 20 + Math.random() * 30,
						height: 8 + Math.random() * 12,
						backgroundColor: "#4a3a28",
						boxShadow: "inset 0 1px 3px rgba(0,0,0,0.3)",
					}}
				/>
			))}
			{/* Mud splatter dots */}
			{Array.from({ length: Math.floor(width / 15) }, (_, i) => (
				<div
					key={`splat-${i}`}
					className="absolute rounded-full"
					style={{
						left: i * 15 + Math.random() * 8,
						top: Math.random() * 100 + "%",
						width: 3 + Math.random() * 4,
						height: 3 + Math.random() * 4,
						backgroundColor: "#3d2e1e",
					}}
				/>
			))}
		</div>
	);
}

function StairsTexture({ width }: { width: number }) {
	const steps = 6;
	const stepWidth = width / steps;
	return (
		<div className="absolute inset-0 overflow-hidden">
			{Array.from({ length: steps }, (_, i) => (
				<div
					key={i}
					className="absolute border-r border-white/10"
					style={{
						left: i * stepWidth,
						width: stepWidth,
						top: `${(steps - 1 - i) * (100 / steps)}%`,
						height: `${((i + 1) * 100) / steps}%`,
						backgroundColor: i % 2 === 0 ? "#4a4a4a" : "#525252",
					}}
				>
					{/* Step edge highlight */}
					<div className="absolute inset-x-0 top-0 h-px bg-white/15" />
				</div>
			))}
		</div>
	);
}

// --- Robot Silhouette ---

function RobotSilhouette({
	gaitPhase,
	stanceWidth,
	stepHeight,
	isRunning,
}: {
	gaitPhase: number;
	stanceWidth: number;
	stepHeight: number;
	isRunning: boolean;
}) {
	// Quadruped walking gait: diagonal pairs move together (trot)
	// Phase 0-0.5: FL+BR up, FR+BL down
	// Phase 0.5-1: FR+BL up, FL+BR down
	const phase = gaitPhase * Math.PI * 2;
	const baseStance = 10 * stanceWidth;
	const baseStep = 8 * stepHeight;

	// Leg vertical offsets (positive = up)
	const flLift = Math.max(0, Math.sin(phase)) * baseStep;
	const brLift = Math.max(0, Math.sin(phase)) * baseStep;
	const frLift = Math.max(0, Math.sin(phase + Math.PI)) * baseStep;
	const blLift = Math.max(0, Math.sin(phase + Math.PI)) * baseStep;

	// Leg horizontal offsets (for forward/back swing)
	const flSwing = Math.sin(phase) * 4;
	const brSwing = Math.sin(phase) * 4;
	const frSwing = Math.sin(phase + Math.PI) * 4;
	const blSwing = Math.sin(phase + Math.PI) * 4;

	// Body bob
	const bodyBob = isRunning ? Math.abs(Math.sin(phase * 2)) * 2 : 0;

	return (
		<div className="relative" style={{ width: 100, height: 100 }}>
			{/* Body */}
			<div
				className="absolute left-1/2 -translate-x-1/2"
				style={{ top: 20 - bodyBob, transition: "top 0.05s linear" }}
			>
				{/* Head / sensor dome */}
				<div className="relative">
					<div className="absolute -left-2 -top-5 h-5 w-10 rounded-t-md bg-gray-600">
						<div className="absolute left-1 top-1 h-2 w-2 rounded-full bg-cw-green/80" />
						<div className="absolute right-1 top-1 h-2 w-2 rounded-full bg-cw-green/60" />
					</div>
					{/* Main body */}
					<div className="h-10 w-16 rounded-md bg-gray-700 shadow-lg" style={{ marginLeft: -5 }}>
						<div className="absolute left-1/2 top-2 h-1.5 w-8 -translate-x-1/2 rounded-full bg-cw-green/30" />
						<div className="absolute left-1/2 top-5 h-1 w-6 -translate-x-1/2 rounded-full bg-cw-green/20" />
					</div>
				</div>
			</div>

			{/* Front-Left leg */}
			<Leg x={25 - baseStance / 2 + flSwing} y={55 - flLift - bodyBob} />
			{/* Front-Right leg */}
			<Leg x={25 + baseStance / 2 + frSwing} y={55 - frLift - bodyBob} />
			{/* Back-Left leg */}
			<Leg x={55 - baseStance / 2 + blSwing} y={55 - blLift - bodyBob} />
			{/* Back-Right leg */}
			<Leg x={55 + baseStance / 2 + brSwing} y={55 - brLift - bodyBob} />

			{/* Ground shadow */}
			<div
				className="absolute left-1/2 -translate-x-1/2 rounded-full bg-black/30"
				style={{ bottom: 0, width: 60, height: 6 }}
			/>
		</div>
	);
}

function Leg({ x, y }: { x: number; y: number }) {
	return (
		<div
			className="absolute"
			style={{ left: x, top: y, transition: "top 0.05s linear, left 0.05s linear" }}
		>
			{/* Upper leg */}
			<div className="h-8 w-3 rounded-sm bg-gray-600" />
			{/* Lower leg */}
			<div className="ml-0.5 h-6 w-2 rounded-b-sm bg-gray-500" />
			{/* Foot */}
			<div className="-ml-0.5 h-1.5 w-4 rounded-b-md bg-gray-400" />
		</div>
	);
}
