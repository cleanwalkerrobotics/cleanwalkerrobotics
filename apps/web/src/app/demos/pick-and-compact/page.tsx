// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// --- Phase definitions ---

interface Phase {
	id: string;
	label: string;
	description: string;
	durationMs: number;
}

const PHASES: Phase[] = [
	{
		id: "detect",
		label: "Detect",
		description:
			"YOLO v3.2 identifies a plastic bottle at 2.3m — 97% confidence. Bounding box locked, classification confirmed against 50+ litter categories.",
		durationMs: 2500,
	},
	{
		id: "approach",
		label: "Approach",
		description:
			"Path planner generates approach vector. Robot advances at 0.6 m/s, adjusting heading with visual servoing to center the target in the gripper's workspace.",
		durationMs: 3000,
	},
	{
		id: "grip",
		label: "Grip",
		description:
			"Mechanical gripper with silicone-tipped fingers descends and closes adaptively. Force sensors detect contact at 2.1N per finger — gentle enough for thin cans, firm enough for wet bottles.",
		durationMs: 2500,
	},
	{
		id: "lift",
		label: "Lift",
		description:
			"Arm retracts with the item secured. Accelerometer confirms stable hold — no slip detected. Wrist positions over the open collection bag on the X-frame.",
		durationMs: 2000,
	},
	{
		id: "compact",
		label: "Deposit",
		description:
			"Arm positions over the open collection bag held by the X-frame. Item released into bag. The Bag Cassette System tracks fill level — bag 34% full.",
		durationMs: 3000,
	},
	{
		id: "deposit",
		label: "Compress",
		description:
			"Gripper arm presses down into the collection bag, compressing items to maximize capacity. Bag fill updated: 34%. System ready — scanning for next target.",
		durationMs: 2000,
	},
];

// --- Stats ---

interface StatConfig {
	label: string;
	unit: string;
	baseValue: number;
	fluctuation: number;
	decimals: number;
}

const STAT_CONFIGS: StatConfig[] = [
	{ label: "Grip Force", unit: "N", baseValue: 6.3, fluctuation: 0.8, decimals: 1 },
	{ label: "Compression", unit: "%", baseValue: 70, fluctuation: 5, decimals: 0 },
	{ label: "Bag Fill", unit: "%", baseValue: 34, fluctuation: 0, decimals: 0 },
	{ label: "Items / Hour", unit: "items", baseValue: 210, fluctuation: 15, decimals: 0 },
];

// --- Component ---

export default function PickAndCompactPage() {
	const [phaseIndex, setPhaseIndex] = useState(0);
	const [phaseProgress, setPhaseProgress] = useState(0);
	const [cycleCount, setCycleCount] = useState(0);
	const [totalItems, setTotalItems] = useState(0);
	const [isRunning, setIsRunning] = useState(true);
	const [stats, setStats] = useState(() =>
		STAT_CONFIGS.map((s) => s.baseValue),
	);

	const animRef = useRef<number>(0);
	const phaseStartRef = useRef(0);
	const lastStatsRef = useRef(0);

	const currentPhase = PHASES[phaseIndex];

	// Main animation loop
	useEffect(() => {
		if (!isRunning) return;

		phaseStartRef.current = performance.now();
		lastStatsRef.current = 0;

		const animate = (timestamp: number) => {
			const elapsed = timestamp - phaseStartRef.current;
			const duration = PHASES[phaseIndex].durationMs;
			const progress = Math.min(elapsed / duration, 1);

			setPhaseProgress(progress);

			// Update stats occasionally
			if (timestamp - lastStatsRef.current > 500) {
				lastStatsRef.current = timestamp;
				setStats(
					STAT_CONFIGS.map((s) => {
						const jitter = (Math.random() - 0.5) * 2 * s.fluctuation;
						return Math.max(0, s.baseValue + jitter);
					}),
				);
			}

			if (progress >= 1) {
				// Advance to next phase
				const nextIndex = (phaseIndex + 1) % PHASES.length;
				if (nextIndex === 0) {
					setCycleCount((c) => c + 1);
					setTotalItems((t) => t + 1);
					// Increment bag fill level
					setStats((prev) =>
						prev.map((v, i) =>
							STAT_CONFIGS[i].label === "Bag Fill"
								? Math.min(100, v + 2)
								: v,
						),
					);
				}
				setPhaseIndex(nextIndex);
				phaseStartRef.current = timestamp;
			}

			animRef.current = requestAnimationFrame(animate);
		};

		animRef.current = requestAnimationFrame(animate);
		return () => cancelAnimationFrame(animRef.current);
	}, [isRunning, phaseIndex]);

	const toggleRunning = useCallback(() => {
		setIsRunning((prev) => !prev);
	}, []);

	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Inline keyframes */}
			<style>{`
				@keyframes pulse-ring {
					0% { transform: scale(1); opacity: 0.6; }
					100% { transform: scale(2.2); opacity: 0; }
				}
				@keyframes arm-descend {
					0% { transform: translateY(-30px); }
					100% { transform: translateY(0px); }
				}
				@keyframes arm-lift {
					0% { transform: translateY(0px); }
					100% { transform: translateY(-30px); }
				}
				@keyframes compact-press {
					0% { transform: scaleY(1); }
					50% { transform: scaleY(0.3); }
					100% { transform: scaleY(0.3); }
				}
				@keyframes item-slide {
					0% { transform: translateX(0); opacity: 1; }
					100% { transform: translateX(40px); opacity: 0; }
				}
				@keyframes scan-sweep {
					0% { transform: rotate(-30deg); opacity: 0.3; }
					50% { opacity: 0.8; }
					100% { transform: rotate(30deg); opacity: 0.3; }
				}
				@keyframes robot-advance {
					0% { transform: translateX(-20px); }
					100% { transform: translateX(0px); }
				}
				@keyframes gripper-close {
					0% { gap: 20px; }
					100% { gap: 4px; }
				}
			`}</style>

			{/* Header */}
			<header className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-[1600px] items-center justify-between px-6 py-3">
					<div className="flex items-center gap-3">
						<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
							<svg
								className="h-4 w-4 text-cw-green"
								viewBox="0 0 24 24"
								fill="none"
								stroke="currentColor"
								strokeWidth={2}
							>
								<path d="M12 2L12 8M12 8L8 6M12 8L16 6" />
								<path d="M5 12H19" />
								<rect x="8" y="14" width="8" height="6" rx="1" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">
								Pick &amp; Compact
							</h1>
							<p className="font-mono text-[10px] text-gray-500">
								Full litter collection cycle simulation
							</p>
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
								{isRunning && (
									<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
								)}
								<span
									className={`relative inline-flex h-2 w-2 rounded-full ${isRunning ? "bg-cw-green" : "bg-gray-500"}`}
								/>
							</span>
							<span
								className={`font-mono text-xs ${isRunning ? "text-cw-green" : "text-gray-500"}`}
							>
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
					<svg
						className="h-3 w-3"
						fill="none"
						viewBox="0 0 24 24"
						stroke="currentColor"
					>
						<path
							strokeLinecap="round"
							strokeLinejoin="round"
							strokeWidth={2}
							d="M15 19l-7-7 7-7"
						/>
					</svg>
					Back to Demos
				</a>

				{/* Main Layout: Visualization + Stats */}
				<div className="mb-8 grid gap-6 lg:grid-cols-4">
					{/* Visualization Area */}
					<div className="lg:col-span-3">
						<div
							className="relative w-full overflow-hidden rounded-xl border border-white/10"
							style={{ height: 500, backgroundColor: "#0d1117" }}
						>
							{/* Phase Steps (top bar) */}
							<div className="absolute left-0 right-0 top-0 z-10 flex items-center gap-0 border-b border-white/10 bg-black/60 px-4 py-3 backdrop-blur-sm">
								{PHASES.map((phase, i) => {
									const isActive = i === phaseIndex;
									const isDone = i < phaseIndex || (phaseIndex === 0 && cycleCount > 0 && i > 0 ? false : i < phaseIndex);
									return (
										<div key={phase.id} className="flex flex-1 items-center">
											<div className="flex flex-1 flex-col items-center">
												<div
													className={`flex h-7 w-7 items-center justify-center rounded-full text-[10px] font-bold transition-all duration-300 ${
														isActive
															? "bg-cw-green text-white shadow-lg shadow-cw-green/30"
															: isDone
																? "bg-cw-green/20 text-cw-green"
																: "bg-white/10 text-gray-500"
													}`}
												>
													{isDone && !isActive ? (
														<svg className="h-3 w-3" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={3}>
															<path strokeLinecap="round" strokeLinejoin="round" d="M5 13l4 4L19 7" />
														</svg>
													) : (
														i + 1
													)}
												</div>
												<span
													className={`mt-1 font-mono text-[9px] uppercase tracking-wider ${
														isActive ? "text-cw-green" : "text-gray-500"
													}`}
												>
													{phase.label}
												</span>
											</div>
											{i < PHASES.length - 1 && (
												<div
													className={`mx-1 h-px flex-shrink-0 transition-colors duration-300 ${
														i < phaseIndex ? "bg-cw-green/40" : "bg-white/10"
													}`}
													style={{ width: 20 }}
												/>
											)}
										</div>
									);
								})}
							</div>

							{/* Phase progress bar */}
							<div className="absolute left-0 right-0 top-[72px] z-10 px-4">
								<div className="h-1 overflow-hidden rounded-full bg-white/10">
									<div
										className="h-full rounded-full bg-cw-green transition-[width] duration-100"
										style={{ width: `${phaseProgress * 100}%` }}
									/>
								</div>
							</div>

							{/* Animated Scene */}
							<div className="absolute inset-0 flex items-center justify-center pt-20">
								<AnimatedScene phaseId={currentPhase.id} progress={phaseProgress} />
							</div>

							{/* Phase Description */}
							<div className="absolute bottom-0 left-0 right-0 z-10 border-t border-white/10 bg-black/70 px-6 py-4 backdrop-blur-sm">
								<div className="flex items-start gap-4">
									<div className="flex h-8 w-8 flex-shrink-0 items-center justify-center rounded-lg bg-cw-green/20">
										<PhaseIcon phaseId={currentPhase.id} />
									</div>
									<div className="min-w-0">
										<p className="font-mono text-xs font-bold uppercase tracking-wider text-cw-green">
											{currentPhase.label}
										</p>
										<p className="mt-1 text-sm leading-relaxed text-gray-400">
											{currentPhase.description}
										</p>
									</div>
								</div>
							</div>

							{/* Cycle counter */}
							<div className="absolute right-4 top-[84px] z-10 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] text-gray-400">
									Cycle #{cycleCount + 1}
								</span>
							</div>
						</div>
					</div>

					{/* Stats Panel */}
					<div className="lg:col-span-1">
						<div className="flex flex-col gap-4">
							{/* Live Stats */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-4 flex items-center gap-2">
									<span className="relative flex h-2 w-2">
										{isRunning && (
											<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
										)}
										<span className={`relative inline-flex h-2 w-2 rounded-full ${isRunning ? "bg-cw-green" : "bg-gray-500"}`} />
									</span>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
										Gripper Telemetry
									</span>
								</div>

								{STAT_CONFIGS.map((config, i) => (
									<div key={config.label} className={i < STAT_CONFIGS.length - 1 ? "mb-4" : ""}>
										<p className="text-[10px] uppercase tracking-wider text-gray-500">
											{config.label}
										</p>
										<div className="mt-1 flex items-center gap-3">
											<p className="font-mono text-2xl font-bold text-white">
												{stats[i].toFixed(config.decimals)}
												<span className="ml-1 text-sm text-gray-500">{config.unit}</span>
											</p>
											{config.label === "Bag Fill" && (
												<div className="h-2 flex-1 overflow-hidden rounded-full bg-white/10">
													<div
														className="h-full rounded-full bg-cw-green transition-all duration-300"
														style={{ width: `${stats[i]}%` }}
													/>
												</div>
											)}
										</div>
									</div>
								))}
							</div>

							{/* Session Stats */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-3 flex items-center gap-2">
									<svg className="h-3.5 w-3.5 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
										<path strokeLinecap="round" strokeLinejoin="round" d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
									</svg>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
										Session
									</span>
								</div>
								<div className="space-y-3">
									<div className="flex items-center justify-between">
										<span className="text-sm text-gray-500">Cycles completed</span>
										<span className="font-mono text-sm font-bold text-white">{cycleCount}</span>
									</div>
									<div className="flex items-center justify-between">
										<span className="text-sm text-gray-500">Items collected</span>
										<span className="font-mono text-sm font-bold text-cw-green">{totalItems}</span>
									</div>
									<div className="flex items-center justify-between">
										<span className="text-sm text-gray-500">Current phase</span>
										<span className="font-mono text-sm font-bold text-cw-green">{currentPhase.label}</span>
									</div>
									<div className="flex items-center justify-between">
										<span className="text-sm text-gray-500">Avg cycle time</span>
										<span className="font-mono text-sm font-bold text-white">15.0s</span>
									</div>
								</div>
							</div>
						</div>
					</div>
				</div>

				{/* Technical Explanation */}
				<section className="mb-8">
					<h2 className="mb-6 text-2xl font-bold text-white">
						Gripper System
					</h2>
					<div className="grid gap-6 md:grid-cols-3">
						{/* 3-Finger Mechanical Gripper */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M12 2v6M8 5l4 3 4-3" />
									<path d="M7 11c0 0-2 2-2 5s3 5 7 5 7-2 7-5-2-5-2-5" />
									<circle cx="12" cy="16" r="2" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								3-Finger Mechanical Gripper
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								Three steel fingers with silicone grip pads and embedded strain
								gauges provide precise yet gentle grasping across irregular
								shapes. The 120-degree symmetric layout ensures stable
								tri-point contact on bottles, cans, wrappers, and organic
								debris. The silicone-tipped fingers grip firmly without
								crushing thin aluminum.
							</p>
						</div>

						{/* Adaptive Grip Control */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M13 10V3L4 14h7v7l9-11h-7z" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								Adaptive Grip Control
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								A closed-loop PID controller adjusts finger pressure in real-time
								based on object classification. Rigid items (glass bottles) get
								firm 8N grip; deformable items (paper cups) get gentle 3N contact.
								Slip detection via vibration sensors triggers automatic re-grip
								in under 50ms — maintaining hold on wet or irregular surfaces.
							</p>
						</div>

						{/* Bag Cassette & Compression */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<rect x="4" y="4" width="16" height="16" rx="2" />
									<path d="M4 12h16M9 4v16M15 4v16" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								Bag Cassette &amp; Compression
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								The Bag Cassette System uses replaceable cartridges containing
								20-30 collection bags. An X-shaped raising frame holds each bag
								open during collection. The gripper arm periodically presses down
								to compress contents. When a bag is full, a snap-close mechanism
								seals it, the frame lowers, and the robot drops the sealed bag
								at the nearest curb for municipal pickup. The frame rises with a
								fresh bag from the cassette and collection continues.
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
									["Gripper type", "2-3 finger adaptive mechanical"],
									["Finger material", "Steel fingers with silicone grip pads and strain gauges"],
									["Max grip force", "12N per finger (36N total)"],
									["Grip response time", "< 200ms close, < 50ms re-grip"],
									["Compression", "Gripper arm press-down into bag"],
									["Bag Cassette", "20-30 bags per cartridge"],
									["Curb Drop", "Sealed bags at municipal collection points"],
									["Items per charge", "200+ across mixed litter types"],
									["Integration", "Works with existing waste pickup infrastructure"],
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

				{/* Footer Note */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						This is a simulated demo. In production, gripper telemetry streams from
						deployed CleanWalker robots via onboard force sensors and ROS2 control nodes.
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

// --- Phase Icons ---

function PhaseIcon({ phaseId }: { phaseId: string }) {
	const cls = "h-4 w-4 text-cw-green";
	switch (phaseId) {
		case "detect":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<circle cx="11" cy="11" r="8" />
					<path d="M21 21l-4.35-4.35" />
				</svg>
			);
		case "approach":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path strokeLinecap="round" strokeLinejoin="round" d="M13 7l5 5m0 0l-5 5m5-5H6" />
				</svg>
			);
		case "grip":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M12 2v6M8 5l4 3 4-3M7 12h10M9 12v6M15 12v6" />
				</svg>
			);
		case "lift":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path strokeLinecap="round" strokeLinejoin="round" d="M5 10l7-7m0 0l7 7m-7-7v18" />
				</svg>
			);
		case "compact":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<rect x="6" y="6" width="12" height="12" rx="1" />
					<path d="M6 12h12M12 6v12" />
				</svg>
			);
		case "deposit":
			return (
				<svg className={cls} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path strokeLinecap="round" strokeLinejoin="round" d="M19 14l-7 7m0 0l-7-7m7 7V3" />
				</svg>
			);
		default:
			return null;
	}
}

// --- Animated Scene ---

function AnimatedScene({ phaseId, progress }: { phaseId: string; progress: number }) {
	return (
		<div className="relative flex h-72 w-full max-w-lg items-end justify-center">
			{/* Ground line */}
			<div className="absolute bottom-8 left-4 right-4 h-px bg-white/10" />
			<div className="absolute bottom-6 left-4 right-4 font-mono text-[8px] uppercase tracking-widest text-white/10">
				ground plane
			</div>

			{/* Scene content based on phase */}
			{phaseId === "detect" && <DetectScene progress={progress} />}
			{phaseId === "approach" && <ApproachScene progress={progress} />}
			{phaseId === "grip" && <GripScene progress={progress} />}
			{phaseId === "lift" && <LiftScene progress={progress} />}
			{phaseId === "compact" && <CompactScene progress={progress} />}
			{phaseId === "deposit" && <DepositScene progress={progress} />}
		</div>
	);
}

// --- Individual Phase Scenes ---

function LitterItem({ x, bottom, scale }: { x: string; bottom: number; scale?: number }) {
	return (
		<div
			className="absolute"
			style={{ left: x, bottom, transform: `scale(${scale ?? 1})` }}
		>
			{/* Bottle shape */}
			<div className="flex flex-col items-center">
				<div className="h-3 w-4 rounded-t-sm bg-blue-400/80" />
				<div className="h-8 w-6 rounded-b-sm bg-blue-400/60 shadow-lg shadow-blue-400/20" />
			</div>
		</div>
	);
}

function RobotBody({ x, bottom }: { x: string; bottom: number }) {
	return (
		<div className="absolute" style={{ left: x, bottom }}>
			<div className="flex flex-col items-center">
				{/* Head / sensor array */}
				<div className="mb-1 h-4 w-8 rounded-t-md bg-gray-600 shadow-inner">
					<div className="mx-auto mt-1 h-1.5 w-4 rounded-full bg-cw-green/60" />
				</div>
				{/* Body */}
				<div className="h-12 w-12 rounded-md bg-gray-700 shadow-lg">
					<div className="mx-auto mt-2 h-2 w-8 rounded-full bg-cw-green/30" />
					<div className="mx-auto mt-1 h-1 w-6 rounded-full bg-cw-green/20" />
				</div>
				{/* Legs */}
				<div className="flex gap-2 pt-1">
					<div className="h-6 w-2 rounded-b-sm bg-gray-600" />
					<div className="h-6 w-2 rounded-b-sm bg-gray-600" />
					<div className="h-6 w-2 rounded-b-sm bg-gray-600" />
					<div className="h-6 w-2 rounded-b-sm bg-gray-600" />
				</div>
			</div>
		</div>
	);
}

function GripperArm({ y, open, hasItem }: { y: number; open: boolean; hasItem: boolean }) {
	return (
		<div className="absolute left-1/2 -translate-x-1/2" style={{ top: y }}>
			<div className="flex flex-col items-center">
				{/* Arm shaft */}
				<div className="h-10 w-2 bg-gray-500" />
				{/* Wrist joint */}
				<div className="h-3 w-4 rounded-sm bg-gray-600" />
				{/* Fingers */}
				<div
					className="flex items-start transition-all duration-500"
					style={{ gap: open ? 20 : 4 }}
				>
					<div className="h-6 w-2 rounded-b-md bg-cw-green/80" style={{ transform: open ? "rotate(-15deg)" : "rotate(0deg)", transition: "transform 0.5s" }} />
					<div className="h-7 w-2 rounded-b-md bg-cw-green/80" />
					<div className="h-6 w-2 rounded-b-md bg-cw-green/80" style={{ transform: open ? "rotate(15deg)" : "rotate(0deg)", transition: "transform 0.5s" }} />
				</div>
				{/* Item in gripper */}
				{hasItem && !open && (
					<div className="absolute bottom-0 flex flex-col items-center">
						<div className="h-2 w-3 rounded-t-sm bg-blue-400/80" />
						<div className="h-5 w-4 rounded-b-sm bg-blue-400/60" />
					</div>
				)}
			</div>
		</div>
	);
}

function DetectScene({ progress }: { progress: number }) {
	const scanAngle = Math.sin(progress * Math.PI * 3) * 30;
	return (
		<>
			<RobotBody x="25%" bottom={32} />
			<LitterItem x="65%" bottom={32} />

			{/* Scanning beam */}
			<div
				className="absolute left-[32%] top-[35%] h-px w-32 origin-left bg-gradient-to-r from-cw-green/80 to-transparent"
				style={{ transform: `rotate(${scanAngle}deg)`, transition: "transform 0.1s" }}
			/>

			{/* Detection box (appears as progress increases) */}
			{progress > 0.3 && (
				<div
					className="absolute border-2 border-cw-green/60 transition-opacity duration-300"
					style={{
						left: "61%",
						bottom: 28,
						width: 40,
						height: 52,
						opacity: Math.min(1, (progress - 0.3) * 3),
					}}
				>
					<span
						className="absolute -top-5 left-0 whitespace-nowrap rounded bg-cw-green/80 px-1.5 py-0.5 font-mono text-[9px] text-white"
						style={{ opacity: progress > 0.5 ? 1 : 0, transition: "opacity 0.3s" }}
					>
						bottle — 97%
					</span>
				</div>
			)}

			{/* Radar pulse */}
			{progress > 0.1 && progress < 0.8 && (
				<div
					className="absolute rounded-full border border-cw-green/30"
					style={{
						left: "28%",
						top: "38%",
						width: 60 + progress * 100,
						height: 60 + progress * 100,
						transform: "translate(-50%, -50%)",
						opacity: Math.max(0, 1 - progress),
					}}
				/>
			)}
		</>
	);
}

function ApproachScene({ progress }: { progress: number }) {
	const robotX = 25 + progress * 25;
	return (
		<>
			<RobotBody x={`${robotX}%`} bottom={32} />
			<LitterItem x="65%" bottom={32} />

			{/* Distance indicator */}
			<div
				className="absolute bottom-16 font-mono text-[10px] text-gray-500"
				style={{ left: `${robotX + 8}%` }}
			>
				{(2.3 * (1 - progress)).toFixed(1)}m
			</div>

			{/* Approach path (dashed line) */}
			<svg className="absolute inset-0" viewBox="0 0 400 300">
				<line
					x1={robotX * 4 + 24}
					y1={240}
					x2={268}
					y2={240}
					stroke="#22c55e"
					strokeWidth="1"
					strokeDasharray="4,4"
					opacity="0.4"
				/>
			</svg>

			{/* Target reticle */}
			<div
				className="absolute"
				style={{ left: "65%", bottom: 48, transform: "translate(-50%, 50%)" }}
			>
				<div className="h-8 w-8 rounded-full border border-cw-green/40">
					<div className="absolute left-1/2 top-0 h-full w-px bg-cw-green/20" />
					<div className="absolute left-0 top-1/2 h-px w-full bg-cw-green/20" />
				</div>
			</div>
		</>
	);
}

function GripScene({ progress }: { progress: number }) {
	const armY = 20 + progress * 60;
	const isClosing = progress > 0.6;
	return (
		<>
			<RobotBody x="50%" bottom={32} />
			<LitterItem x="65%" bottom={32} />

			{/* Gripper arm descending */}
			<GripperArm y={armY} open={!isClosing} hasItem={false} />

			{/* Force readout */}
			{isClosing && (
				<div className="absolute right-[15%] top-[30%] rounded bg-black/60 px-2 py-1 font-mono text-[10px] text-cw-green backdrop-blur-sm">
					F: {((progress - 0.6) * 15.75).toFixed(1)}N
				</div>
			)}
		</>
	);
}

function LiftScene({ progress }: { progress: number }) {
	const armY = 80 - progress * 60;
	return (
		<>
			<RobotBody x="50%" bottom={32} />

			{/* Gripper arm lifting with item */}
			<GripperArm y={armY} open={false} hasItem={true} />

			{/* Stability indicator */}
			<div className="absolute right-[15%] top-[30%] rounded bg-black/60 px-2 py-1 backdrop-blur-sm">
				<div className="flex items-center gap-1.5">
					<span className="h-1.5 w-1.5 rounded-full bg-cw-green" />
					<span className="font-mono text-[10px] text-cw-green">STABLE</span>
				</div>
			</div>
		</>
	);
}

function CompactScene({ progress }: { progress: number }) {
	const itemFall = Math.min(1, progress * 2);
	return (
		<>
			<RobotBody x="35%" bottom={32} />

			{/* X-frame with open bag */}
			<div className="absolute right-[22%] bottom-[32px]">
				<div className="relative flex flex-col items-center">
					{/* X-frame arms */}
					<div className="flex items-end gap-0">
						<div className="h-6 w-1 -rotate-12 bg-gray-500" />
						<div className="h-6 w-1 rotate-12 bg-gray-500" />
					</div>
					{/* Open bag */}
					<div className="h-16 w-14 rounded-b-lg border-b-2 border-l-2 border-r-2 border-cw-green/40 bg-cw-green/10">
						{/* Fill level */}
						<div className="absolute bottom-0 left-0 right-0 rounded-b-lg bg-cw-green/20" style={{ height: "34%" }} />
						<div className="absolute inset-0 flex items-center justify-center">
							<span className="font-mono text-[8px] text-gray-500">BAG</span>
						</div>
					</div>
				</div>
			</div>

			{/* Falling item into bag */}
			{progress < 0.8 && (
				<div
					className="absolute right-[25%] transition-all duration-100"
					style={{
						top: `${15 + itemFall * 40}%`,
						opacity: Math.max(0, 1 - progress * 1.5),
					}}
				>
					<div className="flex flex-col items-center">
						<div className="h-2 w-3 rounded-t-sm bg-blue-400/80" />
						<div className="h-5 w-4 rounded-b-sm bg-blue-400/60" />
					</div>
				</div>
			)}

			{/* Status readout */}
			{progress > 0.5 && (
				<div className="absolute right-[10%] top-[20%] rounded bg-black/60 px-2 py-1 font-mono text-[10px] text-cw-green backdrop-blur-sm">
					BAG: 34%
				</div>
			)}
		</>
	);
}

function DepositScene({ progress }: { progress: number }) {
	const pressDown = Math.min(1, progress * 1.5);
	return (
		<>
			<RobotBody x="35%" bottom={32} />

			{/* Bag with items */}
			<div className="absolute right-[22%] bottom-[32px]">
				<div className="relative flex flex-col items-center">
					{/* X-frame arms */}
					<div className="flex items-end gap-0">
						<div className="h-6 w-1 -rotate-12 bg-gray-500" />
						<div className="h-6 w-1 rotate-12 bg-gray-500" />
					</div>
					{/* Bag */}
					<div className="h-16 w-14 rounded-b-lg border-b-2 border-l-2 border-r-2 border-cw-green/40 bg-cw-green/10">
						{/* Fill level — compresses as gripper pushes */}
						<div
							className="absolute bottom-0 left-0 right-0 rounded-b-lg bg-cw-green/30 transition-all duration-200"
							style={{ height: `${34 + pressDown * 10}%` }}
						/>
						<div className="absolute inset-0 flex items-center justify-center">
							<span className="font-mono text-[8px] text-gray-500">BAG</span>
						</div>
					</div>
				</div>
			</div>

			{/* Gripper arm pressing down */}
			<div
				className="absolute right-[24%] transition-all duration-200"
				style={{ top: `${20 + pressDown * 30}%` }}
			>
				<div className="flex flex-col items-center">
					<div className="h-8 w-2 bg-gray-500" />
					<div className="h-3 w-6 rounded-b-md bg-cw-green/80" />
				</div>
			</div>

			{/* Status */}
			{progress > 0.5 && (
				<div className="absolute left-[15%] top-[25%] rounded bg-black/60 px-3 py-2 backdrop-blur-sm">
					<p className="font-mono text-[10px] text-cw-green">COMPRESSED</p>
					<p className="font-mono text-[9px] text-gray-500">Bag: 34% full</p>
					<p className="mt-1 font-mono text-[9px] text-gray-500">Scanning...</p>
				</div>
			)}
		</>
	);
}
