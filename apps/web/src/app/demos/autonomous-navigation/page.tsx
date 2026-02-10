// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// --- Path & Map Data ---

interface Point {
	x: number;
	y: number;
}

interface LitterItem {
	id: number;
	x: number;
	y: number;
	collected: boolean;
}

interface ObstacleEvent {
	id: number;
	time: string;
	message: string;
}

const ZONE_NAMES = [
	"Main Promenade",
	"Lakeside Path",
	"Playground Loop",
	"Picnic Meadow",
	"East Garden",
	"Fountain Plaza",
];

// Waypoints the robot follows — a winding park path
const PATH_WAYPOINTS: Point[] = [
	{ x: 8, y: 75 },
	{ x: 15, y: 60 },
	{ x: 22, y: 45 },
	{ x: 30, y: 38 },
	{ x: 42, y: 35 },
	{ x: 55, y: 30 },
	{ x: 65, y: 28 },
	{ x: 75, y: 32 },
	{ x: 82, y: 42 },
	{ x: 80, y: 55 },
	{ x: 72, y: 62 },
	{ x: 60, y: 65 },
	{ x: 48, y: 68 },
	{ x: 38, y: 72 },
	{ x: 28, y: 78 },
	{ x: 20, y: 85 },
	{ x: 30, y: 88 },
	{ x: 45, y: 82 },
	{ x: 58, y: 78 },
	{ x: 70, y: 75 },
	{ x: 82, y: 70 },
	{ x: 88, y: 60 },
	{ x: 90, y: 48 },
	{ x: 85, y: 35 },
	{ x: 78, y: 22 },
	{ x: 65, y: 15 },
	{ x: 50, y: 12 },
	{ x: 35, y: 18 },
	{ x: 22, y: 25 },
	{ x: 12, y: 35 },
	{ x: 8, y: 50 },
	{ x: 8, y: 75 },
];

// Initial litter positions (near the paths)
function generateLitter(): LitterItem[] {
	return [
		{ id: 1, x: 18, y: 55, collected: false },
		{ id: 2, x: 35, y: 36, collected: false },
		{ id: 3, x: 50, y: 32, collected: false },
		{ id: 4, x: 70, y: 30, collected: false },
		{ id: 5, x: 78, y: 48, collected: false },
		{ id: 6, x: 68, y: 63, collected: false },
		{ id: 7, x: 42, y: 70, collected: false },
		{ id: 8, x: 25, y: 82, collected: false },
		{ id: 9, x: 52, y: 80, collected: false },
		{ id: 10, x: 85, y: 55, collected: false },
		{ id: 11, x: 88, y: 42, collected: false },
		{ id: 12, x: 72, y: 20, collected: false },
		{ id: 13, x: 40, y: 16, collected: false },
		{ id: 14, x: 15, y: 40, collected: false },
		{ id: 15, x: 60, y: 76, collected: false },
	];
}

// Trees and benches as static park features
const TREES: Point[] = [
	{ x: 5, y: 20 },
	{ x: 35, y: 10 },
	{ x: 60, y: 8 },
	{ x: 88, y: 15 },
	{ x: 92, y: 38 },
	{ x: 95, y: 72 },
	{ x: 10, y: 90 },
	{ x: 50, y: 50 },
	{ x: 75, y: 50 },
	{ x: 20, y: 68 },
	{ x: 3, y: 45 },
	{ x: 45, y: 92 },
];

const BENCHES: Point[] = [
	{ x: 28, y: 42 },
	{ x: 62, y: 26 },
	{ x: 76, y: 58 },
	{ x: 38, y: 75 },
	{ x: 86, y: 28 },
];

// --- Helpers ---

function lerp(a: number, b: number, t: number): number {
	return a + (b - a) * t;
}

function dist(a: Point, b: Point): number {
	return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
}

function formatTime(date: Date): string {
	return date.toLocaleTimeString("en-US", {
		hour: "2-digit",
		minute: "2-digit",
		second: "2-digit",
		hour12: false,
	});
}

function getZoneName(pos: Point): string {
	if (pos.y < 30) return "East Garden";
	if (pos.x < 30 && pos.y < 60) return "Main Promenade";
	if (pos.x > 70 && pos.y < 60) return "Fountain Plaza";
	if (pos.y > 75) return "Lakeside Path";
	if (pos.x < 50) return "Playground Loop";
	return "Picnic Meadow";
}

// --- Main Page ---

export default function AutonomousNavigationPage() {
	const [robotPos, setRobotPos] = useState<Point>(PATH_WAYPOINTS[0]);
	const [trail, setTrail] = useState<Point[]>([PATH_WAYPOINTS[0]]);
	const [litter, setLitter] = useState<LitterItem[]>(generateLitter);
	const [speed, setSpeed] = useState(1.0);
	const [battery, setBattery] = useState(87);
	const [itemsCollected, setItemsCollected] = useState(0);
	const [distanceTraveled, setDistanceTraveled] = useState(0);
	const [obstacleEvents, setObstacleEvents] = useState<ObstacleEvent[]>([]);
	const [isRunning, setIsRunning] = useState(true);

	const animRef = useRef<number>(0);
	const waypointIndexRef = useRef(0);
	const segmentProgressRef = useRef(0);
	const lastTimeRef = useRef(0);
	const eventIdRef = useRef(0);
	const trailCounterRef = useRef(0);

	// Robot movement animation
	useEffect(() => {
		if (!isRunning) return;

		const animate = (timestamp: number) => {
			if (!lastTimeRef.current) lastTimeRef.current = timestamp;
			const delta = (timestamp - lastTimeRef.current) / 1000;
			lastTimeRef.current = timestamp;

			// Speed variation
			const currentSpeed = 0.8 + Math.sin(timestamp / 2000) * 0.2 + Math.random() * 0.2;
			setSpeed(Math.round(currentSpeed * 100) / 100);

			const wpIdx = waypointIndexRef.current;
			const nextIdx = (wpIdx + 1) % PATH_WAYPOINTS.length;
			const from = PATH_WAYPOINTS[wpIdx];
			const to = PATH_WAYPOINTS[nextIdx];
			const segmentDist = dist(from, to);
			const moveRate = (currentSpeed * 3 * delta) / Math.max(segmentDist, 0.1);

			segmentProgressRef.current += moveRate;

			if (segmentProgressRef.current >= 1) {
				segmentProgressRef.current = 0;
				waypointIndexRef.current = nextIdx;
			}

			const t = segmentProgressRef.current;
			const newPos: Point = {
				x: lerp(from.x, to.x, t),
				y: lerp(from.y, to.y, t),
			};

			setRobotPos(newPos);

			// Add trail point every few frames
			trailCounterRef.current++;
			if (trailCounterRef.current % 3 === 0) {
				setTrail((prev) => [...prev.slice(-150), newPos]);
			}

			// Distance
			setDistanceTraveled((prev) =>
				Math.round((prev + currentSpeed * delta * 0.8) * 10) / 10,
			);

			// Litter collection: check if robot is near any uncollected litter
			setLitter((prev) => {
				let collected = false;
				const updated = prev.map((item) => {
					if (!item.collected && dist(newPos, item) < 5) {
						collected = true;
						return { ...item, collected: true };
					}
					return item;
				});
				if (collected) {
					setItemsCollected((c) => c + 1);
				}
				return updated;
			});

			// Occasional obstacle avoidance events
			if (Math.random() < 0.002) {
				eventIdRef.current++;
				const messages = [
					"Pedestrian detected ahead — rerouting left",
					"Dog walker approaching — slowing to 0.4 m/s",
					"Bench obstacle — adjusting path +0.5m",
					"Cyclist incoming — pausing 2s",
					"Uneven terrain — switching to cautious gait",
					"Puddle detected — navigating around",
					"Low branch — ducking posture engaged",
					"Jogger group — yielding right-of-way",
				];
				const msg = messages[Math.floor(Math.random() * messages.length)];
				setObstacleEvents((prev) =>
					[
						{
							id: eventIdRef.current,
							time: formatTime(new Date()),
							message: msg,
						},
						...prev,
					].slice(0, 8),
				);
			}

			animRef.current = requestAnimationFrame(animate);
		};

		animRef.current = requestAnimationFrame(animate);
		return () => cancelAnimationFrame(animRef.current);
	}, [isRunning]);

	// Battery drain
	useEffect(() => {
		if (!isRunning) return;
		const interval = setInterval(() => {
			setBattery((prev) => {
				const drain = Math.random() < 0.3 ? 0.2 : 0.1;
				return Math.round(Math.max(0, prev - drain) * 10) / 10;
			});
		}, 3000);
		return () => clearInterval(interval);
	}, [isRunning]);

	// Reset litter when all collected
	useEffect(() => {
		if (litter.every((l) => l.collected)) {
			const timeout = setTimeout(() => {
				setLitter(generateLitter());
			}, 2000);
			return () => clearTimeout(timeout);
		}
	}, [litter]);

	const toggleRunning = useCallback(() => {
		setIsRunning((prev) => {
			if (prev) {
				lastTimeRef.current = 0;
			}
			return !prev;
		});
	}, []);

	const currentZone = getZoneName(robotPos);
	const remainingLitter = litter.filter((l) => !l.collected).length;

	return (
		<div className="min-h-screen bg-cw-dark">
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
								<circle cx="12" cy="12" r="10" />
								<polygon points="10,8 16,12 10,16" fill="currentColor" stroke="none" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">
								Autonomous Navigation
							</h1>
							<p className="font-mono text-[10px] text-gray-500">
								Real-time park patrol simulation
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

				{/* Main Layout: Map + Telemetry */}
				<div className="mb-8 grid gap-6 lg:grid-cols-4">
					{/* Park Map */}
					<div className="lg:col-span-3">
						<div
							className="relative w-full overflow-hidden rounded-xl border border-white/10"
							style={{ height: 500, backgroundColor: "#0f2918" }}
						>
							{/* Path lines */}
							<svg
								className="absolute inset-0 h-full w-full"
								viewBox="0 0 100 100"
								preserveAspectRatio="none"
							>
								{/* Main walking paths */}
								<polyline
									points={PATH_WAYPOINTS.map((p) => `${p.x},${p.y}`).join(" ")}
									fill="none"
									stroke="#2d5a3a"
									strokeWidth="1.2"
									strokeLinecap="round"
									strokeLinejoin="round"
								/>
								{/* Secondary paths */}
								<line x1="50" y1="50" x2="50" y2="12" stroke="#1e4030" strokeWidth="0.6" />
								<line x1="30" y1="38" x2="8" y2="20" stroke="#1e4030" strokeWidth="0.6" />
								<line x1="82" y1="42" x2="92" y2="38" stroke="#1e4030" strokeWidth="0.6" />
								<line x1="48" y1="68" x2="45" y2="92" stroke="#1e4030" strokeWidth="0.6" />

								{/* Robot trail (dotted) */}
								{trail.length > 1 && (
									<polyline
										points={trail.map((p) => `${p.x},${p.y}`).join(" ")}
										fill="none"
										stroke="#22c55e"
										strokeWidth="0.4"
										strokeDasharray="0.8,0.8"
										opacity="0.5"
									/>
								)}
							</svg>

							{/* Trees */}
							{TREES.map((tree, i) => (
								<div
									key={`tree-${i}`}
									className="absolute -translate-x-1/2 -translate-y-1/2"
									style={{ left: `${tree.x}%`, top: `${tree.y}%` }}
								>
									<div className="flex flex-col items-center">
										<div
											className="rounded-full"
											style={{
												width: 14,
												height: 14,
												backgroundColor: "#1a4d2e",
												boxShadow: "0 0 6px #1a4d2e88",
											}}
										/>
										<div
											className="rounded-sm"
											style={{
												width: 2,
												height: 4,
												backgroundColor: "#3d2b1a",
												marginTop: -1,
											}}
										/>
									</div>
								</div>
							))}

							{/* Benches */}
							{BENCHES.map((bench, i) => (
								<div
									key={`bench-${i}`}
									className="absolute -translate-x-1/2 -translate-y-1/2"
									style={{ left: `${bench.x}%`, top: `${bench.y}%` }}
								>
									<div
										className="rounded-sm"
										style={{
											width: 12,
											height: 5,
											backgroundColor: "#5c3d2e",
											border: "1px solid #7a5240",
										}}
									/>
								</div>
							))}

							{/* Litter dots */}
							{litter.map((item) =>
								!item.collected ? (
									<div
										key={`litter-${item.id}`}
										className="absolute -translate-x-1/2 -translate-y-1/2 transition-opacity duration-300"
										style={{
											left: `${item.x}%`,
											top: `${item.y}%`,
										}}
									>
										<span
											className="block rounded-full"
											style={{
												width: 6,
												height: 6,
												backgroundColor: "#ef4444",
												boxShadow: "0 0 4px #ef444488",
											}}
										/>
									</div>
								) : null,
							)}

							{/* Robot dot */}
							<div
								className="absolute -translate-x-1/2 -translate-y-1/2"
								style={{
									left: `${robotPos.x}%`,
									top: `${robotPos.y}%`,
									zIndex: 20,
								}}
							>
								<span
									className="absolute -inset-3 animate-ping rounded-full bg-cw-green opacity-30"
								/>
								<span
									className="relative block rounded-full bg-cw-green"
									style={{
										width: 12,
										height: 12,
										boxShadow: "0 0 12px #22c55e, 0 0 24px #22c55e44",
									}}
								/>
							</div>

							{/* Zone labels */}
							<span className="absolute left-[8%] top-[12%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								Main Promenade
							</span>
							<span className="absolute left-[60%] top-[8%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								East Garden
							</span>
							<span className="absolute left-[75%] top-[38%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								Fountain Plaza
							</span>
							<span className="absolute left-[25%] top-[55%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								Playground Loop
							</span>
							<span className="absolute left-[55%] top-[55%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								Picnic Meadow
							</span>
							<span className="absolute left-[25%] top-[88%] font-mono text-[9px] uppercase tracking-widest text-white/20">
								Lakeside Path
							</span>

							{/* Map legend */}
							<div className="absolute bottom-3 left-3 flex gap-4 rounded-lg bg-black/60 px-3 py-2 backdrop-blur-sm">
								<div className="flex items-center gap-1.5">
									<span className="h-2 w-2 rounded-full bg-cw-green" />
									<span className="font-mono text-[10px] text-gray-400">Robot</span>
								</div>
								<div className="flex items-center gap-1.5">
									<span className="h-2 w-2 rounded-full bg-red-500" />
									<span className="font-mono text-[10px] text-gray-400">Litter</span>
								</div>
								<div className="flex items-center gap-1.5">
									<span className="h-2 w-2 rounded-full" style={{ backgroundColor: "#1a4d2e" }} />
									<span className="font-mono text-[10px] text-gray-400">Tree</span>
								</div>
								<div className="flex items-center gap-1.5">
									<span className="h-1.5 w-3 rounded-sm" style={{ backgroundColor: "#5c3d2e" }} />
									<span className="font-mono text-[10px] text-gray-400">Bench</span>
								</div>
							</div>

							{/* Map title */}
							<div className="absolute right-3 top-3 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] uppercase tracking-widest text-gray-400">
									Riverside Park — Top View
								</span>
							</div>

							{/* Remaining litter count */}
							<div className="absolute bottom-3 right-3 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
								<span className="font-mono text-[10px] text-red-400">
									{remainingLitter} items remaining
								</span>
							</div>
						</div>
					</div>

					{/* Telemetry Panel */}
					<div className="lg:col-span-1">
						<div className="flex flex-col gap-4">
							{/* Robot Status */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-4 flex items-center gap-2">
									<span className="relative flex h-2 w-2">
										{isRunning && (
											<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
										)}
										<span className={`relative inline-flex h-2 w-2 rounded-full ${isRunning ? "bg-cw-green" : "bg-gray-500"}`} />
									</span>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
										CW-001 Telemetry
									</span>
								</div>

								{/* Speed */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">
										Speed
									</p>
									<p className="font-mono text-2xl font-bold text-white">
										{speed.toFixed(1)}{" "}
										<span className="text-sm text-gray-500">m/s</span>
									</p>
								</div>

								{/* Battery */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">
										Battery
									</p>
									<div className="mt-1 flex items-center gap-3">
										<p
											className="font-mono text-2xl font-bold"
											style={{
												color:
													battery > 50
														? "#22c55e"
														: battery > 20
															? "#eab308"
															: "#ef4444",
											}}
										>
											{battery.toFixed(0)}%
										</p>
										<div className="h-2 flex-1 overflow-hidden rounded-full bg-white/10">
											<div
												className="h-full rounded-full transition-all duration-300"
												style={{
													width: `${battery}%`,
													backgroundColor:
														battery > 50
															? "#22c55e"
															: battery > 20
																? "#eab308"
																: "#ef4444",
												}}
											/>
										</div>
									</div>
								</div>

								{/* Items Collected */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">
										Items Collected
									</p>
									<p className="font-mono text-2xl font-bold text-cw-green">
										{itemsCollected}
									</p>
								</div>

								{/* Distance */}
								<div className="mb-4">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">
										Distance Traveled
									</p>
									<p className="font-mono text-2xl font-bold text-white">
										{distanceTraveled.toFixed(1)}{" "}
										<span className="text-sm text-gray-500">m</span>
									</p>
								</div>

								{/* Current Zone */}
								<div>
									<p className="text-[10px] uppercase tracking-wider text-gray-500">
										Current Zone
									</p>
									<p className="font-mono text-sm font-medium text-cw-green">
										{currentZone}
									</p>
								</div>
							</div>

							{/* Obstacle Avoidance Log */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-3 flex items-center gap-2">
									<svg
										className="h-3.5 w-3.5 text-yellow-400"
										viewBox="0 0 24 24"
										fill="none"
										stroke="currentColor"
										strokeWidth={2}
									>
										<path
											strokeLinecap="round"
											strokeLinejoin="round"
											d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4.5c-.77-.833-2.694-.833-3.464 0L3.34 16.5c-.77.833.192 2.5 1.732 2.5z"
										/>
									</svg>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
										Obstacle Events
									</span>
								</div>
								{obstacleEvents.length === 0 ? (
									<p className="font-mono text-[10px] text-gray-600">
										No events yet — monitoring...
									</p>
								) : (
									<div className="space-y-2">
										{obstacleEvents.map((event) => (
											<div
												key={event.id}
												className="border-l-2 border-yellow-400/30 pl-2"
											>
												<p className="font-mono text-[11px] text-gray-300">
													{event.message}
												</p>
												<p className="font-mono text-[9px] text-gray-600">
													{event.time}
												</p>
											</div>
										))}
									</div>
								)}
							</div>
						</div>
					</div>
				</div>

				{/* How It Works */}
				<section className="mb-8">
					<h2 className="mb-6 text-2xl font-bold text-white">How It Works</h2>
					<div className="grid gap-6 md:grid-cols-3">
						{/* LiDAR + Visual SLAM */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg
									className="h-6 w-6 text-cw-green"
									viewBox="0 0 24 24"
									fill="none"
									stroke="currentColor"
									strokeWidth={1.5}
								>
									<circle cx="12" cy="12" r="3" />
									<path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								LiDAR + Visual SLAM
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								360-degree LiDAR scanning at 20Hz combined with stereo camera
								Visual SLAM creates a real-time 3D map of the environment.
								The robot builds and updates its spatial understanding
								continuously, achieving centimeter-level localization accuracy
								even in GPS-denied environments like dense tree cover.
							</p>
						</div>

						{/* Path Planning */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg
									className="h-6 w-6 text-cw-green"
									viewBox="0 0 24 24"
									fill="none"
									stroke="currentColor"
									strokeWidth={1.5}
								>
									<path d="M3 12h4l3-9 4 18 3-9h4" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								Adaptive Path Planning
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								A hybrid A* and RRT* planner generates optimal coverage routes
								that maximize area cleaned per charge cycle. The planner
								dynamically re-routes based on detected litter density,
								pedestrian traffic patterns, and terrain difficulty — ensuring
								the robot spends time where it matters most.
							</p>
						</div>

						{/* Obstacle Avoidance */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg
									className="h-6 w-6 text-cw-green"
									viewBox="0 0 24 24"
									fill="none"
									stroke="currentColor"
									strokeWidth={1.5}
								>
									<path d="M9 12l2 2 4-4" />
									<path d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">
								Obstacle Avoidance
							</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								Multi-layer safety system using both LiDAR point cloud analysis
								and depth camera fusion. Detects and classifies obstacles —
								pedestrians, pets, cyclists, furniture — and applies
								context-appropriate avoidance behaviors. Pedestrians get wide
								berths; static objects get precise navigation margins.
							</p>
						</div>
					</div>
				</section>

				{/* Footer Note */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						This is a simulated demo. In production, navigation data streams
						from deployed CleanWalker robots via onboard ROS2 nodes and MQTT telemetry.
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
