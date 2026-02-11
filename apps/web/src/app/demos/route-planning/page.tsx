// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// === Types ===
type Strategy = "boustrophedon" | "boundary" | "hotspot";

// === Grid Constants ===
const PW = 800;
const PH = 520;
const CS = 10;
const COLS = PW / CS;
const ROWS = PH / CS;
const ROBOT_COLORS = ["#22c55e", "#3b82f6", "#f59e0b", "#ef4444", "#a855f7"];
const SPEEDS = [1, 2, 5, 10];

const STRATEGY_META: Record<Strategy, { label: string; desc: string }> = {
	boustrophedon: {
		label: "Boustrophedon",
		desc: "Systematic back-and-forth sweep lines. Minimizes missed areas. Best for uniform litter distribution across open spaces.",
	},
	boundary: {
		label: "Boundary First",
		desc: "Perimeter sweep followed by inward spiral. Targets edges where wind-blown litter accumulates near fences and paths.",
	},
	hotspot: {
		label: "Hotspot Priority",
		desc: "AI-predicted high-litter zones cleaned first. Maximizes early impact near food courts, benches, and bin overflows.",
	},
};

// === Park Grid Builder ===
// Cell values: 0=grass, 1=walkway, 2=water, 3=obstacle

function buildGrid(): number[][] {
	const g: number[][] = Array.from({ length: ROWS }, () => Array(COLS).fill(0));
	const fill = (x: number, y: number, w: number, h: number, v: number) => {
		for (let r = Math.max(0, y); r < Math.min(y + h, ROWS); r++)
			for (let c = Math.max(0, x); c < Math.min(x + w, COLS); c++) g[r][c] = v;
	};
	// Walkways
	fill(0, 22, 80, 3, 1);
	fill(38, 0, 3, 52, 1);
	fill(0, 44, 80, 2, 1);
	fill(18, 0, 2, 25, 1);
	fill(60, 22, 2, 24, 1);
	// Pond (ellipse)
	for (let r = 0; r < ROWS; r++)
		for (let c = 0; c < COLS; c++) {
			const dx = (c - 53) / 7,
				dy = (r - 9) / 4.5;
			if (dx * dx + dy * dy <= 1) g[r][c] = 2;
		}
	// Benches
	fill(10, 10, 3, 2, 3);
	fill(30, 18, 3, 1, 3);
	fill(50, 31, 3, 1, 3);
	fill(70, 36, 3, 1, 3);
	fill(14, 36, 3, 1, 3);
	fill(45, 15, 1, 2, 3);
	fill(25, 28, 3, 1, 3);
	fill(68, 48, 3, 1, 3);
	// Trees
	for (const [tx, ty] of [
		[8, 5], [25, 8], [72, 4], [13, 32], [46, 36],
		[71, 18], [28, 40], [66, 48], [5, 16], [75, 28],
	]) {
		for (let dr = -1; dr <= 1; dr++)
			for (let dc = -1; dc <= 1; dc++) {
				const nr = ty + dr, nc = tx + dc;
				if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) g[nr][nc] = 3;
			}
	}
	return g;
}

const GRID = buildGrid();
const TOTAL_GRASS = GRID.flat().filter((v) => v === 0).length;

const HOTSPOTS = [
	{ cx: 12, cy: 15, r: 6, intensity: 0.85 },
	{ cx: 52, cy: 33, r: 5, intensity: 0.95 },
	{ cx: 68, cy: 10, r: 5, intensity: 0.7 },
	{ cx: 30, cy: 40, r: 6, intensity: 0.6 },
];

// === Route Generation ===

const isGrass = (r: number, c: number) =>
	r >= 0 && r < ROWS && c >= 0 && c < COLS && GRID[r][c] === 0;

function genBoustrophedon(): [number, number][] {
	const wp: [number, number][] = [];
	for (let r = 0; r < ROWS; r++) {
		if (r % 2 === 0) {
			for (let c = 0; c < COLS; c++) if (isGrass(r, c)) wp.push([r, c]);
		} else {
			for (let c = COLS - 1; c >= 0; c--) if (isGrass(r, c)) wp.push([r, c]);
		}
	}
	return wp;
}

function genBoundary(): [number, number][] {
	const wp: [number, number][] = [];
	const seen = new Set<string>();
	let t = 0, b = ROWS - 1, l = 0, ri = COLS - 1;
	while (t <= b && l <= ri) {
		for (let c = l; c <= ri; c++) {
			const k = `${t},${c}`;
			if (isGrass(t, c) && !seen.has(k)) { wp.push([t, c]); seen.add(k); }
		}
		t++;
		for (let r = t; r <= b; r++) {
			const k = `${r},${ri}`;
			if (isGrass(r, ri) && !seen.has(k)) { wp.push([r, ri]); seen.add(k); }
		}
		ri--;
		for (let c = ri; c >= l; c--) {
			const k = `${b},${c}`;
			if (isGrass(b, c) && !seen.has(k)) { wp.push([b, c]); seen.add(k); }
		}
		b--;
		for (let r = b; r >= t; r--) {
			const k = `${r},${l}`;
			if (isGrass(r, l) && !seen.has(k)) { wp.push([r, l]); seen.add(k); }
		}
		l++;
	}
	return wp;
}

function genHotspot(): [number, number][] {
	const wp: [number, number][] = [];
	const seen = new Set<string>();
	for (const hs of [...HOTSPOTS].sort((a, b) => b.intensity - a.intensity)) {
		for (let layer = 0; layer <= hs.r; layer++)
			for (let dr = -layer; dr <= layer; dr++)
				for (let dc = -layer; dc <= layer; dc++) {
					if (Math.abs(dr) !== layer && Math.abs(dc) !== layer) continue;
					const r = hs.cy + dr, c = hs.cx + dc, k = `${r},${c}`;
					if (isGrass(r, c) && !seen.has(k)) { wp.push([r, c]); seen.add(k); }
				}
	}
	for (let r = 0; r < ROWS; r++) {
		const cols = r % 2 === 0
			? Array.from({ length: COLS }, (_, i) => i)
			: Array.from({ length: COLS }, (_, i) => COLS - 1 - i);
		for (const c of cols) {
			const k = `${r},${c}`;
			if (isGrass(r, c) && !seen.has(k)) { wp.push([r, c]); seen.add(k); }
		}
	}
	return wp;
}

function genRoute(s: Strategy) {
	return s === "boustrophedon" ? genBoustrophedon() : s === "boundary" ? genBoundary() : genHotspot();
}

// === Canvas Drawing ===

const CELL_COLORS: Record<number, string> = {
	0: "#162016", 1: "#252525", 2: "#142840", 3: "#3d2b1f",
};

function drawFrame(
	ctx: CanvasRenderingContext2D,
	w: number, h: number,
	covered: Set<string>,
	bots: { id: number; x: number; y: number; color: string; wpIdx: number }[],
	strategy: Strategy,
	wp: [number, number][],
	nBots: number,
) {
	const sx = w / PW, sy = h / PH;
	ctx.clearRect(0, 0, w, h);

	// Park cells
	for (let r = 0; r < ROWS; r++)
		for (let c = 0; c < COLS; c++) {
			ctx.fillStyle = CELL_COLORS[GRID[r][c]];
			ctx.fillRect(c * CS * sx, r * CS * sy, CS * sx + 0.5, CS * sy + 0.5);
		}

	// Subtle grid on grass
	ctx.strokeStyle = "#1e2e1e";
	ctx.lineWidth = 0.3;
	for (let r = 0; r < ROWS; r++)
		for (let c = 0; c < COLS; c++)
			if (GRID[r][c] === 0) ctx.strokeRect(c * CS * sx, r * CS * sy, CS * sx, CS * sy);

	// Walkway center dashes
	ctx.strokeStyle = "#333";
	ctx.lineWidth = 0.5;
	ctx.setLineDash([4, 6]);
	ctx.beginPath();
	ctx.moveTo(0, 23.5 * CS * sy);
	ctx.lineTo(w, 23.5 * CS * sy);
	ctx.stroke();
	ctx.beginPath();
	ctx.moveTo(39.5 * CS * sx, 0);
	ctx.lineTo(39.5 * CS * sx, h);
	ctx.stroke();
	ctx.setLineDash([]);

	// Hotspot heat circles
	if (strategy === "hotspot") {
		for (const hs of HOTSPOTS) {
			const cx = (hs.cx * CS + CS / 2) * sx, cy = (hs.cy * CS + CS / 2) * sy;
			const rad = hs.r * CS * Math.max(sx, sy);
			const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, rad);
			grad.addColorStop(0, `rgba(239,68,68,${hs.intensity * 0.35})`);
			grad.addColorStop(1, "rgba(239,68,68,0)");
			ctx.fillStyle = grad;
			ctx.beginPath();
			ctx.arc(cx, cy, rad, 0, Math.PI * 2);
			ctx.fill();
		}
	}

	// Coverage overlay
	ctx.fillStyle = "rgba(34,197,94,0.22)";
	for (const key of covered) {
		const [r, c] = key.split(",").map(Number);
		ctx.fillRect(c * CS * sx, r * CS * sy, CS * sx + 0.5, CS * sy + 0.5);
	}

	// Route preview per robot
	const wpPer = Math.ceil(wp.length / Math.max(1, nBots));
	for (const bot of bots) {
		const end = Math.min((bot.id + 1) * wpPer, wp.length);
		const previewN = Math.min(80, end - bot.wpIdx);
		if (previewN <= 0) continue;
		ctx.strokeStyle = bot.color + "20";
		ctx.lineWidth = 1;
		ctx.setLineDash([2, 4]);
		ctx.beginPath();
		for (let i = 0; i < previewN; i++) {
			const idx = bot.wpIdx + i;
			if (idx >= end) break;
			const [wr, wc] = wp[idx];
			const px = (wc * CS + CS / 2) * sx, py = (wr * CS + CS / 2) * sy;
			i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
		}
		ctx.stroke();
		ctx.setLineDash([]);
	}

	// Zone labels
	ctx.font = "9px monospace";
	ctx.fillStyle = "#3a4a3a";
	ctx.textAlign = "center";
	for (const [text, gx, gy] of [
		["CENTRAL LAWN", 10, 6], ["POND", 53, 5], ["PICNIC AREA", 10, 30],
		["FLOWER GARDEN", 67, 30], ["PLAYGROUND", 28, 49], ["SPORTS FIELD", 52, 49],
	] as const) {
		ctx.fillText(text, (gx as number) * CS * sx, (gy as number) * CS * sy);
	}

	// Robots
	for (const bot of bots) {
		const px = bot.x * sx, py = bot.y * sy;
		const glow = ctx.createRadialGradient(px, py, 0, px, py, 14);
		glow.addColorStop(0, bot.color + "50");
		glow.addColorStop(1, bot.color + "00");
		ctx.fillStyle = glow;
		ctx.beginPath();
		ctx.arc(px, py, 14, 0, Math.PI * 2);
		ctx.fill();
		ctx.fillStyle = bot.color;
		ctx.beginPath();
		ctx.arc(px, py, 5, 0, Math.PI * 2);
		ctx.fill();
		ctx.strokeStyle = "#fff";
		ctx.lineWidth = 1.5;
		ctx.stroke();
		ctx.fillStyle = "#fff";
		ctx.font = "bold 9px monospace";
		ctx.textAlign = "center";
		ctx.fillText(`CW-${bot.id + 1}`, px, py - 10);
	}

	// Legend bar
	ctx.fillStyle = "rgba(0,0,0,0.65)";
	const lgX = 6, lgY = h - 28;
	ctx.beginPath();
	ctx.roundRect(lgX, lgY, 265, 22, 4);
	ctx.fill();
	ctx.font = "8px monospace";
	let lx = lgX + 8;
	for (const [color, label] of [
		["#162016", "Grass"], ["#252525", "Path"], ["#142840", "Water"],
		["#3d2b1f", "Obstacle"], ["rgba(34,197,94,0.5)", "Covered"],
	]) {
		ctx.fillStyle = color;
		ctx.fillRect(lx, lgY + 6, 8, 10);
		ctx.strokeStyle = "#555";
		ctx.lineWidth = 0.5;
		ctx.strokeRect(lx, lgY + 6, 8, 10);
		ctx.fillStyle = "#9ca3af";
		ctx.textAlign = "left";
		ctx.fillText(label, lx + 11, lgY + 14);
		lx += label.length * 5.5 + 18;
	}
}

// === Helpers ===

function fmtTime(sec: number): string {
	const h = Math.floor(sec / 3600);
	const m = Math.floor((sec % 3600) / 60);
	const s = Math.floor(sec % 60);
	return h > 0
		? `${h}:${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`
		: `${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
}

// === Main Page Component ===

export default function RoutePlanningPage() {
	const canvasRef = useRef<HTMLCanvasElement>(null);
	const containerRef = useRef<HTMLDivElement>(null);
	const animRef = useRef(0);

	const [strategy, setStrategy] = useState<Strategy>("boustrophedon");
	const [playing, setPlaying] = useState(true);
	const [speed, setSpeed] = useState(1);
	const [robotCount, setRobotCount] = useState(3);
	const [coveredCount, setCoveredCount] = useState(0);
	const [ticks, setTicks] = useState(0);
	const [totalDist, setTotalDist] = useState(0);
	const [complete, setComplete] = useState(false);

	// Refs for animation loop
	const playingRef = useRef(playing);
	const speedRef = useRef(speed);
	const strategyRef = useRef(strategy);
	const wpRef = useRef<[number, number][]>([]);
	const botsRef = useRef<{ id: number; x: number; y: number; color: string; wpIdx: number; dist: number }[]>([]);
	const coveredRef = useRef(new Set<string>());
	const ticksRef = useRef(0);
	const completeRef = useRef(false);

	useEffect(() => { playingRef.current = playing; }, [playing]);
	useEffect(() => { speedRef.current = speed; }, [speed]);
	useEffect(() => { strategyRef.current = strategy; }, [strategy]);

	// Reset simulation
	const reset = useCallback(() => {
		const wp = genRoute(strategy);
		wpRef.current = wp;
		coveredRef.current = new Set();
		ticksRef.current = 0;
		completeRef.current = false;
		const per = Math.ceil(wp.length / robotCount);
		const bots = [];
		for (let i = 0; i < robotCount; i++) {
			const si = Math.min(i * per, wp.length - 1);
			const pt = wp[si] || [0, 0];
			bots.push({
				id: i,
				x: pt[1] * CS + CS / 2,
				y: pt[0] * CS + CS / 2,
				color: ROBOT_COLORS[i],
				wpIdx: i * per,
				dist: 0,
			});
		}
		botsRef.current = bots;
		setCoveredCount(0);
		setTicks(0);
		setTotalDist(0);
		setComplete(false);
	}, [strategy, robotCount]);

	useEffect(() => { reset(); }, [reset]);

	// Canvas resize
	useEffect(() => {
		const ct = containerRef.current, cv = canvasRef.current;
		if (!ct || !cv) return;
		const resize = () => {
			const w = ct.clientWidth;
			const h = Math.round(w * (PH / PW));
			const dpr = window.devicePixelRatio || 1;
			cv.width = w * dpr;
			cv.height = h * dpr;
			cv.style.width = `${w}px`;
			cv.style.height = `${h}px`;
		};
		resize();
		const obs = new ResizeObserver(resize);
		obs.observe(ct);
		return () => obs.disconnect();
	}, []);

	// Permanent animation loop
	useEffect(() => {
		let lastTick = 0;
		const loop = (ts: number) => {
			if (playingRef.current && !completeRef.current) {
				const interval = 50 / speedRef.current;
				if (!lastTick) lastTick = ts;
				if (ts - lastTick >= interval) {
					lastTick = ts;
					const wp = wpRef.current;
					const bots = botsRef.current;
					const per = Math.ceil(wp.length / Math.max(1, bots.length));
					let allDone = true;
					for (const bot of bots) {
						const end = Math.min((bot.id + 1) * per, wp.length);
						if (bot.wpIdx < end) {
							allDone = false;
							const [r, c] = wp[bot.wpIdx];
							bot.x = c * CS + CS / 2;
							bot.y = r * CS + CS / 2;
							coveredRef.current.add(`${r},${c}`);
							bot.dist += 0.5;
							bot.wpIdx++;
						}
					}
					ticksRef.current++;
					if (allDone) completeRef.current = true;
					if (ticksRef.current % 4 === 0 || allDone) {
						setCoveredCount(coveredRef.current.size);
						setTicks(ticksRef.current);
						setTotalDist(bots.reduce((s, b) => s + b.dist, 0));
						if (allDone) setComplete(true);
					}
				}
			}
			const cv = canvasRef.current;
			if (cv && cv.width > 0) {
				const ctx = cv.getContext("2d");
				if (ctx) {
					const dpr = window.devicePixelRatio || 1;
					ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
					const cw = cv.width / dpr, ch = cv.height / dpr;
					drawFrame(ctx, cw, ch, coveredRef.current, botsRef.current, strategyRef.current, wpRef.current, botsRef.current.length);
				}
			}
			animRef.current = requestAnimationFrame(loop);
		};
		animRef.current = requestAnimationFrame(loop);
		return () => cancelAnimationFrame(animRef.current);
		// eslint-disable-next-line react-hooks/exhaustive-deps
	}, []);

	// Derived stats
	const coveragePct = TOTAL_GRASS > 0 ? (coveredCount / TOTAL_GRASS) * 100 : 0;
	const simSec = ticks * 0.5;
	const itemsFound = Math.floor(coveredCount / 35);
	const etaSec = robotCount > 0 && !complete ? ((TOTAL_GRASS - coveredCount) / robotCount) * 0.5 : 0;

	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Header */}
			<header className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-[1600px] items-center justify-between px-6 py-3">
					<div className="flex items-center gap-3">
						<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
							<svg className="h-4 w-4 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
								<path d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l5.447 2.724A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">Route Planning Visualization</h1>
							<p className="font-mono text-[10px] text-gray-500">Coverage path optimization engine</p>
						</div>
					</div>
					<div className="flex items-center gap-4">
						<button
							onClick={() => setPlaying((p) => !p)}
							className="flex items-center gap-2 rounded-lg border border-white/10 px-3 py-1.5 font-mono text-xs text-white transition-colors hover:bg-white/5"
						>
							{playing ? (
								<svg className="h-3.5 w-3.5" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16" /><rect x="14" y="4" width="4" height="16" /></svg>
							) : (
								<svg className="h-3.5 w-3.5" viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z" /></svg>
							)}
							{playing ? "Pause" : "Play"}
						</button>
						<div className="hidden items-center gap-2 sm:flex">
							<span className="relative flex h-2 w-2">
								{playing && <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />}
								<span className={`relative inline-flex h-2 w-2 rounded-full ${playing ? "bg-cw-green" : "bg-gray-500"}`} />
							</span>
							<span className={`font-mono text-xs ${playing ? "text-cw-green" : "text-gray-500"}`}>
								{playing ? "RUNNING" : "PAUSED"}
							</span>
						</div>
					</div>
				</div>
			</header>

			<div className="mx-auto max-w-[1600px] px-6 py-6">
				{/* Back link */}
				<a href="/demos" className="mb-6 inline-flex items-center gap-2 font-mono text-xs text-gray-500 transition-colors hover:text-cw-green">
					<svg className="h-3 w-3" fill="none" viewBox="0 0 24 24" stroke="currentColor">
						<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
					</svg>
					Back to Demos
				</a>

				{/* Main Grid: Canvas + Controls */}
				<div className="mb-6 grid gap-6 lg:grid-cols-5">
					{/* Canvas */}
					<div className="lg:col-span-3">
						<div ref={containerRef} className="overflow-hidden rounded-xl border border-white/10 bg-[#0d1117]">
							<canvas ref={canvasRef} className="block w-full" />
						</div>
						{/* Coverage bar below canvas */}
						<div className="mt-3 rounded-lg border border-white/10 bg-white/[0.03] px-4 py-3">
							<div className="mb-2 flex items-center justify-between">
								<span className="font-mono text-[10px] uppercase tracking-wider text-gray-400">Area Coverage</span>
								<span className="font-mono text-sm font-bold text-cw-green">{coveragePct.toFixed(1)}%</span>
							</div>
							<div className="h-2.5 overflow-hidden rounded-full bg-white/10">
								<div
									className="h-full rounded-full bg-cw-green transition-all duration-300"
									style={{ width: `${Math.min(100, coveragePct)}%` }}
								/>
							</div>
							{complete && (
								<p className="mt-2 text-center font-mono text-xs font-semibold text-cw-green">
									Coverage Complete
								</p>
							)}
						</div>
					</div>

					{/* Controls + Stats Panel */}
					<div className="flex flex-col gap-4 lg:col-span-2">
						{/* Strategy Selector */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">Route Strategy</p>
							<div className="grid grid-cols-3 gap-2">
								{(["boustrophedon", "boundary", "hotspot"] as Strategy[]).map((s) => (
									<button
										key={s}
										onClick={() => setStrategy(s)}
										className={`rounded-lg border px-3 py-2 font-mono text-[11px] font-medium transition-all ${
											strategy === s
												? "border-cw-green/50 bg-cw-green/10 text-cw-green"
												: "border-white/10 text-gray-400 hover:border-white/20 hover:text-white"
										}`}
									>
										{STRATEGY_META[s].label}
									</button>
								))}
							</div>
							<p className="mt-3 font-mono text-[10px] leading-relaxed text-gray-500">
								{STRATEGY_META[strategy].desc}
							</p>
						</div>

						{/* Speed Control */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">Simulation Speed</p>
							<div className="flex gap-2">
								{SPEEDS.map((s) => (
									<button
										key={s}
										onClick={() => setSpeed(s)}
										className={`flex-1 rounded-lg border py-1.5 font-mono text-xs font-medium transition-all ${
											speed === s
												? "border-cw-green/50 bg-cw-green/10 text-cw-green"
												: "border-white/10 text-gray-400 hover:border-white/20 hover:text-white"
										}`}
									>
										{s}x
									</button>
								))}
							</div>
						</div>

						{/* Robot Count */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">Fleet Size</p>
							<div className="flex items-center gap-3">
								<button
									onClick={() => setRobotCount((n) => Math.max(1, n - 1))}
									className="flex h-8 w-8 items-center justify-center rounded-lg border border-white/10 font-mono text-lg text-gray-400 transition-colors hover:border-white/20 hover:text-white"
								>
									-
								</button>
								<div className="flex-1 text-center">
									<span className="font-mono text-2xl font-bold text-white">{robotCount}</span>
									<span className="ml-1 font-mono text-xs text-gray-500">robot{robotCount !== 1 ? "s" : ""}</span>
								</div>
								<button
									onClick={() => setRobotCount((n) => Math.min(5, n + 1))}
									className="flex h-8 w-8 items-center justify-center rounded-lg border border-white/10 font-mono text-lg text-gray-400 transition-colors hover:border-white/20 hover:text-white"
								>
									+
								</button>
							</div>
							{/* Robot color indicators */}
							<div className="mt-3 flex justify-center gap-2">
								{Array.from({ length: robotCount }).map((_, i) => (
									<div key={i} className="flex items-center gap-1">
										<span className="h-2.5 w-2.5 rounded-full" style={{ backgroundColor: ROBOT_COLORS[i] }} />
										<span className="font-mono text-[9px] text-gray-500">CW-{i + 1}</span>
									</div>
								))}
							</div>
						</div>

						{/* Stats */}
						<div className="grid grid-cols-2 gap-3">
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">Coverage</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">{coveragePct.toFixed(1)}%</p>
								<p className="font-mono text-[10px] text-gray-600">{coveredCount.toLocaleString()} / {TOTAL_GRASS.toLocaleString()} cells</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">Elapsed</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">{fmtTime(simSec)}</p>
								<p className="font-mono text-[10px] text-gray-600">ETA {complete ? "--:--" : fmtTime(etaSec)}</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">Distance</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">{(totalDist / 1000).toFixed(2)} km</p>
								<p className="font-mono text-[10px] text-gray-600">{totalDist.toFixed(0)} m total</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">Items Found</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">{itemsFound}</p>
								<p className="font-mono text-[10px] text-gray-600">
									{simSec > 0 ? Math.round((itemsFound / simSec) * 3600) : 0} items/hr
								</p>
							</div>
						</div>

						{/* Reset Button */}
						<button
							onClick={reset}
							className="rounded-lg border border-white/10 bg-white/[0.03] px-4 py-2.5 font-mono text-xs text-gray-400 transition-colors hover:border-white/20 hover:text-white"
						>
							Reset Simulation
						</button>
					</div>
				</div>

				{/* Footer */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						Simulated route planning demo. In production, routes are computed using SLAM maps, real-time litter heatmaps, and multi-robot task allocation.
					</p>
					<a
						href="/contact"
						className="mt-3 inline-block rounded-lg bg-cw-green/10 px-6 py-2 font-mono text-xs font-medium text-cw-green transition-colors hover:bg-cw-green/20"
					>
						Schedule a Fleet Demo
					</a>
				</div>
			</div>
		</div>
	);
}
