// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// === Constants ===
const PW = 800;
const PH = 600;
const SPEEDS = [1, 2, 5, 10];
const BAG_CAPACITY = 12;
const DETECTION_RADIUS = 50;
const PICK_DURATION = 40; // frames
const DOCK_DURATION = 60;
const ROBOT_SPEED = 1.2;

// Litter types with colors and shapes
const LITTER_TYPES = [
	{ type: "bottle", color: "#3b82f6", size: 5 },
	{ type: "can", color: "#ef4444", size: 4 },
	{ type: "wrapper", color: "#f59e0b", size: 3.5 },
	{ type: "cup", color: "#8b5cf6", size: 4.5 },
	{ type: "bag", color: "#ec4899", size: 4 },
	{ type: "cigarette", color: "#6b7280", size: 2.5 },
] as const;

// Park layout
const DOCK = { x: 60, y: 560 };

interface LitterItem {
	id: number;
	x: number;
	y: number;
	type: (typeof LITTER_TYPES)[number];
	collected: boolean;
}

interface Tree {
	x: number;
	y: number;
	radius: number;
}

interface Bench {
	x: number;
	y: number;
	w: number;
	h: number;
	angle: number;
}

type RobotState =
	| "patrol"
	| "detecting"
	| "approaching"
	| "picking"
	| "bag_full"
	| "docking"
	| "resuming";

interface Robot {
	x: number;
	y: number;
	angle: number;
	state: RobotState;
	targetLitter: LitterItem | null;
	bagCount: number;
	bagsFilled: number;
	patrolIdx: number;
	pickTimer: number;
	dockTimer: number;
	legPhase: number;
	armExtension: number;
	detectionPulse: number;
	totalCollected: number;
	pathHistory: { x: number; y: number }[];
}

// === Park elements ===

const TREES: Tree[] = [
	{ x: 140, y: 80, radius: 22 },
	{ x: 350, y: 60, radius: 18 },
	{ x: 580, y: 90, radius: 25 },
	{ x: 720, y: 70, radius: 20 },
	{ x: 100, y: 250, radius: 16 },
	{ x: 280, y: 300, radius: 22 },
	{ x: 500, y: 200, radius: 20 },
	{ x: 680, y: 280, radius: 24 },
	{ x: 190, y: 440, radius: 18 },
	{ x: 440, y: 450, radius: 20 },
	{ x: 620, y: 420, radius: 22 },
	{ x: 750, y: 500, radius: 16 },
	{ x: 400, y: 120, radius: 14 },
	{ x: 160, y: 540, radius: 18 },
];

const BENCHES: Bench[] = [
	{ x: 220, y: 170, w: 30, h: 10, angle: 0 },
	{ x: 460, y: 140, w: 30, h: 10, angle: 0.3 },
	{ x: 600, y: 330, w: 30, h: 10, angle: -0.2 },
	{ x: 340, y: 380, w: 30, h: 10, angle: 0 },
	{ x: 150, y: 350, w: 30, h: 10, angle: 0.5 },
];

// Paths as line segments
const PATHS: { x1: number; y1: number; x2: number; y2: number; width: number }[] = [
	// Main horizontal path
	{ x1: 0, y1: 300, x2: 800, y2: 300, width: 28 },
	// Main vertical path
	{ x1: 400, y1: 0, x2: 400, y2: 600, width: 24 },
	// Diagonal path
	{ x1: 100, y1: 500, x2: 350, y2: 300, width: 18 },
	// Winding path to dock
	{ x1: 0, y1: 560, x2: 200, y2: 560, width: 20 },
];

// Patrol waypoints forming a patrol route
const PATROL_POINTS: { x: number; y: number }[] = [
	{ x: 200, y: 150 },
	{ x: 400, y: 100 },
	{ x: 600, y: 150 },
	{ x: 700, y: 250 },
	{ x: 650, y: 380 },
	{ x: 500, y: 450 },
	{ x: 350, y: 500 },
	{ x: 200, y: 450 },
	{ x: 120, y: 350 },
	{ x: 150, y: 200 },
	{ x: 300, y: 200 },
	{ x: 450, y: 280 },
	{ x: 550, y: 200 },
	{ x: 700, y: 150 },
	{ x: 750, y: 350 },
	{ x: 600, y: 500 },
	{ x: 400, y: 530 },
	{ x: 250, y: 480 },
	{ x: 130, y: 400 },
	{ x: 100, y: 200 },
];

function generateLitter(count: number): LitterItem[] {
	const items: LitterItem[] = [];
	for (let i = 0; i < count; i++) {
		let x: number;
		let y: number;
		let valid = false;
		// Keep retrying until we get a spot not overlapping trees or dock
		for (let attempt = 0; attempt < 50; attempt++) {
			x = 40 + Math.random() * (PW - 80);
			y = 40 + Math.random() * (PH - 100);
			valid = true;
			for (const tree of TREES) {
				const dx = x! - tree.x;
				const dy = y! - tree.y;
				if (Math.sqrt(dx * dx + dy * dy) < tree.radius + 10) {
					valid = false;
					break;
				}
			}
			if (valid) break;
		}
		if (!valid) continue;
		items.push({
			id: i,
			x: x!,
			y: y!,
			type: LITTER_TYPES[Math.floor(Math.random() * LITTER_TYPES.length)],
			collected: false,
		});
	}
	return items;
}

// === Drawing functions ===

function drawPark(ctx: CanvasRenderingContext2D, w: number, h: number) {
	const sx = w / PW;
	const sy = h / PH;

	// Grass background
	ctx.fillStyle = "#162016";
	ctx.fillRect(0, 0, w, h);

	// Grass texture (subtle dots)
	ctx.fillStyle = "#1a2a1a";
	for (let i = 0; i < 200; i++) {
		const gx = Math.random() * w;
		const gy = Math.random() * h;
		ctx.fillRect(gx, gy, 1, 1);
	}

	// Paths
	for (const p of PATHS) {
		ctx.strokeStyle = "#2a2a2a";
		ctx.lineWidth = p.width * sx;
		ctx.lineCap = "round";
		ctx.beginPath();
		ctx.moveTo(p.x1 * sx, p.y1 * sy);
		ctx.lineTo(p.x2 * sx, p.y2 * sy);
		ctx.stroke();

		// Path edge lines
		ctx.strokeStyle = "#333";
		ctx.lineWidth = 0.5;
		ctx.setLineDash([4, 6]);
		ctx.beginPath();
		ctx.moveTo(p.x1 * sx, p.y1 * sy);
		ctx.lineTo(p.x2 * sx, p.y2 * sy);
		ctx.stroke();
		ctx.setLineDash([]);
	}

	// Trees (trunk + canopy)
	for (const tree of TREES) {
		const tx = tree.x * sx;
		const ty = tree.y * sy;
		const r = tree.radius * Math.min(sx, sy);

		// Shadow
		ctx.fillStyle = "rgba(0,0,0,0.25)";
		ctx.beginPath();
		ctx.ellipse(tx + 3, ty + 3, r, r * 0.7, 0, 0, Math.PI * 2);
		ctx.fill();

		// Canopy
		const grad = ctx.createRadialGradient(tx, ty, 0, tx, ty, r);
		grad.addColorStop(0, "#1a4a1a");
		grad.addColorStop(0.7, "#143814");
		grad.addColorStop(1, "#0e2e0e");
		ctx.fillStyle = grad;
		ctx.beginPath();
		ctx.arc(tx, ty, r, 0, Math.PI * 2);
		ctx.fill();

		// Trunk dot
		ctx.fillStyle = "#3d2b1f";
		ctx.beginPath();
		ctx.arc(tx, ty, 3 * Math.min(sx, sy), 0, Math.PI * 2);
		ctx.fill();
	}

	// Benches
	for (const bench of BENCHES) {
		ctx.save();
		ctx.translate(bench.x * sx, bench.y * sy);
		ctx.rotate(bench.angle);
		ctx.fillStyle = "#5a3825";
		ctx.fillRect((-bench.w / 2) * sx, (-bench.h / 2) * sy, bench.w * sx, bench.h * sy);
		// Legs
		ctx.fillStyle = "#444";
		ctx.fillRect((-bench.w / 2 + 2) * sx, (-bench.h / 2 - 1) * sy, 2 * sx, (bench.h + 2) * sy);
		ctx.fillRect((bench.w / 2 - 4) * sx, (-bench.h / 2 - 1) * sy, 2 * sx, (bench.h + 2) * sy);
		ctx.restore();
	}

	// Docking station
	const dsx = DOCK.x * sx;
	const dsy = DOCK.y * sy;
	// Base
	ctx.fillStyle = "#1a2530";
	ctx.beginPath();
	ctx.roundRect(dsx - 18 * sx, dsy - 14 * sy, 36 * sx, 28 * sy, 4);
	ctx.fill();
	ctx.strokeStyle = "#22c55e";
	ctx.lineWidth = 1.5;
	ctx.stroke();
	// Lightning bolt icon
	ctx.fillStyle = "#22c55e";
	ctx.font = `${12 * Math.min(sx, sy)}px monospace`;
	ctx.textAlign = "center";
	ctx.textBaseline = "middle";
	ctx.fillText("\u26A1", dsx, dsy - 2 * sy);
	// Label
	ctx.fillStyle = "#22c55e";
	ctx.font = `bold ${7 * Math.min(sx, sy)}px monospace`;
	ctx.fillText("DOCK", dsx, dsy + 10 * sy);
}

function drawLitter(
	ctx: CanvasRenderingContext2D,
	litter: LitterItem[],
	sx: number,
	sy: number,
) {
	for (const item of litter) {
		if (item.collected) continue;
		const lx = item.x * sx;
		const ly = item.y * sy;
		const s = item.type.size * Math.min(sx, sy);

		// Glow
		ctx.fillStyle = item.type.color + "30";
		ctx.beginPath();
		ctx.arc(lx, ly, s + 3, 0, Math.PI * 2);
		ctx.fill();

		// Item shape
		ctx.fillStyle = item.type.color;
		if (item.type.type === "bottle" || item.type.type === "can") {
			ctx.beginPath();
			ctx.ellipse(lx, ly, s * 0.5, s, 0, 0, Math.PI * 2);
			ctx.fill();
		} else if (item.type.type === "wrapper" || item.type.type === "bag") {
			ctx.fillRect(lx - s * 0.7, ly - s * 0.5, s * 1.4, s);
		} else {
			ctx.beginPath();
			ctx.arc(lx, ly, s * 0.6, 0, Math.PI * 2);
			ctx.fill();
		}
	}
}

function drawRobot(
	ctx: CanvasRenderingContext2D,
	robot: Robot,
	sx: number,
	sy: number,
	tick: number,
) {
	const rx = robot.x * sx;
	const ry = robot.y * sy;
	const scale = Math.min(sx, sy);

	ctx.save();
	ctx.translate(rx, ry);
	ctx.rotate(robot.angle);

	// Detection radius (when detecting or approaching)
	if (robot.state === "detecting" || robot.state === "approaching") {
		const pulse = 0.8 + 0.2 * Math.sin(robot.detectionPulse * 0.15);
		const detR = DETECTION_RADIUS * scale * pulse;
		ctx.strokeStyle = "#22c55e50";
		ctx.lineWidth = 1.5;
		ctx.setLineDash([4, 4]);
		ctx.beginPath();
		ctx.arc(0, 0, detR, 0, Math.PI * 2);
		ctx.stroke();
		ctx.setLineDash([]);

		// Fill
		const grad = ctx.createRadialGradient(0, 0, 0, 0, 0, detR);
		grad.addColorStop(0, "#22c55e10");
		grad.addColorStop(1, "#22c55e00");
		ctx.fillStyle = grad;
		ctx.fill();
	}

	// Path trail (subtle)
	ctx.restore();
	if (robot.pathHistory.length > 1) {
		ctx.strokeStyle = "#22c55e15";
		ctx.lineWidth = 2 * scale;
		ctx.beginPath();
		const startIdx = Math.max(0, robot.pathHistory.length - 120);
		for (let i = startIdx; i < robot.pathHistory.length; i++) {
			const p = robot.pathHistory[i];
			if (i === startIdx) ctx.moveTo(p.x * sx, p.y * sy);
			else ctx.lineTo(p.x * sx, p.y * sy);
		}
		ctx.stroke();
	}

	ctx.save();
	ctx.translate(rx, ry);
	ctx.rotate(robot.angle);

	const moving =
		robot.state === "patrol" ||
		robot.state === "approaching" ||
		robot.state === "bag_full" ||
		robot.state === "resuming";
	const legAnim = moving ? Math.sin(robot.legPhase * 0.3) : 0;

	// Body shadow
	ctx.fillStyle = "rgba(0,0,0,0.3)";
	ctx.fillRect(-10 * scale + 2, -6 * scale + 2, 20 * scale, 12 * scale);

	// Legs (4 stick legs with walking animation)
	ctx.strokeStyle = "#555";
	ctx.lineWidth = 2.5 * scale;
	ctx.lineCap = "round";

	// Front-left
	const fl = legAnim * 3 * scale;
	ctx.beginPath();
	ctx.moveTo(7 * scale, -6 * scale);
	ctx.lineTo(12 * scale, -10 * scale + fl);
	ctx.stroke();

	// Front-right
	ctx.beginPath();
	ctx.moveTo(7 * scale, 6 * scale);
	ctx.lineTo(12 * scale, 10 * scale - fl);
	ctx.stroke();

	// Rear-left
	ctx.beginPath();
	ctx.moveTo(-7 * scale, -6 * scale);
	ctx.lineTo(-12 * scale, -10 * scale - fl);
	ctx.stroke();

	// Rear-right
	ctx.beginPath();
	ctx.moveTo(-7 * scale, 6 * scale);
	ctx.lineTo(-12 * scale, 10 * scale + fl);
	ctx.stroke();

	// Feet (small circles at leg ends)
	ctx.fillStyle = "#444";
	for (const [fx, fy] of [
		[12, -10 + fl / scale],
		[12, 10 - fl / scale],
		[-12, -10 - fl / scale],
		[-12, 10 + fl / scale],
	]) {
		ctx.beginPath();
		ctx.arc(fx * scale, fy * scale, 1.5 * scale, 0, Math.PI * 2);
		ctx.fill();
	}

	// Body
	ctx.fillStyle = "#1a1f2e";
	ctx.beginPath();
	ctx.roundRect(-10 * scale, -6 * scale, 20 * scale, 12 * scale, 3 * scale);
	ctx.fill();
	ctx.strokeStyle = "#22c55e";
	ctx.lineWidth = 1;
	ctx.stroke();

	// Head (front sensor)
	ctx.fillStyle = "#22c55e";
	ctx.beginPath();
	ctx.arc(10 * scale, 0, 2.5 * scale, 0, Math.PI * 2);
	ctx.fill();

	// Eye/sensor pulse
	const eyePulse = 0.5 + 0.5 * Math.sin(tick * 0.08);
	ctx.fillStyle = `rgba(34,197,94,${0.3 + eyePulse * 0.4})`;
	ctx.beginPath();
	ctx.arc(10 * scale, 0, 4 * scale, 0, Math.PI * 2);
	ctx.fill();

	// LED strips on body (2 small green dots)
	ctx.fillStyle = "#22c55e";
	ctx.beginPath();
	ctx.arc(-4 * scale, -6 * scale, 1 * scale, 0, Math.PI * 2);
	ctx.fill();
	ctx.beginPath();
	ctx.arc(-4 * scale, 6 * scale, 1 * scale, 0, Math.PI * 2);
	ctx.fill();

	// Arm (extends when picking)
	const armLen = robot.armExtension * 14 * scale;
	if (armLen > 0) {
		ctx.strokeStyle = "#666";
		ctx.lineWidth = 2 * scale;
		ctx.beginPath();
		ctx.moveTo(8 * scale, 0);
		ctx.lineTo(8 * scale + armLen, 0);
		ctx.stroke();

		// Gripper
		ctx.strokeStyle = "#22c55e";
		ctx.lineWidth = 1.5 * scale;
		const gx = 8 * scale + armLen;
		ctx.beginPath();
		ctx.moveTo(gx, -2 * scale);
		ctx.lineTo(gx + 3 * scale, -3 * scale);
		ctx.stroke();
		ctx.beginPath();
		ctx.moveTo(gx, 2 * scale);
		ctx.lineTo(gx + 3 * scale, 3 * scale);
		ctx.stroke();
	}

	// Bag fill indicator on body
	const bagPct = robot.bagCount / BAG_CAPACITY;
	ctx.fillStyle = "#0a0f0d";
	ctx.fillRect(-8 * scale, -4 * scale, 6 * scale, 8 * scale);
	ctx.fillStyle =
		bagPct > 0.8 ? "#ef4444" : bagPct > 0.5 ? "#f59e0b" : "#22c55e";
	const fillH = 8 * scale * bagPct;
	ctx.fillRect(-8 * scale, (4 - 8 * bagPct) * scale, 6 * scale, fillH);
	ctx.strokeStyle = "#444";
	ctx.lineWidth = 0.5;
	ctx.strokeRect(-8 * scale, -4 * scale, 6 * scale, 8 * scale);

	ctx.restore();

	// Label above robot
	ctx.fillStyle = "#fff";
	ctx.font = `bold ${8 * scale}px monospace`;
	ctx.textAlign = "center";
	ctx.fillText("CW-1", rx, ry - 16 * scale);

	// State label
	const stateLabels: Record<RobotState, string> = {
		patrol: "PATROL",
		detecting: "DETECTED",
		approaching: "APPROACH",
		picking: "PICKING",
		bag_full: "BAG FULL",
		docking: "DOCKING",
		resuming: "RESUMING",
	};
	const stateColors: Record<RobotState, string> = {
		patrol: "#22c55e",
		detecting: "#f59e0b",
		approaching: "#3b82f6",
		picking: "#a855f7",
		bag_full: "#ef4444",
		docking: "#f59e0b",
		resuming: "#3b82f6",
	};
	ctx.fillStyle = stateColors[robot.state];
	ctx.font = `${6 * scale}px monospace`;
	ctx.fillText(stateLabels[robot.state], rx, ry - 22 * scale);
}

function drawFrame(
	ctx: CanvasRenderingContext2D,
	w: number,
	h: number,
	robot: Robot,
	litter: LitterItem[],
	tick: number,
) {
	const sx = w / PW;
	const sy = h / PH;

	ctx.clearRect(0, 0, w, h);
	drawPark(ctx, w, h);
	drawLitter(ctx, litter, sx, sy);
	drawRobot(ctx, robot, sx, sy, tick);

	// Legend
	ctx.fillStyle = "rgba(0,0,0,0.65)";
	ctx.beginPath();
	ctx.roundRect(6, h - 28, 320, 22, 4);
	ctx.fill();
	ctx.font = "8px monospace";
	let lx = 14;
	for (const [color, label] of [
		["#162016", "Grass"],
		["#2a2a2a", "Path"],
		["#1a4a1a", "Tree"],
		["#5a3825", "Bench"],
		["#22c55e", "Robot"],
		["#22c55e50", "Detection"],
	] as const) {
		ctx.fillStyle = color;
		ctx.fillRect(lx, h - 22, 8, 10);
		ctx.strokeStyle = "#555";
		ctx.lineWidth = 0.5;
		ctx.strokeRect(lx, h - 22, 8, 10);
		ctx.fillStyle = "#9ca3af";
		ctx.textAlign = "left";
		ctx.fillText(label, lx + 11, h - 14);
		lx += label.length * 5.5 + 18;
	}
}

// === Simulation Logic ===

function angleTo(from: { x: number; y: number }, to: { x: number; y: number }): number {
	return Math.atan2(to.y - from.y, to.x - from.x);
}

function dist(
	a: { x: number; y: number },
	b: { x: number; y: number },
): number {
	const dx = a.x - b.x;
	const dy = a.y - b.y;
	return Math.sqrt(dx * dx + dy * dy);
}

function moveToward(
	robot: Robot,
	target: { x: number; y: number },
	speed: number,
): boolean {
	const d = dist(robot, target);
	if (d < speed * 2) {
		robot.x = target.x;
		robot.y = target.y;
		return true;
	}
	const a = angleTo(robot, target);
	robot.angle = a;
	robot.x += Math.cos(a) * speed;
	robot.y += Math.sin(a) * speed;
	return false;
}

function stepSimulation(robot: Robot, litter: LitterItem[], speed: number) {
	const spd = ROBOT_SPEED * speed;

	// Track path
	if (robot.pathHistory.length === 0 || dist(robot, robot.pathHistory[robot.pathHistory.length - 1]) > 5) {
		robot.pathHistory.push({ x: robot.x, y: robot.y });
		if (robot.pathHistory.length > 500) robot.pathHistory.shift();
	}

	switch (robot.state) {
		case "patrol": {
			robot.legPhase++;
			robot.armExtension = Math.max(0, robot.armExtension - 0.05);

			// Check for nearby litter
			let nearest: LitterItem | null = null;
			let nearestDist = DETECTION_RADIUS;
			for (const item of litter) {
				if (item.collected) continue;
				const d = dist(robot, item);
				if (d < nearestDist) {
					nearestDist = d;
					nearest = item;
				}
			}

			if (nearest) {
				robot.state = "detecting";
				robot.targetLitter = nearest;
				robot.detectionPulse = 0;
				break;
			}

			// Move along patrol route
			const wp = PATROL_POINTS[robot.patrolIdx];
			const arrived = moveToward(robot, wp, spd);
			if (arrived) {
				robot.patrolIdx = (robot.patrolIdx + 1) % PATROL_POINTS.length;
			}
			break;
		}

		case "detecting": {
			robot.detectionPulse++;
			if (robot.detectionPulse > 15) {
				robot.state = "approaching";
			}
			break;
		}

		case "approaching": {
			robot.legPhase++;
			robot.detectionPulse++;

			if (!robot.targetLitter || robot.targetLitter.collected) {
				robot.state = "patrol";
				robot.targetLitter = null;
				break;
			}

			const arrived = moveToward(robot, robot.targetLitter, spd * 0.8);
			if (arrived) {
				robot.state = "picking";
				robot.pickTimer = 0;
			}
			break;
		}

		case "picking": {
			robot.pickTimer++;
			// Arm extends then retracts
			if (robot.pickTimer < PICK_DURATION / 2) {
				robot.armExtension = Math.min(1, robot.armExtension + 0.08);
			} else {
				robot.armExtension = Math.max(0, robot.armExtension - 0.06);
			}

			if (robot.pickTimer >= PICK_DURATION) {
				if (robot.targetLitter) {
					robot.targetLitter.collected = true;
					robot.bagCount++;
					robot.totalCollected++;
				}
				robot.targetLitter = null;

				if (robot.bagCount >= BAG_CAPACITY) {
					robot.state = "bag_full";
				} else {
					robot.state = "patrol";
				}
			}
			break;
		}

		case "bag_full": {
			robot.legPhase++;
			const arrived = moveToward(robot, DOCK, spd);
			if (arrived) {
				robot.state = "docking";
				robot.dockTimer = 0;
			}
			break;
		}

		case "docking": {
			robot.dockTimer++;
			if (robot.dockTimer >= DOCK_DURATION) {
				robot.bagsFilled++;
				robot.bagCount = 0;
				robot.state = "resuming";
			}
			break;
		}

		case "resuming": {
			robot.legPhase++;
			const wp = PATROL_POINTS[robot.patrolIdx];
			const arrived = moveToward(robot, wp, spd);
			if (arrived) {
				robot.state = "patrol";
			}
			break;
		}
	}
}

// === Helpers ===

function fmtTime(sec: number): string {
	const m = Math.floor(sec / 60);
	const s = Math.floor(sec % 60);
	return `${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
}

// === Main Page Component ===

export default function SimulationPage() {
	const canvasRef = useRef<HTMLCanvasElement>(null);
	const containerRef = useRef<HTMLDivElement>(null);
	const animRef = useRef(0);

	const [playing, setPlaying] = useState(true);
	const [speed, setSpeed] = useState(1);
	const [stats, setStats] = useState({
		collected: 0,
		bagCount: 0,
		bagsFilled: 0,
		state: "patrol" as RobotState,
		elapsed: 0,
		pathLen: 0,
	});

	// Simulation refs
	const playingRef = useRef(playing);
	const speedRef = useRef(speed);
	const robotRef = useRef<Robot>({
		x: DOCK.x,
		y: DOCK.y,
		angle: 0,
		state: "patrol",
		targetLitter: null,
		bagCount: 0,
		bagsFilled: 0,
		patrolIdx: 0,
		pickTimer: 0,
		dockTimer: 0,
		legPhase: 0,
		armExtension: 0,
		detectionPulse: 0,
		totalCollected: 0,
		pathHistory: [],
	});
	const litterRef = useRef<LitterItem[]>(generateLitter(35));
	const tickRef = useRef(0);

	useEffect(() => {
		playingRef.current = playing;
	}, [playing]);
	useEffect(() => {
		speedRef.current = speed;
	}, [speed]);

	const reset = useCallback(() => {
		robotRef.current = {
			x: DOCK.x,
			y: DOCK.y,
			angle: 0,
			state: "patrol",
			targetLitter: null,
			bagCount: 0,
			bagsFilled: 0,
			patrolIdx: 0,
			pickTimer: 0,
			dockTimer: 0,
			legPhase: 0,
			armExtension: 0,
			detectionPulse: 0,
			totalCollected: 0,
			pathHistory: [],
		};
		litterRef.current = generateLitter(35);
		tickRef.current = 0;
		setStats({
			collected: 0,
			bagCount: 0,
			bagsFilled: 0,
			state: "patrol",
			elapsed: 0,
			pathLen: 0,
		});
	}, []);

	// Canvas resize
	useEffect(() => {
		const ct = containerRef.current;
		const cv = canvasRef.current;
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

	// Animation loop
	useEffect(() => {
		let lastTick = 0;
		const loop = (ts: number) => {
			const robot = robotRef.current;
			const litter = litterRef.current;

			if (playingRef.current) {
				const interval = 33 / speedRef.current;
				if (!lastTick) lastTick = ts;
				if (ts - lastTick >= interval) {
					lastTick = ts;
					tickRef.current++;
					stepSimulation(robot, litter, speedRef.current > 5 ? 2 : 1);

					// Check if all litter collected — respawn more
					const remaining = litter.filter((l) => !l.collected).length;
					if (remaining === 0) {
						const newLitter = generateLitter(25 + Math.floor(Math.random() * 15));
						litterRef.current = newLitter;
					}

					// Update stats periodically
					if (tickRef.current % 5 === 0) {
						setStats({
							collected: robot.totalCollected,
							bagCount: robot.bagCount,
							bagsFilled: robot.bagsFilled,
							state: robot.state,
							elapsed: tickRef.current * 0.033,
							pathLen: robot.pathHistory.length * 5,
						});
					}
				}
			}

			const cv = canvasRef.current;
			if (cv && cv.width > 0) {
				const ctx = cv.getContext("2d");
				if (ctx) {
					const dpr = window.devicePixelRatio || 1;
					ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
					const cw = cv.width / dpr;
					const ch = cv.height / dpr;
					drawFrame(ctx, cw, ch, robot, litterRef.current, tickRef.current);
				}
			}

			animRef.current = requestAnimationFrame(loop);
		};
		animRef.current = requestAnimationFrame(loop);
		return () => cancelAnimationFrame(animRef.current);
	}, []);

	const bagPct = (stats.bagCount / BAG_CAPACITY) * 100;

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
								<path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">
								Robot Simulation
							</h1>
							<p className="font-mono text-[10px] text-gray-500">
								Autonomous litter collection in action
							</p>
						</div>
					</div>
					<div className="flex items-center gap-4">
						<button
							onClick={() => setPlaying((p) => !p)}
							className="flex items-center gap-2 rounded-lg border border-white/10 px-3 py-1.5 font-mono text-xs text-white transition-colors hover:bg-white/5"
						>
							{playing ? (
								<svg
									className="h-3.5 w-3.5"
									viewBox="0 0 24 24"
									fill="currentColor"
								>
									<rect x="6" y="4" width="4" height="16" />
									<rect x="14" y="4" width="4" height="16" />
								</svg>
							) : (
								<svg
									className="h-3.5 w-3.5"
									viewBox="0 0 24 24"
									fill="currentColor"
								>
									<path d="M8 5v14l11-7z" />
								</svg>
							)}
							{playing ? "Pause" : "Play"}
						</button>
						<div className="hidden items-center gap-2 sm:flex">
							<span className="relative flex h-2 w-2">
								{playing && (
									<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
								)}
								<span
									className={`relative inline-flex h-2 w-2 rounded-full ${playing ? "bg-cw-green" : "bg-gray-500"}`}
								/>
							</span>
							<span
								className={`font-mono text-xs ${playing ? "text-cw-green" : "text-gray-500"}`}
							>
								{playing ? "RUNNING" : "PAUSED"}
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

				{/* Main Grid: Canvas + Controls */}
				<div className="mb-6 grid gap-6 lg:grid-cols-5">
					{/* Canvas */}
					<div className="lg:col-span-3">
						<div
							ref={containerRef}
							className="overflow-hidden rounded-xl border border-white/10 bg-[#0d1117]"
						>
							<canvas ref={canvasRef} className="block w-full" />
						</div>

						{/* Bag fill bar below canvas */}
						<div className="mt-3 rounded-lg border border-white/10 bg-white/[0.03] px-4 py-3">
							<div className="mb-2 flex items-center justify-between">
								<span className="font-mono text-[10px] uppercase tracking-wider text-gray-400">
									Bag Capacity
								</span>
								<span
									className={`font-mono text-sm font-bold ${bagPct > 80 ? "text-red-400" : bagPct > 50 ? "text-yellow-400" : "text-cw-green"}`}
								>
									{stats.bagCount} / {BAG_CAPACITY}
								</span>
							</div>
							<div className="h-2.5 overflow-hidden rounded-full bg-white/10">
								<div
									className={`h-full rounded-full transition-all duration-300 ${bagPct > 80 ? "bg-red-500" : bagPct > 50 ? "bg-yellow-500" : "bg-cw-green"}`}
									style={{ width: `${Math.min(100, bagPct)}%` }}
								/>
							</div>
							<p className="mt-1 font-mono text-[10px] text-gray-600">
								{stats.bagsFilled} bag{stats.bagsFilled !== 1 ? "s" : ""}{" "}
								completed at dock
							</p>
						</div>
					</div>

					{/* Controls + Stats Panel */}
					<div className="flex flex-col gap-4 lg:col-span-2">
						{/* Speed Control */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								Simulation Speed
							</p>
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

						{/* Robot Status */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								Robot Status
							</p>
							<div className="flex items-center gap-3">
								<div
									className={`h-3 w-3 rounded-full ${
										stats.state === "patrol"
											? "bg-cw-green"
											: stats.state === "picking"
												? "bg-purple-500"
												: stats.state === "bag_full" ||
													  stats.state === "docking"
													? "bg-yellow-500"
													: "bg-blue-500"
									}`}
								/>
								<span className="font-mono text-sm font-medium text-white">
									{stats.state === "patrol" && "Patrolling Park"}
									{stats.state === "detecting" && "Litter Detected"}
									{stats.state === "approaching" && "Approaching Target"}
									{stats.state === "picking" && "Collecting Litter"}
									{stats.state === "bag_full" && "Returning to Dock"}
									{stats.state === "docking" && "Swapping Bag at Dock"}
									{stats.state === "resuming" && "Resuming Patrol"}
								</span>
							</div>
							<div className="mt-3 space-y-2">
								<div className="flex justify-between font-mono text-[10px]">
									<span className="text-gray-500">Current Bag</span>
									<span
										className={
											bagPct > 80
												? "text-red-400"
												: bagPct > 50
													? "text-yellow-400"
													: "text-cw-green"
										}
									>
										{stats.bagCount}/{BAG_CAPACITY} items
									</span>
								</div>
								<div className="flex justify-between font-mono text-[10px]">
									<span className="text-gray-500">Bags Completed</span>
									<span className="text-white">{stats.bagsFilled}</span>
								</div>
							</div>
						</div>

						{/* Stats Grid */}
						<div className="grid grid-cols-2 gap-3">
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
									Collected
								</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">
									{stats.collected}
								</p>
								<p className="font-mono text-[10px] text-gray-600">
									litter items
								</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
									Elapsed
								</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">
									{fmtTime(stats.elapsed)}
								</p>
								<p className="font-mono text-[10px] text-gray-600">
									sim time
								</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
									Distance
								</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">
									{(stats.pathLen / 1000).toFixed(1)}
								</p>
								<p className="font-mono text-[10px] text-gray-600">
									km traveled
								</p>
							</div>
							<div className="rounded-xl border border-white/10 bg-white/[0.03] px-4 py-3">
								<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
									Rate
								</p>
								<p className="mt-1 font-mono text-xl font-bold text-white">
									{stats.elapsed > 0
										? Math.round(
												(stats.collected / stats.elapsed) * 3600,
											)
										: 0}
								</p>
								<p className="font-mono text-[10px] text-gray-600">
									items/hr
								</p>
							</div>
						</div>

						{/* How It Works */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4">
							<p className="mb-3 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								How It Works
							</p>
							<div className="space-y-2 font-mono text-[10px] leading-relaxed text-gray-500">
								<p>
									<span className="text-cw-green">1.</span> Robot patrols
									the park along an optimized route
								</p>
								<p>
									<span className="text-cw-green">2.</span> AI detects
									litter within a 50m radius
								</p>
								<p>
									<span className="text-cw-green">3.</span> Arm extends,
									gripper picks up the item
								</p>
								<p>
									<span className="text-cw-green">4.</span> Item deposited
									in sealed collection bag
								</p>
								<p>
									<span className="text-cw-green">5.</span> When bag is
									full, robot returns to dock
								</p>
								<p>
									<span className="text-cw-green">6.</span> Bag dropped at
									dock, fresh bag loaded
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
						Simplified simulation of CleanWalker CW-1 autonomous litter
						collection. In production, the robot uses AI-powered computer
						vision, LiDAR SLAM navigation, and adaptive gait control for
						all-terrain operation.
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
