// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// --- Data types ---

interface Robot {
	id: string;
	name: string;
	status: "active" | "charging" | "returning";
	battery: number;
	items: number;
	x: number;
	y: number;
	zone: string;
}

interface ActivityEvent {
	id: number;
	time: string;
	robot: string;
	message: string;
	type: "pickup" | "battery" | "complete" | "dock" | "alert";
}

// --- Constants ---

const ZONES = [
	"Park Zone A",
	"Park Zone B",
	"Riverside Walk",
	"Sector 4",
	"Campus North",
	"Beach Front",
	"Parking Area C",
	"Sports Fields",
	"Playground East",
	"Garden Path",
];

const LITTER_TYPES = [
	"plastic bottle",
	"aluminum can",
	"cigarette butt",
	"candy wrapper",
	"paper cup",
	"plastic bag",
	"food container",
	"straw",
	"napkin",
	"glass bottle",
	"chip bag",
	"coffee cup lid",
	"tissue",
	"bottle cap",
	"styrofoam piece",
];

const INITIAL_ROBOTS: Robot[] = [
	{ id: "CW-001", name: "CW-001", status: "active", battery: 87, items: 34, x: 18, y: 22, zone: "Park Zone A" },
	{ id: "CW-002", name: "CW-002", status: "active", battery: 72, items: 28, x: 45, y: 15, zone: "Riverside Walk" },
	{ id: "CW-003", name: "CW-003", status: "active", battery: 91, items: 41, x: 72, y: 35, zone: "Campus North" },
	{ id: "CW-004", name: "CW-004", status: "charging", battery: 23, items: 52, x: 88, y: 82, zone: "Dock Station 1" },
	{ id: "CW-005", name: "CW-005", status: "active", battery: 65, items: 19, x: 32, y: 58, zone: "Park Zone B" },
	{ id: "CW-006", name: "CW-006", status: "returning", battery: 14, items: 47, x: 55, y: 70, zone: "Beach Front" },
	{ id: "CW-007", name: "CW-007", status: "active", battery: 78, items: 22, x: 15, y: 75, zone: "Playground East" },
	{ id: "CW-008", name: "CW-008", status: "active", battery: 83, items: 31, x: 60, y: 48, zone: "Sports Fields" },
	{ id: "CW-009", name: "CW-009", status: "charging", battery: 8, items: 63, x: 92, y: 18, zone: "Dock Station 2" },
	{ id: "CW-010", name: "CW-010", status: "active", battery: 56, items: 15, x: 38, y: 38, zone: "Garden Path" },
	{ id: "CW-011", name: "CW-011", status: "active", battery: 94, items: 8, x: 25, y: 48, zone: "Sector 4" },
	{ id: "CW-012", name: "CW-012", status: "returning", battery: 11, items: 55, x: 68, y: 62, zone: "Parking Area C" },
];

// --- Helpers ---

function formatTime(date: Date): string {
	return date.toLocaleTimeString("en-US", {
		hour: "2-digit",
		minute: "2-digit",
		second: "2-digit",
		hour12: false,
	});
}

function formatDate(date: Date): string {
	return date.toLocaleDateString("en-US", {
		weekday: "short",
		month: "short",
		day: "numeric",
		year: "numeric",
	});
}

function randomFrom<T>(arr: T[]): T {
	return arr[Math.floor(Math.random() * arr.length)];
}

function statusColor(status: Robot["status"]): string {
	switch (status) {
		case "active":
			return "#22c55e";
		case "charging":
			return "#eab308";
		case "returning":
			return "#3b82f6";
	}
}

function statusLabel(status: Robot["status"]): string {
	switch (status) {
		case "active":
			return "Active";
		case "charging":
			return "Charging";
		case "returning":
			return "Returning";
	}
}

function batteryColor(pct: number): string {
	if (pct > 50) return "#22c55e";
	if (pct > 20) return "#eab308";
	return "#ef4444";
}

function generateEvent(robots: Robot[], eventId: number): ActivityEvent {
	const now = new Date();
	const time = formatTime(now);
	const robot = randomFrom(robots);
	const roll = Math.random();

	if (robot.status === "returning" || robot.battery < 15) {
		return {
			id: eventId,
			time,
			robot: robot.name,
			message: `returning to dock — battery ${robot.battery}%`,
			type: "battery",
		};
	}

	if (robot.status === "charging") {
		return {
			id: eventId,
			time,
			robot: robot.name,
			message: `charging at dock — ${robot.battery}% complete`,
			type: "dock",
		};
	}

	if (roll < 0.5) {
		return {
			id: eventId,
			time,
			robot: robot.name,
			message: `picked up ${randomFrom(LITTER_TYPES)} at ${randomFrom(ZONES)}`,
			type: "pickup",
		};
	}

	if (roll < 0.75) {
		return {
			id: eventId,
			time,
			robot: robot.name,
			message: `completed sweep of ${randomFrom(ZONES)}`,
			type: "complete",
		};
	}

	if (roll < 0.9) {
		return {
			id: eventId,
			time,
			robot: robot.name,
			message: `scanning ${randomFrom(ZONES)} — ${2 + Math.floor(Math.random() * 6)} items detected`,
			type: "alert",
		};
	}

	return {
		id: eventId,
		time,
		robot: robot.name,
		message: `navigating to ${randomFrom(ZONES)}`,
		type: "complete",
	};
}

// --- Components ---

function StatusDot({ status }: { status: Robot["status"] }) {
	const color = statusColor(status);
	return (
		<span className="relative inline-flex h-2.5 w-2.5">
			{status === "active" && (
				<span
					className="absolute inline-flex h-full w-full animate-ping rounded-full opacity-75"
					style={{ backgroundColor: color }}
				/>
			)}
			<span
				className="relative inline-flex h-2.5 w-2.5 rounded-full"
				style={{ backgroundColor: color }}
			/>
		</span>
	);
}

function EventIcon({ type }: { type: ActivityEvent["type"] }) {
	switch (type) {
		case "pickup":
			return <span className="text-cw-green">&#9650;</span>;
		case "battery":
			return <span className="text-yellow-400">&#9889;</span>;
		case "complete":
			return <span className="text-blue-400">&#10003;</span>;
		case "dock":
			return <span className="text-yellow-400">&#9632;</span>;
		case "alert":
			return <span className="text-orange-400">&#9679;</span>;
	}
}

function StatCard({
	label,
	value,
	sub,
}: {
	label: string;
	value: string;
	sub?: string;
}) {
	return (
		<div className="rounded-xl border border-white/10 bg-white/[0.03] px-5 py-4">
			<p className="text-xs uppercase tracking-wider text-gray-500">{label}</p>
			<p className="mt-1 font-mono text-2xl font-bold text-white">{value}</p>
			{sub && <p className="mt-0.5 font-mono text-xs text-gray-500">{sub}</p>}
		</div>
	);
}

function MapView({
	robots,
	selected,
	onSelect,
}: {
	robots: Robot[];
	selected: string | null;
	onSelect: (id: string) => void;
}) {
	return (
		<div className="relative h-full min-h-[400px] overflow-hidden rounded-xl border border-white/10 bg-[#0d1117]">
			{/* Grid background */}
			<div
				className="absolute inset-0 opacity-20"
				style={{
					backgroundImage:
						"radial-gradient(circle, #22c55e 0.5px, transparent 0.5px)",
					backgroundSize: "24px 24px",
				}}
			/>
			{/* Zone labels */}
			<span className="absolute left-[8%] top-[10%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Park Zone A
			</span>
			<span className="absolute left-[40%] top-[5%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Riverside
			</span>
			<span className="absolute left-[65%] top-[25%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Campus N
			</span>
			<span className="absolute left-[25%] top-[50%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Park Zone B
			</span>
			<span className="absolute left-[50%] top-[60%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Beach Front
			</span>
			<span className="absolute left-[8%] top-[68%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Playground
			</span>
			<span className="absolute left-[55%] top-[40%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Sports
			</span>
			<span className="absolute left-[82%] top-[75%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Dock 1
			</span>
			<span className="absolute left-[85%] top-[10%] font-mono text-[10px] uppercase tracking-widest text-gray-600">
				Dock 2
			</span>
			{/* Robots */}
			{robots.map((robot) => {
				const isSelected = selected === robot.id;
				const color = statusColor(robot.status);
				return (
					<button
						key={robot.id}
						onClick={() => onSelect(robot.id)}
						className="absolute z-10 -translate-x-1/2 -translate-y-1/2 transition-transform hover:scale-125"
						style={{ left: `${robot.x}%`, top: `${robot.y}%` }}
						title={`${robot.name} — ${statusLabel(robot.status)}`}
					>
						{/* Pulse ring for active */}
						{robot.status === "active" && (
							<span
								className="absolute -inset-2 animate-ping rounded-full opacity-30"
								style={{ backgroundColor: color }}
							/>
						)}
						{/* Dot */}
						<span
							className="relative block h-3.5 w-3.5 rounded-full border-2"
							style={{
								backgroundColor: color,
								borderColor: isSelected ? "#fff" : color,
								boxShadow: `0 0 8px ${color}88`,
							}}
						/>
						{/* Label */}
						<span
							className="absolute left-1/2 top-5 -translate-x-1/2 whitespace-nowrap rounded px-1 py-0.5 font-mono text-[9px] font-medium"
							style={{
								backgroundColor: isSelected ? color : `${color}33`,
								color: isSelected ? "#000" : color,
							}}
						>
							{robot.name}
						</span>
					</button>
				);
			})}
			{/* Map legend */}
			<div className="absolute bottom-3 left-3 flex gap-4 rounded-lg bg-black/60 px-3 py-2 backdrop-blur-sm">
				{(["active", "charging", "returning"] as const).map((s) => (
					<div key={s} className="flex items-center gap-1.5">
						<span
							className="h-2 w-2 rounded-full"
							style={{ backgroundColor: statusColor(s) }}
						/>
						<span className="font-mono text-[10px] text-gray-400">
							{statusLabel(s)}
						</span>
					</div>
				))}
			</div>
			{/* Map title */}
			<div className="absolute right-3 top-3 rounded-lg bg-black/60 px-3 py-1.5 backdrop-blur-sm">
				<span className="font-mono text-[10px] uppercase tracking-widest text-gray-400">
					Live Fleet Map
				</span>
			</div>
		</div>
	);
}

function ActivityFeed({ events }: { events: ActivityEvent[] }) {
	const feedRef = useRef<HTMLDivElement>(null);

	useEffect(() => {
		if (feedRef.current) {
			feedRef.current.scrollTop = 0;
		}
	}, [events.length]);

	return (
		<div className="flex h-full min-h-[400px] flex-col rounded-xl border border-white/10 bg-white/[0.03]">
			<div className="flex items-center justify-between border-b border-white/10 px-4 py-3">
				<div className="flex items-center gap-2">
					<span className="relative flex h-2 w-2">
						<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
						<span className="relative inline-flex h-2 w-2 rounded-full bg-cw-green" />
					</span>
					<span className="font-mono text-xs uppercase tracking-wider text-gray-400">
						Live Activity
					</span>
				</div>
				<span className="font-mono text-[10px] text-gray-600">
					{events.length} events
				</span>
			</div>
			<div
				ref={feedRef}
				className="flex-1 overflow-y-auto scroll-smooth"
				style={{ maxHeight: "calc(100% - 44px)" }}
			>
				{events.map((event, i) => (
					<div
						key={event.id}
						className={`flex items-start gap-3 border-b border-white/5 px-4 py-3 transition-colors ${
							i === 0 ? "bg-cw-green/5" : "hover:bg-white/[0.02]"
						}`}
					>
						<div className="mt-0.5 text-xs">
							<EventIcon type={event.type} />
						</div>
						<div className="min-w-0 flex-1">
							<p className="text-sm text-gray-300">
								<span className="font-mono font-semibold text-white">
									{event.robot}
								</span>{" "}
								{event.message}
							</p>
							<p className="mt-0.5 font-mono text-[10px] text-gray-600">
								{event.time}
							</p>
						</div>
					</div>
				))}
			</div>
		</div>
	);
}

function RobotCard({
	robot,
	isSelected,
	onClick,
}: {
	robot: Robot;
	isSelected: boolean;
	onClick: () => void;
}) {
	const color = statusColor(robot.status);
	return (
		<button
			onClick={onClick}
			className={`flex-shrink-0 rounded-xl border px-4 py-3 text-left transition-all ${
				isSelected
					? "border-cw-green/50 bg-cw-green/10"
					: "border-white/10 bg-white/[0.03] hover:border-white/20 hover:bg-white/[0.05]"
			}`}
			style={{ width: 180 }}
		>
			<div className="flex items-center justify-between">
				<span className="font-mono text-sm font-bold text-white">
					{robot.name}
				</span>
				<StatusDot status={robot.status} />
			</div>
			<div className="mt-2 flex items-center gap-1.5">
				<svg
					className="h-3 w-3"
					viewBox="0 0 24 24"
					fill="none"
					stroke={batteryColor(robot.battery)}
					strokeWidth={2}
				>
					<rect x="2" y="7" width="16" height="10" rx="1" />
					<rect x="18" y="10" width="3" height="4" rx="0.5" />
					<rect
						x="4"
						y="9"
						width={`${(robot.battery / 100) * 12}`}
						height="6"
						rx="0.5"
						fill={batteryColor(robot.battery)}
						stroke="none"
					/>
				</svg>
				<span
					className="font-mono text-xs font-medium"
					style={{ color: batteryColor(robot.battery) }}
				>
					{robot.battery}%
				</span>
			</div>
			<div className="mt-1.5 flex items-center justify-between">
				<span className="font-mono text-[10px] text-gray-500">
					{robot.items} items
				</span>
				<span
					className="rounded-full px-2 py-0.5 font-mono text-[9px] font-medium"
					style={{
						backgroundColor: `${color}22`,
						color,
					}}
				>
					{statusLabel(robot.status)}
				</span>
			</div>
			<p className="mt-1 truncate font-mono text-[10px] text-gray-600">
				{robot.zone}
			</p>
		</button>
	);
}

// --- Main Page ---

export default function FleetDashboardPage() {
	const [time, setTime] = useState(new Date());
	const [robots, setRobots] = useState<Robot[]>(INITIAL_ROBOTS);
	const [events, setEvents] = useState<ActivityEvent[]>([]);
	const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
	const [totalItems, setTotalItems] = useState(1847);
	const [areaCovered, setAreaCovered] = useState(23.4);
	const eventIdRef = useRef(0);

	// Clock tick
	useEffect(() => {
		const interval = setInterval(() => setTime(new Date()), 1000);
		return () => clearInterval(interval);
	}, []);

	// Generate initial events
	useEffect(() => {
		const initial: ActivityEvent[] = [];
		for (let i = 0; i < 15; i++) {
			eventIdRef.current++;
			initial.push(generateEvent(INITIAL_ROBOTS, eventIdRef.current));
		}
		setEvents(initial);
	}, []);

	// Activity feed: new event every 2-4 seconds
	useEffect(() => {
		const interval = setInterval(() => {
			eventIdRef.current++;
			setEvents((prev) => {
				const event = generateEvent(robots, eventIdRef.current);
				return [event, ...prev].slice(0, 100);
			});
		}, 2000 + Math.random() * 2000);
		return () => clearInterval(interval);
	}, [robots]);

	// Drift robots slightly
	useEffect(() => {
		const interval = setInterval(() => {
			setRobots((prev) =>
				prev.map((r) => {
					if (r.status === "charging") return r;
					const dx = (Math.random() - 0.5) * 2;
					const dy = (Math.random() - 0.5) * 2;
					return {
						...r,
						x: Math.max(5, Math.min(95, r.x + dx)),
						y: Math.max(8, Math.min(90, r.y + dy)),
					};
				}),
			);
		}, 3000);
		return () => clearInterval(interval);
	}, []);

	// Slowly increment stats
	useEffect(() => {
		const interval = setInterval(() => {
			setTotalItems((prev) => prev + Math.floor(Math.random() * 3));
			setAreaCovered((prev) =>
				Math.round((prev + Math.random() * 0.1) * 10) / 10,
			);
		}, 5000);
		return () => clearInterval(interval);
	}, []);

	// Battery drain / charge simulation
	useEffect(() => {
		const interval = setInterval(() => {
			setRobots((prev) =>
				prev.map((r) => {
					if (r.status === "charging") {
						const newBat = Math.min(100, r.battery + 1);
						if (newBat >= 100) {
							return { ...r, battery: newBat, status: "active" };
						}
						return { ...r, battery: newBat };
					}
					if (r.status === "active") {
						const newBat = Math.max(0, r.battery - (Math.random() < 0.3 ? 1 : 0));
						const newItems = r.items + (Math.random() < 0.2 ? 1 : 0);
						if (newBat < 10) {
							return { ...r, battery: newBat, items: newItems, status: "returning" };
						}
						return { ...r, battery: newBat, items: newItems };
					}
					// returning
					const newBat = Math.max(0, r.battery - 1);
					if (newBat <= 5) {
						return { ...r, battery: newBat, status: "charging", zone: `Dock Station ${Math.random() < 0.5 ? 1 : 2}` };
					}
					return { ...r, battery: newBat };
				}),
			);
		}, 8000);
		return () => clearInterval(interval);
	}, []);

	const activeCount = robots.filter(
		(r) => r.status === "active",
	).length;
	const uptime = (
		98 + Math.random() * 1.5
	).toFixed(1);

	const handleSelectRobot = useCallback((id: string) => {
		setSelectedRobot((prev) => (prev === id ? null : id));
	}, []);

	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Inline keyframes for robot drift */}
			<style>{`
				@keyframes fadeSlideIn {
					from { opacity: 0; transform: translateY(-4px); }
					to { opacity: 1; transform: translateY(0); }
				}
			`}</style>

			{/* Top Bar */}
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
								<rect x="3" y="3" width="7" height="7" rx="1" />
								<rect x="14" y="3" width="7" height="7" rx="1" />
								<rect x="3" y="14" width="7" height="7" rx="1" />
								<rect x="14" y="14" width="7" height="7" rx="1" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">
								CleanWalker Fleet Manager
							</h1>
							<p className="font-mono text-[10px] text-gray-500">
								Real-time operations dashboard
							</p>
						</div>
					</div>
					<div className="flex items-center gap-4">
						<div className="hidden items-center gap-2 sm:flex">
							<span className="relative flex h-2 w-2">
								<span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-cw-green opacity-75" />
								<span className="relative inline-flex h-2 w-2 rounded-full bg-cw-green" />
							</span>
							<span className="font-mono text-xs text-cw-green">LIVE</span>
						</div>
						<div className="text-right">
							<p className="font-mono text-sm font-medium text-white">
								{formatTime(time)}
							</p>
							<p className="font-mono text-[10px] text-gray-500">
								{formatDate(time)}
							</p>
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

				{/* Stats Row */}
				<div className="mb-6 grid grid-cols-2 gap-4 lg:grid-cols-4">
					<StatCard
						label="Active Robots"
						value={`${activeCount}/${robots.length}`}
						sub={`${robots.filter((r) => r.status === "charging").length} charging`}
					/>
					<StatCard
						label="Litter Collected"
						value={totalItems.toLocaleString()}
						sub="items today"
					/>
					<StatCard
						label="Area Covered"
						value={`${areaCovered} km²`}
						sub="across all zones"
					/>
					<StatCard
						label="Fleet Uptime"
						value={`${uptime}%`}
						sub="last 24 hours"
					/>
				</div>

				{/* Main Area: Map + Feed */}
				<div className="mb-6 grid gap-6 lg:grid-cols-5">
					<div className="lg:col-span-3">
						<MapView
							robots={robots}
							selected={selectedRobot}
							onSelect={handleSelectRobot}
						/>
					</div>
					<div className="lg:col-span-2">
						<ActivityFeed events={events} />
					</div>
				</div>

				{/* Robot Detail Cards */}
				<div className="mb-6">
					<div className="mb-3 flex items-center justify-between">
						<h2 className="font-mono text-xs uppercase tracking-wider text-gray-500">
							Fleet Status
						</h2>
						<span className="font-mono text-[10px] text-gray-600">
							{robots.length} units deployed
						</span>
					</div>
					<div className="flex gap-3 overflow-x-auto pb-3">
						{robots.map((robot) => (
							<RobotCard
								key={robot.id}
								robot={robot}
								isSelected={selectedRobot === robot.id}
								onClick={() => handleSelectRobot(robot.id)}
							/>
						))}
					</div>
				</div>

				{/* Footer Note */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						This is a simulated demo. In production, data streams from deployed
						CleanWalker robots via MQTT telemetry.
					</p>
					<a
						href="/contact"
						className="mt-3 inline-block rounded-lg bg-cw-green/10 px-6 py-2 font-mono text-xs font-medium text-cw-green transition-colors hover:bg-cw-green/20"
					>
						Request Live Fleet Access
					</a>
				</div>
			</div>
		</div>
	);
}
