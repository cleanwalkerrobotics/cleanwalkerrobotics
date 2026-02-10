// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// --- Timeline segments ---

interface TimeSegment {
	start: number; // hour (0-24)
	end: number;
	type: "patrol" | "collecting" | "charging" | "standby";
	label: string;
}

const SEGMENTS: TimeSegment[] = [
	{ start: 0, end: 4, type: "standby", label: "Night standby" },
	{ start: 4, end: 4.5, type: "charging", label: "Pre-dawn charge top-up" },
	{ start: 4.5, end: 7, type: "patrol", label: "Dawn patrol" },
	{ start: 7, end: 9.5, type: "collecting", label: "Morning collection sweep" },
	{ start: 9.5, end: 10, type: "charging", label: "Mid-morning recharge" },
	{ start: 10, end: 12, type: "patrol", label: "Late morning patrol" },
	{ start: 12, end: 14, type: "collecting", label: "Lunch rush collection" },
	{ start: 14, end: 14.5, type: "charging", label: "Afternoon recharge" },
	{ start: 14.5, end: 17, type: "patrol", label: "Afternoon patrol" },
	{ start: 17, end: 19.5, type: "collecting", label: "Evening collection sweep" },
	{ start: 19.5, end: 20, type: "charging", label: "Evening recharge" },
	{ start: 20, end: 22, type: "patrol", label: "Night patrol (IR mode)" },
	{ start: 22, end: 23, type: "collecting", label: "Late-night sweep" },
	{ start: 23, end: 24, type: "standby", label: "Overnight standby" },
];

const SEGMENT_COLORS: Record<TimeSegment["type"], string> = {
	patrol: "#3b82f6",
	collecting: "#22c55e",
	charging: "#eab308",
	standby: "#6b7280",
};

const SEGMENT_LABELS: Record<TimeSegment["type"], string> = {
	patrol: "Patrol",
	collecting: "Collecting",
	charging: "Charging",
	standby: "Standby",
};

// --- Event log entries (keyed by approximate hour) ---

interface EventEntry {
	time: string;
	message: string;
	type: "info" | "alert" | "success";
}

const EVENTS_POOL: EventEntry[] = [
	{ time: "04:30", message: "Dawn patrol started, Zone A — visibility improving", type: "info" },
	{ time: "05:12", message: "3 items collected near east fountain (bottles)", type: "success" },
	{ time: "05:45", message: "Dew detected on sensors — wipers activated", type: "alert" },
	{ time: "06:15", message: "Zone A sweep complete — 12 items collected", type: "success" },
	{ time: "06:50", message: "Moving to Zone B, jogger traffic increasing", type: "info" },
	{ time: "07:15", message: "Peak foot traffic — switching to pedestrian-aware mode", type: "alert" },
	{ time: "07:42", message: "Dog walker encountered — yielding path, wide berth", type: "info" },
	{ time: "08:10", message: "High-density litter cluster near picnic area — 8 items", type: "success" },
	{ time: "08:30", message: "Bag cassette at 45% — collection rate excellent", type: "info" },
	{ time: "09:05", message: "Zone B complete — transitioning to Zone C", type: "success" },
	{ time: "09:30", message: "Battery 22% — initiating dock return", type: "alert" },
	{ time: "09:45", message: "Docked at charging station — fast charge engaged", type: "info" },
	{ time: "10:00", message: "Charge complete (98%) — resuming patrol", type: "success" },
	{ time: "10:30", message: "Rain starting — switching to wet-surface gait mode", type: "alert" },
	{ time: "11:15", message: "Rain stopped — resuming normal patrol speed", type: "info" },
	{ time: "11:45", message: "Playground area clear — zero litter detected", type: "success" },
	{ time: "12:00", message: "Lunch rush detected — high pedestrian density", type: "alert" },
	{ time: "12:30", message: "Collecting near food court — 15 items in 10 min", type: "success" },
	{ time: "13:00", message: "Bag 18 of 25 — compression ratio holding at 68%", type: "info" },
	{ time: "13:45", message: "Wind gust detected — stabilizing posture", type: "alert" },
	{ time: "14:15", message: "Battery 18% — returning to dock", type: "alert" },
	{ time: "14:30", message: "Docked — fast charge + bag cassette swapped by maintenance", type: "info" },
	{ time: "15:00", message: "Afternoon patrol started — clear skies", type: "success" },
	{ time: "15:30", message: "School group in park — extra cautious mode enabled", type: "alert" },
	{ time: "16:00", message: "Zone D sweep — 6 items collected near basketball court", type: "success" },
	{ time: "16:45", message: "Cyclist near-miss avoided — emergency stop triggered", type: "alert" },
	{ time: "17:15", message: "Evening traffic increasing — pedestrian-aware mode", type: "info" },
	{ time: "17:45", message: "Collection run near main path — 11 items in 15 min", type: "success" },
	{ time: "18:30", message: "Sunset approaching — activating headlights", type: "info" },
	{ time: "19:00", message: "Low light — switching to IR-assisted perception", type: "alert" },
	{ time: "19:30", message: "Battery 25% — docking for evening charge", type: "alert" },
	{ time: "20:00", message: "Night patrol started — IR + LiDAR fusion mode", type: "info" },
	{ time: "20:30", message: "Stray cat detected — pausing collection temporarily", type: "info" },
	{ time: "21:00", message: "Night sweep Zone A — 4 items collected", type: "success" },
	{ time: "21:45", message: "Park mostly empty — increasing patrol speed", type: "info" },
	{ time: "22:15", message: "Final sweep complete — returning to dock", type: "success" },
	{ time: "22:45", message: "Docked — entering overnight standby", type: "info" },
	{ time: "23:00", message: "Daily report uploaded — all systems nominal", type: "success" },
];

// --- Weather states ---

interface WeatherState {
	hour: number;
	icon: string;
	label: string;
	effect: string;
}

const WEATHER_SCHEDULE: WeatherState[] = [
	{ hour: 0, icon: "moon", label: "Clear Night", effect: "IR perception active" },
	{ hour: 5, icon: "sunrise", label: "Dawn", effect: "Visibility improving" },
	{ hour: 7, icon: "sun", label: "Sunny", effect: "Optimal conditions" },
	{ hour: 10, icon: "cloud", label: "Cloudy", effect: "Nominal operation" },
	{ hour: 11, icon: "rain", label: "Light Rain", effect: "Wet-surface gait active" },
	{ hour: 12, icon: "sun", label: "Clearing", effect: "Optimal conditions" },
	{ hour: 16, icon: "cloud", label: "Partly Cloudy", effect: "Nominal operation" },
	{ hour: 18, icon: "sunset", label: "Sunset", effect: "Headlights on" },
	{ hour: 20, icon: "moon", label: "Night", effect: "IR + LiDAR fusion" },
];

function getWeatherAt(hour: number): WeatherState {
	let current = WEATHER_SCHEDULE[0];
	for (const w of WEATHER_SCHEDULE) {
		if (hour >= w.hour) current = w;
	}
	return current;
}

function getSegmentAt(hour: number): TimeSegment {
	for (const seg of SEGMENTS) {
		if (hour >= seg.start && hour < seg.end) return seg;
	}
	return SEGMENTS[0];
}

function formatHour(h: number): string {
	const hours = Math.floor(h);
	const minutes = Math.round((h - hours) * 60);
	return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}`;
}

// --- Component ---

export default function AutonomousOpsPage() {
	const [currentHour, setCurrentHour] = useState(4.5);
	const [visibleEvents, setVisibleEvents] = useState<EventEntry[]>([]);
	const [isRunning, setIsRunning] = useState(true);
	const [dailyStats, setDailyStats] = useState({
		itemsCollected: 0,
		distance: 0,
		uptime: 0,
		batteryCycles: 0,
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

			setCurrentHour((prev) => {
				const newHour = prev + delta * 0.4; // ~24h in 60s
				if (newHour >= 24) {
					setVisibleEvents([]);
					setDailyStats({ itemsCollected: 0, distance: 0, uptime: 0, batteryCycles: 0 });
					return 0;
				}

				// Check for new events to show
				const newEvents = EVENTS_POOL.filter((e) => {
					const eventHour = parseInt(e.time.split(":")[0]) + parseInt(e.time.split(":")[1]) / 60;
					return eventHour <= newHour && eventHour > prev;
				});

				if (newEvents.length > 0) {
					setVisibleEvents((old) => [...newEvents, ...old].slice(0, 12));
				}

				// Update daily stats based on current segment
				const seg = getSegmentAt(newHour);
				setDailyStats((s) => {
					const rate = delta * 0.4;
					return {
						itemsCollected: Math.round(
							s.itemsCollected + (seg.type === "collecting" ? rate * 120 : seg.type === "patrol" ? rate * 30 : 0),
						),
						distance: Math.round((s.distance + (seg.type !== "standby" && seg.type !== "charging" ? rate * 8 : 0)) * 10) / 10,
						uptime: Math.round(((seg.type !== "standby" ? newHour : newHour - (SEGMENTS.find((s2) => s2.type === "standby")?.end ?? 0 - (SEGMENTS.find((s2) => s2.type === "standby")?.start ?? 0))) / newHour) * 100) || 0,
						batteryCycles: seg.type === "charging" ? Math.ceil(newHour / 6) : s.batteryCycles,
					};
				});

				return newHour;
			});

			animRef.current = requestAnimationFrame(animate);
		};

		animRef.current = requestAnimationFrame(animate);
		return () => cancelAnimationFrame(animRef.current);
	}, [isRunning]);

	const toggleRunning = useCallback(() => {
		setIsRunning((prev) => {
			if (prev) lastTimeRef.current = 0;
			return !prev;
		});
	}, []);

	const currentSegment = getSegmentAt(currentHour);
	const currentWeather = getWeatherAt(currentHour);

	// Calculate uptime more accurately
	const activeHours = SEGMENTS.filter((s) => s.type !== "standby").reduce((sum, s) => {
		const segEnd = Math.min(s.end, currentHour);
		const segStart = s.start;
		if (segEnd > segStart) return sum + (segEnd - segStart);
		return sum;
	}, 0);
	const uptimePercent = currentHour > 0 ? Math.round((activeHours / currentHour) * 100) : 0;

	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Header */}
			<header className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-[1600px] items-center justify-between px-6 py-3">
					<div className="flex items-center gap-3">
						<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
							<svg className="h-4 w-4 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
								<circle cx="12" cy="12" r="10" />
								<path d="M12 6v6l4 2" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">24/7 Autonomous Operation</h1>
							<p className="font-mono text-[10px] text-gray-500">A day in the life of CleanWalker</p>
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

				{/* Current Time + Weather + Status Bar */}
				<div className="mb-6 grid gap-4 sm:grid-cols-4">
					<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4 text-center">
						<p className="text-[10px] uppercase tracking-wider text-gray-500">Time</p>
						<p className="font-mono text-3xl font-bold text-white">{formatHour(currentHour)}</p>
					</div>
					<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4 text-center">
						<p className="text-[10px] uppercase tracking-wider text-gray-500">Weather</p>
						<div className="flex items-center justify-center gap-2">
							<WeatherIcon icon={currentWeather.icon} />
							<p className="font-mono text-lg font-bold text-white">{currentWeather.label}</p>
						</div>
						<p className="mt-0.5 font-mono text-[10px] text-gray-500">{currentWeather.effect}</p>
					</div>
					<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4 text-center">
						<p className="text-[10px] uppercase tracking-wider text-gray-500">Mode</p>
						<p className="font-mono text-lg font-bold" style={{ color: SEGMENT_COLORS[currentSegment.type] }}>
							{SEGMENT_LABELS[currentSegment.type]}
						</p>
						<p className="mt-0.5 font-mono text-[10px] text-gray-500">{currentSegment.label}</p>
					</div>
					<div className="rounded-xl border border-white/10 bg-white/[0.03] p-4 text-center">
						<p className="text-[10px] uppercase tracking-wider text-gray-500">Robot</p>
						<p className="font-mono text-lg font-bold text-cw-green">CW-001</p>
						<p className="mt-0.5 font-mono text-[10px] text-gray-500">
							{currentSegment.type === "standby" ? "Sleeping" : currentSegment.type === "charging" ? "At Dock" : "Active"}
						</p>
					</div>
				</div>

				{/* Timeline */}
				<div className="mb-8 rounded-xl border border-white/10 bg-white/[0.03] p-6">
					<div className="mb-3 flex items-center justify-between">
						<span className="font-mono text-xs uppercase tracking-wider text-gray-400">24-Hour Timeline</span>
						{/* Legend */}
						<div className="flex gap-4">
							{(["patrol", "collecting", "charging", "standby"] as const).map((type) => (
								<div key={type} className="flex items-center gap-1.5">
									<div className="h-2.5 w-2.5 rounded-sm" style={{ backgroundColor: SEGMENT_COLORS[type] }} />
									<span className="font-mono text-[10px] text-gray-500">{SEGMENT_LABELS[type]}</span>
								</div>
							))}
						</div>
					</div>

					{/* Timeline bar */}
					<div className="relative">
						<div className="relative h-10 w-full overflow-hidden rounded-lg bg-white/5">
							{SEGMENTS.map((seg, i) => (
								<div
									key={i}
									className="absolute top-0 h-full transition-opacity"
									style={{
										left: `${(seg.start / 24) * 100}%`,
										width: `${((seg.end - seg.start) / 24) * 100}%`,
										backgroundColor: SEGMENT_COLORS[seg.type],
										opacity: currentHour >= seg.start ? (currentHour >= seg.end ? 0.6 : 0.9) : 0.2,
									}}
								/>
							))}

							{/* Current time cursor */}
							<div
								className="absolute top-0 z-10 h-full w-0.5 bg-white shadow-lg shadow-white/50"
								style={{ left: `${(currentHour / 24) * 100}%`, transition: "left 0.1s linear" }}
							>
								<div className="absolute -top-3 left-1/2 -translate-x-1/2">
									<div className="h-0 w-0 border-l-[5px] border-r-[5px] border-t-[6px] border-l-transparent border-r-transparent border-t-white" />
								</div>
							</div>
						</div>

						{/* Hour markers */}
						<div className="relative mt-1 h-4">
							{Array.from({ length: 25 }, (_, i) => (
								<div
									key={i}
									className="absolute text-center"
									style={{ left: `${(i / 24) * 100}%`, transform: "translateX(-50%)" }}
								>
									<span className="font-mono text-[9px] text-gray-600">{i.toString().padStart(2, "0")}</span>
								</div>
							))}
						</div>
					</div>
				</div>

				{/* Event Log + Stats */}
				<div className="mb-8 grid gap-6 lg:grid-cols-3">
					{/* Event Log */}
					<div className="lg:col-span-2">
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
							<div className="mb-4 flex items-center gap-2">
								<svg className="h-3.5 w-3.5 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
									<path strokeLinecap="round" strokeLinejoin="round" d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
								</svg>
								<span className="font-mono text-xs uppercase tracking-wider text-gray-400">Event Log</span>
							</div>

							<div className="h-[320px] space-y-2 overflow-y-auto pr-2">
								{visibleEvents.length === 0 ? (
									<p className="font-mono text-[11px] text-gray-600">Waiting for events...</p>
								) : (
									visibleEvents.map((event, i) => (
										<div
											key={`${event.time}-${i}`}
											className={`flex items-start gap-3 rounded-lg border-l-2 bg-white/[0.02] px-3 py-2 ${
												event.type === "alert"
													? "border-yellow-400/50"
													: event.type === "success"
														? "border-cw-green/50"
														: "border-blue-400/50"
											}`}
											style={{ opacity: i === 0 ? 1 : Math.max(0.4, 1 - i * 0.05) }}
										>
											<span className="mt-0.5 flex-shrink-0">
												{event.type === "alert" ? (
													<svg className="h-3 w-3 text-yellow-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
														<path strokeLinecap="round" strokeLinejoin="round" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4.5c-.77-.833-2.694-.833-3.464 0L3.34 16.5c-.77.833.192 2.5 1.732 2.5z" />
													</svg>
												) : event.type === "success" ? (
													<svg className="h-3 w-3 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
														<path strokeLinecap="round" strokeLinejoin="round" d="M5 13l4 4L19 7" />
													</svg>
												) : (
													<svg className="h-3 w-3 text-blue-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
														<path strokeLinecap="round" strokeLinejoin="round" d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
													</svg>
												)}
											</span>
											<div className="min-w-0 flex-1">
												<p className="font-mono text-[11px] text-gray-300">{event.message}</p>
												<p className="font-mono text-[9px] text-gray-600">{event.time}</p>
											</div>
										</div>
									))
								)}
							</div>
						</div>
					</div>

					{/* Daily Stats */}
					<div className="lg:col-span-1">
						<div className="flex flex-col gap-4">
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-4 flex items-center gap-2">
									<svg className="h-3.5 w-3.5 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
										<path strokeLinecap="round" strokeLinejoin="round" d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
									</svg>
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">Daily Stats</span>
								</div>

								<div className="mb-5">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Items Collected</p>
									<p className="font-mono text-3xl font-bold text-cw-green">{dailyStats.itemsCollected}</p>
								</div>

								<div className="mb-5">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Distance</p>
									<p className="font-mono text-2xl font-bold text-white">
										{dailyStats.distance.toFixed(1)} <span className="text-sm text-gray-500">km</span>
									</p>
								</div>

								<div className="mb-5">
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Uptime</p>
									<div className="mt-1 flex items-center gap-3">
										<p className="font-mono text-2xl font-bold text-white">{uptimePercent}%</p>
										<div className="h-2 flex-1 overflow-hidden rounded-full bg-white/10">
											<div
												className="h-full rounded-full bg-cw-green transition-all duration-300"
												style={{ width: `${uptimePercent}%` }}
											/>
										</div>
									</div>
								</div>

								<div>
									<p className="text-[10px] uppercase tracking-wider text-gray-500">Battery Cycles</p>
									<p className="font-mono text-2xl font-bold text-white">{dailyStats.batteryCycles}</p>
									<div className="mt-1 flex gap-1">
										{Array.from({ length: 4 }, (_, i) => (
											<div
												key={i}
												className="h-2 flex-1 rounded-sm"
												style={{
													backgroundColor: i < dailyStats.batteryCycles ? "#eab308" : "rgba(255,255,255,0.1)",
												}}
											/>
										))}
									</div>
								</div>
							</div>

							{/* Weather Forecast */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-5">
								<div className="mb-3 flex items-center gap-2">
									<WeatherIcon icon={currentWeather.icon} />
									<span className="font-mono text-xs uppercase tracking-wider text-gray-400">Weather</span>
								</div>
								<div className="space-y-2">
									{WEATHER_SCHEDULE.filter((w) => w.hour >= Math.floor(currentHour) - 1).slice(0, 5).map((w, i) => (
										<div
											key={w.hour}
											className={`flex items-center gap-3 rounded px-2 py-1.5 ${
												w.hour <= currentHour && (WEATHER_SCHEDULE[WEATHER_SCHEDULE.indexOf(w) + 1]?.hour ?? 24) > currentHour
													? "bg-white/5"
													: ""
											}`}
										>
											<span className="w-10 font-mono text-[10px] text-gray-500">{formatHour(w.hour)}</span>
											<WeatherIcon icon={w.icon} size="sm" />
											<span className="font-mono text-[11px] text-gray-400">{w.label}</span>
										</div>
									))}
								</div>
							</div>
						</div>
					</div>
				</div>

				{/* How It Works */}
				<section className="mb-8">
					<h2 className="mb-6 text-2xl font-bold text-white">How 24/7 Operation Works</h2>
					<div className="grid gap-6 md:grid-cols-3">
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M13 10V3L4 14h7v7l9-11h-7z" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">Smart Charging</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								CleanWalker autonomously returns to its charging dock when battery drops below 20%.
								Fast-charge technology provides 80% capacity in 25 minutes. The robot schedules
								charge cycles during low-traffic periods to maximize active cleaning time — typically
								achieving 18+ hours of daily operation with 3-4 short charge sessions.
							</p>
						</div>

						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<circle cx="12" cy="12" r="10" />
									<path d="M12 6v6l4 2" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">Adaptive Scheduling</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								The robot learns daily patterns — when foot traffic peaks, when litter accumulates
								fastest, and when areas are quiet. It adjusts patrol routes and collection intensity
								accordingly. During lunch rush it focuses on high-traffic food areas; at dawn it
								sweeps the entire park while paths are empty.
							</p>
						</div>

						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<div className="mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
								<svg className="h-6 w-6 text-cw-green" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5}>
									<path d="M20.354 15.354A9 9 0 018.646 3.646 9.003 9.003 0 0012 21a9.003 9.003 0 008.354-5.646z" />
								</svg>
							</div>
							<h3 className="mb-2 text-lg font-semibold text-white">Night Operation</h3>
							<p className="text-sm leading-relaxed text-gray-400">
								After dark, CleanWalker switches to IR-assisted perception with LiDAR fusion.
								Headlights illuminate the immediate work area for the gripper camera while
								long-range LiDAR handles navigation. The robot operates at reduced speed during
								night hours, prioritizing thoroughness over coverage area.
							</p>
						</div>
					</div>
				</section>

				{/* Footer */}
				<div className="border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						This is a simulated demo. In production, operation timelines and event logs stream from
						deployed CleanWalker robots via MQTT telemetry and the fleet management dashboard.
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

// --- Weather Icon ---

function WeatherIcon({ icon, size = "md" }: { icon: string; size?: "sm" | "md" }) {
	const cls = size === "sm" ? "h-3.5 w-3.5" : "h-5 w-5";

	switch (icon) {
		case "sun":
			return (
				<svg className={`${cls} text-yellow-400`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<circle cx="12" cy="12" r="5" />
					<path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
				</svg>
			);
		case "cloud":
			return (
				<svg className={`${cls} text-gray-400`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M18 10h-1.26A8 8 0 109 20h9a5 5 0 000-10z" />
				</svg>
			);
		case "rain":
			return (
				<svg className={`${cls} text-blue-400`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M16 13v8M8 13v8M12 15v8M20 8h-1.26A8 8 0 107 16h13a5 5 0 000-10z" />
				</svg>
			);
		case "moon":
			return (
				<svg className={`${cls} text-indigo-300`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M21 12.79A9 9 0 1111.21 3 7 7 0 0021 12.79z" />
				</svg>
			);
		case "sunrise":
			return (
				<svg className={`${cls} text-orange-400`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M17 18a5 5 0 00-10 0M12 2v7M4.22 10.22l1.42 1.42M1 18h2M21 18h2M18.36 11.64l1.42-1.42M23 22H1M8 6l4-4 4 4" />
				</svg>
			);
		case "sunset":
			return (
				<svg className={`${cls} text-orange-500`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2}>
					<path d="M17 18a5 5 0 00-10 0M12 9v7M4.22 10.22l1.42 1.42M1 18h2M21 18h2M18.36 11.64l1.42-1.42M23 22H1M16 5l-4 4-4-4" />
				</svg>
			);
		default:
			return null;
	}
}
