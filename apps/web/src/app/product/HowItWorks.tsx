// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useEffect, useRef, useState } from "react";

const steps = [
	{
		number: "01",
		label: "DETECT",
		title: "AI Identifies Litter",
		desc: "AI-powered vision scans the environment in real-time, detecting 50+ litter types — bottles, cans, wrappers, cigarette butts, and more.",
	},
	{
		number: "02",
		label: "NAVIGATE",
		title: "Plans Optimal Path",
		desc: "Stereo depth cameras and LiDAR build a 3D map. The robot plans efficient routes, avoids obstacles, and traverses complex terrain.",
	},
	{
		number: "03",
		label: "COLLECT",
		title: "Precision Pickup",
		desc: "A 2-DOF robotic arm with silicone-tipped gripper precisely picks up items from 5g to 500g — from cigarette butts to water bottles.",
	},
	{
		number: "04",
		label: "DISPOSE",
		title: "Autonomous Disposal",
		desc: "Collected litter is deposited into the bag cassette system. When full, the robot returns to dock for bag replacement and recharging.",
	},
];

function DetectSVG({ active }: { active: boolean }) {
	return (
		<svg viewBox="0 0 200 160" className="h-full w-full" aria-hidden="true">
			{/* Ground line */}
			<line
				x1="20"
				y1="130"
				x2="180"
				y2="130"
				stroke="#374151"
				strokeWidth="1"
				strokeDasharray="4 4"
			/>
			{/* Robot body */}
			<rect
				x="30"
				y="90"
				width="40"
				height="30"
				rx="4"
				fill="#1a1f2e"
				stroke="#22c55e"
				strokeWidth="1.5"
			/>
			{/* Robot legs */}
			<line x1="38" y1="120" x2="35" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="48" y1="120" x2="45" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="58" y1="120" x2="55" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="68" y1="120" x2="65" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			{/* Sensor eye */}
			<circle cx="60" cy="100" r="3" fill="#22c55e" opacity={active ? 1 : 0.3}>
				{active && (
					<animate
						attributeName="opacity"
						values="1;0.4;1"
						dur="1.5s"
						repeatCount="indefinite"
					/>
				)}
			</circle>
			{/* Scanning beam */}
			<polygon
				points="63,100 170,70 170,120"
				fill="#22c55e"
				className={active ? "hiw-scan-beam" : ""}
				opacity={active ? 0.08 : 0}
			/>
			<line
				x1="63"
				y1="100"
				x2="170"
				y2="70"
				stroke="#22c55e"
				strokeWidth="0.5"
				opacity={active ? 0.4 : 0}
			/>
			<line
				x1="63"
				y1="100"
				x2="170"
				y2="120"
				stroke="#22c55e"
				strokeWidth="0.5"
				opacity={active ? 0.4 : 0}
			/>
			{/* Scanning sweep line */}
			{active && (
				<line
					x1="63"
					y1="100"
					x2="170"
					y2="95"
					stroke="#22c55e"
					strokeWidth="1"
					opacity="0.6"
					className="hiw-sweep-line"
				/>
			)}
			{/* Litter items */}
			<rect x="120" y="118" width="12" height="16" rx="2" fill="#374151" stroke="#4b5563" strokeWidth="0.5" />
			<circle cx="150" cy="126" r="5" fill="#374151" stroke="#4b5563" strokeWidth="0.5" />
			<rect x="95" y="122" width="14" height="8" rx="1" fill="#374151" stroke="#4b5563" strokeWidth="0.5" />
			{/* Detection boxes */}
			{active && (
				<>
					<rect
						x="114"
						y="112"
						width="24"
						height="24"
						rx="2"
						fill="none"
						stroke="#22c55e"
						strokeWidth="1.5"
						className="hiw-detect-box hiw-detect-box-1"
					/>
					<rect
						x="141"
						y="117"
						width="18"
						height="18"
						rx="2"
						fill="none"
						stroke="#22c55e"
						strokeWidth="1.5"
						className="hiw-detect-box hiw-detect-box-2"
					/>
					<rect
						x="89"
						y="117"
						width="24"
						height="16"
						rx="2"
						fill="none"
						stroke="#22c55e"
						strokeWidth="1.5"
						className="hiw-detect-box hiw-detect-box-3"
					/>
				</>
			)}
		</svg>
	);
}

function NavigateSVG({ active }: { active: boolean }) {
	return (
		<svg viewBox="0 0 200 160" className="h-full w-full" aria-hidden="true">
			{/* Ground */}
			<line
				x1="20"
				y1="130"
				x2="180"
				y2="130"
				stroke="#374151"
				strokeWidth="1"
				strokeDasharray="4 4"
			/>
			{/* Obstacles */}
			<rect x="80" y="100" width="16" height="30" rx="2" fill="#1a1f2e" stroke="#374151" strokeWidth="1" />
			<rect x="120" y="85" width="12" height="45" rx="2" fill="#1a1f2e" stroke="#374151" strokeWidth="1" />
			{/* Robot body */}
			<rect
				x="20"
				y="100"
				width="32"
				height="22"
				rx="4"
				fill="#1a1f2e"
				stroke="#22c55e"
				strokeWidth="1.5"
			/>
			{/* Robot legs */}
			<line x1="27" y1="122" x2="25" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="35" y1="122" x2="33" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="43" y1="122" x2="41" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="49" y1="122" x2="47" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			{/* Target litter */}
			<rect x="162" y="120" width="12" height="14" rx="2" fill="#374151" stroke="#22c55e" strokeWidth="1" strokeDasharray="2 2" />
			{/* Navigation path */}
			<path
				d="M52,111 C65,111 70,85 80,80 C90,75 100,80 105,85 C110,90 115,75 125,70 C135,65 145,90 155,100 C158,103 162,115 168,120"
				fill="none"
				stroke="#22c55e"
				strokeWidth="1.5"
				strokeDasharray="6 4"
				opacity={active ? 0.8 : 0}
				className={active ? "hiw-path-draw" : ""}
			/>
			{/* Waypoint dots along path */}
			{active && (
				<>
					<circle cx="80" cy="80" r="2.5" fill="#22c55e" className="hiw-waypoint hiw-waypoint-1" />
					<circle cx="105" cy="85" r="2.5" fill="#22c55e" className="hiw-waypoint hiw-waypoint-2" />
					<circle cx="125" cy="70" r="2.5" fill="#22c55e" className="hiw-waypoint hiw-waypoint-3" />
					<circle cx="155" cy="100" r="2.5" fill="#22c55e" className="hiw-waypoint hiw-waypoint-4" />
				</>
			)}
			{/* Moving dot */}
			{active && (
				<circle r="3.5" fill="#22c55e" opacity="0.9">
					<animateMotion
						dur="3s"
						repeatCount="indefinite"
						path="M52,111 C65,111 70,85 80,80 C90,75 100,80 105,85 C110,90 115,75 125,70 C135,65 145,90 155,100 C158,103 162,115 168,120"
					/>
				</circle>
			)}
		</svg>
	);
}

function CollectSVG({ active }: { active: boolean }) {
	return (
		<svg viewBox="0 0 200 160" className="h-full w-full" aria-hidden="true">
			{/* Ground */}
			<line
				x1="20"
				y1="130"
				x2="180"
				y2="130"
				stroke="#374151"
				strokeWidth="1"
				strokeDasharray="4 4"
			/>
			{/* Robot body */}
			<rect
				x="60"
				y="85"
				width="40"
				height="30"
				rx="4"
				fill="#1a1f2e"
				stroke="#22c55e"
				strokeWidth="1.5"
			/>
			{/* Robot legs */}
			<line x1="68" y1="115" x2="65" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="78" y1="115" x2="75" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="88" y1="115" x2="85" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="98" y1="115" x2="95" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			{/* Arm segments */}
			<g className={active ? "hiw-arm-animate" : ""}>
				{/* Upper arm */}
				<line
					x1="100"
					y1="90"
					x2="130"
					y2="80"
					stroke="#22c55e"
					strokeWidth="2"
					strokeLinecap="round"
				/>
				{/* Lower arm */}
				<line
					x1="130"
					y1="80"
					x2="140"
					y2={active ? "120" : "90"}
					stroke="#22c55e"
					strokeWidth="2"
					strokeLinecap="round"
					className={active ? "hiw-arm-lower" : ""}
				/>
				{/* Gripper */}
				<g className={active ? "hiw-gripper" : ""} transform={active ? undefined : "translate(140, 85)"}>
					{active ? (
						<>
							<line x1="136" y1="118" x2="133" y2="128" stroke="#22c55e" strokeWidth="1.5" strokeLinecap="round" className="hiw-gripper-l" />
							<line x1="144" y1="118" x2="147" y2="128" stroke="#22c55e" strokeWidth="1.5" strokeLinecap="round" className="hiw-gripper-r" />
						</>
					) : (
						<>
							<line x1="-4" y1="-2" x2="-7" y2="8" stroke="#22c55e" strokeWidth="1.5" strokeLinecap="round" />
							<line x1="4" y1="-2" x2="7" y2="8" stroke="#22c55e" strokeWidth="1.5" strokeLinecap="round" />
						</>
					)}
				</g>
			</g>
			{/* Joint circles */}
			<circle cx="100" cy="90" r="2.5" fill="#0a0f0d" stroke="#22c55e" strokeWidth="1" />
			<circle cx="130" cy="80" r="2.5" fill="#0a0f0d" stroke="#22c55e" strokeWidth="1" />
			{/* Litter item (bottle) */}
			<rect
				x="134"
				y="116"
				width="12"
				height="16"
				rx="2"
				fill="#374151"
				stroke="#4b5563"
				strokeWidth="0.5"
				className={active ? "hiw-item-lift" : ""}
			/>
			{/* Pickup indicator */}
			{active && (
				<circle cx="140" cy="124" r="16" fill="none" stroke="#22c55e" strokeWidth="0.5" className="hiw-pickup-ring" />
			)}
		</svg>
	);
}

function DisposeSVG({ active }: { active: boolean }) {
	return (
		<svg viewBox="0 0 200 160" className="h-full w-full" aria-hidden="true">
			{/* Ground */}
			<line
				x1="20"
				y1="130"
				x2="180"
				y2="130"
				stroke="#374151"
				strokeWidth="1"
				strokeDasharray="4 4"
			/>
			{/* Robot body */}
			<rect
				x="40"
				y="85"
				width="40"
				height="30"
				rx="4"
				fill="#1a1f2e"
				stroke="#22c55e"
				strokeWidth="1.5"
			/>
			{/* Robot legs */}
			<line x1="48" y1="115" x2="45" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="58" y1="115" x2="55" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="68" y1="115" x2="65" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			<line x1="78" y1="115" x2="75" y2="130" stroke="#22c55e" strokeWidth="1.5" />
			{/* Bag cassette on robot */}
			<rect
				x="50"
				y="88"
				width="20"
				height="24"
				rx="2"
				fill="#0a0f0d"
				stroke="#374151"
				strokeWidth="1"
			/>
			{/* Bag opening */}
			<rect x="52" y="88" width="16" height="4" rx="1" fill="#22c55e" opacity="0.3" />
			{/* Fill level (animated) */}
			<rect
				x="52"
				y={active ? "96" : "110"}
				width="16"
				rx="1"
				fill="#22c55e"
				opacity="0.2"
				className={active ? "hiw-bag-fill" : ""}
			>
				{active && (
					<animate
						attributeName="height"
						values="2;14"
						dur="2.5s"
						fill="freeze"
						repeatCount="1"
					/>
				)}
				{active && (
					<animate
						attributeName="y"
						values="110;98"
						dur="2.5s"
						fill="freeze"
						repeatCount="1"
					/>
				)}
			</rect>
			{/* Dropping item animation */}
			{active && (
				<rect
					x="56"
					y="60"
					width="8"
					height="10"
					rx="1.5"
					fill="#374151"
					stroke="#4b5563"
					strokeWidth="0.5"
					className="hiw-item-drop"
				/>
			)}
			{/* Fill level indicator lines */}
			<line x1="73" y1="96" x2="78" y2="96" stroke="#374151" strokeWidth="0.5" />
			<line x1="73" y1="102" x2="78" y2="102" stroke="#374151" strokeWidth="0.5" />
			<line x1="73" y1="108" x2="78" y2="108" stroke="#374151" strokeWidth="0.5" />
			{/* Capacity indicator */}
			{active && (
				<>
					<text x="82" y="100" fill="#22c55e" fontSize="8" fontFamily="monospace" className="hiw-capacity-text">
						<animate attributeName="opacity" values="0;1" dur="2s" fill="freeze" />
					</text>
					{/* Percentage text */}
					<text x="82" y="100" fill="#22c55e" fontSize="8" fontFamily="monospace" opacity="0">
						75%
						<animate attributeName="opacity" values="0;0;1" dur="2.5s" fill="freeze" />
					</text>
				</>
			)}
			{/* Collected count display */}
			<g transform="translate(120, 80)">
				<rect x="0" y="0" width="50" height="44" rx="4" fill="#1a1f2e" stroke="#374151" strokeWidth="0.5" />
				<text x="25" y="16" fill="#6b7280" fontSize="6" textAnchor="middle" fontFamily="sans-serif">
					COLLECTED
				</text>
				<text
					x="25"
					y="36"
					fill="#22c55e"
					fontSize="16"
					textAnchor="middle"
					fontFamily="monospace"
					fontWeight="bold"
				>
					{active ? (
						<>
							47
							<animate attributeName="opacity" values="0;1" dur="1s" fill="freeze" />
						</>
					) : (
						<tspan opacity="0.3">--</tspan>
					)}
				</text>
			</g>
		</svg>
	);
}

const svgComponents = [DetectSVG, NavigateSVG, CollectSVG, DisposeSVG];

export default function HowItWorks() {
	const [activeSteps, setActiveSteps] = useState<Set<number>>(new Set());
	const stepRefs = useRef<(HTMLDivElement | null)[]>([]);
	const sectionRef = useRef<HTMLElement>(null);

	useEffect(() => {
		const observers: IntersectionObserver[] = [];

		stepRefs.current.forEach((el, index) => {
			if (!el) return;

			const observer = new IntersectionObserver(
				(entries) => {
					entries.forEach((entry) => {
						if (entry.isIntersecting) {
							setActiveSteps((prev) => new Set([...prev, index]));
						}
					});
				},
				{ threshold: 0.4, rootMargin: "0px 0px -10% 0px" },
			);

			observer.observe(el);
			observers.push(observer);
		});

		return () => observers.forEach((o) => o.disconnect());
	}, []);

	return (
		<section ref={sectionRef} className="bg-cw-dark px-6 py-24 md:py-32">
			<style>{`
				/* Scan beam pulse */
				.hiw-scan-beam {
					animation: hiw-beam-pulse 2s ease-in-out infinite;
				}
				@keyframes hiw-beam-pulse {
					0%, 100% { opacity: 0.06; }
					50% { opacity: 0.14; }
				}

				/* Sweep line rotation */
				.hiw-sweep-line {
					transform-origin: 63px 100px;
					animation: hiw-sweep 2.5s ease-in-out infinite;
				}
				@keyframes hiw-sweep {
					0% { transform: rotate(-12deg); }
					50% { transform: rotate(12deg); }
					100% { transform: rotate(-12deg); }
				}

				/* Detection boxes appearing */
				.hiw-detect-box {
					opacity: 0;
					animation: hiw-detect-appear 0.4s ease-out forwards;
				}
				.hiw-detect-box-1 { animation-delay: 0.6s; }
				.hiw-detect-box-2 { animation-delay: 0.9s; }
				.hiw-detect-box-3 { animation-delay: 1.2s; }
				@keyframes hiw-detect-appear {
					0% { opacity: 0; transform: scale(1.3); }
					100% { opacity: 0.9; transform: scale(1); }
				}

				/* Path drawing */
				.hiw-path-draw {
					stroke-dasharray: 300;
					stroke-dashoffset: 300;
					animation: hiw-draw-path 2s ease-out forwards;
				}
				@keyframes hiw-draw-path {
					to { stroke-dashoffset: 0; }
				}

				/* Waypoints appearing */
				.hiw-waypoint {
					opacity: 0;
					animation: hiw-wp-pop 0.3s ease-out forwards;
				}
				.hiw-waypoint-1 { animation-delay: 0.5s; }
				.hiw-waypoint-2 { animation-delay: 1.0s; }
				.hiw-waypoint-3 { animation-delay: 1.3s; }
				.hiw-waypoint-4 { animation-delay: 1.7s; }
				@keyframes hiw-wp-pop {
					0% { opacity: 0; r: 0; }
					100% { opacity: 0.7; r: 2.5; }
				}

				/* Arm animation */
				.hiw-arm-lower {
					transition: all 1s ease-in-out;
				}
				.hiw-arm-animate {
					animation: hiw-arm-reach 2.5s ease-in-out forwards;
				}

				/* Item lifting */
				.hiw-item-lift {
					animation: hiw-lift 2.5s ease-in-out forwards;
				}
				@keyframes hiw-lift {
					0%, 50% { transform: translateY(0); }
					80%, 100% { transform: translateY(-20px); }
				}

				/* Pickup ring */
				.hiw-pickup-ring {
					animation: hiw-ring-pulse 1.5s ease-out forwards;
				}
				@keyframes hiw-ring-pulse {
					0% { r: 6; opacity: 0.6; }
					100% { r: 22; opacity: 0; }
				}

				/* Gripper fingers close */
				.hiw-gripper-l {
					animation: hiw-grip-l 2.5s ease-in-out forwards;
				}
				.hiw-gripper-r {
					animation: hiw-grip-r 2.5s ease-in-out forwards;
				}
				@keyframes hiw-grip-l {
					0%, 40% { transform: translateX(0); }
					60%, 100% { transform: translateX(2px); }
				}
				@keyframes hiw-grip-r {
					0%, 40% { transform: translateX(0); }
					60%, 100% { transform: translateX(-2px); }
				}

				/* Item dropping into bag */
				.hiw-item-drop {
					animation: hiw-drop 2s ease-in forwards;
				}
				@keyframes hiw-drop {
					0% { transform: translateY(0); opacity: 1; }
					60% { transform: translateY(30px); opacity: 1; }
					80% { transform: translateY(30px); opacity: 0.5; }
					100% { transform: translateY(30px); opacity: 0; }
				}

				/* Step card entrance */
				.hiw-step-card {
					opacity: 0;
					transform: translateY(32px);
					transition: opacity 0.6s ease-out, transform 0.6s ease-out;
				}
				.hiw-step-card.hiw-visible {
					opacity: 1;
					transform: translateY(0);
				}

				/* Connecting line fill */
				.hiw-connector-fill {
					transition: height 0.8s ease-out;
				}

				/* Number glow */
				.hiw-number-active {
					animation: hiw-num-glow 2s ease-in-out infinite;
				}
				@keyframes hiw-num-glow {
					0%, 100% { box-shadow: 0 0 0 0 rgba(34, 197, 94, 0); }
					50% { box-shadow: 0 0 20px 4px rgba(34, 197, 94, 0.15); }
				}
			`}</style>

			<div className="mx-auto max-w-7xl">
				{/* Header */}
				<div className="text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						How It Works
					</h2>
					<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-400">
						Detect, navigate, collect, and dispose — all autonomously.
					</p>
				</div>

				{/* Steps */}
				<div className="mt-20 space-y-16 md:space-y-0 md:grid md:grid-cols-[auto_1fr] md:gap-x-0 md:gap-y-0">
					{steps.map((step, i) => {
						const SVGComponent = svgComponents[i];
						const isActive = activeSteps.has(i);

						return (
							<div
								key={step.number}
								className="contents md:contents"
							>
								{/* Vertical connector (desktop) */}
								<div className="hidden md:flex md:flex-col md:items-center md:w-16">
									{/* Number circle */}
									<div
										className={`relative flex h-12 w-12 items-center justify-center rounded-full border-2 transition-all duration-700 ${
											isActive
												? "border-cw-green bg-cw-green/10 hiw-number-active"
												: "border-gray-700 bg-cw-dark"
										}`}
									>
										<span
											className={`text-sm font-bold font-mono transition-colors duration-700 ${
												isActive ? "text-cw-green" : "text-gray-600"
											}`}
										>
											{step.number}
										</span>
									</div>
									{/* Connector line */}
									{i < steps.length - 1 && (
										<div className="relative w-0.5 flex-1 min-h-[200px] bg-gray-800">
											<div
												className="absolute inset-x-0 top-0 w-full bg-cw-green/40 hiw-connector-fill"
												style={{ height: isActive ? "100%" : "0%" }}
											/>
										</div>
									)}
								</div>

								{/* Step card */}
								<div
									ref={(el) => { stepRefs.current[i] = el; }}
									className={`hiw-step-card ${isActive ? "hiw-visible" : ""} md:pb-16 md:pl-8`}
								>
									{/* Mobile number */}
									<div className="mb-4 flex items-center gap-3 md:hidden">
										<div
											className={`flex h-10 w-10 items-center justify-center rounded-full border-2 transition-all duration-700 ${
												isActive
													? "border-cw-green bg-cw-green/10"
													: "border-gray-700"
											}`}
										>
											<span
												className={`text-xs font-bold font-mono transition-colors duration-700 ${
													isActive ? "text-cw-green" : "text-gray-600"
												}`}
											>
												{step.number}
											</span>
										</div>
										<span
											className={`text-xs font-semibold uppercase tracking-widest transition-colors duration-700 ${
												isActive ? "text-cw-green" : "text-gray-600"
											}`}
										>
											{step.label}
										</span>
									</div>

									<div className="rounded-2xl border border-gray-800 bg-cw-charcoal/50 p-6 md:p-8">
										<div className="grid gap-6 md:grid-cols-2 md:gap-8 items-center">
											{/* SVG illustration */}
											<div className="order-1 md:order-2 aspect-[5/4] w-full rounded-xl bg-cw-dark/60 p-4">
												<SVGComponent active={isActive} />
											</div>

											{/* Text content */}
											<div className="order-2 md:order-1">
												<div
													className={`mb-2 hidden text-xs font-semibold uppercase tracking-widest transition-colors duration-700 md:block ${
														isActive ? "text-cw-green" : "text-gray-600"
													}`}
												>
													{step.label}
												</div>
												<h3 className="text-xl font-bold text-white md:text-2xl">
													{step.title}
												</h3>
												<p className="mt-3 leading-relaxed text-gray-400">
													{step.desc}
												</p>
											</div>
										</div>
									</div>
								</div>
							</div>
						);
					})}
				</div>
			</div>
		</section>
	);
}
