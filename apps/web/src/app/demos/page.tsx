// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import Image from "next/image";

export const metadata: Metadata = {
	title: "Demos",
	description:
		"See CleanWalker's autonomous litter collection technology in action. AI perception, autonomous navigation, and fleet management demos.",
	openGraph: {
		title: "Technology Demos — CleanWalker Robotics",
		description:
			"Interactive demos of CleanWalker's AI litter detection, autonomous navigation, quadrupedal locomotion, and fleet management technology.",
	},
};

const FEATURED_SLUGS = new Set(["litter-detection", "3d-robot-viewer", "simulation"]);

const demos = [
	{
		title: "Real-Time AI Litter Detection",
		status: "live",
		slug: "litter-detection",
		description:
			"Our perception system identifies and classifies 50+ types of litter in real-time — bottles, cans, wrappers, cigarette butts, and more. Runs entirely in your browser, no server needed.",
		placeholder: "Upload an image or use your camera to see AI detection live",
		tags: ["Computer Vision", "Real-Time", "On-Device AI"],
	},
	{
		title: "3D Robot Viewer",
		status: "live",
		slug: "3d-robot-viewer",
		description:
			"Explore the CleanWalker CW-1 quadruped in interactive 3D. Spin, zoom, and inspect the 12-DOF robot model built from our engineering URDF — 23 links, 22 joints, accurate dimensions and materials.",
		placeholder: "Interactive 3D model — rotate and zoom to explore",
		tags: ["Three.js", "URDF", "3D Model"],
		disclaimer: "Concept visualization — not final production design",
	},
	{
		title: "Robot Simulation",
		status: "live",
		slug: "simulation",
		description:
			"Watch a CW-1 robot autonomously patrol a park, detect and collect litter, fill its bag, and return to the dock for a bag swap. The full operational cycle, visualized.",
		placeholder: "Live simulation — autonomous litter collection in a park",
		tags: ["Simulation", "Real-Time", "Autonomy"],
	},
	{
		title: "Quadrupedal Locomotion",
		status: "live",
		slug: "quadrupedal-locomotion",
		description:
			"See how our robot traverses curbs, grass, gravel, and uneven terrain that wheeled robots can't handle. Adaptive gait control for any surface.",
		placeholder: "Physics simulation — terrain traversal across multiple surface types",
		tags: ["Robotics", "Gait Control", "All-Terrain"],
	},
	{
		title: "Pick & Compact",
		status: "live",
		slug: "pick-and-compact",
		description:
			"The full litter collection cycle: detect, approach, pick up with articulated gripper, deposit into collection bag, compress, and continue. Sealed bags dropped at curbside for municipal pickup.",
		placeholder: "Simulation demo — full litter collection pipeline",
		tags: ["Manipulation", "Gripper", "Bag Cassette"],
	},
	{
		title: "Fleet Dashboard",
		status: "live",
		slug: "fleet-dashboard",
		description:
			"Real-time fleet management for city-wide deployments. Monitor every robot, track coverage, analyze litter patterns, and optimize routes — all from one dashboard.",
		placeholder: "Interactive fleet management dashboard preview",
		tags: ["Dashboard", "Analytics", "Fleet Management"],
	},
	{
		title: "24/7 Autonomous Operation",
		status: "live",
		slug: "autonomous-ops",
		description:
			"A day in the life of CleanWalker: dawn patrol, peak-hour navigation around pedestrians, self-charging, night operation with IR perception. Fully autonomous.",
		placeholder: "Time-lapse simulation — 24 hours of autonomous park cleaning",
		tags: ["Autonomy", "Self-Charging", "Night Vision"],
	},
	{
		title: "Route Planning",
		status: "live",
		slug: "route-planning",
		description:
			"Watch how CleanWalker plans optimal coverage routes across parks and campuses. Compare boustrophedon, boundary-first, and hotspot-priority strategies with multi-robot fleets.",
		placeholder: "Interactive route planning simulation — toggle strategies and fleet size",
		tags: ["Path Planning", "Fleet Optimization", "Coverage"],
	},
	{
		title: "Cost Savings Calculator",
		status: "live",
		slug: "cost-calculator",
		description:
			"Calculate your facility's ROI from deploying CleanWalker. Input your current labor costs and see annual savings, payback period, and 5-year projections — instantly.",
		placeholder: "Interactive ROI calculator — estimate your savings",
		tags: ["ROI", "Cost Analysis", "Calculator"],
	},
];

const featuredDemos = demos.filter((d) => FEATURED_SLUGS.has(d.slug));
const otherDemos = demos.filter((d) => !FEATURED_SLUGS.has(d.slug));

export default function DemosPage() {
	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Hero — Subtle background image */}
			<section className="relative px-6 py-24 text-center">
				<Image
					src="/renders/v3/hero-park.webp"
					alt="CleanWalker robot in a park setting"
					fill
					className="object-cover opacity-15"
					priority
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-gradient-to-b from-cw-dark/40 to-cw-dark" />
				<div className="relative z-10 mx-auto max-w-4xl">
					<span className="mb-4 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1 text-sm text-cw-green backdrop-blur-sm">
						Technology Demos
					</span>
					<h1 className="mt-4 text-4xl font-bold tracking-tight text-white md:text-5xl">
						See the Technology in Action
					</h1>
					<p className="mt-6 text-lg leading-relaxed text-gray-400">
						From AI-powered litter detection to autonomous navigation across complex terrain.
						Explore the capabilities that make CleanWalker the future of public space maintenance.
					</p>
				</div>
			</section>

			{/* Featured Demos */}
			<section className="px-6 pb-16">
				<div className="mx-auto max-w-7xl">
					<div className="mb-8 flex items-center gap-3">
						<span className="rounded-full bg-cw-green/10 px-3 py-1 text-xs font-semibold tracking-wide text-cw-green">
							Featured
						</span>
						<div className="h-px flex-1 bg-white/10" />
					</div>
					<div className="grid gap-8 md:grid-cols-3">
						{featuredDemos.map((demo) => (
							<a
								key={demo.title}
								href={`/demos/${demo.slug}`}
								className="group relative overflow-hidden rounded-2xl border border-cw-green/20 bg-gradient-to-b from-cw-green/[0.06] to-white/5 transition-all hover:border-cw-green/40 hover:from-cw-green/[0.1] hover:to-white/[0.07]"
							>
								<div className="absolute left-3 top-3 z-10">
									<span className="rounded-full bg-cw-green/20 px-2.5 py-0.5 text-[10px] font-semibold uppercase tracking-wider text-cw-green">
										Featured
									</span>
								</div>
								<FeaturedCardContent demo={demo} />
							</a>
						))}
					</div>
				</div>
			</section>

			{/* All Demos */}
			<section className="px-6 pb-24">
				<div className="mx-auto max-w-7xl">
					<div className="mb-8 flex items-center gap-3">
						<span className="text-sm font-medium text-gray-400">
							All Demos
						</span>
						<div className="h-px flex-1 bg-white/10" />
					</div>
					<div className="grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
						{otherDemos.map((demo) => {
							const href = demo.status === "live" && "slug" in demo ? `/demos/${demo.slug}` : undefined;
							return href ? (
								<a
									key={demo.title}
									href={href}
									className="group overflow-hidden rounded-2xl border border-white/10 bg-white/5 transition-all hover:border-cw-green/30 hover:bg-white/[0.07]"
								>
									<DemoCardContent demo={demo} />
								</a>
							) : (
								<div
									key={demo.title}
									className="group overflow-hidden rounded-2xl border border-white/10 bg-white/5 transition-all hover:border-cw-green/30 hover:bg-white/[0.07]"
								>
									<DemoCardContent demo={demo} />
								</div>
							);
						})}
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold text-white">Want to See a Live Demo?</h2>
					<p className="mt-4 text-lg text-gray-400">
						Schedule a walkthrough with our team. We&apos;ll show you how CleanWalker can transform
						litter management at your site.
					</p>
					<a
						href="/contact"
						className="mt-8 inline-block rounded-lg bg-cw-green px-8 py-4 text-lg font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
				</div>
			</section>
		</div>
	);
}

function FeaturedCardContent({ demo }: { demo: (typeof demos)[number] }) {
	return (
		<>
			{/* Demo Preview Area */}
			<div className="relative flex aspect-[4/3] items-center justify-center bg-gradient-to-br from-cw-green/10 to-cw-dark p-6">
				<div className="text-center">
					<div className="mx-auto mb-3 flex h-20 w-20 items-center justify-center rounded-full bg-cw-green/20">
						<svg className="h-10 w-10 text-cw-green" fill="none" viewBox="0 0 24 24" stroke="currentColor">
							<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
							<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
						</svg>
					</div>
					<p className="text-sm text-gray-400">{demo.placeholder}</p>
				</div>
			</div>

			{/* Content */}
			<div className="p-6">
				<h3 className="text-xl font-bold text-white group-hover:text-cw-green transition-colors">{demo.title}</h3>
				<p className="mt-2 text-sm leading-relaxed text-gray-400">{demo.description}</p>
				{"disclaimer" in demo && demo.disclaimer && (
					<p className="mt-2 text-xs italic text-gray-500">{demo.disclaimer}</p>
				)}
				<div className="mt-4 flex flex-wrap gap-2">
					{demo.tags.map((tag) => (
						<span
							key={tag}
							className="rounded-full bg-cw-green/10 px-3 py-1 text-xs text-cw-green/80"
						>
							{tag}
						</span>
					))}
				</div>
				<div className="mt-4 flex items-center gap-1.5 text-sm font-medium text-cw-green">
					<span>Try it now</span>
					<svg className="h-4 w-4 transition-transform group-hover:translate-x-1" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
						<path strokeLinecap="round" strokeLinejoin="round" d="M13.5 4.5L21 12m0 0l-7.5 7.5M21 12H3" />
					</svg>
				</div>
			</div>
		</>
	);
}

function DemoCardContent({ demo }: { demo: (typeof demos)[number] }) {
	return (
		<>
			{/* Demo Preview Area */}
			<div className="relative flex aspect-video items-center justify-center bg-gradient-to-br from-cw-green/5 to-cw-dark p-6">
				{demo.status === "live" ? (
					<div className="text-center">
						<div className="mx-auto mb-3 flex h-16 w-16 items-center justify-center rounded-full bg-cw-green/20">
							<svg className="h-8 w-8 text-cw-green" fill="none" viewBox="0 0 24 24" stroke="currentColor">
								<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
								<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
							</svg>
						</div>
						<p className="text-sm text-gray-400">{demo.placeholder}</p>
					</div>
				) : (
					<div className="text-center">
						<div className="mx-auto mb-3 flex h-16 w-16 items-center justify-center rounded-full bg-white/10">
							<svg className="h-8 w-8 text-gray-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
								<path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547M8 4h8l-1 1v5.172a2 2 0 00.586 1.414l5 5c1.26 1.26.367 3.414-1.415 3.414H4.828c-1.782 0-2.674-2.154-1.414-3.414l5-5A2 2 0 009 10.172V5L8 4z" />
							</svg>
						</div>
						<p className="text-sm text-gray-500">{demo.placeholder}</p>
					</div>
				)}
				{/* Status Badge */}
				<div className="absolute right-3 top-3">
					{demo.status === "live" ? (
						<span className="rounded-full bg-cw-green/20 px-3 py-1 text-xs font-medium text-cw-green">
							Live
						</span>
					) : (
						<span className="rounded-full bg-white/10 px-3 py-1 text-xs font-medium text-gray-400">
							Coming Soon
						</span>
					)}
				</div>
			</div>

			{/* Content */}
			<div className="p-6">
				<h3 className="text-lg font-semibold text-white">{demo.title}</h3>
				<p className="mt-2 text-sm leading-relaxed text-gray-400">{demo.description}</p>
				<div className="mt-4 flex flex-wrap gap-2">
					{demo.tags.map((tag) => (
						<span
							key={tag}
							className="rounded-full bg-white/5 px-3 py-1 text-xs text-gray-400"
						>
							{tag}
						</span>
					))}
				</div>
			</div>
		</>
	);
}
