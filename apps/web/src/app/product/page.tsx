// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";

export const metadata: Metadata = {
	title: "Product — CleanWalker Robotics",
	description:
		"Discover the CleanWalker autonomous litter-collecting robot. Quadrupedal, weatherproof, AI-powered, and built for 24/7 outdoor operation.",
};

function ImagePlaceholder({
	label,
	description,
	aspect = "4/3",
}: {
	label: string;
	description: string;
	aspect?: string;
}) {
	return (
		<div
			className="relative overflow-hidden rounded-2xl bg-gradient-to-br from-cw-charcoal to-cw-gray"
			style={{ aspectRatio: aspect }}
		>
			<div className="absolute inset-0 flex flex-col items-center justify-center p-6 text-center">
				<div className="mb-3 rounded-full bg-cw-green/20 p-3">
					<svg
						className="h-6 w-6 text-cw-green"
						fill="none"
						viewBox="0 0 24 24"
						stroke="currentColor"
						strokeWidth={1.5}
					>
						<path
							strokeLinecap="round"
							strokeLinejoin="round"
							d="M2.25 15.75l5.159-5.159a2.25 2.25 0 013.182 0l5.159 5.159m-1.5-1.5l1.409-1.409a2.25 2.25 0 013.182 0l2.909 2.909M3.75 21h16.5A2.25 2.25 0 0022.5 18.75V5.25A2.25 2.25 0 0020.25 3H3.75A2.25 2.25 0 001.5 5.25v13.5A2.25 2.25 0 003.75 21z"
						/>
					</svg>
				</div>
				<p className="text-sm font-medium text-gray-300">[{label}]</p>
				<p className="mt-1 text-xs text-gray-500">{description}</p>
			</div>
		</div>
	);
}

export default function ProductPage() {
	return (
		<div>
			{/* Hero */}
			<section className="bg-cw-dark px-6 py-24 md:py-32">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<h1 className="text-4xl font-bold tracking-tight text-white md:text-5xl">
								Meet the <span className="text-cw-green">CleanWalker</span> Robot
							</h1>
							<p className="mt-6 text-lg leading-relaxed text-gray-400">
								A quadrupedal, autonomous litter-collecting robot built for real-world
								outdoor environments. AI-powered perception, all-weather operation, and
								zero human intervention required.
							</p>
						</div>
						<ImagePlaceholder
							label="Front 3/4 view of CleanWalker robot"
							description="Matte charcoal body, green LED accents, four articulated legs, stereo camera system"
						/>
					</div>
				</div>
			</section>

			{/* How It Works */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							How It Works
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Detect, navigate, collect, and dispose — all autonomously.
						</p>
					</div>
					<div className="mt-16 grid gap-12 md:grid-cols-2">
						<div className="space-y-8">
							{[
								{
									title: "AI Litter Detection",
									desc: "YOLO-based vision model identifies bottles, cans, wrappers, cups, cigarette butts, and 50+ litter types in real-time. Continuously improving through over-the-air model updates.",
								},
								{
									title: "Autonomous Navigation",
									desc: "Stereo depth cameras and LiDAR create a 3D map of the environment. The robot plans efficient routes, avoids obstacles, and navigates complex terrain including grass, paths, and slopes.",
								},
								{
									title: "Precision Collection",
									desc: "A 2-DOF robotic arm with soft silicone gripper gently picks up litter items from 5g to 500g. The gripper handles everything from cigarette butts to water bottles.",
								},
								{
									title: "Self-Charging",
									desc: "When the battery runs low or the waste bin is full, the robot autonomously returns to its charging dock. Pogo-pin contacts and self-alignment make docking fully automatic.",
								},
							].map((item) => (
								<div key={item.title} className="flex gap-4">
									<div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-lg bg-cw-green/10">
										<svg
											className="h-5 w-5 text-cw-green"
											fill="none"
											viewBox="0 0 24 24"
											stroke="currentColor"
											strokeWidth={2}
										>
											<path
												strokeLinecap="round"
												strokeLinejoin="round"
												d="M4.5 12.75l6 6 9-13.5"
											/>
										</svg>
									</div>
									<div>
										<h3 className="text-lg font-semibold text-gray-900">
											{item.title}
										</h3>
										<p className="mt-1 leading-relaxed text-gray-600">{item.desc}</p>
									</div>
								</div>
							))}
						</div>
						<div className="space-y-6">
							<ImagePlaceholder
								label="Close-up of gripper picking up aluminum can"
								description="Soft silicone 3-finger gripper, macro photography, studio lighting"
							/>
							<ImagePlaceholder
								label="Robot at charging dock in park setting"
								description="Weatherproof dock with rain canopy, green LED charging indicator"
								aspect="16/9"
							/>
						</div>
					</div>
				</div>
			</section>

			{/* Specs */}
			<section id="specs" className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Technical Specifications
						</h2>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-2">
						<div className="rounded-2xl border border-gray-200 p-8">
							<h3 className="text-lg font-semibold text-gray-900">Physical</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Form Factor", "Quadrupedal (4-leg)"],
									["Size", "~80cm L x 40cm W x 50cm H"],
									["Weight", "~25kg (estimated)"],
									["Weatherproofing", "IP54+ rated"],
									["Waste Bin", "~20L compacting bin, top-loading"],
									["Mobility", "Grass, pavement, gravel, slopes"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex justify-between border-b border-gray-100 pb-3"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 p-8">
							<h3 className="text-lg font-semibold text-gray-900">Intelligence</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Compute", "NVIDIA Jetson Orin Nano"],
									["Vision", "OAK-D Pro stereo depth camera"],
									["LiDAR", "2D RPLiDAR for obstacle avoidance"],
									["AI Model", "Custom YOLO litter detection"],
									["Connectivity", "4G LTE (always-on cellular)"],
									["Updates", "Over-the-air firmware & ML models"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex justify-between border-b border-gray-100 pb-3"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 p-8">
							<h3 className="text-lg font-semibold text-gray-900">Performance</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Coverage", "5-10 acres per day"],
									["Daily Operation", "20+ hours (with self-charging)"],
									["Battery", "48V 20Ah Li-ion"],
									["Litter Range", "5g to 500g items"],
									["Litter Types", "Bottles, cans, wrappers, cups, butts, paper"],
									["Charging", "Autonomous self-docking"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex justify-between border-b border-gray-100 pb-3"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 p-8">
							<h3 className="text-lg font-semibold text-gray-900">Safety</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Emergency Stop", "Physical button + remote"],
									["Obstacle Avoidance", "Multi-sensor fusion"],
									["Pedestrian Detection", "Real-time AI detection"],
									["Weather Safety", "Auto-dock in severe weather"],
									["Liability", "Comprehensive commercial insurance"],
									["Replacement", "48hr replacement SLA (24hr Fleet)"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex justify-between border-b border-gray-100 pb-3"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
					</div>
				</div>
			</section>

			{/* Features Grid */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Fleet Management Software
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Every subscription includes our real-time fleet dashboard.
						</p>
					</div>
					<div className="mt-16 grid gap-6 md:grid-cols-3">
						{[
							{
								title: "Real-Time Map",
								desc: "See every robot's position, status, and route on a live map of your site.",
							},
							{
								title: "Collection Analytics",
								desc: "Track litter collected by type, location, and time. Identify hotspots and trends.",
							},
							{
								title: "Route Planning",
								desc: "Set coverage zones, schedules, and priorities. Optimize for your site's needs.",
							},
							{
								title: "Battery & Health",
								desc: "Monitor battery levels, component health, and maintenance status across your fleet.",
							},
							{
								title: "Alerts & Notifications",
								desc: "Get notified about bin capacity, maintenance needs, unusual events, and performance reports.",
							},
							{
								title: "API Access",
								desc: "Integrate robot data with your existing systems via REST API (Fleet tier).",
							},
						].map((feature) => (
							<div
								key={feature.title}
								className="rounded-xl border border-gray-200 bg-white p-6"
							>
								<h3 className="font-semibold text-gray-900">{feature.title}</h3>
								<p className="mt-2 text-sm leading-relaxed text-gray-600">
									{feature.desc}
								</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* Product Images Gallery */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Gallery
						</h2>
					</div>
					<div className="mt-12 grid gap-6 md:grid-cols-3">
						<ImagePlaceholder
							label="Side profile — full robot on white background"
							description="Standing pose, visible joints, folded gripper arm, LiDAR puck"
						/>
						<ImagePlaceholder
							label="Sensor array close-up — front face"
							description="Stereo camera lenses, IR dot projector, green LED status indicator"
						/>
						<ImagePlaceholder
							label="Night operations — robot on city street"
							description="Green LED strip illuminating sidewalk, cinematic lighting"
						/>
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						See CleanWalker in action
					</h2>
					<p className="mx-auto mt-4 max-w-xl text-lg text-gray-400">
						Schedule a site assessment and we&apos;ll show you exactly how our robots
						would operate at your facility.
					</p>
					<a
						href="/contact"
						className="mt-10 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Schedule a Demo
					</a>
				</div>
			</section>
		</div>
	);
}
