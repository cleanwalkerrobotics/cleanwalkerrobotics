// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import Image from "next/image";

export const metadata: Metadata = {
	title: "Product",
	description:
		"Discover the CleanWalker autonomous litter-collecting robot. Quadrupedal, weatherproof, AI-powered, and built for 24/7 outdoor operation.",
	openGraph: {
		title: "CleanWalker Robot — Product & Specifications",
		description:
			"Quadrupedal, weatherproof, AI-powered autonomous litter collection robot. Built for 24/7 outdoor operation across parks, campuses, and public spaces.",
	},
};

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
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-side-profile.webp"
								alt="CleanWalker robot side profile showing quadrupedal design, articulated legs, and sensor array"
								fill
								className="object-cover"
								priority
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
						</div>
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
									desc: "A 2-DOF robotic arm with a mechanical gripper with silicone-tipped fingers precisely picks up litter items from 5g to 500g. The gripper handles everything from cigarette butts to water bottles.",
								},
								{
									title: "Self-Charging",
									desc: "When the battery runs low or the bag cassette needs replacing, the robot autonomously returns to its charging dock. Pogo-pin contacts and self-alignment make docking fully automatic.",
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
							<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
								<Image
									src="/renders/v3/detail-gripper.webp"
									alt="Close-up of CleanWalker's mechanical gripper picking up an aluminum can with silicone-tipped rigid fingers"
									fill
									className="object-cover"
									sizes="(max-width: 768px) 100vw, 50vw"
								/>
							</div>
							<div className="relative aspect-[16/9] overflow-hidden rounded-2xl bg-cw-charcoal">
								<Image
									src="/renders/v3/detail-charging-dock.webp"
									alt="CleanWalker robot docked at its weatherproof charging station in a park setting"
									fill
									className="object-cover"
									sizes="(max-width: 768px) 100vw, 50vw"
								/>
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* Exploded View — Technology Deep Dive */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div className="relative aspect-square overflow-hidden rounded-2xl bg-cw-light">
							<Image
								src="/renders/v3/tech-exploded-view.webp"
								alt="Exploded view of CleanWalker robot showing internal components — motors, sensors, compute module, battery, gripper mechanism"
								fill
								className="object-contain"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
						</div>
						<div>
							<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
								Engineering
							</div>
							<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
								Purpose-Built for Outdoor Autonomy
							</h2>
							<p className="mt-6 leading-relaxed text-gray-600">
								Every component is selected for reliability, performance, and
								serviceability in real-world outdoor environments. From the NVIDIA
								Jetson Orin Nano compute module to the 48V Li-ion battery pack — no
								compromises.
							</p>
							<div className="mt-8 space-y-4">
								{[
									"12 servo actuators for adaptive quadrupedal locomotion",
									"OAK-D Pro stereo depth + 2D RPLiDAR sensor fusion",
									"2-DOF gripper arm with 5g–500g pickup range",
									"Bag Cassette System with 20-30 bags per cartridge",
									"IP54+ weatherproof enclosure for year-round operation",
								].map((feature) => (
									<div key={feature} className="flex items-start gap-3">
										<svg
											className="mt-0.5 h-5 w-5 shrink-0 text-cw-green"
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
										<span className="text-sm text-gray-700">{feature}</span>
									</div>
								))}
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* Specs */}
			<section id="specs" className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Technical Specifications
						</h2>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-2">
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">Physical</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Form Factor", "Quadrupedal (4-leg)"],
									["Size", "~80cm L x 40cm W x 50cm H"],
									["Weight", "~25kg (estimated)"],
									["Weatherproofing", "IP54+ rated"],
									["Collection", "Bag Cassette System, 20-30 bags per cartridge"],
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
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
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
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">Performance</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Coverage", "5-10 acres per day"],
									["Daily Operation", "20+ hours (with self-charging)"],
									["Battery", "48V 20Ah Li-ion"],
									["Litter Range", "5g to 500g items"],
									["Litter Types", "Bottles, cans, wrappers, cups, butts, paper"],
									["Curb Drop", "Sealed bags at municipal collection points"],
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
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">Safety</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Emergency Stop", "Physical button + remote"],
									["Obstacle Avoidance", "Multi-sensor fusion"],
									["Pedestrian Detection", "Real-time AI detection"],
									["Weather Safety", "Auto-dock in severe weather"],
									["Liability", "Comprehensive commercial insurance"],
									["Replacement", "48hr replacement SLA"],
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

			{/* Sensor Detail */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
								Perception System
							</div>
							<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
								See Everything, Miss Nothing
							</h2>
							<p className="mt-6 leading-relaxed text-gray-600">
								CleanWalker&apos;s multi-sensor perception system combines stereo depth
								cameras, LiDAR, and AI-powered detection to understand its
								environment in full 3D — identifying litter, avoiding obstacles, and
								navigating complex terrain with centimeter-level precision.
							</p>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-sensors.webp"
								alt="Close-up of CleanWalker sensor array showing stereo camera lenses, IR projector, and LiDAR puck"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
						</div>
					</div>
				</div>
			</section>

			{/* Fleet Management Software */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Fleet Management Software
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Every deployment includes our real-time fleet dashboard.
						</p>
					</div>
					<div className="mt-12">
						<div className="relative aspect-[16/9] overflow-hidden rounded-2xl border border-gray-200 bg-white shadow-lg">
							<Image
								src="/renders/v3/tech-dashboard-mockup.webp"
								alt="CleanWalker fleet management dashboard showing real-time robot locations, analytics, and route planning on screen"
								fill
								className="object-cover"
								sizes="(max-width: 1280px) 100vw, 1280px"
							/>
						</div>
					</div>
					<div className="mt-12 grid gap-6 md:grid-cols-3">
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
								desc: "Get notified about bag cassette status, maintenance needs, unusual events, and performance reports.",
							},
							{
								title: "API Access",
								desc: "Integrate robot data with your existing systems via REST API.",
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

			{/* Product Gallery */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Gallery
						</h2>
					</div>
					<div className="mt-12 grid gap-6 md:grid-cols-3">
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-side-profile.webp"
								alt="CleanWalker robot clean side profile showing four articulated legs, folded gripper arm, and LiDAR sensor"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-sensors.webp"
								alt="CleanWalker front-face sensor array with stereo camera lenses and green LED status indicator"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/lifestyle-night-ops.webp"
								alt="CleanWalker robot operating at night on a city street with green LED strip illumination"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/component-actuator.webp"
								alt="CleanWalker brushless actuator motor with integrated controller board alongside robot"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/component-pcb.webp"
								alt="CleanWalker robot with exposed main control PCB showing sensor and motor driver connections"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 33vw"
							/>
						</div>
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
