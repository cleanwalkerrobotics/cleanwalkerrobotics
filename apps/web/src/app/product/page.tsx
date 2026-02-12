// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import Image from "next/image";
import HowItWorks from "./HowItWorks";

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
							<div className="mt-8 flex flex-wrap gap-4">
								<a
									href="/contact"
									className="rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
								>
									Schedule a Demo
								</a>
								<a
									href="/demos"
									className="rounded-lg border border-white/20 bg-white/5 px-8 py-3.5 text-sm font-semibold text-white backdrop-blur-sm transition-colors hover:border-white/40 hover:bg-white/10"
								>
									Try Interactive Demos
								</a>
							</div>
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

			{/* How It Works — Animated */}
			<HowItWorks />

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
									"Dual OAK-D Pro stereo depth + Livox Mid-360 3D LiDAR",
									"2-DOF gripper arm with 5g–500g pickup range",
									"Bag Cassette System with 20 bags per cartridge",
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
						<div className="rounded-2xl border border-gray-200 bg-white p-6 sm:p-8">
							<h3 className="text-lg font-semibold text-gray-900">Physical</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Form Factor", "Quadrupedal (4-leg)"],
									["Size", "600 x 150 x 350mm"],
									["Weight", "~15kg"],
									["Weatherproofing", "IP54+ rated"],
									["Collection", "Bag Cassette, 20/cartridge"],
									["Mobility", "Grass, pavement, gravel, slopes"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex flex-col gap-1 border-b border-gray-100 pb-3 sm:flex-row sm:justify-between sm:gap-4"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-6 sm:p-8">
							<h3 className="text-lg font-semibold text-gray-900">Intelligence</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Compute", "NVIDIA Jetson Orin Nano Super (67 TOPS)"],
									["Vision", "Dual OAK-D Pro stereo depth"],
									["LiDAR", "Livox Mid-360 3D"],
									["AI Model", "YOLO26s (50+ litter types)"],
									["Connectivity", "LTE + WiFi"],
									["Updates", "OTA firmware & ML models"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex flex-col gap-1 border-b border-gray-100 pb-3 sm:flex-row sm:justify-between sm:gap-4"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-6 sm:p-8">
							<h3 className="text-lg font-semibold text-gray-900">Performance</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Coverage", "5-10 acres per day"],
									["Daily Operation", "20+ hours (self-charging)"],
									["Battery", "48V 20Ah Li-ion"],
									["Litter Range", "5g to 500g items"],
									["Litter Types", "Bottles, cans, wrappers, cups, butts"],
									["Charging", "Autonomous self-docking"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex flex-col gap-1 border-b border-gray-100 pb-3 sm:flex-row sm:justify-between sm:gap-4"
									>
										<dt className="text-sm text-gray-500">{label}</dt>
										<dd className="text-sm font-medium text-gray-900">{value}</dd>
									</div>
								))}
							</dl>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-6 sm:p-8">
							<h3 className="text-lg font-semibold text-gray-900">Safety</h3>
							<dl className="mt-6 space-y-4">
								{[
									["Emergency Stop", "Physical button + remote"],
									["Obstacle Avoidance", "Multi-sensor fusion"],
									["Pedestrian Detection", "Real-time AI"],
									["Weather Safety", "Auto-dock in severe weather"],
									["Liability", "Commercial insurance"],
									["Replacement", "48hr replacement SLA"],
								].map(([label, value]) => (
									<div
										key={label}
										className="flex flex-col gap-1 border-b border-gray-100 pb-3 sm:flex-row sm:justify-between sm:gap-4"
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
					<div className="mt-12 grid gap-6 sm:grid-cols-2 md:grid-cols-3">
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
								desc: "Get notified about bag cassette status, maintenance needs, and performance reports.",
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

			{/* Try Interactive Demos */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="rounded-2xl border border-white/10 bg-white/5 p-8 sm:p-12">
						<div className="grid items-center gap-8 md:grid-cols-2">
							<div>
								<div className="mb-4 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm text-cw-green">
									Interactive Demos
								</div>
								<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
									Try the Technology Yourself
								</h2>
								<p className="mt-4 leading-relaxed text-gray-400">
									Explore 10 interactive demos running in your browser — from AI litter
									detection to 3D robot visualization and fleet management simulations.
								</p>
								<a
									href="/demos"
									className="mt-8 inline-flex items-center gap-2 rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
								>
									Explore All Demos
									<svg className="h-4 w-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
										<path strokeLinecap="round" strokeLinejoin="round" d="M13.5 4.5L21 12m0 0l-7.5 7.5M21 12H3" />
									</svg>
								</a>
							</div>
							<div className="grid grid-cols-2 gap-4">
								{[
									{ label: "AI Detection", href: "/demos/litter-detection" },
									{ label: "3D Viewer", href: "/demos/3d-robot-viewer" },
									{ label: "Navigation", href: "/demos/route-planning" },
									{ label: "Cost Calculator", href: "/demos/cost-calculator" },
								].map((demo) => (
									<a
										key={demo.label}
										href={demo.href}
										className="flex items-center justify-center rounded-xl border border-white/10 bg-white/5 p-4 text-sm font-medium text-gray-300 transition-all hover:border-cw-green/30 hover:bg-white/[0.07] hover:text-cw-green"
									>
										{demo.label}
									</a>
								))}
							</div>
						</div>
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
					<div className="mt-12 grid gap-4 sm:grid-cols-2 md:grid-cols-3 sm:gap-6">
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-side-profile.webp"
								alt="CleanWalker robot clean side profile showing four articulated legs, folded gripper arm, and LiDAR sensor"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-sensors.webp"
								alt="CleanWalker front-face sensor array with stereo camera lenses and green LED status indicator"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/lifestyle-night-ops.webp"
								alt="CleanWalker robot operating at night on a city street with green LED strip illumination"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-gripper.webp"
								alt="CleanWalker gripper mechanism close-up"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/component-actuator.webp"
								alt="CleanWalker brushless actuator motor with integrated controller board alongside robot"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
							/>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-cw-charcoal">
							<Image
								src="/renders/v3/detail-charging-dock.webp"
								alt="CleanWalker autonomous charging dock station"
								fill
								className="object-cover"
								sizes="(max-width: 640px) 100vw, (max-width: 768px) 50vw, 33vw"
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
					<div className="mt-10 flex flex-wrap justify-center gap-4">
						<a
							href="/contact"
							className="rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
						>
							Schedule a Demo
						</a>
						<a
							href="/pilot"
							className="rounded-lg border border-white/20 bg-white/5 px-8 py-3.5 text-sm font-semibold text-white backdrop-blur-sm transition-colors hover:border-white/40 hover:bg-white/10"
						>
							Join Pilot Program
						</a>
					</div>
				</div>
			</section>
		</div>
	);
}
