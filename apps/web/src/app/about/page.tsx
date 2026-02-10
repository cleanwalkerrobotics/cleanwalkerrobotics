// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import Image from "next/image";

export const metadata: Metadata = {
	title: "About — CleanWalker Robotics",
	description:
		"CleanWalker Robotics is building autonomous robots that keep public spaces clean. Learn about our mission, team, and technology.",
};

function TeamPlaceholder({ name, role }: { name: string; role: string }) {
	return (
		<div className="text-center">
			<div className="mx-auto flex h-32 w-32 items-center justify-center rounded-full bg-gradient-to-br from-cw-charcoal to-cw-gray">
				<svg
					className="h-12 w-12 text-gray-500"
					fill="none"
					viewBox="0 0 24 24"
					stroke="currentColor"
					strokeWidth={1}
				>
					<path
						strokeLinecap="round"
						strokeLinejoin="round"
						d="M15.75 6a3.75 3.75 0 11-7.5 0 3.75 3.75 0 017.5 0zM4.501 20.118a7.5 7.5 0 0114.998 0A17.933 17.933 0 0112 21.75c-2.676 0-5.216-.584-7.499-1.632z"
					/>
				</svg>
			</div>
			<h3 className="mt-4 font-semibold text-gray-900">{name}</h3>
			<p className="text-sm text-gray-500">{role}</p>
		</div>
	);
}

export default function AboutPage() {
	return (
		<div>
			{/* Hero — Full-bleed with sidewalk image */}
			<section className="relative bg-cw-dark px-6 py-24 md:py-32">
				<Image
					src="/renders/hero-sidewalk.png"
					alt="CleanWalker robot navigating a European city sidewalk alongside pedestrians"
					fill
					className="object-cover opacity-40"
					priority
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-gradient-to-b from-cw-dark/60 to-cw-dark/80" />
				<div className="relative z-10 mx-auto max-w-4xl text-center">
					<h1 className="text-4xl font-bold tracking-tight text-white md:text-5xl">
						Making Public Spaces <span className="text-cw-green">Cleaner</span>,
						Autonomously
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg text-gray-300">
						We&apos;re building the next generation of autonomous robots to solve one
						of the most persistent challenges facing cities, parks, and communities:
						litter.
					</p>
				</div>
			</section>

			{/* Mission */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
								Our Mission
							</div>
							<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
								Clean spaces for every community
							</h2>
							<p className="mt-6 leading-relaxed text-gray-600">
								Litter is one of the most visible and persistent problems in public
								spaces. It degrades quality of life, harms wildlife, pollutes
								waterways, and costs municipalities billions annually to manage.
							</p>
							<p className="mt-4 leading-relaxed text-gray-600">
								CleanWalker exists to solve this problem with robotics and AI. We
								believe autonomous litter collection can be more effective, more
								consistent, and more affordable than manual methods — freeing
								municipal workers for higher-value tasks while keeping public spaces
								pristine.
							</p>
							<p className="mt-4 leading-relaxed text-gray-600">
								Our Robot-as-a-Service model eliminates the capital and operational
								complexity of deploying robotics, making autonomous cleaning accessible
								to communities of all sizes.
							</p>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl">
							<Image
								src="/renders/lifestyle-city-worker.png"
								alt="CleanWalker robot working alongside a city maintenance worker in an urban park"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
						</div>
					</div>
				</div>
			</section>

			{/* By the Numbers */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							The Opportunity
						</h2>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-4">
						{[
							{
								stat: "$27M",
								label: "Projected Year 5 revenue",
							},
							{
								stat: "1,000",
								label: "Robots in deployment by Year 5",
							},
							{
								stat: "90%",
								label: "Gross margin at Standard tier",
							},
							{
								stat: "Year 3",
								label: "Profitability milestone",
							},
						].map((item) => (
							<div key={item.label} className="text-center">
								<div className="text-4xl font-bold text-cw-green">{item.stat}</div>
								<p className="mt-2 text-sm text-gray-600">{item.label}</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* Technology */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
							Technology
						</div>
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Purpose-Built for Outdoor Autonomy
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Our technology stack combines proven robotics hardware with cutting-edge
							AI to deliver reliable, all-weather litter collection.
						</p>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-3">
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">
								Quadrupedal Platform
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Four-legged design navigates terrain that wheeled robots cannot —
								grass, gravel, curbs, and slopes. Twelve servo actuators provide
								stable, adaptive locomotion across any surface found in parks and
								public spaces.
							</p>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">
								AI Perception &amp; Detection
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Stereo depth cameras (OAK-D Pro) and LiDAR provide rich 3D
								environmental understanding. Our custom YOLO-based detection model
								identifies 50+ litter types in real-time, continuously improving
								through OTA model updates.
							</p>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<h3 className="text-lg font-semibold text-gray-900">Edge Compute</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								NVIDIA Jetson Orin Nano provides powerful on-device AI inference,
								enabling real-time litter detection and path planning without cloud
								dependency. 4G connectivity enables fleet management, telemetry, and
								remote updates.
							</p>
						</div>
					</div>
				</div>
			</section>

			{/* Team */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
							Team
						</div>
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Built by Roboticists and Engineers
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Our team brings together expertise in robotics, machine learning,
							mechanical engineering, and municipal operations.
						</p>
					</div>
					<div className="mt-16 grid gap-12 md:grid-cols-4">
						<TeamPlaceholder name="Founder &amp; CEO" role="Robotics &amp; Business" />
						<TeamPlaceholder name="CTO" role="Software &amp; AI" />
						<TeamPlaceholder
							name="Head of Hardware"
							role="Mechanical Engineering"
						/>
						<TeamPlaceholder
							name="Head of Operations"
							role="Fleet &amp; Customer Success"
						/>
					</div>
				</div>
			</section>

			{/* Values */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Our Values
						</h2>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-3">
						{[
							{
								title: "Pragmatic Innovation",
								desc: "We build what works. Our robots use proven hardware components and battle-tested algorithms, not science projects. Utility vehicle pragmatism over sci-fi fantasies.",
							},
							{
								title: "Environmental Impact",
								desc: "Every piece of litter our robots collect is one less item entering waterways and ecosystems. We measure success in cleaner parks, not just revenue.",
							},
							{
								title: "Accessible Robotics",
								desc: "Our RaaS model removes capital barriers. Communities of all sizes should benefit from autonomous technology, not just those with the biggest budgets.",
							},
						].map((value) => (
							<div
								key={value.title}
								className="rounded-2xl border border-gray-200 bg-white p-8"
							>
								<h3 className="text-lg font-semibold text-gray-900">
									{value.title}
								</h3>
								<p className="mt-3 leading-relaxed text-gray-600">{value.desc}</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* CTA with hero-sidewalk background */}
			<section className="relative bg-cw-dark px-6 py-24">
				<Image
					src="/renders/hero-sidewalk.png"
					alt="CleanWalker robot on a European sidewalk"
					fill
					className="object-cover opacity-20"
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-cw-dark/60" />
				<div className="relative z-10 mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						Join us in making public spaces cleaner
					</h2>
					<p className="mx-auto mt-4 max-w-xl text-lg text-gray-400">
						Whether you&apos;re a municipality, property manager, or potential team
						member — we&apos;d love to hear from you.
					</p>
					<a
						href="/contact"
						className="mt-10 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Get in Touch
					</a>
				</div>
			</section>
		</div>
	);
}
