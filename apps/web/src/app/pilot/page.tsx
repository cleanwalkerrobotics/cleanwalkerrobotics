// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import Image from "next/image";

export const metadata: Metadata = {
	title: "Pilot Program",
	description:
		"Partner with CleanWalker Robotics. Deploy 10 autonomous litter collection robots at your site for a 6-month pilot program. Limited slots available for 2027.",
	openGraph: {
		title: "CleanWalker Pilot Program — Partner With Us",
		description:
			"Deploy 10 autonomous litter collection robots at your site. We handle deployment, training, and support. 3 pilot slots available for 2027.",
	},
};

export default function PilotPage() {
	return (
		<div>
			{/* Hero */}
			<section className="relative bg-cw-dark px-6 py-24 md:py-32">
				<Image
					src="/renders/v3/hero-sidewalk.webp"
					alt="CleanWalker autonomous robots deployed in a park setting"
					fill
					className="object-cover opacity-30"
					priority
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-gradient-to-b from-cw-dark/60 to-cw-dark/90" />
				<div className="relative z-10 mx-auto max-w-4xl text-center">
					<div className="mx-auto mb-6 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm text-cw-green">
						Limited Availability &mdash; 3 Pilot Slots for 2027
					</div>
					<h1 className="text-4xl font-bold tracking-tight text-white md:text-5xl">
						Partner with <span className="text-cw-green">CleanWalker</span>
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg text-gray-300">
						Join the autonomous litter collection revolution. Deploy our robots
						at your site and see the results firsthand &mdash; we handle
						everything.
					</p>
					<a
						href="/contact"
						className="mt-10 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Schedule a Call
					</a>
				</div>
			</section>

			{/* What a Pilot Looks Like */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
							The Pilot Program
						</div>
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							What a Pilot Looks Like
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							A straightforward deployment designed to prove the value of
							autonomous litter collection at your site.
						</p>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-3">
						<div className="rounded-2xl border border-gray-200 bg-white p-8 text-center">
							<div className="mx-auto flex h-14 w-14 items-center justify-center rounded-full bg-cw-green/10">
								<span className="text-2xl font-bold text-cw-green">10</span>
							</div>
							<h3 className="mt-4 text-lg font-semibold text-gray-900">
								Units Deployed
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								A fleet of 10 CleanWalker robots deployed across your chosen
								site, providing comprehensive coverage and meaningful data.
							</p>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-8 text-center">
							<div className="mx-auto flex h-14 w-14 items-center justify-center rounded-full bg-cw-green/10">
								<span className="text-2xl font-bold text-cw-green">6</span>
							</div>
							<h3 className="mt-4 text-lg font-semibold text-gray-900">
								Months Duration
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Six months gives us all four seasons of data and enough time to
								demonstrate long-term operational reliability and cost savings.
							</p>
						</div>
						<div className="rounded-2xl border border-gray-200 bg-white p-8 text-center">
							<div className="mx-auto flex h-14 w-14 items-center justify-center rounded-full bg-cw-green/10">
								<span className="text-2xl font-bold text-cw-green">1</span>
							</div>
							<h3 className="mt-4 text-lg font-semibold text-gray-900">
								Your Choice of Site
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Parks, campuses, commercial districts, waterfront promenades
								&mdash; you choose the location that matters most to your
								community.
							</p>
						</div>
					</div>
					<div className="mt-12 rounded-2xl border border-gray-200 bg-white p-8">
						<h3 className="text-lg font-semibold text-gray-900">
							We Handle Everything
						</h3>
						<div className="mt-4 grid gap-4 md:grid-cols-4">
							{[
								"Site survey & deployment planning",
								"Robot installation & calibration",
								"On-site training for your team",
								"Ongoing maintenance & support",
							].map((item) => (
								<div key={item} className="flex items-start gap-3">
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
											d="M5 13l4 4L19 7"
										/>
									</svg>
									<span className="text-sm text-gray-600">{item}</span>
								</div>
							))}
						</div>
					</div>
				</div>
			</section>

			{/* What We Measure */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
							Data-Driven Results
						</div>
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							What We Measure
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Every pilot generates comprehensive performance data so you can
							make informed decisions about full-scale deployment.
						</p>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-4">
						{[
							{
								metric: "Items Collected",
								desc: "Total litter items detected and collected, categorized by type and location density.",
								icon: "M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4",
							},
							{
								metric: "Area Covered",
								desc: "Square meters patrolled daily. GPS-tracked route efficiency and coverage completeness.",
								icon: "M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7",
							},
							{
								metric: "Uptime %",
								desc: "Fleet operational availability, including weather resilience and autonomous charging cycles.",
								icon: "M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z",
							},
							{
								metric: "Cost per Item",
								desc: "Direct cost comparison: CleanWalker autonomous collection vs. your current manual baseline.",
								icon: "M12 8c-1.657 0-3 .895-3 2s1.343 2 3 2 3 .895 3 2-1.343 2-3 2m0-8c1.11 0 2.08.402 2.599 1M12 8V7m0 1v8m0 0v1m0-1c-1.11 0-2.08-.402-2.599-1M21 12a9 9 0 11-18 0 9 9 0 0118 0z",
							},
						].map((item) => (
							<div key={item.metric} className="text-center">
								<div className="mx-auto flex h-12 w-12 items-center justify-center rounded-full bg-cw-green/10">
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
											d={item.icon}
										/>
									</svg>
								</div>
								<h3 className="mt-4 text-lg font-semibold text-gray-900">
									{item.metric}
								</h3>
								<p className="mt-2 text-sm leading-relaxed text-gray-600">
									{item.desc}
								</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* The Technology */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
								The Technology
							</div>
							<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
								Purpose-Built for Outdoor Autonomy
							</h2>
							<p className="mt-6 leading-relaxed text-gray-400">
								CleanWalker robots combine quadrupedal mobility, AI-powered
								litter detection, and autonomous navigation to operate 24/7 in
								parks, campuses, and public spaces &mdash; rain or shine.
							</p>
							<ul className="mt-6 space-y-3">
								{[
									"Quadrupedal platform — navigates grass, gravel, curbs, and slopes",
									"AI perception — detects 50+ litter types in real time",
									"Autonomous operation — self-charging, self-navigating, weather-resistant",
									"Fleet dashboard — monitor your entire deployment from one screen",
								].map((item) => (
									<li
										key={item}
										className="flex items-start gap-3 text-sm text-gray-300"
									>
										<svg
											className="mt-0.5 h-4 w-4 shrink-0 text-cw-green"
											fill="none"
											viewBox="0 0 24 24"
											stroke="currentColor"
											strokeWidth={2}
										>
											<path
												strokeLinecap="round"
												strokeLinejoin="round"
												d="M5 13l4 4L19 7"
											/>
										</svg>
										{item}
									</li>
								))}
							</ul>
							<a
								href="/product"
								className="mt-8 inline-flex items-center gap-2 text-sm font-semibold text-cw-green transition-colors hover:text-cw-green-dark"
							>
								View Full Specifications
								<svg
									className="h-4 w-4"
									fill="none"
									viewBox="0 0 24 24"
									stroke="currentColor"
									strokeWidth={2}
								>
									<path
										strokeLinecap="round"
										strokeLinejoin="round"
										d="M13.5 4.5L21 12m0 0l-7.5 7.5M21 12H3"
									/>
								</svg>
							</a>
						</div>
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl">
							<Image
								src="/renders/v3/lifestyle-city-worker.webp"
								alt="CleanWalker robot collecting litter in a public park"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
						</div>
					</div>
				</div>
			</section>

			{/* Pilot Availability / Urgency */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<div className="mb-4 text-sm font-semibold uppercase tracking-wider text-cw-green">
						Limited Availability
					</div>
					<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
						3 Pilot Slots Available for 2027
					</h2>
					<p className="mx-auto mt-6 max-w-2xl text-lg text-gray-600">
						We&apos;re selecting a small number of launch partners for our 2027
						pilot cohort. Each partner gets dedicated engineering support and
						priority access to full-scale deployment pricing.
					</p>
					<div className="mt-12 grid gap-6 md:grid-cols-3">
						{[
							{ slot: "Slot 1", status: "Available", open: true },
							{ slot: "Slot 2", status: "Available", open: true },
							{ slot: "Slot 3", status: "Available", open: true },
						].map((s) => (
							<div
								key={s.slot}
								className="rounded-xl border border-gray-200 bg-white p-6"
							>
								<div className="text-sm font-medium text-gray-500">
									{s.slot}
								</div>
								<div
									className={`mt-1 text-lg font-semibold ${s.open ? "text-cw-green" : "text-gray-400"}`}
								>
									{s.status}
								</div>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="relative bg-cw-dark px-6 py-24">
				<Image
					src="/renders/v3/hero-sidewalk.webp"
					alt="CleanWalker robot on a sidewalk"
					fill
					className="object-cover opacity-20"
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-cw-dark/60" />
				<div className="relative z-10 mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						Ready to see CleanWalker in action?
					</h2>
					<p className="mx-auto mt-4 max-w-xl text-lg text-gray-400">
						Schedule a call with our partnerships team. We&apos;ll walk you
						through the pilot process, discuss your site requirements, and
						answer any questions.
					</p>
					<a
						href="/contact"
						className="mt-10 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Schedule a Call
					</a>
				</div>
			</section>
		</div>
	);
}
