// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import Image from "next/image";

export default function Home() {
	return (
		<div>
			{/* Hero Section — Full-bleed with robot image */}
			<section className="relative min-h-[90vh] bg-cw-dark">
				<Image
					src="/renders/hero-park.png"
					alt="CleanWalker autonomous robot picking up a plastic bottle in a sunlit park"
					fill
					className="object-cover"
					priority
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-gradient-to-r from-cw-dark via-cw-dark/80 to-cw-dark/30" />
				<div className="relative z-10 flex min-h-[90vh] items-center px-6">
					<div className="mx-auto w-full max-w-7xl">
						<div className="max-w-2xl">
							<div className="mb-6 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm font-medium text-cw-green backdrop-blur-sm">
								Robot-as-a-Service
							</div>
							<h1 className="text-4xl font-bold leading-tight tracking-tight text-white md:text-6xl">
								Autonomous Litter Collection for{" "}
								<span className="text-cw-green">Cleaner Spaces</span>
							</h1>
							<p className="mt-6 max-w-lg text-lg leading-relaxed text-gray-300">
								CleanWalker deploys intelligent, autonomous robots that keep parks,
								campuses, and public spaces clean — 24/7, rain or shine. No upfront
								capital. No maintenance headaches. Just cleaner spaces.
							</p>
							<div className="mt-10 flex flex-wrap gap-4">
								<a
									href="/contact"
									className="rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
								>
									Schedule a Demo
								</a>
								<a
									href="/product"
									className="rounded-lg border border-white/20 bg-white/5 px-8 py-3.5 text-sm font-semibold text-white backdrop-blur-sm transition-colors hover:border-white/40 hover:bg-white/10"
								>
									See How It Works
								</a>
							</div>
							<div className="mt-10 flex items-center gap-8 rounded-xl border border-white/10 bg-black/30 px-6 py-4 text-sm text-gray-400 backdrop-blur-sm">
								<div>
									<span className="block text-2xl font-bold text-white">24/7</span>
									Autonomous operation
								</div>
								<div className="h-10 w-px bg-white/20" />
								<div>
									<span className="block text-2xl font-bold text-white">48%</span>
									Cost reduction
								</div>
								<div className="h-10 w-px bg-white/20" />
								<div>
									<span className="block text-2xl font-bold text-white">5-10</span>
									Acres per day
								</div>
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* Value Props */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Why CleanWalker?
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Replace costly manual litter collection with autonomous robots that work
							harder, longer, and smarter.
						</p>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-3">
						{/* Autonomous */}
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<div className="flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
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
										d="M9.813 15.904L9 18.75l-.813-2.846a4.5 4.5 0 00-3.09-3.09L2.25 12l2.846-.813a4.5 4.5 0 003.09-3.09L9 5.25l.813 2.846a4.5 4.5 0 003.09 3.09L15.75 12l-2.846.813a4.5 4.5 0 00-3.09 3.09zM18.259 8.715L18 9.75l-.259-1.035a3.375 3.375 0 00-2.455-2.456L14.25 6l1.036-.259a3.375 3.375 0 002.455-2.456L18 2.25l.259 1.035a3.375 3.375 0 002.455 2.456L21.75 6l-1.036.259a3.375 3.375 0 00-2.455 2.456z"
									/>
								</svg>
							</div>
							<h3 className="mt-6 text-xl font-semibold text-gray-900">
								Fully Autonomous
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								AI-powered navigation and litter detection. Self-charging, self-routing,
								self-managing. Our robots identify, pick up, and dispose of litter
								without human intervention.
							</p>
						</div>

						{/* 24/7 */}
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<div className="flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
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
										d="M12 6v6h4.5m4.5 0a9 9 0 11-18 0 9 9 0 0118 0z"
									/>
								</svg>
							</div>
							<h3 className="mt-6 text-xl font-semibold text-gray-900">
								24/7 Operation
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Over 20 hours of daily coverage with autonomous self-charging. Operates
								in rain, at night, and on weekends. Weatherproof IP54+ rated for
								year-round deployment.
							</p>
						</div>

						{/* Cost Savings */}
						<div className="rounded-2xl border border-gray-200 bg-white p-8">
							<div className="flex h-12 w-12 items-center justify-center rounded-lg bg-cw-green/10">
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
										d="M2.25 18.75a60.07 60.07 0 0115.797 2.101c.727.198 1.453-.342 1.453-1.096V18.75M3.75 4.5v.75A.75.75 0 013 6h-.75m0 0v-.375c0-.621.504-1.125 1.125-1.125H20.25M2.25 6v9m18-10.5v.75c0 .414.336.75.75.75h.75m-1.5-1.5h.375c.621 0 1.125.504 1.125 1.125v9.75c0 .621-.504 1.125-1.125 1.125h-.375m1.5-1.5H21a.75.75 0 00-.75.75v.75m0 0H3.75m0 0h-.375a1.125 1.125 0 01-1.125-1.125V15m1.5 1.5v-.75A.75.75 0 003 15h-.75M15 10.5a3 3 0 11-6 0 3 3 0 016 0zm3 0h.008v.008H18V10.5zm-12 0h.008v.008H6V10.5z"
									/>
								</svg>
							</div>
							<h3 className="mt-6 text-xl font-semibold text-gray-900">
								25-48% Cost Savings
							</h3>
							<p className="mt-3 leading-relaxed text-gray-600">
								Save $52,000 to $100,000 per site annually compared to manual litter
								collection. No labor overhead, no benefits costs, no sick days. Simple
								flexible service agreements.
							</p>
						</div>
					</div>
				</div>
			</section>

			{/* How It Works */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Simple to Deploy, Easy to Scale
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							From site assessment to fully operational in under 4 weeks.
						</p>
					</div>
					<div className="mt-16 grid gap-8 md:grid-cols-4">
						{[
							{
								step: "01",
								title: "Site Assessment",
								desc: "We survey your site, map terrain, and identify optimal coverage zones and dock placement.",
							},
							{
								step: "02",
								title: "Deployment",
								desc: "Robots and docks delivered, configured for your site, and activated within 2 weeks.",
							},
							{
								step: "03",
								title: "Training",
								desc: "Your team receives dashboard training and a complete operational overview.",
							},
							{
								step: "04",
								title: "Optimization",
								desc: "We fine-tune routes and schedules based on real litter data from your site.",
							},
						].map((item) => (
							<div key={item.step} className="text-center">
								<div className="mx-auto flex h-14 w-14 items-center justify-center rounded-full bg-cw-green/10 text-lg font-bold text-cw-green">
									{item.step}
								</div>
								<h3 className="mt-4 text-lg font-semibold text-gray-900">
									{item.title}
								</h3>
								<p className="mt-2 text-sm leading-relaxed text-gray-600">
									{item.desc}
								</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* Fleet Image — Real render */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="relative aspect-[21/9] overflow-hidden rounded-2xl">
						<Image
							src="/renders/hero-fleet.png"
							alt="Three CleanWalker robots working together in an urban park, aerial drone perspective"
							fill
							className="object-cover"
							sizes="(max-width: 1280px) 100vw, 1280px"
						/>
						<div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-black/20" />
						<div className="absolute bottom-0 left-0 right-0 p-8">
							<p className="text-xl font-semibold text-white">
								Fleet Deployment
							</p>
							<p className="mt-1 max-w-lg text-sm text-gray-300">
								Multiple robots coordinate across your site for maximum coverage —
								managed from a single dashboard
							</p>
						</div>
					</div>
				</div>
			</section>

			{/* Capabilities Visual — Night ops + Before/After */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Always On, Always Working
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Rain or shine, day or night — CleanWalker robots keep your spaces pristine.
						</p>
					</div>
					<div className="mt-12 grid gap-6 md:grid-cols-2">
						<div className="relative aspect-[16/10] overflow-hidden rounded-2xl">
							<Image
								src="/renders/lifestyle-night-ops.png"
								alt="CleanWalker robot operating at night with LED illumination on a city sidewalk"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
							<div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-transparent" />
							<div className="absolute bottom-0 left-0 right-0 p-6">
								<p className="text-lg font-semibold text-white">Night Operations</p>
								<p className="mt-1 text-sm text-gray-300">
									IR perception and LED lighting enable round-the-clock cleaning
								</p>
							</div>
						</div>
						<div className="relative aspect-[16/10] overflow-hidden rounded-2xl">
							<Image
								src="/renders/lifestyle-before-after.png"
								alt="Before and after comparison showing a littered park transformed to a clean space by CleanWalker robots"
								fill
								className="object-cover"
								sizes="(max-width: 768px) 100vw, 50vw"
							/>
							<div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-transparent" />
							<div className="absolute bottom-0 left-0 right-0 p-6">
								<p className="text-lg font-semibold text-white">Visible Results</p>
								<p className="mt-1 text-sm text-gray-300">
									Measurable impact on cleanliness from day one
								</p>
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* Social Proof */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<p className="text-center text-sm font-medium uppercase tracking-wider text-gray-500">
						Designed for
					</p>
					<div className="mt-8 grid grid-cols-2 gap-8 md:grid-cols-5">
						{[
							"Municipal Parks",
							"University Campuses",
							"Commercial Districts",
							"Theme Parks",
							"HOA Communities",
						].map((name) => (
							<div
								key={name}
								className="flex h-16 items-center justify-center rounded-xl border border-gray-200 bg-cw-light px-6"
							>
								<span className="text-sm font-medium text-gray-400">{name}</span>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* ROI Section */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
								Save up to $100,000 per site, per year
							</h2>
							<p className="mt-4 text-lg leading-relaxed text-gray-600">
								Manual litter collection costs $9,300-$13,400/month for a two-worker
								crew. CleanWalker delivers equivalent coverage for $7,000/month — with
								better data, more hours, and zero sick days.
							</p>
							<a
								href="/contact"
								className="mt-8 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
							>
								Contact Sales
							</a>
						</div>
						<div className="grid grid-cols-2 gap-6">
							{[
								{ value: "$7,000", label: "Monthly cost for 2 robots" },
								{ value: "20+", label: "Hours of daily coverage" },
								{ value: "$52K-$100K", label: "Annual savings per site" },
								{ value: "$0", label: "Setup fees" },
							].map((stat) => (
								<div
									key={stat.label}
									className="rounded-xl border border-gray-200 bg-white p-6"
								>
									<div className="text-2xl font-bold text-cw-green">{stat.value}</div>
									<div className="mt-1 text-sm text-gray-600">{stat.label}</div>
								</div>
							))}
						</div>
					</div>
				</div>
			</section>

			{/* CTA Section with background image */}
			<section className="relative bg-cw-dark px-6 py-24">
				<Image
					src="/renders/hero-sidewalk.png"
					alt="CleanWalker robot on a European city sidewalk"
					fill
					className="object-cover opacity-30"
					sizes="100vw"
				/>
				<div className="absolute inset-0 bg-cw-dark/60" />
				<div className="relative z-10 mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						Ready for cleaner public spaces?
					</h2>
					<p className="mx-auto mt-4 max-w-xl text-lg text-gray-400">
						Schedule a site assessment and see how CleanWalker can transform litter
						collection at your facility.
					</p>
					<div className="mt-10 flex flex-wrap justify-center gap-4">
						<a
							href="/contact"
							className="rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
						>
							Schedule a Demo
						</a>
						<a
							href="/contact"
							className="rounded-lg border border-white/20 bg-white/5 px-8 py-3.5 text-sm font-semibold text-white backdrop-blur-sm transition-colors hover:border-white/40 hover:bg-white/10"
						>
							Contact Sales
						</a>
					</div>
				</div>
			</section>
		</div>
	);
}
