// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

export default function Home() {
	return (
		<div>
			{/* Hero Section */}
			<section className="relative bg-cw-dark px-6 py-24 md:py-32">
				<div className="mx-auto max-w-7xl">
					<div className="grid items-center gap-12 md:grid-cols-2">
						<div>
							<div className="mb-6 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm font-medium text-cw-green">
								Robot-as-a-Service
							</div>
							<h1 className="text-4xl font-bold leading-tight tracking-tight text-white md:text-6xl">
								Autonomous Litter Collection for{" "}
								<span className="text-cw-green">Cleaner Spaces</span>
							</h1>
							<p className="mt-6 max-w-lg text-lg leading-relaxed text-gray-400">
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
									className="rounded-lg border border-white/20 px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:border-white/40 hover:bg-white/5"
								>
									See How It Works
								</a>
							</div>
							<div className="mt-10 flex items-center gap-8 text-sm text-gray-400">
								<div>
									<span className="block text-2xl font-bold text-white">24/7</span>
									Autonomous operation
								</div>
								<div className="h-10 w-px bg-white/10" />
								<div>
									<span className="block text-2xl font-bold text-white">48%</span>
									Cost reduction
								</div>
								<div className="h-10 w-px bg-white/10" />
								<div>
									<span className="block text-2xl font-bold text-white">5-10</span>
									Acres per day
								</div>
							</div>
						</div>
						{/* Hero Image Placeholder */}
						<div className="relative aspect-[4/3] overflow-hidden rounded-2xl bg-gradient-to-br from-cw-charcoal to-cw-gray">
							<div className="absolute inset-0 flex flex-col items-center justify-center p-8 text-center">
								<div className="mb-4 rounded-full bg-cw-green/20 p-4">
									<svg
										className="h-8 w-8 text-cw-green"
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
								<p className="text-lg font-medium text-gray-300">
									Robot Render Coming Soon
								</p>
								<p className="mt-2 text-sm text-gray-500">
									CleanWalker robot in park setting — golden hour lighting
								</p>
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

			{/* Fleet Image Placeholder */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="relative aspect-[21/9] overflow-hidden rounded-2xl bg-gradient-to-br from-cw-charcoal to-cw-gray">
						<div className="absolute inset-0 flex flex-col items-center justify-center p-8 text-center">
							<div className="mb-4 rounded-full bg-cw-green/20 p-4">
								<svg
									className="h-8 w-8 text-cw-green"
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
							<p className="text-lg font-medium text-gray-300">
								Fleet of CleanWalker Robots Working in Urban Park
							</p>
							<p className="mt-2 text-sm text-gray-500">
								Three robots collecting litter — drone perspective, morning light
							</p>
						</div>
					</div>
				</div>
			</section>

			{/* Social Proof */}
			<section className="bg-cw-light px-6 py-24">
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
								className="flex h-16 items-center justify-center rounded-xl border border-gray-200 bg-white px-6"
							>
								<span className="text-sm font-medium text-gray-400">{name}</span>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* ROI Section */}
			<section className="bg-white px-6 py-24">
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
									className="rounded-xl border border-gray-200 bg-cw-light p-6"
								>
									<div className="text-2xl font-bold text-cw-green">{stat.value}</div>
									<div className="mt-1 text-sm text-gray-600">{stat.label}</div>
								</div>
							))}
						</div>
					</div>
				</div>
			</section>

			{/* CTA Section */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
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
							className="rounded-lg border border-white/20 px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:border-white/40 hover:bg-white/5"
						>
							Contact Sales
						</a>
					</div>
				</div>
			</section>
		</div>
	);
}
