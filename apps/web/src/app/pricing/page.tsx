// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";

export const metadata: Metadata = {
	title: "Pricing — CleanWalker Robotics",
	description:
		"Robot-as-a-Service pricing starting at $2,800/mo. No upfront capital, no maintenance costs. Pilot, Standard, and Fleet tiers available.",
};

const tiers = [
	{
		name: "Pilot",
		price: "$2,800",
		period: "/mo per robot",
		tagline: "Try before you scale",
		commitment: "6-month minimum",
		units: "1-3 robots",
		highlight: false,
		features: [
			"Full onboarding and site assessment",
			"Weekly performance reports",
			"Business hours support",
			"All hardware and maintenance included",
			"Charging dock included",
			"Software updates (OTA)",
		],
		bestFor:
			"Municipalities and property managers evaluating autonomous litter collection for the first time.",
		cta: "Start a Pilot",
	},
	{
		name: "Standard",
		price: "$3,500",
		period: "/mo per robot",
		tagline: "The full CleanWalker experience",
		commitment: "12-month contract",
		units: "1-49 robots",
		highlight: true,
		features: [
			"Dedicated account manager",
			"Real-time fleet dashboard",
			"Monthly performance analytics",
			"Priority support (4-hour SLA)",
			"All hardware and maintenance included",
			"Charging dock included",
			"Software updates (OTA)",
		],
		bestFor:
			"Established deployments at parks, campuses, commercial districts, and HOA communities.",
		cta: "Get Started",
	},
	{
		name: "Fleet",
		price: "$3,000",
		period: "/mo per robot",
		tagline: "Scale across your operation",
		commitment: "24-month contract",
		units: "50+ robots",
		highlight: false,
		features: [
			"Custom deployment planning",
			"API access for system integration",
			"Dedicated field service team",
			"Quarterly business reviews",
			"Priority support (4-hour SLA)",
			"All hardware and maintenance included",
			"Charging dock included",
			"Software updates (OTA)",
		],
		bestFor:
			"City-wide deployments, large university systems, theme parks, and national property management companies.",
		cta: "Contact Sales",
	},
];

const included = [
	{
		item: "Autonomous robot",
		detail: "Quadrupedal litter-collecting robot, weatherproof (IP54+)",
	},
	{
		item: "Charging dock",
		detail: "Self-docking, no manual charging required",
	},
	{
		item: "Fleet software",
		detail: "Real-time dashboard, route planning, telemetry, alerts",
	},
	{
		item: "4G connectivity",
		detail: "Always-on cellular connection, no WiFi required",
	},
	{
		item: "Preventive maintenance",
		detail: "Scheduled service visits, consumable replacements",
	},
	{
		item: "Software updates",
		detail: "Over-the-air firmware and ML model updates",
	},
	{
		item: "Replacement coverage",
		detail: "Failed components replaced at no additional cost",
	},
	{
		item: "Onboarding",
		detail: "Site assessment, deployment, and staff training",
	},
];

const addOns = [
	{
		name: "Extended Hours Support",
		price: "+$200/mo",
		desc: "24/7 phone support with 2-hour response SLA",
	},
	{
		name: "Additional Charging Dock",
		price: "+$150/mo",
		desc: "For sites requiring multiple docking locations",
	},
	{
		name: "Advanced Analytics",
		price: "+$100/mo",
		desc: "Litter heatmaps, trend analysis, waste composition reporting",
	},
	{
		name: "Custom Branding Wrap",
		price: "$500 one-time",
		desc: "Your logo and colors on the robot body",
	},
	{
		name: "API Integration Support",
		price: "$2,500 one-time",
		desc: "Engineering support to integrate with your existing systems",
	},
	{
		name: "Weekend/Holiday Priority",
		price: "+$300/mo",
		desc: "Guaranteed operation scheduling for peak visitor periods",
	},
];

export default function PricingPage() {
	return (
		<div>
			{/* Hero */}
			<section className="bg-cw-dark px-6 py-24 md:py-32">
				<div className="mx-auto max-w-4xl text-center">
					<h1 className="text-4xl font-bold tracking-tight text-white md:text-5xl">
						Simple, Transparent <span className="text-cw-green">Pricing</span>
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg text-gray-400">
						Robot-as-a-Service. No upfront capital, no hidden fees. We handle
						hardware, software, maintenance, and support — you get cleaner spaces.
					</p>
				</div>
			</section>

			{/* Pricing Cards */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="grid gap-8 md:grid-cols-3">
						{tiers.map((tier) => (
							<div
								key={tier.name}
								className={`relative rounded-2xl p-8 ${
									tier.highlight
										? "border-2 border-cw-green bg-white shadow-lg"
										: "border border-gray-200 bg-white"
								}`}
							>
								{tier.highlight && (
									<div className="absolute -top-4 left-1/2 -translate-x-1/2 rounded-full bg-cw-green px-4 py-1 text-xs font-semibold text-white">
										Most Popular
									</div>
								)}
								<div>
									<h3 className="text-lg font-semibold text-gray-900">{tier.name}</h3>
									<p className="mt-1 text-sm text-gray-500">{tier.tagline}</p>
								</div>
								<div className="mt-6">
									<span className="text-4xl font-bold text-gray-900">
										{tier.price}
									</span>
									<span className="text-sm text-gray-500">{tier.period}</span>
								</div>
								<div className="mt-2 space-y-1 text-sm text-gray-500">
									<p>{tier.commitment}</p>
									<p>{tier.units}</p>
								</div>
								<a
									href="/contact"
									className={`mt-8 block rounded-lg py-3 text-center text-sm font-semibold transition-colors ${
										tier.highlight
											? "bg-cw-green text-white hover:bg-cw-green-dark"
											: "border border-gray-300 text-gray-700 hover:bg-gray-50"
									}`}
								>
									{tier.cta}
								</a>
								<ul className="mt-8 space-y-3">
									{tier.features.map((feature) => (
										<li key={feature} className="flex items-start gap-3 text-sm">
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
													d="M4.5 12.75l6 6 9-13.5"
												/>
											</svg>
											<span className="text-gray-600">{feature}</span>
										</li>
									))}
								</ul>
								<p className="mt-8 border-t border-gray-100 pt-6 text-xs leading-relaxed text-gray-400">
									Best for: {tier.bestFor}
								</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* What's Included */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Everything Included in Every Plan
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							No surprises. Your monthly fee covers the complete service.
						</p>
					</div>
					<div className="mt-16 grid gap-6 md:grid-cols-2 lg:grid-cols-4">
						{included.map((item) => (
							<div
								key={item.item}
								className="rounded-xl border border-gray-200 bg-cw-light p-6"
							>
								<div className="flex h-10 w-10 items-center justify-center rounded-lg bg-cw-green/10">
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
								<h3 className="mt-4 font-semibold text-gray-900">{item.item}</h3>
								<p className="mt-1 text-sm text-gray-600">{item.detail}</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* Add-Ons */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Optional Add-Ons
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							Customize your service with these optional extras.
						</p>
					</div>
					<div className="mt-16 grid gap-6 md:grid-cols-2 lg:grid-cols-3">
						{addOns.map((addon) => (
							<div
								key={addon.name}
								className="rounded-xl border border-gray-200 bg-white p-6"
							>
								<h3 className="font-semibold text-gray-900">{addon.name}</h3>
								<p className="mt-1 text-sm font-medium text-cw-green">
									{addon.price}
								</p>
								<p className="mt-1 text-sm text-gray-600">{addon.desc}</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* ROI Calculator */}
			<section className="bg-white px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							ROI Calculator
						</h2>
						<p className="mx-auto mt-4 max-w-2xl text-lg text-gray-600">
							See how CleanWalker compares to manual litter collection.
						</p>
					</div>

					<div className="mx-auto mt-16 max-w-4xl">
						<div className="grid gap-8 md:grid-cols-2">
							{/* Manual Cost */}
							<div className="rounded-2xl border border-gray-200 p-8">
								<h3 className="text-lg font-semibold text-gray-900">
									Manual Collection (2 Workers)
								</h3>
								<div className="mt-6 space-y-4">
									<div className="flex justify-between border-b border-gray-100 pb-3">
										<span className="text-sm text-gray-500">Labor (2 FTE)</span>
										<span className="text-sm font-medium text-gray-900">
											$6,400-$8,400/mo
										</span>
									</div>
									<div className="flex justify-between border-b border-gray-100 pb-3">
										<span className="text-sm text-gray-500">
											Benefits &amp; overhead (30-40%)
										</span>
										<span className="text-sm font-medium text-gray-900">
											$1,920-$3,360/mo
										</span>
									</div>
									<div className="flex justify-between border-b border-gray-100 pb-3">
										<span className="text-sm text-gray-500">
											Equipment &amp; supplies
										</span>
										<span className="text-sm font-medium text-gray-900">
											$200-$400/mo
										</span>
									</div>
									<div className="flex justify-between border-b border-gray-100 pb-3">
										<span className="text-sm text-gray-500">Supervision</span>
										<span className="text-sm font-medium text-gray-900">
											$800-$1,200/mo
										</span>
									</div>
									<div className="flex justify-between pt-2">
										<span className="font-semibold text-gray-900">Total</span>
										<span className="font-semibold text-red-600">
											$9,320-$13,360/mo
										</span>
									</div>
								</div>
							</div>

							{/* CleanWalker Cost */}
							<div className="rounded-2xl border-2 border-cw-green bg-cw-green/5 p-8">
								<h3 className="text-lg font-semibold text-gray-900">
									CleanWalker (2 Robots)
								</h3>
								<div className="mt-6 space-y-4">
									<div className="flex justify-between border-b border-cw-green/20 pb-3">
										<span className="text-sm text-gray-500">
											2 robots (Standard tier)
										</span>
										<span className="text-sm font-medium text-gray-900">
											$7,000/mo
										</span>
									</div>
									<div className="flex justify-between border-b border-cw-green/20 pb-3">
										<span className="text-sm text-gray-500">Hardware</span>
										<span className="text-sm font-medium text-cw-green">
											Included
										</span>
									</div>
									<div className="flex justify-between border-b border-cw-green/20 pb-3">
										<span className="text-sm text-gray-500">Maintenance</span>
										<span className="text-sm font-medium text-cw-green">
											Included
										</span>
									</div>
									<div className="flex justify-between border-b border-cw-green/20 pb-3">
										<span className="text-sm text-gray-500">Setup fees</span>
										<span className="text-sm font-medium text-cw-green">$0</span>
									</div>
									<div className="flex justify-between pt-2">
										<span className="font-semibold text-gray-900">Total</span>
										<span className="font-semibold text-cw-green">$7,000/mo</span>
									</div>
								</div>
							</div>
						</div>

						{/* Savings Summary */}
						<div className="mt-8 rounded-2xl bg-cw-dark p-8 text-center">
							<div className="grid gap-8 md:grid-cols-3">
								<div>
									<div className="text-3xl font-bold text-cw-green">
										$2,320-$6,360
									</div>
									<p className="mt-1 text-sm text-gray-400">Monthly savings</p>
								</div>
								<div>
									<div className="text-3xl font-bold text-cw-green">$52K-$100K</div>
									<p className="mt-1 text-sm text-gray-400">
										Annual savings per site
									</p>
								</div>
								<div>
									<div className="text-3xl font-bold text-cw-green">25-48%</div>
									<p className="mt-1 text-sm text-gray-400">Cost reduction</p>
								</div>
							</div>
						</div>

						{/* Additional benefits */}
						<div className="mt-8 grid gap-4 md:grid-cols-3">
							{[
								{
									title: "More Coverage",
									desc: "20+ hours/day vs. 16 hours with two 8-hour shifts",
								},
								{
									title: "Real-Time Data",
									desc: "Dashboard analytics, litter heatmaps, and performance tracking",
								},
								{
									title: "Weather Proof",
									desc: "Operates in rain, at night, weekends, and holidays",
								},
							].map((benefit) => (
								<div
									key={benefit.title}
									className="rounded-xl border border-gray-200 bg-cw-light p-6 text-center"
								>
									<h4 className="font-semibold text-gray-900">{benefit.title}</h4>
									<p className="mt-1 text-sm text-gray-600">{benefit.desc}</p>
								</div>
							))}
						</div>
					</div>
				</div>
			</section>

			{/* FAQ */}
			<section className="bg-cw-light px-6 py-24">
				<div className="mx-auto max-w-3xl">
					<div className="text-center">
						<h2 className="text-3xl font-bold tracking-tight text-gray-900 md:text-4xl">
							Frequently Asked Questions
						</h2>
					</div>
					<div className="mt-16 space-y-8">
						{[
							{
								q: "How many robots do I need?",
								a: "One robot covers approximately 5-10 acres of parkland or campus per day, depending on terrain and litter density. We recommend starting with a pilot to determine optimal coverage for your site.",
							},
							{
								q: "What happens if a robot breaks down?",
								a: "All plans include replacement coverage. If a robot requires repair beyond field service, we deploy a replacement within 48 hours (24 hours for Fleet customers).",
							},
							{
								q: "Can the robots operate in rain?",
								a: "Yes. CleanWalker robots are weatherproof (IP54+) and operate in light to moderate rain. They automatically dock during severe weather events.",
							},
							{
								q: "What types of litter can it pick up?",
								a: "Bottles, cans, wrappers, cups, cigarette butts, paper, and most common litter items from 5g to 500g. The AI detection system is continuously trained on new litter types.",
							},
							{
								q: "Is there a setup fee?",
								a: "No. All onboarding, site assessment, and deployment costs are included in your monthly rate.",
							},
							{
								q: "What about liability?",
								a: "CleanWalker carries comprehensive commercial liability insurance. Robots are equipped with multiple safety systems including emergency stop, obstacle avoidance, and pedestrian detection.",
							},
						].map((faq) => (
							<div
								key={faq.q}
								className="rounded-xl border border-gray-200 bg-white p-6"
							>
								<h3 className="font-semibold text-gray-900">{faq.q}</h3>
								<p className="mt-3 leading-relaxed text-gray-600">{faq.a}</p>
							</div>
						))}
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="bg-cw-dark px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
						Ready to get started?
					</h2>
					<p className="mx-auto mt-4 max-w-xl text-lg text-gray-400">
						Contact us for a site assessment and custom quote. Government and
						multi-year pricing available.
					</p>
					<a
						href="/contact"
						className="mt-10 inline-block rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
					<p className="mt-4 text-sm text-gray-500">
						All pricing effective February 2026. Custom pricing available for
						government contracts and multi-year agreements.
					</p>
				</div>
			</section>
		</div>
	);
}
