// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import { articles } from "./articles";

export const metadata: Metadata = {
	title: "Insights",
	description:
		"Articles on autonomous robotics, litter collection technology, and the future of municipal waste management from the CleanWalker team.",
};

export default function InsightsPage() {
	return (
		<div className="bg-cw-dark">
			{/* Hero */}
			<section className="border-b border-white/10 py-20">
				<div className="mx-auto max-w-7xl px-6">
					<p className="mb-3 text-sm font-semibold uppercase tracking-wider text-cw-green">
						Insights
					</p>
					<h1 className="text-4xl font-bold text-white md:text-5xl">
						Thinking About the Future of Clean Cities
					</h1>
					<p className="mt-4 max-w-2xl text-lg text-gray-400">
						Research, analysis, and perspectives on autonomous robotics, urban
						cleanliness, and the technology driving the next generation of
						municipal services.
					</p>
				</div>
			</section>

			{/* Article Grid */}
			<section className="py-20">
				<div className="mx-auto max-w-7xl px-6">
					<div className="grid gap-8 md:grid-cols-2 lg:grid-cols-3">
						{articles.map((article) => (
							<a
								key={article.slug}
								href={`/insights/${article.slug}`}
								className="group rounded-2xl border border-white/10 bg-white/[0.02] p-8 transition-all hover:border-cw-green/30 hover:bg-white/[0.04]"
							>
								<div className="mb-4 flex items-center gap-3 text-sm text-gray-500">
									<time dateTime={article.date}>
										{new Date(article.date).toLocaleDateString("en-US", {
											year: "numeric",
											month: "long",
											day: "numeric",
										})}
									</time>
									<span className="h-1 w-1 rounded-full bg-gray-600" />
									<span>{article.readTime}</span>
								</div>
								<h2 className="text-xl font-bold text-white transition-colors group-hover:text-cw-green">
									{article.title}
								</h2>
								<p className="mt-3 text-sm leading-relaxed text-gray-400">
									{article.description}
								</p>
								<div className="mt-6 flex items-center gap-2 text-sm font-medium text-cw-green">
									Read article
									<svg
										className="h-4 w-4 transition-transform group-hover:translate-x-1"
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
								</div>
							</a>
						))}
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="border-t border-white/10 py-20">
				<div className="mx-auto max-w-3xl px-6 text-center">
					<h2 className="text-3xl font-bold text-white">
						Ready to See It in Action?
					</h2>
					<p className="mt-4 text-gray-400">
						Talk to our team about deploying autonomous litter collection in
						your city, campus, or facility.
					</p>
					<a
						href="/contact"
						className="mt-8 inline-block rounded-lg bg-cw-green px-8 py-3 font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
				</div>
			</section>
		</div>
	);
}
