// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import { notFound } from "next/navigation";
import { articles } from "../articles";

export function generateStaticParams() {
	return articles.map((article) => ({ slug: article.slug }));
}

export async function generateMetadata({
	params,
}: { params: Promise<{ slug: string }> }): Promise<Metadata> {
	const { slug } = await params;
	const article = articles.find((a) => a.slug === slug);
	if (!article) return {};
	return {
		title: article.title,
		description: article.description,
	};
}

export default async function ArticlePage({
	params,
}: { params: Promise<{ slug: string }> }) {
	const { slug } = await params;
	const article = articles.find((a) => a.slug === slug);
	if (!article) notFound();

	return (
		<div className="bg-cw-dark">
			<article className="mx-auto max-w-3xl px-6 py-20">
				{/* Header */}
				<div className="mb-12">
					<a
						href="/insights"
						className="mb-6 inline-flex items-center gap-2 text-sm text-gray-400 transition-colors hover:text-cw-green"
					>
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
								d="M10.5 19.5L3 12m0 0l7.5-7.5M3 12h18"
							/>
						</svg>
						All Insights
					</a>
					<h1 className="text-3xl font-bold leading-tight text-white md:text-4xl">
						{article.title}
					</h1>
					<div className="mt-4 flex items-center gap-3 text-sm text-gray-500">
						<span>{article.author}</span>
						<span className="h-1 w-1 rounded-full bg-gray-600" />
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
				</div>

				{/* Body */}
				<div
					className="prose-cw"
					dangerouslySetInnerHTML={{ __html: article.body }}
				/>

				{/* CTA */}
				<div className="mt-16 rounded-2xl border border-white/10 bg-white/[0.02] p-8 text-center">
					<h2 className="text-2xl font-bold text-white">
						Interested in Autonomous Litter Collection?
					</h2>
					<p className="mt-3 text-gray-400">
						Learn how CleanWalker can help your city, campus, or facility stay
						clean around the clock.
					</p>
					<a
						href="/contact"
						className="mt-6 inline-block rounded-lg bg-cw-green px-8 py-3 font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Get in Touch
					</a>
				</div>
			</article>
		</div>
	);
}
