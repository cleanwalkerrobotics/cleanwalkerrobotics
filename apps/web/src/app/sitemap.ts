// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { MetadataRoute } from "next";
import { articles } from "./insights/articles";

const siteUrl = "https://cleanwalkerrobotics.com";

export default function sitemap(): MetadataRoute.Sitemap {
	const staticPages = [
		{ url: siteUrl, lastModified: new Date(), priority: 1.0 },
		{ url: `${siteUrl}/product`, lastModified: new Date(), priority: 0.9 },
		{ url: `${siteUrl}/pilot`, lastModified: new Date(), priority: 0.8 },
		{ url: `${siteUrl}/demos`, lastModified: new Date(), priority: 0.7 },
		{ url: `${siteUrl}/about`, lastModified: new Date(), priority: 0.7 },
		{ url: `${siteUrl}/contact`, lastModified: new Date(), priority: 0.8 },
		{ url: `${siteUrl}/insights`, lastModified: new Date(), priority: 0.8 },
		{ url: `${siteUrl}/privacy`, lastModified: new Date(), priority: 0.3 },
		{ url: `${siteUrl}/terms`, lastModified: new Date(), priority: 0.3 },
	];

	const articlePages = articles.map((article) => ({
		url: `${siteUrl}/insights/${article.slug}`,
		lastModified: new Date(article.date),
		priority: 0.6,
	}));

	return [...staticPages, ...articlePages];
}
