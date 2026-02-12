#!/usr/bin/env node

// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

/**
 * OpenGraph Cache Invalidation Script
 *
 * Pings social platforms to re-scrape OG metadata after deploys
 * that change OG images or meta tags.
 *
 * Usage:
 *   node scripts/invalidate-og-cache.mjs                     # all default pages
 *   node scripts/invalidate-og-cache.mjs /product /contact    # specific paths
 */

const BASE_URL = "https://cleanwalkerrobotics.com";

const DEFAULT_PATHS = [
	"/",
	"/product",
	"/pilot",
	"/demos",
	"/contact",
	"/about",
	"/insights",
];

const paths = process.argv.length > 2 ? process.argv.slice(2) : DEFAULT_PATHS;
const urls = paths.map((p) => `${BASE_URL}${p.startsWith("/") ? p : `/${p}`}`);

const results = [];

async function invalidateFacebook(url) {
	try {
		const res = await fetch(
			`https://graph.facebook.com/?id=${encodeURIComponent(url)}&scrape=true`,
			{ method: "POST" },
		);
		const status = res.ok ? "success" : `error (${res.status})`;
		let detail = "";
		if (res.ok) {
			try {
				const data = await res.json();
				detail = data.title ? ` — title: "${data.title}"` : "";
			} catch {
				// response may not be JSON
			}
		}
		return { platform: "Facebook/Meta", url, status, detail };
	} catch (err) {
		return {
			platform: "Facebook/Meta",
			url,
			status: `error: ${err.message}`,
			detail: "",
		};
	}
}

console.log("OpenGraph Cache Invalidation");
console.log("============================");
console.log(`Base URL: ${BASE_URL}`);
console.log(`Pages to invalidate: ${urls.length}\n`);

for (const url of urls) {
	console.log(`--- ${url} ---`);

	// Facebook/Meta — POST scrape endpoint (no auth needed)
	const fbResult = await invalidateFacebook(url);
	results.push(fbResult);
	console.log(`  Facebook: ${fbResult.status}${fbResult.detail}`);

	// Twitter/X — No public API for cache invalidation
	// Users must manually paste URLs into https://cards-dev.twitter.com/validator
	console.log("  Twitter/X: manual — use https://cards-dev.twitter.com/validator");

	// LinkedIn — Requires OAuth2 auth, documented for future implementation
	// POST https://api.linkedin.com/v2/ugcPosts needs auth token
	console.log("  LinkedIn: skipped — requires OAuth2 authentication");

	// Telegram — No public cache invalidation API
	// Telegram caches OG data for ~24h and refreshes automatically
	console.log("  Telegram: automatic — refreshes within ~24 hours");

	console.log("");
}

// Summary
const succeeded = results.filter((r) => r.status === "success").length;
const failed = results.filter((r) => r.status !== "success").length;

console.log("============================");
console.log("Summary");
console.log(`  Total URLs: ${urls.length}`);
console.log(`  Facebook scrape requests: ${results.length}`);
console.log(`  Succeeded: ${succeeded}`);
console.log(`  Failed: ${failed}`);
console.log("");
console.log("Platforms requiring manual action:");
console.log("  - Twitter/X: Paste URLs into https://cards-dev.twitter.com/validator");
console.log(
	"  - LinkedIn: POST to https://api.linkedin.com/v2/ugcPosts (needs OAuth2 token)",
);
console.log("  - Telegram: Auto-refreshes within ~24 hours, no action needed");
