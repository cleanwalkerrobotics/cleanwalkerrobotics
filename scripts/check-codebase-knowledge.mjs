#!/usr/bin/env node
/**
 * check-codebase-knowledge.mjs
 *
 * Build-time validation for the codebase knowledge management system.
 * Runs as prebuild to ensure:
 *   1. CODEBASE.md exists and is current (regenerates + diffs)
 *   2. All division roots have PURPOSE.md with minimum content
 *
 * Exit codes:
 *   0 — All checks pass
 *   1 — PURPOSE.md missing or empty in a division root
 *
 * Note: If CODEBASE.md is stale, the script regenerates it and warns
 * (non-blocking). Missing PURPOSE.md files block the build.
 */

import { readFile, access } from "node:fs/promises";
import { join } from "node:path";
import { execSync } from "node:child_process";

const ROOT = new URL("..", import.meta.url).pathname.replace(/\/$/, "");

const DIVISION_ROOTS = [
	"apps/web",
	"firmware",
	"ml",
	"hardware",
	"docs",
	"services",
	"packages",
];

const REQUIRED_FIELDS = ["Owner:", "What belongs here:"];

let errors = 0;
let warnings = 0;

// --- Check 1: PURPOSE.md existence and content ---
console.log("Checking PURPOSE.md files...");

for (const div of DIVISION_ROOTS) {
	const purposePath = join(ROOT, div, "PURPOSE.md");

	try {
		await access(purposePath);
	} catch {
		console.error(`  ✗ MISSING: ${div}/PURPOSE.md`);
		errors++;
		continue;
	}

	const content = await readFile(purposePath, "utf-8");
	const trimmed = content.trim();

	if (trimmed.length < 20) {
		console.error(`  ✗ EMPTY/TOO SHORT: ${div}/PURPOSE.md (${trimmed.length} chars, need 20+)`);
		errors++;
		continue;
	}

	// Check for required fields (warn only, don't block)
	for (const field of REQUIRED_FIELDS) {
		if (!content.includes(field)) {
			console.warn(`  ⚠ ${div}/PURPOSE.md missing "${field}" field`);
			warnings++;
		}
	}

	console.log(`  ✓ ${div}/PURPOSE.md`);
}

// --- Check 2: CODEBASE.md freshness ---
console.log("\nChecking CODEBASE.md freshness...");

const codebasePath = join(ROOT, "CODEBASE.md");

try {
	await access(codebasePath);
} catch {
	console.warn("  ⚠ CODEBASE.md not found — generating...");
	execSync("node scripts/generate-codebase-map.mjs", { cwd: ROOT, stdio: "inherit" });
	warnings++;
}

// Regenerate to a temp comparison (read current, regenerate, compare structure)
// We compare structural content only (skip timestamp line)
try {
	const currentContent = await readFile(codebasePath, "utf-8");
	const currentLines = currentContent
		.split("\n")
		.filter((l) => !l.includes("Last generated:"))
		.join("\n");

	// Regenerate
	execSync("node scripts/generate-codebase-map.mjs", { cwd: ROOT, stdio: "pipe" });

	const freshContent = await readFile(codebasePath, "utf-8");
	const freshLines = freshContent
		.split("\n")
		.filter((l) => !l.includes("Last generated:"))
		.join("\n");

	if (currentLines !== freshLines) {
		console.warn("  ⚠ CODEBASE.md was stale — regenerated. Remember to commit the updated file.");
		warnings++;
	} else {
		console.log("  ✓ CODEBASE.md is current");
	}
} catch (err) {
	console.warn(`  ⚠ Could not verify CODEBASE.md freshness: ${err.message}`);
	warnings++;
}

// --- Check 3: Demo registry (if registry.json exists) ---
const registryPath = join(ROOT, "apps/web/src/app/demos/registry.json");
try {
	await access(registryPath);
	console.log("\nChecking demo registry...");

	const { readdirSync } = await import("node:fs");
	const demosDir = join(ROOT, "apps/web/src/app/demos");
	const registryData = JSON.parse(await readFile(registryPath, "utf-8"));
	const demos = Array.isArray(registryData) ? registryData : registryData.demos || [];
	const registeredIds = new Set(demos.map((d) => d.id));

	const dirs = readdirSync(demosDir, { withFileTypes: true })
		.filter((d) => d.isDirectory() && !d.name.startsWith(".") && !d.name.startsWith("_"))
		.map((d) => d.name);

	const unregistered = dirs.filter((d) => !registeredIds.has(d));
	if (unregistered.length > 0) {
		console.error(`  ✗ Unregistered demos: ${unregistered.join(", ")}`);
		console.error("    Add them to apps/web/src/app/demos/registry.json");
		errors++;
	} else {
		console.log(`  ✓ All ${dirs.length} demos registered`);
	}
} catch {
	// No registry.json — skip silently
}

// --- Summary ---
console.log("");
if (errors > 0) {
	console.error(`✗ ${errors} error(s), ${warnings} warning(s). Fix errors before building.`);
	process.exit(1);
}

if (warnings > 0) {
	console.log(`✓ Passed with ${warnings} warning(s).`);
} else {
	console.log("✓ All codebase knowledge checks passed.");
}
