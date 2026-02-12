#!/usr/bin/env node
/**
 * generate-codebase-map.mjs
 *
 * Auto-generates CODEBASE.md at the repo root — a compact structural map
 * of the CleanWalker Robotics monorepo for AI agent context injection.
 *
 * Usage: node scripts/generate-codebase-map.mjs
 *
 * Output: CODEBASE.md (~200 lines max, Markdown)
 */

import { readdir, stat, readFile } from "node:fs/promises";
import { join, relative, extname, basename } from "node:path";
import { execSync } from "node:child_process";
import { writeFile } from "node:fs/promises";

const ROOT = new URL("..", import.meta.url).pathname.replace(/\/$/, "");

const IGNORE = new Set([
	"node_modules",
	".git",
	"__pycache__",
	"build",
	".next",
	"dist",
	".turbo",
	"target",
	".vercel",
	".crew",
	".github",
	".claude",
	".vscode",
	".turbo",
	"coverage",
]);

const DIVISIONS = [
	{ path: "apps/web", label: "Marketing Website & Demos", stack: "Next.js 15, React 19, Tailwind CSS 4" },
	{ path: "apps/dashboard", label: "Fleet Dashboard App", stack: "Next.js 15, React 19" },
	{ path: "firmware", label: "Robot Firmware", stack: "Rust, ROS2" },
	{ path: "ml", label: "Machine Learning", stack: "Python, PyTorch, YOLO" },
	{ path: "hardware", label: "Mechanical & PCB Design", stack: "KiCad, URDF" },
	{ path: "docs", label: "Documentation", stack: null },
	{ path: "services/api", label: "Backend API", stack: "Hono, TypeScript" },
	{ path: "packages", label: "Shared Packages", stack: "TypeScript" },
	{ path: "ops", label: "Operations", stack: null },
];

const KEY_FILES = new Set([
	"package.json",
	"Cargo.toml",
	"Cargo.lock",
	"tsconfig.json",
	"next.config.ts",
	"next.config.js",
	"next.config.mjs",
	"turbo.json",
	"PURPOSE.md",
	"README.md",
	"requirements.txt",
	"pyproject.toml",
	"registry.json",
	"drizzle.config.ts",
]);

async function walkDir(dir, depth = 0, maxDepth = 3) {
	const entries = [];
	let files;
	try {
		files = await readdir(dir, { withFileTypes: true });
	} catch {
		return entries;
	}

	for (const f of files) {
		if (IGNORE.has(f.name)) continue;
		if (f.name.startsWith(".") && f.name !== ".env.example") continue;

		const fullPath = join(dir, f.name);
		const relPath = relative(ROOT, fullPath);

		if (f.isDirectory()) {
			const children = depth < maxDepth ? await walkDir(fullPath, depth + 1, maxDepth) : [];
			entries.push({ name: f.name, type: "dir", path: relPath, children });
		} else {
			entries.push({ name: f.name, type: "file", path: relPath });
		}
	}

	return entries;
}

function countFiles(entries) {
	let count = 0;
	for (const e of entries) {
		if (e.type === "file") count++;
		else if (e.children) count += countFiles(e.children);
	}
	return count;
}

function collectExtensions(entries) {
	const exts = {};
	for (const e of entries) {
		if (e.type === "file") {
			const ext = extname(e.name) || "(no ext)";
			exts[ext] = (exts[ext] || 0) + 1;
		} else if (e.children) {
			const childExts = collectExtensions(e.children);
			for (const [k, v] of Object.entries(childExts)) {
				exts[k] = (exts[k] || 0) + v;
			}
		}
	}
	return exts;
}

function findKeyFiles(entries) {
	const found = [];
	for (const e of entries) {
		if (e.type === "file" && KEY_FILES.has(e.name)) {
			found.push(e.path);
		} else if (e.children) {
			found.push(...findKeyFiles(e.children));
		}
	}
	return found;
}

function buildTree(entries, indent = "", maxDepth = 2, currentDepth = 0) {
	const lines = [];
	const sorted = [...entries].sort((a, b) => {
		if (a.type === "dir" && b.type !== "dir") return -1;
		if (a.type !== "dir" && b.type === "dir") return 1;
		return a.name.localeCompare(b.name);
	});

	for (let i = 0; i < sorted.length; i++) {
		const e = sorted[i];
		const isLast = i === sorted.length - 1;
		const connector = isLast ? "└── " : "├── ";
		const childIndent = indent + (isLast ? "    " : "│   ");

		if (e.type === "dir") {
			const fileCount = countFiles(e.children || []);
			const suffix = fileCount > 0 ? ` (${fileCount} files)` : "";
			lines.push(`${indent}${connector}${e.name}/${suffix}`);
			if (currentDepth < maxDepth && e.children) {
				lines.push(...buildTree(e.children, childIndent, maxDepth, currentDepth + 1));
			}
		} else if (KEY_FILES.has(e.name)) {
			lines.push(`${indent}${connector}${e.name}`);
		}
	}
	return lines;
}

function getRecentlyModified() {
	try {
		const out = execSync(
			'git log --since="7 days ago" --diff-filter=ACMR --name-only --pretty=format:"" HEAD',
			{ cwd: ROOT, encoding: "utf-8", timeout: 10000 }
		);
		const files = out.split("\n").filter(Boolean);
		// Group by top-level directory
		const groups = {};
		for (const f of files) {
			if (f.startsWith(".") || f.includes("node_modules")) continue;
			const parts = f.split("/");
			const key = parts.length > 1 ? parts.slice(0, 2).join("/") : parts[0];
			if (!groups[key]) groups[key] = 0;
			groups[key]++;
		}
		return Object.entries(groups)
			.sort((a, b) => b[1] - a[1])
			.slice(0, 10)
			.map(([dir, count]) => `- ${dir} (${count} files changed)`);
	} catch {
		return ["- (git log unavailable)"];
	}
}

function getEntryPoints(entries, divPath) {
	const points = [];
	for (const e of entries) {
		if (e.type === "file") {
			const n = e.name;
			if (n === "page.tsx" || n === "layout.tsx" || n === "route.ts") {
				points.push(e.path);
			} else if (n === "main.rs" || n === "lib.rs" || n === "mod.rs") {
				points.push(e.path);
			} else if (n === "index.ts" || n === "index.tsx" || n === "index.js") {
				points.push(e.path);
			} else if (n === "main.py" || n === "__main__.py") {
				points.push(e.path);
			}
		} else if (e.children) {
			points.push(...getEntryPoints(e.children, divPath));
		}
	}
	return points;
}

async function readPurpose(divPath) {
	try {
		const content = await readFile(join(ROOT, divPath, "PURPOSE.md"), "utf-8");
		const firstLine = content.split("\n").find(l => l.trim() && !l.startsWith("#"));
		return firstLine ? firstLine.trim() : null;
	} catch {
		return null;
	}
}

function findSubdirs(entries) {
	return entries.filter(e => e.type === "dir").map(e => e.name);
}

async function getDemoList() {
	try {
		const reg = await readFile(join(ROOT, "apps/web/src/app/demos/registry.json"), "utf-8");
		const data = JSON.parse(reg);
		const demos = Array.isArray(data) ? data : data.demos || [];
		return demos.filter(d => d.status === "active").map(d => d.id);
	} catch {
		return [];
	}
}

async function getRustCrates() {
	try {
		const cargo = await readFile(join(ROOT, "firmware/Cargo.toml"), "utf-8");
		const match = cargo.match(/members\s*=\s*\[([\s\S]*?)\]/);
		if (match) {
			return match[1].match(/"([^"]+)"/g)?.map(s => s.replace(/"/g, "")) || [];
		}
	} catch {}
	return [];
}

async function getPackagesList() {
	try {
		const dirs = await readdir(join(ROOT, "packages"), { withFileTypes: true });
		return dirs.filter(d => d.isDirectory() && !IGNORE.has(d.name) && !d.name.startsWith(".")).map(d => d.name);
	} catch {
		return [];
	}
}

async function main() {
	const now = new Date().toISOString().replace(/\.\d+Z$/, "Z");
	const lines = [];

	lines.push("# CleanWalker Robotics — Codebase Map");
	lines.push("<!-- AUTO-GENERATED by scripts/generate-codebase-map.mjs — DO NOT EDIT -->");
	lines.push(`<!-- Last generated: ${now} -->`);
	lines.push("");

	// Count total tracked files
	const allEntries = await walkDir(ROOT, 0, 4);
	const totalFiles = countFiles(allEntries);
	lines.push("## Overview");
	lines.push(`${totalFiles} tracked files across ${DIVISIONS.length} divisions. Monorepo managed with pnpm + Turborepo.`);
	lines.push("");

	// Root-level tree (depth 1)
	lines.push("## Repository Root");
	lines.push("```");
	const rootTree = buildTree(allEntries, "", 0);
	lines.push(...rootTree);
	lines.push("```");
	lines.push("");

	lines.push("## Divisions");
	lines.push("");

	for (const div of DIVISIONS) {
		const divFullPath = join(ROOT, div.path);
		let divEntries;
		try {
			divEntries = await walkDir(divFullPath, 0, 3);
		} catch {
			continue; // Division doesn't exist
		}
		if (divEntries.length === 0) continue;

		const fileCount = countFiles(divEntries);
		const purpose = await readPurpose(div.path);
		const stackStr = div.stack ? ` | **Stack:** ${div.stack}` : "";
		const keyFiles = findKeyFiles(divEntries);

		lines.push(`### ${div.path}/ — ${div.label}`);
		lines.push(`**Files:** ${fileCount}${stackStr}`);
		if (keyFiles.length > 0) {
			lines.push(`**Key files:** ${keyFiles.join(", ")}`);
		}
		if (purpose) {
			lines.push(`**Purpose:** ${purpose}`);
		}
		lines.push("");

		// Division-specific details
		if (div.path === "apps/web") {
			const demos = await getDemoList();
			if (demos.length > 0) {
				lines.push(`**Demos (${demos.length}):** ${demos.join(", ")}`);
			}
		} else if (div.path === "firmware") {
			const crates = await getRustCrates();
			if (crates.length > 0) {
				lines.push(`**Crates:** ${crates.join(", ")}`);
			}
			const subdirs = findSubdirs(divEntries);
			const ros2 = subdirs.filter(d => d === "ros2");
			if (ros2.length > 0) {
				try {
					const ros2Entries = await readdir(join(ROOT, "firmware/ros2"), { withFileTypes: true });
					const ros2Pkgs = ros2Entries.filter(d => d.isDirectory() && !IGNORE.has(d.name)).map(d => d.name);
					if (ros2Pkgs.length > 0) {
						lines.push(`**ROS2 packages:** ${ros2Pkgs.join(", ")}`);
					}
				} catch {}
			}
		} else if (div.path === "packages") {
			const pkgs = await getPackagesList();
			if (pkgs.length > 0) {
				lines.push(`**Packages:** ${pkgs.join(", ")}`);
			}
		} else if (div.path === "ml") {
			const subdirs = findSubdirs(divEntries);
			if (subdirs.length > 0) {
				lines.push(`**Modules:** ${subdirs.join(", ")}`);
			}
		} else if (div.path === "hardware") {
			const subdirs = findSubdirs(divEntries);
			if (subdirs.length > 0) {
				lines.push(`**Subsystems:** ${subdirs.join(", ")}`);
			}
		} else if (div.path === "docs") {
			const subdirs = findSubdirs(divEntries);
			if (subdirs.length > 0) {
				lines.push(`**Categories:** ${subdirs.join(", ")}`);
			}
		}

		// Compact tree (depth 2)
		lines.push("");
		lines.push("```");
		const tree = buildTree(divEntries, "", 1);
		lines.push(...tree);
		lines.push("```");
		lines.push("");
	}

	// Recently modified
	lines.push("## Recently Modified (last 7 days)");
	const recent = getRecentlyModified();
	lines.push(...recent);
	lines.push("");

	// Scripts
	lines.push("## Build & Scripts");
	lines.push("| Script | Purpose |");
	lines.push("|--------|---------|");
	lines.push("| `pnpm build` | Turborepo build (all packages) |");
	lines.push("| `pnpm prebuild` | Validate CODEBASE.md freshness + PURPOSE.md existence |");
	lines.push("| `node scripts/generate-codebase-map.mjs` | Regenerate this file |");
	lines.push("");

	const output = lines.join("\n");
	const outputPath = join(ROOT, "CODEBASE.md");
	await writeFile(outputPath, output, "utf-8");

	const lineCount = output.split("\n").length;
	console.log(`✓ Generated CODEBASE.md (${lineCount} lines, ${output.length} bytes)`);

	if (lineCount > 200) {
		console.warn(`⚠ CODEBASE.md exceeds 200-line target (${lineCount} lines). Consider trimming.`);
	}
}

main().catch((err) => {
	console.error("✗ Failed to generate CODEBASE.md:", err.message);
	process.exit(1);
});
