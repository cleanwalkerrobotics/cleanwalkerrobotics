#!/usr/bin/env node
// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

/**
 * Codebase Manifest Checker
 *
 * Validates that every significant file in each division is tracked
 * in its MANIFEST.md. Supersedes the demo registry check.
 *
 * Exit code 1 = untracked files found (build fails).
 * Exit code 0 = all files tracked.
 */

import { readFileSync, readdirSync, statSync, existsSync } from "node:fs";
import { join, resolve, relative, extname } from "node:path";

const ROOT = resolve(import.meta.dirname, "..");

// ── Division definitions ────────────────────────────────────────
const DIVISIONS = [
  { name: "apps/web", dir: join(ROOT, "apps/web"), manifest: join(ROOT, "apps/web/MANIFEST.md") },
  { name: "firmware", dir: join(ROOT, "firmware"), manifest: join(ROOT, "firmware/MANIFEST.md") },
  { name: "ml", dir: join(ROOT, "ml"), manifest: join(ROOT, "ml/MANIFEST.md") },
  { name: "hardware", dir: join(ROOT, "hardware"), manifest: join(ROOT, "hardware/MANIFEST.md") },
  { name: "docs", dir: join(ROOT, "docs"), manifest: join(ROOT, "docs/MANIFEST.md") },
];

// ── Directories to always skip ──────────────────────────────────
const SKIP_DIRS = new Set([
  "node_modules", ".next", "target", "__pycache__", ".turbo",
  "dist", "build", ".git", ".vercel",
]);

// ── Files to always skip (by exact name) ────────────────────────
const SKIP_FILES = new Set([
  ".gitkeep", ".gitignore", ".env", ".env.local", ".env.production",
  "tsconfig.tsbuildinfo", "Cargo.lock", "pnpm-lock.yaml",
  "package-lock.json", "MANIFEST.md", ".DS_Store", "thumbs.db",
]);

// ── Extensions to skip (binary assets, caches) ─────────────────
const SKIP_EXTENSIONS = new Set([
  ".png", ".jpg", ".jpeg", ".webp", ".gif", ".ico", ".svg",
  ".mp4", ".mp3", ".wav", ".ogg", ".woff", ".woff2", ".ttf", ".eot",
  ".zip", ".tar", ".gz", ".bz2", ".7z",
  ".stl", ".obj", ".3mf", ".dxf", ".step", ".stp",
  ".pyc", ".pyo", ".class", ".o", ".so", ".dylib",
  ".log",
]);

// ── Parse MANIFEST.md ───────────────────────────────────────────
function parseManifest(filepath) {
  if (!existsSync(filepath)) {
    return null;
  }
  const content = readFileSync(filepath, "utf-8");
  const paths = new Set();

  for (const line of content.split("\n")) {
    // Match table rows: | path | purpose | status | updated |
    const match = line.match(/^\|\s*`?([^|`]+?)`?\s*\|/);
    if (!match) continue;
    const path = match[1].trim();
    // Skip header row and separator
    if (path === "Path" || path.startsWith("-")) continue;
    paths.add(path);
  }
  return paths;
}

// ── Recursively scan directory for significant files ────────────
function scanDirectory(dir, baseDir) {
  const files = [];

  function walk(currentDir) {
    let entries;
    try {
      entries = readdirSync(currentDir, { withFileTypes: true });
    } catch {
      return;
    }

    for (const entry of entries) {
      if (SKIP_DIRS.has(entry.name)) continue;

      const fullPath = join(currentDir, entry.name);

      if (entry.isDirectory()) {
        walk(fullPath);
      } else if (entry.isFile() || entry.isSymbolicLink()) {
        // Skip by exact filename
        if (SKIP_FILES.has(entry.name)) continue;
        // Skip by extension
        if (SKIP_EXTENSIONS.has(extname(entry.name).toLowerCase())) continue;
        // Get relative path from division root
        const relPath = relative(baseDir, fullPath);
        files.push(relPath);
      }
    }
  }

  walk(dir);
  return files.sort();
}

// ── Main ────────────────────────────────────────────────────────
let hasErrors = false;
let totalFiles = 0;
let totalTracked = 0;

console.log("Manifest check\n");

for (const div of DIVISIONS) {
  if (!existsSync(div.dir)) {
    console.warn(`  SKIP: ${div.name}/ does not exist`);
    continue;
  }

  const manifestPaths = parseManifest(div.manifest);
  if (manifestPaths === null) {
    console.error(`  ERROR: ${div.name}/MANIFEST.md not found`);
    hasErrors = true;
    continue;
  }

  const diskFiles = scanDirectory(div.dir, div.dir);
  totalFiles += diskFiles.length;

  // Find untracked files (on disk but not in manifest)
  const untracked = diskFiles.filter((f) => !manifestPaths.has(f));

  // Find stale entries (in manifest but not on disk)
  const stale = [...manifestPaths].filter(
    (p) => !existsSync(join(div.dir, p))
  );

  const tracked = diskFiles.length - untracked.length;
  totalTracked += tracked;
  const coverage =
    diskFiles.length > 0
      ? ((tracked / diskFiles.length) * 100).toFixed(0)
      : 100;

  if (untracked.length > 0) {
    hasErrors = true;
    console.error(`  FAIL: ${div.name}/ — ${untracked.length} untracked file(s), coverage ${coverage}%`);
    for (const f of untracked) {
      console.error(`    + ${f}`);
    }
  } else {
    console.log(`  OK:   ${div.name}/ — ${diskFiles.length} files, coverage ${coverage}%`);
  }

  if (stale.length > 0) {
    console.warn(`  WARN: ${div.name}/ — ${stale.length} stale manifest entry/entries (file deleted):`);
    for (const f of stale) {
      console.warn(`    - ${f}`);
    }
  }

  console.log();
}

// ── Summary ─────────────────────────────────────────────────────
const overallCoverage =
  totalFiles > 0 ? ((totalTracked / totalFiles) * 100).toFixed(0) : 100;

console.log(`Total: ${totalTracked}/${totalFiles} files tracked (${overallCoverage}% coverage)`);

if (hasErrors) {
  console.error(
    "\nManifest check FAILED. Add missing files to the relevant MANIFEST.md.\n"
  );
  process.exit(1);
}

console.log("\nManifest check passed.\n");
