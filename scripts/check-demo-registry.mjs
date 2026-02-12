#!/usr/bin/env node
// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

/**
 * Validates that every demo directory is registered in registry.json
 * and warns about demos with overlapping purposes.
 *
 * Exit code 1 = unregistered demos found (build fails).
 * Overlap warnings are non-fatal.
 */

import { readFileSync, readdirSync, statSync } from "node:fs";
import { join, resolve } from "node:path";

const ROOT = resolve(import.meta.dirname, "..");
const DEMOS_DIR = join(ROOT, "apps/web/src/app/demos");
const REGISTRY_PATH = join(DEMOS_DIR, "registry.json");

// ── Load registry ────────────────────────────────────────────────
let registry;
try {
  registry = JSON.parse(readFileSync(REGISTRY_PATH, "utf-8"));
} catch (err) {
  console.error("ERROR: Cannot read registry.json at", REGISTRY_PATH);
  console.error(err.message);
  process.exit(1);
}

const registeredIds = new Set(registry.demos.map((d) => d.id));

// ── Discover demo directories ────────────────────────────────────
const demoDirs = readdirSync(DEMOS_DIR).filter((entry) => {
  const full = join(DEMOS_DIR, entry);
  return statSync(full).isDirectory();
});

// ── Check for unregistered demos ─────────────────────────────────
const unregistered = demoDirs.filter((dir) => !registeredIds.has(dir));

if (unregistered.length > 0) {
  console.error("\nERROR: Unregistered demo directories found:");
  for (const dir of unregistered) {
    console.error(`  - ${dir}`);
  }
  console.error(
    "\nEvery demo directory must have an entry in apps/web/src/app/demos/registry.json."
  );
  console.error(
    "Add an entry with id, name, purpose, status, and added date, then re-run.\n"
  );
  process.exit(1);
}

// ── Check for stale registry entries ─────────────────────────────
const dirSet = new Set(demoDirs);
const stale = registry.demos.filter(
  (d) => d.status === "active" && !dirSet.has(d.id)
);
if (stale.length > 0) {
  console.warn("\nWARNING: Registry entries with no matching directory:");
  for (const d of stale) {
    console.warn(`  - ${d.id} (${d.name})`);
  }
  console.warn('  Consider setting their status to "deprecated".\n');
}

// ── Check for overlapping purposes (keyword similarity) ──────────
function extractKeywords(text) {
  const stopWords = new Set([
    "a", "an", "the", "of", "and", "in", "for", "with", "on", "to",
    "from", "by", "at", "is", "its", "that", "this", "or",
  ]);
  return new Set(
    text
      .toLowerCase()
      .replace(/[^a-z0-9\s]/g, "")
      .split(/\s+/)
      .filter((w) => w.length > 2 && !stopWords.has(w))
  );
}

function jaccardSimilarity(a, b) {
  const intersection = new Set([...a].filter((x) => b.has(x)));
  const union = new Set([...a, ...b]);
  return union.size === 0 ? 0 : intersection.size / union.size;
}

const OVERLAP_THRESHOLD = 0.5;
const activeDemos = registry.demos.filter((d) => d.status === "active");
const overlaps = [];

for (let i = 0; i < activeDemos.length; i++) {
  for (let j = i + 1; j < activeDemos.length; j++) {
    const a = activeDemos[i];
    const b = activeDemos[j];
    const sim = jaccardSimilarity(
      extractKeywords(a.purpose),
      extractKeywords(b.purpose)
    );
    if (sim >= OVERLAP_THRESHOLD) {
      overlaps.push({ a: a.id, b: b.id, similarity: sim });
    }
  }
}

if (overlaps.length > 0) {
  console.warn("\nWARNING: Demos with potentially overlapping purposes:");
  for (const { a, b, similarity } of overlaps) {
    console.warn(
      `  - "${a}" <-> "${b}" (${(similarity * 100).toFixed(0)}% keyword overlap)`
    );
  }
  console.warn(
    "  Review these demos to ensure they serve distinct purposes.\n"
  );
}

// ── Success ──────────────────────────────────────────────────────
console.log(
  `Demo registry OK: ${demoDirs.length} directories, ${registeredIds.size} registered entries.`
);
