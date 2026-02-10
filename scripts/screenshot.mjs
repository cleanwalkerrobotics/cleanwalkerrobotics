#!/usr/bin/env node
/**
 * Screenshot tool for QA — takes full-page screenshots of local or deployed pages.
 * 
 * Usage:
 *   node scripts/screenshot.mjs <url> [output-path]
 *   node scripts/screenshot.mjs http://localhost:3000 /tmp/home.png
 *   node scripts/screenshot.mjs https://cleanwalkerrobotics.com /tmp/live-home.png
 *   node scripts/screenshot.mjs http://localhost:3000/product  (saves to /tmp/screenshot-product.png)
 */
import { chromium } from 'playwright';
import { resolve } from 'path';

const url = process.argv[2];
if (!url) {
  console.error('Usage: node scripts/screenshot.mjs <url> [output-path]');
  process.exit(1);
}

const pageName = new URL(url).pathname.replace(/\//g, '-').replace(/^-/, '') || 'home';
const output = process.argv[3] || `/tmp/screenshot-${pageName}.png`;

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });

console.log(`Navigating to ${url}...`);
await page.goto(url, { waitUntil: 'networkidle', timeout: 30000 });

// Wait for images to load
await page.waitForTimeout(2000);

await page.screenshot({ path: resolve(output), fullPage: true });
console.log(`Screenshot saved: ${output}`);

await browser.close();
