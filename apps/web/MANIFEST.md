# apps/web — Codebase Manifest

All pages, components, API routes, demos, configs, and source files owned by the web team.
Every file added or removed MUST be reflected here. Build will fail on untracked files.

## Configuration

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `package.json` | Package metadata, dependencies, scripts | active | 2026-02-12 |
| `tsconfig.json` | TypeScript compiler configuration | active | 2026-02-12 |
| `next.config.ts` | Next.js build configuration (webpack aliases) | active | 2026-02-12 |
| `postcss.config.mjs` | PostCSS config with Tailwind CSS v4 plugin | active | 2026-02-12 |
| `next-env.d.ts` | Next.js TypeScript ambient declarations | active | 2026-02-12 |

## Layout & Global

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/layout.tsx` | Root layout wrapper (metadata, fonts, body) | active | 2026-02-12 |
| `src/app/page.tsx` | Home page | active | 2026-02-12 |
| `src/app/globals.css` | Global Tailwind styles and CSS variables | active | 2026-02-12 |
| `src/app/sitemap.ts` | Dynamic sitemap generation for SEO | active | 2026-02-12 |

## Pages

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/about/page.tsx` | About / team page | active | 2026-02-12 |
| `src/app/contact/page.tsx` | Contact form page | active | 2026-02-12 |
| `src/app/contact/layout.tsx` | Contact section layout wrapper | active | 2026-02-12 |
| `src/app/product/page.tsx` | Product details page | active | 2026-02-12 |
| `src/app/product/HowItWorks.tsx` | How-it-works feature section component | active | 2026-02-12 |
| `src/app/pilot/page.tsx` | Pilot program sign-up page | active | 2026-02-12 |

## Insights / Blog

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/insights/page.tsx` | Insights listing page | active | 2026-02-12 |
| `src/app/insights/[slug]/page.tsx` | Dynamic insight article page | active | 2026-02-12 |
| `src/app/insights/articles.ts` | Article metadata and content definitions | active | 2026-02-12 |

## Shared Components

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/components/Navbar.tsx` | Global navigation bar | active | 2026-02-12 |
| `src/app/components/HeroSection.tsx` | Reusable hero section with CTA | active | 2026-02-12 |

## API Routes

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/api/contact/route.ts` | Contact form submission handler (Resend) | active | 2026-02-12 |
| `src/app/api/detect/route.ts` | Server-side litter detection ML endpoint | active | 2026-02-12 |
| `src/app/api/email/webhook/route.ts` | Inbound email webhook handler (Resend) | active | 2026-02-12 |

## Demos

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `src/app/demos/page.tsx` | Demo hub / gallery listing page | active | 2026-02-12 |
| `src/app/demos/registry.json` | Demo registry (id, name, purpose, status) | active | 2026-02-12 |
| `src/app/demos/3d-robot-viewer/page.tsx` | Interactive 3D CW-1 robot viewer (Three.js) | active | 2026-02-12 |
| `src/app/demos/3d-robot-viewer/types.ts` | Type definitions for 3D robot viewer | active | 2026-02-12 |
| `src/app/demos/3d-robot-viewer/build-robot-body.ts` | Three.js robot body geometry builder | active | 2026-02-12 |
| `src/app/demos/3d-robot-viewer/build-bag-system.ts` | Three.js bag cassette geometry builder | active | 2026-02-12 |
| `src/app/demos/autonomous-ops/page.tsx` | 24/7 autonomous operation time-lapse sim | active | 2026-02-12 |
| `src/app/demos/cost-calculator/page.tsx` | Interactive ROI / cost savings calculator | active | 2026-02-12 |
| `src/app/demos/fleet-dashboard/page.tsx` | Real-time fleet management dashboard | active | 2026-02-12 |
| `src/app/demos/litter-detection/page.tsx` | Client-side YOLO litter detection demo | active | 2026-02-12 |
| `src/app/demos/pick-and-compact/page.tsx` | Litter pick and bag compaction cycle sim | active | 2026-02-12 |
| `src/app/demos/quadrupedal-locomotion/page.tsx` | Terrain traversal physics simulation | active | 2026-02-12 |
| `src/app/demos/route-planning/page.tsx` | Multi-strategy route planning demo | active | 2026-02-12 |
| `src/app/demos/simulation/page.tsx` | Full autonomous patrol cycle simulation | active | 2026-02-12 |

## Public Assets (SEO)

| Path | Purpose | Status | Updated |
|------|---------|--------|---------|
| `public/robots.txt` | Search engine crawler directives | active | 2026-02-12 |
| `public/sitemap.xml` | Static XML sitemap | active | 2026-02-12 |
