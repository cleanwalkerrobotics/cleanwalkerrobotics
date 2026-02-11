# Team Log — Chronological

## 2026-02-10

### Unitree Go2 Platform Analysis (cw-research) — COMPLETED
- **Task:** Strategic research on Unitree Go2 as base platform for CleanWalker (pivot from from-scratch build)
- **Scope:** All Go2 variants specs/pricing, open source ecosystem, mounting/payload, compute/perception, BOM comparison, legal/competitive analysis
- **Key Finding:** Go2 EDU ($14,500) viable for rapid prototyping (4-6 weeks to demo) but ~2x cost of from-scratch at production scale, no weatherproofing, tight payload
- **Recommendation:** Use Go2 EDU for prototype/pilot validation, transition to custom platform for production
- **File:** docs/research/unitree-go2-platform-analysis.md

| 2026-02-10 | cw-research | Unitree Go2 platform analysis — specs, ecosystem, BOM, recommendation | pending |

### 3D Viewer: Bag Always Open + Play/Pause Button (cw-software) — COMPLETED

| 2025-02-10 17:00 | cw-software | Bag defaults to open (135°), removed continuous fold loop, added play button for on-demand bag swap cycle | 8da2803 |

### 3D Bag Dispensary System + Body Shell (cw-hardware) — COMPLETED
- **Task:** Detailed mechanical design of bag dispensary system and robot body shell as Three.js 3D models
- **Team A (Bag System):** Roll dispenser, folding frame, clip mechanisms, catenary bag mesh, drawstrings, animated fold cycle
- **Team B (Body):** Chamfered body shell, panel seams, hinge mount brackets, roll cradle, tapered head, turret flange
- **Coordination:** Mounting interface verified — hinge at (-0.30, ±0.05, 0.065), roll at X=-0.06, frame 150×220mm
- **Files:** build-bag-system.ts, build-robot-body.ts, types.ts, page.tsx
- **Commit:** 972e0fa
- **Status:** Complete, pushed to main

### Website Image Optimization (cw-software) — SPAWNED 14:50 UTC
- **Assigned:** WebP conversion of 14 renders, next/image optimization, OG meta images, SEO meta tags, Core Web Vitals fixes
- **Status:** In progress
- **Purpose:** Deal-advancing — faster page loads for prospects visiting our site

### Municipality Deployment Proposals (cw-bizdev) — SPAWNED 14:50 UTC
- **Assigned:** 3 one-pager deployment proposals (Amsterdam, Dubai/Sharjah, Veolia ANZ)
- **Status:** In progress
- **Purpose:** Deal-advancing — tangible pilot proposals to attach to outreach emails

### Outreach Escalation (Walker) — 14:46 UTC
- **Action:** Sent outreach readiness message to main session for relay to Maurits
- **Content:** 3 draft emails ready, Eurostars deadline (Mar 19), LLC→BV question
- **Note:** This was overdue by 4 hours (flagged in Assessments #8, #9, #10)

### Grant Research (cw-research)
- **Assigned:** Research and document top grant opportunities across EU, US, NL, Singapore, UAE
- **Result:** Complete. Created docs/ceo/grant-opportunities.md with 17+ grants analyzed across 5 regions, priority ranking, fit scores, deadlines, and action items
- **Key findings:** WBSO 50% starter rate (not 32%), Eurostars URGENT deadline Mar 19 2026, VFF up to €450K, MBRIF/DFA for UAE access, SBIR/DOT SMART both dead. LLC→BV conversion strongly recommended.
- **Commits:** 9051e92, 41e0a9e, d50f0a8
- **Duration:** ~30 min (plus NL enrichment pass)
- **Session killed:** 12:46 UTC (idle ~90 min post-completion)

## 2026-02-09

### ML Pipeline (cw-software → session `fresh-sage`)
- **Assigned:** YOLO training pipeline, TrashDetector class, demo scripts
- **Result:** ✅ Complete. Committed c540330.
- **Duration:** ~10 min

### Render Prompts (cw-software → session `fresh-sage`)
- **Assigned:** AI render prompt guide for marketing visuals
- **Result:** ✅ Complete. 14 prompts created. Committed 15d077d.

### Financial Model + Pricing (cw-bizdev → session `warm-shore`)
- **Assigned:** Revised financial model at $3,500/mo, pricing sheet, pitch deck
- **Result:** ✅ Complete. Committed c84b51b, ceb4630, 9c5f3c3.

### Pitch Deck (cw-bizdev → session `warm-shore`)
- **Assigned:** Investor/customer pitch deck
- **Result:** ✅ Complete. Committed 9c5f3c3.

### Demos & Contact Page (cw-software → session unknown)
- **Time:** ~23:46–23:58 UTC
- **Tasks completed:**
  - Replaced pricing page with demos showcase (cc9fbac)
  - Interactive AI litter detection demo with mock YOLO inference, 6 sample scenes, drag-and-drop upload (017636a)
  - Interactive fleet management dashboard demo with live clock, animated map, 12 robot dots, activity feed (724d021)
  - Contact Sales page with form, FAQ, and consultation flow (190c94e)
  - CLAUDE.md updates: team review process (51adb3a), VERCEL_TOKEN note (776568c)
- **Result:** ✅ All complete and deployed to Vercel production
- **Note:** This batch was NOT tracked in team-log or team-status at time of completion. Caught in Assessment #3.

## 2026-02-10

| Date | Team | Task | Commit |
|------|------|------|--------|
| 2026-02-10 13:15 | cw-software | Bag Cassette System design refactor: rewrote all 14 render prompts + regenerated all renders with Seedream 4.5 | c7b04b0 |
| 2026-02-10 | cw-software | Bag Cassette System copy refactor: updated all website pages (product, demos, pick-and-compact, autonomous-ops) to replace waste bin references | 352e1c9 |
| 2026-02-10 | cw-research | Comprehensive competitive landscape analysis: direct competitors, adjacent competitors, competitive matrix, moats, market gaps, strategic implications | 7184699 |
| 2026-02-10 | cw-software | Soften landing page + product page copy: remove all pricing/savings/$, replace RaaS with vision-focused language, benefit metrics instead of dollar stats | f5058c5 |

| 2026-02-10 | cw-renders | Regenerated all 14 robot renders with Seedream 4.5 using canonical design spec — all prompts start with exact core description, all images 1.2-2.0MB at 4K 16:9 | 20cb846 |
| 2026-02-10 | cw-bizdev | Pitch deck overhaul: bag cassette system refs, enriched competitive landscape (scoring matrix, 4 moats), removed all pricing/RaaS, rewritten Ask slide for pilot partner conviction | 5c4dee5 |

| 2026-02-10 06:46 | cw-ops (Walker) | Strategy.md IFAT cleanup: marked IFAT as SCRATCHED in priority stack, decisions table, trade shows, closing requirements | 9e2fc30 |
| 2026-02-10 06:46 | cw-hardware | URDF robot model v1 — 12-DOF quadruped from design spec | c134b39 |

| 2026-02-10 | cw-hardware | Website build verified (14/14 pages clean), all 14 renders confirmed referenced. URDF validated: fixed 4 cosmetic links missing inertials, created validate.py | 735d802 |
| 2026-02-10 08:50 | cw-software | Three.js 3D robot viewer page — interactive URDF visualization on website demos | e0525d3 |
| 2026-02-10 08:50 | cw-bizdev | Outreach email drafts — Veolia ANZ, BEEAH Group, Amsterdam municipality | 8db4ddd |

| 2026-02-10 09:10 | cw-bizdev | Outreach tracker (13 targets) + contacts database — 3 draft-ready + 10 researched expansion targets with budget estimates | a3de77a |
| 2026-02-10 | cw-research | Comprehensive autonomy stack architecture — 10-section R&D document covering perception, SLAM, nav, grasping, locomotion, behavior trees, integration, compute budget, 25+ repo links, phased roadmap | fc50dcd |

| 2026-02-10 | cw-software | Perception demo options research + Replicate API PoC — 4 options analyzed, working litter detection via YOLOv8s-WorldV2 | 2fd4fe8 |
| 2026-02-10 | cw-bizdev | Additional grant research: Innovate UK, NWO, DARPA, EIT Climate-KIC, Digital Europe, Innosuisse, IRAP, NSF/EPA/DOE, XPRIZE — all blocked by eligibility. Confirms NL/UAE strategy correct. | 27b3d64 |
| 2026-02-10 | cw-bizdev | Municipality deployment proposals: Amsterdam (Schoon & Afvalvrij), Dubai/BEEAH (AI City Vision), Veolia ANZ (Bondi-to-commercial) — 3 one-pagers in docs/sales/proposals/ | ecdcd3d |

| 2026-02-10 | cw-software | Integrate real AI litter detection into demo — Replicate YOLOv8 API route + rewritten UploadSection with real inference, bounding boxes, mock fallback, URL/sample input | d1641d3 |
| 2026-02-10 | cw-software | Website image optimization: converted 14 PNG renders to WebP (<200KB each), updated all refs, added OG/Twitter meta images, added SEO keywords + per-page meta, contact page metadata | 17346ea |

---
*cw-software complete. cw-bizdev complete. cw-hardware killed (URDF done, Gazebo N/A).*
| 2026-02-10 15:15 | render-team | Regenerated 11 renders without competitor branding, updated prompts, visual QA passed all | 2584a4d |
| 2026-02-10 | render-team | V2.2 full render regeneration: updated all 14 prompts to V2.2 spec (folding frame, bag roll, sensor head, upper-leg LEDs), regenerated all 14 via Seedream 4.5, all passed visual QA first try, converted to WebP, build verified | a5f3f4d |
| 2026-02-10 | cw-software | 3D viewer V2.2 update: split into 3 toggleable views (Full Robot, Robot Body, Bag System), updated body to V2.2 proportions (450×300×150), added multi-joint arm with gripper, bag system (roll dispenser + folding frame at 135° + hanging bag), LED strips on upper legs, ghost body for bag view, dark grey joints | f19681b |
| 2026-02-10 17:55 | cw-renders | Full QA render regeneration: all 14 renders via Seedream 4.5 with 3-role QA (Generator/Inspector/Art Director). 9 accepted R1, 3 accepted R2, 2 accepted R3. Rejects: hero-fleet (basket+white arms), detail-gripper (LED rings+basket+bin), lifestyle-city-worker (white arm), tech-exploded-view (flat-lay+basket), tech-dashboard-mockup (no dashboard+white arm). All 14 final renders pass V2.2 spec. WebP converted, build verified, page screenshots QA'd. | 04b2825 |
| 2026-02-10 18:46 | cw-software | 3D viewer lighting fix (light studio bg) + gripper text corrections (silicone-tipped fingers). COMMS FAILURE — session idle at prompt, no callback. Killed by ops check. | f96c01d |
| 2026-02-10 | cw-software | Full website QA: screenshotted all 12 pages, visually verified all renders (no branding/logos/artifacts), confirmed 3D viewer + contact form + all demos working, added 2 unreferenced renders (component-actuator, component-pcb) to product gallery, build clean, deployed to Vercel prod | 3715631 |
| 2026-02-10 | cw-renders | MERCILESS QA render attempt: 44+ generations across Seedream 4.5 (3 rounds × 14) + FLUX 1.1 Pro (2 renders). 0/14 passed. ALL failed on criterion #1 (bag roll axis parallel instead of perpendicular). Tried 3 prompt strategies: original V2.2, "rolling pin PERPENDICULAR" reinforcement, "crosswise widthwise towel rack" analogy. Neither model can orient a cylinder perpendicular to a quadruped body. FLUX worse (vertical cylinders, wrong robot design). No renders pushed — existing ones preserved. | no commit |
| 2026-02-10 19:42 | cw-bizdev | Research Tier 1 contacts: Veolia ANZ, BEEAH, Amsterdam | 0293c45 |
| 2026-02-10 | cw-software | 3D viewer V2.3 update: body proportions to 600×150×120mm (4:1 L:W ratio), head flush-integrated with body front (not separate block), arm turret base (8cm dia cylinder), bag system repositioned for new body, background to #f0f0f0, sidebar specs updated. Fixed "soft silicone" gripper text in pitch-deck.md + hardware-bom-research.md → "mechanical gripper with silicone-tipped fingers". Build clean, all pages screenshotted and verified. | 1d5f0d8 |
| 2026-02-10 | cw-ops | Design spec V2.3: fixed body width 30→15cm, frame width 30→15cm, track width 35→20cm, render prompt 45×30×15→60×15×12, bag roll width 28→13cm. Added V2.3 to evolution log. | 0dca758 |
| 2026-02-10 | cw-software | Inbound email webhook: Resend email.received → Vercel serverless → OpenClaw gateway forwarding. Svix HMAC verification, replay protection, setup docs. BLOCKER: gateway needs public URL (Cloudflare Tunnel or Traefik route). | acd66ad |
| 2026-02-10 | cw-bizdev | Tier 2 contact research: 10 targets, 20+ contacts with emails/LinkedIn/phones, full dossiers with news hooks, confidence ratings, outreach strategies. Updated contacts.md + outreach-tracker.md. | db92bcc |
| 2026-02-10 20:46 | cw-ops (comms-optimizer) | Fixed: team-status.json was stale (showed cw-software active, no sessions running). Updated to accurate state. Added Traefik route for gateway /hooks → port 18789 (unblocks email webhook). Gateway now reachable at https://cleanwalker.maurits-bos.me/hooks/gmail (verified 401). Scheduled Veolia outreach email for 22:00 UTC (9 AM AEDT). Vercel env OPENCLAW_GATEWAY_URL needs to be set to https://cleanwalker.maurits-bos.me — requires Maurits to update in Vercel dashboard. | — |
| 2026-02-10 | cw-software | Tier 2 outreach emails: Copenhagen (Solutions Lab), Helsinki (Forum Virium/Trombia), Barcelona (Cuidem Barcelona), Vienna (ICRA 2026/MA48), Emaar Dubai (premium property). 5 emails + timing + notes. | 4d0de49 |
| 2026-02-10 | cw-software | Tier 2 outreach emails 9-13: MAF/Enova (Veolia JV), City of Sydney (AUD 2.7B open space), City of Melbourne (Emerging Tech Testbed), UT Austin (NSF robotics grant), Walt Disney Parks (zero waste 2030). 5 emails + timing + notes. | d6f77e6 |
| 2026-02-10 21:15 | cw-research | State-of-the-art edge detection research: comprehensive analysis of YOLO family (v8/v9/v10/v11/v12/26), RT-DETR/v2, RF-DETR, YOLO-World for Jetson Orin Nano Super. Created docs/research/edge-detection-sota-2026.md with model comparison tables, Jetson performance benchmarks, license analysis, litter detection datasets, deployment recommendations (YOLO26n/s primary, RT-DETRv2-S Apache 2.0 alternative). | f59b759 |
| 2026-02-10 | cw-research | Semantic segmentation SOTA research: 9 models analyzed (SegFormer B0-B5, PIDNet, DDRNet, BiSeNet V2, PP-LiteSeg, OneFormer, Mask2Former, FastSAM, EfficientSAM) for outdoor terrain classification on Jetson Orin Nano Super. 7 terrain datasets reviewed (RUGD, RELLIS-3D, GOOSE, WildScenes, TAS500, YCOR, GANav). Isaac ROS integration mapped. Recommendations: SegFormer-B0 primary, PIDNet-S fallback. Created docs/research/semantic-segmentation-sota-2026.md. | c00d253 |
| 2026-02-10 | cw-research | Visual SLAM & VO SOTA research: 7 systems analyzed (ORB-SLAM3, RTAB-Map, Stella-VSLAM, cuVSLAM, VINS-Fusion, Kimera, DPVO/DROID-SLAM) for outdoor litter-picking robot on Jetson Orin Nano Super. Benchmark accuracy tables, resource usage profiles, license analysis, IMU hardware comparison, outdoor challenges & mitigations, occupancy grid generation options. Recommendation: cuVSLAM (visual odometry, 9% GPU) + RTAB-Map (mapping/occupancy grid) hybrid stack. Created docs/research/visual-slam-vo-research-2026.md. | dca4377 |
| 2026-02-10 23:14 | cw-renders | V3 renders via nano-banana: 14/14 succeeded. Used reference image as image_input for design consistency. All files 962KB–2.1MB (~20MB total). Visual QA passed: no branding/logos/watermarks, consistent robot design across all renders. | 53a19c7 |
| 2026-02-10 | cw-software | Swap all website image refs to v3 renders: 20 paths updated across 4 pages (page.tsx, about, product, demos) from /renders/*.webp to /renders/v3/*.png. Zero broken refs. | pending |
| 2026-02-10 23:54 | cw-software | Go2 URDF + reference render: Downloaded Unitree Go2 URDF/xacro/MuJoCo XML from unitree_ros + unitree_mujoco repos, generated Go2 reference render via nano-banana (Replicate). Mesh files (24.7MB DAE, 27.1MB OBJ) not committed — README with download instructions provided. | pending |

## 2026-02-11

| Date | Team | Task | Commit |
|------|------|------|--------|
| 2026-02-11 | cw-software | WebGPU YOLO26 litter detection demo: replaced server-side Replicate API with client-side YOLO26 nano via Transformers.js + WebGPU. Real-time webcam detection with FPS counter, image upload with bounding boxes, confidence threshold slider, WASM fallback, production model note. | ad45fa5 |
2026-02-11 00:33 | research-team | Update depth camera hardware research with CES 2026 releases (Orbbec Gemini 345Lg, 305) | 5ecc218
| 2026-02-11 00:38 | cw-research | Consolidated perception stack research: depth estimation (DA V2/V3, Isaac ROS ESS), grasp planning (GR-ConvNet v2, VGN), camera hardware (OAK-D Pro, Gemini 345Lg, ZED) | c82b81c |
| 2026-02-11 | cw-research | Unified perception pipeline architecture: consolidated 6 research docs into single pilot-partner-ready reference. ROS2 node graph, latency/memory/GPU budgets, model specs table, deployment strategy, phase plan. Updated autonomy-stack-architecture.md v1.0→v1.1. | 8b73152 |
| 2026-02-11 | cw-bizdev | Sales one-pagers: 3 municipality-specific one-pagers (Veolia ANZ, BEEAH, Amsterdam) in docs/sales/one-pagers/ — tailored challenge/solution/pilot/economics/CTA, PDF-ready | pending |
| 2026-02-11 | cw-software | Contact form wired to Resend API (walker@ + sales@ emails), WebP conversion of 15 v3 renders (87-97% smaller), all image refs updated | pending |
