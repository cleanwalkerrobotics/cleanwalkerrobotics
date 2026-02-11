# CleanWalker Robotics — SRTIP SAIA Application

**Programme:** SAIA 5th Edition — Sustainability Cohort (Sharjah Accelerator for Innovation and Advancement)
**Organiser:** Sharjah Research Technology and Innovation Park (SRTIP)
**Application Window:** February 22 – May 9, 2026
**Applicant:** MB Software Studio LLC (CleanWalker Robotics)
**Contact:** Maurits Bos, Founder & CEO — walker@cleanwalkerrobotics.com
**Website:** cleanwalkerrobotics.com
**Date:** February 2026

---

## 1. Company Overview

**CleanWalker Robotics** (MB Software Studio LLC) is building the world's first commercial quadrupedal litter-collecting robot. We combine AI-powered litter detection, all-terrain four-legged locomotion, and an autonomous bag cassette waste collection system to clean parks, waterfronts, plazas, and pedestrian zones without human intervention.

| Parameter | Detail |
|-----------|--------|
| **Legal entity** | MB Software Studio LLC (United States) |
| **Founder** | Maurits Bos — Dutch national, based in Colombia, software engineering and full-stack robotics background |
| **Product** | Autonomous quadrupedal litter-collecting robot (CW-1) |
| **Business model** | Robot-as-a-Service (RaaS) — USD 3,000–3,500/month per unit, all-inclusive |
| **Stage** | Pre-production — ML pipeline operational, URDF model complete, RL training configured, live AI demo at cleanwalkerrobotics.com |
| **SAIA focus sector** | Waste Management & Circularity |
| **License** | AGPL-3.0 |

CleanWalker is a new category of cleaning infrastructure. We do not compete with street sweepers or wheeled vacuum machines — we address the gap they leave behind: individual litter items on complex terrain where no automated solution exists today.

---

## 2. Problem Statement

### The UAE Waste Management Challenge

The UAE generates approximately 6.5 million tonnes of waste annually, with street-level litter remaining one of the most visible and persistent urban challenges. In Sharjah and across the Emirates, this problem is amplified by three structural factors:

**Extreme heat exposure.** Manual litter pickers work in ambient temperatures exceeding 45°C during summer months. Despite midday work bans (June–September), outdoor cleaning workers face severe heat stress during early morning and evening shifts. This is a worker welfare crisis that automation directly addresses.

**Labour dependency.** The UAE's cleaning sector relies heavily on imported manual labour. BEEAH Tandeef alone operates city-scale street cleaning across Sharjah, employing thousands of workers for litter collection in parks, waterfronts, and residential areas. This model faces rising costs, labour availability constraints, and growing public expectation for higher service levels.

**Tourism and public space standards.** Sharjah's Al Majaz Waterfront, Al Noor Island, and University City are high-visibility public spaces where cleanliness directly impacts resident satisfaction and tourist experience. These areas require continuous cleaning — not once-daily sweeping runs — to maintain standards expected of a modern Gulf city.

### The Technology Gap

BEEAH Tandeef has already deployed autonomous electric vacuum machines for street sweeping, demonstrating an appetite for cleaning automation. However, these wheeled machines operate on paved roads and cannot navigate grass, sand, gravel, or uneven walkways. They sweep bulk debris — they do not detect and pick up individual litter items like cigarette butts, plastic bottles, food wrappers, or beverage cans scattered across parks and waterfronts.

No commercial product exists globally that autonomously identifies, navigates to, picks up, and bags individual litter items on mixed outdoor terrain. This is the problem CleanWalker was built to solve.

### Market Opportunity

- **Global outdoor cleaning services TAM:** USD 3–5 billion
- **GCC waste management market:** USD 7.5+ billion (growing 5–6% annually)
- **UAE municipal cleaning spend:** Hundreds of millions in annual contracts, with Sharjah's cleaning operations among the largest in the region via BEEAH Tandeef
- **Dubai Robotics and Automation Program (DRAP):** Targets 200,000 robots deployed by 2032, with robotics representing 9% of Dubai's GDP — creating an unprecedented demand environment for service robots

CleanWalker does not require this market to be created. The market exists — it is simply served entirely by manual labour today.

---

## 3. Solution

### How CleanWalker Works

The CW-1 robot operates autonomously through three integrated systems:

**1. Perception — See Every Piece of Litter**

Our AI perception pipeline runs three concurrent neural networks on a single NVIDIA Jetson Orin Nano Super (67 TOPS, 8 GB RAM):

| Component | Model | Performance | Purpose |
|-----------|-------|-------------|---------|
| Litter detection | YOLO26s (TensorRT INT8) | 5–10 ms, 95% accuracy | Detects 20+ litter categories: bottles, cans, cigarette butts, wrappers, food waste, bags |
| Terrain segmentation | SegFormer-B0 (TensorRT FP16) | 25 ms @ 5 Hz | Classifies grass, sand, pavement, water, obstacles for safe navigation |
| Grasp planning | GR-ConvNet v2 (TensorRT INT8) | 5–10 ms | Generates optimal grasp poses for each detected item |

Stereo depth from OAK-D Pro ($399) plus 360-degree LiDAR from Livox Mid-360 ($749) provide 3D environmental awareness. Visual odometry (NVIDIA cuVSLAM) and RTAB-Map deliver GPS-independent localisation. Total perception hardware cost: **USD 1,148 per unit**.

**Full pipeline latency: 50–75 ms** from camera frame to grasp command — faster than a human can blink. The perception stack runs at 52% GPU utilisation during navigation, leaving substantial headroom.

**2. Locomotion — Go Where Wheels Cannot**

Quadrupedal locomotion is CleanWalker's core differentiator. Four legs with 12 degrees of freedom (3 per leg) navigate:

- Sand (beaches, desert-adjacent areas)
- Grass (parks, waterfront lawns)
- Paved plazas and cobblestones
- Curbs, steps, and uneven walkways
- Gravel paths

This is terrain where wheeled sweepers and vacuum machines cannot operate. The robot weighs 15 kg, moves at walking speed, and produces under 55 dB — quieter than a conversation, suitable for tourist areas and night operation.

Locomotion is controlled by a reinforcement learning policy trained in NVIDIA IsaacLab with terrain curriculum (flat, rough, slopes) and domain randomisation for sim-to-real transfer.

**3. Collection — Integrate with Existing Infrastructure**

The robot arm (5 DOF, mechanical gripper with silicone-tipped fingers) picks up items from 5 g to 500 g and deposits them into a bag cassette system. Bags are standard-size, replaceable, and designed for pickup by existing waste collection routes — including BEEAH Tandeef's current infrastructure.

No new waste processing infrastructure is required. CleanWalker fills bags. Tandeef collects bags. The integration is that simple.

### Robot-as-a-Service (RaaS) Model

| Tier | Monthly Rate (USD) | Contract | Includes |
|------|-------------------|----------|----------|
| Pilot | 3,000/unit | 6 months | Hardware, software, fleet dashboard, maintenance, OTA updates |
| Standard | 3,500/unit | 12 months | Same + SLA guarantees |
| Fleet | 3,000/unit | 24+ months, 50+ units | Volume pricing, dedicated support |

Each unit operates 20+ hours/day via autonomous dock charging, replacing the equivalent of 1.5–2.5 FTEs of manual litter collection. At standard pricing, CleanWalker delivers 40–60% cost savings versus manual labour at scale.

---

## 4. Technology Readiness

### What Exists Today

CleanWalker is pre-production with the following components operational:

| Component | Status | Evidence |
|-----------|--------|----------|
| **AI litter detection** | Operational | Live WebGPU YOLO26 demo at cleanwalkerrobotics.com — real-time inference in browser, 20+ litter categories |
| **Perception pipeline architecture** | Fully designed | Three concurrent pipelines (detection-to-grasp, terrain segmentation, SLAM/localisation) with complete ROS 2 node graph, latency budgets, memory budgets, and GPU utilisation profiles |
| **URDF robot model** | Complete | 18 DOF (12 leg + 5 arm + 1 bag hinge), 27 links, 15 kg mass, validated geometry — ready for simulation and RL training |
| **RL locomotion training** | Configured | NVIDIA IsaacLab DirectRLEnv, PPO via rsl_rl, terrain curriculum, domain randomisation — ready to train |
| **Hardware BOM** | Researched and costed | Full bill of materials with supplier pricing at prototype (USD 10,224/unit) and production scale (USD 8,799/unit at 10 units, USD 5,503/unit at 1,000 units) |
| **Fleet management dashboard** | Demo operational | Interactive dashboard demo on website with real-time monitoring UI, map view, activity feed |
| **Business model** | Validated | Pilot financial model with full cost breakdown, sensitivity analysis, and unit economics at scale |

### Perception Stack Detail

The full perception architecture is documented in a pilot-partner-ready reference specification. Key technical metrics:

| Metric | Value |
|--------|-------|
| Detection-to-grasp latency | 50–75 ms end-to-end |
| GPU utilisation (navigation) | ~52% of Orin Nano Super |
| GPU utilisation (picking) | ~37% peak burst |
| Memory footprint | ~3.6 GB of 8 GB budget (57% headroom) |
| Camera + LiDAR cost | USD 1,148 |
| Compute cost | USD 249 (Jetson Orin Nano Super) |
| YOLO26s detection speed | 5–10 ms (TRT INT8) |
| Models running concurrently | 5 (YOLO26s, SegFormer-B0, ESS, GR-ConvNet v2, cuVSLAM) |

### What SAIA Would Accelerate

The gap between current state and pilot deployment is **physical prototyping** — building the robot, testing locomotion, validating grasping in real conditions, and weatherproofing for Gulf heat. This is precisely what SRTIP's SoiLAB prototyping facilities would enable.

| Gap | What's Needed | SRTIP/SAIA Value |
|-----|---------------|------------------|
| Physical prototype build | 3D printing, CNC machining, electronics assembly | SoiLAB: 3D printers, laser cutters, CNC machines, electronics workbenches |
| Gulf-condition testing | Heat testing (45°C+), sand/dust ingress, UV exposure | Sharjah's climate IS the test environment |
| BEEAH integration scoping | Technical discussions with Tandeef operations team | SRTIP proximity to BEEAH HQ + corporate matchmaking |
| Investor readiness | Prototype demo + performance data for fundraising | Seal the Deal investor matching (AED 4M+ committed in 2025 edition) |

---

## 5. Market Opportunity

### UAE Positioning

CleanWalker's UAE strategy centres on three strategic relationships:

**BEEAH Group (Sharjah)** — Our primary target customer. BEEAH operates the largest waste management infrastructure in Sharjah through Tandeef and has already deployed autonomous street cleaning machines. CleanWalker's quadrupedal robot complements their existing wheeled autonomous fleet by addressing terrain and litter types their current machines cannot handle. BEEAH's headquarters are in Sharjah — SRTIP provides the most direct pathway to this partnership.

**Emaar Properties (Dubai)** — Premium hospitality and retail properties (Dubai Mall, Downtown Dubai, Dubai Marina) where public space cleanliness directly impacts property values and visitor experience. Emaar's premium positioning justifies RaaS pricing and provides high-visibility deployment sites.

**Enova (Dubai/Abu Dhabi)** — Veolia-Majid Al Futtaim joint venture managing facilities across 350+ properties. Their integrated waste management contracts create a natural channel for CleanWalker deployment at scale.

### Competitive Landscape

**There is no commercial competitor in quadrupedal litter collection — globally.** The adjacent landscape includes:

| Category | Examples | Why CleanWalker is Different |
|----------|----------|------------------------------|
| Autonomous sweepers | Trombia, Enway, Idriverplus | Wheeled, paved roads only, sweep bulk debris — cannot pick individual items on grass/sand |
| Indoor cleaning robots | Avidbots, Brain Corp | Indoor-only, flat floors — no outdoor, no litter picking |
| Research quadrupeds | Unitree, Boston Dynamics Spot | General-purpose platforms — no litter detection, no grasping, no waste collection |
| Manual litter pickers | Thousands globally | The incumbent — expensive, heat-exposed, unscalable |

CleanWalker occupies a category of one. We are not a better sweeper. We are a new type of cleaning robot that addresses a problem no existing machine can solve.

### Global Scaling Path

The Sharjah pilot is the launchpad for a global scaling strategy:

1. **Sharjah → Dubai** — Expand from BEEAH/Tandeef to Emaar and Dubai Municipality (aligned with DRAP's 200,000-robot target)
2. **UAE → GCC** — BEEAH's expansion into Egypt and KSA creates a built-in regional scaling channel
3. **GCC → Global** — Municipality contracts in Amsterdam, Helsinki, Copenhagen, Sydney, and Melbourne are already in our pipeline with proposals prepared

A successful BEEAH partnership in Sharjah becomes the reference case for every conversation globally. No city will be the first — but every city wants to be the second.

---

## 6. Pilot Proposal for Sharjah

### Overview

| Parameter | Detail |
|-----------|--------|
| **Units** | 10 robots |
| **Duration** | 6 months |
| **Partner** | BEEAH Tandeef (waste collection integration) |
| **Monthly rate** | USD 3,000/unit (pilot pricing) |
| **Total contract value** | USD 180,000 |

### Proposed Deployment Zones

Three sites selected for terrain diversity, public visibility, and BEEAH operational overlap:

**1. Al Majaz Waterfront**
- Sharjah's premier public leisure destination — waterfront promenade, gardens, playgrounds
- Mixed terrain: paved walkways, grass areas, sand-adjacent zones
- High foot traffic, high litter density, high visibility
- 3–4 units deployed

**2. University City of Sharjah**
- Large campus environment with parks, walkways, and open spaces
- SRTIP is located within University City — maximum proximity for monitoring and iteration
- Academic partnership opportunity with University of Sharjah
- 3–4 units deployed

**3. Al Noor Island**
- Curated public art and nature destination — Sharjah's signature cultural attraction
- Sensitive environment requiring quiet, non-intrusive cleaning
- CW-1's <55 dB operation and precise individual-item pickup is ideal
- 2–3 units deployed

### Metrics Tracked

| KPI | Target | Measurement Method |
|-----|--------|-------------------|
| Litter items collected per unit per day | Tracked and reported weekly | On-board AI detection logging |
| Coverage area per shift | 2,000–4,000 m²/hr per unit | Odometry and GPS telemetry |
| System uptime | >90% across pilot period | Fleet management dashboard |
| Operation in 45°C+ conditions | Continuous summer operation | Temperature sensors + uptime logs |
| Detection accuracy (field-validated) | >90% across 20+ categories | Manual audit of detection logs vs. ground truth |
| Integration with Tandeef routes | Bag handoff to existing collection schedule | Coordination logs and bag count |
| Public safety incidents | Zero | Safety system logs + incident reports |
| Public engagement | Media coverage, social impressions | PR tracking |

### Integration with BEEAH Tandeef

CleanWalker's bag cassette system produces standard waste bags ready for collection. Integration with Tandeef's existing routes requires no new infrastructure:

1. **Bag handoff:** Full bags left at designated collection points on Tandeef's existing route schedule
2. **Data sharing:** Litter heatmap data from CleanWalker fleet shared with Tandeef operations for route optimisation
3. **Fleet coordination:** CleanWalker dashboard provides real-time robot positions and bag fill status — Tandeef dispatchers see when and where bags need collection
4. **Complementary coverage:** CleanWalker robots clean the parks and waterfronts; Tandeef's autonomous sweepers clean the roads. Together, they cover the full public space cleaning spectrum.

### Pilot Timeline

| Phase | Period | Activities |
|-------|--------|------------|
| **Site survey** | Month 1 | Deployment zone mapping, dock locations, electrical hookup, Tandeef coordination |
| **Installation** | Month 2 | Dock installation, robot deployment, system calibration, staff training |
| **Active operation** | Month 3–8 | Full autonomous operation, weekly KPI reporting, iterative optimisation |
| **Evaluation** | Month 8 | Performance analysis, ROI calculation, expansion planning |

---

## 7. Team

### Founder

**Maurits Bos** — Founder & CEO

Dutch national with a software engineering background spanning full-stack development, machine learning, and systems architecture. Leads all technical development (ML pipeline, perception architecture, hardware design, firmware) and commercial strategy. Based in Colombia with flexibility to relocate to Sharjah for the SAIA programme and pilot deployment.

### AI-Augmented Operations Model

CleanWalker operates with an AI-augmented team structure that allows a single founder to manage the equivalent output of a multi-person startup:

- **ML/perception development** — AI-assisted code generation, model training pipeline, automated testing
- **Hardware design** — CAD/URDF modelling, BOM research, supplier sourcing
- **Business development** — Market research, proposal generation, grant applications, outreach
- **Operations** — Fleet management software, monitoring dashboards, documentation

This is not a limitation — it is a deliberate advantage. Lower burn rate means longer runway, less dilution, and faster iteration. The SAIA programme's 200+ industry mentors would complement this model by providing domain expertise in UAE waste management, regulatory navigation, and investor relations that a solo technical founder cannot replicate.

### Advisory and Collaboration (Planned)

- Mechanical engineering consultant for physical prototype build
- BEEAH/Tandeef operations contact for integration scoping
- UAE legal advisor for entity establishment and regulatory compliance

SRTIP's corporate matchmaking and SBAN angel network would accelerate the build-out of this advisory layer.

---

## 8. Ask — What We Seek from SAIA

### Primary Requests

| Request | Why It Matters |
|---------|---------------|
| **SoiLAB prototyping access** | Our biggest gap is physical prototyping. SoiLAB's 3D printers, CNC machines, and electronics workbenches would enable us to build our first physical prototype during the SAIA programme — transforming digital designs into a walking, picking, demonstrable robot. |
| **Seal the Deal investor matching** | We need USD 250K–350K to fund a 10-unit pilot. The 2025 Seal the Deal saw AED 4M+ committed to 9 startups. Access to SBAN and regional investors through this programme is the fastest path to our seed round. |
| **BEEAH/Tandeef introduction** | SRTIP's proximity to BEEAH HQ and SAIA's corporate matchmaking programme can facilitate the introduction that unlocks our primary customer relationship. A letter of intent from BEEAH would transform our fundraising and commercial trajectory. |
| **Sharjah business license facilitation** | SRTIP entity establishment or facilitation of a Sharjah free zone license to enable local operations, pilot contracting, and long-term UAE presence. |

### Secondary Benefits

| Benefit | Value to CleanWalker |
|---------|---------------------|
| Discounted SPark workspace | Base of operations during prototype development and pilot preparation |
| 200+ industry mentors | UAE waste management domain expertise, regulatory guidance, investor introductions |
| Academic partnerships | University of Sharjah collaboration for testing, validation, and research |
| Demo Day (July 18, 2026) | Public showcase to investors, government stakeholders, and potential customers |

### What SAIA Gets

CleanWalker in SAIA's Sustainability Cohort delivers a high-visibility, globally unique robotics story:

1. **Media magnet** — A walking robot cleaning Sharjah's waterfront generates immediate international coverage. This is not another software startup.
2. **SRTIP sector validation** — A litter-collecting robot in the Waste Management & Circularity track validates SRTIP's thesis that Sharjah is where sustainability innovation happens.
3. **BEEAH synergy** — Connecting a SAIA startup directly with BEEAH's cleaning operations demonstrates the accelerator's ability to create real commercial outcomes, not just mentorship.
4. **DRAP alignment** — Positions SAIA as contributing to the UAE's 200,000-robot target, elevating the programme's national relevance beyond Sharjah.

---

## Appendix: Financial Overview

### Funding Requirements

| Phase | Cost (Scrappy Founder) | Timeline |
|-------|----------------------|----------|
| Pre-pilot development | USD 140,000 | Month 1–6 |
| Pilot production (10 units) | USD 118,000 | Month 6–8 |
| Deployment | USD 14,000 | Month 8–10 |
| Operations (6 months) | USD 15,000 | Month 10–16 |
| **Total to pilot** | **USD 288,000** | |

### Revenue Projection

| Metric | Value |
|--------|-------|
| 10-unit pilot revenue (6 months) | USD 180,000 |
| Annual RaaS revenue per unit (at scale) | USD 36,000–42,000 |
| Unit economics at 100 units | 50–63% gross margin |
| Unit economics at 1,000 units | 71–79% gross margin |
| Hardware payback period (at scale) | 3–5 months |

### Why This Is a Good Investment

The pilot is not designed to be profitable in isolation. It generates:

- A live customer reference and video case study
- Real-world performance data across 6 months of Gulf-condition operation
- Proof of product-market fit with the UAE's largest waste management operator
- Foundation for a Series A raise of USD 2–5M to fund production scale-up

---

*CleanWalker Robotics | MB Software Studio LLC*
*walker@cleanwalkerrobotics.com | cleanwalkerrobotics.com*
