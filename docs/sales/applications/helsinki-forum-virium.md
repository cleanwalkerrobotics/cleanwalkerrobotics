# CleanWalker Robotics — Application to Urban Mobile Robotics Open Call

**Programme:** Urban Mobile Robotics (UMR) — Forum Virium Helsinki
**Funding:** European Regional Development Fund (ERDF / EAKR)
**Project Period:** April 2025 – May 2027
**Applicant:** MB Software Studio LLC (CleanWalker Robotics)
**Contact:** Maurits Bos, Founder & CEO — walker@cleanwalkerrobotics.com
**Date:** February 2026

---

> **Note:** The first open call deadline (December 5, 2025) has passed. This application is prepared for the next round or direct engagement with the project team. Contact: Aleksis Pillai (aleksis.pillai@forumvirium.fi, +358 41 315 2519).

---

## 1. Company Overview

**CleanWalker Robotics** (MB Software Studio LLC) is building the world's first commercial quadrupedal litter-collecting robot for urban environments. Our autonomous robot combines AI-powered litter detection, all-terrain four-legged locomotion, and a bag cassette waste collection system to autonomously clean parks, waterfronts, plazas, and pedestrian zones.

**Key facts:**

- **Founded:** 2025, Austin TX (US entity) / Dutch founder (Maurits Bos)
- **Product:** Autonomous quadrupedal litter-collecting robot (V2.3 design)
- **Business model:** Robot-as-a-Service (RaaS) — EUR 2,500–3,200/mo per unit, all-inclusive
- **Stage:** Pre-production, prototype development phase — ideal TRL for the UMR programme's advancement goals
- **Website:** cleanwalkerrobotics.com

We are a small, fast-moving startup — exactly the type of SME the Urban Mobile Robotics programme is designed to accelerate.

---

## 2. Response to Urban Challenge: Street-Level Litter in Helsinki

### The Problem

Helsinki's parks, waterfront promenades, and public plazas accumulate litter that manual collection cannot address consistently. Stara's workforce manages city-scale maintenance, but street-level litter picking — particularly on mixed terrain like gravel paths, grass, and cobblestones — remains labour-intensive, difficult to schedule continuously, and expensive to scale. As Helsinki pursues Carbon-neutral 2035, electric and automated alternatives to manual operations become essential.

### Why This Problem Fits UMR

- **Ground-based mobile robotics** performing a real urban maintenance task
- **Environmental sustainability** — fully electric, zero-emission operation
- **Data-driven urban services** — AI detection generates litter density heatmaps, waste composition data, and cleanliness metrics for city operations
- **Strong city significance** — directly improves public space maintenance and resident quality of life

### Helsinki-Specific Opportunity

Forum Virium Helsinki already validated this category when Trombia's autonomous electric sweeper was piloted in Jätkäsaari (2021) — a collaboration that helped Trombia break into international markets. CleanWalker addresses the complementary problem: not sweeping streets, but **detecting and collecting individual litter items on complex terrain** where sweepers cannot operate (grass, gravel, park paths, waterfront walkways).

---

## 3. Technical Capabilities

### Perception Pipeline

Our unified perception architecture runs on NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8 GB RAM):

| Component | Model | Latency | Purpose |
|-----------|-------|---------|---------|
| **Object Detection** | YOLO26s (custom-trained, 25K+ litter images) | 5–10 ms | 20+ litter categories: bottles, cans, cigarette butts, wrappers, food waste |
| **Terrain Segmentation** | SegFormer-B0 (TensorRT FP16) | 25 ms | Classifies grass, gravel, pavement, water, obstacles for safe navigation |
| **Depth Estimation** | OAK-D Pro stereo + DA V2 Small | 15–25 ms | 3D litter localisation, grasp planning |
| **Grasp Planning** | GR-ConvNet v2 (TensorRT INT8) | 5–10 ms | Generates grasp poses for mechanical gripper |
| **Visual Odometry** | cuVSLAM (NVIDIA Isaac ROS) | ~9% GPU | Real-time localisation without GPS dependency |
| **Mapping** | RTAB-Map (stereo + LiDAR) | Async | Occupancy grid for Nav2 path planning |

**Full pipeline latency:** 50–75 ms (detection → depth → grasp → 3D projection)
**Memory footprint:** <3.6 GB of 8 GB budget

### Hardware Platform

| Spec | Value |
|------|-------|
| Locomotion | Quadrupedal (12 DOF) — navigates grass, gravel, cobblestones, curbs, stairs |
| Body | 600 × 150 × 120 mm, aluminium frame, IP65 weatherproof |
| Waste Collection | Bag cassette system — 15–20L capacity, replaceable bags for existing collection routes |
| Gripper | Mechanical arm with silicone-tipped fingers — picks items 5 g to 500 g |
| Operation | 20+ hours/day via autonomous dock charging, multi-shift capability |
| Sensors | OAK-D Pro (stereo + IR) + Livox Mid-360 LiDAR + IMU |
| Noise | <55 dB — quieter than conversation, suitable for residential areas and night operation |
| Compute | NVIDIA Jetson Orin Nano Super ($249) |

### Software Stack

- **ROS 2 Humble** — perception, navigation, behaviour trees
- **Nav2** — autonomous path planning and obstacle avoidance
- **Fleet management dashboard** — real-time monitoring, route planning, telemetry, remote override
- **OTA updates** — firmware and ML model updates deployed over-the-air

---

## 4. Proposed Pilot Scope

### Deployment Plan: Helsinki Urban Litter Collection Pilot

**Duration:** 6 months (within the UMR project timeline, ending before May 2027)

**Pilot zones (proposed, subject to city input):**

1. **Jätkäsaari / Hietaranta Beach area** — mixed terrain (sand, grass, paved paths), high pedestrian traffic, litter-intensive. Trombia was piloted here — a complementary deployment.
2. **Esplanadi / Kaivopuisto Park** — central Helsinki, high tourist and resident foot traffic, cobblestone paths and grass areas.
3. **Kalasatama** — new urban district, smart city testbed area, mixed surfaces.

**Deployment scale:** 2–4 units

### KPIs

| KPI | Target | Measurement |
|-----|--------|-------------|
| Litter items detected and collected per unit/day | Tracked and reported weekly | On-board AI logging |
| Area coverage per shift | 2,000–4,000 m²/hr per unit | GPS/odometry telemetry |
| System uptime | >90% across pilot period | Fleet dashboard monitoring |
| Public safety incidents | Zero | On-board safety logging + incident reports |
| Terrain types successfully navigated | Grass, gravel, cobblestone, paved, curbs | Terrain segmentation logs |
| Cleanliness score improvement | Pre/post measurement in pilot zones | City-defined metrics or photo surveys |
| Waste composition data | Full category breakdown | AI detection classification |

### Integration with City Operations

- Collected waste is bagged for pickup by existing Stara collection routes — **no new infrastructure required**
- Litter heatmap data shared with city operations for route optimisation
- Fleet dashboard access provided to Stara/FVH staff

---

## 5. Innovativeness

CleanWalker represents several firsts:

1. **First commercial quadrupedal litter-collecting robot** — no equivalent product exists globally
2. **AI-first approach** — trained on 25,000+ real-world litter images, not rule-based detection
3. **All-terrain capability** — four-legged locomotion handles surfaces where wheeled robots and sweepers fail
4. **Category-aware grasp planning** — YOLO detection class maps to optimal gripper strategy (rigid items → fingers, flat items → suction in Phase 2)
5. **Complementary to sweepers** — addresses individual litter items that Trombia-class machines cannot pick up

This is not an incremental improvement over existing methods. It is a new category of urban maintenance robot.

---

## 6. Commercial Potential

### Market Size

- **Global outdoor cleaning services:** USD 3–5 billion TAM
- **European municipal cleaning:** EUR 10+ billion annually
- **Nordic market alone:** Estimated EUR 500M+ in municipal street/park maintenance

### Business Model

Robot-as-a-Service (RaaS):
- **Pilot:** EUR 2,500/mo per unit (6-month trial)
- **Standard:** EUR 3,200/mo per unit (12-month contract)
- **Fleet:** EUR 2,800/mo per unit (24-month, 50+ units)

Each unit offsets 1.5–2.5 FTEs of manual litter picking, delivering 40–60% cost reduction at scale.

### International Scalability

A Helsinki pilot provides:
- **Credible European reference** for expansion to other Nordic and EU municipalities
- **Cold-climate validation** — proves year-round operation in challenging weather
- **Smart city showcase** — Helsinki's global smart city brand enhances CleanWalker's market positioning
- **Direct path to Copenhagen, Amsterdam, Vienna, Barcelona** — all active targets in our pipeline

We are already in contact with municipalities and waste operators in 10+ countries. Helsinki validation would accelerate commercial traction across all of them.

---

## 7. Budget Alignment

The UMR programme offers EUR 5,000–10,000 per pilot plus up to EUR 25,000 in expert services from Metropolia Robo Garage.

**Proposed budget:** EUR 10,000 (direct funding) + Metropolia expert support

| Item | Cost (EUR) | Funding Source |
|------|-----------|----------------|
| Robot deployment, operation, and monitoring (6 months) | Company-funded | CleanWalker |
| Site assessment, terrain mapping, route planning | Company-funded | CleanWalker |
| Pilot data collection, reporting, and city coordination | 5,000 | UMR funding |
| Pilot documentation, case study production, and public presentation | 5,000 | UMR funding |
| **Metropolia Robo Garage support** (co-development, testing, technology mapping) | Up to 25,000 (in-kind) | UMR / Metropolia |

We are prepared to invest significantly beyond the programme funding to make this pilot successful. The EUR 10,000 direct funding plus Metropolia expertise provides essential local support, testing infrastructure, and academic validation.

---

## 8. Team

| Name | Role | Background |
|------|------|------------|
| **Maurits Bos** | Founder & CEO | Dutch national. Software engineering background. Full-stack robotics (ML, firmware, hardware, business). Based in Austin, TX. |

**Advisors and collaborators** (in development):
- Robotics engineering (mechanical, electrical, embedded systems)
- Computer vision and ML research
- Operations and deployment

As a pre-production startup, we are lean by design. The UMR programme's expert support and Metropolia Robo Garage access would be particularly valuable for accelerating our development and testing in a real urban environment.

---

## 9. Timeline

| Phase | Period | Activities |
|-------|--------|------------|
| **Preparation** | Month 1–2 | Site assessment, terrain mapping, Helsinki regulatory review, Metropolia Robo Garage onboarding |
| **Deployment** | Month 3 | Robot delivery, dock installation, system configuration, staff training |
| **Active Pilot** | Month 3–8 | Autonomous operation across pilot zones, weekly data reporting, iterative optimisation |
| **Evaluation** | Month 8–9 | Data analysis, cleanliness score comparison, ROI calculation, case study production |
| **Presentation** | Month 9 | Results presentation to FVH, Stara, and city stakeholders |

This timeline fits comfortably within the UMR project period (ending May 2027).

---

## 10. Why Helsinki, Why Now

1. **Forum Virium's track record:** The Trombia autonomous sweeper went from Helsinki pilot to international markets. CleanWalker aims to follow the same path — but for litter collection, not sweeping.

2. **Regulatory leadership:** Helsinki and Finland have Europe's most permissive regulations for autonomous mobile robots in public spaces, removing a key barrier that blocks pilots elsewhere.

3. **Carbon-neutral Helsinki 2035:** Electric, autonomous cleaning directly supports the city's emissions reduction targets for municipal operations.

4. **Complementary to existing pilots:** CleanWalker does not compete with Trombia or delivery robots already tested in Helsinki. It fills the gap between sweeping (macro) and manual litter picking (micro).

5. **International reference value:** A successful Helsinki pilot is worth more than most city pilots globally — the Helsinki/Forum Virium brand carries weight with every municipality in our pipeline.

---

## 11. Contact & Next Steps

We would welcome a conversation about participating in the Urban Mobile Robotics programme — whether through a second open call round, direct project engagement, or an alternative collaboration format.

**Maurits Bos** — Founder & CEO
- Email: walker@cleanwalkerrobotics.com
- Web: cleanwalkerrobotics.com

**Programme contacts we are ready to engage:**
- Aleksis Pillai — PM, Urban Mobile Robotics, FVH (aleksis.pillai@forumvirium.fi)
- Antti Liljaniemi — PM, Robo Garage, Metropolia (antti.liljaniemi@metropolia.fi)
- Antti Rautiainen — Director, Environmental Management, Stara (antti.rautiainen@hel.fi)

---

cleanwalkerrobotics.com | walker@cleanwalkerrobotics.com
*CleanWalker Robotics — MB Software Studio LLC*
