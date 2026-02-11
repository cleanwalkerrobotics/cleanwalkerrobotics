# CleanWalker Robotics — Sheraa Accelerator Application

**Programme:** Sheraa Sharjah Entrepreneurship Center
**Applicant:** MB Software Studio LLC (CleanWalker Robotics)
**Contact:** Maurits Bos, Founder & CEO — walker@cleanwalkerrobotics.com
**Date:** February 2026

---

## 1. Company Overview

**CleanWalker Robotics** is building the world's first commercial quadrupedal litter-collecting robot for urban environments. Our autonomous robot uses AI-powered litter detection, four-legged all-terrain locomotion, and a bag cassette waste collection system to clean parks, waterfronts, plazas, and pedestrian zones without human intervention.

- **Founded:** 2025 — US entity (MB Software Studio LLC), Dutch founder
- **Stage:** Pre-production prototype development
- **Business model:** Robot-as-a-Service (RaaS) — monthly subscription per unit, all-inclusive (maintenance, software updates, fleet management)
- **Website:** cleanwalkerrobotics.com

We are a sustainability-focused deep tech startup operating at the intersection of robotics, AI, and waste management — three of Sheraa's core sectors.

---

## 2. Why Sharjah

Sharjah is the most strategically aligned location in the world for CleanWalker's first commercial deployment:

**BEEAH Group is headquartered in Sharjah.** BEEAH is the region's leading environmental management company, and their cleaning subsidiary Tandeef already operates autonomous electric sweeping machines on Sharjah's streets. CleanWalker's quadrupedal robot is the natural complement — picking up individual litter items on grass, gravel, and mixed terrain where wheeled sweepers cannot operate.

**Sheraa's partnership with BEEAH** through the DARE 2.0 MoU (signed February 4, 2026) creates a direct pathway. As a Sheraa portfolio company, we would benefit from Sheraa's established relationship with BEEAH leadership to secure pilot introductions and potential procurement discussions.

**Sharjah's waste management ecosystem** — including SRTIP, BEEAH, Tandeef, and government commitment to sustainability — makes it the ideal launchpad for autonomous cleaning technology. A successful Sharjah pilot becomes a reference case for the entire GCC region.

---

## 3. What We Need from Sheraa

1. **BEEAH / Tandeef introduction** — Our highest-value ask. A facilitated introduction to BEEAH's technology partnerships team or Tandeef's operations leadership to explore a pilot deployment. CleanWalker complements (not competes with) their existing autonomous sweepers.

2. **Sharjah business license** — Sheraa's free first-year license removes the entity barrier for UAE operations, allowing us to sign pilot agreements and establish local presence immediately.

3. **Prototyping and testing support** — Access to workspace, mentorship, and the Sharjah entrepreneurship ecosystem to support local prototype assembly and testing before a BEEAH/Tandeef pilot.

4. **Investor and partner introductions** — Sheraa's mentor network and the Sharjah Entrepreneurship Festival for visibility with regional investors and potential municipal customers beyond BEEAH.

---

## 4. Technology Overview

### Perception Stack

Our AI perception pipeline runs on NVIDIA Jetson Orin Nano Super (67 TOPS):

| Component | Purpose |
|-----------|---------|
| YOLO26s (custom-trained, 25K+ litter images) | Detects 20+ litter categories: bottles, cans, cigarette butts, wrappers, food waste |
| SegFormer-B0 terrain segmentation | Classifies grass, gravel, pavement, obstacles for safe navigation |
| OAK-D Pro stereo depth + DA V2 | 3D litter localisation and grasp planning |
| GR-ConvNet v2 grasp planning | Generates optimal grasp poses for the mechanical gripper |
| cuVSLAM + RTAB-Map | Real-time localisation and mapping without GPS dependency |

**Full pipeline latency:** 50–75 ms (detection to grasp command)

### Hardware

- **Quadrupedal locomotion** (12 DOF) — navigates grass, gravel, cobblestones, curbs, stairs
- **Bag cassette system** — 15–20L capacity, replaceable bags compatible with existing collection routes
- **Mechanical gripper** with silicone-tipped fingers — picks items 5 g to 500 g
- **20+ hours/day** operation via autonomous dock charging
- **<55 dB** noise — suitable for residential areas and night operation

### Business Model

Robot-as-a-Service: clients pay a monthly subscription per unit. We handle maintenance, software updates, fleet management, and OTA model improvements. Each unit offsets 1.5–2.5 FTEs of manual litter picking.

---

## 5. Market Opportunity

- **Global outdoor cleaning services:** USD 3–5 billion TAM
- **GCC waste management market:** Growing rapidly with government sustainability mandates
- **UAE Robotics and Automation Program (DRAP):** Dubai targets 200,000 robots deployed by 2032 — CleanWalker directly serves this national priority
- **No direct competitor** produces a commercial quadrupedal litter-collecting robot — we are creating a new product category

**Sharjah-specific opportunity:** BEEAH/Tandeef manage cleaning across Sharjah's public spaces. Their existing autonomous sweepers handle macro-cleaning (streets, roads). CleanWalker handles micro-cleaning (individual litter items on mixed terrain) — a complementary service gap worth millions in annual contract value.

---

## 6. Team

| Name | Role | Background |
|------|------|------------|
| **Maurits Bos** | Founder & CEO | Dutch national. Full-stack software engineering, robotics (ML, firmware, hardware, business). |

We are lean by design at pre-production stage. Sheraa mentorship and the Sharjah ecosystem would accelerate team-building for our UAE operations.

---

*CleanWalker Robotics — MB Software Studio LLC*
*cleanwalkerrobotics.com | walker@cleanwalkerrobotics.com*
