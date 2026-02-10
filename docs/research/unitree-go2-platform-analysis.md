# Unitree Go2 as Base Platform for CleanWalker — Strategic Analysis

**Date:** 2026-02-10
**Author:** CW Research Team
**Revision:** 1.0
**Status:** Research Complete — Awaiting CEO Decision

---

## Executive Summary

Using a Unitree Go2 as the base quadruped platform instead of building from scratch could reduce prototype hardware cost from **~$10,200** (from-scratch BOM) to **~$8,200–$11,200** depending on variant, while eliminating 6–12 months of locomotion engineering. However, there are critical trade-offs: **no IP rating** (not waterproof), **8–12 kg payload limit**, **SDK lock-in to EDU tier ($14,500+)**, and potential **supply chain dependency** on a single Chinese manufacturer.

**Recommendation:** The Go2 EDU is viable as a **rapid prototyping and pilot platform** to validate our arm, bag dispensary, and perception stack. It should NOT be the long-term production platform due to cost-at-scale issues, lack of weatherproofing, and limited payload headroom. Use it to de-risk the product, then transition to custom hardware for production.

---

## Table of Contents

1. [Go2 Variants Comparison](#1-go2-variants-comparison)
2. [Open Source Ecosystem](#2-open-source-ecosystem)
3. [Custom Hardware Mounting](#3-custom-hardware-mounting)
4. [Compute & Perception](#4-compute--perception)
5. [Updated BOM & Cost Model](#5-updated-bom--cost-model)
6. [Competitive & Legal Considerations](#6-competitive--legal-considerations)
7. [Risk Assessment](#7-risk-assessment)
8. [Recommendation](#8-recommendation)

---

## 1. Go2 Variants Comparison

### 1.1 Full Variant Matrix

| Spec | Go2 Air | Go2 Pro | Go2 X | Go2 EDU | Go2 EDU Plus | Go2-W |
|------|---------|---------|-------|---------|--------------|-------|
| **Price (USD)** | ~$1,600 | ~$2,800 | ~$4,500 | ~$14,500 | ~$22,500 | ~$14,000 |
| **Weight** | ~15 kg | ~15 kg | ~15 kg | ~15 kg | ~15 kg | ~18 kg |
| **Dimensions (standing)** | 70×31×40 cm | 70×31×40 cm | 70×31×40 cm | 70×31×40 cm | 70×31×40 cm | 70×43×50 cm |
| **Max Speed** | 2.5 m/s | 3.5 m/s | 3.7 m/s (5 peak) | 3.7 m/s (5 peak) | 3.7 m/s (5 peak) | 2.5 m/s |
| **Payload (recommended)** | ~7 kg | ~8 kg | ~8 kg | ~8 kg | ~8 kg | ~8 kg |
| **Payload (max)** | ~10 kg | ~10 kg | ~12 kg | ~12 kg | ~12 kg | ~12 kg |
| **Climb Angle** | 30° | 40° | 40° | 40° | 40° | 40° |
| **Step Height** | ~15 cm | ~16 cm | ~16 cm | ~16 cm | ~16 cm | ~16 cm |
| **Battery** | 8,000 mAh | 8,000 mAh | 8,000 mAh | 15,000 mAh | 15,000 mAh | 15,000 mAh |
| **Runtime** | 1–2 hrs | 1–2 hrs | 1–2 hrs | 2–4 hrs | 2–4 hrs | 1.5–3 hrs |
| **SDK Access** | None | Limited | App scripting | Full (Python/C++) | Full (Python/C++) | Full |
| **ROS2 Support** | No | No | No | Yes | Yes | Yes |
| **Onboard Compute** | Basic | 8-core CPU | 8-core CPU | Jetson Orin Nano (40 TOPS) | Jetson Orin NX (100 TOPS) | Jetson Orin NX |
| **LiDAR** | 4D LiDAR L2 | 4D LiDAR L2 | 4D LiDAR L2 | 4D LiDAR L2 | 4D LiDAR L2 + Mid-360 | 4D LiDAR L2 |
| **Foot Force Sensors** | No | No | No | Yes | Yes | Yes |
| **IP Rating** | **None** | **None** | **None** | **None** | **None** | **None** |
| **Programmable** | No | No | Limited | Full | Full | Full |

Sources: [Unitree Official](https://www.unitree.com/go2/), [Robozaps Review](https://blog.robozaps.com/b/unitree-go2-review), [RobotShop Comparison](https://community.robotshop.com/blog/show/comprehensive-unitree-go2-robot-comparison), [Roboworks EDU](https://www.roboworks.net/store/p/unitree-go2-wc48f)

### 1.2 Key Takeaways for CleanWalker

- **Only EDU and EDU Plus give us the SDK/ROS2 access we need** for custom autonomy, perception, and arm control
- **No IP rating on any variant** — this is a major concern for outdoor litter collection in rain/dust
- **8 kg recommended payload** must accommodate: arm (~2.4 kg) + bag system (~1–2 kg) + compute (~0.5 kg) + sensors (~0.5 kg) + litter = **~5–6 kg minimum before any litter collected**
- **Payload headroom is tight** — only 2–6 kg remaining for actual litter depending on variant
- **Battery life of 2–4 hrs (EDU)** is adequate for scheduled patrol missions but payload will reduce runtime
- **15,000 mAh battery** is only standard on EDU; Air/Pro require upgrade purchase

### 1.3 Variant Recommendation for CleanWalker

**Go2 EDU ($14,500)** is the minimum viable variant. Reasons:
- Full Python/C++ SDK access required for custom arm control
- ROS2 support required for our perception pipeline
- Jetson Orin Nano onboard for ML inference
- 15,000 mAh battery for 2–4 hr runtime
- Foot force sensors for terrain adaptation
- 12 kg max payload

The EDU Plus ($22,500) adds Jetson Orin NX (100 TOPS vs 40 TOPS) and Mid-360 LiDAR, but the price premium is steep. We can add our own Jetson Orin Nano Super ($249) as secondary compute if needed.

---

## 2. Open Source Ecosystem

### 2.1 Key GitHub Repositories

| Repository | License | Purpose | Go2 Support |
|-----------|---------|---------|-------------|
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | BSD-3-Clause | Official SDK v2 (C++, CycloneDDS-based) | Yes |
| [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | BSD-3-Clause | Python interface for SDK v2 | Yes |
| [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | BSD-3-Clause | Official ROS2 support (state, control, LiDAR) | Yes (Go2, B2, H1) |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | BSD-3-Clause | RL locomotion training (Isaac Gym + MuJoCo) | Yes |
| [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) | BSD-3-Clause | RL locomotion (Isaac Lab based) | Yes |
| [unitree_model](https://github.com/unitreerobotics/unitree_model) | BSD-3-Clause | 3D models (USD format, deprecated → HuggingFace) | Yes |
| [unitree_ros](https://github.com/unitreerobotics/unitree_ros) | BSD-3-Clause | URDF/mesh files for RViz/Gazebo | Older models (Go1, A1) |
| [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) (unofficial) | Apache-2.0 | Community ROS2 SDK for Air/Pro/EDU | Yes (WebRTC + DDS) |
| [awesome-unitree-robots](https://github.com/shaoxiang/awesome-unitree-robots) | — | Curated project list | Comprehensive |

### 2.2 What This Means for CleanWalker

**Positives:**
- **BSD-3-Clause on all official repos** — fully permissive for commercial use
- **unitree_sdk2** gives high-level (sportmode) and low-level (joint torque/position/velocity) control
- **unitree_ros2** enables direct ROS2 integration via CycloneDDS — our perception pipeline can publish/subscribe natively
- **RL locomotion** frameworks available — we could train custom gaits (e.g., payload-optimized walking) and deploy via sim2real
- **URDF/mesh files available** — useful for simulation, renders, and digital twin development

**Gaps:**
- **unitree_model is deprecated** — 3D models moving to HuggingFace datasets
- **URDF for Go2 may be in unitree_ros** (older repo) but Go2-specific URDF availability needs verification
- **No official arm + locomotion co-control examples** — we'd need to build the loco-manipulation stack ourselves

### 2.3 SDK Capabilities (EDU variant)

The unitree_sdk2 provides:
- **High-level control:** Velocity commands, gait switching, obstacle avoidance
- **Low-level control:** Direct joint torque, position, and velocity control for all 12 joints
- **State reading:** IMU, joint states, foot force sensors, battery status
- **Sensor access:** LiDAR point clouds, camera feeds
- **Communication:** CycloneDDS (compatible with ROS2 middleware)

Sources: [Unitree SDK Docs](https://support.unitree.com/home/en/developer), [Unitree ROS2 Services](https://support.unitree.com/home/en/developer/ROS2_service)

---

## 3. Custom Hardware Mounting

### 3.1 Mounting Surface and Attachment Points

- **Top surface dimensions:** ~40 × 25 cm usable area on the back shell
- **Existing attachment:** Strap anchors on rear; beneath the rear anchor plate are **Ethernet RJ45, S-BUS, and XT30 power ports**
- **Official mounting kit:** [Futurology Modular Mounting Kit](https://futurology.tech/products/unitree-go2-modular-mounting-kit) — Picatinny rail system, 3D-printed PLA, quick-release, no permanent modifications required
- **Community designs:** [Thingiverse Go2 mount](https://www.thingiverse.com/thing:6463518) — 3D printable mounting platforms
- **Internal mounting holes:** Standard M4 screw pattern available

### 3.2 Robotic Arm Options

#### Option A: Unitree D1 Servo Arm (Official Accessory)

| Spec | Value |
|------|-------|
| **DOF** | 6 + 1 jaw clamp |
| **Weight** | 2.37 kg |
| **Payload** | 500 g |
| **Reach** | 495–670 mm (with jaw) |
| **Power** | 24V, 60W max (2.5A nominal, 5A peak) |
| **Interface** | Ethernet RJ45 |
| **Material** | Aluminum alloy |
| **Joint Range** | J1 ±135°, J2 ±90°, J3 ±90°, J4 ±135°, J5 ±90°, J6 ±135° |
| **Price** | ~$5,400 (D1 kit) |

**Assessment for CleanWalker:** The D1's 500g payload is **too low** for picking up full litter bags or heavier items. The reach (495–670mm) is adequate for ground-level pickup but the payload is a critical limitation. At $5,400 it's also expensive for what it offers.

Sources: [RobotShop D1](https://www.robotshop.com/products/unitree-go2-servo-robotic-arm-d1), [STEMfinity D1](https://stemfinity.com/products/d1-servo-arm), [Canada Satellite D1 Specs](https://www.canadasatellite.ca/Unitree-ARM-D1-Go2-Servo-Robotic-Arm-D1.htm)

#### Option B: Custom Lightweight Arm (Recommended)

For litter collection, we need:
- **1–2 kg payload** (bags, bottles, cans, etc.)
- **600–800 mm reach** (ground to back-mounted bin)
- **Weatherproof** (IP54+ for outdoor use)
- **Weight under 3 kg** (preserve payload budget)

Better alternatives:
- **Custom arm using Dynamixel XM540 servos** — build a 4–5 DOF arm optimized for pick-and-place, ~$2,000–$2,500 in parts
- **Foxtechrobot KN600** — 3 kg weight, 2 kg payload, IP55 rated, designed for outdoor use on mobile platforms
- **Our existing gripper arm design** from the from-scratch BOM (~$705) could be adapted to mount on Go2

### 3.3 Power Output for External Peripherals

| Port | Spec | Notes |
|------|------|-------|
| **XT30** | 28.8V (24–33.6V depending on charge) | Direct from battery, hidden under rear anchor plate |
| **S-BUS** | 3.3V | For RC receiver signals only |
| **USB 3.0 Type-A** | 5V (EDU only) | Standard USB power |
| **USB Type-C** | 5V (EDU only) | USB 2.0 and 3.0 ports |
| **Ethernet (2x RJ45)** | Data only (EDU) | Gigabit Ethernet for compute/sensors |

**Power Budget Assessment:**
- XT30 can power the arm (24V, 60W) and additional compute
- For Jetson Orin Nano Super: needs 5–25W at 5–19V — requires a buck converter from XT30
- Total external power draw estimate: arm (60W) + Jetson (25W) + sensors (10W) = ~95W — feasible but will reduce battery runtime by ~20–30%

Sources: [RoboVerse Internal Photos](https://wiki.theroboverse.com/en/internal-photos), [STEMfinity EDU Plus](https://stemfinity.com/products/go2-edu-plus)

---

## 4. Compute & Perception

### 4.1 Onboard Compute by Variant

| Variant | Main CPU | AI Compute | GPU | RAM | OS |
|---------|----------|------------|-----|-----|-----|
| Air/Pro/X | 8-core ARM | Minimal | Integrated | — | Proprietary |
| **EDU** | 8-core ARM + **Jetson Orin Nano** | **40 TOPS** | 1024 CUDA cores | 8 GB | Ubuntu 20.04 + ROS2 Foxy |
| **EDU Plus** | 8-core ARM + **Jetson Orin NX** | **100 TOPS** | 1024 CUDA cores | 16 GB | Ubuntu 20.04 + ROS2 Foxy |

### 4.2 Can We Add Secondary Compute?

**Yes.** The Go2 EDU's Ethernet ports and XT30 power make it straightforward to add a companion computer:

- **Jetson Orin Nano Super ($249):** 67 TOPS, 8 GB RAM, 25W envelope — ideal as a dedicated perception processor
- **Mount location:** On the back platform via Picatinny rail or custom 3D-printed mount
- **Connection:** Ethernet to Go2's onboard Jetson for ROS2 communication
- **Power:** Buck converter from XT30 (28.8V → 19V)

Evidence of successful integration: Research documented at [OreateAI blog](https://www.oreateai.com/blog/research-on-the-integrated-application-of-unitree-go2-robot-dog-with-nvidia-jetson-orin-nx-motherboard-and-arduino-peripherals/0aba334f9e4dea874adca729c180d73f) shows Jetson + Arduino peripherals integrated with Go2.

### 4.3 Camera/Sensor Integration

**Built-in sensors (all variants):**
- 4D LiDAR L2 — 360° × 90° hemispherical FOV, 0.05m min range
- HD wide-angle camera (front-facing)

**Adding custom cameras (EDU only):**
- USB cameras via USB 3.0 port (e.g., OAK-D Pro for our depth/YOLO pipeline)
- Ethernet cameras via RJ45
- RealSense depth camera (some EDU units ship with one pre-installed)

**For CleanWalker, we would add:**
- OAK-D Pro or similar stereo depth camera ($199–$299) — for litter detection
- Downward-facing camera for close-range grasp planning
- These connect via USB to either the onboard Jetson or our secondary compute

### 4.4 ROS2 Architecture for CleanWalker on Go2

```
┌─────────────────────────────────────────────┐
│               Go2 EDU Onboard               │
│  ┌─────────────┐    ┌──────────────────┐    │
│  │ Locomotion   │    │ Jetson Orin Nano │    │
│  │ Controller   │◄──►│ (40 TOPS)        │    │
│  │ (Unitree     │    │ - ROS2 Foxy      │    │
│  │  firmware)   │    │ - SDK2 bridge     │    │
│  └─────────────┘    │ - Arm control     │    │
│                      └───────┬──────────┘    │
│                              │ Ethernet      │
└──────────────────────────────┼───────────────┘
                               │
                    ┌──────────┴──────────┐
                    │ Secondary Jetson     │
                    │ Orin Nano Super      │
                    │ (67 TOPS)            │
                    │ - YOLO litter detect │
                    │ - Grasp planning     │
                    │ - Navigation/SLAM    │
                    │ - Mission control    │
                    └─────────────────────┘
```

---

## 5. Updated BOM & Cost Model

### 5.1 Go2-Based Prototype BOM

| Component | Option | Cost | Notes |
|-----------|--------|------|-------|
| **Go2 EDU base platform** | Unitree Go2 EDU | $14,500 | Includes locomotion, LiDAR, Jetson Orin Nano, 15Ah battery |
| **Custom bag dispensary** | Our design (CNC + 3D print) | $400–$600 | Aluminum frame + printed parts, servo-actuated |
| **Robotic arm** | Custom Dynamixel-based (4 DOF) | $2,000–$2,500 | XM540 servos, custom links, 1–2 kg payload |
| **Secondary compute** | Jetson Orin Nano Super | $249 | 67 TOPS, dedicated to perception |
| **Depth camera** | OAK-D Pro | $299 | Stereo depth + RGB for litter detection |
| **Downward camera** | OAK-D Lite | $149 | Close-range grasp planning |
| **Mounting system** | Custom CNC + Picatinny adapters | $150–$250 | Aluminum platform, M4 mounting |
| **Buck converter + wiring** | Off-the-shelf | $50–$80 | 28.8V → 19V for Jetson, 24V for arm |
| **Weather enclosure** | Custom 3D print + silicone seals | $100–$200 | Protect electronics from light rain/dust |
| **Shipping** | DHL/FedEx | $500–$1,000 | From Unitree (China) to US |
| | | | |
| **Subtotal** | | **$18,397–$19,879** | |
| **Contingency (10%)** | | $1,840–$1,988 | |
| **TOTAL** | | **$20,237–$21,867** | |

### 5.2 Alternative: Go2 Pro + External Compute (Budget Option)

If we use Go2 Pro instead (forgoing built-in Jetson and ROS2 support):

| Component | Cost | Notes |
|-----------|------|-------|
| Go2 Pro | $2,800 | No SDK — limited to high-level app control |
| Jetson Orin Nano Super | $249 | All compute external |
| Everything else (arm, bag, sensors) | ~$3,400 | Same as above |
| **TOTAL** | **~$7,100–$8,100** | |

**Problem:** Go2 Pro has no official SDK or ROS2 support. The [unofficial go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) supports Pro via WebRTC but this is unreliable for production. **Not recommended.**

### 5.3 Comparison: Go2-Based vs From-Scratch

| Metric | From-Scratch (current BOM) | Go2 EDU-Based | Go2 Pro (budget) |
|--------|---------------------------|---------------|------------------|
| **Prototype unit cost** | $10,200–$14,600 | $20,200–$21,900 | $7,100–$8,100 |
| **Cost at 10 units** | ~$9,000–$12,000 | ~$18,000–$20,000 | N/A (no SDK) |
| **Cost at 100 units** | ~$6,300–$8,800 | ~$16,000–$18,000* | N/A |
| **Cost at 1,000 units** | ~$4,500–$6,200 | ~$14,000–$16,000* | N/A |
| **Time to walking prototype** | 6–12 months | 2–4 weeks | 2–4 weeks |
| **Locomotion quality** | Unproven (custom) | Proven (Unitree) | Proven |
| **Weatherproofing** | Custom (IP54+) | **None** — must add | **None** |
| **Payload headroom** | Designed for task | Tight (2–6 kg margin) | Tight |
| **Supply chain risk** | Multi-vendor | Single vendor (Unitree) | Single vendor |
| **Customizability** | Full control | Limited to SDK | Very limited |
| **Regulatory/certification** | Self-certified | Depends on Unitree | Depends on Unitree |

*Go2 volume pricing assumes no discount from Unitree. With enterprise agreement, 10–20% discount possible.

### 5.4 Cost at Scale — Critical Issue

The Go2 EDU's $14,500 base cost **does not decrease significantly at volume** because it's a finished product, not a component. Our from-scratch BOM drops from $10,200 → $4,500 at 1,000 units due to actuator bulk pricing. The Go2 stays at ~$12,000–$13,000 even at volume.

**This means:** At production scale (100+ units), from-scratch is **$8,000–$10,000 cheaper per unit.**

---

## 6. Competitive & Legal Considerations

### 6.1 Commercial Use / Resale Rights

- **SDK License:** BSD-3-Clause — **fully permissive** for commercial use, modification, and redistribution
- **Hardware:** Standard commercial purchase — no known restrictions on resale of products built on the platform
- **Enterprise variants exist:** Go2 ENT ($14,500) is specifically designed for commercial deployment (law enforcement, inspection, agriculture), indicating Unitree supports commercial use
- **No exclusive distribution agreements** found in public documentation

**Caveat:** Unitree's [Terms & Policies](https://www.unitree.com/mobile/terms/policy/) should be reviewed by legal counsel before committing. Specific OEM or white-label agreements may require direct negotiation with Unitree.

### 6.2 Companies Building on Go2

- **DroneBlocks** — Educational robotics platform using Go2 EDU ([source](https://droneblocks.io/introducing-the-unitree-go2-edu-quadruped-a-new-era-of-robotics-education-with-droneblocks/))
- **InDro Robotics** — Authorized reseller and integration partner ([source](https://indrorobotics.ca/good-dogs-a-look-at-the-newest-unitree-quadrupeds/))
- **Agriculture partnerships** — Unitree partnered with research institutions for smart farming ([source](https://www.prnewswire.com/news-releases/from-streets-to-fields-unitrees-go2-steps-into-smart-farming-302513086.html))
- **Go2 ENT variants** — Used in search & rescue, security patrol, and industrial inspection
- **Academic research** — Extensive use in university robotics labs worldwide (locomotion, manipulation, SLAM)

### 6.3 Geopolitical / Supply Chain Risk

- **Unitree is Chinese (Hangzhou)** — subject to potential tariffs, export controls, and trade restrictions
- **Current tariff situation:** 49% tariff on Chinese robotics components (per our logistics-transport-costs.md analysis)
- **No known US-based alternatives** at this price point (Boston Dynamics Spot is $74,500)
- **Mitigation:** Stock extra units and spare batteries; negotiate with Unitree for US warehouse fulfillment

---

## 7. Risk Assessment

### 7.1 High-Risk Issues

| Risk | Severity | Mitigation |
|------|----------|------------|
| **No weatherproofing (no IP rating)** | **CRITICAL** | Must add custom enclosure + conformal coating. Light rain will damage electronics. Cannot operate in wet conditions without modification. |
| **Payload limit** | **HIGH** | 8 kg recommended, 12 kg max. Our system needs ~6 kg before litter. Only 2–6 kg headroom for collected litter. May need to limit mission scope. |
| **Single-vendor dependency** | **HIGH** | If Unitree discontinues Go2, changes pricing, or faces trade restrictions, we have no fallback. Mitigate with spares stockpile. |
| **Cost at scale** | **HIGH** | Go2 EDU doesn't get cheaper at volume. At 100+ units, from-scratch saves $8–10K per unit. Go2 is a prototype tool, not a production platform. |

### 7.2 Medium-Risk Issues

| Risk | Severity | Mitigation |
|------|----------|------------|
| **Battery life under payload** | MEDIUM | 2–4 hrs nominal, expect 1.5–2.5 hrs with full payload + arm + compute. Carry spare batteries ($200–$300 each). |
| **Custom arm integration** | MEDIUM | No off-the-shelf arm meets our needs. D1 arm has only 500g payload. Must build custom. |
| **Tariff exposure** | MEDIUM | 49% tariff on Chinese robotics. $14,500 base becomes ~$21,600 landed. |
| **Software updates** | MEDIUM | Unitree firmware updates could break our integration. Pin firmware versions and test updates. |
| **Warranty void** | MEDIUM | Custom mounting, weatherproofing, and power draw may void warranty. Confirm with Unitree. |

### 7.3 Low-Risk Issues

| Risk | Severity | Mitigation |
|------|----------|------------|
| SDK limitations | LOW | BSD-3-Clause, full access on EDU. Low-level joint control available. |
| ROS2 compatibility | LOW | Official support + active community. CycloneDDS native. |
| 3D models for renders | LOW | USD models available on GitHub/HuggingFace. URDF via unitree_ros. |
| RL locomotion | LOW | Can train custom gaits via unitree_rl_gym if needed. |

---

## 8. Recommendation

### 8.1 Recommended Strategy: "Go2 for Prototyping, Custom for Production"

**Phase 1 — Rapid Prototype (Months 1–3): Use Go2 EDU**
- Purchase 1x Go2 EDU ($14,500)
- Mount custom bag dispensary, arm, and perception stack
- Validate the complete litter collection workflow
- Test in controlled outdoor environments (dry weather)
- **Budget: ~$21,000**

**Phase 2 — Pilot Validation (Months 3–6): 2–3 Go2 EDU units**
- Build 2–3 more pilot units for field testing
- Develop weatherproofing enclosure
- Prove the business model with real customers
- Collect operational data for custom platform design
- **Budget: ~$55,000–$65,000**

**Phase 3 — Custom Platform Transition (Months 6–12)**
- Use Go2 operational data to spec the custom quadruped
- Our from-scratch BOM ($10,200 prototype, $4,500 at 1,000 units) is far more economical at scale
- Custom platform gets IP54+ weatherproofing, optimized payload, and no vendor lock-in
- Reuse all software (ROS2 nodes, YOLO models, arm control) — just swap the locomotion layer

### 8.2 If Budget Is Extremely Tight

Skip to the **Go2 Pro ($2,800) + unofficial ROS2 SDK** path as a proof-of-concept only. This doesn't give production-grade SDK access but can demonstrate the concept for investor pitches at ~$7,000 total. **Do not ship this to customers.**

### 8.3 Key Decision Factors for Walker

| Factor | Go2 EDU | From-Scratch |
|--------|---------|--------------|
| **Time to first demo** | 4–6 weeks | 6–12 months |
| **Prototype cost** | ~$21K | ~$10–15K |
| **Production cost (100 units)** | ~$18K/unit | ~$7–9K/unit |
| **Weatherproofing** | Must add (risky) | Built-in |
| **Investor optics** | Fast demo = good | Longer but more defensible |
| **IP/defensibility** | Low (anyone can buy Go2) | High (custom platform) |
| **Engineering risk** | Low (proven locomotion) | High (unproven gait) |

**Bottom line:** The Go2 EDU is an excellent **risk-reduction tool** — it gets us to a walking, picking prototype in weeks instead of months. But it's ~2x the cost of from-scratch at production scale, has no weatherproofing, and creates vendor dependency. Use it to validate, then graduate to custom hardware.

---

## Appendix A: Useful Links

- Unitree Official Go2 page: https://www.unitree.com/go2/
- Unitree Shop: https://shop.unitree.com/products/unitree-go2
- SDK Documentation: https://support.unitree.com/home/en/developer
- GitHub — unitree_sdk2: https://github.com/unitreerobotics/unitree_sdk2
- GitHub — unitree_ros2: https://github.com/unitreerobotics/unitree_ros2
- GitHub — unitree_rl_gym: https://github.com/unitreerobotics/unitree_rl_gym
- GitHub — unitree_model: https://github.com/unitreerobotics/unitree_model
- GitHub — go2_ros2_sdk (unofficial): https://github.com/abizovnuralem/go2_ros2_sdk
- GitHub — awesome-unitree-robots: https://github.com/shaoxiang/awesome-unitree-robots
- D1 Arm specs: https://www.canadasatellite.ca/Unitree-ARM-D1-Go2-Servo-Robotic-Arm-D1.htm
- Mounting Kit: https://futurology.tech/products/unitree-go2-modular-mounting-kit
- Jetson Orin Nano Super: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/
- Community 3D mount: https://www.thingiverse.com/thing:6463518

## Appendix B: Go2 EDU vs Boston Dynamics Spot

For completeness — Spot is the obvious alternative commercial platform:

| Spec | Go2 EDU | Spot (Explorer) |
|------|---------|-----------------|
| Price | ~$14,500 | ~$74,500 |
| Weight | 15 kg | 32 kg |
| Payload | 8–12 kg | 14 kg |
| Runtime | 2–4 hrs | ~90 min |
| IP Rating | None | IP54 |
| SDK | Full (BSD-3) | Full (proprietary) |
| Speed | 3.7 m/s | 1.6 m/s |

Spot has IP54 weatherproofing and higher payload, but at 5x the cost, it's not viable for our unit economics ($2,800–$3,500/mo RaaS pricing).
