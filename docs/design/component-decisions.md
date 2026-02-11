# CW-1 Component Selection Decision Matrix

**Version:** 1.0
**Date:** 2026-02-11
**Purpose:** Procurement reference for all major CW-1 subsystem components. Every decision is backed by research from the project's technical documents, URDF mass model, and supplier data.
**Status:** READY FOR PROCUREMENT (pending open questions noted per subsystem)

---

## Reference Documents

| Document | Key Data Used |
|----------|--------------|
| `docs/design/robot-design-spec.md` (V2.3) | Physical dimensions, mass budget, mechanical layout |
| `docs/research/depth-grasp-cameras-2026.md` | Camera, LiDAR, depth estimation, grasp planning |
| `docs/research/edge-detection-sota-2026.md` | Object detection models, Jetson performance |
| `docs/technical/perception-pipeline-architecture.md` | Full perception stack, memory/GPU/latency budgets |
| `docs/hardware-bom-research.md` | Supplier pricing, BOM at scale |
| `docs/assembly-iteration-costs.md` | Iteration costs, tooling, engineering estimates |
| `hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf` | 15 kg mass model, link masses, joint limits, geometry |

---

## Mass Budget Summary (from CW-1 URDF)

| Subsystem | Mass (kg) | % of Total |
|-----------|-----------|-----------|
| Body (chassis + electronics + battery) | 8.0 | 53.3% |
| Head (sensor module) | 0.5 | 3.3% |
| Legs (4 × 1.2 kg: hip 0.4 + thigh 0.5 + calf 0.2 + foot 0.1) | 4.8 | 32.0% |
| Arm (turret 0.3 + upper 0.3 + forearm 0.3 + wrist 0.15 + gripper 0.15) | 1.2 | 8.0% |
| Bag system (roll 0.2 + hinge 0.1 + frame 0.2) | 0.5 | 3.3% |
| **Total** | **15.0** | **100%** |

---

## 1. Compute: Jetson Orin Nano Super

### Decision

**NVIDIA Jetson Orin Nano Super 8GB** — $249 (dev kit)

### Why

- 67 TOPS INT8 handles our full perception stack at 52% GPU utilization with 48% headroom
- 8 GB unified memory: our stack uses ~3.6 GB, leaving 4.6 GB (57%) headroom
- Native TensorRT, CUDA, Isaac ROS support — no custom inference framework needed
- 6-core A78AE CPU adequate for ROS2 + Nav2 + behavior tree + EKF
- 7W/15W/25W power modes; 15W typical in field = 9.5% of power budget
- JetPack 6.x ecosystem with pre-built Docker containers for Isaac ROS

### Alternatives Considered

| Alternative | TOPS | RAM | Price | Verdict |
|-------------|------|-----|-------|---------|
| Jetson Orin NX 16GB | 100 | 16 GB | ~$599 | Overkill — our stack uses <5 GB RAM, <52% GPU. 2.4× cost for marginal benefit. Reserve as upgrade path if memory pressure emerges. |
| Raspberry Pi 5 + Hailo-8 | 26 | 8 GB | $285 | No CUDA/TensorRT. Would require complete inference pipeline rewrite. No Isaac ROS. 2.6× less AI throughput. Not viable for our multi-model stack. |
| Raspberry Pi 5 + Coral USB | 4 | 8 GB | $160 | Far too weak. Cannot run YOLO26s + ESS + GR-ConvNet concurrently. |

### Cost at Scale

| Volume | Unit Cost | Notes |
|--------|-----------|-------|
| 1 (dev kit) | $249 | Includes carrier board |
| 10 | $249 | Dev kit pricing holds |
| 100 | $280 | Orin Nano module ($200) + custom carrier ($80) |
| 1,000 | $190 | Module volume pricing ($150) + carrier ($40) |

### Lead Time

- Dev kit: In stock at NVIDIA, Amazon, Arrow — ships same week
- Module (100+ qty): 4-8 weeks via NVIDIA distribution (Arrow, Mouser)

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | Mature product, multiple distributors, no allocation issues as of Feb 2026 |
| Technical | LOW | Production-proven on thousands of robots (Unitree, AgileX, etc.) |
| Integration | LOW | Isaac ROS Docker containers eliminate most integration work |

### Open Questions

1. **Custom carrier board design** — needed at 100+ units. Budget 40-80 hours EE time + $200-400 per board prototype run at JLCPCB.
2. **25W Super mode thermal management** — may need active cooling in enclosed body. Test in prototype.

### Confidence: **HIGH**

---

## 2. Camera: Luxonis OAK-D Pro (Fixed-Focus)

### Decision

**Luxonis OAK-D Pro (Fixed-Focus variant)** — $399

### Why

- **IR dot projector** solves the outdoor low-texture problem (pavement, concrete, grass) that kills passive stereo
- **Myriad X VPU** (4 TOPS) offloads lightweight preprocessing — zero Jetson GPU cost for on-device stereo depth
- **Fixed-focus variant** designed specifically for vibrating platforms (drones, robots) — eliminates defocus from gait vibration
- **Native RGB-D alignment** — exactly the input format GR-ConvNet v2 needs (224×224 RGB-D)
- **12 MP RGB sensor** (IMX378) — excellent resolution for YOLO26s litter detection
- **BNO086 IMU** — 400 Hz accel, 1000 Hz gyro, hardware-synced timestamps for cuVSLAM visual-inertial SLAM
- **DepthAI SDK + ROS2 Humble** — well-maintained, active community, MIT license
- Stereo baseline: 75 mm; depth range: 0.7-15 m (active IR), 0.2-35 m (passive)

### Alternatives Considered

| Alternative | Price | Pros | Cons | Verdict |
|-------------|-------|------|------|---------|
| **Orbbec Gemini 345Lg** | TBD (CES 2026) | IP67, 100+ klux, GMSL2, -20°C to 65°C — purpose-built for extreme outdoor robotics | Pricing unknown, no eval units available yet, new product with no field track record | **Watch closely.** Contact Orbbec for eval unit. Best potential outdoor camera if priced <$500. Could replace OAK-D Pro in DVT/PVT phase. |
| **Stereolabs ZED 2i** | $499-$549 | IP66, excellent sunlight (passive stereo), 2208×1242 | No on-device VPU (all processing on Jetson), $100-$150 more, 166g vs 91g, no IR projector (fails on textureless surfaces) | Good backup if OAK-D Pro sunlight issues are worse than expected. |
| **Stereolabs ZED X Mini** | ~$549 + $200 deserializer | IP67, global shutter, GMSL2 | $749 total, needs deserializer board, no IR projector | Too expensive for MVP. Consider for production if weatherproofing is critical. |
| **Intel RealSense D435i** | $334 | Cheaper, IMU included | EOL product line (wound down 2023), minimal firmware updates, community-driven only. Avoid for new designs. | **Rejected.** Dead platform. |
| **OAK-D Lite** | $149 | Budget option | No IR projector (struggles outdoors), lower resolution, no IMU | Acceptable for indoor lab testing only. Not field-viable. |

### Cost at Scale

| Volume | Unit Cost | Notes |
|--------|-----------|-------|
| 1 | $399 | Luxonis direct |
| 10 | $380 | Small volume discount |
| 100 | $340 | Volume pricing via Luxonis sales |
| 1,000 | $280 | OEM pricing (estimate; contact Luxonis sales) |

### Lead Time

- 1-10 units: In stock at Luxonis shop, ships within 1 week
- 100+ units: 4-6 weeks (contact sales@luxonis.com)

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | Established product, in-stock, multiple sales channels |
| Technical: sunlight IR degradation | MEDIUM | Falls back to passive stereo gracefully; DA V2 Small as supplementary dense depth |
| Technical: 0.7m minimum depth | MEDIUM | May miss items directly under robot. Test in prototype. Consider OAK-D S2 (0.2m min) as wrist camera supplement. |
| Technical: no IP rating | MEDIUM | Requires custom 3D-printed weatherproof enclosure with clear lens window. Budget $20-50 per enclosure. |
| Integration | LOW | DepthAI ROS2 driver is mature, well-documented |

### Open Questions

1. **How badly does IR degrade at solar noon in summer?** — Needs field testing with prototype in direct sunlight.
2. **0.7m minimum depth for close-range grasp verification** — May need secondary close-range camera (OAK-D S2, $149, 0.2m min) mounted on arm wrist.
3. **Orbbec Gemini 345Lg pricing** — Contact Orbbec for eval unit. If <$500 with IP67, it becomes the clear winner.
4. **Weatherproof enclosure design** — Need to maintain optical path quality through enclosure window. Test with polycarbonate and glass windows.

### Confidence: **HIGH** (for MVP); **MEDIUM** (for production — Gemini 345Lg may supersede)

---

## 3. LiDAR: Livox Mid-360

### Decision

**Livox Mid-360** — $749

### Why

- **360° × 59° FOV** — full surround coverage for RTAB-Map scan matching and Nav2 costmap generation
- **70 m range** at 10% reflectivity — far exceeds our 10-15 m operational needs, provides generous safety margin
- **200,000 pts/s** with non-repetitive scanning — yields dense maps over time
- **IP67** — fully weatherproof out of the box, no enclosure needed
- **265 g** — lightweight for our 15 kg mass budget
- **100BASE-T1 Ethernet** interface — reliable, no USB bandwidth contention with OAK-D Pro
- Proven on quadruped platforms (multiple research groups, Unitree Go2 EDU integrations)
- ROS2 driver available via Livox SDK2

### Alternatives Considered

| Alternative | Price | FOV | Range | IP | Verdict |
|-------------|-------|-----|-------|----|---------|
| **RPLidar A1 (2D)** | $99 | 360° H only | 12 m | None | Cheap safety layer, but 2D only — no ground plane estimation, no 3D obstacle mapping. Acceptable for POC (v0.1) only. |
| **RPLidar S2E (2D)** | $249 | 360° H only | 30 m | None | Better range but still 2D. Missing vertical FOV needed for step/curb detection. |
| **RPLidar A3 (2D)** | $599 | 360° H | 25 m | None | Still 2D. At $599, the $150 upgrade to 3D Livox is a no-brainer. |
| **Ouster OS0-32** | ~$2,000+ | 360° × 90° | 50 m | IP68 | Superior FOV but 2-3× the price. Overkill for park/sidewalk navigation. |
| **Velodyne VLP-16** | ~$4,000 | 360° × 30° | 100 m | IP67 | Classic but grossly overpriced for our application. Legacy product. |
| **No LiDAR (camera only)** | $0 | — | — | — | Viable for very early prototype (v0.1) using VIO only. Adds risk for outdoor obstacle avoidance. Not recommended for field trials. |

### Cost at Scale

| Volume | Unit Cost | Notes |
|--------|-----------|-------|
| 1 | $749 | DJI Store direct (listed at $979 some places; $749 via DJI/Livox) |
| 10 | $700 | Volume inquiry via Livox sales |
| 100 | $600 | OEM pricing estimate |
| 1,000 | $500 | Direct OEM negotiation |

### Lead Time

- 1-10 units: In stock at DJI Store, 1-2 weeks shipping
- 100+ units: 4-8 weeks via Livox/DJI distribution

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | DJI/Livox is a major manufacturer, global distribution |
| Technical | LOW | Well-characterized on quadrupeds, extensive field data from drone/robotics community |
| Integration | LOW | Livox SDK2 + ROS2 driver available, Ethernet interface reliable |
| Price | MEDIUM | At $749, LiDAR is the second-most-expensive sensor. Could defer to v0.2 if budget constrained for POC. |

### Open Questions

1. **Can we defer LiDAR to EVT phase?** — POC (v0.1) can use camera-only VIO + 2D RPLidar A1 ($99) for basic obstacle avoidance. Mid-360 adds $650 to each early prototype.
2. **Ethernet interface on custom carrier board** — Need 100BASE-T1 automotive Ethernet support on our Jetson carrier. Standard Gigabit Ethernet should work with adapter.

### Confidence: **HIGH**

---

## 4. Leg Actuators (12 joints)

### Decision

**CubeMars AK70-10** (hip pitch, knee pitch — 8 units) + **CubeMars AK60-6** (hip yaw — 4 units)

### Torque Analysis (from CW-1 URDF)

**Robot parameters:**
- Total mass: 15 kg
- Upper leg: 0.20 m, lower leg: 0.20 m
- Mass per leg: 1.2 kg (hip 0.4 + thigh 0.5 + calf 0.2 + foot 0.1)
- Mass above legs: 10.2 kg (body 8.0 + head 0.5 + arm 1.2 + bag 0.5)

**Ground reaction forces (GRF):**

| Gait | Legs on Ground | GRF per Leg | Notes |
|------|---------------|-------------|-------|
| Static standing (4 legs) | 4 | 36.8 N (3.75 kg) | Baseline |
| Trot gait (2 legs) | 2 | 73.6 N (7.5 kg) | Normal walking |
| Dynamic peak (stumble recovery) | 1-2 | 147-294 N (15-30 kg) | 2-3× body weight |

**Joint torque requirements:**

| Joint | Static (4-leg) | Trot (2-leg) | Dynamic Peak (3×) | URDF Effort Limit | Required Motor Peak |
|-------|---------------|-------------|-------------------|-------------------|-------------------|
| **Hip yaw** (abduction) | ~1.5 Nm | ~3 Nm | ~6 Nm | 30 Nm | ≥9 Nm (1.5× safety) |
| **Hip pitch** (swing) | ~3.7 Nm | ~7.4 Nm | ~15 Nm | 30 Nm | ≥22 Nm (1.5× safety) |
| **Knee pitch** (bend) | ~3.7 Nm | ~7.4 Nm | ~22 Nm | 30 Nm | ≥25 Nm (1.1× safety) |

*Hip pitch calculation: GRF × moment arm. At typical 30° stance angle, horizontal moment arm from hip to foot ≈ 0.20 m. τ = 73.6 N × 0.20 m × safety factor.*

*Knee pitch calculation: Worst case during push-off, knee straightening against full body weight. τ = 147 N × 0.15 m moment arm = 22 Nm.*

**Motor selection mapping:**

| Joint (×4 legs) | Qty | Motor | Rated Torque | Peak Torque | Margin vs Dynamic Peak | Voltage |
|-----------------|-----|-------|-------------|-------------|----------------------|---------|
| Hip yaw | 4 | **AK60-6 V1.1** | 6 Nm | **9 Nm** | 9/6 = 1.5× | 24V |
| Hip pitch | 4 | **AK70-10** | 10 Nm | **24.8 Nm** | 24.8/15 = 1.65× | 48V |
| Knee pitch | 4 | **AK70-10** | 10 Nm | **24.8 Nm** | 24.8/22 = 1.13× | 48V |

**Assessment:** AK70-10 provides adequate margin for hip pitch (1.65×) but tight margin for knee peak dynamic loads (1.13×). This is acceptable because:
1. Dynamic peaks (3× body weight) are brief transients, not sustained loads
2. CubeMars AK series has field-oriented control (FOC) with current limiting — prevents damage
3. The Unitree Go2 (also 15 kg) uses motors with similar ~23-25 Nm peak torque
4. Our 15 kg mass is conservative — actual prototype may weigh 12-14 kg

**Mixed voltage concern:** AK60-6 runs at 24V while AK70-10 runs at 48V. Options:
- Run AK60-6 at 48V with firmware voltage limit (supported by CubeMars driver)
- Use 48V→24V DC-DC converter for hip yaw motors
- Upgrade all 12 to AK70-10 for voltage uniformity (+$800 for prototype)

### Alternatives Considered

| Alternative | Torque (peak) | Price (×12) | Pros | Cons | Verdict |
|-------------|-------------|-------------|------|------|---------|
| **CubeMars AK60-6 + AK70-10 (mixed)** | 9/24.8 Nm | $5,188 | Best price/performance, proven in research quadrupeds, CAN bus + FOC built in | Mixed voltage (24V/48V), AK60-6 tight for hip yaw if robot gains weight | **Selected for prototype** |
| **MyActuator RMD-X6-H + RMD-X8 (mixed)** | 6/18 Nm | $4,620 | Cheaper, good CAN bus support | RMD-X8 peak only 18 Nm — insufficient for knee (needs 22+ Nm). RMD-X6-H only 6 Nm — same as AK60-6. | **Selected for production** (if RMD-X10 or higher torque variant becomes available) |
| **All AK70-10 (uniform)** | 24.8 Nm all | $5,988 | Uniform voltage (48V), ample torque everywhere, simpler power distribution | $800 more than mixed, slightly heavier (500g vs 305g for hip yaw) | **Strong alternative.** Consider if weight budget allows +0.8 kg for legs. |
| **Dynamixel XM540-W270** | 10.6 Nm stall | $5,796 | Excellent SDK, daisy-chain, easy programming | Stall torque only 10.6 Nm — insufficient for dynamic locomotion. Not designed for QDD control. Will overheat under continuous load. | **Rejected for legs.** |
| **Custom QDD (MIT Mini Cheetah style)** | Tunable | ~$900-$1,500 per motor | Maximum performance, proprietary advantage | 12-18 months development, $10K-$18K for 12 motors, huge engineering risk | **Phase 3+.** Not viable for MVP. |

### Cost at Scale

| Volume | Cost (12 actuators) | Per-Motor Average |
|--------|-------------------|-------------------|
| 1 (prototype) | $5,188 (8×$499 + 4×$299) | $432 |
| 10 | $4,800 | $400 |
| 100 | $4,360 (est. 10-15% discount) | $363 |
| 1,000 | $3,500 (direct OEM) | $292 |

### Lead Time

- 1-10 units: In stock at CubeMars store — ships within 1-2 weeks
- 50+ units: Contact sales@cubemars.com for volume pricing and lead time (typically 4-6 weeks)

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | CubeMars (T-Motor) is established, global distribution |
| Technical: thermal | MEDIUM | Sustained walking at 15 kg may push thermal limits. Add aluminum heat sinks to motor housings. Monitor temperature via CAN bus telemetry. |
| Technical: knee margin | MEDIUM | 1.13× margin on knee peak. Mitigate with gait optimization (avoid full-extension push-off), weight reduction, or upgrade to all-AK70-10. |
| Technical: mixed voltage | LOW | CubeMars driver supports configurable voltage limits. 48V bus with per-motor regulation is standard practice. |

### Open Questions

1. **All-AK70-10 vs mixed?** — Adding 0.8 kg to legs (4 × 200g heavier motors) saves voltage complexity. Decision depends on final weight budget after full system integration.
2. **Thermal performance under sustained outdoor operation** — Need bench testing: run motors at expected walking load for 2+ hours, monitor winding temperature via CAN bus.
3. **CAN bus topology** — 12 motors on one CAN bus is near the limit. Consider dual CAN bus (left legs + right legs) for redundancy and bandwidth.
4. **Production transition to MyActuator** — RMD-X8 peak torque (18 Nm) is insufficient for knee. Need to evaluate RMD-X10 or negotiate custom gear ratio with MyActuator for production volumes.

### Confidence: **HIGH** (prototype); **MEDIUM** (production — need to resolve supply chain for 1000+ units)

---

## 5. Arm Actuators (5 joints)

### Decision

**Dynamixel XM430-W350-T** (turret yaw, shoulder pitch, elbow pitch) + **Dynamixel XL430-W250-T** (wrist pitch, gripper)

### Torque Analysis (from CW-1 URDF)

**Arm parameters:**
- Upper arm: 0.18 m, 0.3 kg
- Forearm: 0.18 m, 0.3 kg
- Wrist: 0.05 m, 0.15 kg
- Gripper: 0.10 m, 0.15 kg
- Max payload: 0.5 kg (500 mL water bottle)
- Total mass below shoulder: 1.4 kg (including payload)

**Joint torque requirements (arm horizontal, worst case):**

| Joint | Mass Below | CG Distance | Static Torque | With 2× Safety | URDF Effort Limit |
|-------|-----------|-------------|--------------|----------------|-------------------|
| **Turret yaw** | 1.2 kg (arm) | N/A (rotation) | ~1-2 Nm (inertial) | ~3 Nm | 15 Nm |
| **Shoulder pitch** | 1.4 kg | ~0.23 m | 3.2 Nm | **6.3 Nm** | 15 Nm |
| **Elbow pitch** | 1.1 kg | ~0.15 m | 1.6 Nm | **3.2 Nm** | 15 Nm |
| **Wrist pitch** | 0.65 kg | ~0.05 m | 0.32 Nm | **0.64 Nm** | 10 Nm |
| **Gripper** | 0.5 kg payload | ~0.04 m grip | ~5-15 N force | ~10-30 N | 5 Nm |

**Motor selection mapping:**

| Joint | Motor | Stall Torque | Price | Margin |
|-------|-------|-------------|-------|--------|
| Turret yaw | XM430-W350-T | **4.1 Nm** | $270 | 4.1/3.0 = 1.4× |
| Shoulder pitch | XM430-W350-T | **4.1 Nm** | $270 | 4.1/6.3 = **0.65×** (see note) |
| Elbow pitch | XM430-W350-T | **4.1 Nm** | $270 | 4.1/3.2 = 1.3× |
| Wrist pitch | XL430-W250-T | **1.5 Nm** | $45 | 1.5/0.64 = 2.3× |
| Gripper | XL430-W250-T | **1.5 Nm** | $45 | Ample for litter gripping |

**Shoulder concern:** XM430-W350-T stall torque (4.1 Nm) is below the 2× safety margin (6.3 Nm) for shoulder pitch with full payload at horizontal extension. Mitigations:
1. Arm rarely holds 500g payload at full horizontal extension simultaneously
2. During pickup, arm approaches from above (gravity-assisted, not fighting gravity)
3. Upgrade to **XM540-W270-T** ($483, 10.6 Nm stall) for shoulder if XM430 proves insufficient in testing
4. Alternatively, use a CubeMars AK60-6 ($299, 9 Nm peak) for shoulder — matches leg voltage bus

### Alternatives Considered

| Alternative | Stall Torque | Price (5 joints) | Pros | Cons | Verdict |
|-------------|-------------|-------------------|------|------|---------|
| **Dynamixel XM430 + XL430 (mixed)** | 4.1/1.5 Nm | $855 (3×$270 + 2×$45) | Excellent SDK, daisy-chain TTL, ROS2 drivers, position/velocity/current modes | Shoulder margin tight with heavy payload | **Selected** |
| **All Dynamixel XM540-W270** | 10.6 Nm | $2,415 (5×$483) | Ample torque everywhere | Overkill for wrist/gripper, 3× the cost | Overkill for most joints |
| **CubeMars AK60-6 for shoulder + Dynamixel for rest** | 9/4.1/1.5 Nm | ~$654 (1×$299 + 2×$133 est. + 2×$45) | Best shoulder torque, CAN bus matches legs | Mixed protocol (CAN + TTL), more complex wiring | **Consider if XM430 shoulder fails testing** |
| **Hobby servos (MG996R etc.)** | ~1-2 Nm | ~$100 total | Very cheap | No feedback, no current control, unreliable for continuous operation, will fail in field | **Rejected.** Hobby-grade is not acceptable for outdoor continuous operation. |

### Cost at Scale

| Volume | Cost (5 arm joints) | Notes |
|--------|-------------------|-------|
| 1 | $855 | 3× XM430-W350-T ($270) + 2× XL430-W250-T ($45) |
| 10 | $750 | ROBOTIS volume discount |
| 100 | $600 | Academic/volume pricing |
| 1,000 | $400 | OEM negotiation |

### Lead Time

- 1-10 units: In stock at ROBOTIS US — ships within 1 week
- 100+ units: 2-4 weeks via ROBOTIS distribution

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | ROBOTIS is the most established servo manufacturer in robotics |
| Technical: shoulder torque | MEDIUM | XM430 may be insufficient for horizontal hold with full payload. Upgrade path: XM540 ($483) or CubeMars AK60-6 ($299). |
| Integration | LOW | Dynamixel SDK2 + ROS2 drivers are mature. TTL daisy-chain simplifies wiring. |

### Open Questions

1. **Shoulder motor selection** — Test XM430-W350-T with 500g payload at horizontal extension. If it struggles, upgrade shoulder to XM540-W270-T (+$213) or AK60-6 (+$29 with CAN complexity).
2. **Arm protocol bus** — Dynamixel TTL is separate from leg CAN bus. Need USB2Dynamixel adapter on Jetson ($50). Consider Dynamixel-to-CAN bridge for unified bus.
3. **Arm IP rating** — Dynamixel motors are not weatherproof. Arm needs enclosure/sleeve design for rain operation.

### Confidence: **HIGH** (wrist, gripper, elbow); **MEDIUM** (shoulder — need physical testing)

---

## 6. Battery System

### Decision

**48V 20Ah Li-ion NMC pack (13S configuration)** — ~$350-$500 custom, ~$200-$350 off-shelf e-bike

### Why

- **960 Wh** capacity → 816 Wh usable (85% DoD) → **4-5 hours runtime** at 158W average
- **48V nominal** matches AK70-10 motor voltage (48V) directly — no DC-DC conversion for main power bus
- **~5-5.5 kg** — fits within 8 kg body mass budget alongside electronics
- Li-ion NMC: best energy density for weight, widely available in 18650/21700 cells
- 13S7P configuration using Samsung 50E (21700, 5000 mAh) cells: proven chemistry, high cycle life

### Power Budget

| Subsystem | Typical Draw | Peak Draw |
|-----------|-------------|-----------|
| 12x leg actuators (walking) | 120W | 400W |
| 3x arm actuators | 5W | 20W |
| Compute (Jetson at 15W mode) | 15W | 25W |
| Sensors (OAK-D Pro + Mid-360) | 10W | 15W |
| Communications (LTE + WiFi) | 3W | 5W |
| Misc (LEDs, fans, bag servo) | 5W | 10W |
| **Total** | **158W** | **475W** |

### Runtime Estimate

| Scenario | Power | Runtime (816 Wh usable) |
|----------|-------|------------------------|
| Flat park, low litter density | 140W | 5.8 hours |
| Typical mixed terrain | 158W | 5.2 hours |
| Hilly terrain, frequent picking | 200W | 4.1 hours |
| Peak sustained (steep hills) | 250W | 3.3 hours |

### Alternatives Considered

| Alternative | Voltage | Energy | Weight | Price | Verdict |
|-------------|---------|--------|--------|-------|---------|
| **48V 20Ah NMC (off-shelf e-bike)** | 48V | 960 Wh | ~5.5 kg | $200-350 | **Selected for prototype.** Cheap, available, proven. Downside: generic BMS, non-optimal form factor. |
| **Custom 48V 21Ah 18650 pack (13S7P)** | 48V | 1008 Wh | ~5 kg | $350-500 | **Selected for production.** Optimized form factor, custom BMS with CAN telemetry, fits body cavity exactly. |
| **48V 10Ah (half capacity)** | 48V | 480 Wh | ~2.8 kg | $150-200 | 2.5-3 hours runtime — marginal for commercial use but saves 2.5 kg. Consider for weight-limited prototype. |
| **LiFePO4 (48V 20Ah)** | 51.2V | 1024 Wh | ~8 kg | $300-400 | Safer chemistry, better cycle life (3000+ vs 800 cycles), better thermal stability. **3 kg heavier** — pushes mass budget. Consider for hot-climate deployments (Dubai). |
| **24V system (2x 12V in series)** | 24V | 480 Wh per pack | ~3 kg per pack | $100-200 per pack | Requires DC-DC boost for AK70-10 motors. Inefficient. Lower power capability. Not recommended. |

### Cost at Scale

| Volume | Cost | Notes |
|--------|------|-------|
| 1 (off-shelf) | $350 | E-bike battery from Amazon/Aegis |
| 1 (custom) | $500 | CM Batteries or similar |
| 10 | $380 | Custom pack, batch pricing |
| 100 | $280 | Direct cell sourcing + custom BMS |
| 1,000 | $180 | Volume cell pricing + in-house BMS design |

### Lead Time

- Off-shelf e-bike battery: 1-2 weeks (Amazon/distributor)
- Custom pack (CM Batteries): 3-6 weeks for first article
- Volume (100+): 4-8 weeks

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Supply chain | LOW | 18650/21700 cells are commodity items |
| Technical: thermal | MEDIUM | NMC degrades above 45°C. Need temperature-controlled battery bay with ventilation. Consider LiFePO4 for hot-climate markets. |
| Safety: fire risk | MEDIUM | Li-ion NMC can thermal runaway. Require proper BMS with cell balancing, overcurrent protection, thermal cutoff. LiPo-rated fire extinguisher in workshop. |
| Weight: 5.5 kg | LOW | Within body mass budget. If weight is tight, use 48V 15Ah (3/4 capacity) to save ~1.3 kg. |

### Open Questions

1. **NMC vs LiFePO4** — NMC for weight-sensitive prototype, LiFePO4 for production (especially hot-climate markets). LiFePO4 adds ~3 kg but 3-4× cycle life.
2. **Custom BMS with CAN bus telemetry** — Production packs need BMS that reports SOC, cell voltages, temperature to Jetson via CAN bus. Budget $30-50 per BMS in volume.
3. **Battery form factor** — Body cavity is 0.60 × 0.15 × 0.12 m. Battery must fit alongside Jetson, PCB, DC-DC converters. CAD integration needed.
4. **Charging rate** — 5A charge rate = 4 hours for 20Ah. Consider 10A fast charge (2 hours) for commercial deployments.

### Confidence: **HIGH** (chemistry and capacity); **MEDIUM** (form factor integration — needs CAD)

---

## 7. Motor Drivers / Communication Bus

### Decision

**CAN bus via integrated CubeMars drivers** (legs) + **Dynamixel TTL bus** (arm) + **USB-CAN adapter** (Jetson interface)

### Why

CubeMars AK series actuators have **integrated motor driver boards** with CAN bus communication and field-oriented control (FOC). No separate motor controller PCBs needed for legs. This dramatically simplifies the electronics:

- **Legs (12 joints):** CAN bus → each AK motor has built-in driver with position/velocity/torque modes
- **Arm (5 joints):** Dynamixel TTL protocol → daisy-chain wiring, USB2Dynamixel adapter
- **Bag hinge (1 joint):** Standard hobby servo (SG90 or equivalent) on PWM GPIO

### Architecture

```
Jetson Orin Nano
├── USB → Canable 2.0 (CAN adapter) → CAN bus → 12× AK motors (legs)
├── USB → USB2Dynamixel → TTL bus → 5× Dynamixel motors (arm)
└── GPIO PWM → bag frame servo
```

### Alternatives Considered

| Alternative | Price | Pros | Cons | Verdict |
|-------------|-------|------|------|---------|
| **Integrated CubeMars CAN + Dynamixel TTL** | $65-$90 (adapters only) | Simplest — motors handle their own FOC. Proven, minimal custom electronics. | Two different protocols (CAN + TTL). | **Selected** |
| **PCA9685 PWM servo driver** | $5-$15 | 16-channel PWM, cheap, I2C | Only works with hobby servos. Cannot drive CubeMars AK motors. No closed-loop control. | **Rejected.** Not compatible with our actuators. |
| **ODrive S1 per motor** | $149 × 12 = $1,788 | Excellent BLDC controller, CAN+USB | Redundant — AK motors already have integrated drivers. Would add $1,788 in unnecessary hardware. | **Rejected.** Redundant with AK series. |
| **moteus r4.11 per motor** | $79 × 12 = $948 | Open-source, excellent performance | Same as ODrive — redundant with AK integrated drivers. | **Rejected.** |
| **Custom CAN bus controller PCB** | $65-180 per board | Could integrate CAN hub, power distribution, safety MCU | Engineering time (80-120 hours). Viable for production but not prototype. | **Phase 2 (production).** |

### Components Needed

| Component | Purpose | Price | Source |
|-----------|---------|-------|--------|
| Canable 2.0 (USB-CAN) | CAN interface for Jetson | $25-40 | canable.io |
| USB2Dynamixel | TTL interface for arm | $50 | ROBOTIS |
| CAN bus termination resistors | 120Ω at each end | $1 | Any electronics supplier |
| CAN wiring harness | 4-wire (CANH, CANL, +48V, GND) | $20-30 | Custom |
| Bag frame servo (SG90/MG90S) | Bag hinge actuation | $5-10 | Amazon |
| **Total** | | **$100-130** | |

### Cost at Scale

| Volume | Cost | Notes |
|--------|------|-------|
| 1 | $130 | Dev board approach (adapters + wiring) |
| 10 | $180 | Custom PCB for CAN hub + power distribution |
| 100 | $65 | Production PCB (JLCPCB assembly) |
| 1,000 | $35 | Volume PCB production |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| CAN bus reliability (12 nodes) | MEDIUM | Standard CAN bus supports up to 32 nodes. 12 is well within spec. Use 1 Mbps CAN FD if bandwidth is tight. Consider dual CAN for redundancy (6+6 split). |
| Mixed protocol complexity | LOW | CAN and TTL are independent buses. No cross-talk concerns. |

### Open Questions

1. **Dual CAN bus vs single?** — 12 motors at 1 kHz control loop = 12,000 messages/sec. Standard CAN 2.0 at 1 Mbps supports ~7,000 msg/sec. Need CAN FD or dual bus. Decision needed before PCB design.
2. **Safety MCU** — Consider adding STM32 safety co-processor on CAN bus for hardware e-stop, watchdog timer, and motor overcurrent monitoring independent of Jetson. Budget $5-20.

### Confidence: **HIGH**

---

## 8. IMU

### Decision

**Use OAK-D Pro's built-in BNO086** as primary IMU. Add a secondary **ICM-42688-P** on the custom PCB for redundancy.

### Why

- **BNO086** (built into OAK-D Pro): 9-axis IMU with sensor fusion, 400 Hz accel, 1000 Hz gyro, hardware-synced timestamps with camera frames — ideal for cuVSLAM visual-inertial odometry
- **ICM-42688-P** (secondary, on PCB): 6-axis (accel + gyro), high-precision, low-noise, SPI interface, commonly used in quadruped locomotion controllers. Provides IMU data to the gait controller at 1 kHz independently of camera frame rate.

### Why Two IMUs?

| Use Case | Primary IMU | Rate | Consumer |
|----------|-------------|------|----------|
| Visual-inertial SLAM (cuVSLAM) | BNO086 (OAK-D Pro) | 400 Hz | Localization pipeline |
| Gait control / balance | ICM-42688-P (on PCB) | 1 kHz | Locomotion controller |
| EKF state estimation | Both (fused) | 50 Hz output | Nav2, behavior tree |

The gait controller needs IMU data tightly synchronized with motor commands at 1 kHz. The BNO086 on the camera is physically distant from the body center and its data arrives via USB with variable latency. A body-mounted IMU on the main PCB provides deterministic timing for the locomotion control loop.

### Alternatives Considered

| Alternative | Price | Pros | Cons | Verdict |
|-------------|-------|------|------|---------|
| **BNO086 only (from OAK-D Pro)** | $0 (included) | Zero additional cost, excellent fusion | Not body-center mounted, USB latency, 400 Hz max | Sufficient for SLAM, insufficient for high-rate gait control |
| **ICM-42688-P (breakout or on-PCB)** | $5-15 | High precision, 32 kHz max, SPI, body-mountable | 6-axis only (no magnetometer) | **Selected as secondary.** Magnetometer not needed for gait control. |
| **MPU-6050** | $2-5 | Very cheap, ubiquitous | Old design (2012), noisy, no longer recommended for new designs | **Rejected.** Obsolete. |
| **BNO055** | $10-15 | 9-axis with fusion, popular in ROS2 | Outdated vs BNO086, slower fusion, known drift issues | **Rejected.** BNO086 is strictly superior. |
| **VectorNav VN-100** | $500-$800 | Industrial-grade, GPS-aided INS | Massively overkill for 15 kg park robot. Cost prohibitive. | **Rejected.** |
| **Livox Mid-360 built-in IMU** | $0 (included) | Already buying this sensor | Unknown specs/quality, 10 Hz LiDAR rate doesn't help high-rate IMU needs | Bonus input for EKF, not primary |

### Cost at Scale

| Volume | Cost | Notes |
|--------|------|-------|
| 1 (breakout board) | $15 | SparkFun ICM-42688-P breakout |
| 10 (on custom PCB) | $8 | IC + passives on PCB |
| 100 | $5 | Volume IC pricing |
| 1,000 | $3 | Direct from TDK InvenSense |

### Lead Time

- Breakout boards: In stock at SparkFun/Adafruit — ships same week
- IC for PCB: Standard IC, 2-4 weeks via Mouser/DigiKey

### Risk Level: **LOW** across all dimensions

### Confidence: **HIGH**

---

## 9. Communications: LTE + WiFi

### Decision

**SIMCom SIM7600G-H 4G LTE module** ($35-50 module, $55-75 as HAT) + **Jetson's built-in WiFi**

### Why

- **4G LTE** provides cellular connectivity for fleet management telemetry, remote monitoring, and OTA updates anywhere the robot operates
- **Cat-4 LTE**: 150 Mbps download / 50 Mbps upload — ample for telemetry (low bandwidth) and occasional video streaming (medium bandwidth)
- **Global quad-band**: Works in EU, US, UAE, APAC — critical for international pilot deployments
- **GNSS built-in**: GPS/GLONASS positioning for fleet tracking without a separate GPS module
- **Jetson built-in WiFi**: For development, local debugging, and high-bandwidth data transfer at the charging dock

### Alternatives Considered

| Alternative | Price | Pros | Cons | Verdict |
|-------------|-------|------|------|---------|
| **SIM7600G-H (4G LTE Cat-4)** | $35-75 | Global LTE + GNSS, proven, low power, UART/USB | 4G only (no 5G), ~150 Mbps max | **Selected.** 4G is sufficient for telemetry. |
| **Quectel RM500Q-GL (5G)** | $150-200 | 5G Sub-6GHz, future-proof | 3-4× cost, higher power draw, 5G coverage spotty in parks/outdoor areas | **Rejected for MVP.** 5G coverage is sparse in the outdoor areas where the robot operates. Revisit for v2. |
| **LoRa (RAK811)** | $15-25 | Long range (3+ km), very low power | Low bandwidth (<50 kbps), one-way or very slow bidirectional | **Phase 2 addition** for mesh networking between multiple robots. Not a replacement for LTE. |
| **WiFi only** | $0 (built-in) | Free, high bandwidth | Range limited to ~50-100m from AP. Robot operates in parks far from WiFi. | **Development only.** Not viable for field deployment. |

### Antenna

- External 4G omnidirectional antenna ($5-15) mounted as small nub on body (per design spec)
- SMA connector on PCB, short coax cable to antenna
- Consider IP67 antenna ($15-25) for weatherproofing

### Data Plan

- IoT SIM plan: $10-15/month per robot
- Expected data usage: 50-200 MB/month (telemetry + status updates + occasional low-res video)
- Consider Hologram.io or 1NCE for global IoT SIM with roaming

### Cost at Scale

| Volume | Cost | Notes |
|--------|------|-------|
| 1 | $80 | SIM7600G-H HAT + antenna |
| 10 | $65 | Module + custom PCB integration |
| 100 | $45 | Volume pricing on module |
| 1,000 | $30 | OEM pricing |

### Lead Time: In stock, ships within 1 week

### Risk Level: **LOW**

### Open Questions

1. **GNSS accuracy in urban parks** — SIM7600G GNSS may have 2-5m accuracy under tree canopy. Sufficient for fleet tracking but not for localization (SLAM handles precise positioning).
2. **Dual SIM for roaming** — Consider dual-SIM support for international pilots. SIM7600G-H supports this.

### Confidence: **HIGH**

---

## 10. Frame: Materials and Construction

### Decision

**CNC machined 6061-T6 aluminum** for structural frame (body chassis, leg linkages, motor mounts) + **3D printed ASA/PA-CF** for enclosure panels and non-structural parts

### Why

- **6061-T6 aluminum**: Excellent strength-to-weight ratio, easy to machine, corrosion-resistant, proven in every research quadruped (MIT Mini Cheetah, ODRI Solo-12, etc.)
- **CNC machining**: Tight tolerances for motor mounting (actuator alignment critical for gait quality), fast turnaround (5-10 days from Xometry)
- **3D printed enclosure**: Fast iteration on non-structural body panels, weatherproof gasket channels, cable routing features
- **ASA** for outdoor UV resistance; **PA-CF** (carbon-filled nylon) for high-strength brackets

### Alternatives Considered

| Alternative | Price (proto) | Pros | Cons | Verdict |
|-------------|-------------|------|------|---------|
| **CNC aluminum + 3D printed panels** | $1,100 frame + $120 panels = $1,220 | Best balance: aluminum where strength matters, plastic where it doesn't | Two manufacturing processes | **Selected** |
| **Full CNC aluminum** (including enclosure) | $1,500-$2,000 | Maximum precision and durability | Expensive, slow iteration on enclosure design, heavy | For production (sheet metal enclosure). Not prototype. |
| **Full 3D printed (PA12 Nylon SLS/MJF)** | $450-$900 | Cheapest, fastest iteration | Lower strength than aluminum, UV degradation outdoors, flex in motor mounts degrades gait quality | **POC only (v0.1).** Not suitable for field trials. |
| **Carbon fiber plates + aluminum joints** | $1,000-$1,900 | Lightest option, highest stiffness | Expensive, difficult to repair, hard to machine for motor mounts, delamination risk at bolt holes | **Not recommended for MVP.** Consider for production weight optimization. |
| **Aluminum extrusion (80/20 style)** | $200-$400 | Cheap, modular, easy to reconfigure | Bulky, heavy for given stiffness, poor aesthetics, difficult to weatherproof | **Lab test rig only.** Not suitable for outdoor product. |
| **Injection molded (ABS/PC)** | $8,000-$15,000 tooling | Cheapest at 500+ units ($5-10/part) | Massive tooling investment, 6-8 week tooling lead time, no iteration possible | **Production phase (1000+ units).** |

### Cost at Scale

| Volume | Frame | Enclosure | Feet | Total |
|--------|-------|-----------|------|-------|
| 1 | $1,100 (CNC) | $120 (3D print) | $30 | $1,250 |
| 10 | $700 (batch CNC) | $250 (sheet metal) | $25 | $975 |
| 100 | $400 (volume CNC) | $100 (sheet metal) | $12 | $512 |
| 1,000 | $250 (volume + injection mold for panels) | $25 | $8 | $283 |

### Lead Time

- CNC prototype: 5-10 business days (Xometry expedited)
- 3D printed enclosure: 1-3 days (in-house Bambu Lab P1S)
- Sheet metal enclosure (10+ units): 2-3 weeks

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| CNC tolerances | LOW | Xometry guarantees ±0.005" standard. Sufficient for motor mounting. |
| 3D print UV degradation | MEDIUM | Use ASA (UV-resistant) instead of PETG/PLA. Consider paint or UV-resistant coating for exposed panels. |
| Weight | LOW | 6061-T6 aluminum: frame should weigh 1.5-2.5 kg. Well within 8 kg body budget. |
| Iteration cost | LOW | $500-$1,500 per mechanical iteration. Budget 4-5 iterations in development plan. |

### Open Questions

1. **CNC supplier selection** — Xometry for US/EU prototypes. PCBWay for cost-competitive China sourcing ($300-$500 cheaper per frame set). Lead time is 5-7 days vs 7-14 days.
2. **Panel attachment** — Snap-fit vs M3 fasteners vs quarter-turn latches for enclosure panels. Snap-fit is fastest for service access but weaker. Quarter-turn is best for IP65.
3. **Foot pad material** — Design spec calls for "rounded black rubber foot pads, ~5cm diameter." Need to source or mold custom rubber feet. Shore A 60-70 durometer for outdoor grip.

### Confidence: **HIGH**

---

## 11. Gripper: Mechanism and Materials

### Decision

**2-3 finger mechanical gripper with silicone-tipped steel fingers** (per design spec V2.3). Custom CNC steel fingers + molded silicone grip pads.

### Why

- **Mechanical gripper** (not soft/silicone body): Design spec explicitly requires "rigid mechanical — NOT a fully soft/silicone gripper"
- **Silicone fingertips only**: Grip pads (Shore A 20-30) for conforming to irregular litter shapes without crushing
- **2-3 finger radial design**: Handles bottles, cans, flat wrappers, small items
- **Actuated by single Dynamixel XL430**: Open/close via worm gear or linkage mechanism

### Force Requirements

| Litter Type | Typical Mass | Required Grip Force | Notes |
|-------------|-------------|-------------------|-------|
| Cigarette butt | 1-5 g | 1-2 N | Pinch grip, very light |
| Plastic wrapper | 5-20 g | 2-5 N | Flat item, needs friction |
| Aluminum can | 15-30 g | 3-5 N | Cylindrical, easy grip |
| PET bottle (empty) | 25-50 g | 3-8 N | Variable shape, may need compression |
| PET bottle (partial liquid) | 200-500 g | 10-20 N | Heaviest expected item |
| Plastic bag | 5-30 g | 2-5 N | Hardest to grasp — may need suction in Phase 2 |

**XL430-W250-T provides**: ~1.5 Nm stall torque. With a 3cm effective finger radius, this yields ~50 N grip force — far exceeding requirements. The gripper motor is not the bottleneck.

### Design

| Component | Material | Source | Unit Cost |
|-----------|----------|--------|-----------|
| Finger bodies (2-3) | Stainless steel 304 or 6061-T6 aluminum, CNC | Xometry | $30-60 |
| Silicone grip pads | Smooth-On Ecoflex 00-30 or Dragon Skin 30, cast in 3D-printed molds | Amazon/Reynolds Advanced Materials | $5-15 per set |
| Linkage mechanism | Steel pins + 3D-printed PA-CF housing | In-house | $10-20 |
| Gripper servo | Dynamixel XL430-W250-T | ROBOTIS | $45 |
| **Total** | | | **$90-140** |

### Phase 2: Hybrid Gripper (Fingers + Suction)

For flat/deformable items (wrappers, cigarette butts, bags), add a suction cup:
- Small vacuum pump (~$15-30) + silicone suction cup (~$5)
- YOLO class drives mode: rigid items → finger grasp, flat items → suction
- Centroid heuristic for suction placement (no ML needed)
- Additional $50-80 per unit, Phase 2 addition

### Alternatives Considered

| Alternative | Verdict |
|-------------|---------|
| **Soft silicone gripper (Fin Ray)** | Design spec explicitly rejects this: "NOT a fully soft/silicone gripper." Fin Ray has poor grip on smooth cylindrical items (bottles, cans). |
| **Parallel jaw gripper** | Simpler but limited to objects matching jaw width. 2-3 finger radial is more versatile. |
| **Vacuum-only** | Cannot grip 3D objects (bottles, cans). Only useful for flat items. |
| **Robotiq 2F-85** | $5,000-$6,000. Massively overspecced and cost-prohibitive. |

### Cost at Scale

| Volume | Cost |
|--------|------|
| 1 | $140 |
| 10 | $100 |
| 100 | $60 |
| 1,000 | $35 (injection-molded housing, stamped fingers) |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Silicone wear | LOW | Replaceable finger pads, $5-15 per replacement set. Design for snap-fit or screw-on pads. |
| Grip failure on wet items | MEDIUM | Silicone provides good wet grip (better than rubber). Add textured pattern to pads. |
| Grip failure on flat items | HIGH | Fingers alone cannot reliably grasp flat wrappers or film. **Phase 2 suction required.** |

### Open Questions

1. **2 vs 3 fingers?** — 2-finger parallel is simpler to manufacture. 3-finger radial is more versatile. Prototype both and test on real litter.
2. **Finger geometry** — Curved vs straight fingers. Curved fingers better for cylindrical items (bottles, cans). Need iterative testing.
3. **Wrist camera integration** — Design spec calls for "small round camera lens (~2cm) near the gripper wrist." Consider OAK-D Lite ($149) as wrist camera for close-range grasp verification.

### Confidence: **MEDIUM** (mechanism needs iterative physical testing)

---

## 12. Bag System

### Decision

**Folding frame + roll dispenser + gravity-powered drawstring seal** (per design spec V2.3, Section 6)

### Components

| Component | Material | Specification | Est. Cost |
|-----------|----------|--------------|-----------|
| **Bag roll dispenser** | Black anodized aluminum cylinder | 8cm dia × 13cm wide, horizontal mount orthogonal to body | $30-50 (CNC) |
| **Folding bag frame** | Dark grey/black steel or aluminum tubing (1cm dia) | 15cm wide × 22cm deep, hinged at rear body edge | $40-60 (bent tube + welded) |
| **Hinge mechanism** | Steel hinge pins + bearings | Single-axis at rear body edge | $10-15 |
| **Frame servo** | Standard hobby servo (MG996R or equivalent) | ~15 kg·cm stall torque | $10-15 |
| **Clip mechanism** (inner + outer) | Spring-loaded plastic/metal clips | Hold bag edges to roll and frame rim | $10-20 (custom 3D printed + springs) |
| **Drawstring retention** | Small hooks or guides on frame | Hold drawstring while bag drops | $5-10 |
| **Total** | | | **$105-170** |

### Bag Specifications

- **Preferred:** Standard off-the-shelf heavy-duty drawstring trash bags on a roll (30-50L capacity)
- **Material:** Heavy-duty LDPE, strong enough for 5-10 kg of mixed litter
- **Color:** Black or dark grey, semi-transparent for fill-level visibility
- **Customization (if needed):** Reinforced edges for clip system, pre-scored perforations between bags
- **Goal:** Off-the-shelf bags if possible; custom bags acceptable but add supply chain complexity
- **Cost:** ~$0.10-0.30 per bag (off-shelf), ~$0.50-1.00 per bag (custom reinforced)

### Alternatives Considered

| Alternative | Verdict |
|-------------|---------|
| **Built-in bin/box** | Design spec rejects: "NO basket, NO cage, NO mesh, NO rigid walls around the bag." A bin adds permanent weight, requires emptying mechanism. Bag system is lighter and self-disposing. |
| **Compaction bin with linear actuator** | Original BOM included this ($200). Replaced by bag system design. The arm doubles as compressor per spec. |
| **Conveyor belt delivery** | Over-engineered for litter. Bags are simpler, lighter, cheaper. |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Bag tearing | MEDIUM | Use heavy-duty bags (40+ micron LDPE). Test with sharp litter (glass, metal). |
| Clip mechanism reliability | HIGH | Most novel component — no off-the-shelf reference. Requires iterative physical prototyping. Budget 3-5 iterations. |
| Drawstring gravity seal | MEDIUM | Works in theory (like closing a drawstring bag by letting it hang from the string). Test with various fill levels and litter compositions. |
| Wind interference | MEDIUM | Open hanging bag may flutter in wind, affecting litter drop accuracy. Consider partial wind shield or weighted bag bottom. |
| Off-the-shelf bag compatibility | MEDIUM | Standard drawstring bags may not clip reliably. May need custom bags with reinforced clip edges. |

### Open Questions

1. **Clip mechanism design** — This is the highest-risk custom component. No existing product does this. Need dedicated prototyping sprint.
2. **Off-the-shelf vs custom bags** — Test with standard 30L drawstring trash bags first. If clips fail, design custom bags with reinforced edges.
3. **Arm-as-compressor effectiveness** — Can the arm generate enough force pressing into an open hanging bag to meaningfully compress litter? The bag lacks rigid walls to compress against. May need to compress against the frame rim.
4. **Bag change reliability in rain** — Wet bags may stick to clips. Test in wet conditions.

### Confidence: **LOW** (most novel subsystem, requires extensive physical prototyping)

---

## 13. Weatherproofing

### Decision

**Target IP54 for prototype, IP65 for production.** Gasket-sealed enclosure panels with cable glands.

### IP Rating Breakdown

| Rating | Dust | Water | Our Approach |
|--------|------|-------|--------------|
| IP54 (prototype) | Dust-protected (ingress won't harm operation) | Splash-proof from any direction | Silicone gaskets on panel seams, cable glands for wire pass-through, 3D-printed ASA enclosure |
| IP65 (production) | Dust-tight (no ingress) | Water jets from any direction | Sheet metal enclosure, molded gaskets, IP67 connectors, conformal coating on PCBs |

### Weatherproofing Components

| Component | Purpose | Cost (proto) | Cost (100 units) |
|-----------|---------|-------------|------------------|
| Silicone gaskets (custom cut) | Seal panel joints | $10-20 | $3-5 |
| Cable glands (PG7/PG9) | Weatherproof wire pass-through | $10-15 (set) | $3-5 |
| IP67 connectors (Deutsch/M12) | Motor and sensor connections | $30-60 | $10-20 |
| Conformal coating (Arathane 5750) | PCB moisture protection | $10 (spray can) | $2-3/board |
| Drain valves (2-3) | Let condensation escape | $5-10 | $2-3 |
| Camera lens window (polycarbonate) | Protect OAK-D Pro lens | $5-10 | $2-3 |
| **Total** | | **$70-115** | **$22-39** |

### Already-Weatherproof Components

| Component | IP Rating | Notes |
|-----------|-----------|-------|
| Livox Mid-360 | **IP67** | No enclosure needed |
| CubeMars AK70-10/AK60-6 | **IP54** (housing) | Motor housings are sealed. Cable entry points need glands. |
| Battery pack | Typically sealed | Most e-bike packs have built-in BMS enclosure. Verify. |

### Components Needing Protection

| Component | Vulnerability | Solution |
|-----------|--------------|----------|
| Jetson Orin Nano | Not weatherproof | Inside sealed body enclosure |
| OAK-D Pro | No IP rating | Custom 3D-printed enclosure with clear window |
| Dynamixel arm motors | No IP rating | Arm sleeve/enclosure or conformal coating + boot seals |
| Custom PCB | No protection | Conformal coating + inside body enclosure |
| Connectors | Exposed | IP67 M12 or Deutsch connectors |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Water ingress at panel seams | MEDIUM | Double gasket design, sloped panel surfaces to shed water |
| Camera window fogging | MEDIUM | Anti-fog coating, small desiccant pack, ventilation holes with membrane filters |
| Connector corrosion | LOW | IP67 connectors with gold-plated contacts |
| Arm motor water damage | HIGH | Dynamixel motors are not rated for outdoor use. Need boot seals on each joint or full arm sleeve. This is a design challenge. |

### Open Questions

1. **Arm weatherproofing** — Most critical unsolved weatherproofing challenge. Dynamixel motors + linkages + cabling are exposed. Options: (a) bellows boots at each joint, (b) full arm sleeve/cover, (c) accept limited rain operation and dry dock. Needs prototyping.
2. **IP65 lab testing cost** — $500-$2,000 per test at external lab. Schedule during DVT phase. DIY garden-hose test for prototype.
3. **Thermal management with sealed enclosure** — Sealed body traps heat from Jetson + PCB + battery. Need ventilation fans with IP67 membrane filters, or heat pipes to chassis.

### Confidence: **MEDIUM** (body enclosure straightforward; arm weatherproofing is hard)

---

## 14. Charging Dock

### Decision

**Contact-based pogo pin charging** with guided ramp alignment

### Why

- Simplest, cheapest, most reliable approach for v1
- Spring-loaded pogo pins on robot underside + matching flat contacts on dock surface
- Robot drives onto ramp, alignment guides center the contacts
- 48V 5A charging via standard charger electronics

### Design

| Component | Specification | Cost |
|-----------|--------------|------|
| Pogo pin array (robot side) | 4-pin (48V+, GND, signal, signal), spring-loaded, gold-plated | $15-25 |
| Contact plate (dock side) | Gold-plated copper pads on PCB or machined copper | $10-20 |
| 48V 5A charger module | CC/CV charger, off-shelf | $30-50 |
| Dock frame | Sheet metal or 3D printed ramp + alignment rails | $30-50 |
| Alignment guides | V-shaped rails that center the robot over contacts | Included in frame |
| Microcontroller (dock) | ESP32 or similar — charge monitoring, status LED, fleet integration | $5-10 |
| **Total dock BOM** | | **$90-155** |

### Charging Performance

| Parameter | Value |
|-----------|-------|
| Charge voltage | 54.6V (13S Li-ion full) |
| Charge current | 5A standard, 10A fast |
| Charge time (5A, 20Ah) | ~4 hours |
| Charge time (10A, 20Ah) | ~2 hours |
| Connector type | Pogo pins (self-aligning, spring-loaded) |

### Alternatives Considered

| Alternative | Price | Pros | Cons | Verdict |
|-------------|-------|------|------|---------|
| **Pogo pin contact charging** | $90-155 | Simple, cheap, reliable, no precision alignment needed | Contacts can corrode outdoors, debris on contacts | **Selected** |
| **Magnetic connector (MagSafe-style)** | $50-100 | Self-aligning, easy connect/disconnect | Lower current capacity, custom connector design needed | Good alternative. Consider for v2. |
| **WiBotic wireless charging** | $2,000-$5,000 per station | No physical contact, weatherproof, elegant | 10-30× cost. Utterly cost-prohibitive for MVP. | **Phase 3 / production.** |
| **Manual plug-in (barrel jack)** | $10-20 | Cheapest possible | Requires human to plug in. Defeats purpose of autonomous operation. | **Lab/development only.** |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Contact corrosion outdoors | MEDIUM | Gold-plated contacts, periodic cleaning. Dock can have covers/shutters when not charging. |
| Misalignment | LOW | V-shaped ramp guides are forgiving. Pogo pins self-align over ±5mm. |
| Debris on contacts | MEDIUM | Spring-loaded pins self-clean somewhat. Add brush/wiper mechanism if needed. |
| Robot docking accuracy | MEDIUM | Needs consistent approach trajectory. Use AprilTag or reflector on dock for camera-guided final approach. |

### Open Questions

1. **Autonomous docking navigation** — Robot needs to navigate to dock and align precisely. AprilTag on dock + camera-guided final approach. This is a software challenge, not hardware.
2. **Outdoor dock weatherproofing** — Dock electronics need enclosure. Contact plate needs rain cover that opens when robot approaches.
3. **Fast charge (10A)** — Requires higher-rated BMS and charger. May generate heat in battery. Evaluate thermal impact.

### Confidence: **MEDIUM** (hardware straightforward; autonomous docking navigation is the challenge)

---

## 15. Emergency Stop

### Decision

**Dual e-stop: physical button (on robot) + wireless kill switch (hand-held)**

### Why

Safety-critical. Required for any robot operating near people. EU Machinery Regulation 2023/1230 mandates accessible emergency stop.

### Components

| Component | Specification | Cost | Source |
|-----------|--------------|------|--------|
| **Physical e-stop button** | Mushroom-head, twist-to-release, IP65, normally-closed | $10-20 | Amazon/McMaster-Carr |
| **Wireless kill switch** | 433 MHz or 2.4 GHz relay receiver + hand-held transmitter, 100m range | $25-50 | Amazon (industrial remote relay) |
| **Safety relay / contactor** | 48V DC contactor, cuts main power bus | $15-30 | Omron G7L or equivalent |
| **Safety MCU** (optional, recommended) | STM32 or ATmega, independent of Jetson, monitors heartbeat | $5-15 | On PCB |
| **Total** | | **$55-115** |

### E-Stop Behavior

When triggered (either physical or wireless):
1. **Immediately** cut power to all actuator motors (48V bus disconnect via contactor)
2. Robot collapses safely (joints go slack, robot settles under gravity)
3. Jetson + sensors remain powered (12V/5V rail stays active) for logging and status reporting
4. Physical button: twist to release and reset
5. Wireless: requires deliberate re-enable on hand-held transmitter

### Architecture

```
[Physical Button (NC)] ──┐
                         ├──► [Safety Relay] ──► [48V Motor Bus Contactor]
[Wireless Receiver] ─────┘                              │
                                                    [Motor Power ON/OFF]

[Safety MCU] ─── monitors Jetson heartbeat
              ─── if heartbeat lost for 500ms → triggers e-stop
              ─── reports e-stop status to Jetson via UART
```

### Alternatives Considered

| Alternative | Verdict |
|-------------|---------|
| **Software-only e-stop** | **Rejected.** Software can hang/crash. Hardware e-stop is mandatory for safety. |
| **Physical button only (no wireless)** | Acceptable for lab testing. Wireless required for field operation (operator may be 10-50m away). |
| **Bluetooth kill switch** | Shorter range (~30m), potential pairing issues. 433 MHz industrial remote is more reliable and longer range. |

### Risk Level

| Risk | Level | Mitigation |
|------|-------|------------|
| Wireless interference | LOW | Use dedicated 433 MHz channel with unique rolling code. Industrial remote relays are designed for interference-heavy environments. |
| False trigger | LOW | Require deliberate 2-second hold or dual-button activation on wireless transmitter. |
| Failure to trigger | CRITICAL | Test e-stop on every power-on. Safety MCU provides independent watchdog. Physical button is hardwired (no software in path). |

### Confidence: **HIGH**

---

## Summary: Total BOM per Configuration

### Prototype (1 unit)

| # | Subsystem | Component | Cost |
|---|-----------|-----------|------|
| 1 | Compute | Jetson Orin Nano Super Dev Kit | $249 |
| 2 | Camera | OAK-D Pro (fixed-focus) | $399 |
| 3 | LiDAR | Livox Mid-360 | $749 |
| 4 | Leg actuators (12) | 8× AK70-10 + 4× AK60-6 | $5,188 |
| 5 | Arm actuators (5) | 3× XM430-W350 + 2× XL430-W250 | $855 |
| 6 | Battery | 48V 20Ah Li-ion NMC | $350 |
| 7 | Motor drivers / bus | CAN adapter + USB2Dynamixel + wiring | $130 |
| 8 | IMU | ICM-42688-P breakout (secondary) | $15 |
| 9 | Communications | SIM7600G-H HAT + antenna | $80 |
| 10 | Frame + enclosure | CNC aluminum + 3D printed ASA | $1,250 |
| 11 | Gripper | Custom CNC fingers + silicone + XL430 servo | $140 |
| 12 | Bag system | Roll dispenser + frame + hinge servo + clips | $170 |
| 13 | Weatherproofing | Gaskets + cable glands + connectors + coating | $115 |
| 14 | Charging dock | Pogo pins + 48V charger + dock frame | $155 |
| 15 | Emergency stop | Button + wireless kill + contactor + safety MCU | $115 |
| | Wiring/connectors/misc | Cables, heat shrink, fasteners | $150 |
| | | | |
| | **Subtotal** | | **$10,110** |
| | **Contingency (15%)** | | **$1,517** |
| | **TOTAL PROTOTYPE** | | **$11,627** |

### Scaling Cost Table

| Subsystem | 1 unit | 10 units | 100 units | 1,000 units |
|-----------|--------|----------|-----------|-------------|
| Compute | $249 | $249 | $280 | $190 |
| Camera | $399 | $380 | $340 | $280 |
| LiDAR | $749 | $700 | $600 | $500 |
| Leg actuators (12) | $5,188 | $4,800 | $4,360 | $3,500 |
| Arm actuators (5) | $855 | $750 | $600 | $400 |
| Battery + power | $350 | $380 | $280 | $180 |
| Motor drivers / PCB | $130 | $180 | $65 | $35 |
| IMU | $15 | $8 | $5 | $3 |
| Communications | $80 | $65 | $45 | $30 |
| Frame + enclosure | $1,250 | $975 | $512 | $283 |
| Gripper | $140 | $100 | $60 | $35 |
| Bag system | $170 | $130 | $80 | $50 |
| Weatherproofing | $115 | $80 | $40 | $22 |
| Charging dock | $155 | $130 | $90 | $60 |
| Emergency stop | $115 | $90 | $60 | $35 |
| Wiring/connectors | $150 | $120 | $80 | $50 |
| **Subtotal** | **$10,110** | **$9,137** | **$7,497** | **$5,653** |
| Contingency | $1,517 (15%) | $914 (10%) | $750 (10%) | $565 (10%) |
| **Total per unit** | **$11,627** | **$10,051** | **$8,247** | **$6,218** |

### Cost Composition (Prototype)

| Category | Cost | % of BOM |
|----------|------|---------|
| **Actuators (legs + arm)** | $6,043 | 60% |
| **Sensors (camera + LiDAR + IMU)** | $1,163 | 12% |
| **Frame + enclosure** | $1,250 | 12% |
| **Compute** | $249 | 2% |
| **Power (battery)** | $350 | 3% |
| **Everything else** | $1,055 | 10% |

**Actuators are 60% of cost.** This is the single biggest lever for cost reduction at scale. Custom motor design (12-18 month project) could cut actuator cost by 40-50%.

---

## Confidence Summary

| Subsystem | Confidence | Primary Risk |
|-----------|-----------|--------------|
| 1. Compute (Jetson Orin Nano Super) | **HIGH** | None significant |
| 2. Camera (OAK-D Pro) | **HIGH** (MVP) / **MEDIUM** (production) | Sunlight IR degradation; Gemini 345Lg may supersede |
| 3. LiDAR (Livox Mid-360) | **HIGH** | None significant |
| 4. Leg actuators (CubeMars AK series) | **HIGH** (proto) / **MEDIUM** (production) | Knee torque margin tight; thermal under sustained load |
| 5. Arm actuators (Dynamixel) | **HIGH** (most joints) / **MEDIUM** (shoulder) | Shoulder torque with heavy payload |
| 6. Battery (48V 20Ah NMC) | **HIGH** | Form factor integration with body cavity |
| 7. Motor drivers (CAN + TTL) | **HIGH** | CAN bus bandwidth at 1 kHz × 12 motors |
| 8. IMU (BNO086 + ICM-42688) | **HIGH** | None significant |
| 9. Communications (SIM7600G-H) | **HIGH** | None significant |
| 10. Frame (CNC aluminum + 3D print) | **HIGH** | None significant |
| 11. Gripper | **MEDIUM** | Mechanism design needs iterative testing; flat items need Phase 2 suction |
| 12. Bag system | **LOW** | Most novel component. Clip mechanism, gravity seal, wind interference all unproven. |
| 13. Weatherproofing | **MEDIUM** | Arm weatherproofing is unsolved design challenge |
| 14. Charging dock | **MEDIUM** | Hardware simple; autonomous docking navigation is the real challenge |
| 15. Emergency stop | **HIGH** | None significant |

---

## Critical Path: What to Buy First

### Immediate (Week 1) — Long-Lead or Critical Items

| Item | Why First | Cost |
|------|-----------|------|
| 8× CubeMars AK70-10 | Highest cost, need thermal testing | $3,992 |
| 4× CubeMars AK60-6 | Complete leg set | $1,196 |
| 1× Jetson Orin Nano Super Dev Kit | Software development starts immediately | $249 |
| 1× OAK-D Pro (fixed-focus) | Driver development, camera characterization | $399 |

### Week 2 — Frame and Arm

| Item | Why | Cost |
|------|-----|------|
| CNC frame order (Xometry) | 5-10 day lead time | $1,100 |
| 3× Dynamixel XM430-W350-T | Arm development | $810 |
| 2× Dynamixel XL430-W250-T | Wrist + gripper | $90 |

### Week 3 — Electronics and Power

| Item | Why | Cost |
|------|-----|------|
| 48V 20Ah battery | Power testing | $350 |
| Livox Mid-360 | SLAM development | $749 |
| Canable 2.0 + USB2Dynamixel | CAN + TTL interfaces | $90 |
| SIM7600G-H HAT | Telemetry development | $80 |

### Week 4+ — Integration Components

| Item | Why | Cost |
|------|-----|------|
| E-stop button + wireless kit | Safety before first power-on | $55-115 |
| Gripper materials (steel, silicone) | Iterative prototyping | $50-100 |
| Bag system components | Frame + clips + servo | $100-170 |
| Weatherproofing supplies | Gaskets, glands, connectors | $70-115 |

---

## Appendix: Key Contacts for Volume Pricing

| Supplier | Contact | Components | Volume Threshold |
|----------|---------|-----------|-----------------|
| CubeMars (T-Motor) | sales@cubemars.com | AK series actuators | 50+ units for 10-15% discount |
| MyActuator | sales@myactuator.com | RMD-X series (production alternative) | 100+ units |
| ROBOTIS | sales@robotis.us | Dynamixel servos | 50+ units for academic/volume discount |
| Luxonis | sales@luxonis.com | OAK-D cameras | 100+ units for OEM pricing |
| Livox/DJI | enterprise@livoxtech.com | Mid-360 LiDAR | 50+ units |
| Orbbec | Contact via orbbec.com | Gemini 345Lg eval unit | Request eval unit, pricing TBD |
| JLCPCB | jlcpcb.com | PCB fab + assembly | Any volume (auto-scaling pricing) |
| Xometry | xometry.com | CNC machining | Instant quoting, 5-10 day turnaround |
| CM Batteries | cmbatteries.com | Custom battery packs | 10+ units |
| NVIDIA | developer.nvidia.com/buy-jetson | Jetson modules | Via distributors (Arrow, Mouser) |

---

*This document supersedes the component-level sections of hardware-bom-research.md. It incorporates updated specifications from the CW-1 URDF (15 kg mass model), perception pipeline architecture, and latest sensor research. All prices in USD as of February 2026.*
