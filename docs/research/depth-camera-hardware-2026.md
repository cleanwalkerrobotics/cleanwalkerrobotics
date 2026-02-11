# Depth Camera Hardware Research for CleanWalker Robot

**Target Platform:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB RAM)
**Use Case:** Outdoor litter-picking robot — depth perception for object detection, grasping, navigation
**Environment:** Outdoor — sun, shade, rain, varying lighting conditions
**Date:** February 11, 2026 (Updated with CES 2026 releases)

---

## Executive Summary

For the CleanWalker outdoor litter-picking robot on Jetson Orin Nano Super, the best depth camera options are:

1. **NEW: Best for Extreme Outdoor Conditions: Orbbec Gemini 345Lg** — Released at CES 2026, this is purpose-built for outdoor robotics. Reliably outputs high-quality depth data in pitch-black conditions or under intense midday sunlight exceeding 100 klux. IP67-rated, operates -20°C to 65°C, GMSL2 interface, dual-mode depth FOV up to 104° × 87°. One of the few stereo cameras capable of handling extreme outdoor environments. Pricing not yet disclosed.

2. **Best Overall Value: Luxonis OAK-D Pro** — Active stereo with IR dot projector, onboard Myriad X VPU for offloading pre-processing, excellent ROS2 support, reasonable price ($399). The IR projector helps in low-texture scenes (ground, pavement). No IP rating on the dev board but can be housed.

3. **Best for Outdoor Ruggedness (Proven): Stereolabs ZED 2i** — IP66 rated, passive stereo with neural depth engine, 20m range, built-in polarizing filter for outdoor glare. Price $549. Purpose-built for outdoor/industrial use.

4. **Best Embedded Integration: Stereolabs ZED X Mini** — GMSL2 interface (no USB bandwidth issues), IP67 rated, 120fps capability, global shutter (critical for walking robot vibration), designed specifically for Jetson Orin. Requires GMSL2 deserializer board. Price ~$549.

5. **Budget Option: Luxonis OAK-D Lite** — Cheapest at $149, lightweight at 61g, but no IR projector and mono cameras are only 640x480. Acceptable for prototyping.

**Key Insight:** For a walking robot operating outdoors, **passive stereo** (ZED series, Orbbec Gemini 345Lg) or **active stereo with IR** (OAK-D Pro) significantly outperforms structured light or ToF in direct sunlight. Global shutter cameras (ZED X series) handle vibration from walking gaits better than rolling shutter alternatives. The new Orbbec Gemini 345Lg is specifically designed for harsh outdoor environments and should be seriously considered.

---

## 1. Camera Comparison Table

### 1.1 Core Specifications

| Camera | Technology | RGB Resolution | RGB FPS | Depth Resolution | Depth Range | Depth Accuracy |
|--------|-----------|---------------|---------|-----------------|-------------|----------------|
| **Luxonis OAK-D Pro** | Active Stereo + IR dot projector | 4056x3040 (12MP) | 30fps (4K), 60fps (1080p) | 1280x800 (stereo pair) | 0.7–12m (ideal) | ~1-2% at 2m |
| **Luxonis OAK-D Lite** | Passive Stereo | 4208x3120 (13MP) | 30fps | 640x480 (mono pair) | 0.4–8m (ideal) | ~2-3% at 2m |
| **Luxonis OAK-D S2** | Passive Stereo | 4056x3040 (12MP) | 30fps (4K), 60fps (1080p) | 1280x800 (stereo pair) | 0.7–12m (ideal) | ~1-2% at 2m |
| **Intel RealSense D435i** | Active IR Stereo | 1920x1080 | 30fps | 1280x720 | 0.3–10m | ~2% at 2m |
| **Intel RealSense D455** | Active IR Stereo | 1280x800 | 30fps | 1280x720 | 0.5–10m | <2% at 4m |
| **Intel RealSense D405** | Active IR Stereo | 1280x720 | 30fps | 1280x720 | 0.07–0.5m | Sub-mm at 7cm |
| **Stereolabs ZED 2i** | Passive Stereo + Neural Depth | 4416x1242 (2x 2K) | 15fps (4K), 30fps (2K), 60fps (1080p) | Up to 1920x1080 | 0.2–20m | Sub-mm close, ~1% at 3m |
| **Stereolabs ZED Mini** | Passive Stereo + Neural Depth | 2688x1520 (2x 2K) | 30fps (2K), 60fps (1080p) | Up to 1280x720 | 0.1–12m (20m ultra) | ~1% at 3m |
| **Stereolabs ZED X Mini** | Passive Stereo + Neural Depth Gen2 | 1920x1200 (global shutter) | 60fps (1200p), 120fps (720p) | Up to 1920x1200 | 0.1–8m (2.2mm) / 0.15–12m (4mm) | <1% at 2m |
| **Orbbec Femto Bolt** | Time-of-Flight (iToF) | 3840x2160 (4K) | 30fps | 1024x1024 (WFOV) / 640x576 (NFOV) | 0.25–5.46m (mode dependent) | <11mm + 0.1% distance |
| **Azure Kinect DK** | Time-of-Flight (iToF) | 3840x2160 (4K) | 30fps | 1024x1024 (WFOV) / 640x576 (NFOV) | 0.25–5.46m (mode dependent) | <11mm + 0.1% distance |
| **Orbbec Gemini 2** | Active Stereo IR | 1920x1080 (RGB) | 30fps | 1280x800 | 0.12–10m | <2% RMSE at 2m |
| **Orbbec Gemini 2 XL** | Active Stereo IR (outdoor) | 1920x1080 (global shutter) | 30fps | 1280x800 | 0.4–20m | <2% RMSE at 2m |
| **Orbbec Gemini 345Lg** | Active Stereo IR (rugged outdoor) | High-res (details TBD) | 30fps | Dual-mode depth | 0.4–20m (est.) | High accuracy in 100+ klux sunlight |
| **Orbbec Gemini 305** | Active Stereo IR (close-range) | High-res (details TBD) | 30fps | High-res depth | 0.08–5m (est.) | Sub-2% depth accuracy |

### 1.2 Field of View

| Camera | Stereo/Depth FOV (HxVxD) | RGB FOV (HxVxD) |
|--------|-------------------------|-----------------|
| **OAK-D Pro** | 80 x 55 x 89.5 | 66 x 54 x 78 |
| **OAK-D Lite** | 73 x 58 x 86 (est.) | 69 x 54 x 81 |
| **OAK-D S2** | 80 x 55 x 89.5 | 66 x 54 x 78 |
| **RealSense D435i** | 87 x 58 x 95 | 69 x 42 x 77 |
| **RealSense D455** | 87 x 58 x 95 | 90 x 65 x 97 |
| **RealSense D405** | 87 x 58 x — | (from depth sensor) |
| **ZED 2i** | 110 x 70 x 120 | 110 x 70 x 120 |
| **ZED Mini** | 102 x 57 x 118 | 102 x 57 x 118 |
| **ZED X Mini** | 110 x 80 x 120 | 110 x 80 x 120 |
| **Femto Bolt (WFOV)** | 120 x 120 x — | 80 x 51 x — |
| **Femto Bolt (NFOV)** | 75 x 65 x — | 80 x 51 x — |
| **Azure Kinect (WFOV)** | 120 x 120 x — | 90 x 59 x — |
| **Gemini 2** | 91 x 66 x 101 | 71 x 52 x — |
| **Gemini 2 XL** | 91 x 66 x — | 71 x 52 x — |
| **Gemini 345Lg** | 104 x 87 / 91 x 78 (dual-mode) | 137 x 71 x — |
| **Gemini 305** | TBD (close-range optimized) | TBD |

### 1.3 Physical Specifications & Sensors

| Camera | Weight | Dimensions (mm) | Power | Interface | IP Rating | IMU | IR Projector |
|--------|--------|-----------------|-------|-----------|-----------|-----|-------------|
| **OAK-D Pro** | 91g (dev board) | 97 x 29.5 x 22.9 | 7.5W max | USB-C (USB2/3) | None (dev) | BNO086 9-axis | Yes (dot + flood) |
| **OAK-D Lite** | 61g | 91 x 28 x 17.5 | 5W max | USB-C (USB2/3) | None | BMI270 6-axis | No |
| **OAK-D S2** | 300g | 97 x 30 x 23 | 5W max | USB-C (USB2/3) | None | BNO086 9-axis | No |
| **RealSense D435i** | 72g | 90 x 25 x 25 | 2-3.5W | USB-C (USB3) | None | BMI055 6-axis | IR pattern projector |
| **RealSense D455** | 380g | 124 x 26 x 29 | 2-3.5W | USB-C (USB3.1) | None | BMI055 6-axis | IR pattern projector |
| **RealSense D405** | 60g | 42 x 42 x 23 | 2-3.5W | USB-C (USB3) | None | No | Active IR illumination |
| **ZED 2i** | 166g | 175 x 30 x 43 | ~4W | USB-C (USB3.0) | **IP66** | 9-axis + baro + mag | No |
| **ZED Mini** | 60g | 124.5 x 30.5 x 26.5 | ~4W | USB-C (USB3.0) | None | 6-axis | No |
| **ZED X Mini** | ~100g (est.) | 94 x 32 x 37 | ~5W | **GMSL2** | **IP67** | High-perf IMU | No |
| **Femto Bolt** | 348g | 115 x 65 x 40 | 7.7–8.7W | USB-C (USB3.2 Gen1) | None | ICM-40608 6-axis | ToF IR illumination |
| **Azure Kinect DK** | ~440g | 103 x 39 x 126 | ~5.9W | USB-C (USB3) | None | 6-axis (208Hz) | ToF IR illumination |
| **Gemini 2** | ~200g (est.) | ~100 x 70 x 35 (est.) | ~5W | USB-C (USB3.0) | None | IMU included | IR dot projector |
| **Gemini 2 XL** | ~250g (est.) | ~120 x 75 x 40 (est.) | ~6W | USB-C (USB3.0) | None | IMU included | IR dot projector |
| **Gemini 345Lg** | TBD | TBD | TBD | **GMSL2 + FAKRA** | **IP67** | IMU included | IR dot projector |
| **Gemini 305** | 156g | Compact (details TBD) | <5W | USB-C | TBD | IMU included | IR dot projector |

### 1.4 Pricing & Onboard Processing

| Camera | Price (USD) | Onboard Processing | SDK / Jetson Support |
|--------|------------|-------------------|---------------------|
| **OAK-D Pro** | $399 | **Intel Myriad X VPU (4 TOPS, 1.4 AI TOPS)** — runs NN inference, encoding, CV on-device | DepthAI SDK, ROS2 Humble, full Jetson support |
| **OAK-D Lite** | $149 | **Intel Myriad X VPU (4 TOPS, 1.4 AI TOPS)** | DepthAI SDK, ROS2 Humble, full Jetson support |
| **OAK-D S2** | $299 | **Intel Myriad X VPU (4 TOPS, 1.4 AI TOPS)** | DepthAI SDK, ROS2 Humble, full Jetson support |
| **RealSense D435i** | $334 | Intel D4 ASIC (stereo processing only) | librealsense2, ROS2, Jetson supported |
| **RealSense D455** | ~$350 | Intel D4 ASIC (stereo processing only) | librealsense2, ROS2, Jetson supported |
| **RealSense D405** | $259 | Intel D4 ASIC (stereo processing only) | librealsense2, ROS2, Jetson supported |
| **ZED 2i** | $549 | None (host GPU processing via ZED SDK) | ZED SDK 5.x, ROS2 Humble, full Jetson Orin support |
| **ZED Mini** | $449 | None (host GPU processing via ZED SDK) | ZED SDK 5.x, ROS2, Jetson supported |
| **ZED X Mini** | ~$549 | None (host GPU processing via ZED SDK) | ZED SDK 5.x, ROS2, native Jetson Orin GMSL2 |
| **Femto Bolt** | $490 | None | Orbbec SDK, Azure Kinect SDK compat, ROS2 |
| **Azure Kinect DK** | **Discontinued** | None | Azure Kinect SDK (limited future support) |
| **Gemini 2** | ~$199-249 (est.) | Orbbec MX6600 ASIC (depth processing) | OrbbecSDK, ROS2 wrapper available |
| **Gemini 2 XL** | ~$299-349 (est.) | Orbbec MX6600 ASIC (depth processing) | OrbbecSDK, ROS2 wrapper available |
| **Gemini 345Lg** | **TBD (CES 2026 release)** | Orbbec ASIC (depth processing) | OrbbecSDK, ROS2, Jetson Thor optimized |
| **Gemini 305** | **TBD (CES 2026 release)** | Arm Cortex-A55 Edge-AI SoC (on-camera YOLO/TF) | OrbbecSDK, ROS2, <100ms perception latency |

---

## 2. NEW: CES 2026 Releases — Orbbec Gemini 345Lg and 305

### 2.1 Orbbec Gemini 345Lg — Rugged Outdoor Stereo Camera

At CES 2026 (January 2026), Orbbec unveiled the **Gemini 345Lg**, specifically designed for outdoor robotics and harsh environments. This is one of the most significant releases for outdoor depth cameras in 2026.

**Key Features:**
- **Extreme sunlight performance:** Reliably outputs high-quality depth data whether in pitch-black nighttime conditions or under intense midday sunlight exceeding **100 klux**
- **IP67 rating:** Dust-tight and protected against water immersion
- **Wide temperature range:** Operates from -20°C to 65°C (-4°F to 149°F)
- **Dual-mode depth FOV:** Up to 104° × 87° / 91° × 78° (switchable modes)
- **Wide RGB FOV:** 137° × 71°
- **Ultra-wide IR FOV:** 130° × 95°
- **GMSL2 interface with FAKRA connector:** High-bandwidth depth + color data over long cable distances in demanding environments
- **Optimized for NVIDIA Jetson Thor** (and presumably Orin platforms)

**Why This Matters for CleanWalker:**
The Gemini 345Lg is described as "one of the few stereo vision cameras in the robotics field capable of easily handling extreme environments." This directly addresses the outdoor sunlight challenge that has plagued depth cameras. The IP67 rating means no custom enclosure needed, and GMSL2 provides reliable long-distance connectivity.

**Status:** Product pages already available, pricing not yet disclosed (expected to be competitive with ZED X Mini at $500-600 range).

### 2.2 Orbbec Gemini 305 — Close-Range Stereo with On-Camera AI

Also released at CES 2026, the **Gemini 305** is optimized for close-range manipulation tasks (0.08-5m range).

**Key Features:**
- **Sub-2% depth accuracy** in compact 156g package
- **On-camera Arm Cortex-A55 Edge-AI SoC:** Runs YOLO/TensorFlow models locally
- **<100ms perception latency** without requiring external GPU
- **Close-range optimized:** 0.08m minimum distance (perfect for manipulation)
- **Low latency:** Designed for real-time robotic control

**Why This Matters for CleanWalker:**
While the 345Lg is the primary outdoor navigation camera candidate, the 305 could be excellent as a secondary close-range camera for grasp planning. The on-camera AI processing means it can run object detection or segmentation models without consuming Jetson GPU resources.

**Status:** Product pages available, pricing TBD.

### 2.3 Comparison: Gemini 345Lg vs. ZED X Mini vs. OAK-D Pro

| Feature | Gemini 345Lg | ZED X Mini | OAK-D Pro |
|---------|-------------|-----------|-----------|
| **Outdoor sunlight** | **100+ klux rated** | Excellent (passive stereo) | Good (IR degrades in sun) |
| **IP rating** | **IP67** | **IP67** | None (needs housing) |
| **Temperature range** | **-20°C to 65°C** | Standard (0-45°C est.) | Standard (0-45°C) |
| **Interface** | **GMSL2 + FAKRA** | **GMSL2** | USB-C |
| **Shutter type** | Global shutter (est.) | **Global shutter** | Rolling shutter (RGB) |
| **FOV (depth)** | **104° × 87°** | 110° × 80° | 80° × 55° |
| **On-device compute** | Orbbec ASIC | None (host GPU) | **Myriad X VPU** |
| **Price** | TBD (~$500-600 est.) | ~$549 | **$399** |
| **Best for** | Extreme outdoor + rugged | Walking robot (vibration) | Value + onboard AI |

**Recommendation Update:** The Gemini 345Lg should be added to the shortlist as the top candidate for outdoor operation, pending pricing confirmation and real-world testing reports. If priced competitively, it may become the best choice for CleanWalker's outdoor litter detection mission.

---

## 3. Outdoor Depth Sensing Challenges

### 3.1 Sunlight IR Interference

This is the single biggest challenge for outdoor depth cameras. Direct sunlight contains massive amounts of near-infrared (NIR) radiation, which directly interferes with:

- **Structured Light cameras:** Almost completely non-functional outdoors in sunlight. The projected IR pattern is overwhelmed by solar IR.
- **Time-of-Flight (ToF) cameras:** Significantly degraded. The Femto Bolt and Azure Kinect lose range and accuracy in bright conditions. They use 850nm wavelength which overlaps with solar spectrum.
- **Active IR Stereo (RealSense, OAK-D Pro):** Moderately affected. The IR dot projector helps in shade/indoor but provides diminishing returns in direct sun. The stereo matching can fall back to passive mode.
- **Passive Stereo (ZED series, OAK-D without IR):** Least affected by sunlight. Performs better outdoors as they rely on visible light or neural depth estimation. However, they struggle with low-texture surfaces (plain concrete, blank walls).

**Ranking for outdoor sunlight performance (best to worst):**
1. **Orbbec Gemini 345Lg** — Rated for 100+ klux direct sunlight, purpose-built for extreme outdoor
2. Passive stereo with neural depth (ZED 2i, ZED X Mini, Gemini 2 XL) — sunlight is beneficial
3. Active stereo with IR fallback (OAK-D Pro, Gemini 2/305) — graceful degradation
4. Pure passive stereo (OAK-D Lite, OAK-D S2) — good but texture-dependent
5. Active IR stereo (RealSense D4xx) — moderate outdoor degradation
6. Time-of-Flight (Femto Bolt, Azure Kinect) — severely degraded outdoors

### 3.2 Rain and Moisture

- Water droplets on the lens cause diffraction and blur depth readings
- Rain creates IR scatter (affects ToF and active stereo)
- IP-rated cameras: ZED 2i (IP66), ZED X Mini (IP67) are rated for rain exposure
- Non-IP cameras need custom weatherproof enclosures
- Heated lens options (available on some ZED models) prevent condensation

### 3.3 Fog and Low Visibility

- All depth technologies degrade in fog due to backscatter
- Stereo cameras handle fog slightly better than ToF/structured light
- LiDAR outperforms depth cameras in fog (especially 905nm systems)
- Multi-modal fusion (depth camera + LiDAR) provides best robustness

### 3.4 Glare and Reflections

- Wet surfaces create specular reflections that confuse all depth sensors
- ZED 2i includes a built-in CPL (circular polarizing) filter option to reduce glare
- Global shutter cameras (ZED X series) avoid rolling shutter artifacts from rapid light changes

---

## 4. Monocular RGB + Learned Depth vs. Hardware Depth Cameras

### 4.1 Monocular Depth Estimation (MDE) — State of the Art

**Depth Anything V2** (NeurIPS 2024) is the current state-of-the-art:
- Achieves AbsRel of 0.129 on ETH3D (14.6% improvement over prior work)
- Real-time inference possible with smaller model variants
- Works from a single RGB image — no special hardware needed

**Strengths of MDE:**
- Zero additional hardware cost (use existing RGB camera)
- No IR interference issues outdoors
- Works at any range (limited by training data, not physics)
- Handles transparent/reflective objects better than structured light
- Can complement stereo by handling textureless regions

**Weaknesses of MDE:**
- **Not metrically accurate** — relative depth is good, absolute distances have high error
- Thin structures and edges are problematic
- Generalization to unseen domains requires fine-tuning
- Latency: running inference on Jetson consumes GPU resources needed for other tasks
- Not suitable for precise grasping or obstacle avoidance requiring <5cm accuracy

### 4.2 When to Use Each Approach

| Scenario | Best Approach |
|----------|--------------|
| Precise grasping of litter (need mm accuracy) | Hardware depth camera (stereo or ToF) |
| Long-range navigation (>10m) | Monocular depth or passive stereo (ZED) |
| Outdoor sunlight operation | Passive stereo or MDE (avoid ToF) |
| Indoor/controlled lighting | Any hardware depth (ToF gives best accuracy) |
| Low-texture surfaces (concrete, walls) | Active stereo with IR projector (OAK-D Pro) |
| Budget-constrained prototype | MDE with standard webcam |
| High-speed obstacle avoidance | Hardware depth (lower, consistent latency) |
| Walking robot with vibration | Global shutter stereo (ZED X series) |

### 4.3 Recommended Hybrid Approach for CleanWalker

Use **hardware depth camera as primary** with **MDE as supplementary**:
1. Primary: Hardware stereo depth for 0.2-5m range (litter detection + grasping zone)
2. Secondary: Depth Anything V2 on Jetson for longer-range scene understanding
3. Fusion: Combine hardware depth (metrically accurate) with MDE (dense, long-range) for robust depth maps

---

## 5. Multi-Camera Setup Recommendations

### 5.1 Proposed CleanWalker Camera Layout

```
[Front-facing depth camera]  — Primary: object detection + depth for grasping
         |
[Downward-angled camera]     — Secondary: ground plane detection, step planning
         |
[Optional rear camera]       — Navigation safety, reversing
```

### 5.2 Front Depth Camera (Primary)

**Purpose:** Detect litter objects, estimate depth for grasp planning, obstacle avoidance in walking direction.

**Best Options:**
1. **Orbbec Gemini 345Lg** (TBD, est. $500-600) — Purpose-built for extreme outdoor, 100+ klux sunlight rating, IP67, GMSL2, wide FOV (104° × 87°), -20°C to 65°C operation. **Top choice if priced competitively.**
2. **Luxonis OAK-D Pro** ($399) — IR projector helps with ground texture, onboard VPU offloads some processing, 80-degree HFOV is adequate, best value
3. **Stereolabs ZED 2i** ($549) — Wider 110-degree HFOV, IP66 for outdoor, neural depth, but no onboard processing
4. **Stereolabs ZED X Mini** (~$549) — Best for vibration (global shutter), IP67, GMSL2 for clean integration

### 5.3 Navigation / Ground Plane Camera

**Purpose:** Terrain assessment, step planning for walking gait, curb/step detection.

**Best Options:**
1. **Downward-tilted version of primary camera** (if single-camera budget)
2. **Intel RealSense D405** ($259) — Short-range (7-50cm), perfect for close ground inspection, very compact
3. **Additional OAK-D Lite** ($149) — Budget option for secondary camera

### 5.4 Multi-Camera Bandwidth Considerations on Jetson Orin Nano

- **USB3:** Max ~5 Gbps shared. Two USB depth cameras can saturate the bus.
- **GMSL2:** Dedicated per-camera bandwidth, no sharing. ZED X Mini is ideal for multi-camera.
- **Recommendation:** If using 2+ cameras, prefer GMSL2 (ZED X) or use OAK-D cameras with on-device processing to reduce USB bandwidth.

---

## 6. LiDAR Options for Ground Plane Detection

### 6.1 2D LiDAR (Planar Scanning)

| LiDAR | Range | Scan Rate | Sample Rate | Price | Weight | Notes |
|-------|-------|-----------|-------------|-------|--------|-------|
| **RPLIDAR A1** | 12m | 5.5Hz (config to 10Hz) | 8,000 pts/s | $99 | ~170g | Budget option, indoor-focused |
| **RPLIDAR A2** | 12m | 10Hz typical | 8,000 pts/s | $229 | ~190g | Industrial grade, spinning |
| **RPLIDAR A3** | 25m | 10-15Hz | 16,000 pts/s | $599 | ~190g | **Outdoor mode**, extended range |

**Use case for CleanWalker:** Tilted 2D LiDAR can detect ground plane changes, curbs, steps. RPLIDAR A3 is the only one with explicit outdoor mode.

### 6.2 3D LiDAR

| LiDAR | Type | Range | FOV | Points/sec | Price | Weight | IP Rating |
|-------|------|-------|-----|-----------|-------|--------|-----------|
| **Livox Mid-360** | Solid-state (rotating mirror) | 70m (40m @ 10%) | **360 x 59** | 200,000 | $749 | 265g | IP67 |
| **Ouster OS0-32** | Digital spinning | 35m (@ 10%) | 360 x 90 | ~655,000 | ~$2,500 | 447g | IP68/IP69K |
| **Ouster OS1-32** | Digital spinning | 90m (@ 10%) | 360 x 42.4 | ~655,000 | ~$3,000 | ~447g | IP68/IP69K |

**Analysis for CleanWalker:**

- **Livox Mid-360** ($749) is the standout choice:
  - 360-degree x 59-degree FOV covers navigation + ground plane
  - 70m range far exceeds needs
  - 265g is light enough for a walking robot
  - IP67 rated for outdoor use
  - Resistant to 100 kilolux direct sunlight
  - 200k points/sec is sufficient for ground plane + obstacle detection
  - Used on many robot dog platforms already

- **Ouster sensors** are excellent (IP68/IP69K, pressure-washable) but expensive ($2,500+) and heavy (447g)

- **RPLIDAR A3** ($599) is budget-friendly for 2D ground plane scanning only

**Recommended:** Livox Mid-360 for navigation + ground plane, paired with a front depth camera for object detection + grasping.

---

## 7. Camera Mounting Considerations for a Walking Robot

### 7.1 Vibration and Shock

Walking robots produce significant vibration from leg impacts:
- **Gait frequency:** Typically 1-3 Hz for quadrupeds
- **Impact shock:** Can exceed 5-10g peak accelerations at foot strike
- **Camera effects:** Motion blur (rolling shutter), depth noise spikes, IMU saturation

**Mitigation strategies:**
1. **Global shutter cameras** (ZED X series) eliminate rolling shutter artifacts
2. **Fixed-focus lenses** recommended over auto-focus (AF hunts during vibration). The OAK-D Pro fixed-focus variant is specifically recommended for "heavy-vibration applications, such as drones, lawnmowers, or vehicles"
3. **Vibration damping mounts** — rubber/silicone isolators between camera and body
4. **High frame rate capture** (60+ fps) reduces motion blur per frame
5. **IMU-aided deblurring** — use accelerometer data to compensate for vibration in depth processing

### 7.2 Mounting Position

Research on quadruped robots (Boston Dynamics Spot, ANYmal) shows:
- **Head/front mount:** Primary forward-facing depth for navigation + detection
- **Chin/downward mount:** Ground plane and foothold assessment
- **Hierarchical vision:** Some robots use both a "Front Camera" and a "Foothold Camera" simultaneously

**For CleanWalker:**
- Mount primary depth camera on a **vibration-isolated head platform** tilted 15-20 degrees downward
- Field of view should cover 1-5m ahead and the ground immediately in front
- Protect camera with transparent polycarbonate shield or use IP-rated camera

### 7.3 Environmental Protection

For cameras without IP ratings, a custom enclosure needs:
- Transparent optical window (anti-reflective coated polycarbonate or glass)
- Drainage holes (bottom-facing) to prevent water pooling
- Ventilation to prevent fogging (or heated window)
- Lens wiper/air blast system for rain operation
- Cable strain relief and waterproof connectors

---

## 8. What Do Industry Robots Use?

### 8.1 Boston Dynamics Spot

- **5 stereo camera pairs** (custom) — two front, one rear, one each side
- 360-degree perception coverage
- All cameras provide both color RGB and depth
- Custom-built cameras, not commercially available sensors
- Optional payloads: SpotCAM+ (PTZ), thermal camera

### 8.2 ANYbotics ANYmal

- **6 Intel RealSense depth cameras** — surrounding the body for close-range obstacle avoidance
- **360-degree LiDAR** — for long-range mapping and SLAM (centimeter-accurate, 4km mapping range)
- **2 optical cameras** — for tele-operation
- Uses "Trekker" AI software that fuses depth camera + LiDAR + terrain map
- Total: 6 depth + 1 LiDAR + 2 RGB = comprehensive multi-modal perception

### 8.3 Key Takeaway

Professional quadruped robots use **multi-camera + LiDAR fusion**. A minimum viable sensing suite for CleanWalker would be:
- 1x front depth camera (litter detection + close navigation)
- 1x LiDAR (360-degree navigation + ground plane)
- Optional: 1x downward camera (foothold planning)

---

## 9. Intel RealSense Discontinuation — What Changed

### 9.1 Timeline

- **2021:** Intel announced winding down the RealSense business unit
- **2022:** LiDAR (L515), tracking (T265), and facial auth products discontinued
- **2023:** Business unit officially wound down
- **2025:** Stereo cameras (D435i, D455, D405) continue to be sold through existing distribution

### 9.2 Current Status (2026)

- D400 series stereo cameras **remain available** through Intel's distribution partners and RealSenseAI (the successor brand)
- No new product development expected
- Driver/firmware updates are minimal (community-driven via librealsense GitHub)
- Long-term supply chain risk: eventually stock will run out

### 9.3 Recommended Replacements

| RealSense Camera | Best Replacement | Why |
|-----------------|-----------------|-----|
| D435i | **Luxonis OAK-D Pro** | Similar stereo + IR, adds VPU, better software momentum |
| D435i | **Luxonis OAK-D S2** | Direct analog without IR projector, has IMU, cheaper |
| D455 | **Stereolabs ZED 2i** | Better outdoor performance, IP66, wider FOV, neural depth |
| D405 (short range) | **Orbbec Femto Bolt** | ToF gives excellent close-range accuracy (replaces Azure Kinect too) |
| L515 (LiDAR) | **Livox Mid-360** | 3D LiDAR for mapping/navigation |

### 9.4 Other Emerging Alternatives

- **Orbbec Gemini 2 series:** Active stereo, RealSense-compatible API, lower cost
- **Youyeetoo FHL-D435i:** Hardware clone of D435i using same Intel D4 ASIC
- **Luxonis OAK 4 D Pro:** Next-generation with improved VPU (RVC4), coming soon

---

## 10. Final Recommendation for CleanWalker (Updated February 2026)

### 10.1 NEW Top Recommendation: Orbbec Gemini 345Lg (Pricing TBD)

**If pricing is competitive ($500-650 range), this becomes the #1 choice.**

**Rationale:**
- **Purpose-built for extreme outdoor robotics** — Rated for 100+ klux direct sunlight AND pitch-black operation
- **IP67 rating** — Dust-tight, waterproof, no custom enclosure needed
- **Wide temperature range** — -20°C to 65°C operation (handles extreme weather)
- **GMSL2 + FAKRA interface** — High-bandwidth, long-distance connectivity, no USB bandwidth issues
- **Ultra-wide FOV** — 104° × 87° depth, 137° × 71° RGB, 130° × 95° IR
- **Global shutter (likely)** — Based on outdoor robotics focus, should handle vibration well
- **Jetson-optimized** — Specifically designed for NVIDIA Jetson platform

**Limitations to address:**
- **Pricing unknown** — Product just announced at CES 2026, no public pricing yet
- **Limited field testing** — Brand new product, real-world performance data not yet available
- **Requires GMSL2 deserializer** — Additional hardware/complexity vs. USB cameras
- **Wait for availability** — Product pages exist but shipping timeline unclear

**Recommendation:** Contact Orbbec for early access pricing and evaluation unit. If priced under $650, this should be the primary camera for CleanWalker.

### 10.2 Current Best Available: Luxonis OAK-D Pro ($399)

**Rationale:**
- Active stereo with IR dot projector handles outdoor ground textures
- Onboard Myriad X VPU can run lightweight NN (e.g., object detection pre-filtering) to reduce Jetson GPU load
- Fixed-focus variant specifically designed for vibration environments
- 12MP RGB is excellent for litter detection model input
- 80-degree HFOV is adequate for forward-facing litter detection
- 7.5cm baseline provides good depth accuracy in the 0.7-5m working range
- Full ROS2 and Jetson Orin Nano support
- Well-maintained open-source SDK (DepthAI)
- Lower cost than ZED alternatives

**Limitations to address:**
- No IP rating — needs custom weatherproof enclosure
- Rolling shutter on RGB (use fixed-focus, high-FPS to mitigate)
- USB bandwidth shared if multi-camera

### 10.3 Alternative Recommendation: Stereolabs ZED X Mini (~$549)

**If budget allows and GMSL2 is feasible:**
- Global shutter eliminates vibration artifacts (critical advantage for walking robot)
- IP67 means no custom enclosure needed for rain
- GMSL2 provides dedicated bandwidth per camera (supports multi-camera cleanly)
- Neural depth engine runs on Jetson GPU but is highly optimized
- 120fps max capture rate handles fast motion
- Designed specifically for Jetson Orin platform

**Limitations:**
- Requires GMSL2 deserializer board (additional cost/complexity)
- No onboard processing — all computation on Jetson GPU
- Passive stereo only — may struggle on perfectly uniform surfaces

### 10.4 Recommended Full Sensor Suite (Best Value, Available Now)

| Sensor | Purpose | Price | Priority |
|--------|---------|-------|----------|
| **Luxonis OAK-D Pro** (fixed-focus) | Front depth + litter detection | $399 | Must-have |
| **Livox Mid-360** | 360-degree navigation + ground plane | $749 | Must-have |
| **Luxonis OAK-D Lite** | Downward foothold camera | $149 | Nice-to-have |
| **Total (must-have)** | | **$1,148** | |
| **Total (full suite)** | | **$1,297** | |

### 10.5 Alternative Full Sensor Suite (Premium — ZED X)

| Sensor | Purpose | Price | Priority |
|--------|---------|-------|----------|
| **Stereolabs ZED X Mini** | Front depth + litter detection (IP67, global shutter) | $549 | Must-have |
| **Livox Mid-360** | 360-degree navigation + ground plane | $749 | Must-have |
| **ZED Carrier Mini / GMSL2 board** | Interface for ZED X Mini | ~$200 | Must-have |
| **Total** | | **~$1,498** | |

### 10.6 Future-Ready Full Sensor Suite (Orbbec Gemini 345Lg)

**Recommended if Gemini 345Lg pricing is under $650:**

| Sensor | Purpose | Price | Priority |
|--------|---------|-------|----------|
| **Orbbec Gemini 345Lg** | Front depth + litter detection (IP67, extreme outdoor) | TBD (~$550 est.) | Must-have |
| **Livox Mid-360** | 360-degree navigation + ground plane | $749 | Must-have |
| **GMSL2 deserializer board** | Interface for Gemini 345Lg | ~$150-200 | Must-have |
| **Orbbec Gemini 305** (optional) | Close-range manipulation camera with on-camera AI | TBD (~$300 est.) | Nice-to-have |
| **Total (must-have)** | | **~$1,450-1,500** | |
| **Total (full suite)** | | **~$1,750-1,800** | |

**Action Items:**
1. **Contact Orbbec immediately** for Gemini 345Lg and 305 early access/pricing
2. **Request evaluation units** for field testing in outdoor litter detection scenario
3. **Fallback plan:** Proceed with OAK-D Pro ($399) if Gemini 345Lg pricing/availability is unfavorable
4. **Monitor:** ZED X Mini remains strong alternative if GMSL2 integration is already planned

---

## Sources

- [Luxonis OAK-D Pro Product Page](https://shop.luxonis.com/products/oak-d-pro)
- [Luxonis OAK-D Lite Product Page](https://shop.luxonis.com/products/oak-d-lite-1)
- [Luxonis OAK-D S2 Product Page](https://shop.luxonis.com/products/oak-d-s2)
- [Luxonis OAK-D Pro Documentation](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)
- [Intel RealSense D435i Specifications](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
- [Intel RealSense D455 Specifications](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html)
- [Intel RealSense D405 Specifications](https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)
- [Stereolabs ZED 2i Product Page](https://www.stereolabs.com/store/products/zed-2i)
- [Stereolabs ZED Mini Product Page](https://www.stereolabs.com/store/products/zed-mini)
- [Stereolabs ZED X Mini Product Page](https://www.stereolabs.com/store/products/zed-x-mini-stereo-camera)
- [Stereolabs ZED X Product Page](https://www.stereolabs.com/products/zed-x)
- [Orbbec Femto Bolt Product Page](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- [Orbbec Femto Bolt Hardware Specifications](https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Femto%20Bolt%20Hardware%20Specifications)
- [Azure Kinect DK Hardware Specifications](https://learn.microsoft.com/en-us/previous-versions/azure/kinect-dk/hardware-specification)
- [Intel RealSense Discontinuation Discussion](https://github.com/IntelRealSense/librealsense/issues/9653)
- [Intel RealSense Stereo Cameras Continuing](https://spectrum.ieee.org/intel-realsense)
- [Luxonis vs RealSense Comparison](https://www.generationrobots.com/blog/en/luxonis-vs-realsense-which-depth-camera-should-i-choose/)
- [Depth Anything V2 (NeurIPS 2024)](https://github.com/DepthAnything/Depth-Anything-V2)
- [Assessing Depth Anything V2 as LiDAR Alternative](https://www.researchgate.net/publication/397955860_Assessing_Depth_Anything_V2_monocular_depth_estimation_as_a_LiDAR_alternative_in_robotics)
- [Boston Dynamics Spot Specifications](https://support.bostondynamics.com/s/article/About-the-Spot-Robot-72005)
- [ANYbotics ANYmal Sensors](https://www.anybotics.com/robotics/anymal/)
- [ANYbotics Uses RealSense](https://www.realsenseai.com/case-studies/realsense-helps-autonomous-mobile-robots-map-facilities-avoid-obstacles-climb-stairs-and-navigate-to-dynamic-docking-stations/)
- [Livox Mid-360 Specifications](https://www.livoxtech.com/mid-360)
- [Ouster OS0 Specifications](https://ouster.com/products/hardware/os0-lidar-sensor)
- [Ouster OS1 Specifications](https://ouster.com/products/hardware/os1-lidar-sensor)
- [RPLIDAR Comparison (Slamtec)](https://tannatechbiz.com/blog/post/comparative-analysis-of-slamtec-lidar-sensors-rplidar-a1-vs-a2-vs-a3)
- [Depth Camera Outdoor Challenges (PatSnap)](https://eureka.patsnap.com/article/depth-camera-accuracy-issues-ambient-light-and-reflective-surfaces)
- [Outdoor Depth Camera Technology (In Compliance)](https://incompliancemag.com/new-technology-for-depth-cameras-that-can-operate-outdoors/)
- [DepthAI ROS2 Driver](https://docs.luxonis.com/software-v3/depthai/ros/driver/)
- [ZED SDK for Jetson Orin](https://support.stereolabs.com/hc/en-us/articles/15331222747799-Get-started-with-Jetson-Orin-Nano-NX-devkit-and-ZED-X)
- [ZED 2i IP66 Rating Details](https://support.stereolabs.com/hc/en-us/articles/10462132852631-What-is-the-IP-rating-of-the-ZED-2i-and-ZED-X-and-what-does-it-mean)
