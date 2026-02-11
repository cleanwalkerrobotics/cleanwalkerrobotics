# Unified Perception Stack: Depth Estimation, Grasp Planning & Camera Hardware

**Target Platform:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB shared RAM, 1024 CUDA cores)
**Use Case:** Autonomous outdoor litter-picking quadruped robot
**Date:** February 11, 2026
**Status:** Research Complete — Consolidated from individual component research

---

## Executive Summary

This document consolidates our depth estimation, grasp planning, and camera hardware research into a single unified perception stack recommendation for the CleanWalker robot. The three subsystems are deeply interdependent — camera choice constrains depth methods, depth output format constrains grasp planners, and Jetson memory budget constrains everything.

### Recommended Stack (TL;DR)

| Component | Primary Choice | Backup | Rationale |
|-----------|---------------|--------|-----------|
| **Depth Camera** | Luxonis OAK-D Pro ($399) | Stereolabs ZED X Mini ($549) | IR projector for outdoor ground texture; VPU offloads processing; vibration-tolerant fixed-focus variant available |
| **Navigation LiDAR** | Livox Mid-360 ($749) | RPLIDAR A3 ($599) | 360° × 59° FOV, IP67, 70m range, 265g, used on many quadrupeds |
| **Monocular Depth** | Depth Anything V2 Small (TensorRT) | — | 25-42 FPS on Jetson, 24.8M params, Apache-2.0, relative depth for scene understanding |
| **Stereo Depth** | NVIDIA Isaac ROS ESS | SGM fallback | 17M params, native TensorRT, production-ready with confidence maps, real-time on Orin Nano |
| **Grasp Planner (MVP)** | GR-ConvNet v2 | Centroid heuristic | 1.9M params, 20ms (5-10ms TRT), 95.4% success, planar grasps for rigid litter |
| **Grasp Planner (6-DOF)** | VGN | — | 10ms inference, BSD-3-Clause, TSDF input, for Phase 2+ |
| **Gripper** | Hybrid Fin Ray + Suction | Parallel jaw only | Category-adaptive: fingers for rigid, suction for flat/deformable |

### Memory Budget on Jetson Orin Nano Super (8GB)

| Component | Estimated Memory | Notes |
|-----------|-----------------|-------|
| OS + System | ~1.5 GB | Ubuntu + ROS2 + framework overhead |
| YOLO Litter Detection | ~0.5-1.0 GB | YOLO26s or YOLO11s with TensorRT INT8 |
| Depth Estimation | ~0.6-0.7 GB | DA V2 Small (TRT) or ESS |
| Grasp Planning | ~0.3-0.5 GB | GR-ConvNet v2 (TRT) |
| Point Cloud / Buffers | ~0.5 GB | ROI extraction, TSDF if needed |
| ROS2 / Application | ~0.5 GB | Message passing, state management |
| **Total** | **~4.4-4.7 GB** | **~3.3-3.6 GB headroom** |

The stack fits comfortably. Even adding VGN (Phase 2) would only add ~0.3 GB.

---

## Part A: Depth Estimation for Grasp Planning

### A.1 Monocular Depth Models — Comparison Table

| Model | Type | Params | FPS on Jetson | Metric Depth? | Accuracy (KITTI AbsRel) | License | Jetson-Ready? |
|-------|------|--------|---------------|---------------|------------------------|---------|---------------|
| **Depth Anything V2 Small** | Mono | **24.8M** | **25-42** (TRT) | Relative (metric w/ finetune) | 0.078 | **Apache-2.0** | **Yes (tested)** |
| DA V2 Base | Mono | 97.5M | — | Relative | 0.078 | CC-BY-NC-4.0 | No (OOM) |
| DA V2 Large | Mono | 335.3M | — | Relative | 0.074 | CC-BY-NC-4.0 | No (OOM) |
| UniDepthV2-ViT-S | Mono | ~30M est. | Unknown | **Metric (zero-shot)** | ~0.05 est. | CC-BY-NC-4.0 | Untested |
| Metric3D v2 ViT-S | Mono | ~40M est. | Unknown | **Metric + Normals** | — | **BSD-2-Clause** | Untested |
| ZoeDepth (Swin2-T) | Mono | 42M | Unknown | Metric (domain-aware) | — | **MIT** | Untested, unmaintained |

### A.2 Stereo Depth Models — Comparison Table

| Model | Params | Memory | FPS on Jetson | Accuracy (KITTI D1-all) | Confidence Map? | License | Jetson-Ready? |
|-------|--------|--------|---------------|------------------------|----------------|---------|---------------|
| **NVIDIA Isaac ROS ESS** | **17M** | Optimized | **Est. 20-40+** | — (8.27% BP2 Middlebury) | **Yes** | NVIDIA prop. (free for Jetson) | **Yes (production)** |
| FoundationStereo | 60.6M | — | <1 | 5.31% BP2 | No | NVIDIA prop. | Yes (too slow) |
| RT-IGEV++ | ~20M est. | 0.66 GB | Est. 3-8 | 1.51% | No | **MIT** | Likely (untested) |
| RAFT-Stereo | 11.23M | <1 GB | Est. 3-8 | 1.96% | No | **MIT** | Likely (untested) |
| CREStereo | ~12M est. | — | Unknown | Top-tier | No | **Apache-2.0** | Possible (ONNX exists) |
| SGM (classical) | N/A | Minimal | 100+ | Lower | No | Apache-2.0 | **Yes** |

### A.3 Key Findings

1. **Depth Anything V2 Small is the clear winner for monocular depth on Jetson.** 24.8M params, Apache-2.0 license, 25-42 FPS with TensorRT at 308-364px resolution, well-tested on Jetson hardware. Provides relative depth (ordering), with metric fine-tuned variants available for KITTI (outdoor) and NYU (indoor).

2. **NVIDIA Isaac ROS ESS is the production stereo depth solution.** 17M params, natively optimized for Jetson with TensorRT, integrated into ROS2 pipeline, provides confidence maps for filtering unreliable depth. Estimated 20-40+ FPS on Orin Nano.

3. **Metric depth from monocular is still unreliable for grasping.** UniDepthV2 and Metric3D v2 offer zero-shot metric depth, but neither has been tested on Jetson, both are too large in their best variants, and UniDepthV2 has a non-commercial license. For grasp planning, use stereo camera hardware for metric depth.

4. **For litter picking specifically:** The robot stops, then picks. This gives 1-2 seconds of "think time." Even 3-8 FPS stereo depth is sufficient for grasp planning on static ground litter. Real-time (20+ FPS) depth is needed only for navigation/obstacle avoidance during walking.

5. **Memory is the binding constraint.** Only models under ~100M params fit alongside detection + grasp planning in 8GB. This rules out all Large/Giant variants of everything.

### A.4 Recommended Depth Strategy

**Primary (navigation + scene understanding):** Depth Anything V2 Small on monocular RGB from the depth camera, running at 25+ FPS via TensorRT. Provides dense relative depth for obstacle avoidance during locomotion.

**Primary (grasp planning):** Hardware stereo depth from the OAK-D Pro camera. Either:
- Isaac ROS ESS (if using stereo pair input) — best accuracy + confidence maps
- On-device stereo from OAK-D Pro's Myriad X VPU — zero Jetson GPU overhead, lower quality but free in terms of Jetson compute

**Hybrid fusion (advanced):** Combine hardware stereo (metrically accurate, 0.7-5m) with DA V2 Small (dense, longer range) for robust depth at all ranges.

### A.5 Inference Speed Detail — Depth Anything V2 Small on Jetson

| Input Resolution | Inference Time | FPS | Memory Usage |
|-----------------|----------------|-----|--------------|
| 308×308 | 23.5 ms | ~42.6 | 626 MB |
| 364×364 | 39.2 ms | ~25.5 | 640 MB |
| 406×406 | 47.7 ms | ~20.9 | 649 MB |
| 518×518 | 98.0 ms | ~10.2 | 689 MB |

**Recommendation:** 364×364 at 25 FPS — good balance of speed and quality. Input size must be divisible by 14 (DINOv2 patch size).

**Deployment pipeline:** PyTorch → ONNX → TensorRT (INT64 weights require workaround). Community resources: [spacewalk01/depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt), [IRCVLab/Depth-Anything-for-Jetson-Orin](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin).

---

## Part B: Grasp Planning for Litter Pickup

### B.1 Do We Need 6-DOF or Is Top-Down Sufficient?

**Answer: Top-down (planar, 4-DOF) is sufficient for MVP and handles 80-90% of litter scenarios.**

Reasoning:
- Ground litter lies flat on surfaces. The dominant approach angle is **top-down** (looking straight down at the pavement).
- A walking robot stops, extends its arm downward, and picks. This is a planar grasp scenario.
- 6-DOF is needed only for: objects wedged against walls, items in bins/containers, litter on inclined surfaces, or objects requiring approach from the side.
- Literature confirms: the LitterBot paper (Frontiers 2022) and the JIRS 2023 tactile litter system both use 3-4 DOF grasp estimation with 80-95% success rates on ground litter.

**Conclusion:** Start with planar (4-DOF) grasp planning. Add 6-DOF in Phase 2 for edge cases.

### B.2 Grasp Planning Systems — Comparison Table

| System | DOF | Input | Params | Inference (GPU) | Jetson Feasible? | Success Rate | License | Suction? |
|--------|-----|-------|--------|----------------|-----------------|-------------|---------|----------|
| **GR-ConvNet v2** | **4 (planar)** | **RGB-D image** | **1.9M** | **20ms (5-10ms TRT)** | **HIGH** | **95.4%** | **Permissive** | No |
| **VGN** | **6** | **TSDF volume** | **~1M** | **10ms + TSDF** | **HIGH** | 74% real | **BSD-3-Clause** | No |
| FC-GQ-CNN (Dex-Net) | 4 (planar) | Depth image | ~1M | 50ms (15-25ms TRT) | MEDIUM | 95% | UC Berkeley Non-Commercial | **Yes (Dex-Net 4.0)** |
| AnyGrasp | 7 | Point cloud | ~10M+ | ~100ms | LOW-MEDIUM | 93.3% | Proprietary SDK | No |
| GraspNet Baseline | 6 | RGB-D + PC | ~5M | ~100ms | MARGINAL | 96% (analytic) | Apache-2.0 | Yes (SuctionNet) |
| Contact-GraspNet | 6 | Point cloud | ~5M | ~280ms | LOW | >90% | NVIDIA Research (non-commercial) | No |
| GPD | 6 | Point cloud | ~2M | >1000ms | VERY LOW | 93% | BSD-2-Clause | No |

### B.3 Simple Heuristic Approaches

Before deploying a learned grasp planner, simple heuristics work surprisingly well for litter:

**Centroid-Based Top-Down Grasp:**
1. YOLO detects litter → bounding box
2. Project bounding box center into 3D using depth
3. Plan a top-down grasp at the centroid, oriented along the bbox major axis
4. Execute open-loop

**Expected success rate:** ~60-70% for rigid items (bottles, cans), much lower for deformable.
**Advantage:** Zero ML overhead, instant, deterministic.
**Use case:** Initial prototype before GR-ConvNet is deployed.

**Category-Specific Heuristics:**
| Litter Type | Heuristic Strategy |
|-------------|-------------------|
| Bottles | Grasp perpendicular to long axis at centroid |
| Cans | Grasp at centroid, any orientation (cylindrical symmetry) |
| Flat wrappers | Suction at centroid |
| Cigarette butts | Suction at centroid (small objects) |
| Plastic bags | Pinch protruding edge (detect via contour analysis) |

### B.4 What Works for Irregular Litter Items?

Litter is uniquely challenging because it spans the full spectrum of object properties:

| Challenge | Litter Examples | Solution |
|-----------|----------------|----------|
| Rigid, cylindrical | Bottles, cans | Parallel jaw grasp → GR-ConvNet excels |
| Rigid, irregular | Crumpled cans, broken glass | Adaptive fingers (Fin Ray) + GR-ConvNet |
| Flat, on surface | Wrappers, cigarette butts | Suction cup → centroid heuristic |
| Deformable, thin | Plastic bags, wet paper | Pinch edges → contour detection + finger grasp |
| Very small | Cigarette butts (7mm Ø) | Precision suction or fine-tip fingers |
| Wet/slippery | Rain-soaked items | Soft gripper + tactile feedback for re-grasp |

**Key insight from literature:** No single grasp planner handles all litter types. A **category-based grasp strategy** (YOLO class → grasp mode) outperforms a universal planner. Use YOLO detection class to select between finger grasp (GR-ConvNet) and suction (heuristic).

### B.5 Integration with Depth Estimation

The grasp planner and depth estimator must be compatible:

| Grasp Planner | Required Depth Input | Compatible Depth Sources |
|---------------|---------------------|------------------------|
| GR-ConvNet v2 | RGB-D image (224×224) | Any camera providing aligned RGB-D |
| VGN | TSDF volume | Depth camera + known camera pose |
| FC-GQ-CNN | Depth image only | Any depth camera |
| AnyGrasp | Point cloud | Depth camera → point cloud conversion |
| Centroid heuristic | Single depth value at centroid | Any depth source |

**GR-ConvNet v2 integration is the simplest:** It takes a 224×224 RGB-D image directly. The OAK-D Pro provides aligned RGB-D natively. No point cloud conversion, no TSDF construction. Just crop the detection bbox region, resize to 224×224, and feed to the network.

**Pipeline:**
```
[OAK-D Pro] → [Aligned RGB-D stream]
                     ↓
[YOLO Detection] → [Bbox crop of RGB-D] → [Resize 224×224]
                                                  ↓
                                          [GR-ConvNet v2 (TRT)]
                                                  ↓
                                          [Grasp pose (x, y, θ, w)]
                                                  ↓
                                          [Project to 3D via depth]
                                                  ↓
                                          [Arm IK + Execute]
```

### B.6 Recommended Grasp Planning Architecture

**Phase 1 — MVP:**
- **Primary:** GR-ConvNet v2 with TensorRT INT8 on Jetson
- **Fallback:** Centroid-based top-down heuristic
- **Gripper:** Parallel jaw only
- **Input:** RGB-D from OAK-D Pro, cropped to detection ROI
- **Expected latency:** 5-10ms grasp planning + 30ms detection = <50ms total perception
- **Expected success:** 85-90% on rigid litter (bottles, cans)

**Phase 2 — Hybrid Gripper:**
- Add suction to gripper
- YOLO class → gripper mode selection: rigid → fingers, flat → suction
- Suction uses centroid heuristic (no ML needed)
- **Expected success:** 80-85% across all litter categories

**Phase 3 — 6-DOF:**
- Add VGN for non-planar grasps
- TSDF construction from depth camera (wrist-mounted or external)
- Closed-loop visual servoing for fine adjustment
- **Expected success:** 90%+ across all categories

---

## Part C: Camera Hardware Recommendation

### C.1 Camera Comparison for Outdoor Litter-Picking Robot

| Camera | Technology | Outdoor Perf. | IP Rating | ROS2 | On-device AI | Weight | Price | Vibration Handling |
|--------|-----------|--------------|-----------|------|-------------|--------|-------|-------------------|
| **Luxonis OAK-D Pro** | Active stereo + IR | Good | None | Yes | **Myriad X VPU** | 91g | $399 | Fixed-focus variant |
| Luxonis OAK-D Lite | Passive stereo | Fair | None | Yes | Myriad X VPU | 61g | $149 | Rolling shutter |
| Intel RealSense D435i | Active IR stereo | Fair | None | Yes | No | 72g | $334 | Rolling shutter |
| Intel RealSense D455 | Active IR stereo | Fair | None | Yes | No | 380g | ~$350 | Rolling shutter |
| **Stereolabs ZED 2i** | Passive stereo + neural | **Excellent** | **IP66** | Yes | No | 166g | $549 | Rolling shutter + CPL |
| **Stereolabs ZED X Mini** | Passive stereo + neural | **Excellent** | **IP67** | Yes | No | ~100g | ~$549 | **Global shutter** |
| Orbbec Femto Bolt | Time-of-Flight | **Poor** | None | Yes | No | 348g | $490 | Rolling shutter |

### C.2 Depth Technology Performance Outdoors

| Technology | Sunlight | Rain | Low Texture | Close Range | Long Range |
|-----------|---------|------|------------|------------|------------|
| Passive stereo + neural (ZED) | Excellent | Good (if IP-rated) | Moderate | Good (0.2m) | Excellent (20m) |
| Active stereo + IR (OAK-D Pro) | Good (falls back to passive) | Needs enclosure | **Excellent** (IR helps) | Good (0.7m) | Good (12m) |
| Active IR stereo (RealSense) | Moderate (IR degraded) | Needs enclosure | Good | **Excellent** (0.1m D435) | Fair (10m) |
| ToF (Femto Bolt) | **Poor** (IR overwhelmed) | Poor (scatter) | Excellent | Good | Fair (5m) |

**Key insight for walking robots:** Global shutter cameras (ZED X series) eliminate rolling shutter artifacts from gait vibration. This is a significant advantage over OAK-D and RealSense (both rolling shutter). However, the ZED X Mini requires GMSL2 interface (additional deserializer board cost/complexity).

### C.3 RealSense Discontinuation Warning

Intel wound down the RealSense business unit in 2023. D400 series cameras remain available through distributors, but:
- No new product development
- Minimal firmware/driver updates (community-driven)
- Long-term supply chain risk

**Recommendation:** Avoid RealSense for new designs. Prefer Luxonis (growing ecosystem) or Stereolabs (stable, funded).

### C.4 Primary Recommendation: Luxonis OAK-D Pro ($399)

**Why OAK-D Pro wins for CleanWalker:**

1. **IR dot projector** solves the low-texture outdoor problem. Pavement, concrete, and bare ground are low-texture — passive stereo struggles. The IR projector adds texture for stereo matching. Falls back to passive in bright sunlight.

2. **Myriad X VPU** (4 TOPS) runs lightweight NN on-device. This can offload YOLO detection pre-filtering from Jetson, freeing GPU for depth + grasp planning. Runs MobileNet-SSD, YOLO-tiny, or other small models directly on camera.

3. **Fixed-focus variant** designed for vibration environments (drones, robots). No auto-focus hunting during walking gait.

4. **Native RGB-D alignment** provides exactly what GR-ConvNet v2 needs — no point cloud conversion required.

5. **12MP RGB** is excellent resolution for litter detection input.

6. **DepthAI SDK** + ROS2 Humble support is well-maintained and active.

7. **$399** — significantly cheaper than ZED alternatives.

**Limitations to address:**
- No IP rating — needs custom 3D-printed weatherproof enclosure with polycarbonate window
- Rolling shutter on RGB — mitigated by fixed-focus and high FPS capture
- 0.7m minimum depth range — fine for navigation, but close-range grasp verification may need supplementary sensor

### C.5 Alternative: Stereolabs ZED X Mini (~$549)

Choose this if:
- Budget allows the premium
- GMSL2 interface is feasible (requires deserializer board ~$200)
- Global shutter is critical (severe vibration from walking gait)
- IP67 weatherproofing needed without custom enclosure
- Multi-camera setup planned (GMSL2 provides dedicated bandwidth)

### C.6 Recommended Full Sensor Suite

**Standard Configuration ($1,148 must-have):**

| Sensor | Purpose | Mount Location | Price |
|--------|---------|---------------|-------|
| **Luxonis OAK-D Pro** (fixed-focus) | Front depth + litter detection | Head platform, 15-20° downward tilt | $399 |
| **Livox Mid-360** | 360° navigation + ground plane | Top center of body | $749 |
| **Total (must-have)** | | | **$1,148** |

**Enhanced Configuration ($1,297):**

| Sensor | Purpose | Mount Location | Price |
|--------|---------|---------------|-------|
| Luxonis OAK-D Pro (fixed-focus) | Front depth + litter detection | Head platform | $399 |
| Livox Mid-360 | 360° navigation + ground plane | Top center | $749 |
| **Luxonis OAK-D Lite** | Downward foothold camera | Chin, angled down | $149 |
| **Total (enhanced)** | | | **$1,297** |

**Premium Configuration ($1,498):**

| Sensor | Purpose | Mount Location | Price |
|--------|---------|---------------|-------|
| Stereolabs ZED X Mini | Front depth + detection (IP67, global shutter) | Head platform | $549 |
| GMSL2 deserializer board | Interface for ZED X Mini | Mounted with compute | ~$200 |
| Livox Mid-360 | 360° navigation + ground plane | Top center | $749 |
| **Total (premium)** | | | **~$1,498** |

---

## Part D: Integrated Perception Pipeline

### D.1 Full Pipeline Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                    SENSOR INPUTS                                    │
│                                                                    │
│  [OAK-D Pro]              [Livox Mid-360]                         │
│  ├─ RGB 1080p @30fps      └─ 3D point cloud                      │
│  ├─ Stereo depth @30fps     (360° × 59°, 200K pts/s)             │
│  └─ On-device YOLO-tiny                                           │
│     (pre-filter detections)                                       │
└───────────┬──────────────────────┬────────────────────────────────┘
            │                      │
┌───────────▼──────────┐  ┌───────▼────────────────────────────────┐
│  DETECTION PIPELINE  │  │  NAVIGATION PIPELINE                    │
│                      │  │                                         │
│  YOLO26s (TRT INT8)  │  │  LiDAR SLAM (e.g., FAST-LIO2)         │
│  ├─ Litter detection │  │  ├─ Localization                       │
│  ├─ Class + bbox     │  │  ├─ Obstacle map                       │
│  └─ 5-10ms           │  │  └─ Ground plane estimation            │
│                      │  │                                         │
│  + DA V2 Small (TRT) │  │  DA V2 Small (TRT)                     │
│  ├─ Dense relative   │  │  └─ Supplementary dense depth          │
│  │   depth           │  │     for close-range obstacles           │
│  └─ 25 FPS @ 364×364 │  │                                         │
└───────────┬──────────┘  └────────────────────────────────────────┘
            │
┌───────────▼──────────────────────────────────────────────────────┐
│  GRASP PLANNING PIPELINE                                          │
│                                                                    │
│  1. Crop RGB-D to detection bbox                                  │
│  2. Classify: rigid? → GR-ConvNet v2    flat? → suction heuristic │
│  3. GR-ConvNet v2 (TRT INT8): 5-10ms                             │
│     ├─ Input: 224×224 RGB-D crop                                  │
│     ├─ Output: quality Q, angle θ, width W per pixel              │
│     └─ Select best grasp (highest Q)                              │
│  4. Project grasp to 3D using stereo depth                        │
│  5. Output: 3D grasp pose (x, y, z, θ, w)                        │
└───────────┬──────────────────────────────────────────────────────┘
            │
┌───────────▼──────────────────────────────────────────────────────┐
│  ARM EXECUTION                                                    │
│                                                                    │
│  Analytical IK → joint trajectory → execute                       │
│  Open-loop for static ground litter                               │
│  Tactile feedback → re-grasp if failed                            │
└──────────────────────────────────────────────────────────────────┘
```

### D.2 Latency Budget

| Stage | Target | Actual (Estimated) | Notes |
|-------|--------|--------------------|-------|
| YOLO detection | <30ms | 5-10ms (TRT INT8) | YOLO26s on Jetson |
| Depth acquisition | <15ms | ~33ms (30fps) | One frame from OAK-D Pro |
| ROI crop + resize | <5ms | <2ms | GPU-accelerated |
| Grasp planning | <100ms | 5-10ms (TRT) | GR-ConvNet v2 |
| Depth → 3D projection | <5ms | <2ms | Simple math with intrinsics |
| Motion planning (IK) | <50ms | <20ms | Analytical IK for simple arm |
| **Total perception** | **<200ms** | **~50-75ms** | **Well within budget** |
| Arm execution | 500-2000ms | Physical | Mechanical speed limit |

**The perception pipeline is NOT the bottleneck.** At 50-75ms total, we could run the full detect→plan→project loop at 13-20 Hz. The bottleneck is arm physical movement (0.5-2s).

### D.3 Concurrent Execution Model

On Jetson Orin Nano Super (1024 CUDA cores), pipelines run concurrently:

```
Timeline (continuous):
─────────────────────────────────────────────────────────
Frame N:  [YOLO detect] [DA V2 depth] [ESS stereo]
Frame N+1:               [YOLO detect] [DA V2 depth]
Grasp trigger:                          [GR-ConvNet]
─────────────────────────────────────────────────────────
```

- YOLO + DA V2 run continuously at ~25 FPS during walking
- When litter is detected and robot approaches: stop, run GR-ConvNet on the detection ROI
- GR-ConvNet inference is on-demand (not continuous), so it doesn't compete for GPU during walking

### D.4 ROS2 Node Graph

```
/oak_camera_node
  └─ publishes: /rgb/image, /stereo/depth, /stereo/points

/yolo_detection_node
  ├─ subscribes: /rgb/image
  └─ publishes: /detections (bbox, class, confidence)

/depth_estimation_node (Depth Anything V2)
  ├─ subscribes: /rgb/image
  └─ publishes: /depth/relative

/isaac_ess_node (or OAK-D on-device stereo)
  ├─ subscribes: /stereo/left, /stereo/right
  └─ publishes: /depth/metric, /depth/confidence

/grasp_planning_node
  ├─ subscribes: /detections, /rgb/image, /depth/metric
  └─ publishes: /grasp_pose (stamped 3D pose)

/arm_controller_node
  ├─ subscribes: /grasp_pose
  └─ publishes: /joint_commands

/livox_lidar_node
  └─ publishes: /pointcloud2, /imu

/slam_node
  ├─ subscribes: /pointcloud2, /imu
  └─ publishes: /odom, /map
```

---

## Part E: Comparison with What Industry Robots Use

### E.1 Professional Quadruped Perception Stacks

| Robot | Depth Cameras | LiDAR | Grasp Planning | Total Sensors | Price |
|-------|--------------|-------|---------------|--------------|-------|
| Boston Dynamics Spot | 5 custom stereo pairs (360°) | Optional payload | Optional Spot Arm (simple grasp) | 10+ cameras | $74,500 |
| ANYbotics ANYmal | 6× Intel RealSense | 360° LiDAR | N/A (inspection, not grasping) | 6 depth + 1 LiDAR + 2 RGB | ~$150,000 |
| **CleanWalker (proposed)** | **1× OAK-D Pro** | **1× Livox Mid-360** | **GR-ConvNet v2 (TRT)** | **1 depth + 1 LiDAR** | **~$1,150 sensors** |

Our sensor suite is minimal compared to industry robots, but sufficient for the litter-picking use case:
- We only need forward-facing depth (not 360° depth) because the robot approaches detected litter head-on
- The Livox Mid-360 provides 360° obstacle awareness for navigation
- Adding a second camera (downward OAK-D Lite, $149) would approach ANYmal-level sensing at a fraction of the cost

---

## Part F: Risk Analysis & Mitigations

### F.1 Technical Risks

| Risk | Severity | Probability | Mitigation |
|------|----------|------------|------------|
| GR-ConvNet trained on tabletop, not outdoor | HIGH | HIGH | Fine-tune on litter-specific dataset (target 10K+ annotated grasps) |
| OAK-D Pro rolling shutter + gait vibration | MEDIUM | MEDIUM | Fixed-focus variant + high FPS + vibration dampening mount |
| Sunlight degrades IR projector | LOW | MEDIUM | OAK-D Pro falls back to passive stereo; neural depth as supplement |
| 8GB memory insufficient | LOW | LOW | Current estimate uses <5GB; significant headroom |
| Wet/reflective surfaces confuse depth | MEDIUM | MEDIUM | Confidence maps (ESS) filter bad depth; multi-frame averaging |
| Point cloud noise on Jetson | LOW | LOW | Use aligned RGB-D instead of point cloud for GR-ConvNet (avoids conversion) |

### F.2 Unresolved Questions

1. **GR-ConvNet v2 license clarity:** The GitHub repo appears permissive (academic origin) but has no explicit license file. Need to verify before commercial use.

2. **NVIDIA Isaac ROS ESS on Orin Nano Super specifically:** ESS runs on Orin Nano, but most benchmarks are on AGX Orin. Need to verify actual FPS on our specific hardware.

3. **OAK-D Pro close-range limitation:** 0.7m minimum depth range may be insufficient for wrist-mounted close-range grasp verification. May need Intel RealSense D405 ($259, 7cm minimum) as secondary close-range sensor.

4. **Multi-model TensorRT scheduling:** Running YOLO + DA V2 + GR-ConvNet on one Jetson GPU requires careful CUDA stream management to avoid interference. Need to profile concurrent execution.

---

## Sources

### Depth Estimation
- [Depth Anything V2 GitHub](https://github.com/DepthAnything/Depth-Anything-V2) — Apache-2.0 (Small only)
- [Depth Anything V2 Paper (NeurIPS 2024)](https://arxiv.org/abs/2406.09414)
- [Depth Anything for Jetson Orin (IRCVLab)](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin)
- [Depth Anything TensorRT](https://github.com/spacewalk01/depth-anything-tensorrt)
- [UniDepth GitHub](https://github.com/lpiccinelli-eth/UniDepth) — CC-BY-NC-4.0
- [UniDepthV2 Paper](https://arxiv.org/abs/2502.20110)
- [Metric3D GitHub](https://github.com/YvanYin/Metric3D) — BSD-2-Clause
- [Metric3D v2 Paper](https://arxiv.org/abs/2404.15506)
- [ZoeDepth GitHub](https://github.com/isl-org/ZoeDepth) — MIT (unmaintained)
- [RAFT-Stereo GitHub](https://github.com/princeton-vl/RAFT-Stereo) — MIT
- [CREStereo GitHub](https://github.com/megvii-research/CREStereo) — Apache-2.0
- [IGEV-Stereo / IGEV++ GitHub](https://github.com/gangweiX/IGEV-plusplus) — MIT
- [NVIDIA Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)
- [FoundationStereo GitHub](https://github.com/NVlabs/FoundationStereo)
- [RT-MonoDepth GitHub](https://github.com/Ecalpal/RT-MonoDepth)

### Grasp Planning
- [GR-ConvNet (robotic-grasping) GitHub](https://github.com/skumra/robotic-grasping)
- [GR-ConvNet v2 Paper (Sensors 2022)](https://www.mdpi.com/1424-8220/22/16/6208)
- [VGN GitHub](https://github.com/ethz-asl/vgn) — BSD-3-Clause
- [AnyGrasp SDK GitHub](https://github.com/graspnet/anygrasp_sdk) — Proprietary
- [Contact-GraspNet GitHub](https://github.com/NVlabs/contact_graspnet) — NVIDIA Research
- [GraspNet-1Billion Baseline](https://github.com/graspnet/graspnet-baseline) — Apache-2.0
- [Dex-Net / GQCNN GitHub](https://github.com/BerkeleyAutomation/gqcnn) — UC Berkeley Non-Commercial
- [GPD GitHub](https://github.com/atenpas/gpd) — BSD-2-Clause

### Camera Hardware
- [Luxonis OAK-D Pro](https://shop.luxonis.com/products/oak-d-pro)
- [Stereolabs ZED 2i](https://www.stereolabs.com/store/products/zed-2i)
- [Stereolabs ZED X Mini](https://www.stereolabs.com/store/products/zed-x-mini-stereo-camera)
- [Intel RealSense D435i](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
- [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- [Livox Mid-360](https://www.livoxtech.com/mid-360)

### Litter-Specific Research
- [LitterBot (Frontiers 2022)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1064853/full)
- [Vision and Tactile System for Outdoor Litter (JIRS 2023)](https://link.springer.com/article/10.1007/s10846-023-01930-2)
- [3D-Printed Gripper for Waste Collection (Robotics MDPI 2025)](https://www.mdpi.com/2218-6581/14/7/87)
