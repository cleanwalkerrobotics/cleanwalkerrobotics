# Perception Stack: Depth Estimation, Grasp Planning & Camera Hardware (Feb 2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB shared RAM, 1024 CUDA cores)
**Use Case:** Autonomous outdoor litter-picking quadruped robot with mechanical gripper
**Date:** February 11, 2026
**Consolidates:** depth-estimation-sota-2026.md, grasp-planning-systems-2026.md, depth-camera-hardware-2026.md

---

## Executive Summary & Recommendations

### TL;DR — What to Buy and Deploy

| Component | Primary Choice | Backup | Why |
|-----------|---------------|--------|-----|
| **Depth Camera** | Luxonis OAK-D Pro ($399) | Orbbec Gemini 345Lg (TBD, CES 2026) | IR projector for low-texture ground; VPU offloads processing; fixed-focus for vibration |
| **Navigation LiDAR** | Livox Mid-360 ($749) | RPLIDAR A3 ($599) | 360x59 FOV, IP67, 70m range, 265g, proven on quadrupeds |
| **Monocular Depth** | Depth Anything V2 Small (TRT) | DA V3 Small (when mature) | 25-42 FPS on Jetson, 24.8M params, Apache-2.0, relative depth |
| **Stereo Depth** | NVIDIA Isaac ROS ESS | OAK-D on-device stereo | 17M params, native TRT, confidence maps, production-ready |
| **Grasp Planner (MVP)** | GR-ConvNet v2 (TRT INT8) | Centroid heuristic | 1.9M params, 5-10ms TRT, 95.4% success, planar grasps |
| **Grasp Planner (6-DOF)** | VGN | -- | 10ms inference, BSD-3-Clause, TSDF input, Phase 2+ |
| **Gripper** | Hybrid Fin Ray + Suction | Parallel jaw only | Fingers for rigid litter, suction for flat/deformable items |

### Memory Budget (8GB Shared)

| Component | Estimated Memory | Notes |
|-----------|-----------------|-------|
| OS + System | ~1.5 GB | Ubuntu + ROS2 + framework overhead |
| YOLO Litter Detection | ~0.5-1.0 GB | YOLO26s with TensorRT INT8 |
| Depth Estimation | ~0.6-0.7 GB | DA V2 Small (TRT) or ESS |
| Grasp Planning | ~0.3-0.5 GB | GR-ConvNet v2 (TRT) |
| Point Cloud / Buffers | ~0.5 GB | ROI extraction, TSDF if needed |
| ROS2 / Application | ~0.5 GB | Message passing, state management |
| **Total** | **~4.4-4.7 GB** | **~3.3-3.6 GB headroom** |

---

## Part 1: Depth Estimation

### 1.1 Monocular Depth Models

| Model | Params | Output Type | KITTI AbsRel | FPS on Jetson (TRT) | Metric Depth? | License | Jetson-Ready? |
|-------|--------|-------------|-------------|---------------------|---------------|---------|---------------|
| **DA V2 Small** | **24.8M** | Relative (metric w/ finetune) | 0.078 | **25-42** | Relative (metric w/ finetune) | **Apache-2.0** | **Yes (tested)** |
| **DA V3 Small** (NEW) | ~25M | Depth + Ray | TBD | Est. 25-40 | Metric (focal-based) | TBD (likely Apache-2.0) | Est. yes (not yet tested) |
| DA V3 Metric-Large (NEW) | ~335M | Direct metric | TBD | -- | **Yes (direct)** | TBD (likely CC-BY-NC-4.0) | No (OOM) |
| Apple Depth Pro (NEW) | ~600-800M | Metric (zero-shot) | SOTA | -- | **Yes (zero-shot)** | Apple Sample Code | No (OOM) |
| UniDepthV2-ViT-S | ~30M est. | Metric (zero-shot) | ~0.05 est. | Unknown | **Yes (zero-shot)** | CC-BY-NC-4.0 | Untested |
| Metric3D v2 ViT-S | ~40M est. | Metric + Normals | -- | Unknown | **Yes + normals** | **BSD-2-Clause** | Untested |
| ZoeDepth (Swin2-T) | 42M | Metric (domain-aware) | -- | Unknown | Yes | **MIT** | Untested, unmaintained |

**Key finding:** Depth Anything V2 Small remains the only proven monocular option on Jetson Orin Nano. DA V3 (Nov 2025) is the new SOTA with metric depth support, but Jetson deployment is immature (Q1-Q2 2026 expected).

#### DA V2 Small — Jetson Performance Detail

| Input Resolution | Inference Time | FPS | Memory |
|-----------------|----------------|-----|--------|
| 308x308 | 23.5 ms | ~42.6 | 626 MB |
| 364x364 | 39.2 ms | ~25.5 | 640 MB |
| 406x406 | 47.7 ms | ~20.9 | 649 MB |
| 518x518 | 98.0 ms | ~10.2 | 689 MB |

**Recommended:** 364x364 at 25 FPS. Input size must be divisible by 14 (DINOv2 patch size).

#### DA V3 — What's New (Nov 2025)

- Dual-prediction head: depth maps AND ray maps for cross-view reasoning
- Metric depth via focal length estimation: `metric_depth = focal * net_output / 300`
- 4K resolution support
- Streaming inference for ultra-long video (<12GB memory)
- ROS2 TensorRT wrapper already available: [ros2-depth-anything-v3-trt](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt)
- Jetson AGX Orin guide exists; Orin Nano not yet validated

### 1.2 Stereo Depth Models

| Model | Params | Memory | FPS on Jetson | Confidence Map? | License | Jetson-Ready? |
|-------|--------|--------|---------------|----------------|---------|---------------|
| **Isaac ROS ESS** | **17M** | Optimized | **Est. 20-40+** | **Yes** | NVIDIA prop. (free for Jetson) | **Yes (production)** |
| FoundationStereo | 60.6M | -- | <1 | No | NVIDIA prop. | Yes (too slow) |
| RT-IGEV++ | ~20M est. | 0.66 GB | Est. 3-8 | No | **MIT** | Likely (untested) |
| RAFT-Stereo | 11.23M | <1 GB | Est. 3-8 | No | **MIT** | Likely (untested) |
| CREStereo | ~12M est. | -- | Unknown | No | **Apache-2.0** | Possible (ONNX exists) |
| SGM (classical) | N/A | Minimal | 100+ | No | Apache-2.0 | **Yes** |

### 1.3 Metric Depth: Which Models Give Absolute Distance?

| Model | Metric Depth? | How | Reliability for Grasping |
|-------|--------------|-----|------------------------|
| DA V2 Small | Relative only (metric w/ finetune) | Fine-tuned KITTI/NYU variants | Low -- finetune domain-specific |
| **DA V3 Metric-Large** | **Yes (direct)** | Focal length prediction + depth | Medium -- new, unvalidated on Jetson |
| **Depth Pro** | **Yes (zero-shot)** | Predicts focal + metric depth | High quality but too large for edge |
| **UniDepthV2** | **Yes (zero-shot)** | Self-promptable camera module | Medium -- CC-BY-NC license |
| **Metric3D v2** | **Yes + surface normals** | Canonical Camera Space Transform | Medium -- BSD-2 license is excellent |
| **Stereo (any)** | **Yes (triangulation)** | Known baseline + focal length | **HIGH -- physics-based, reliable** |

**Verdict for grasp planning:** Use **hardware stereo** for metric depth. Monocular metric depth is improving rapidly (DA V3, Depth Pro) but stereo triangulation remains more reliable for sub-centimeter grasping accuracy.

### 1.4 Recommended Depth Strategy

**Primary (navigation):** DA V2 Small on monocular RGB at 25+ FPS via TensorRT. Dense relative depth for obstacle avoidance.

**Primary (grasping):** Hardware stereo from OAK-D Pro. Either:
- Isaac ROS ESS (stereo pair input) -- best accuracy + confidence maps
- On-device stereo from OAK-D Pro's Myriad X VPU -- zero Jetson GPU overhead

**Future (2026):** DA V3 Metric-Large may eventually replace stereo for some grasping tasks once Jetson deployment matures.

---

## Part 2: Grasp Planning

### 2.1 Do We Need 6-DOF? Answer: Top-Down Is Sufficient for MVP

**Top-down (planar, 4-DOF) handles 80-90% of litter scenarios.**

Reasoning:
- Ground litter lies flat on surfaces -- dominant approach angle is top-down
- Robot stops, extends arm downward, picks -- this is a planar grasp scenario
- LitterBot (Frontiers 2022) and JIRS 2023 tactile system both use 3-4 DOF with 80-95% success
- 6-DOF needed only for: items wedged against walls, in bins, on inclined surfaces, or requiring side approach

### 2.2 Grasp Planning Systems Comparison

| System | DOF | Input | Params | Inference | Jetson? | Success Rate | License | Suction? |
|--------|-----|-------|--------|-----------|---------|-------------|---------|----------|
| **GR-ConvNet v2** | **4 (planar)** | **RGB-D image** | **1.9M** | **20ms (5-10ms TRT)** | **HIGH** | **95.4%** | **Permissive** | No |
| **VGN** | **6** | **TSDF volume** | **~1M** | **10ms + TSDF** | **HIGH** | 74% real | **BSD-3-Clause** | No |
| FC-GQ-CNN (Dex-Net 4.0) | 4 (planar) | Depth image | ~1M | 50ms (15-25ms TRT) | MEDIUM | 95% | UC Berkeley Non-Commercial | **Yes** |
| AnyGrasp | 7 | Point cloud | ~10M+ | ~100ms | LOW-MEDIUM | 93.3% | **Proprietary SDK** | No |
| GraspNet-1Billion baseline | 6 | RGB-D + PC | ~5M | ~100ms | MARGINAL | 96% (analytic) | Apache-2.0 (code), CC-BY-NC-SA (data) | Yes (SuctionNet) |
| Contact-GraspNet | 6 | Point cloud | ~5M | ~280ms | LOW | >90% | NVIDIA Research (non-commercial) | No |
| GPD | 6 | Point cloud | ~2M | >1000ms | VERY LOW | 93% | BSD-2-Clause | No |

### 2.3 Simple Heuristic Approaches (MVP Fallback)

**Centroid-Based Top-Down Grasp:**
1. YOLO detects litter -> bounding box
2. Project bbox center into 3D using depth
3. Plan top-down grasp at centroid, oriented along bbox major axis
4. Execute open-loop

Expected success: ~60-70% rigid items. Zero ML overhead, instant, deterministic.

**Category-Specific Heuristics:**

| Litter Type | Heuristic Strategy |
|-------------|-------------------|
| Bottles | Grasp perpendicular to long axis at centroid |
| Cans | Grasp at centroid, any orientation (cylindrical symmetry) |
| Flat wrappers | Suction at centroid |
| Cigarette butts | Suction at centroid (small objects) |
| Plastic bags | Pinch protruding edge (contour analysis) |

### 2.4 Learning-Based vs Heuristic Tradeoffs

| Aspect | Learning-Based (GR-ConvNet) | Heuristic (Centroid) |
|--------|---------------------------|---------------------|
| Success rate | 85-95% | 60-70% |
| Inference time | 5-20ms | <1ms |
| Generalization | Good (trained on diverse objects) | Poor (fixed strategy) |
| Development effort | Medium (TRT export, fine-tuning) | Low (geometry only) |
| Edge cases | Handles irregular shapes | Fails on asymmetric objects |
| **Recommendation** | Phase 1+ production | Phase 0 prototype only |

### 2.5 Integration: Depth -> Grasp Pipeline

```
[OAK-D Pro] -> [Aligned RGB-D stream]
                     |
[YOLO Detection] -> [Bbox crop of RGB-D] -> [Resize 224x224]
                                                  |
                                          [GR-ConvNet v2 (TRT)]
                                                  |
                                          [Grasp pose (x, y, theta, w)]
                                                  |
                                          [Project to 3D via stereo depth]
                                                  |
                                          [Arm IK + Execute]
```

GR-ConvNet v2 takes 224x224 RGB-D image directly. OAK-D Pro provides aligned RGB-D natively. No point cloud conversion needed.

### 2.6 Grasp Planner Input Compatibility

| Grasp Planner | Required Depth Input | Compatible Depth Sources |
|---------------|---------------------|------------------------|
| GR-ConvNet v2 | RGB-D image (224x224) | Any camera providing aligned RGB-D |
| VGN | TSDF volume | Depth camera + known camera pose |
| FC-GQ-CNN | Depth image only | Any depth camera |
| AnyGrasp | Point cloud | Depth camera -> point cloud conversion |
| Centroid heuristic | Single depth value at centroid | Any depth source |

### 2.7 Latency Budget

| Stage | Target | Estimated | Notes |
|-------|--------|-----------|----|
| YOLO detection | <30ms | 5-10ms (TRT INT8) | YOLO26s on Jetson |
| Depth acquisition | <15ms | ~33ms (30fps) | One frame from OAK-D Pro |
| ROI crop + resize | <5ms | <2ms | GPU-accelerated |
| Grasp planning | <100ms | 5-10ms (TRT) | GR-ConvNet v2 |
| Depth -> 3D projection | <5ms | <2ms | Intrinsics math |
| Motion planning (IK) | <50ms | <20ms | Analytical IK |
| **Total perception** | **<200ms** | **~50-75ms** | **Well within budget** |
| Arm execution | 500-2000ms | Physical | Mechanical speed limit |

**The perception pipeline is NOT the bottleneck.** Arm physical movement (0.5-2s) dominates. For static ground litter, even 200ms grasp planning is fine since litter doesn't move.

---

## Part 3: Camera Hardware

### 3.1 Camera Comparison (Outdoor Litter-Picking Robot)

| Camera | Technology | Outdoor Perf. | IP Rating | ROS2 | On-device AI | Weight | Price |
|--------|-----------|--------------|-----------|------|-------------|--------|-------|
| **Luxonis OAK-D Pro** | Active stereo + IR | Good | None | Yes | **Myriad X VPU** | 91g | $399 |
| **Orbbec Gemini 345Lg** (NEW) | Active stereo IR (rugged) | **Excellent (100+ klux)** | **IP67** | Yes | No | TBD | TBD (CES 2026) |
| Orbbec Gemini 2 XL | Active stereo IR | Good (outdoor mode) | None | Yes | No | TBD | ~$300 est. |
| Luxonis OAK-D Lite | Passive stereo | Fair | None | Yes | Myriad X VPU | 61g | $149 |
| Intel RealSense D435i | Active IR stereo | Fair | None | Yes | No | 72g | $334 |
| Intel RealSense D455 | Active IR stereo | Fair | None | Yes | No | 380g | ~$350 |
| **Stereolabs ZED 2i** | Passive stereo + neural | **Excellent** | **IP66** | Yes | No | 166g | $549 |
| **Stereolabs ZED X Mini** | Passive stereo + neural | **Excellent** | **IP67** | Yes | No | ~100g | ~$549 |
| Orbbec Femto Bolt | Time-of-Flight | **Poor** | None | Yes | No | 348g | $490 |

### 3.2 Outdoor Sunlight Performance Ranking

| Rank | Technology | Sunlight Tolerance | Examples |
|------|-----------|-------------------|----------|
| 1 | **Active stereo, 100+ klux rated** | Excellent -- designed for direct sun | Orbbec Gemini 345Lg |
| 2 | Passive stereo + neural depth | Excellent -- sunlight is beneficial | ZED 2i, ZED X Mini |
| 3 | Active stereo with IR fallback | Good -- graceful degradation in sun | OAK-D Pro |
| 4 | Pure passive stereo | Good but texture-dependent | OAK-D Lite, OAK-D S2 |
| 5 | Active IR stereo | Moderate -- IR degraded in sun | RealSense D4xx |
| 6 | Time-of-Flight | **Poor** -- IR overwhelmed by sun | Femto Bolt, Azure Kinect |

### 3.3 Intel RealSense Status (2026)

- Business unit wound down in 2023. No new product development.
- D400 series still available through distributors at $334 (D435i).
- Minimal firmware/driver updates (community-driven via librealsense GitHub).
- **Recommendation: Avoid for new designs.** Prefer Luxonis or Stereolabs.

### 3.4 Orbbec Gemini 345Lg (CES 2026 -- NEW)

Purpose-built for extreme outdoor robotics:
- Reliably outputs depth in 100+ klux direct sunlight AND pitch-dark conditions
- IP67 rated -- rain, dust, washdown proof
- Operating temp: -20C to 65C
- GMSL2 interface (Jetson-native, no USB bandwidth issues)
- Dual-mode depth FOV up to 104x87
- Active stereo with IR (outdoor-optimized wavelengths)
- ROS2 support via Orbbec SDK
- **Pricing not yet disclosed -- contact Orbbec for eval unit**

### 3.5 Why OAK-D Pro Wins for MVP

1. **IR dot projector** solves low-texture outdoor problem (pavement, concrete)
2. **Myriad X VPU** runs lightweight NN on-device, freeing Jetson GPU
3. **Fixed-focus variant** designed for vibration (drones, robots)
4. **Native RGB-D alignment** -- exactly what GR-ConvNet v2 needs
5. **12MP RGB** -- excellent for YOLO detection input
6. **DepthAI SDK + ROS2 Humble** -- well-maintained, active community
7. **$399** -- significantly cheaper than ZED alternatives

**Limitations:** No IP rating (needs 3D-printed enclosure), rolling shutter (mitigated by fixed-focus + high FPS), 0.7m minimum depth range.

### 3.6 Recommended Sensor Suites

**Standard ($1,148):**

| Sensor | Purpose | Price |
|--------|---------|-------|
| Luxonis OAK-D Pro (fixed-focus) | Front depth + litter detection | $399 |
| Livox Mid-360 | 360 navigation + ground plane | $749 |

**Enhanced ($1,297):**

| Sensor | Purpose | Price |
|--------|---------|-------|
| OAK-D Pro (fixed-focus) | Front depth + litter detection | $399 |
| Livox Mid-360 | 360 navigation + ground plane | $749 |
| OAK-D Lite | Downward foothold camera | $149 |

**Premium ($1,498):**

| Sensor | Purpose | Price |
|--------|---------|-------|
| Stereolabs ZED X Mini | Front depth (IP67, global shutter) | $549 |
| GMSL2 deserializer board | Interface for ZED X | ~$200 |
| Livox Mid-360 | 360 navigation + ground plane | $749 |

**Future-Ready (pending pricing):**

| Sensor | Purpose | Price |
|--------|---------|-------|
| Orbbec Gemini 345Lg | Front depth (IP67, 100+ klux) | TBD |
| Livox Mid-360 | 360 navigation + ground plane | $749 |

---

## Part 4: Integrated Architecture

### 4.1 Full Pipeline

```
+----------------------------------------------------------------------+
|                         SENSOR INPUTS                                 |
|                                                                       |
|  [OAK-D Pro]                    [Livox Mid-360]                      |
|  +- RGB 1080p @30fps            +- 3D point cloud                    |
|  +- Stereo depth @30fps           (360 x 59, 200K pts/s)            |
|  +- On-device YOLO-tiny                                              |
|     (pre-filter detections)                                          |
+--------+-----------------------------+-------------------------------+
         |                             |
+--------v-----------+    +------------v-------------------------------+
|  DETECTION PIPELINE |    |  NAVIGATION PIPELINE                      |
|                     |    |                                            |
|  YOLO26s (TRT INT8) |    |  LiDAR SLAM (FAST-LIO2)                  |
|  +- Litter detection|    |  +- Localization                          |
|  +- Class + bbox    |    |  +- Obstacle map                          |
|  +- 5-10ms         |    |  +- Ground plane estimation               |
|                     |    |                                            |
|  + DA V2 Small (TRT)|    |  DA V2 Small (TRT)                        |
|  +- Dense relative  |    |  +- Supplementary dense depth             |
|  |   depth          |    |     for close-range obstacles              |
|  +- 25 FPS          |    |                                            |
+---------+-----------+    +-------------------------------------------+
          |
+---------v------------------------------------------------------------+
|  GRASP PLANNING PIPELINE                                              |
|                                                                       |
|  1. Crop RGB-D to detection bbox                                     |
|  2. Classify: rigid? -> GR-ConvNet v2    flat? -> suction heuristic  |
|  3. GR-ConvNet v2 (TRT INT8): 5-10ms                                |
|     +- Input: 224x224 RGB-D crop                                     |
|     +- Output: quality Q, angle theta, width W per pixel             |
|     +- Select best grasp (highest Q)                                 |
|  4. Project grasp to 3D using stereo depth                           |
|  5. Output: 3D grasp pose (x, y, z, theta, w)                       |
+---------+------------------------------------------------------------+
          |
+---------v------------------------------------------------------------+
|  ARM EXECUTION                                                        |
|                                                                       |
|  Analytical IK -> joint trajectory -> execute                        |
|  Open-loop for static ground litter                                  |
|  Tactile feedback -> re-grasp if failed                              |
+----------------------------------------------------------------------+
```

### 4.2 Concurrent Execution Model

```
Timeline (continuous during walking):
-------------------------------------------------------------------
Frame N:  [YOLO detect] [DA V2 depth] [ESS stereo]
Frame N+1:               [YOLO detect] [DA V2 depth]
Grasp trigger:                          [GR-ConvNet] (on-demand)
-------------------------------------------------------------------
```

- YOLO + DA V2 run continuously at ~25 FPS during walking
- GR-ConvNet runs on-demand when robot stops to pick, so it doesn't compete for GPU

### 4.3 ROS2 Node Graph

```
/oak_camera_node
  +- publishes: /rgb/image, /stereo/depth, /stereo/points

/yolo_detection_node
  +- subscribes: /rgb/image
  +- publishes: /detections (bbox, class, confidence)

/depth_estimation_node (Depth Anything V2)
  +- subscribes: /rgb/image
  +- publishes: /depth/relative

/grasp_planning_node
  +- subscribes: /detections, /rgb/image, /depth/metric
  +- publishes: /grasp_pose (stamped 3D pose)

/arm_controller_node
  +- subscribes: /grasp_pose
  +- publishes: /joint_commands

/livox_lidar_node
  +- publishes: /pointcloud2, /imu

/slam_node
  +- subscribes: /pointcloud2, /imu
  +- publishes: /odom, /map
```

---

## Part 5: Implementation Phases

### Phase 1 -- MVP (Months 1-3)
- **Grasp:** GR-ConvNet v2 with TensorRT INT8
- **Depth:** OAK-D Pro on-device stereo + DA V2 Small for navigation
- **Camera:** Luxonis OAK-D Pro ($399) + Livox Mid-360 ($749)
- **Gripper:** Parallel jaw only
- **Grasp mode:** Open-loop, top-down
- **Targets:** Bottles, cans, rigid litter
- **Expected success:** 85-90%

### Phase 2 -- Hybrid Gripper (Months 3-6)
- Add suction cup (hybrid Fin Ray + suction)
- YOLO class drives gripper mode: rigid -> fingers, flat -> suction
- Suction uses centroid heuristic (no ML needed)
- Tactile feedback for grasp verification
- **Expected success:** 80-85% across ALL litter categories

### Phase 3 -- Advanced Grasping (Months 6-12)
- VGN for 6-DOF grasps on challenging objects
- Closed-loop visual servoing for fine adjustment
- Custom litter-specific grasp dataset + fine-tuning (10K+ annotated grasps)
- DA V3 Metric for monocular metric depth (if Jetson deployment matures)
- Temporal tracking for wind-blown items
- **Expected success:** 90%+ across all categories

---

## Part 6: Risk Analysis

| Risk | Severity | Probability | Mitigation |
|------|----------|------------|------------|
| GR-ConvNet trained on tabletop, not outdoor | HIGH | HIGH | Fine-tune on litter-specific dataset |
| OAK-D Pro rolling shutter + gait vibration | MEDIUM | MEDIUM | Fixed-focus variant + high FPS + damping mount |
| Sunlight degrades IR projector | LOW | MEDIUM | Falls back to passive stereo; DA V2 as supplement |
| 8GB memory insufficient | LOW | LOW | Current estimate uses <5GB; significant headroom |
| Wet/reflective surfaces confuse depth | MEDIUM | MEDIUM | Confidence maps (ESS) filter bad depth |
| GR-ConvNet v2 license unclear | MEDIUM | MEDIUM | Verify with authors before commercial use |

### Unresolved Questions

1. **GR-ConvNet v2 license:** GitHub repo appears permissive but has no explicit license file. Verify before commercial use.
2. **Isaac ROS ESS on Orin Nano Super:** Most benchmarks on AGX Orin. Verify actual FPS on our hardware.
3. **OAK-D Pro 0.7m minimum depth:** May need Intel RealSense D405 ($259, 7cm min) for close-range grasp verification.
4. **Orbbec Gemini 345Lg pricing:** Contact Orbbec for eval unit. Could be the best outdoor camera if priced competitively.

---

## Sources

### Depth Estimation
- [Depth Anything V2 GitHub](https://github.com/DepthAnything/Depth-Anything-V2) -- Apache-2.0 (Small)
- [Depth Anything V3 GitHub](https://github.com/ByteDance-Seed/Depth-Anything-3) -- Nov 2025
- [DA V3 Paper (arXiv 2511.10647)](https://arxiv.org/abs/2511.10647)
- [DA V2 Paper (NeurIPS 2024)](https://arxiv.org/abs/2406.09414)
- [Apple Depth Pro GitHub](https://github.com/apple/ml-depth-pro) -- ICLR 2025
- [Depth Anything for Jetson Orin](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin)
- [Depth Anything TensorRT](https://github.com/spacewalk01/depth-anything-tensorrt)
- [ROS2 Depth Anything V3 TRT](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt)
- [UniDepth GitHub](https://github.com/lpiccinelli-eth/UniDepth) -- CC-BY-NC-4.0
- [Metric3D GitHub](https://github.com/YvanYin/Metric3D) -- BSD-2-Clause
- [ZoeDepth GitHub](https://github.com/isl-org/ZoeDepth) -- MIT
- [RAFT-Stereo GitHub](https://github.com/princeton-vl/RAFT-Stereo) -- MIT
- [CREStereo GitHub](https://github.com/megvii-research/CREStereo) -- Apache-2.0
- [IGEV-Stereo / IGEV++ GitHub](https://github.com/gangweiX/IGEV-plusplus) -- MIT
- [NVIDIA Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)
- [FoundationStereo GitHub](https://github.com/NVlabs/FoundationStereo)

### Grasp Planning
- [GR-ConvNet GitHub](https://github.com/skumra/robotic-grasping) -- IROS 2020
- [GR-ConvNet v2 Paper (Sensors 2022)](https://www.mdpi.com/1424-8220/22/16/6208)
- [VGN GitHub](https://github.com/ethz-asl/vgn) -- BSD-3-Clause
- [AnyGrasp SDK GitHub](https://github.com/graspnet/anygrasp_sdk) -- Proprietary
- [Contact-GraspNet GitHub](https://github.com/NVlabs/contact_graspnet) -- NVIDIA Research
- [GraspNet-1Billion Baseline](https://github.com/graspnet/graspnet-baseline) -- Apache-2.0
- [Dex-Net / GQCNN GitHub](https://github.com/BerkeleyAutomation/gqcnn) -- UC Berkeley Non-Commercial
- [GPD GitHub](https://github.com/atenpas/gpd) -- BSD-2-Clause

### Camera Hardware
- [Luxonis OAK-D Pro](https://shop.luxonis.com/products/oak-d-pro)
- [Orbbec Gemini 345Lg (CES 2026)](https://www.orbbec.com/products/stereo-vision-camera/)
- [Stereolabs ZED 2i](https://www.stereolabs.com/store/products/zed-2i)
- [Stereolabs ZED X Mini](https://www.stereolabs.com/store/products/zed-x-mini-stereo-camera)
- [Intel RealSense D435i](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
- [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- [Livox Mid-360](https://www.livoxtech.com/mid-360)

### Litter-Specific Research
- [LitterBot (Frontiers 2022)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1064853/full)
- [Vision and Tactile System for Outdoor Litter (JIRS 2023)](https://link.springer.com/article/10.1007/s10846-023-01930-2)
- [3D-Printed Gripper for Waste Collection (Robotics MDPI 2025)](https://www.mdpi.com/2218-6581/14/7/87)
