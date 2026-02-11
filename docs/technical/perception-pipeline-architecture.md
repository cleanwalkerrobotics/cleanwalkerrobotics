# CleanWalker Perception Pipeline Architecture

**Version:** 1.0
**Date:** 2026-02-11
**Status:** Architecture Reference — Pilot Partner Ready
**Hardware:** NVIDIA Jetson Orin Nano Super (8 GB, 67 TOPS INT8)
**Camera:** Luxonis OAK-D Pro ($399) + Livox Mid-360 ($749)
**Middleware:** ROS 2 Humble + Isaac ROS

---

## 1. System Overview

The CleanWalker perception stack runs three concurrent pipelines on a single Jetson Orin Nano Super: (1) a **detection-to-grasp pipeline** that spots litter via YOLO26, estimates depth via hardware stereo, and plans grasps via GR-ConvNet v2 — all within 75 ms; (2) a **terrain segmentation pipeline** that classifies ground surfaces via SegFormer-B0 and feeds traversability costs into Nav2; and (3) a **localization pipeline** that fuses GPU-accelerated visual odometry (cuVSLAM) with RTAB-Map mapping and 2D LiDAR to produce a drift-corrected occupancy grid. These three pipelines share 8 GB of unified memory and are orchestrated via ROS 2 topics, with TensorRT INT8/FP16 optimization on every neural network. The full stack fits within 65% GPU utilization and ~5 GB RAM, leaving headroom for the navigation stack, behavior tree, and locomotion controller.

---

## 2. Hardware Platform

### 2.1 Compute: Jetson Orin Nano Super

| Specification | Value |
|--------------|-------|
| GPU | 1024-core NVIDIA Ampere, 32 Tensor Cores |
| CPU | 6-core Arm Cortex-A78AE v8.2 64-bit |
| AI Performance | 67 TOPS (INT8) / 34 TOPS (FP16) |
| Memory | 8 GB 128-bit LPDDR5, 68 GB/s bandwidth |
| Power | 7W / 15W / 25W (Super mode) |
| TensorRT | Yes (JetPack 6.x) |
| Price | ~$249 (developer kit) |

### 2.2 Primary Camera: OAK-D Pro (Fixed-Focus)

| Specification | Value |
|--------------|-------|
| RGB Sensor | 12MP IMX378, 4056x3040 |
| Stereo Baseline | 75 mm |
| Depth Range | 0.7 m – 15 m (active IR), 0.2 m – 35 m (passive) |
| Depth FPS | 30 FPS @ 640x480 |
| IR Projector | Active structured light (outdoor-capable) |
| IMU | BNO086 9-axis (400 Hz accel, 1000 Hz gyro) |
| On-device VPU | Intel Myriad X (4 TOPS) |
| Interface | USB 3.1 Gen 1 |
| Price | $399 |

**Why OAK-D Pro:** IR projector solves low-texture ground (pavement, concrete). Myriad X VPU offloads lightweight preprocessing. Fixed-focus variant handles gait vibration. Native RGB-D alignment matches GR-ConvNet v2 input format. BNO086 IMU provides hardware-synced timestamps for visual-inertial SLAM.

### 2.3 Navigation LiDAR: Livox Mid-360

| Specification | Value |
|--------------|-------|
| FOV | 360° × 59° |
| Range | 70 m (10% reflectivity) |
| Point Rate | 200,000 pts/s |
| Weight | 265 g |
| IP Rating | IP67 |
| Interface | 100BASE-T1 Ethernet |
| Price | $749 |

**Why Livox Mid-360:** 360° coverage for RTAB-Map scan matching and Nav2 costmap. IP67 for outdoor operation. Proven on quadruped platforms. Non-repetitive scanning pattern yields dense maps over time.

### 2.4 Sensor Suite Summary

| Sensor | Purpose | Outputs | Rate | Cost |
|--------|---------|---------|------|------|
| OAK-D Pro | Litter detection, stereo depth, grasping, visual SLAM | RGB, depth, IMU | 30 Hz | $399 |
| Livox Mid-360 | 360° obstacle map, SLAM scan matching, ground plane | 3D point cloud | 10 Hz | $749 |
| **Total** | | | | **$1,148** |

---

## 3. ROS 2 Node Graph

### 3.1 Pipeline Overview (Text Diagram)

```
═══════════════════════════════════════════════════════════════════
                    THREE CONCURRENT PIPELINES
═══════════════════════════════════════════════════════════════════

PIPELINE 1: DETECTION → DEPTH → GRASP → ARM
──────────────────────────────────────────────────────────────────
  OAK-D Pro RGB ──► YOLO26s (TRT INT8) ──► /detections
       30 Hz            5-10 ms              (bbox, class, conf)
                                                    │
  OAK-D Pro Stereo ──► Isaac ROS ESS ──► /depth/metric
       30 Hz              ~20 ms           (metric depth + conf)
                                                    │
                                    ┌───────────────┘
                                    ▼
                           ROI Crop + Resize ──► GR-ConvNet v2
                                <2 ms              5-10 ms (TRT)
                                                      │
                                                      ▼
                                            Grasp Pose (x,y,z,θ,w)
                                                      │
                                                      ▼
                                              Arm IK + Execute
                                                 (analytical)


PIPELINE 2: SEGMENTATION → NAVIGATION
──────────────────────────────────────────────────────────────────
  OAK-D Pro RGB ──► SegFormer-B0 (TRT FP16) ──► /terrain/mask
       30 Hz             ~25 ms @ 5 Hz            (8-class map)
                                                       │
                                                       ▼
                                              Terrain Costmap Layer
                                                       │
                                                       ▼
                                              Nav2 Path Planning


PIPELINE 3: SLAM → LOCALIZATION → MAP
──────────────────────────────────────────────────────────────────
  OAK-D Pro Stereo ──┐
       30 Hz          │
                      ├──► cuVSLAM (GPU) ──► /visual_odom
  OAK-D Pro IMU ─────┘      ~9% GPU           (30 Hz VO)
       400 Hz                                      │
                                                   ▼
  Wheel/Leg Odom ──────────► robot_localization (EKF)
       50 Hz                        │
                                    ▼
                              /odometry/filtered
                                    │
  Livox Mid-360 ───────┐           │
       10 Hz            │           │
                        ├───► RTAB-Map (mapping mode)
  OAK-D Pro Depth ─────┘      ~5 Hz, 2 CPU cores
                                    │
                         ┌──────────┴──────────┐
                         ▼                     ▼
                    /map (OccupancyGrid)   Loop Closure
                    (for Nav2)             (drift correction)
```

### 3.2 ROS 2 Topic Map

| Topic | Message Type | Publisher | Subscriber(s) | Rate |
|-------|-------------|-----------|---------------|------|
| `/oak/rgb/image` | `sensor_msgs/Image` | OAK-D driver | YOLO node, SegFormer node, cuVSLAM | 30 Hz |
| `/oak/stereo/depth` | `sensor_msgs/Image` | OAK-D driver | ESS node, RTAB-Map, costmap | 30 Hz |
| `/oak/stereo/left` | `sensor_msgs/Image` | OAK-D driver | cuVSLAM | 30 Hz |
| `/oak/stereo/right` | `sensor_msgs/Image` | OAK-D driver | cuVSLAM | 30 Hz |
| `/oak/imu` | `sensor_msgs/Imu` | OAK-D driver | cuVSLAM, EKF | 400 Hz |
| `/livox/pointcloud` | `sensor_msgs/PointCloud2` | Livox driver | RTAB-Map | 10 Hz |
| `/livox/scan` | `sensor_msgs/LaserScan` | Livox driver | Nav2 costmap | 10 Hz |
| `/detections` | `vision_msgs/Detection3DArray` | YOLO node | Behavior tree, grasp planner | 30 Hz |
| `/depth/metric` | `sensor_msgs/Image` | ESS node | Grasp planner | 30 Hz |
| `/terrain/mask` | `sensor_msgs/Image` | SegFormer node | Terrain costmap | 5 Hz |
| `/visual_odom` | `nav_msgs/Odometry` | cuVSLAM | EKF | 30 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | Nav2, RTAB-Map | 50 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | Nav2, coverage planner | 1 Hz |
| `/grasp_pose` | `geometry_msgs/PoseStamped` | Grasp planner | Arm controller | Event |

---

## 4. Latency Budget

### 4.1 Detection-to-Grasp Pipeline (Critical Path)

| Stage | Target | Estimated | Method |
|-------|--------|-----------|--------|
| Camera frame acquisition | - | 33 ms (1 frame @ 30 FPS) | Hardware |
| YOLO26s litter detection | <30 ms | 5-10 ms | TensorRT INT8 |
| Stereo depth (ESS or on-device) | <15 ms | ~20 ms (ESS) / 0 ms (VPU) | TensorRT / VPU |
| ROI crop + resize to 224×224 | <5 ms | <2 ms | GPU memcpy |
| GR-ConvNet v2 grasp planning | <50 ms | 5-10 ms | TensorRT INT8 |
| 2D→3D projection | <5 ms | <2 ms | Intrinsics math |
| Arm IK (analytical) | <5 ms | <1 ms | Closed-form 2-link |
| **Total perception latency** | **<200 ms** | **~50-75 ms** | |
| Arm physical execution | - | 500-2000 ms | Mechanical limit |

**The perception pipeline is not the bottleneck.** Arm physical movement dominates. For static ground litter, even 200 ms grasp planning is acceptable since litter does not move.

### 4.2 Segmentation Pipeline

| Stage | Target | Estimated |
|-------|--------|-----------|
| Camera frame | - | 33 ms |
| SegFormer-B0 inference | <50 ms | ~25 ms (TRT FP16) |
| Costmap update | <10 ms | ~5 ms |
| **Total** | **<100 ms** | **~63 ms @ 5 Hz** |

### 4.3 SLAM/Localization Pipeline

| Stage | Target | Estimated |
|-------|--------|-----------|
| cuVSLAM visual odometry | <33 ms | ~10 ms (GPU) |
| EKF sensor fusion | <5 ms | <1 ms |
| RTAB-Map mapping cycle | <200 ms | ~100-150 ms @ 5 Hz |
| Loop closure (async) | <1 s | 200-500 ms (background) |

---

## 5. Memory Budget (8 GB Shared)

| Component | GPU Memory | CPU Memory | Total | Notes |
|-----------|-----------|------------|-------|-------|
| OS + ROS 2 runtime | - | 1,200 MB | 1,200 MB | Ubuntu 22.04 + JetPack + DDS |
| YOLO26s (TRT INT8) | 150 MB | 50 MB | 200 MB | ~8M params, INT8 weights |
| SegFormer-B0 (TRT FP16) | 100 MB | 50 MB | 150 MB | 3.8M params |
| Isaac ROS ESS (TRT) | 200 MB | 50 MB | 250 MB | 17M params, confidence maps |
| GR-ConvNet v2 (TRT INT8) | 50 MB | 30 MB | 80 MB | 1.9M params, on-demand |
| cuVSLAM | 150 MB | 50 MB | 200 MB | GPU-accelerated tracking |
| RTAB-Map | 50 MB | 500 MB | 550 MB | Map DB grows; memory-managed |
| robot_localization (EKF) | - | 50 MB | 50 MB | Pure CPU |
| Camera driver + buffers | 100 MB | 150 MB | 250 MB | RGB + stereo + depth frames |
| LiDAR driver + buffers | - | 100 MB | 100 MB | Point cloud buffers |
| Nav2 + behavior tree | - | 250 MB | 250 MB | Costmaps, planner state |
| ROS 2 framework overhead | - | 300 MB | 300 MB | DDS, tf2, logging |
| **TOTAL** | **~800 MB** | **~2,780 MB** | **~3,580 MB** | |
| **Remaining headroom** | | | **~4,600 MB (57%)** | Comfortable margin |

**Analysis:** The full perception + navigation stack uses under 3.6 GB of the 8 GB budget. This leaves 4.6 GB for the locomotion controller, RL policy, map growth, and OS swap buffer. Memory is not the bottleneck.

---

## 6. Model Specifications Table

| Model | Task | Params | Input | FPS (Jetson TRT) | Memory | Precision | License | Phase |
|-------|------|--------|-------|-------------------|--------|-----------|---------|-------|
| **YOLO26s** | Litter detection | 8M | 640×640 RGB | ~260 (INT8) | 200 MB | INT8 | AGPL-3.0* | MVP |
| **YOLO26n** | Litter detection (alt) | 2.3M | 640×640 RGB | ~370 (INT8) | 100 MB | INT8 | AGPL-3.0* | MVP |
| **SegFormer-B0** | Terrain segmentation | 3.8M | 512×512 RGB | 30-50 (FP16) | 150 MB | FP16 | NVIDIA Research** | MVP |
| **Isaac ROS ESS** | Stereo depth | 17M | 640×480 stereo pair | 20-40 (FP16) | 250 MB | FP16 | NVIDIA (free) | MVP |
| **GR-ConvNet v2** | Grasp planning | 1.9M | 224×224 RGB-D | 100-200 (INT8) | 80 MB | INT8 | Permissive*** | MVP |
| **cuVSLAM** | Visual odometry | N/A (binary) | 640×360 stereo + IMU | 30 | 200 MB | Native | Apache-2.0 | MVP |
| **DA V2 Small** | Monocular depth (nav) | 24.8M | 364×364 RGB | 25 (TRT) | 640 MB | FP16 | Apache-2.0 | Phase 2 |
| **VGN** | 6-DOF grasping | ~1M | TSDF volume | 100 | 100 MB | FP32 | BSD-3 | Phase 2 |
| **DA V3 Small** | Metric monocular depth | ~25M | 364×364 RGB | est. 25-40 | ~650 MB | FP16 | TBD | Phase 3 |

*AGPL-3.0: Compatible with our AGPL-3.0 codebase. Enterprise license (~$2-5K/yr) needed if we later change project license.
**NVIDIA Source Code License for NVlabs weights; HuggingFace community weights may differ. TAO-trained models under NVIDIA terms. PIDNet-S (MIT) available as drop-in alternative.
***GR-ConvNet v2 repo has no explicit license file — verify with authors before commercial deployment.

---

## 7. Deployment Strategy

### 7.1 TensorRT Optimization Pipeline

```
Training (cloud/desktop GPU)
         │
         ▼
PyTorch model (.pt / .pth)
         │
         ▼
ONNX export (torch.onnx.export)
         │
         ▼
TensorRT engine build on Jetson (trtexec)
  ├── FP16 (default, 2-3× speedup, <1% accuracy loss)
  ├── INT8 (calibration required, 3-5× speedup, 1-2% accuracy loss)
  └── Engine file cached per hardware (.engine)
         │
         ▼
ROS 2 inference node (Isaac ROS DNN Inference or custom)
```

**Key rules:**
- **Always build TensorRT engines on the target Jetson.** Engine files are hardware-specific.
- **INT8 calibration** uses a representative dataset (500-1000 images). If accuracy drops >2% mAP, use FP16.
- **YOLO26** exports NMS-free (no post-processing operator needed in TensorRT).
- **SegFormer-B0** does not support INT8 via TAO as of 2025. Use FP16.

### 7.2 Isaac ROS Container Strategy

NVIDIA provides pre-built Docker containers for Jetson with all Isaac ROS packages:

```
Isaac ROS Docker Container (JetPack 6.x base)
├── isaac_ros_visual_slam (cuVSLAM)
├── isaac_ros_dnn_stereo_depth (ESS)
├── isaac_ros_segformer (SegFormer inference)
├── isaac_ros_object_detection (YOLO/DETR inference)
├── Nav2 (pre-configured)
└── rtabmap_ros (via apt)
```

**Development workflow:**
1. Development and training on desktop (x86 + RTX GPU)
2. Model export to ONNX
3. Deploy to Jetson via Isaac ROS container
4. Build TensorRT engines on first boot
5. ROS 2 launch file orchestrates all nodes

### 7.3 Model Versioning

| Model | Version Scheme | Storage | Update Mechanism |
|-------|---------------|---------|------------------|
| YOLO26s | `yolo26s_litter_v{N}.engine` | `/opt/models/detection/` | OTA via fleet management |
| SegFormer-B0 | `segformer_b0_terrain_v{N}.engine` | `/opt/models/segmentation/` | OTA |
| GR-ConvNet v2 | `grconvnet_v2_grasp_v{N}.engine` | `/opt/models/grasping/` | OTA |
| ESS | `ess_stereo_v{N}.engine` | `/opt/models/depth/` | Isaac ROS container update |
| cuVSLAM | Binary in Isaac ROS | Container image | Container update |

---

## 8. GPU Utilization Timeline

```
                  GPU Usage During Operation Cycle
═══════════════════════════════════════════════════════════════

Walking (continuous):
┌─────────────────────────────────────────────────────────────┐
│  YOLO26s detection          ████████████   ~20%             │
│  SegFormer-B0 (5 Hz)        ██████         ~10%             │
│  cuVSLAM (30 Hz)            █████          ~9%              │
│  RTAB-Map features          ███            ~5%              │
│  ESS stereo depth           ████           ~8%              │
│                                                             │
│  TOTAL                      ████████████████████  ~52%      │
│  HEADROOM                   ████████████████████  ~48%      │
└─────────────────────────────────────────────────────────────┘

Picking (on-demand, robot stopped):
┌─────────────────────────────────────────────────────────────┐
│  YOLO26s (still running)    ████████████   ~20%             │
│  GR-ConvNet v2 (burst)      ████           ~8% (5-10ms)    │
│  ESS depth (single frame)   ██             ~4%              │
│  cuVSLAM (reduced)          ███            ~5%              │
│                                                             │
│  TOTAL (peak burst)         █████████████████   ~37%        │
│  HEADROOM                   ███████████████████████  ~63%   │
└─────────────────────────────────────────────────────────────┘

Note: SegFormer paused during picking (terrain doesn't change).
      GR-ConvNet runs on-demand, not continuously.
```

---

## 9. Phase Plan

### Phase 1 — MVP (Months 1-3): What Ships First

| Component | Model | Status | Ships With |
|-----------|-------|--------|------------|
| Litter detection | YOLO26s (TRT INT8) | Ready to train | Custom litter dataset (TACO + RoLID-11K) |
| Stereo depth (grasping) | OAK-D Pro on-device VPU | Off-the-shelf | Zero GPU cost |
| Grasp planning | GR-ConvNet v2 (TRT INT8) | Needs TRT export | Pre-trained weights, fine-tune on litter |
| Visual odometry | cuVSLAM (Isaac ROS) | Off-the-shelf | Native Jetson integration |
| Mapping | RTAB-Map (mapping mode) | Off-the-shelf | Stereo + LiDAR input |
| State estimation | robot_localization (EKF) | Off-the-shelf | Configuration only |
| Terrain awareness | **Geometric rules** (depth → traversable/obstacle) | Simple | No ML needed |

**MVP avoids:** Learned terrain segmentation, monocular depth, 6-DOF grasping, visual servoing.

**MVP cost:** $1,148 sensors + $249 compute = **$1,397 perception hardware**.

### Phase 2 — Enhanced Perception (Months 3-6)

| Addition | Model | Why |
|----------|-------|-----|
| Terrain segmentation | SegFormer-B0 (TRT FP16) | Speed-dependent navigation on grass vs pavement |
| Isaac ROS ESS | Stereo depth with confidence | Better depth quality + confidence filtering for grasps |
| DA V2 Small | Monocular dense depth | Dense depth for close-range obstacle avoidance |
| Hybrid gripper | Suction + fingers | YOLO class drives mode: rigid → fingers, flat → suction |

### Phase 3 — Advanced Capabilities (Months 6-12)

| Addition | Model | Why |
|----------|-------|-----|
| 6-DOF grasping | VGN (TSDF input) | Objects against walls, in bins, inclined surfaces |
| DA V3 Metric | Metric monocular depth | Replace hardware stereo for some use cases |
| Visual servoing | ViSP / custom PID | Precise arm alignment on close approach |
| Closed-loop grasping | Tactile feedback | Re-grasp on failure, force-limited for fragile items |

---

## 10. Open Questions

### Must Resolve Before Pilot

| # | Question | Impact | How to Resolve |
|---|----------|--------|----------------|
| 1 | **GR-ConvNet v2 license** — repo has no LICENSE file | Blocks commercial deployment | Email authors (Sulabh Kumra, IITD) |
| 2 | **Isaac ROS ESS on Orin Nano Super** — most benchmarks on AGX Orin | May be slower than estimated | Benchmark on actual hardware |
| 3 | **YOLO26s AGPL-3.0 in production** — our project is AGPL-3.0 today, but future license change blocked | Budget ~$2-5K/yr for Ultralytics Enterprise if needed | Decision at Series A |
| 4 | **OAK-D Pro 0.7m minimum depth** — too far for close-range grasp verification | May miss items directly under robot | Test; consider OAK-D S2 (min 0.2m) as supplement |
| 5 | **cuVSLAM outdoor robustness** — limited field testing data in parks | Could lose tracking on featureless grass | Fallback: RTAB-Map in full SLAM mode (+1 CPU core) |

### Needs Prototyping

| # | Question | What We Don't Know Yet |
|---|----------|----------------------|
| 6 | **GR-ConvNet v2 on outdoor litter** — trained on tabletop objects (Cornell/Jacquard) | Will it generalize to ground-level litter in variable lighting? |
| 7 | **SegFormer-B0 on 8 terrain classes** — pre-trained on ADE20K (150 classes) | How much accuracy after fine-tuning on RUGD + RELLIS-3D? |
| 8 | **Multi-model GPU contention** — 5 models sharing 1024 CUDA cores | Do CUDA streams prevent latency spikes? Need profiling with `nsys`. |
| 9 | **Orbbec Gemini 345Lg** — IP67, 100+ klux, GMSL2. Announced CES 2026 | Pricing TBD. Could replace OAK-D Pro if competitive. Contact Orbbec. |
| 10 | **IR projector in direct sunlight** — OAK-D Pro may degrade | How much does active stereo degrade at noon in summer? |

---

## Appendix A: Updated Architecture Diagram vs. Original

The following table summarizes where the latest research updates the original `autonomy-stack-architecture.md`:

| Component | Original (v1.0, Feb 10) | Updated (v1.1, Feb 11) | Reason |
|-----------|------------------------|------------------------|--------|
| Detection | YOLO11n | **YOLO26s** | STAL (Small-Target-Aware Label Assignment), NMS-free, newer |
| Terrain seg | PIDNet-S | **SegFormer-B0** (primary), PIDNet-S (fallback) | NVIDIA ecosystem (TAO, Isaac ROS, TensorRT), better generalization |
| SLAM | RTAB-Map (full SLAM) | **cuVSLAM + RTAB-Map** (hybrid) | cuVSLAM: 9% GPU, 5% CPU vs RTAB-Map full: 2-3 cores. Frees CPU for Nav2 |
| Stereo depth | OAK-D Pro VPU only | OAK-D Pro VPU + **Isaac ROS ESS** (Phase 2) | ESS adds confidence maps for grasp filtering |
| Monocular depth | DA V2 Small (backup) | DA V2 Small (Phase 2, navigation) | Role clarified: supplementary dense depth, not primary |
| Grasp planning | Rule-based primitives | **GR-ConvNet v2** (Phase 1+), rule-based (Phase 0) | 95.4% success rate vs ~65% heuristic; 5-10ms TRT |
| Nav LiDAR | RPLidar A1 (Phase 1) | **Livox Mid-360** (Phase 1) | 360° × 59° FOV, IP67, 70m range, proven on quadrupeds |

## Appendix B: Key Repositories

| Component | Repository | License |
|-----------|-----------|---------|
| YOLO26 | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | AGPL-3.0 |
| SegFormer | [NVlabs/SegFormer](https://github.com/NVlabs/SegFormer) | NVIDIA Research |
| Isaac ROS Visual SLAM | [NVIDIA-ISAAC-ROS/isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) | Apache-2.0 |
| Isaac ROS ESS | [NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth) | Apache-2.0 |
| Isaac ROS SegFormer | [NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation) | Apache-2.0 |
| RTAB-Map | [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | BSD-3 |
| GR-ConvNet | [skumra/robotic-grasping](https://github.com/skumra/robotic-grasping) | Verify |
| Depth Anything V2 | [DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) | Apache-2.0 |
| VGN | [ethz-asl/vgn](https://github.com/ethz-asl/vgn) | BSD-3 |
| OAK-D ROS 2 | [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros) | MIT |
| robot_localization | [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | BSD-3 |
| PIDNet (fallback) | [XuJiacong/PIDNet](https://github.com/XuJiacong/PIDNet) | MIT |

## Appendix C: License Compatibility

Our project is AGPL-3.0. All dependencies are compatible:

| License | Compatible? | Components |
|---------|-------------|------------|
| Apache-2.0 | Yes | cuVSLAM, ESS, Isaac ROS, DA V2, CREStereo |
| BSD-3 / BSD-2 | Yes | RTAB-Map, VGN, robot_localization |
| MIT | Yes | PIDNet, DDRNet, OAK-D driver |
| AGPL-3.0 | Yes (same) | YOLO26 (Ultralytics) |
| NVIDIA proprietary (free) | Yes (runtime) | TensorRT, JetPack, cuVSLAM binary |
| NVIDIA Research | Verify | SegFormer NVlabs weights (commercial inquiry needed) |

---

*This document consolidates findings from six research documents into a single architecture reference. It supersedes perception-related sections of autonomy-stack-architecture.md v1.0. Source research: edge-detection-sota-2026.md, visual-slam-vo-research-2026.md, semantic-segmentation-sota-2026.md, depth-grasp-cameras-2026.md, unitree-opensource-reusability-analysis-2026.md.*
