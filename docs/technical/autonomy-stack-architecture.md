# CleanWalker Robotics — Autonomy Stack Architecture

**Version:** 1.0
**Date:** 2026-02-10
**Status:** R&D Foundation Document
**Hardware Target:** NVIDIA Jetson Orin Nano Super (8 GB, 67 TOPS)
**Middleware:** ROS 2 Humble Hawksbill (LTS through May 2027)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Perception Module](#2-perception-module)
3. [Localization & Mapping (SLAM)](#3-localization--mapping-slam)
4. [Navigation & Path Planning](#4-navigation--path-planning)
5. [Object Approach & Grasp Planning](#5-object-approach--grasp-planning)
6. [Locomotion Control](#6-locomotion-control)
7. [Decision Engine / Behavior System](#7-decision-engine--behavior-system)
8. [Integration Architecture](#8-integration-architecture)
9. [Recommended Stack](#9-recommended-stack)
10. [Development Roadmap](#10-development-roadmap)

---

## 1. System Overview

### 1.1 Full Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        CLEANWALKER AUTONOMY STACK                          │
│                                                                             │
│  ┌──────────┐   ┌──────────────┐   ┌──────────┐   ┌───────────────────┐   │
│  │ PERCEIVE │──▸│    DECIDE    │──▸│   PLAN   │──▸│      ACT          │   │
│  └──────────┘   └──────────────┘   └──────────┘   └───────────────────┘   │
│       │                │                 │                   │              │
│  ┌────┴─────┐   ┌──────┴──────┐   ┌─────┴─────┐   ┌───────┴────────┐    │
│  │ Litter   │   │  Behavior   │   │ Coverage  │   │ Locomotion     │    │
│  │ Detection│   │  Tree       │   │ Planner   │   │ Controller     │    │
│  │          │   │             │   │           │   │                │    │
│  │ Depth    │   │  State:     │   │ Nav2      │   │ Arm IK +       │    │
│  │ Mapping  │   │  Patrol     │   │ Path      │   │ Grasp          │    │
│  │          │   │  Detect     │   │ Planner   │   │ Controller     │    │
│  │ Terrain  │   │  Approach   │   │           │   │                │    │
│  │ Classify │   │  Pick       │   │ Obstacle  │   │ Motor          │    │
│  │          │   │  Deposit    │   │ Avoidance │   │ Drivers        │    │
│  │ SLAM     │   │  Dock       │   │           │   │ (CAN bus)      │    │
│  └──────────┘   └─────────────┘   └───────────┘   └────────────────┘    │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     SAFETY SUPERVISOR (independent)                  │   │
│  │  E-stop monitor │ Collision avoidance │ Watchdog │ Force limiter    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Data Flow Between Modules

```
Stereo Camera (OAK-D Pro)    RPLidar A1        IMU (OAK-D Pro internal)
     │ 30 Hz                   │ 10 Hz              │ 200 Hz
     ├──▸ /camera/color        ├──▸ /scan           ├──▸ /imu/data
     ├──▸ /camera/depth        │                     │
     │                         │                     │
     ▼                         ▼                     ▼
┌─────────────┐        ┌─────────────┐       ┌──────────────┐
│ Litter Det. │        │ SLAM Node   │◂──────│ State Est.   │
│ (YOLO11n)   │        │ (RTAB-Map)  │       │ (EKF)        │
│ 15-30 FPS   │        │ 10 Hz       │       │ 200 Hz       │
└──────┬──────┘        └──────┬──────┘       └──────┬───────┘
       │                      │                      │
       │ /detections          │ /map, /odom          │ /odometry/filtered
       │ (Detection3DArray)   │ (OccupancyGrid,      │ (Odometry)
       │                      │  Odometry)            │
       ▼                      ▼                      ▼
┌──────────────────────────────────────────────────────────┐
│              BEHAVIOR TREE (BehaviorTree.CPP)             │
│                                                           │
│  Subscribes to: /detections, /map, /odom, /battery_state │
│  Publishes to:  /cmd_vel, /arm/goal_pose, /navigate_to   │
│  Controls:      Coverage planner, Nav2, arm controller    │
└──────────────────┬────────────────────┬──────────────────┘
                   │                    │
         /goal_pose                /arm/goal_pose
                   │                    │
                   ▼                    ▼
          ┌────────────┐      ┌─────────────────┐
          │   Nav2     │      │  MoveIt2        │
          │  (TEB/DWB) │      │  Arm Controller │
          │  20 Hz     │      │  50 Hz          │
          └─────┬──────┘      └────────┬────────┘
                │                      │
          /cmd_vel                /joint_commands
                │                      │
                ▼                      ▼
   ┌───────────────────────────────────────────┐
   │        MOTOR CONTROLLER (Rust firmware)    │
   │        CAN bus @ 1 kHz to 12 leg motors   │
   │        + 3 arm servos via Dynamixel bus    │
   └───────────────────────────────────────────┘
```

### 1.3 Real-Time Requirements

| Module | Update Rate | Max Latency | Priority | Notes |
|--------|------------|-------------|----------|-------|
| Safety supervisor | 1 kHz | 1 ms | CRITICAL | Hardwired e-stop path, independent MCU |
| Motor control loop | 1 kHz | 1 ms | CRITICAL | Rust firmware on dedicated MCU |
| IMU / state estimation | 200 Hz | 5 ms | High | EKF fusion of IMU + wheel odometry |
| LiDAR scan processing | 10 Hz | 50 ms | High | Obstacle detection safety layer |
| Litter detection (YOLO) | 15-30 Hz | 66 ms | Medium | Can tolerate 1-2 dropped frames |
| SLAM / map update | 1-10 Hz | 200 ms | Medium | Loop closure can be async |
| Path planning | 1-5 Hz | 500 ms | Medium | Replan on new obstacle or detection |
| Behavior tree tick | 10 Hz | 100 ms | Medium | State machine transitions |
| Coverage planner | 0.1 Hz | 5 s | Low | Recalculate on area completion |
| Fleet telemetry upload | 0.1 Hz | 10 s | Low | 4G LTE, best-effort |

---

## 2. Perception Module

### 2.1 Litter Detection

This is the core differentiating capability of CleanWalker. The robot must detect 10 categories of outdoor litter (plastic bottles, cans, cigarette butts, paper, plastic bags, food wrappers, glass bottles, cardboard, styrofoam, other trash) at ranges of 0.3-5 m in variable outdoor lighting.

#### Model Comparison for Jetson Orin Nano Super

| Model | Params | GFLOPs | mAP50-95 (COCO) | FPS on Orin Nano (INT8/TRT) | Size (INT8) | Open Source | Notes |
|-------|--------|--------|------------------|-----------------------------|-------------|-------------|-------|
| **YOLO11n** | 2.6M | 6.5 | 39.5 | **55-70 FPS** | ~2 MB | [Ultralytics](https://github.com/ultralytics/ultralytics) | **Recommended primary.** Best params/accuracy ratio. Successor to YOLOv8n with C2PSA attention. |
| YOLO11s | 9.4M | 21.5 | 47.0 | 30-40 FPS | ~5 MB | [Ultralytics](https://github.com/ultralytics/ultralytics) | Better accuracy, still real-time. Use if n-variant underperforms on small litter. |
| YOLOv8n | 3.2M | 8.7 | 37.3 | 45-62 FPS | ~2 MB | [Ultralytics](https://github.com/ultralytics/ultralytics) | Mature, well-tested. Fallback if YOLO11 has issues. |
| YOLOv10n | 2.3M | 6.7 | 38.5 | 50-65 FPS | ~2.5 MB | [THU-MIG/yolov10](https://github.com/THU-MIG/yolov10) | NMS-free design (lower latency), slightly fewer features in Ultralytics. |
| YOLOv9t | 2.0M | 7.7 | 38.3 | 40-55 FPS | ~3 MB | [WongKinYiu/yolov9](https://github.com/WongKinYiu/yolov9) | PGI + GELAN architecture. Interesting but less Ultralytics ecosystem support. |
| RT-DETR-l | 32M | 110 | 53.0 | 8-12 FPS | ~40 MB | [Ultralytics](https://github.com/ultralytics/ultralytics) | Transformer-based, much heavier. Does NOT fit our compute budget. |
| RT-DETR-R18 | 20M | 60 | 46.5 | 12-18 FPS | ~25 MB | [lyuwenyu/RT-DETR](https://github.com/lyuwenyu/RT-DETR) | Lightest DETR variant. Still too heavy for multi-model workload on Orin Nano. |

**Decision: YOLO11n (primary), YOLO11s (if accuracy insufficient), YOLOv8n (fallback).**

RT-DETR models are too heavy for our compute budget when sharing the GPU with depth estimation, SLAM, and navigation. YOLO11n at 55-70 FPS on INT8/TensorRT leaves ample GPU headroom for concurrent workloads.

#### Framework Choice

| Framework | Pros | Cons | Verdict |
|-----------|------|------|---------|
| **Ultralytics** | Integrated training/export/deploy pipeline, TensorRT export, ROS2 wrapper available, massive community, active development | License (AGPL-3.0 for open, Enterprise for commercial) | **Recommended.** Our project is AGPL-3.0, so license is compatible. |
| MMDetection | More model zoo variety, academic-grade | Complex config system, heavier dependency chain, slower iteration | Useful for experiments, not for production deploy. |
| Detectron2 | Facebook/Meta backing, good segmentation | Less focus on real-time/edge, complex setup | Overkill for our bounding-box detection task. |

#### Training Data Strategy

**Phase 1 — Prototype (target: 30,000 images)**

| Source | Images | Type | Cost | Action |
|--------|--------|------|------|--------|
| [TACO](http://tacodataset.org/) | 5,200 | Instance segmentation → bbox | Free | Download, convert COCO → YOLO format (pipeline exists in `ml/training/data/`) |
| [Litter Dataset](https://github.com/AgaMiko/waste-datasets-review) | 14,000 | Bounding boxes, 24 classes | Free | Download, remap classes to our 10-class taxonomy |
| [Drinking Waste](https://github.com/AgaMiko/waste-datasets-review) | ~10,000 | Bounding boxes, 4 classes | Free | Merge can/bottle classes with ours |
| Copy-paste augmentation | +5,000-10,000 | Synthetic | Free | Cut litter objects, paste onto clean park/sidewalk backgrounds |
| Custom collection | 2,000-5,000 | Raw photos, phone camera | Free (labor) | Team walks through parks, photographs litter in situ |

**Phase 2 — Production (target: 50,000+ images)**

| Source | Images | Cost |
|--------|--------|------|
| [OpenLitterMap](https://openlittermap.com/) (filtered + re-annotated) | 10,000-20,000 | $300-700 (annotation) |
| Stable Diffusion augmentation (underrepresented classes) | 2,000-5,000 | $5-20 (GPU time) |
| Fleet feedback loop (robot cameras capture novel litter) | Ongoing | Marginal (annotation labor) |

**Annotation pipeline:** Roboflow auto-label with pre-trained YOLO → human review in CVAT → export YOLO format. Budget: $200-500 for 20k images.

### 2.2 Depth Estimation

The robot needs per-pixel depth data for:
1. Estimating distance to detected litter (grasp planning)
2. Obstacle detection and avoidance
3. Feeding SLAM pipeline

#### Approach: Hardware Stereo (OAK-D Pro)

The OAK-D Pro provides hardware stereo depth at 640x480 @ 30 FPS with active IR structured light projection for outdoor reliability. This is **strongly preferred over monocular depth estimation** for our use case:

| Method | Pros | Cons | FPS on Orin Nano | Verdict |
|--------|------|------|------------------|---------|
| **OAK-D Pro stereo (hardware)** | Real metric depth, no GPU cost (runs on camera SoC), works outdoors with IR projector, 0.2-15m range | Fixed baseline limits close-range accuracy, ~$399 | 30 FPS (free — runs on camera chip) | **Recommended.** Zero GPU overhead. |
| Depth Anything V2 (Small) | State-of-art monocular depth, good generalization | Relative depth only (not metric without calibration), ~5-10 FPS on Orin Nano GPU, competes for GPU with YOLO | 5-10 FPS (GPU) | Backup for close-range refinement. |
| MiDaS v3.1 Small | Lightweight monocular depth | Lower quality than Depth Anything, still relative depth | 10-15 FPS (GPU) | Not recommended; Depth Anything V2 is better. |
| ZoeDepth | Metric monocular depth | Heavier model, 3-5 FPS on Orin Nano | 3-5 FPS (GPU) | Too slow for concurrent workload. |

**Decision: Use OAK-D Pro hardware stereo as the primary depth source.** It runs on the camera's own VPU (4 TOPS), consuming zero Jetson GPU. Reserve monocular depth (Depth Anything V2 Small) as an optional refinement layer for close-range grasp planning if hardware stereo proves insufficient below 0.5m.

**Repos:**
- OAK-D ROS2 driver: [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)
- Depth Anything V2: [DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2)

### 2.3 Semantic Segmentation — Terrain Classification

The robot needs to distinguish traversable terrain from non-traversable areas:

| Terrain Class | Description | Traversability |
|---------------|-------------|----------------|
| Paved path | Concrete, asphalt, brick | Full speed |
| Grass | Lawn, maintained grass | Reduced speed (0.3 m/s) |
| Gravel / dirt | Packed earth, loose gravel | Reduced speed |
| Stairs / curb | Steps, sharp elevation changes | Avoid (unless step-capable) |
| Water / mud | Puddles, wet mud | Avoid |
| Vegetation | Bushes, flower beds | Avoid |
| Road / parking | Vehicle traffic areas | Avoid (safety) |

#### Model Options

| Model | Backbone | mIoU (Cityscapes) | FPS on Orin Nano (INT8) | Params | Repo |
|-------|----------|-------------------|-------------------------|--------|------|
| **PIDNet-S** | PIDNet | 78.8 | 25-35 FPS | 7.6M | [XuJiacong/PIDNet](https://github.com/XuJiacong/PIDNet) |
| DDRNet-23-slim | DDRNet | 77.8 | 30-40 FPS | 5.7M | [ydhongHIT/DDRNet](https://github.com/ydhongHIT/DDRNet) |
| BiSeNetV2 | BiSeNet | 75.3 | 35-50 FPS | 3.4M | [CoinCheung/BiSeNet](https://github.com/CoinCheung/BiSeNet) |
| SegFormer-B0 | MiT-B0 | 76.2 | 15-20 FPS | 3.8M | [NVlabs/SegFormer](https://github.com/NVlabs/SegFormer) |

**Decision: PIDNet-S for terrain segmentation.** Best accuracy among real-time models. Run at reduced resolution (512x256) and reduced rate (5 Hz) to minimize GPU load. Terrain classification does not need per-frame updates since terrain changes slowly.

**Training data for terrain:** Fine-tune on a combination of:
- Cityscapes (urban scenes, 5,000 images)
- RUGD (Robot Unstructured Ground Dataset, off-road terrain, 37 classes)
- Custom park/sidewalk images (500-1,000, collected during litter data collection)

**Phase 1 shortcut:** Skip dedicated terrain segmentation. Use the depth map + LiDAR to classify "traversable" vs "obstacle" with geometric rules (flat surface = traversable, above knee height = obstacle). Add learned terrain segmentation in Phase 2 when the robot needs to handle grass vs. path at different speeds.

---

## 3. Localization & Mapping (SLAM)

The robot must build a map of its operating area and localize itself within it for coverage planning and return-to-dock navigation. Outdoor park environments present challenges: repetitive visual features (grass, trees), variable lighting, GPS occlusion under tree canopy.

### 3.1 SLAM Options

#### Visual SLAM

| System | Sensor Input | Features | Runs on Jetson? | CPU Load | Repo |
|--------|-------------|----------|-----------------|----------|------|
| **RTAB-Map** | Stereo + depth + IMU + (optional) LiDAR | Visual + LiDAR SLAM, loop closure, robust outdoor, native ROS2 | **Yes.** Well-tested on Jetson. | Moderate (2-3 cores, 10 Hz) | [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) |
| ORB-SLAM3 | Stereo + IMU | Excellent accuracy, multi-map, relocalization | Yes, with optimization. | High (3-4 cores at 30 Hz, can run at 10 Hz) | [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) |
| OpenVINS | Stereo + IMU | VIO only (no mapping), extremely lightweight | Yes, very lightweight. | Low (1 core) | [rpng/open_vins](https://github.com/rpng/open_vins) |
| Stella-VSLAM | Stereo | ORB-SLAM2 derivative, MIT license (vs ORB-SLAM3 GPLv3) | Yes. | Moderate | [stella-cv/stella_vslam](https://github.com/stella-cv/stella_vslam) |

#### LiDAR SLAM (for v2 with Livox Mid-360)

| System | Sensor Input | Features | Runs on Jetson? | CPU Load | Repo |
|--------|-------------|----------|-----------------|----------|------|
| **KISS-ICP** | LiDAR point cloud | Minimal, fast, surprisingly accurate. Works with any LiDAR. | **Yes.** Very lightweight. | Low (1 core, <10% @ 10 Hz) | [PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) |
| LIO-SAM | LiDAR + IMU | Tightly-coupled LiDAR-inertial, very robust outdoors | Yes. | Moderate (2 cores) | [TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) |
| Cartographer | LiDAR (2D or 3D) + IMU | Google-backed, mature, good 2D SLAM | Yes. | Moderate | [cartographer-project/cartographer_ros](https://github.com/cartographer-project/cartographer_ros) |
| FAST-LIO2 | LiDAR + IMU | Extremely fast and accurate, direct method | Yes. | Low-Moderate | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) |

### 3.2 Sensor Fusion Strategy

```
                  ┌──────────────┐
                  │ RTK-GPS      │  (2cm outdoor, when available)
                  │ 10 Hz        │
                  └──────┬───────┘
                         │
                         ▼
┌──────────┐     ┌──────────────┐     ┌──────────┐
│ OAK-D Pro│────▸│ robot_locali-│◂────│ RPLidar  │
│ Stereo + │     │ zation_      │     │ A1       │
│ IMU      │     │ (EKF node)   │     │ 2D scan  │
└──────────┘     └──────┬───────┘     └──────────┘
                        │
                   /odometry/filtered
                        │
                        ▼
                 ┌──────────────┐
                 │  RTAB-Map    │
                 │  (builds map │
                 │   + loop     │
                 │   closure)   │
                 └──────────────┘
```

**Phase 1 (v1 prototype — OAK-D Pro + RPLidar A1):**
- **Primary:** RTAB-Map with stereo visual odometry from OAK-D Pro + 2D LiDAR from RPLidar A1
- **State estimation:** `robot_localization` package (ROS2) running an Extended Kalman Filter (EKF) fusing: stereo visual odometry, IMU, wheel/leg odometry
- **GPS:** RTK-GPS when available (outdoor, open sky) fused via `navsat_transform` node
- **Expected accuracy:** 0.1-0.5m in mapped areas, 1-2m in unmapped areas with GPS

**Phase 2 (v2 — add Livox Mid-360 3D LiDAR):**
- Add KISS-ICP or LIO-SAM for LiDAR odometry
- Fuse LiDAR odometry into the EKF alongside visual odometry
- RTAB-Map can consume both visual and LiDAR features for richer maps
- **Expected accuracy:** 0.05-0.2m consistently

### 3.3 Jetson Benchmark Estimates

| Configuration | CPU Usage | GPU Usage | RAM Usage | Total Latency |
|---------------|-----------|-----------|-----------|---------------|
| RTAB-Map (stereo + 2D LiDAR, 10 Hz) | 2 cores (~30%) | Minimal (feature extraction) | ~500 MB | 50-100 ms/frame |
| ORB-SLAM3 (stereo + IMU, 10 Hz) | 3 cores (~45%) | None | ~400 MB | 30-50 ms/frame |
| robot_localization EKF (200 Hz) | 0.5 core (~8%) | None | ~50 MB | <1 ms |
| KISS-ICP (LiDAR only, 10 Hz) | 1 core (~15%) | None | ~200 MB | 20-40 ms/frame |

**RTAB-Map is the recommended choice:** it handles both mapping and localization, supports multi-sensor input, has excellent ROS2 integration, and is proven on Jetson platforms. Its CPU load is manageable alongside our other workloads.

---

## 4. Navigation & Path Planning

### 4.1 Coverage Planning

The robot's primary task is to systematically cover an assigned area, detecting and collecting litter. This requires a **coverage path planner** — a module that generates a path visiting every point in the operating zone.

#### Coverage Algorithms

| Algorithm | Description | Pros | Cons | Best For |
|-----------|-------------|------|------|----------|
| **Boustrophedon (lawn-mower)** | Back-and-forth parallel sweeps across the area | Simple, complete coverage, easy to implement | Inefficient in complex shapes, many turns | Rectangular open areas (fields, parking lots) |
| **Spiral** | Inward or outward spiral pattern | Fewer turns than boustrophedon, good for convex areas | Poor coverage of concave/complex shapes | Circular areas, gardens |
| **Boustrophedon Cell Decomposition (BCD)** | Decomposes area into cells, runs boustrophedon in each | Handles arbitrary shapes including obstacles | More complex, requires prior map | **Recommended for parks.** Handles trees, benches, etc. |
| **Spanning Tree Coverage (STC)** | Grid-based, follows spanning tree of grid cells | Complete coverage guarantee, handles obstacles | Grid resolution limits precision | Alternative to BCD |

**Decision: Boustrophedon Cell Decomposition (BCD).**

This is the standard approach for robotic coverage in environments with obstacles. The algorithm:
1. Decomposes the free space (from the SLAM map) into trapezoidal cells using vertical cell decomposition
2. Plans a boustrophedon (back-and-forth) path within each cell
3. Connects cells via a shortest-path tour (TSP-like)

**Open-source implementations:**
- [FieldRobotEvent/fields2cover](https://github.com/Fields2Cover/Fields2Cover) — C++ library for coverage planning, supports BCD, swath generation, and path optimization. Has ROS2 wrapper.
- [nobleo/full_coverage_path_planner](https://github.com/nobleo/full_coverage_path_planner) — ROS2 coverage planner plugin for Nav2, implements BCD and STC.
- Custom implementation using the occupancy grid from RTAB-Map.

**Integration with Nav2:** The coverage planner generates a sequence of waypoints. Each waypoint is sent to Nav2 as a `NavigateToPose` action goal. Nav2 handles local obstacle avoidance and path following between waypoints.

### 4.2 Obstacle Avoidance & Local Planning (Nav2)

The [ROS 2 Navigation Stack (Nav2)](https://docs.nav2.org/) is the standard framework for autonomous robot navigation. It provides:

- **Global planner:** Computes optimal path from current position to goal on the costmap (A*, Dijkstra, Theta*, NavFn, or SmacPlanner)
- **Local planner (controller):** Follows the global path while avoiding dynamic obstacles in real-time
- **Costmap:** 2D grid fusing LiDAR, depth camera, and semantic data into traversability costs
- **Recovery behaviors:** Automatic recovery when stuck (backup, spin, wait)

#### Local Planner Comparison

| Planner | Type | Pros | Cons | Verdict |
|---------|------|------|------|---------|
| **DWB (Dynamic Window approach B)** | Velocity sampling | Simple, reliable, well-tested in Nav2, good for differential-drive-like platforms | Tuning-sensitive, can get stuck in narrow passages | **Recommended for Phase 1.** Simple and reliable. |
| **TEB (Timed Elastic Band)** | Trajectory optimization | Handles non-holonomic constraints well, smoother paths, better in tight spaces | Higher CPU usage, more parameters to tune | Recommended for Phase 2. Better path quality. |
| **MPPI (Model Predictive Path Integral)** | Sampling-based MPC | State-of-art, GPU-accelerable, handles complex dynamics | Newer, less battle-tested, requires good dynamics model | Future consideration. GPU-accelerated version ideal for Jetson. |
| **Regulated Pure Pursuit** | Path following | Very simple, low CPU, predictable | No obstacle avoidance (purely follows path) | Use as inner loop combined with DWB/TEB safety layer. |

**Decision: DWB for Phase 1, migrate to TEB or MPPI for Phase 2.**

Nav2 repo: [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)

### 4.3 Pedestrian-Aware Navigation

CleanWalker operates in public spaces with pedestrians, children, cyclists, and pets. The navigation system must implement social navigation behaviors per our product spec (1.5m clearance from adults, 2.0m from children/pets).

#### Approach

1. **Person detection:** Use the YOLO model with added person/child/pet classes (COCO pre-trained covers these). Run as a secondary detection head alongside litter detection (same model, additional classes).

2. **Social costmap layer:** Custom Nav2 costmap layer that inflates detected persons into high-cost zones:
   - Adult detected: 1.5m radius inflation in costmap
   - Child/pet detected: 2.0m radius inflation
   - Moving person: Predict trajectory 2s ahead, inflate predicted path

3. **Social force model (optional, Phase 2):** Implement the Helbing social force model to generate human-like avoidance behavior. The robot treats pedestrians as repulsive force sources.
   - Reference: [Social Force Model for Pedestrian Dynamics](https://journals.aps.org/pre/abstract/10.1103/PhysRevE.51.4282)
   - ROS2 implementation: [robotics-upo/social_nav2](https://github.com/robotics-upo/social_nav2)

4. **Speed modulation:**
   - No persons in sensor range: Normal speed (0.5 m/s operating, 1.5 m/s transit)
   - Person at 5-10m: Reduce to 0.3 m/s
   - Person at 2-5m: Reduce to 0.2 m/s, increase clearance
   - Person at <2m: Full stop, yield

---

## 5. Object Approach & Grasp Planning

### 5.1 Visual Servoing — Approaching Detected Litter

When the behavior tree transitions to "approach litter" state, the robot must navigate precisely to the detected litter item for grasping. This requires:

1. **Far approach (2-5m):** Nav2 handles navigation to a point ~0.5m from the detected litter position (projected from camera detection + depth into world coordinates).

2. **Near approach (0.5m):** Switch to **image-based visual servoing (IBVS)**. The arm-mounted wrist camera provides a close-up view. The controller drives the robot + arm to center the litter in the wrist camera frame and bring it to the grasping distance.

#### Visual Servoing Implementation

```
Wrist Camera (30 FPS)
     │
     ▼
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ Litter Det. │────▸│ Visual Servo │────▸│ Arm + Body  │
│ (YOLO11n on │     │ Controller   │     │ Velocity    │
│ wrist cam)  │     │ (P/PID)      │     │ Commands    │
└─────────────┘     └──────────────┘     └─────────────┘
```

The visual servo controller computes velocity commands to:
- Center the litter detection bounding box in the image
- Drive the arm end-effector to the correct pre-grasp distance
- Align gripper orientation based on litter shape

This is a classical IBVS problem. Implementation with [ViSP (Visual Servoing Platform)](https://visp.inria.fr/) library or custom PID controller on detection centroid.

**Repo:** [lagadic/visp](https://github.com/lagadic/visp) — C++ visual servoing library with ROS2 integration

### 5.2 Grasp Planning

Grasping diverse litter (bottles, cans, flat wrappers, cigarette butts) in unstructured outdoor environments is one of the hardest problems in the stack.

#### Grasp Strategy by Litter Type

| Litter Category | Shape | Grasp Strategy | Gripper Requirements |
|----------------|-------|----------------|---------------------|
| Plastic bottle | Cylindrical, rigid | Enveloping grasp around body | 2-3 finger, 60-80mm aperture |
| Can | Cylindrical, rigid | Enveloping grasp | Same as bottle |
| Glass bottle | Cylindrical, rigid, heavy | Enveloping grasp, higher force | 2-3 finger, secure grip |
| Cigarette butt | Tiny, cylindrical | Pinch grasp (fingertip) | Narrow fingertip capability |
| Paper / receipt | Flat, flexible | Scoop against ground + pinch | Compliant fingertips, ground scraping |
| Plastic bag | Flat, flexible, light | Scoop + bunch + pinch | Wide aperture, compliant |
| Food wrapper | Flat/crumpled, variable | Scoop or pinch depending on shape | Compliant fingertips |
| Cardboard | Flat, rigid, large | Edge grasp or top-down clamp | Wide aperture |
| Styrofoam | Lightweight, crushable | Gentle enveloping grasp | Force-limited |

#### Grasp Planning Approaches

| Approach | Description | Compute Cost | Accuracy | Repo |
|----------|-------------|-------------|----------|------|
| **Rule-based grasp primitives** | Pre-defined grasp poses per litter class (top-down for bottles, scoop for flat items) | Near zero | Moderate (80-85%) | Custom implementation |
| **GraspNet (6-DoF)** | Learned 6-DoF grasp pose prediction from point cloud | 100-500ms on Orin Nano | High (85-90%) | [graspnet/graspnet-baseline](https://github.com/graspnet/graspnet-baseline) |
| **Contact-GraspNet** | Contact-map-based grasp prediction | 200-800ms on Orin Nano | High (88-92%) | [NVlabs/contact_graspnet](https://github.com/NVlabs/contact_graspnet) |
| **AnyGrasp** | Generalizes to novel objects, NVIDIA-backed | 100-300ms on GPU | Very high (90-95%) | [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk) |

**Decision: Phase 1 — Rule-based grasp primitives. Phase 2 — GraspNet or AnyGrasp.**

**Rationale:** For the prototype, rule-based grasps are sufficient and dramatically simpler. The YOLO detection provides the litter class, and each class maps to a pre-defined grasp approach:

```python
GRASP_PRIMITIVES = {
    "plastic_bottle": TopDownGrasp(approach_angle=0, aperture=70mm),
    "can":            TopDownGrasp(approach_angle=0, aperture=65mm),
    "glass_bottle":   TopDownGrasp(approach_angle=0, aperture=75mm),
    "cigarette_butt": PinchGrasp(approach_angle=45, aperture=15mm),
    "paper":          ScoopGrasp(approach_angle=80, scrape=True),
    "plastic_bag":    ScoopGrasp(approach_angle=80, bunch=True),
    "food_wrapper":   ScoopGrasp(approach_angle=70, scrape=True),
    "cardboard":      EdgeGrasp(approach_angle=30, aperture=100mm),
    "styrofoam":      TopDownGrasp(approach_angle=0, aperture=80mm, force_limit=5N),
    "other_trash":    TopDownGrasp(approach_angle=0, aperture=70mm),
}
```

For Phase 2, AnyGrasp is the most promising: it generalizes well to novel objects, runs efficiently on NVIDIA GPUs, and is actively maintained by NVIDIA Research.

### 5.3 Arm Inverse Kinematics — MoveIt2

The 2-DOF arm (shoulder + elbow) + gripper servo needs inverse kinematics to translate desired end-effector (gripper) poses into joint angles.

**For a 2-DOF arm, analytical IK is trivial** — it's a planar 2-link manipulator with a closed-form solution. MoveIt2 is overkill for Phase 1 but becomes necessary if we upgrade to a 4-6 DOF arm.

#### Phase 1: Custom Analytical IK

```
Given: desired (x, z) position of gripper in arm base frame
Solve: shoulder_angle, elbow_angle using 2-link planar IK

shoulder_angle = atan2(z, x) - atan2(L2*sin(elbow_angle), L1 + L2*cos(elbow_angle))
elbow_angle = acos((x² + z² - L1² - L2²) / (2*L1*L2))
```

Implement as a lightweight ROS2 node (~100 lines of C++) that subscribes to `/arm/goal_pose` and publishes `/arm/joint_commands`.

#### Phase 2+: MoveIt2

When the arm is upgraded to 4-6 DOF for more dexterous manipulation:

- [MoveIt2](https://moveit.ros.org/) — Full motion planning framework for ROS2
- Repo: [moveit/moveit2](https://github.com/moveit/moveit2)
- Provides: IK solvers (KDL, TRAC-IK), motion planning (OMPL, Pilz), collision avoidance, trajectory execution
- Runs on Jetson Orin Nano: Yes, CPU-only planning is sufficient for a small arm
- Compute cost: ~1 core during planning, negligible during execution

---

## 6. Locomotion Control

### 6.1 Quadruped Gait Control

The CleanWalker uses a 12-DOF quadruped (3 joints per leg: hip abduction, hip flexion, knee flexion). Generating stable, terrain-adaptive walking gaits is a well-studied problem with two major approaches:

#### Model-Based Control (Classical)

| Approach | Description | Pros | Cons |
|----------|-------------|------|------|
| **Model Predictive Control (MPC)** | Optimizes body trajectory + foot placements over a prediction horizon | Physically grounded, interpretable, reliable on known terrain | Requires accurate dynamics model, struggles with novel terrain |
| **Whole-Body Control (WBC)** | Computes joint torques to achieve desired body motion + contact forces | Elegant, handles contacts well | Complex to implement, needs accurate inertial parameters |
| **Central Pattern Generators (CPG)** | Neural oscillator networks generating rhythmic gait patterns | Simple, biologically inspired, robust | Limited adaptability, hand-tuned parameters |

#### Reinforcement Learning (Learned)

| Approach | Description | Pros | Cons |
|----------|-------------|------|------|
| **RL policy (sim-to-real)** | Train locomotion policy in simulation, deploy on real robot | Handles diverse terrain, robust to disturbances, self-improving | Requires extensive simulation, sim-to-real gap, less interpretable |
| **Teacher-student RL** | Teacher policy in sim with privileged info → student policy from observations only | Best sim-to-real transfer, proven on Unitree/ANYmal | More complex training pipeline |

#### Recommendation

**Phase 1: Model-based MPC.** Start with a proven, interpretable controller. The MIT Cheetah MPC controller is well-documented and runs on our hardware.

**Phase 2: RL policy (sim-to-real).** Once the hardware platform is stable, train RL locomotion policies in Isaac Gym/Lab. RL policies dramatically outperform MPC on uneven terrain, stairs, and recovery from disturbances.

### 6.2 Open-Source Locomotion Controllers

| Controller | Approach | Platform | License | Runs on Jetson? | Repo |
|------------|----------|----------|---------|-----------------|------|
| **legged_control** (ETH Zurich) | MPC + WBC | ANYmal, adaptable to custom quads | BSD-3 | Yes (CPU) | [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control) |
| **Cheetah-Software** (MIT) | Convex MPC | MIT Mini Cheetah | MIT | Yes (CPU) | [mit-biomimetics/Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software) |
| **walk-these-ways** (CMU) | RL (teacher-student) | Unitree Go1/A1 | MIT | Yes | [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) |
| **legged_gym** (ETH Zurich) | RL training framework | Isaac Gym | BSD-3 | Training: PC GPU; Deploy: Jetson | [leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym) |
| **Unitree SDK2** | Low-level API | Unitree Go2/B2 | Apache-2.0 | Yes | [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) |
| **Quadruped PyBullet** | MPC + basic gaits | Generic quadruped | MIT | Yes (CPU) | [StanfordASL/quadruped](https://github.com/StanfordASL) |

**Recommended starting point:** `legged_control` from ETH Zurich. It provides:
- ROS2 integration (via ros2_control)
- MPC-based locomotion with whole-body control
- Configurable for custom quadruped kinematics
- Proven on real hardware (ANYmal robot)
- BSD-3 license (commercially compatible)

### 6.3 Terrain Adaptation

For outdoor operation on grass, gravel, slopes, and curbs:

**MPC approach (Phase 1):**
- Use terrain estimation from depth camera to adjust foot placement
- Adjust MPC ground reaction force targets based on estimated surface normal
- Conservative gait with large stability margins (sacrifice speed for safety)

**RL approach (Phase 2):**
- Train in Isaac Gym with randomized terrains (slopes, stairs, rough ground, deformable surfaces)
- Domain randomization on friction, mass, motor strength for sim-to-real transfer
- The `walk-these-ways` framework demonstrates robust terrain adaptation via RL on quadrupeds

### 6.4 Sim-to-Real Training Pipeline

```
┌────────────────────────────────────────────────────────────┐
│                    TRAINING (PC with GPU)                    │
│                                                              │
│  ┌─────────────┐     ┌──────────────┐     ┌─────────────┐  │
│  │ Isaac Lab /  │────▸│ RL Training  │────▸│ Policy      │  │
│  │ Isaac Gym    │     │ (PPO/SAC)    │     │ (PyTorch    │  │
│  │              │     │ 4096 parallel│     │  .pt file)  │  │
│  │ Robot URDF   │     │ environments │     │             │  │
│  │ + terrain    │     │              │     │             │  │
│  └─────────────┘     └──────────────┘     └──────┬──────┘  │
│                                                    │         │
└────────────────────────────────────────────────────┼─────────┘
                                                     │
                                            Export to TensorRT
                                                     │
                                                     ▼
┌────────────────────────────────────────────────────────────┐
│                  DEPLOYMENT (Jetson Orin Nano)               │
│                                                              │
│  ┌─────────────┐     ┌──────────────┐     ┌─────────────┐  │
│  │ Observation  │────▸│ RL Policy    │────▸│ Joint       │  │
│  │ (IMU, joint  │     │ (TensorRT    │     │ Torque      │  │
│  │  angles,     │     │  engine)     │     │ Commands    │  │
│  │  body vel)   │     │ <1ms infer.  │     │ @ 50 Hz     │  │
│  └─────────────┘     └──────────────┘     └──────┬──────┘  │
│                                                    │         │
│                                              CAN bus @ 1 kHz │
│                                                    │         │
│                                              ┌─────┴──────┐  │
│                                              │ 12x Motors │  │
│                                              └────────────┘  │
└────────────────────────────────────────────────────────────┘
```

**Simulation frameworks:**

| Framework | Features | GPU Required | License | Repo |
|-----------|----------|-------------|---------|------|
| **Isaac Lab** (NVIDIA) | GPU-accelerated physics, massive parallelism (4096+ envs), native Jetson pipeline | Yes (RTX 3070+) | [License](https://github.com/isaac-sim/IsaacLab/blob/main/LICENSE) | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) |
| Isaac Gym (legacy) | Predecessor to Isaac Lab, still widely used in papers | Yes | NVIDIA EULA | [NVIDIA-Omniverse/IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs) |
| PyBullet | CPU physics, free, simpler | No (CPU) | zlib | [bulletphysics/bullet3](https://github.com/bulletphysics/bullet3) |
| MuJoCo | High-fidelity physics, DeepMind-backed | No (CPU, GPU optional) | Apache-2.0 | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) |
| Gazebo (Harmonic) | ROS2 native, multi-physics, rich sensor simulation | No (CPU, GPU for rendering) | Apache-2.0 | [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) |

**Recommended:**
- **Locomotion RL training:** Isaac Lab (best parallelism for PPO training, NVIDIA ecosystem aligns with Jetson deployment)
- **Full-stack integration testing:** Gazebo Harmonic (native ROS2, can simulate cameras, LiDAR, and full autonomy stack)
- **Quick prototyping:** MuJoCo or PyBullet (simpler setup, faster iteration)

---

## 7. Decision Engine / Behavior System

### 7.1 Architecture Comparison

| Approach | Description | Pros | Cons | Verdict |
|----------|-------------|------|------|---------|
| **Behavior Trees (BT)** | Hierarchical tree of conditions and actions, tick-based execution | Modular, composable, easy to debug, industry standard for robotics | Can become complex for dynamic re-planning | **Recommended.** ROS2 standard via Nav2. |
| Finite State Machines (FSM) | States + transitions | Simple to understand | State explosion with complex behaviors, hard to modify | Too rigid for our multi-modal behavior. |
| Hierarchical FSM (HFSM) | Nested state machines | Better than flat FSM | Still limited composability | Workable but BT is better. |
| LLM-based planning | Use language model to decide actions | Flexible, handles novel situations | Unpredictable latency, unreliable, safety concerns | **Not suitable for real-time control.** Useful as high-level mission planner only (Phase 3+). |

**Decision: Behavior Trees using BehaviorTree.CPP v4.**

### 7.2 BehaviorTree.CPP

- **Library:** [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP) — C++ behavior tree library, the standard in ROS2/Nav2
- **Visualization:** [Groot2](https://www.behaviortree.dev/groot/) — Visual editor and real-time debugger for behavior trees
- **ROS2 integration:** Native. Nav2 uses BT.CPP as its behavior engine.

### 7.3 CleanWalker Behavior Tree

```
ROOT [Sequence]
├── CheckSafetyConditions [Condition]
│   ├── BatteryAboveMinimum (>10%)
│   ├── NoEStopActive
│   ├── WithinGeofence
│   └── SystemHealthOK
│
├── ReactiveSequence
│   ├── MonitorBattery [Condition]
│   │   └── [if <15%] → ReturnToDock [Action]
│   │
│   ├── MonitorBinFull [Condition]
│   │   └── [if full] → ReturnToDock [Action]
│   │
│   ├── MonitorPedestrianProximity [Condition]
│   │   └── [if <2m] → StopAndYield [Action]
│   │
│   └── MainMission [Fallback]
│       ├── HandleDetectedLitter [Sequence]
│       │   ├── HasLitterDetection [Condition]
│       │   ├── ApproachLitter [Action]
│       │   │   ├── NavigateToPreGraspPose (Nav2)
│       │   │   └── VisualServoToTarget
│       │   ├── ExecuteGrasp [Action]
│       │   │   ├── SelectGraspPrimitive (class-based)
│       │   │   ├── MoveArmToPreGrasp
│       │   │   ├── MoveArmToGrasp
│       │   │   ├── CloseGripper
│       │   │   └── VerifyGrasp (wrist camera check)
│       │   ├── DepositInBag [Action]
│       │   │   ├── MoveArmToBagFrame
│       │   │   ├── OpenGripper
│       │   │   └── RetractArm
│       │   └── ResumePatrol [Action]
│       │
│       └── CoveragePatrol [Sequence]
│           ├── HasCoverageWaypoints [Condition]
│           ├── GetNextWaypoint [Action]
│           └── NavigateToWaypoint [Action] (Nav2)
```

### 7.4 Edge Case Handling

| Scenario | Behavior Tree Response |
|----------|----------------------|
| **Bag full** | Interrupt patrol → Navigate to dock → Eject bag cassette → Wait for replacement or return with empty → Resume |
| **Low battery (<15%)** | Interrupt any action → Navigate to dock → Begin charging → Resume when >80% |
| **Critical battery (<10%)** | Emergency: stop all actions → Navigate to nearest dock by shortest path → Power-save mode |
| **Obstacle timeout (stuck >30s)** | Try recovery behaviors (backup, spin) → If still stuck, notify fleet operator → Enter teleop-assist mode |
| **Pedestrian encounter** | Distance-based speed modulation → Full stop if <2m → Yield until clear → Resume path |
| **Child / pet detected** | Wider clearance (2.0m) → Reduce speed to 0.2 m/s → Stop and fold arm if <3m |
| **Rain detected** | Continue operation (IP65) → Reduce speed to 0.3 m/s → Increase stopping distance → Skip slippery terrain |
| **Grasp failure (3 attempts)** | Log item location + photo → Skip item → Add to "difficult items" queue → Flag for human review → Continue patrol |
| **Unrecognized object** | Take photo → Log with GPS → Continue patrol → Upload for annotation (model improvement loop) |
| **Geofence breach** | Immediate stop → Recalculate path within geofence → If no valid path, notify operator |
| **Communication loss (>5 min)** | Continue autonomous operation with last-known mission → Navigate to dock if no mission → Attempt reconnect every 30s |

---

## 8. Integration Architecture

### 8.1 ROS 2 as Middleware

**Distribution: ROS 2 Humble Hawksbill (LTS)**
- Supported through May 2027
- Native Jetson support via JetPack 6.x
- Best Nav2 compatibility
- Consider migration to **Jazzy Jalisco** (May 2024 release, supported through May 2029) when Nav2 support matures

**Why ROS 2:**
- Standard middleware for robotics; all chosen components (Nav2, MoveIt2, RTAB-Map, OAK-D driver) provide ROS2 packages
- DDS-based communication (Fast DDS or Cyclone DDS) handles inter-process messaging with QoS
- `ros2_control` provides hardware abstraction for motors and sensors
- `lifecycle` nodes for clean startup/shutdown
- Large ecosystem of tools: rviz2, rqt, rosbag2, tf2

### 8.2 Hardware Abstraction Layer

```
┌──────────────────────────────────────────────────────────────┐
│                     ROS 2 APPLICATION LAYER                   │
│  (Nav2, BT, YOLO node, SLAM, coverage planner, arm planner) │
└──────────────────────────────┬───────────────────────────────┘
                               │ ROS2 topics/services/actions
                               │
┌──────────────────────────────┴───────────────────────────────┐
│                    ros2_control FRAMEWORK                      │
│                                                               │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐  │
│  │ Leg Controller │  │ Arm Controller │  │ Gripper Ctrl.  │  │
│  │ (12 joints)    │  │ (2 joints)     │  │ (1 joint)      │  │
│  │ JointEffort    │  │ JointPosition  │  │ JointPosition  │  │
│  └───────┬────────┘  └───────┬────────┘  └───────┬────────┘  │
│          │                   │                    │           │
│  ┌───────┴────────┐  ┌──────┴─────────┐  ┌──────┴────────┐  │
│  │ CAN Bus HW     │  │ Dynamixel HW   │  │ Dynamixel HW  │  │
│  │ Interface       │  │ Interface      │  │ Interface     │  │
│  │ (CubeMars AK)  │  │ (XM430)        │  │ (XL430)       │  │
│  └───────┬────────┘  └──────┬─────────┘  └──────┬────────┘  │
└──────────┼──────────────────┼────────────────────┼───────────┘
           │                  │                    │
     CAN bus @ 1 kHz    Dynamixel TTL/RS485   Dynamixel TTL
           │                  │                    │
    ┌──────┴──────┐   ┌──────┴──────┐     ┌──────┴──────┐
    │ 12x CubeMars│   │ 2x XM430   │     │ 1x XL430   │
    │ AK60/AK70   │   │ Arm Servos  │     │ Gripper     │
    └─────────────┘   └─────────────┘     └─────────────┘
```

### 8.3 ROS 2 Communication Graph

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|-----------|---------------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | OAK-D driver | YOLO node, RTAB-Map | 30 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | OAK-D driver | RTAB-Map, obstacle costmap | 30 Hz |
| `/camera/imu` | `sensor_msgs/Imu` | OAK-D driver | robot_localization EKF | 200 Hz |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar driver | RTAB-Map, Nav2 costmap | 10 Hz |
| `/detections` | `vision_msgs/Detection3DArray` | YOLO node | Behavior tree | 15-30 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | Nav2, coverage planner | 1 Hz |
| `/odom` | `nav_msgs/Odometry` | RTAB-Map | Nav2, behavior tree | 10 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization | Nav2, RTAB-Map | 50 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller | Locomotion controller | 20 Hz |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Behavior tree / coverage planner | Nav2 | Event |
| `/arm/goal_pose` | `geometry_msgs/PoseStamped` | Behavior tree | Arm IK solver | Event |
| `/arm/joint_commands` | `sensor_msgs/JointState` | Arm IK solver | ros2_control arm HW | 50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | ros2_control | RTAB-Map (tf), MoveIt2 | 50 Hz |
| `/battery_state` | `sensor_msgs/BatteryState` | Power monitor node | Behavior tree | 1 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | All nodes | Fleet monitor | 1 Hz |
| `/tf` | `tf2_msgs/TFMessage` | RTAB-Map, robot_state_publisher | All spatial nodes | 50 Hz |

### 8.4 Compute Budget on Jetson Orin Nano Super (8 GB)

| Module | GPU % | CPU Cores | RAM (MB) | Notes |
|--------|-------|-----------|----------|-------|
| **YOLO11n (TensorRT INT8)** | 25-30% | 0.5 | 200 | Pre/post processing on CPU |
| **PIDNet terrain seg (optional)** | 10-15% | 0.5 | 150 | Run at 5 Hz, 512x256 |
| **RTAB-Map SLAM** | 5-10% | 2.0 | 500 | Primarily CPU, some GPU for feature extraction |
| **robot_localization EKF** | 0% | 0.5 | 50 | Pure CPU |
| **Nav2 (planner + controller)** | 0% | 1.0 | 200 | Pure CPU |
| **Behavior tree** | 0% | 0.2 | 50 | Lightweight |
| **OAK-D driver** | 0% | 0.5 | 150 | Camera processing on VPU |
| **RPLidar driver** | 0% | 0.2 | 30 | Lightweight |
| **ros2_control (motors)** | 0% | 0.5 | 50 | CAN bus I/O |
| **Arm IK + grasp** | 0-5% | 0.3 | 100 | Minimal, event-driven |
| **RL locomotion policy (Phase 2)** | 5-10% | 0.2 | 100 | TensorRT, <1ms per inference |
| **System overhead (DDS, tf2, logging)** | 0% | 0.5 | 300 | |
| **OS + drivers** | 0% | 0.5 | 800 | JetPack Linux |
| | | | | |
| **TOTAL** | **45-70%** | **~7.4** | **~2,680** | |
| **Available** | **100%** | **6 cores** | **8,192** | |

**Analysis:**
- **GPU:** 45-70% utilized. Comfortable headroom. Can add terrain segmentation or monocular depth if needed.
- **CPU:** 7.4 logical cores needed, 6 physical cores available. This is tight. Mitigations:
  - RTAB-Map can run at 5 Hz instead of 10 Hz (saves ~1 core)
  - Nav2 controller only active during navigation (not during stationary grasping)
  - Arm IK and behavior tree are lightweight and intermittent
  - Use process priorities (RT for motor control, normal for SLAM)
- **RAM:** 2.7 GB of 8 GB. Comfortable. Leaves room for map growth and model caching.

**Verdict: Jetson Orin Nano Super (8 GB) is viable for the full autonomy stack in Phase 1.** CPU is the bottleneck, not GPU. If CPU becomes limiting, consider upgrading to Orin NX 16GB (8 cores, 16 GB RAM) or offloading SLAM to reduced rate.

### 8.5 Safety Architecture

The safety system operates independently from the autonomy stack:

```
┌───────────────────────────────────────────────────────┐
│              SAFETY SUPERVISOR (independent)            │
│              Dedicated safety-rated MCU (STM32)         │
│                                                         │
│  Inputs:                    Outputs:                    │
│  ├── Physical E-stop button ├── Motor power relay (cut) │
│  ├── Bumper contact sensors ├── Status LED (red)        │
│  ├── Motor current monitors ├── Alert to Jetson via UART│
│  ├── Watchdog from Jetson   └── Alert to fleet (4G)     │
│  └── Tilt/rollover sensor                               │
│                                                         │
│  Rules (hardcoded, no ML):                              │
│  1. E-stop pressed → cut motor power immediately        │
│  2. Bumper contact → cut motor power within 10 ms       │
│  3. Motor overcurrent → reduce torque / cut if extreme  │
│  4. Jetson watchdog missed (>500ms) → controlled stop   │
│  5. Tilt > 45° → cut motor power (fall detection)       │
│  6. Total force on any surface > 50N → stop + retract   │
└───────────────────────────────────────────────────────┘
```

This safety supervisor is a dedicated microcontroller (STM32F4 or similar) running bare-metal firmware with NO dependency on the Jetson, ROS2, or any software stack. It has direct hardware control over the motor power relay. This satisfies ISO 13849 PL d requirements for an independent safety channel.

---

## 9. Recommended Stack

### 9.1 Complete Module Recommendations

| # | Module | Recommended Model/System | Backup Option | Compute Requirement | Open-Source Repo | Fine-tuning Required? |
|---|--------|--------------------------|---------------|---------------------|------------------|-----------------------|
| 1 | **Litter Detection** | YOLO11n (Ultralytics) | YOLOv8n (Ultralytics) | 25-30% GPU, INT8 TensorRT | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | **Yes.** Fine-tune on TACO + Litter + custom dataset. |
| 2 | **Depth Estimation** | OAK-D Pro hardware stereo | Depth Anything V2 Small (GPU fallback) | 0% GPU (runs on camera VPU) | [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros) | No. Hardware stereo works out-of-box. |
| 3 | **Terrain Segmentation** | PIDNet-S | DDRNet-23-slim | 10-15% GPU, 5 Hz | [XuJiacong/PIDNet](https://github.com/XuJiacong/PIDNet) | **Yes.** Fine-tune on RUGD + Cityscapes + custom. Phase 2. |
| 4 | **SLAM** | RTAB-Map | ORB-SLAM3 | 2 CPU cores, 5-10% GPU | [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | No. Works out-of-box with stereo + LiDAR input. |
| 5 | **State Estimation** | robot_localization (EKF) | — | 0.5 CPU core | [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | No. Configuration only. |
| 6 | **LiDAR SLAM (Phase 2)** | KISS-ICP | LIO-SAM | 1 CPU core | [PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) | No. Works out-of-box. |
| 7 | **Navigation** | Nav2 (DWB controller) | Nav2 (TEB controller, Phase 2) | 1 CPU core | [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | No. Configuration + parameter tuning. |
| 8 | **Coverage Planning** | Fields2Cover (BCD) | full_coverage_path_planner | Minimal CPU | [Fields2Cover/Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | No. Configuration only. |
| 9 | **Behavior System** | BehaviorTree.CPP v4 | — | 0.2 CPU core | [BehaviorTree/BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | Custom tree authoring required. |
| 10 | **Grasp Planning** | Rule-based primitives (Phase 1) | AnyGrasp (Phase 2) | Near zero (Phase 1) | Custom / [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk) | Phase 1: No. Phase 2: Some fine-tuning. |
| 11 | **Arm IK** | Custom analytical (Phase 1) | MoveIt2 (Phase 2+) | Minimal | Custom / [moveit/moveit2](https://github.com/moveit/moveit2) | No. Analytical solution for 2-DOF. |
| 12 | **Visual Servoing** | Custom PID on detection centroid | ViSP library | Minimal CPU | Custom / [lagadic/visp](https://github.com/lagadic/visp) | Custom implementation required. |
| 13 | **Locomotion (Phase 1)** | legged_control (MPC) | Cheetah-Software | 2 CPU cores | [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control) | Configuration for our kinematic model. |
| 14 | **Locomotion (Phase 2)** | RL policy (walk-these-ways style) | legged_gym | 5-10% GPU, <1ms | [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) | **Yes.** Train in Isaac Lab with our URDF. |
| 15 | **Simulation** | Isaac Lab (RL), Gazebo Harmonic (integration) | MuJoCo, PyBullet | PC GPU (training only) | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab), [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | Setup and configuration. |
| 16 | **Safety Supervisor** | STM32 bare-metal firmware | — | Independent MCU | Custom firmware | Custom firmware required. |

### 9.2 Software Versions (as of Q1 2026)

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 22.04 LTS (JetPack) | JetPack 6.x ships Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill (LTS) | Supported through May 2027 |
| CUDA | 12.x (JetPack 6.x) | |
| TensorRT | 10.x (JetPack 6.x) | For model optimization |
| Ultralytics | Latest (pip) | YOLO11 support |
| Nav2 | 1.1.x (Humble) | |
| RTAB-Map | 0.21.x | |
| BehaviorTree.CPP | 4.x | |
| Python | 3.10+ | JetPack default |
| Rust | Latest stable | Firmware motor controller |

---

## 10. Development Roadmap

### 10.1 What Can We Prototype NOW in Simulation

| Item | Simulation Tool | What We Learn | Effort |
|------|----------------|---------------|--------|
| **Litter detection model training** | N/A (Kaggle/Colab GPU) | Model accuracy, class performance, failure modes | 1-2 weeks |
| **Navigation + coverage planning** | Gazebo Harmonic + Nav2 | Coverage efficiency, path quality, obstacle handling | 1-2 weeks |
| **Behavior tree development** | Gazebo + mock detections | State machine logic, edge case handling | 1 week |
| **Locomotion RL training** | Isaac Lab + CleanWalker URDF (exists at `hardware/urdf/`) | Gait quality, terrain handling, sim-to-real gap | 2-4 weeks |
| **Full stack integration** | Gazebo + all ROS2 nodes | End-to-end pipeline validation, timing, resource usage | 2-3 weeks |
| **Visual servoing** | Gazebo + arm model | Approach accuracy, servo convergence | 1 week |
| **SLAM mapping** | Gazebo park world + RTAB-Map | Map quality, loop closure, drift | 1 week |

**Total simulation phase: 6-10 weeks** before needing real hardware.

### 10.2 What Needs Real Hardware

| Item | Why Simulation Is Insufficient | Hardware Needed |
|------|-------------------------------|-----------------|
| **Litter detection in real environments** | Lighting variation, real litter textures, occlusion | Camera + Jetson |
| **Grasp execution on real litter** | Soft-body physics of crumpled wrappers, wet paper, etc. are not well-simulated | Arm + gripper + litter samples |
| **Outdoor SLAM** | Real sensor noise, GPS multipath, dynamic outdoor scenes | Full sensor suite |
| **Motor control tuning** | Sim-to-real gap in motor dynamics, friction, backlash | Actual actuators |
| **Battery life validation** | Thermal effects, actuator efficiency under load | Full robot |
| **IP65 validation** | Water ingress, dust, temperature extremes | Full robot + environmental chamber |
| **Pedestrian interaction** | Human behavior is unpredictable, social dynamics | Full robot in controlled public setting |

### 10.3 Module Risk Assessment

| Module | Technical Risk | Impact if Failed | Mitigation |
|--------|---------------|-----------------|------------|
| **Litter detection (YOLO)** | LOW | High | Well-proven architecture, large training data available, our existing pipeline works |
| **SLAM (outdoor parks)** | MEDIUM | High | RTAB-Map is proven; challenge is repetitive visual features. Mitigate with LiDAR (Phase 2) and GPS fusion |
| **Navigation (Nav2)** | LOW | High | Mature framework, well-documented. Park environments are simpler than indoor/urban |
| **Locomotion (walking)** | **HIGH** | **Critical** | Hardest module. Quadruped walking on varied terrain with payload is an active research area. Mitigate with conservative MPC first, RL later |
| **Grasping diverse litter** | **HIGH** | **High** | Flat/flexible items (wrappers, bags, cigarette butts) are very hard to grasp. Mitigate with specialized gripper + "skip and flag" strategy |
| **Terrain segmentation** | MEDIUM | Medium | Not critical for Phase 1 (use geometric rules). Fine-tuning on park data needed for Phase 2 |
| **Visual servoing** | LOW-MEDIUM | Medium | Classical control problem. Close-range depth accuracy is the main challenge |
| **Behavior tree logic** | LOW | Medium | Deterministic, testable in simulation. Edge cases discoverable through field testing |
| **Compute budget (Orin Nano)** | MEDIUM | Medium | CPU is tight. If insufficient, upgrade to Orin NX ($600) — painful but solvable |
| **Safety system** | LOW (technical) | **Critical** (regulatory) | Independent MCU is straightforward engineering. The certification process is the risk, not the tech |

### 10.4 Suggested Development Order

```
PHASE 1: FOUNDATION (Months 1-3) — Simulation + Core Software
──────────────────────────────────────────────────────────────
  [1] Train YOLO11n litter detection model (TACO + Litter dataset)
  [2] Set up ROS2 Humble + Gazebo Harmonic development environment
  [3] Implement behavior tree (full state machine)
  [4] Integrate Nav2 + coverage planner in simulation
  [5] Set up RTAB-Map SLAM in simulation
  [6] Develop arm IK solver (analytical, 2-DOF)
  [7] Implement rule-based grasp primitives
  [8] Validate URDF model, tune physics parameters

PHASE 2: LOCOMOTION (Months 2-5) — Hardest Problem First
──────────────────────────────────────────────────────────
  [9] Implement MPC locomotion controller (legged_control)
  [10] Test in Isaac Lab / Gazebo with varied terrains
  [11] Begin RL locomotion training in Isaac Lab (parallel with MPC)
  [12] Build first physical leg assembly for motor testing
  [13] Validate CubeMars AK actuator control via CAN bus
  [14] Tune MPC gains on real leg hardware

PHASE 3: HARDWARE INTEGRATION (Months 4-7)
──────────────────────────────────────────
  [15] Assemble full prototype robot
  [16] Integrate Jetson Orin Nano + OAK-D Pro + RPLidar A1
  [17] Deploy ROS2 stack onto Jetson (TensorRT optimization for YOLO)
  [18] Real-world SLAM mapping of test area
  [19] Real-world litter detection validation + model fine-tuning
  [20] Grasping tests with real litter items → iterate gripper design

PHASE 4: FIELD TESTING (Months 6-9)
───────────────────────────────────
  [21] Controlled outdoor testing (private area, no public)
  [22] Coverage planning validation (does it cover the area?)
  [23] End-to-end test: patrol → detect → approach → grasp → deposit
  [24] Battery life validation
  [25] Pedestrian interaction testing (controlled, with safety observers)
  [26] Iterate on all modules based on field data

PHASE 5: HARDENING (Months 8-12)
────────────────────────────────
  [27] Safety supervisor MCU development and testing
  [28] IP65 enclosure integration and water testing
  [29] Add terrain segmentation (PIDNet) if needed
  [30] Deploy RL locomotion policy if MPC insufficient
  [31] Add 3D LiDAR (Livox Mid-360) + KISS-ICP
  [32] Fleet communication + telemetry system
  [33] OTA update mechanism
  [34] Begin certification engagement (TUV / UL)
```

### 10.5 Key Decision Points

| Decision | When | Options | Criteria |
|----------|------|---------|----------|
| YOLO11n vs YOLO11s | After Phase 1 training | n (faster) vs s (more accurate) | If mAP50 on our litter classes > 0.90 with n, use n. Otherwise, upgrade to s. |
| MPC vs RL locomotion | End of Phase 2 | Stick with MPC vs. deploy RL policy | If MPC handles grass + gravel + 15° slopes reliably, keep it. If terrain failures > 5%, deploy RL. |
| Orin Nano vs Orin NX | End of Phase 3 | Keep Orin Nano ($249) vs. upgrade NX ($600) | If any module drops below real-time requirements under full load, upgrade. |
| 2-DOF arm vs 4-6 DOF | End of Phase 4 | Keep simple arm vs. add wrist DOF | If grasp success rate on flat items (paper, wrappers) < 70%, need more DOF + MoveIt2. |
| Add 3D LiDAR | Phase 5 | Keep RPLidar A1 only vs. add Livox Mid-360 | If SLAM drift > 1m over 30-min runs, or obstacle avoidance fails in low-light, add 3D LiDAR. |

---

## Appendix A: Key Repositories Quick Reference

| Category | Repo | URL | License |
|----------|------|-----|---------|
| YOLO11 | ultralytics | https://github.com/ultralytics/ultralytics | AGPL-3.0 |
| Depth Anything V2 | Depth-Anything-V2 | https://github.com/DepthAnything/Depth-Anything-V2 | Apache-2.0 |
| OAK-D ROS2 | depthai-ros | https://github.com/luxonis/depthai-ros | MIT |
| Terrain Segmentation | PIDNet | https://github.com/XuJiacong/PIDNet | MIT |
| SLAM | rtabmap_ros | https://github.com/introlab/rtabmap_ros | BSD-3 |
| ORB-SLAM3 | ORB_SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | GPLv3 |
| KISS-ICP | kiss-icp | https://github.com/PRBonn/kiss-icp | MIT |
| LIO-SAM | LIO-SAM | https://github.com/TixiaoShan/LIO-SAM | BSD-3 |
| Nav2 | navigation2 | https://github.com/ros-navigation/navigation2 | Apache-2.0 |
| Coverage Planning | Fields2Cover | https://github.com/Fields2Cover/Fields2Cover | BSD-3 |
| Behavior Trees | BehaviorTree.CPP | https://github.com/BehaviorTree/BehaviorTree.CPP | MIT |
| Grasp Planning | anygrasp_sdk | https://github.com/graspnet/anygrasp_sdk | Apache-2.0 |
| MoveIt2 | moveit2 | https://github.com/moveit/moveit2 | BSD-3 |
| Visual Servoing | visp | https://github.com/lagadic/visp | GPLv2 |
| Locomotion (MPC) | legged_control | https://github.com/qiayuanl/legged_control | BSD-3 |
| Locomotion (RL) | walk-these-ways | https://github.com/Improbable-AI/walk-these-ways | MIT |
| RL Training | legged_gym | https://github.com/leggedrobotics/legged_gym | BSD-3 |
| Isaac Lab | IsaacLab | https://github.com/isaac-sim/IsaacLab | BSD-3 |
| Gazebo | gz-sim | https://github.com/gazebosim/gz-sim | Apache-2.0 |
| MuJoCo | mujoco | https://github.com/google-deepmind/mujoco | Apache-2.0 |
| Robot Localization | robot_localization | https://github.com/cra-ros-pkg/robot_localization | BSD-3 |
| Cheetah Software | Cheetah-Software | https://github.com/mit-biomimetics/Cheetah-Software | MIT |
| Social Navigation | social_nav2 | https://github.com/robotics-upo/social_nav2 | Apache-2.0 |

## Appendix B: License Compatibility Matrix

Our project is AGPL-3.0. Compatibility with dependencies:

| Dependency License | Compatible with AGPL-3.0? | Notes |
|-------------------|---------------------------|-------|
| MIT | Yes | Permissive, no issues |
| BSD-3 | Yes | Permissive, no issues |
| Apache-2.0 | Yes | Compatible with GPLv3+ family |
| AGPL-3.0 | Yes | Same license |
| GPLv3 | Yes | Compatible (AGPL is stricter superset) |
| GPLv2 | **Caution** | GPLv2-only is NOT compatible. GPLv2-or-later is fine. Check ViSP license. |
| NVIDIA EULA | **Separate** | Isaac Gym/TensorRT are proprietary but free to use. Cannot redistribute. Runtime dependency only. |

---

*This document is a living technical reference for the CleanWalker autonomy stack. It should be updated as architectural decisions are made, models are benchmarked, and the system evolves through prototype to production.*
