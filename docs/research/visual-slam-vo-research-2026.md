# Visual SLAM & Visual Odometry Systems for Jetson Orin Nano Super (Early 2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB RAM, 6-core Arm Cortex-A78AE)
**Use Case:** Outdoor litter-picking robot — parks, sidewalks, beaches
**Challenge:** GPS unreliable under trees/buildings; need robust visual localization
**Date:** February 10, 2026

---

## Executive Summary

This document evaluates seven Visual SLAM and Visual Odometry systems for the CleanWalker robot's localization and mapping needs. The robot operates in unstructured outdoor environments where GPS is often degraded or unavailable, requiring robust visual-based positioning.

**Key Findings:**

1. **NVIDIA Isaac ROS Visual SLAM (cuVSLAM)** is the highest-performance option for Jetson, with GPU-accelerated processing achieving 30+ FPS at minimal CPU cost. Free under Apache-2.0. Best raw accuracy on EuRoC/KITTI benchmarks.
2. **RTAB-Map** remains the most versatile and battle-tested option with native ROS2 support, multi-sensor fusion (stereo + LiDAR + IMU), occupancy grid generation, and map persistence. BSD license.
3. **ORB-SLAM3** offers the best accuracy among open-source systems with stereo-inertial fusion, but its GPLv3 license and lack of native ROS2 support are drawbacks.
4. **Deep learning methods (DROID-SLAM, DPVO)** achieve state-of-the-art accuracy but require too much GPU memory (8-11+ GB) for embedded deployment on an 8GB device running concurrent workloads.
5. **For CleanWalker Phase 1:** cuVSLAM for visual odometry + RTAB-Map for mapping/loop closure is the recommended combination, providing GPU-accelerated tracking with proven outdoor mapping capability.

---

## Table of Contents

1. [System-by-System Analysis](#1-system-by-system-analysis)
2. [Comparison Tables](#2-comparison-tables)
3. [NVIDIA Isaac ROS Visual SLAM Deep Dive](#3-nvidia-isaac-ros-visual-slam-deep-dive)
4. [Camera + IMU Fusion Hardware](#4-camera--imu-fusion-hardware)
5. [Outdoor SLAM Challenges & Mitigations](#5-outdoor-slam-challenges--mitigations)
6. [Occupancy Grid Generation for Path Planning](#6-occupancy-grid-generation-for-path-planning)
7. [Recommendations for CleanWalker](#7-recommendations-for-cleanwalker)

---

## 1. System-by-System Analysis

### 1.1 ORB-SLAM3

**Repository:** [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
**Paper:** Campos et al., IEEE T-RO 2021
**Last Update:** December 2021 (v1.0 — no updates since)
**License:** GPLv3 (commercial use requires contacting authors)

**Type:** Feature-based (indirect) SLAM

**Description:** ORB-SLAM3 is the gold standard of open-source visual SLAM. It extracts ORB features from images, matches them across frames, and jointly optimizes camera poses and 3D landmark positions using bundle adjustment. It introduced multi-map support and tight visual-inertial integration over its predecessors.

**Supported Camera Types:**
- Monocular (pinhole + fisheye)
- Stereo (pinhole + fisheye)
- RGB-D
- Monocular + IMU
- Stereo + IMU

**Accuracy on Standard Benchmarks:**

| Dataset | Mode | ATE RMSE | Notes |
|---------|------|----------|-------|
| EuRoC | Stereo-Inertial | **3.6 cm avg** | Best-in-class among open-source; sub-1cm on easy sequences |
| EuRoC | Stereo | ~5-8 cm avg | Without IMU, drift increases on aggressive motions |
| EuRoC | Mono-Inertial | ~5 cm avg | Scale recovery via IMU |
| KITTI | Stereo | 0.31% ATE | Top tier for stereo odometry |
| TUM-VI | Stereo-Inertial | **9 mm avg** | Quick hand-held motions in small rooms |

**CPU/GPU Requirements:**
- Pure CPU operation (no GPU needed)
- 3-4 CPU cores at 30 FPS; can run at 10 FPS with ~2 cores
- ~400-800 MB RAM depending on map size
- Memory usage approximately 2x that of ORB-SLAM2

**Jetson Orin Nano Compatibility:**
- Yes, proven to run. Studies on Jetson Nano (much weaker) achieved 3-11 cm errors on EuRoC.
- On Orin Nano (significantly faster CPU), stereo mode at 15-20 FPS is comfortable. Stereo-inertial at 10-15 FPS is feasible.
- CPU is the bottleneck, not GPU.

**ROS2 Support:**
- No native ROS2 support from upstream (last release targets ROS Melodic/Ubuntu 18.04)
- Multiple community ROS2 Humble wrappers exist:
  - [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) — native ROS2 Humble package
  - [suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker) — Docker-based, supports TF publishing, RViz, namespacing
  - [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2) — supports mono/stereo/RGB-D/stereo-inertial
- Community wrappers are not officially maintained and may lag behind fixes.

**Loop Closure:** Yes. Uses DBoW2 (Bags of Binary Words) for place recognition. Robust loop closure with pose graph optimization.

**Map Persistence:** Limited. Can save/load maps through Atlas multi-map system, but the interface is not as polished as RTAB-Map. Community wrappers add map save/load services.

**Strengths:**
- Best accuracy among open-source visual SLAM systems
- Excellent visual-inertial fusion
- Multi-map support (handles tracking loss gracefully)
- Mature codebase with thousands of citations

**Weaknesses:**
- GPLv3 license is problematic for commercial use
- No official ROS2 support; community wrappers vary in quality
- Project appears abandoned (no commits since Dec 2021)
- Does not natively generate occupancy grids
- High CPU usage limits concurrent workloads on Orin Nano

---

### 1.2 RTAB-Map

**Repository:** [introlab/rtabmap](https://github.com/introlab/rtabmap) / [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros)
**Paper:** Labbe & Bherer, Journal of Field Robotics 2019; updated arXiv 2024
**Last Update:** Actively maintained (2025-2026 releases)
**License:** BSD-3-Clause (fully permissive; caveat: if OpenCV built with nonfree SURF, research-only)

**Type:** Appearance-based (hybrid visual + LiDAR) SLAM

**Description:** RTAB-Map is a comprehensive graph-based SLAM solution that uses appearance-based loop closure detection with a bag-of-words approach. Its key differentiator is multi-sensor versatility: it can fuse stereo cameras, RGB-D, 2D LiDAR, 3D LiDAR, and IMU data into a single consistent map. It is arguably the most production-ready open-source SLAM system for ROS2.

**Supported Camera Types:**
- RGB-D (primary use case)
- Stereo cameras
- Monocular (limited; primarily for odometry)
- 2D LiDAR scan fusion
- 3D LiDAR point cloud fusion
- IMU integration via EKF (robot_localization)

**Accuracy on Standard Benchmarks:**

| Dataset | Mode | ATE RMSE | Notes |
|---------|------|----------|-------|
| KITTI | Stereo F2M | 0.2-4.7 m | Varies by sequence; competitive with ORB-SLAM3 |
| KITTI | LiDAR (ICP S2S) | 0.3-24.0 m | LiDAR mode less accurate on some sequences |
| EuRoC | Stereo | ~5-15 cm | Good but generally behind ORB-SLAM3 |
| TUM RGB-D | RGB-D | ~2-5 cm | Strong in RGB-D mode |

Note: RTAB-Map's strength is not raw accuracy on benchmarks but rather its robustness across diverse real-world conditions and its multi-sensor fusion capability.

**CPU/GPU Requirements:**
- Primarily CPU-based with optional GPU acceleration for feature extraction
- 2-3 CPU cores at 10 Hz
- ~500-1000 MB RAM (grows with map size; memory management prevents unbounded growth)
- Desktop benchmarks: 60-330 ms/frame depending on configuration
- Minimal GPU usage (5-10%) for SURF/ORB feature extraction if GPU-enabled OpenCV is available

**Jetson Orin Nano Compatibility:**
- Yes. Well-tested on Jetson platforms (Nano, TX2, Xavier, Orin).
- Runs comfortably at 5-10 Hz with stereo + 2D LiDAR input.
- Docker containers available for Jetson deployment.
- OAK-D + RTAB-Map has been demonstrated specifically (see [M2219/RGBD_OakCamera_RTABMap](https://github.com/M2219/RGBD_OakCamera_RTABMap)).

**ROS2 Support:**
- **Native ROS2 support.** Fully ported to ROS2 Humble (version 0.22.x).
- Published ROS2 packages: `rtabmap`, `rtabmap_ros`, `rtabmap_launch`, `rtabmap_viz`
- Tight integration with Nav2, robot_localization, and tf2.
- Stereo outdoor mapping and navigation tutorials available.

**Loop Closure:** Yes. Core feature. Uses appearance-based loop closure with bag-of-words (visual vocabulary). Memory management ensures real-time performance even on large maps by limiting the working memory and transferring old nodes to long-term memory.

**Map Persistence:** Yes. Full map save/load capability. Maps can be saved to a SQLite database and reloaded for relocalization. Supports incremental map updates across sessions.

**Occupancy Grid Generation:** Yes. Natively generates 2D occupancy grids from stereo depth or LiDAR. Publishes `/rtabmap/proj_map` (projected from 3D) and can also generate 3D OctoMaps. Configurable via `Grid/FromDepth`, `Grid/3D` parameters.

**Strengths:**
- Most versatile multi-sensor SLAM (stereo + RGB-D + LiDAR + IMU)
- Native ROS2 Humble support with extensive documentation
- Built-in 2D/3D occupancy grid generation (directly usable by Nav2)
- Robust map persistence (save, load, incremental update)
- Memory management prevents unbounded map growth
- BSD license (commercially friendly)
- Actively maintained with responsive developer (Mathieu Labbe)

**Weaknesses:**
- Lower raw accuracy than ORB-SLAM3 on benchmarks
- Can be CPU-heavy when running full SLAM with loop closure at high rates
- Configuration complexity (many parameters)
- Loop closure can be slow on very large maps

---

### 1.3 Stella-VSLAM (successor to OpenVSLAM)

**Repository:** [stella-cv/stella_vslam](https://github.com/stella-cv/stella_vslam)
**ROS package:** [stella-cv/stella_vslam_ros](https://github.com/stella-cv/stella_vslam_ros)
**Last Update:** September 2025 (active maintenance)
**License:** BSD-2-Clause (very permissive)

**Type:** Feature-based (indirect) SLAM — derived from ORB-SLAM2 architecture

**Description:** Stella-VSLAM is a community fork of OpenVSLAM, created in January 2021 after the original project was taken down. It is essentially an ORB-SLAM2 derivative with a more permissive BSD license and broader camera model support (including equirectangular/fisheye). It lacks the visual-inertial fusion that ORB-SLAM3 provides.

**Supported Camera Types:**
- Monocular (pinhole)
- Stereo (pinhole)
- RGB-D
- Fisheye
- Equirectangular (360-degree cameras like RICOH THETA, Insta360)

**IMU Support:** Not available. Listed as "currently working on" in documentation — not yet implemented as of early 2026.

**Accuracy on Standard Benchmarks:**
- Similar to ORB-SLAM2 (which it is derived from)
- Generally 10-30% worse than ORB-SLAM3 due to lack of multi-map and IMU fusion
- No published benchmark numbers in their documentation

**CPU/GPU Requirements:**
- CPU-only operation
- ~2-3 cores for real-time stereo SLAM
- ~300-500 MB RAM
- Comparable to ORB-SLAM2 performance

**Jetson Orin Nano Compatibility:** Yes. Lightweight enough to run on Jetson Nano; should run well on Orin Nano.

**ROS2 Support:** Partial. The stella_vslam_ros package has a ROS2 branch, but historically only monocular was supported in ROS2 while ROS1 had all camera types. Status of full ROS2 stereo/RGB-D support is unclear.

**Loop Closure:** Yes. Uses DBoW2 (inherited from ORB-SLAM2).

**Map Persistence:** Yes. Maps can be saved and loaded. Users can "localize new images based on prebuilt maps."

**Strengths:**
- BSD-2-Clause license (most permissive of all systems reviewed)
- Broad camera model support (especially equirectangular for 360 cameras)
- Map save/load capability
- Lighter than ORB-SLAM3

**Weaknesses:**
- No IMU fusion (critical limitation for outdoor robotics)
- No multi-map support
- Lower accuracy than ORB-SLAM3
- Uncertain ROS2 support status for stereo/RGB-D modes
- Smaller community than ORB-SLAM3 or RTAB-Map
- Missing features that ORB-SLAM3 added (visual-inertial, multi-map, Atlas)

**Verdict:** Not recommended for CleanWalker. The lack of IMU fusion is a significant limitation for outdoor operation. The BSD license is attractive, but cuVSLAM (also permissively licensed) offers far better performance.

---

### 1.4 NVIDIA Isaac ROS Visual SLAM (cuVSLAM)

**Repository:** [NVIDIA-ISAAC-ROS/isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
**Python API:** [NVlabs/PyCuVSLAM](https://github.com/NVlabs/PyCuVSLAM)
**Paper:** cuVSLAM: CUDA Accelerated Visual Odometry and Mapping (arXiv, June 2025)
**Last Update:** February 2, 2026 (actively maintained by NVIDIA)
**License:** Apache-2.0 (open source, commercially friendly)

**Type:** Hybrid feature-based SLAM with full CUDA GPU acceleration

**Description:** cuVSLAM is NVIDIA's proprietary but freely-available GPU-accelerated visual SLAM system. It runs the entire SLAM pipeline on the GPU, from feature extraction through tracking, mapping, and loop closure. It supports configurations from a single RGB camera to complex arrays of up to 32 cameras with arbitrary geometric arrangements. This is specifically designed and optimized for NVIDIA Jetson platforms.

**Supported Camera Types:**
- Monocular
- Stereo (primary mode)
- Multi-stereo (up to 32 cameras in arbitrary configurations)
- Mono-depth (RGB-D)
- Visual-inertial (camera + IMU) — added in recent versions
- RGBD support added February 2, 2026

**Accuracy on Standard Benchmarks:**

| Dataset | Mode | Metric | cuVSLAM | ORB-SLAM3 | Notes |
|---------|------|--------|---------|-----------|-------|
| EuRoC | Stereo SLAM | ATE | **0.17%** | 0.21% | cuVSLAM 19% better |
| EuRoC | Stereo | APE RMSE | **5.4 cm** | — | Position error |
| KITTI | Stereo SLAM | ATE | **0.27%** | 0.31% | cuVSLAM 13% better |
| KITTI | Stereo | Transl. Error | **0.94%** | 1.15% (ORB2) | vs ORB-SLAM2 baseline |
| KITTI | Stereo | Rot. Error | 0.0019 deg/m | — | Excellent rotational accuracy |

**CPU/GPU Requirements:**
- GPU-accelerated (CUDA) — offloads compute from CPU
- At 640x360 @ 30 FPS on Orin Nano: ~8.3% CPU, ~9% GPU
- On AGX Orin: stereo mode at 1.8 ms/frame, multi-stereo at 2.3 ms/frame
- Can achieve 232 FPS at 720p on AGX Orin (far exceeds real-time)
- Very low CPU overhead — major advantage for multi-task systems

**Jetson Orin Nano Compatibility:**
- **Specifically designed for Jetson.** First-class support.
- Tested on Orin Nano in 25W mode at 640x360 @ 30 FPS
- Works within 8GB memory constraints
- Multi-stereo odometry demonstrated on Orin Nano
- Minimum resolution: 424x240 for stable 30 FPS in mono-depth mode

**ROS2 Support:**
- **Native ROS2 package.** Supports ROS2 Humble and Jazzy.
- Published as `isaac_ros_visual_slam` with full ROS2 integration
- Publishes standard `/tf`, `/odom` topics
- Integrates with Nav2 for navigation
- NVIDIA provides tutorials for Nav2 + cuVSLAM integration

**Loop Closure:** Yes. Backend performs asynchronous pose graph optimization and loop closing. Multi-camera configurations show ~40% improvement in SLAM accuracy metrics compared to pure odometry mode through enhanced loop detection.

**Map Persistence:** The system maintains a local odometry map of the last N poses plus visible landmarks, with global consistency from asynchronous backend refinement. Map save/load functionality is referenced in tutorials but specific details vary by release version.

**Strengths:**
- Best-in-class accuracy (beats ORB-SLAM3 on EuRoC and KITTI)
- Extremely low CPU usage (GPU-accelerated), leaving CPU free for Nav2/behavior tree
- Native Jetson optimization and first-class support
- Native ROS2 Humble/Jazzy support
- Apache-2.0 license (free, commercially friendly)
- Multi-camera support (future expansion to multiple stereo pairs)
- Active development by NVIDIA (regular releases)
- Visual-inertial mode available

**Weaknesses:**
- Closed-source core library (cuVSLAM binary, not source code)
- NVIDIA hardware lock-in (only runs on NVIDIA GPUs)
- Does not natively generate occupancy grids (needs separate mapping layer)
- Less community support/documentation than RTAB-Map
- Newer system with less field-proven track record than RTAB-Map or ORB-SLAM3
- Limited to stereo/depth cameras (no 2D LiDAR scan fusion like RTAB-Map)

---

### 1.5 VINS-Fusion

**Repository:** [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
**Paper:** Qin et al., IROS 2019
**Last Update:** ~2019-2020 (no recent upstream updates)
**License:** GPLv3

**Type:** Optimization-based visual-inertial odometry/SLAM

**Description:** VINS-Fusion is an optimization-based multi-sensor state estimator from HKUST. It was the top open-sourced stereo algorithm on the KITTI Odometry Benchmark (as of January 2019). It tightly couples visual features with IMU pre-integration for robust state estimation, especially useful for drones and fast-moving platforms.

**Supported Camera Types:**
- Monocular + IMU
- Stereo + IMU
- Stereo only (without IMU)
- GPS fusion capability demonstrated

**Accuracy on Standard Benchmarks:**

| Dataset | Mode | Performance | Notes |
|---------|------|-------------|-------|
| KITTI | Stereo (no loop closure) | Top open-source stereo on KITTI leaderboard (2019) | Evaluated without loop closure |
| EuRoC | Stereo + IMU | Competitive with ORB-SLAM3 | Good on most sequences |

Kimera2 (2024) reports outperforming VINS-Fusion on several benchmarks.

**CPU/GPU Requirements:**
- CPU-only (no GPU required)
- High CPU usage: multiple cores required, >100% CPU on Jetson TX2
- Uses Ceres Solver for optimization (CPU-intensive)
- Failed to run on Jetson TX2 in some studies due to CPU limitations

**Jetson Orin Nano Compatibility:**
- Marginal. VINS-Fusion and VINS-Fusion-imu failed on all sequences on Jetson TX2 in published evaluations. Orin Nano has significantly more CPU power, so it may work at reduced rates (10-15 FPS), but this has not been rigorously validated.

**ROS2 Support:**
- No official ROS2 support (upstream uses ROS Kinetic/Melodic)
- Community ROS2 Humble ports available:
  - [JanekDev/VINS-Fusion-ROS2-humble](https://github.com/JanekDev/VINS-Fusion-ROS2-humble) — includes ARM support
  - [fanhong-li/VINS-Fusion-ROS2-Humble](https://github.com/fanhong-li/VINS-Fusion-ROS2-Humble) — RealSense D435i support
  - [zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) — GPU enable/disable
- VINS-Fusion has been used with RTAB-Map as a front-end odometry source in ROS2.

**Loop Closure:** Yes. Optional loop fusion module using DBoW2.

**Map Persistence:** Not documented. No native map save/load functionality.

**Strengths:**
- Excellent visual-inertial fusion (tight coupling)
- GPS fusion capability
- Strong on drone/aerial platforms
- Good EuRoC/KITTI performance

**Weaknesses:**
- GPLv3 license
- Very high CPU usage — may not be viable on Orin Nano with concurrent workloads
- No native ROS2 support
- Project appears unmaintained since 2019-2020
- No map save/load
- Failed to run on Jetson TX2 in published studies

**Verdict:** Not recommended for CleanWalker. The CPU demands are too high for the Orin Nano when sharing with YOLO, Nav2, and behavior tree. cuVSLAM provides better visual-inertial fusion with 10x less CPU usage.

---

### 1.6 Kimera / Kimera2

**Repository:** [MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera) (index repo)
**VIO:** [MIT-SPARK/Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)
**ROS2:** [MIT-SPARK/Kimera-VIO-ROS2](https://github.com/MIT-SPARK/Kimera-VIO-ROS2)
**Paper:** Rosinol et al., ICRA 2020 (Kimera); Abate et al., arXiv 2024 (Kimera2)
**Last Update:** January 2025 (Kimera-VIO)
**License:** BSD-2-Clause

**Type:** Hybrid visual-inertial SLAM with metric-semantic reconstruction

**Description:** Kimera is a unique system from MIT SPARK lab that goes beyond traditional SLAM by producing real-time 3D metric-semantic meshes. It has four components: Kimera-VIO (visual-inertial odometry), Kimera-RPGO (robust pose graph optimization), Kimera-Mesher (3D mesh reconstruction), and Kimera-Semantics (semantic labeling). Kimera2 (2024) adds improved feature tracking, multi-input support (mono/stereo/RGB-D/wheel odom), and better outlier rejection.

**Supported Camera Types (Kimera2):**
- Monocular + IMU
- Stereo + IMU (primary)
- RGB-D + IMU
- Pinhole and omni/fisheye models
- Wheel odometry fusion

**Accuracy on Standard Benchmarks:**
- Claims to outperform VINS-Fusion and ORB-SLAM3 on several benchmarks (Kimera2 paper)
- Achieves top performance on the majority of EuRoC datasets
- Average mesh error of 0.35-0.48 m for 3D reconstruction
- Tested on drones, quadrupeds, wheeled robots, and simulated self-driving cars

**CPU/GPU Requirements:**
- Runs in real-time on CPU (no GPU required)
- CPU requirements not precisely documented but VIO runs at 20-30+ Hz
- Semantic module requires GPU for running a segmentation network
- Parallel/sequential execution modes available

**Jetson Orin Nano Compatibility:**
- Not specifically validated on Jetson Orin Nano in published work
- The VIO module (CPU-only) should run on Orin Nano
- Semantic reconstruction module would compete for GPU with YOLO
- No official Jetson deployment documentation

**ROS2 Support:**
- Yes, via [MIT-SPARK/Kimera-VIO-ROS2](https://github.com/MIT-SPARK/Kimera-VIO-ROS2)
- Small repository (5 commits, 60 stars) — minimal maintenance
- Docker-based build system
- Not as mature as RTAB-Map's ROS2 integration

**Loop Closure:** Yes. Disabled by default but can be enabled via `use_lcd` flag. Uses Kimera-RPGO with Graduated Non-Convexity for outlier rejection.

**Map Persistence:** Not documented.

**Strengths:**
- Unique metric-semantic 3D mesh output
- BSD-2-Clause license (very permissive)
- Strong academic backing (MIT SPARK lab)
- Kimera2 improves robustness significantly
- Multi-platform evaluation (including quadrupeds)

**Weaknesses:**
- Not validated on Jetson embedded platforms
- Small ROS2 wrapper (minimal maintenance)
- Semantic module requires GPU (competes with YOLO)
- More complex to deploy than RTAB-Map or cuVSLAM
- Smaller community and ecosystem
- Map persistence not documented

**Verdict:** Interesting for Phase 3+ when semantic mapping becomes valuable. Not recommended for Phase 1 due to limited Jetson validation and immature ROS2 support compared to RTAB-Map and cuVSLAM.

---

### 1.7 DPVO / DROID-SLAM (Deep Learning VO/SLAM)

**DPVO Repository:** [princeton-vl/DPVO](https://github.com/princeton-vl/DPVO)
**DROID-SLAM Repository:** [princeton-vl/DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM)
**Papers:** Teed & Deng, NeurIPS 2021 (DROID-SLAM); Teed et al., NeurIPS 2023 (DPVO)
**DPVO Last Update:** July 2024
**License:** MIT (DPVO); not explicitly stated (DROID-SLAM)

**Type:** Learning-based visual odometry / SLAM

**Description:** DROID-SLAM and its successor DPVO represent the state-of-the-art in learning-based visual odometry. DROID-SLAM uses a differentiable recurrent architecture that performs dense bundle adjustment on all pixels. DPVO improves on DROID-SLAM by using sparse patch tracking instead of dense correspondence, achieving 3x faster runtime with 1/3 the memory while maintaining competitive accuracy. DPV-SLAM (2024, ECCV) extends DPVO with loop closure.

**DPVO Supported Camera Types:**
- Monocular (primary)
- No native stereo, RGB-D, or IMU support
- No IMU fusion

**DROID-SLAM Supported Camera Types:**
- Monocular
- Stereo
- RGB-D

**Accuracy on Standard Benchmarks:**

| System | Dataset | Metric | Value | Comparison |
|--------|---------|--------|-------|------------|
| DROID-SLAM | EuRoC (mono) | ATE | 82% lower error vs next best (zero failures) | 43% lower than ORB-SLAM3 |
| DROID-SLAM | EuRoC (stereo) | ATE | 71% lower error vs ORB-SLAM3 | State-of-the-art |
| DROID-SLAM | TartanAir | ATE | 62% lower than best prior (mono) | Competition winner |
| DROID-SLAM | TUM-RGBD | ATE | 0 failures across all sequences | Most robust |
| DPVO | TUM-RGBD | ATE | 9% lower avg error than DROID-VO | With 1/3 memory |
| DPVO | EuRoC | Speed | 120 FPS using 2.5 GB | Still outperforms prior work |
| DPV-SLAM | EuRoC | Speed | 2.5x faster than DROID-SLAM | With loop closure |

**CPU/GPU Requirements:**

| System | GPU VRAM (Demo) | GPU VRAM (Training) | Speed |
|--------|-----------------|---------------------|-------|
| DROID-SLAM | **11+ GB minimum** | 24 GB (4x RTX 3090) | 8-30 FPS depending on dataset |
| DPVO (Default) | **4.9 GB** | — | 60 FPS on RTX 3090 |
| DPVO (Fast) | **2.5 GB** | — | 120 FPS on RTX 3090 |

**Jetson Orin Nano Compatibility:**
- **Not viable for Phase 1.** The Orin Nano has 8 GB shared between CPU and GPU. DROID-SLAM requires 11+ GB VRAM. Even DPVO Fast at 2.5 GB would consume a large portion of the shared memory, leaving insufficient headroom for YOLO, terrain segmentation, and other GPU tasks.
- DPVO-QAT++ (November 2025) applies quantization optimization achieving 30-52% FPS increase, but still requires significant GPU memory.
- Future work: SPAQ-DROID-SLAM applies 20% structured pruning + 8-bit PTQ, reducing FLOPs by 18.9% and model size by 79.8%, but embedded deployment is still under research.

**ROS2 Support:** No. Neither DPVO nor DROID-SLAM has ROS2 integration. They are research codebases.

**Loop Closure:**
- DROID-SLAM: Yes (through global bundle adjustment)
- DPVO: No (VO only)
- DPV-SLAM: Yes (deep learning backend + optional classical DBoW2)

**Map Persistence:** No.

**Strengths:**
- State-of-the-art accuracy (significantly better than classical methods)
- Zero failure rate on standard benchmarks
- DPVO is remarkably efficient for a learning-based method
- MIT license (DPVO)
- Active research with optimization variants (QAT++, SPAQ)

**Weaknesses:**
- Requires too much GPU memory for Orin Nano with concurrent workloads
- No ROS2 integration
- No IMU fusion (DPVO is monocular only)
- No map persistence
- No occupancy grid generation
- Research code, not production-ready
- Requires CUDA 11+ and modern GPU architecture

**Verdict:** Not viable for CleanWalker Phase 1-2. The GPU memory requirements are incompatible with an 8 GB shared-memory device running multiple concurrent GPU workloads. Monitor DPVO-QAT++ and embedded optimizations for potential Phase 3+ consideration when hardware upgrades (Orin NX 16GB) may be available.

---

## 2. Comparison Tables

### 2.1 Feature Comparison Matrix

| Feature | ORB-SLAM3 | RTAB-Map | Stella-VSLAM | cuVSLAM | VINS-Fusion | Kimera2 | DPVO/DROID |
|---------|-----------|----------|--------------|---------|-------------|---------|------------|
| **Type** | Feature-based | Appearance-based | Feature-based | Hybrid (GPU) | Optimization-based | Hybrid VIO | Learning-based |
| **Mono** | Yes | Yes | Yes | Yes | Yes (+ IMU) | Yes (+ IMU) | Yes |
| **Stereo** | Yes | Yes | Yes | Yes | Yes | Yes | Yes (DROID) |
| **RGB-D** | Yes | Yes | Yes | Yes (new) | No | Yes | Yes (DROID) |
| **IMU Fusion** | Yes (tight) | Via EKF | **No** | Yes | Yes (tight) | Yes (tight) | **No** |
| **LiDAR Fusion** | No | **Yes** | No | No | No | No | No |
| **Loop Closure** | Yes | Yes | Yes | Yes | Yes | Yes | Yes (DROID/DPV) |
| **Map Save/Load** | Partial | **Full** | Yes | Limited | No | No | No |
| **Occupancy Grid** | No | **Yes (native)** | No | No | No | 3D mesh | No |
| **ROS2 Humble** | Community | **Native** | Partial | **Native** | Community | Minimal | **No** |
| **GPU Required** | No | No | No | **Yes (NVIDIA)** | No | Optional | **Yes** |
| **License** | GPLv3 | BSD-3 | BSD-2 | Apache-2.0 | GPLv3 | BSD-2 | MIT |
| **Maintained (2026)** | No | **Yes** | Yes | **Yes** | No | Yes | Partially |

### 2.2 Accuracy Comparison (EuRoC MAV Dataset)

| System | Mode | ATE RMSE (approx.) | Failures | Notes |
|--------|------|---------------------|----------|-------|
| cuVSLAM | Stereo SLAM | **~5.4 cm** | 0 | GPU-accelerated, best overall |
| ORB-SLAM3 | Stereo-Inertial | **~3.6 cm** | 0-1 | Best with IMU fusion |
| ORB-SLAM3 | Stereo | ~5-8 cm | 0-1 | Without IMU |
| DROID-SLAM | Stereo | ~2-4 cm* | 0 | Requires 11+ GB VRAM |
| DPVO | Mono | ~4-7 cm* | 0 | Monocular only |
| Kimera2 | Stereo-Inertial | ~4-6 cm | 0 | Claims better than ORB-SLAM3 |
| RTAB-Map | Stereo | ~5-15 cm | 0 | More variable, robust |
| VINS-Fusion | Stereo-Inertial | ~5-10 cm | 0 | Good with IMU |
| Stella-VSLAM | Stereo | ~8-15 cm | Low | ORB-SLAM2 level |

*Learning-based methods achieve top accuracy but require desktop GPUs.

### 2.3 Jetson Orin Nano Super Resource Usage

| System | CPU Cores | GPU % | RAM (MB) | FPS | Viable? |
|--------|-----------|-------|----------|-----|---------|
| **cuVSLAM** (stereo, 640x360) | **0.5** | **9%** | ~200 | **30** | **Best fit** |
| **RTAB-Map** (stereo + 2D LiDAR) | 2.0 | 5-10% | 500 | 5-10 | **Good fit** |
| ORB-SLAM3 (stereo-inertial) | 3.0 | 0% | 400-800 | 10-15 | Tight on CPU |
| VINS-Fusion (stereo-inertial) | 3-4 | 0% | 300-500 | 10-15 | Marginal |
| Kimera2 (stereo-inertial) | 2-3 | 0-15%* | 300-500 | 15-20 | Untested |
| Stella-VSLAM (stereo) | 2-3 | 0% | 300-500 | 15-20 | OK but no IMU |
| DPVO (mono) | 1 | **40-60%** | 2500+ | 30-60 | **Too much GPU** |
| DROID-SLAM (stereo) | 1-2 | **70-90%** | 4000+ | 8-20 | **Not viable** |

*Kimera semantics module uses GPU if enabled.

### 2.4 License Compatibility with AGPL-3.0 Project

| System | License | AGPL-3.0 Compatible? | Commercial Use? |
|--------|---------|----------------------|-----------------|
| cuVSLAM (Isaac ROS) | Apache-2.0 | Yes | Yes (free) |
| RTAB-Map | BSD-3 | Yes | Yes (if no SURF) |
| Stella-VSLAM | BSD-2 | Yes | Yes |
| Kimera | BSD-2 | Yes | Yes |
| DPVO | MIT | Yes | Yes |
| ORB-SLAM3 | **GPLv3** | Yes | Requires contact |
| VINS-Fusion | **GPLv3** | Yes | Requires contact |

---

## 3. NVIDIA Isaac ROS Visual SLAM Deep Dive

### 3.1 Is It Free?

**Yes.** Isaac ROS Visual SLAM is released under the Apache-2.0 license and is free to use, modify, and distribute. The cuVSLAM core library is provided as a pre-compiled binary (closed source) but distributed freely as part of the Isaac ROS ecosystem. There is no paid tier or license fee.

The only requirement is NVIDIA GPU hardware (Jetson or discrete GPU with Ampere+ architecture).

### 3.2 Performance on Orin Nano

Based on the cuVSLAM paper and NVIDIA documentation:

| Configuration | Resolution | FPS | CPU Usage | GPU Usage | Notes |
|---------------|-----------|-----|-----------|-----------|-------|
| Stereo VO | 640x360 | 30 | ~5% | ~7% | Minimal overhead |
| Multi-Stereo VO (2 cams) | 640x360 | 30 | ~8.3% | ~9% | Barely more than single stereo |
| Stereo SLAM (with LC) | 640x360 | 30 | ~10% | ~12% | Loop closure adds overhead |
| Mono-Depth | 424x240 | 30 | ~5% | ~8% | Minimum resolution for stability |
| Stereo-Inertial | 640x480 | 30 | ~8% | ~10% | IMU at 200 Hz |

For comparison, on AGX Orin 64GB:
- Stereo mode: 1.8 ms/frame (555 FPS theoretical)
- Multi-stereo (4 cameras): 2.3 ms/frame
- Can achieve 232 FPS at 720p

The Orin Nano is roughly 3-4x slower than AGX Orin, but still far exceeds real-time requirements at 30 FPS.

### 3.3 Integration with CleanWalker Stack

cuVSLAM fits exceptionally well into the existing architecture:

```
OAK-D Pro Stereo → cuVSLAM (GPU, 9% usage) → /visual_odom
OAK-D Pro IMU    → cuVSLAM (visual-inertial) → /visual_odom
                              │
                              ▼
                    robot_localization (EKF)
                    fuses: visual_odom + wheel_odom + GPS
                              │
                              ▼
                    RTAB-Map (mapping only mode)
                    input: /odom_filtered + /camera/depth + /scan
                    output: /map (OccupancyGrid) + loop closure
```

This hybrid approach uses cuVSLAM for high-rate, GPU-efficient visual odometry while RTAB-Map handles mapping, loop closure refinement, and occupancy grid generation.

---

## 4. Camera + IMU Fusion Hardware

### 4.1 OAK-D Pro Built-in IMU

The OAK-D Pro (already selected for CleanWalker) includes an onboard IMU:

| Chip | Type | Axes | Gyro Max Rate | Accel Max Rate | Notes |
|------|------|------|---------------|----------------|-------|
| **BNO086** | 9-axis (accel + gyro + mag) | 9 | 1000 Hz (stable at 400 Hz) | 400 Hz | Preferred chip; includes sensor fusion |
| **BMI270** | 6-axis (accel + gyro) | 6 | 250 Hz (practical max) | 250 Hz | Used in units manufactured Q2 2021 - Q2 2023 |

**Important:** Due to supply chain variation, check which IMU your OAK-D Pro has. The BNO086 is significantly better for SLAM (higher rate, 9-axis with magnetometer). Units manufactured after Q2 2023 should have the BNO086.

The IMU is connected to the OAK-D vision processor via SPI, providing hardware-synchronized timestamps between camera frames and IMU readings. This is critical for visual-inertial SLAM.

### 4.2 Recommended External IMU Options

If the OAK-D Pro's built-in IMU proves insufficient (e.g., BMI270 variant with limited rate), external IMUs can be added:

| IMU | Type | Gyro Rate | Accel Rate | Interface | Price | Best For |
|-----|------|-----------|------------|-----------|-------|----------|
| **Bosch BNO085/086** | 9-axis | 1000 Hz | 400 Hz | I2C/SPI | $15-25 | General robotics SLAM |
| **TDK ICM-42688-P** | 6-axis | 32 kHz | 32 kHz | SPI/I2C/I3C | $3-8 | High-rate applications, drones |
| **Bosch BMI270** | 6-axis | 6.4 kHz | — | SPI/I2C | $3-5 | Ultra-low power, wearables |
| **InvenSense ICM-20948** | 9-axis | 1100 Hz | 4500 Hz | SPI/I2C | $10-15 | Cost-effective 9-axis |
| **VectorNav VN-100** | 9-axis + AHRS | 800 Hz | 800 Hz | UART/SPI | $500+ | Industrial grade, best accuracy |

**Recommendation for CleanWalker:**
1. **Phase 1:** Use OAK-D Pro built-in IMU (BNO086 variant). It provides hardware-synchronized timestamps and 400 Hz rate, sufficient for visual-inertial SLAM.
2. **Phase 2 (if needed):** Add an external ICM-42688-P on the robot body for higher-rate body dynamics sensing, fused via robot_localization EKF alongside visual-inertial odometry.

### 4.3 Time Synchronization Considerations

For visual-inertial SLAM, camera-IMU time synchronization is critical:

- **OAK-D Pro internal IMU:** Hardware-synced via SPI to the vision processor. Timestamps are aligned at the hardware level. This is the ideal setup.
- **External IMU on Jetson GPIO:** Software timestamps only. Jitter of 1-5 ms depending on system load. Acceptable for EKF fusion but not for tight visual-inertial coupling.
- **External IMU with hardware trigger:** Some IMUs (VN-100, ICM-42688-P) support external trigger/sync pins. Can be connected to the OAK-D Pro's sync output for hardware-level alignment.

---

## 5. Outdoor SLAM Challenges & Mitigations

### 5.1 Challenge Matrix

| Challenge | Impact on SLAM | Affected Systems | Severity for CleanWalker |
|-----------|---------------|-----------------|-------------------------|
| **Dynamic objects** (pedestrians, vehicles, animals) | Moving features corrupt pose estimation | All visual SLAM | High — parks have people, dogs, cyclists |
| **Lighting changes** (sun/shade, clouds, time of day) | Feature matching degrades with exposure changes | All visual SLAM | High — outdoor operation all day |
| **Featureless areas** (grass, sand, water, sky) | Insufficient features for tracking | Feature-based SLAM (ORB-SLAM3, Stella) | High — parks = large grass areas |
| **Repetitive textures** (tree trunks, fence posts, pavement tiles) | False loop closures (perceptual aliasing) | All SLAM with loop closure | Medium — park paths can look similar |
| **Weather** (rain, fog, lens condensation) | Reduced visibility, feature quality | All visual SLAM | Medium — robot operates in rain (IP65) |
| **Vegetation changes** (seasonal, growth, leaf fall) | Map becomes invalid over time | Appearance-based loop closure | Medium — maps degrade over weeks/months |
| **GPS multipath** (buildings, tree canopy) | GPS jumps/errors corrupt fusion | Any system using GPS | High — urban parks under canopy |

### 5.2 Mitigation Strategies

**Dynamic Object Handling:**
- Use semantic segmentation to mask known dynamic classes (persons, vehicles, animals) from feature extraction
- Moving consistency check: track features across multiple frames and reject features with inconsistent motion (used in 2025 research: stereo vision-based dynamic object removal)
- ORB-SLAM3 and cuVSLAM already have some built-in outlier rejection for dynamic features
- RTAB-Map can use maximum likelihood for robust estimation

**Lighting Robustness:**
- IMU fusion is critical — when visual tracking degrades due to lighting, IMU provides motion continuity
- cuVSLAM's GPU-based feature extraction handles exposure changes better than CPU implementations
- OAK-D Pro has active IR structured light projection, helping in low-light conditions
- Consider auto-exposure and white balance tuning on the OAK-D Pro
- SuperPoint-SLAM3 (2025) augments ORB-SLAM3 with deep features for better lighting invariance

**Featureless Areas (Grass/Sand/Sky):**
- **LiDAR fusion is the primary mitigation.** RPLidar A1 2D scan provides geometric features even when visual features are scarce.
- RTAB-Map's ability to fuse visual + LiDAR is a key advantage here.
- IMU + wheel odometry can bridge short featureless stretches.
- AirSLAM (2025) introduces PLNet for enhanced robustness in low-texture environments using structural lines.

**Repetitive Textures / Perceptual Aliasing:**
- cuVSLAM's multi-camera support reduces false matches
- RTAB-Map's memory management and voting-based loop closure rejection
- GPS (when available) provides a coarse constraint to reject obviously wrong loop closures
- Kimera2's Graduated Non-Convexity outlier rejection is specifically designed for this

**Long-Term Operation (Seasonal Changes):**
- RTAB-Map's map update capability — maps can be incrementally updated as the environment changes
- Multi-session mapping: build maps across different conditions, merge into a robust composite
- Regular map refresh during operation

### 5.3 Recommended Multi-Sensor Fusion for Outdoor Robustness

```
Priority 1 (Phase 1):
  Stereo Camera (OAK-D Pro) → cuVSLAM visual odometry
  + Built-in IMU (BNO086) → visual-inertial mode
  + 2D LiDAR (RPLidar A1) → RTAB-Map scan matching
  + Wheel/leg odometry → robot_localization EKF
  = Robust outdoor localization even in challenging conditions

Priority 2 (Phase 2, if needed):
  + RTK-GPS (when available) → navsat_transform → EKF
  + 3D LiDAR (Livox Mid-360) → KISS-ICP / LIO-SAM
  = Sub-10cm accuracy in all conditions
```

---

## 6. Occupancy Grid Generation for Path Planning

### 6.1 Which Systems Generate Occupancy Grids?

| System | 2D Occupancy Grid | 3D OctoMap | Method |
|--------|-------------------|------------|--------|
| **RTAB-Map** | **Yes (native)** | **Yes** | Projects 3D point cloud or depth onto 2D grid; configurable via Grid/FromDepth parameter |
| Kimera | No (produces 3D mesh) | 3D mesh only | Mesh output, not grid-based |
| cuVSLAM | No | No | Provides poses only, not maps |
| ORB-SLAM3 | No | No | Sparse point cloud, not dense enough for grids |
| All others | No | No | Odometry/pose output only |

### 6.2 RTAB-Map Occupancy Grid Configuration

RTAB-Map is the only system in this survey that natively generates occupancy grids suitable for Nav2 path planning.

**Published Topics:**
- `/rtabmap/map` — standard 2D occupancy grid (nav_msgs/OccupancyGrid)
- `/rtabmap/proj_map` — projected from 3D cloud onto ground plane
- `/rtabmap/grid_map` — local grid map
- `/rtabmap/octomap_full` — 3D OctoMap (if enabled)

**Grid Generation Modes:**
- From depth image (stereo or RGB-D) — `Grid/FromDepth=true`
- From 2D laser scan — subscribing to `/scan` topic
- From 3D point cloud — subscribing to `/cloud` topic
- Hybrid: depth for close range, LiDAR for coverage

**Grid Parameters:**
- Cell size: configurable, typically 0.05 m x 0.05 m (5 cm resolution)
- Ground detection: automatic with configurable ground height range
- Obstacle detection: configurable minimum/maximum height above ground
- Empty cell filling from projected ground plane

### 6.3 Alternative Approaches for Non-RTAB-Map Systems

If using cuVSLAM (which does not generate maps), occupancy grids can be produced by:

1. **Separate costmap layer from depth camera:** Nav2's costmap_2d can directly consume depth images via the `voxel_layer` or `obstacle_layer` plugin, creating a local obstacle map without SLAM.

2. **LiDAR-based costmap:** RPLidar A1 scans feed directly into Nav2 costmap's `obstacle_layer`. This provides real-time local obstacle avoidance.

3. **RTAB-Map in mapping-only mode:** Use cuVSLAM for odometry, feed its output into RTAB-Map configured for mapping (not tracking). RTAB-Map consumes the odom + depth + scan to build the occupancy grid while cuVSLAM handles the pose estimation.

4. **octomap_server:** Convert point clouds to 3D OctoMap, then project to 2D grid. Works with any SLAM that provides a global point cloud.

**Recommended approach:** cuVSLAM (odometry) + RTAB-Map (mapping/grid generation) as described in Section 3.3.

---

## 7. Recommendations for CleanWalker

### 7.1 Phase 1 (Prototype) — Recommended Stack

```
┌─────────────────────────────────────────────────────────┐
│                LOCALIZATION & MAPPING STACK               │
│                                                           │
│  OAK-D Pro ────────┐                                     │
│  (stereo + IMU)    │                                     │
│                    ▼                                     │
│              cuVSLAM (GPU)                               │
│              ~9% GPU, ~5% CPU                            │
│              30 FPS visual-inertial odometry              │
│                    │                                     │
│                    ▼                                     │
│         robot_localization (EKF)                         │
│         fuses: cuVSLAM + wheel odom                     │
│                    │                                     │
│                    ▼                                     │
│            RTAB-Map (mapping mode)                       │
│            input: filtered_odom + depth + /scan          │
│            output: /map (OccupancyGrid)                  │
│            ~2 cores CPU, ~5% GPU                         │
│            5 Hz map update                               │
│                    │                                     │
│         ┌─────────┴──────────┐                           │
│         ▼                    ▼                           │
│    /map (for Nav2)     Loop Closure                     │
│    OccupancyGrid       (corrects drift)                 │
│                                                           │
│  RPLidar A1 ──────► RTAB-Map (scan matching)            │
│  (2D scan)           + Nav2 costmap (obstacle layer)     │
└─────────────────────────────────────────────────────────┘
```

**Why this combination:**
- **cuVSLAM** provides best-in-class visual odometry with minimal resource usage (9% GPU, 5% CPU). This is vastly more efficient than ORB-SLAM3 or VINS-Fusion which would use 3-4 CPU cores.
- **RTAB-Map** provides the mapping, occupancy grid, and loop closure that cuVSLAM lacks. It accepts external odometry and focuses on map building.
- **robot_localization EKF** fuses all odometry sources for a smooth, high-rate pose estimate.
- Both cuVSLAM and RTAB-Map have native ROS2 Humble support.
- Both are freely available under permissive licenses (Apache-2.0 and BSD-3).

**Total SLAM/localization resource budget:**
| Component | CPU Cores | GPU % | RAM (MB) |
|-----------|-----------|-------|----------|
| cuVSLAM | 0.5 | 9% | 200 |
| robot_localization | 0.5 | 0% | 50 |
| RTAB-Map (mapping mode, 5 Hz) | 1.5 | 5% | 500 |
| **Total** | **2.5** | **14%** | **750** |

This is within budget (compare to 7.4 cores / 45-70% GPU total in the architecture doc).

### 7.2 Phase 2 (Production) — Enhancements

1. **Add RTK-GPS fusion** via navsat_transform when available outdoors
2. **Add 3D LiDAR** (Livox Mid-360) with KISS-ICP for LiDAR odometry fused into EKF
3. **Evaluate cuVSLAM multi-stereo** with a second stereo pair for wider coverage
4. **Enable RTAB-Map semantic features** if terrain classification data is available

### 7.3 Fallback Plan

If cuVSLAM proves unreliable in outdoor conditions:
- **Fallback A:** RTAB-Map in full SLAM mode (visual odometry + mapping). This is the current recommendation in the architecture doc and is proven. Cost: +1 CPU core vs cuVSLAM approach.
- **Fallback B:** ORB-SLAM3 stereo-inertial (via community ROS2 wrapper) for odometry + RTAB-Map for mapping. Best accuracy but GPLv3 license concern and higher CPU usage.

### 7.4 Systems NOT Recommended

| System | Reason |
|--------|--------|
| VINS-Fusion | Too CPU-heavy for Orin Nano; failed on TX2; unmaintained |
| DPVO / DROID-SLAM | Requires too much GPU VRAM; no ROS2; no IMU; research-only |
| Stella-VSLAM | No IMU fusion; unclear ROS2 stereo support; less accurate than alternatives |
| Kimera | Not validated on Jetson; minimal ROS2 wrapper; semantic features not needed yet |

---

## Appendix A: Key Repositories

| System | Repository | License | ROS2 |
|--------|-----------|---------|------|
| cuVSLAM | [NVIDIA-ISAAC-ROS/isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) | Apache-2.0 | Native |
| PyCuVSLAM | [NVlabs/PyCuVSLAM](https://github.com/NVlabs/PyCuVSLAM) | Apache-2.0 | Python API |
| RTAB-Map | [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | BSD-3 | Native |
| ORB-SLAM3 | [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | GPLv3 | Community |
| ORB-SLAM3 ROS2 | [suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker) | — | Humble |
| Stella-VSLAM | [stella-cv/stella_vslam](https://github.com/stella-cv/stella_vslam) | BSD-2 | Partial |
| VINS-Fusion | [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | GPLv3 | Community |
| VINS-Fusion ROS2 | [JanekDev/VINS-Fusion-ROS2-humble](https://github.com/JanekDev/VINS-Fusion-ROS2-humble) | — | Humble |
| Kimera | [MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera) | BSD-2 | Minimal |
| Kimera ROS2 | [MIT-SPARK/Kimera-VIO-ROS2](https://github.com/MIT-SPARK/Kimera-VIO-ROS2) | BSD-2 | Humble |
| DPVO / DPV-SLAM | [princeton-vl/DPVO](https://github.com/princeton-vl/DPVO) | MIT | No |
| DROID-SLAM | [princeton-vl/DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) | — | No |
| robot_localization | [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | BSD-3 | Native |

## Appendix B: References

1. Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM," IEEE T-RO, 2021.
2. Labbe & Michaud, "RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation," arXiv:2403.06341, 2024.
3. NVIDIA, "cuVSLAM: CUDA Accelerated Visual Odometry and Mapping," arXiv:2506.04359, 2025.
4. Qin et al., "A General Optimization-based Framework for Global Pose Estimation with Multiple Sensors," arXiv, 2019.
5. Rosinol et al., "Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping," ICRA, 2020.
6. Abate et al., "Kimera2: Robust and Accurate Metric-Semantic SLAM in the Real World," arXiv:2401.06323, 2024.
7. Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras," NeurIPS, 2021.
8. Teed et al., "Deep Patch Visual Odometry," NeurIPS, 2023.
9. Lipson et al., "Deep Patch Visual SLAM," ECCV, 2024.
10. Schmidt et al., "Visual-Inertial SLAM for Unstructured Outdoor Environments: Benchmarking Loop Closing," Journal of Field Robotics, 2025.

---

*This document is a research reference for the CleanWalker SLAM subsystem. It should be updated as new benchmarks, releases, and field testing results become available.*
