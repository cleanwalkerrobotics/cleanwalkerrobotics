# Grasp Planning Systems for Litter-Picking Robot -- State of the Art (2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB shared RAM, 1024 CUDA cores)
**Use Case:** Autonomous litter picking (bottles, cans, wrappers, bags, cigarette butts) from outdoor surfaces
**Date:** February 10, 2026

---

## Executive Summary

This document evaluates seven leading grasp planning systems for deployment on a litter-picking robot powered by a Jetson Orin Nano Super. The key findings are:

1. **GR-ConvNet v2** is the strongest candidate for initial deployment: 20ms inference, 1.9M parameters, planar grasp output, trivially runs on Jetson, open source, and proven 95.4% success rate on household objects.
2. **VGN** offers the best 6-DOF option at 10ms inference with a lightweight 3D-CNN, BSD-3-Clause licensed, though it requires TSDF volume construction overhead.
3. **Dex-Net / FC-GQ-CNN** provides the most mature suction + parallel jaw combined approach (Dex-Net 4.0), but the UC Berkeley non-commercial license is restrictive.
4. **AnyGrasp** achieves the highest reported bin-clearing success rate (93.3% on 300+ unseen objects) but requires a proprietary license and has heavier compute requirements.
5. **Contact-GraspNet** is excellent for cluttered 6-DOF grasping but at ~280ms per scene on a desktop GPU, it will be too slow on Jetson without significant optimization.
6. **GPD** is the most mature classical approach but at >1s inference per scene, it is not suitable for real-time litter picking.
7. **GraspNet-1Billion baseline** is the reference benchmark system (~100ms inference) but not designed as a deployable system.

**Recommendation:** Start with GR-ConvNet v2 for planar grasps (handles bottles, cans, rigid litter), add a suction module for flat/deformable items (wrappers, bags, butts), and migrate to VGN or a TensorRT-optimized AnyGrasp for full 6-DOF capability as the system matures.

---

## Table of Contents

1. [Grasp Planning Systems Comparison](#1-grasp-planning-systems-comparison)
2. [Detailed System Analysis](#2-detailed-system-analysis)
3. [Master Comparison Table](#3-master-comparison-table)
4. [Jetson Orin Nano Feasibility Analysis](#4-jetson-orin-nano-feasibility-analysis)
5. [Gripper Design for Litter Picking](#5-gripper-design-for-litter-picking)
6. [Handling Deformable Objects](#6-handling-deformable-objects)
7. [Depth Camera Integration](#7-depth-camera-integration)
8. [Grasp Planning Latency Requirements](#8-grasp-planning-latency-requirements)
9. [Open vs Closed Loop Grasp Execution](#9-open-vs-closed-loop-grasp-execution)
10. [Litter-Specific Grasping Research](#10-litter-specific-grasping-research)
11. [Recommended Architecture for CleanWalker](#11-recommended-architecture-for-cleanwalker)
12. [Sources](#12-sources)

---

## 1. Grasp Planning Systems Comparison

### Quick-Reference Comparison Table

| System | Approach | Input | Inference | Jetson-Ready | License | Success Rate |
|--------|----------|-------|-----------|-------------|---------|-------------|
| GR-ConvNet v2 | Planar (4-DOF) | RGB-D image | **20ms** | Yes | Open (check repo) | 95.4% household |
| VGN | 6-DOF | TSDF volume | **10ms** | Likely | BSD-3-Clause | ~80% sim, 74% real |
| Dex-Net 4.0 / FC-GQ-CNN | Planar (4-DOF) | Depth image | **~50ms** (FC) | Possible | UC Berkeley Non-Commercial | 95% (ambidextrous) |
| AnyGrasp | 7-DOF | Point cloud | ~100ms | Unconfirmed | Proprietary (licensed SDK) | 93.3% bin-clearing |
| GraspNet-1Billion baseline | 6-DOF | RGB-D + PC | ~100ms | Marginal | Apache 2.0 | Up to 96% (analytic) |
| Contact-GraspNet | 6-DOF | Point cloud | ~280ms | Difficult | NVIDIA Research (non-commercial) | >90% cluttered |
| GPD | 6-DOF | Point cloud | >1s | No (CPU-heavy) | BSD-2-Clause | 93% novel objects |

---

## 2. Detailed System Analysis

### 2.1 GR-ConvNet v2 (Generative Residual Convolutional Network)

**Paper:** "Antipodal Robotic Grasping using Generative Residual Convolutional Neural Network" (IROS 2020); GR-ConvNet v2 (Sensors 2022)
**Code:** https://github.com/skumra/robotic-grasping

**Approach:**
- Planar grasp detection (4-DOF: x, y, angle, width)
- Generates pixel-wise grasp quality, angle, and width maps from input images
- Generative architecture: encoder (3 conv layers) -> 5 residual layers -> decoder (3 transpose conv layers)
- Outputs four images: grasp quality Q, angle Theta, width W (same size as input 224x224)

**Input Requirements:**
- n-channel input image (RGB-D supported, depth-only works)
- 224x224 pixel resolution
- Direct camera input via pyrealsense2 (Intel RealSense)

**Grasp Success Rate:**
- Cornell dataset: 98.8% accuracy
- Jacquard dataset: 95.1% accuracy
- GraspNet dataset: 97.4% accuracy
- Real-world household objects: 95.4%
- Real-world adversarial objects: 93.0%
- Validated on Ravens-10 benchmark for multi-step sequencing

**Inference Speed:**
- 20ms per frame on GPU (50 FPS)
- Real-time multi-grasp detection

**Compute Requirements:**
- Only 1.9 million parameters (k=32, n=4)
- Extremely lightweight compared to other architectures
- Standard PyTorch model, easy to export to TensorRT
- **Jetson Orin Nano: HIGH CONFIDENCE** -- small model, image-based (no point cloud processing), fast inference

**Camera Integration:**
- Intel RealSense: Native support (pyrealsense2 dependency)
- OAK-D: Compatible via DepthAI SDK (RGB-D output)
- ZED: Compatible (provides RGB-D)

**ROS/ROS2 Support:**
- No official ROS wrapper, but trivial to wrap (single-image inference)
- Community implementations exist

**Open Source:** Yes, GitHub public repository. License file in repo (appears permissive based on academic origin)

**Gripper Assumptions:** Parallel jaw (antipodal grasps)

**Strengths for Litter Picking:**
- Extremely fast inference ideal for mobile robot
- Lightweight model fits Jetson memory budget
- RealSense native support matches our camera options
- High accuracy on diverse objects

**Weaknesses for Litter Picking:**
- Planar grasps only -- cannot handle tilted objects or approach from arbitrary angles
- No native suction grasp support
- Trained on tabletop datasets, not outdoor scenes (requires fine-tuning)

---

### 2.2 VGN (Volumetric Grasping Network)

**Paper:** "Volumetric Grasping Network: Real-time 6 DOF Grasp Detection in Clutter" (CoRL 2020, RSS 2021)
**Code:** https://github.com/ethz-asl/vgn

**Approach:**
- Full 6-DOF grasp detection from volumetric (TSDF) input
- 3D convolutional neural network processes a Truncated Signed Distance Function (TSDF)
- Outputs per-voxel grasp quality, gripper orientation (quaternion), and opening width
- Designed for real-time closed-loop grasping in clutter

**Input Requirements:**
- TSDF (Truncated Signed Distance Function) volume
- Constructed from one or more depth images with known camera pose
- Voxel grid representation of the workspace (typically 40^3 voxels for 0.3m workspace)

**Grasp Success Rate:**
- Simulation (clutter): ~80% grasp success rate
- Real-world (Franka Panda): ~74% grasp success rate
- Lower than image-based methods but provides full 6-DOF poses

**Inference Speed:**
- 10ms on GPU -- extremely fast for 6-DOF method
- TSDF construction adds overhead (~20-50ms depending on resolution)
- Total pipeline: ~30-60ms

**Compute Requirements:**
- Lightweight 3D-CNN architecture
- Small voxel grid input (40^3 = 64K voxels)
- **Jetson Orin Nano: LIKELY FEASIBLE** -- 3D convolutions are well-supported by TensorRT, small input volume

**Camera Integration:**
- Intel RealSense D435: Demonstrated in paper (wrist-mounted)
- Any depth camera producing point clouds works (TSDF is camera-agnostic)
- OAK-D, ZED: Compatible via TSDF construction

**ROS/ROS2 Support:**
- ROS Noetic support for visualization and hardware interface
- Can be cloned into catkin workspace
- ROS2 would require porting (straightforward)

**Open Source:** Yes, BSD-3-Clause license (fully permissive, commercial use allowed)

**Gripper Assumptions:** Parallel jaw (Franka Panda gripper demonstrated)

**Strengths for Litter Picking:**
- Full 6-DOF grasp poses enable approaching litter from any angle
- 10ms inference is fastest of any 6-DOF method
- BSD license allows commercial use
- TSDF representation handles noisy outdoor depth data well (averaging)

**Weaknesses for Litter Picking:**
- TSDF construction requires known camera pose (needs arm kinematics or SLAM)
- Lower real-world success rate than image-based methods
- Trained in simulation -- domain gap for outdoor scenes
- Fixed workspace volume may need adaptation for ground-level picking

---

### 2.3 AnyGrasp

**Paper:** "AnyGrasp: Robust and Efficient Grasp Perception in Spatial and Temporal Domains" (IEEE T-RO 2023)
**Code:** https://github.com/graspnet/anygrasp_sdk (SDK only, licensed)

**Approach:**
- Dense 7-DOF grasp generation from point clouds
- Extends GraspNet-1Billion with improved generalization
- Dense supervision strategy with real perception and analytic labels
- Center-of-mass awareness for grasp stability
- Temporal grasp tracking (grasp correspondence across frames)

**Input Requirements:**
- Point cloud (from any RGB-D camera)
- Supports both single-view and multi-view point clouds
- Works with partial, noisy point clouds

**Grasp Success Rate:**
- 93.3% success rate clearing bins with 300+ unseen objects
- 900+ mean-picks-per-hour on single-arm system
- Robust against large depth-sensing noise

**Inference Speed:**
- ~100ms per scene for grasp pose processing
- Overall grasp decision time (including collision detection): <200ms
- Dense prediction mode trades speed for coverage
- Approximately 5-10 FPS effective throughput

**Compute Requirements:**
- Requires CUDA 11.x/12.x (up to 12.8 as of November 2025)
- PyTorch 1.7.1+
- MinkowskiEngine v0.5.4 (sparse 3D convolution library)
- Python 3.6-3.13
- **Jetson Orin Nano: UNCERTAIN** -- MinkowskiEngine compilation on Jetson is non-trivial; 8GB shared memory may be tight for dense grasp generation

**Camera Integration:**
- Camera-agnostic (operates on point clouds)
- Any RGB-D camera that produces point clouds works
- Tested with RealSense cameras in GraspNet ecosystem

**ROS/ROS2 Support:**
- No official ROS wrapper
- SDK is Python-based, wrappable in ROS node

**Open Source:** Partially. SDK library is proprietary and requires license registration (IP restrictions). Application code is open but the core inference engine is a compiled binary.

**Gripper Assumptions:** Parallel jaw (7-DOF includes approach direction, in-plane rotation, depth, width)

**Strengths for Litter Picking:**
- Highest reported success rate on unseen objects
- Temporal tracking enables moving-target grasping (wind-blown litter)
- Robust to depth noise (critical for outdoor use)
- Dense grasp generation gives many options per scene

**Weaknesses for Litter Picking:**
- Proprietary license -- commercial terms unclear
- MinkowskiEngine dependency complicates Jetson deployment
- 100-200ms latency may be acceptable but not ideal
- No Jetson-specific support documented

---

### 2.4 Contact-GraspNet

**Paper:** "Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes" (ICRA 2021)
**Code:** https://github.com/NVlabs/contact_graspnet

**Approach:**
- End-to-end 6-DOF parallel-jaw grasp generation from point clouds
- Treats 3D points as potential grasp contacts (novel representation)
- Reduces full 6-DOF grasp to 4-DOF per contact point (orientation + width)
- Trained on 17 million simulated grasps
- Class-agnostic -- generalizes to unseen objects

**Input Requirements:**
- Raw scene point cloud (from depth camera)
- Optional: object segmentation masks for targeted grasping
- Point cloud input only (no RGB needed, though depth image is the source)

**Grasp Success Rate:**
- >90% success rate on unseen objects in structured clutter
- State-of-the-art at time of publication for 6-DOF cluttered grasping
- Strong generalization from simulation to real-world

**Inference Speed:**
- ~280ms for full scene (desktop GPU)
- ~190ms for local region around target object
- First call to TensorFlow session is slower (~2s warm-up)

**Compute Requirements:**
- Training: 1x NVIDIA GPU >= 24GB VRAM, >= 64GB RAM
- Inference: 1x NVIDIA GPU >= 8GB VRAM (might work with less)
- TensorFlow 1.x based (legacy framework)
- **Jetson Orin Nano: DIFFICULT** -- 8GB shared memory is at the limit; TensorFlow 1.x on Jetson is problematic; PyTorch port exists (community) which may be easier to optimize

**Camera Integration:**
- Camera-agnostic (point cloud input)
- Any depth camera works
- No specific camera drivers included

**ROS/ROS2 Support:**
- Dockerized ROS service available (community: Thomas Lips)
- PyTorch community ports available for easier integration

**Open Source:** Code is public on GitHub. License: NVIDIA Research License (see License.pdf in repo) -- likely non-commercial research use only.

**Gripper Assumptions:** Parallel jaw

**Strengths for Litter Picking:**
- NVIDIA pedigree -- well-documented, high quality code
- Contact-point representation is intuitive for grasping
- Strong generalization to unseen objects
- Object segmentation support enables targeted picking

**Weaknesses for Litter Picking:**
- ~280ms is slow for real-time outdoor operation
- NVIDIA non-commercial license restricts deployment
- TensorFlow 1.x is end-of-life; porting overhead
- 8GB VRAM requirement is exactly Jetson's total shared memory

---

### 2.5 GPD (Grasp Pose Detection)

**Paper:** "Grasp Pose Detection in Point Clouds" (IJRR 2017)
**Code:** https://github.com/atenpas/gpd

**Approach:**
- Sample-and-classify: generates grasp candidates, then classifies via CNN
- Two-stage pipeline: (1) sample grasps from point cloud, (2) CNN evaluates each candidate
- Classical approach using 3D geometry + learned classification
- C++ implementation with PCL dependency

**Input Requirements:**
- 3D point cloud (from one or more depth sensors)
- Camera position(s) for normal estimation
- Works with single or multi-view point clouds

**Grasp Success Rate:**
- 93% with active wrist-mounted sensor
- 84% with fixed external sensors
- 87.88% on unseen objects (real-world Franka experiments, recent variants)

**Inference Speed:**
- >1 second per scene (sampling + CNN classification)
- Parallelized across CPU cores (set num_threads to CPU core count)
- Not suitable for real-time / closed-loop grasping
- Modern variants (GraspFast) achieve ~42 FPS but are separate systems

**Compute Requirements:**
- PCL 1.9+, Eigen 3.0+, OpenCV 3.4+
- Primarily CPU-based (can use GPU for CNN classification)
- **Jetson Orin Nano: POOR FIT** -- CPU-heavy sampling step is slow on ARM; >1s latency unacceptable

**Camera Integration:**
- Generic depth sensor support
- No specific camera drivers
- Works with any point cloud source

**ROS/ROS2 Support:**
- Separate ROS wrapper repository exists
- ROS2 integration available (community dockerized node with custom services)
- MoveIt integration documented (MoveIt Deep Grasps tutorial)

**Open Source:** Yes, BSD-2-Clause license (fully permissive, commercial use allowed)

**Gripper Assumptions:** Parallel jaw (2-finger)

**Strengths for Litter Picking:**
- Most mature and well-tested system
- BSD license is ideal for commercial use
- MoveIt integration is production-ready
- Multi-view point cloud support improves accuracy

**Weaknesses for Litter Picking:**
- >1 second latency is unacceptable for mobile litter picking
- CPU-heavy design is poorly suited to Jetson ARM cores
- No GPU acceleration for sampling step
- Would need complete redesign for real-time use

---

### 2.6 GraspNet-1Billion Baseline

**Paper:** "GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping" (CVPR 2020)
**Code:** https://github.com/graspnet/graspnet-baseline

**Approach:**
- 6-DOF dense grasp detection from point clouds
- PointNet++ backbone for point cloud feature extraction
- Graspness scoring + grasp operation prediction
- Benchmark system for the GraspNet-1Billion dataset (97,280 RGB-D images, 1B+ grasp poses)

**Input Requirements:**
- RGB-D images + camera intrinsics
- Point cloud downsampled with voxel size 0.005m
- XYZ in camera coordinates

**Grasp Success Rate:**
- Up to 96% using force-closure metric (analytic evaluation)
- Benchmark reference -- other methods compared against it

**Inference Speed:**
- Graspness model (GSNet): ~100ms on RealSense data, ~120ms on Kinect data
- Breakdown: Cascaded Graspness Model (80-100ms) + Grasp Operation Model (20ms)
- Fast inference mode: set collision_thresh to -1
- GPD/PointNetGPD: >1s; GSNet: ~0.1s (10x faster)

**Compute Requirements:**
- PointNet++ backbone + custom CUDA kernels
- Requires compilation of custom C++/CUDA extensions
- **Jetson Orin Nano: MARGINAL** -- custom CUDA kernels need recompilation for Jetson; 100ms inference is workable but memory may be tight

**Camera Integration:**
- Intel RealSense: Native support (separate checkpoint trained on RealSense data)
- Kinect Azure: Native support (separate checkpoint)
- OAK-D, ZED: Need point cloud conversion (standard)

**ROS/ROS2 Support:**
- No official ROS wrapper
- Python inference script is straightforward to wrap

**Open Source:** Yes, Apache 2.0 license (permissive, commercial use allowed) -- note: this refers to the baseline code; the AnyGrasp extension has different licensing.

**Gripper Assumptions:** Parallel jaw (6-DOF)

**Strengths for Litter Picking:**
- Gold-standard benchmark system
- Apache 2.0 license
- Camera-specific trained models (RealSense, Kinect)
- 100ms inference is workable

**Weaknesses for Litter Picking:**
- Benchmark system, not optimized for deployment
- Custom CUDA kernels complicate Jetson deployment
- Not designed for real-time closed-loop operation
- No temporal tracking

---

### 2.7 Dex-Net / FC-GQ-CNN

**Paper:** Dex-Net 2.0 (RSS 2017), Dex-Net 4.0 (Science Robotics 2019), FC-GQ-CNN (RA-L 2019)
**Code:** https://github.com/BerkeleyAutomation/gqcnn + https://github.com/BerkeleyAutomation/dex-net

**Approach:**
- Grasp Quality CNN (GQ-CNN) predicts grasp robustness from depth images
- FC-GQ-CNN: Fully convolutional variant generates grasp heatmaps in single pass
- Dex-Net 4.0: "Ambidextrous" -- learns to choose between parallel jaw AND suction gripper
- Trained on 6.7M+ synthetic depth images with analytic grasp metrics
- Uses Cross-Entropy Method (CEM) or fully convolutional inference

**Input Requirements:**
- Depth image only (no RGB needed)
- Single overhead camera view
- Dex-Net 2.0: Parallel jaw grasps from depth
- Dex-Net 3.0: Suction grasps from depth
- Dex-Net 4.0: Both gripper types, policy selects optimal

**Grasp Success Rate:**
- Dex-Net 2.0: 93% on known objects, 80% on novel objects
- Dex-Net 4.0: 95% overall with ambidextrous gripper selection
- 300 picks/hour with ABB YuMi robot
- 2,500+ grasp attempts validated on physical system with 50 novel objects

**Inference Speed:**
- GQ-CNN (CEM): ~800ms per grasp (iterative sampling)
- FC-GQ-CNN: ~50ms for full image (540 inferences/sec at batch 128)
- FC-GQ-CNN: 22x speedup over iterative CEM method
- 296 Mean Picks Per Hour (MPPH) in deployment

**Compute Requirements:**
- Relatively lightweight CNN architecture
- GQ-CNN: small network, runs on modest GPU
- FC-GQ-CNN: fully convolutional, efficient on GPU
- Python 3.5-3.7 (older codebase)
- **Jetson Orin Nano: POSSIBLE** -- FC-GQ-CNN is small enough; needs porting to modern Python/PyTorch and TensorRT optimization

**Camera Integration:**
- Depth-only input (any depth camera works)
- Overhead camera assumption (looking down at bin/surface)
- PhoXi scanner demonstrated; RealSense compatible

**ROS/ROS2 Support:**
- Official ROS integration (ros_nodes, launch files, services, messages)
- ROS1 only (would need porting for ROS2)

**Open Source:** Yes, but UC Berkeley Copyright and Disclaimer Notice -- educational, research, and not-for-profit purposes only. NOT a standard permissive license. Commercial use requires separate agreement.

**Gripper Assumptions:**
- Dex-Net 2.0: Parallel jaw
- Dex-Net 3.0: Suction cup
- Dex-Net 4.0: Both (ambidextrous -- system selects optimal gripper per object)

**Strengths for Litter Picking:**
- Dex-Net 4.0 ambidextrous approach is ideal for mixed litter (rigid + deformable)
- Depth-only input simplifies pipeline
- FC-GQ-CNN is fast enough for real-time
- Most extensively validated system in literature
- Suction grasp support unique among candidates

**Weaknesses for Litter Picking:**
- Non-commercial license blocks product deployment without agreement
- Planar grasps only (overhead view assumption)
- Older Python codebase needs modernization
- Assumes bin-picking setup, not ground-level outdoor scenes

---

## 3. Master Comparison Table

### 3.1 Technical Specifications

| Feature | GR-ConvNet v2 | VGN | AnyGrasp | Contact-GraspNet | GPD | GraspNet Baseline | Dex-Net FC-GQ-CNN |
|---------|--------------|-----|----------|-----------------|-----|-------------------|-------------------|
| **DOF** | 4 (planar) | 6 | 7 | 6 | 6 | 6 | 4 (planar) |
| **Input** | RGB-D image | TSDF volume | Point cloud | Point cloud | Point cloud | RGB-D + PC | Depth image |
| **Architecture** | CNN (encoder-residual-decoder) | 3D-CNN | Sparse 3D CNN (MinkowskiEngine) | PointNet++ variant | Sample + CNN classify | PointNet++ + custom | CNN (FC variant) |
| **Parameters** | 1.9M | ~1M (est.) | ~10M+ (est.) | ~5M (est.) | ~2M (classifier) | ~5M (est.) | ~1M |
| **Inference (GPU)** | 20ms | 10ms (+ TSDF) | ~100ms | ~280ms | >1000ms | ~100ms | ~50ms (FC) |
| **Min GPU VRAM** | <2GB | <2GB | ~4GB+ | 8GB | ~2GB (classifier) | ~4GB | <2GB |
| **Framework** | PyTorch | PyTorch | PyTorch + MinkowskiEngine | TensorFlow 1.x | C++ / PCL | PyTorch + custom CUDA | TensorFlow |

### 3.2 Deployment Feasibility

| Feature | GR-ConvNet v2 | VGN | AnyGrasp | Contact-GraspNet | GPD | GraspNet Baseline | Dex-Net FC-GQ-CNN |
|---------|--------------|-----|----------|-----------------|-----|-------------------|-------------------|
| **Jetson Feasibility** | HIGH | HIGH | LOW-MEDIUM | LOW | VERY LOW | MEDIUM | MEDIUM |
| **TensorRT Export** | Easy | Moderate | Difficult | Moderate (PyTorch port) | N/A | Difficult | Moderate |
| **License** | Permissive | BSD-3-Clause | Proprietary SDK | NVIDIA Research | BSD-2-Clause | Apache 2.0 | UC Berkeley Non-Commercial |
| **Commercial Use** | Yes | Yes | Requires license | No | Yes | Yes | No (without agreement) |
| **ROS Support** | No (easy wrap) | ROS Noetic | No | Community Docker | ROS wrapper | No (easy wrap) | ROS1 official |
| **ROS2 Support** | No | No | No | No | Community | No | No |
| **Active Maintenance** | Moderate | Low | Active | Low | Low | Active | Low |

### 3.3 Grasp Quality

| Feature | GR-ConvNet v2 | VGN | AnyGrasp | Contact-GraspNet | GPD | GraspNet Baseline | Dex-Net FC-GQ-CNN |
|---------|--------------|-----|----------|-----------------|-----|-------------------|-------------------|
| **Benchmark Accuracy** | 98.8% Cornell | ~80% sim | 93.3% bin-clear | >90% clutter | 93% active | 96% analytic | 95% Dex-Net 4.0 |
| **Real-World Rate** | 95.4% | 74% | 93.3% | >90% | 84-93% | N/A | 95% |
| **Novel Objects** | 93.0% | ~74% | 93.3% (300+ objects) | >90% | 87.88% | N/A | 80-95% |
| **Clutter Handling** | Moderate | Good | Excellent | Excellent | Good | Good | Moderate |
| **Suction Support** | No | No | No | No | No | Yes (SuctionNet) | Yes (Dex-Net 3.0/4.0) |

---

## 4. Jetson Orin Nano Feasibility Analysis

### 4.1 Jetson Orin Nano Super Specifications

| Parameter | Value |
|-----------|-------|
| AI Performance | 67 TOPS (INT8) |
| GPU | 1024-core NVIDIA Ampere, 32 Tensor Cores |
| Memory | 8GB LPDDR5 (shared CPU/GPU) |
| Memory Bandwidth | 102 GB/s |
| CUDA | 12.6 |
| TensorRT | 10.7.0 |
| Power | 7-15W envelope |

### 4.2 Memory Budget Analysis

With 8GB shared between CPU and GPU, a realistic allocation for grasp planning is:

| Component | Memory Budget |
|-----------|--------------|
| OS + System | ~1.5GB |
| Object Detection (YOLO) | ~0.5-1.5GB |
| Depth Processing Pipeline | ~0.5GB |
| **Grasp Planning Model** | **~1-2GB available** |
| Point Cloud / TSDF Buffer | ~0.5GB |
| ROS/Application overhead | ~0.5GB |

This means grasp planning models must fit within approximately 1-2GB. This eliminates Contact-GraspNet (needs 8GB VRAM alone) and makes AnyGrasp risky.

### 4.3 TensorRT Optimization Impact

TensorRT INT8 quantization typically delivers 2-5x speedup with minimal accuracy loss:

| Model | Native Inference | Est. TensorRT INT8 | Feasible on Jetson? |
|-------|-----------------|--------------------|--------------------|
| GR-ConvNet v2 | 20ms | ~5-10ms | YES -- excellent |
| VGN | 10ms (+ TSDF) | ~3-5ms (+ TSDF) | YES -- excellent |
| FC-GQ-CNN | 50ms | ~15-25ms | YES -- good |
| GraspNet Baseline | 100ms | ~30-50ms | MARGINAL -- custom CUDA kernels complicate export |
| AnyGrasp | 100ms | ~30-50ms (if exportable) | UNCERTAIN -- MinkowskiEngine not TensorRT-friendly |
| Contact-GraspNet | 280ms | ~80-140ms | UNLIKELY -- memory constraints |
| GPD | >1000ms | N/A (CPU-bound) | NO |

### 4.4 Recommended Jetson Deployment Strategy

**Phase 1 (MVP):** GR-ConvNet v2 with TensorRT
- Export PyTorch model to ONNX, then TensorRT INT8
- Expected inference: 5-10ms on Jetson
- Total memory footprint: <500MB
- Handles rigid litter (bottles, cans) via planar grasps

**Phase 2 (Enhanced):** Add VGN for 6-DOF grasps
- Run alongside or as alternative to GR-ConvNet
- Use for objects requiring non-planar approach angles
- TSDF construction from RealSense depth stream

**Phase 3 (Advanced):** Investigate AnyGrasp or custom model
- If MinkowskiEngine can be compiled for JetPack
- Or train a custom lightweight 6-DOF model on litter-specific data

---

## 5. Gripper Design for Litter Picking

### 5.1 Gripper Type Comparison for Litter

| Gripper Type | Bottles | Cans | Wrappers | Plastic Bags | Cigarette Butts | Wet Items | Cost |
|-------------|---------|------|----------|-------------|----------------|-----------|------|
| **Parallel Jaw** | Excellent | Excellent | Poor | Poor | Very Poor | Good | $50-200 |
| **Suction Cup** | Good | Good | Good (flat) | Poor (porous) | Good (flat surface) | Poor | $30-100 |
| **Soft Gripper (Fin Ray)** | Good | Good | Good | Moderate | Moderate | Good | $100-400 |
| **Hybrid (Fingers + Suction)** | Excellent | Excellent | Good | Moderate | Good | Good | $200-500 |
| **Vacuum/Scoop** | Poor | Poor | Excellent | Excellent | Excellent | Moderate | $100-300 |

### 5.2 Recommended Gripper: Hybrid Soft + Suction

For litter picking, a **hybrid gripper combining compliant fingers with integrated suction** is optimal:

**Design Concept:**
- Two compliant Fin Ray-type fingers for shape-adaptive grasping
- Small suction cup integrated into palm or fingertip
- Retractable suction (activated for flat/deformable items)
- Total weight target: <250g
- Material: Silicone fingers + PLA/ABS structure (UV-resistant coating for outdoor use)

**Rationale:**
- Fin Ray fingers conform to irregular shapes (crumpled cans, bottles)
- Suction handles flat items (wrappers, cigarette butts on pavement)
- Hybrid approach demonstrated in literature at 80% success on waste items
- Material cost: $20-40 for rapid prototyping
- LitterBot research validated Fin Ray + RealSense D415 for autonomous litter sorting

**Design References:**
- LitterBot (Frontiers in Robotics and AI, 2022): Fin Ray soft gripper + UR10 arm + RealSense D415
- Hybrid vacuum-actuated multi-material gripper (RoboSoft 2024): 250g, lifts up to 2kg
- Compact underactuated gripper with retractable suction cup (Frontiers, 2023)
- 3D-printed two-finger V-shaped gripper for lightweight waste collection (Robotics MDPI, 2025): 80% success rate, $20-40 cost

### 5.3 Gripper-to-Grasp-Planner Mapping

| Litter Type | Gripper Mode | Grasp Planner | Grasp Type |
|-------------|-------------|---------------|------------|
| Bottles, cans | Finger grasp | GR-ConvNet v2 | Planar antipodal |
| Flat wrappers | Suction | Dex-Net 3.0 / heuristic | Top-down suction |
| Plastic bags | Finger pinch + lift | GR-ConvNet v2 (fine-tuned) | Planar pinch |
| Cigarette butts | Suction | Heuristic (center of detection) | Top-down suction |
| Crumpled items | Adaptive fingers | VGN (Phase 2) | 6-DOF |

---

## 6. Handling Deformable Objects

### 6.1 Challenge Categories

| Object Type | Deformation | Key Challenge | Strategy |
|-------------|------------|---------------|----------|
| Plastic bags | High -- wind, crumpling | Shape varies dramatically; porous (suction fails) | Pinch edges + lift; or scoop/vacuum |
| Wrappers (flat) | Low-Medium | Thin, flat on ground; hard to grip edges | Suction from above; or slide finger underneath |
| Wrappers (crumpled) | Medium | Irregular shape, compressible | Adaptive finger grasp; treat as rigid |
| Wet paper/tissue | High | Tears easily; sticks to surface | Gentle suction; wide soft gripper |
| Cigarette butts | None (rigid but small) | Very small (~7mm diameter); on varied surfaces | Suction cup precisely positioned; or fine-tip fingers |

### 6.2 Strategies for Deformable Litter

**Plastic Bags:**
- Primary: Fin Ray fingers pinch any protruding edge and lift
- Secondary: Blow air jet to create liftable edge, then pinch
- Literature: Dual-arm shaking methods for bag opening (ShakingBot, 2023); vest-bag manipulation research
- Fallback: Skip and flag for human collection

**Flat Wrappers on Pavement:**
- Primary: Suction cup placed on center of detected wrapper
- Secondary: Slide thin rigid spatula finger underneath to peel off surface
- Literature: RoTipBot (2024) uses rotatable tactile sensors for thin, flexible objects
- Key insight: Passive compliance in soft hands facilitates adaptive thin-object manipulation

**General Deformable Strategy:**
- Use tactile feedback to detect grasp success (force/torque sensing)
- Implement re-grasp policy: if first attempt fails, try alternative grasp type
- Center-of-mass estimation (AnyGrasp feature) improves stability on irregular items

### 6.3 Sensing Requirements for Deformable Objects

- **Tactile sensing:** Force-sensitive resistors or optical tactile sensors in fingertips
- **Close-range depth:** Wrist-mounted camera for pre-grasp verification
- **Grasp success detection:** Current monitoring on gripper actuator or tactile threshold

---

## 7. Depth Camera Integration

### 7.1 Camera Comparison for Grasp Planning

| Feature | Intel RealSense D435/D455 | Luxonis OAK-D Pro | Stereolabs ZED 2i |
|---------|--------------------------|-------------------|-------------------|
| **Depth Technology** | Active IR stereo | Stereo + structured light | Passive stereo + neural |
| **Range** | 0.1-10m | 0.2-15m (OAK-D Pro) | 0.3-20m |
| **Close Range** | 0.1m (D435) | 0.2m | 0.3m |
| **Depth Resolution** | 1280x720 | 1280x800 | 2208x1242 |
| **Depth FPS** | 90 fps | 120 fps | 100 fps |
| **Point Cloud** | Yes (via SDK) | Yes (via DepthAI) | Yes (via SDK) |
| **On-device AI** | No | Yes (Myriad X VPU) | No |
| **Outdoor Performance** | Fair (IR interference in sunlight) | Good (structured light + stereo) | Good (neural depth) |
| **ROS2 Support** | Official | Official | Official |
| **Price** | $200-300 | $250-350 | $450-550 |
| **Grasp Planner Compatibility** | All 7 systems | All 7 systems | All 7 systems |

### 7.2 Integration Architecture

```
[Depth Camera] --> [Point Cloud Generator] --> [Grasp Planner]
                          |                          |
                    [RGB Stream] -----> [YOLO Litter Detection]
                          |                          |
                    [Crop Region] -----> [Targeted Grasp Generation]
```

**Pipeline:**
1. YOLO detects litter in RGB stream, outputs bounding boxes
2. Bounding box projected into depth frame to extract local point cloud
3. Local point cloud fed to grasp planner (reduces compute vs. full scene)
4. Grasp planner returns ranked grasp poses
5. Best grasp executed by arm controller

### 7.3 Camera Selection Recommendation

**Primary: Intel RealSense D455** (arm-mounted, close-range grasp planning)
- Closest minimum range (0.1m) is critical for pre-grasp verification
- Native RealSense checkpoint support in GraspNet ecosystem
- Proven in LitterBot and multiple grasp research systems
- Direct pyrealsense2 support in GR-ConvNet

**Secondary: Luxonis OAK-D** (if already used for navigation/detection)
- On-device AI offloads YOLO from Jetson
- Better outdoor depth with active stereo
- Slightly worse close-range (0.2m minimum)

---

## 8. Grasp Planning Latency Requirements

### 8.1 Latency Budget for Litter Picking

| Phase | Target Latency | Notes |
|-------|---------------|-------|
| Litter Detection (YOLO) | <30ms | Already validated in edge-detection research |
| Depth Acquisition | <15ms | Single depth frame capture |
| Point Cloud / TSDF Construction | <20ms | Local region only (from detection bbox) |
| **Grasp Planning** | **<100ms** | **Critical bottleneck** |
| Motion Planning (IK) | <50ms | Analytical IK for simple arm |
| Arm Execution | 500-2000ms | Physical movement time |
| **Total Perception-to-Motion** | **<200ms** | Required for closed-loop operation |

### 8.2 Latency Analysis by System

| System | Grasp Planning Latency | Meets <100ms? | Meets <200ms? |
|--------|----------------------|---------------|---------------|
| GR-ConvNet v2 | 20ms (5-10ms TRT) | YES | YES |
| VGN | 10ms + 20-50ms TSDF | YES | YES |
| FC-GQ-CNN | 50ms (15-25ms TRT) | YES | YES |
| AnyGrasp | ~100ms | BORDERLINE | YES |
| GraspNet Baseline | ~100ms | BORDERLINE | YES |
| Contact-GraspNet | ~280ms | NO | NO |
| GPD | >1000ms | NO | NO |

### 8.3 Practical Latency Considerations

- **Static litter (on ground):** 200-500ms total perception-to-motion is acceptable since litter does not move. Open-loop grasp execution works.
- **Wind-blown litter:** <200ms closed-loop required; only GR-ConvNet, VGN, and FC-GQ-CNN qualify.
- **Robot approaching litter:** The robot walks to the litter location, stops, then picks. This gives 1-2 seconds of "think time" during approach. Even slower planners could work in an open-loop paradigm.
- **Key insight:** For a walking robot that stops to pick, the bottleneck is arm movement speed (500-2000ms), not perception. A 100-200ms grasp planner is fast enough for static ground litter.

---

## 9. Open vs Closed Loop Grasp Execution

### 9.1 Comparison

| Aspect | Open Loop | Closed Loop |
|--------|-----------|-------------|
| **Approach** | Plan grasp from single view, execute without feedback | Continuously update grasp during approach |
| **Speed** | Faster (one inference) | Slower (continuous inference) |
| **Accuracy** | Depends on calibration quality | Self-correcting |
| **Moving Objects** | Cannot handle | Handles well |
| **Required Latency** | <500ms acceptable | <100ms per update required |
| **Compute Cost** | Low | High (continuous inference) |
| **Suitable Systems** | All | GR-ConvNet, VGN, FC-GQ-CNN only |

### 9.2 Recommendation for Litter Picking

**Hybrid approach:**
1. **Open-loop for static ground litter** (90% of cases): Robot detects litter, plans grasp during approach, executes single-shot pick
2. **Closed-loop for edge cases**: If first pick fails (tactile feedback), use wrist camera for close-range re-planning and visual servo to grasp

**Rationale:** Ground litter does not move (unless wind-blown). Open-loop grasping reduces compute requirements and allows use of slower but more accurate planners. Visual servoing (IBVS/PBVS) provides correction for the final 5cm of approach.

**Visual Servoing Integration:**
- Image-Based Visual Servoing (IBVS): Uses camera RGB to guide robot to grasp pose
- Position-Based Visual Servoing (PBVS): Uses 3D pose estimation for correction
- Hybrid: Open-loop coarse approach + closed-loop fine positioning (last 5cm)

---

## 10. Litter-Specific Grasping Research

### 10.1 Key Papers

**LitterBot (Frontiers in Robotics and AI, 2022)**
- UR10 arm + Fin Ray soft gripper + RealSense D415
- Deep learning detection + mask-based target selection
- High grasp success rate on sorted litter categories
- Demonstrates end-to-end autonomous litter sorting
- Source: "Autonomous detection and sorting of litter using deep learning and soft robotic grippers"

**Vision and Tactile Robotic System for Outdoor Litter (JIRS, 2023)**
- Color image detection + depth-based 3D localization
- Tactile-based grasping estimation using low-cost visual tactile sensors
- 94% average detection and collection success rate
- 80% first-attempt collection rate
- 3-DOF grasp estimation for robot arm
- Source: "Vision and Tactile Robotic System to Grasp Litter in Outdoor Environments"

**Deep Learning Robot for Grass Environments (2019)**
- GPU-accelerated garbage recognition and segmentation
- Vision-based grasping controllers for manipulator
- Path planning, obstacle avoidance, and environment perception
- Source: "Deep Learning Based Robot for Automatically Picking up Garbage on the Grass"

**VERO Cigarette Butt Robot (Hackaday, 2024)**
- Locomotion-integrated pickup (foot-based suction)
- Combines walking with cigarette butt detection
- Novel approach: plant foot next to butt, suction through sole

**3D-Printed V-Shaped Gripper for Waste Collection (Robotics MDPI, 2025)**
- Two-finger V-shaped profile gripper
- Up to 80% grasp success on typical waste items
- Rapid prototyping: ~10 hours print time
- Material cost: $20-40
- Validated on bottles, cups, plastic bags

### 10.2 Key Findings from Litter Research

1. **Soft grippers outperform rigid grippers** for mixed litter (varied shapes, materials)
2. **Tactile feedback significantly improves** first-attempt success rate (80% -> 94% with re-grasp)
3. **Depth cameras are essential** -- RGB alone insufficient for outdoor grasp planning
4. **Category-specific strategies** work better than one-size-fits-all grasping
5. **3-DOF planar grasps are sufficient** for most ground litter (validates GR-ConvNet approach)
6. **Low-cost grippers ($20-40)** can achieve viable success rates -- expensive hardware not required
7. **Outdoor depth sensing** remains challenging (sunlight interference, reflective surfaces)

---

## 11. Recommended Architecture for CleanWalker

### 11.1 System Architecture

```
                    +--------------------+
                    |   Depth Camera     |
                    | (RealSense D455)   |
                    +--------+-----------+
                             |
                    +--------v-----------+
                    | YOLO Litter        |
                    | Detection          |
                    | (5-10ms TensorRT)  |
                    +--------+-----------+
                             |
                    +--------v-----------+
                    | ROI Extraction     |
                    | (crop depth/PC to  |
                    |  detection bbox)   |
                    +--------+-----------+
                             |
              +--------------+---------------+
              |                              |
    +---------v----------+     +-------------v-----------+
    | GR-ConvNet v2      |     | Suction Heuristic       |
    | (rigid objects)     |     | (flat objects)           |
    | 5-10ms TensorRT    |     | Center-of-detection      |
    +--------+-----------+     +-------------+-----------+
              |                              |
              +--------------+---------------+
                             |
                    +--------v-----------+
                    | Grasp Selector     |
                    | (category-based)   |
                    +--------+-----------+
                             |
                    +--------v-----------+
                    | Arm Motion Planner |
                    | (analytical IK)    |
                    +--------+-----------+
                             |
                    +--------v-----------+
                    | Hybrid Gripper     |
                    | (Fin Ray + Suction)|
                    +--------------------+
```

### 11.2 Implementation Phases

**Phase 1 -- MVP (Months 1-3):**
- GR-ConvNet v2 for parallel-jaw planar grasps
- TensorRT INT8 deployment on Jetson Orin Nano Super
- RealSense D455 depth camera, arm-mounted
- Parallel jaw gripper only
- Open-loop grasp execution
- Target: bottles, cans, rigid litter
- Expected success rate: 85-90%

**Phase 2 -- Hybrid Gripper (Months 3-6):**
- Add suction cup to gripper (hybrid Fin Ray + suction)
- Category-based grasp strategy (YOLO class drives gripper mode)
- Suction for flat items (wrappers, cigarette butts)
- Tactile feedback for grasp verification
- Target: all litter categories
- Expected success rate: 80-85% across all categories

**Phase 3 -- Advanced Grasping (Months 6-12):**
- VGN for 6-DOF grasps on challenging objects
- Closed-loop visual servoing for fine adjustment
- Custom litter-specific grasp dataset and fine-tuning
- Temporal tracking for wind-blown items
- Target: 90%+ success rate across all categories

### 11.3 Training Data Strategy

- Start with pre-trained GR-ConvNet weights (Cornell/Jacquard datasets)
- Collect litter-specific RGB-D data in target environments
- Fine-tune on litter categories with annotated grasps
- Use analytic grasp metrics from GraspNet-1Billion for label generation
- Target: 10,000+ annotated litter grasps across 5 categories

---

## 12. Sources

### Grasp Planning Systems
- [GraspNet-1Billion Baseline (GitHub)](https://github.com/graspnet/graspnet-baseline)
- [GraspNet-1Billion Paper (CVPR 2020)](https://openaccess.thecvf.com/content_CVPR_2020/papers/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.pdf)
- [AnyGrasp SDK (GitHub)](https://github.com/graspnet/anygrasp_sdk)
- [AnyGrasp Paper (arXiv)](https://arxiv.org/abs/2212.08333)
- [AnyGrasp (IEEE T-RO 2023)](https://dl.acm.org/doi/abs/10.1109/TRO.2023.3281153)
- [Contact-GraspNet (GitHub)](https://github.com/NVlabs/contact_graspnet)
- [Contact-GraspNet Paper (ICRA 2021)](https://arxiv.org/abs/2103.14127)
- [Contact-GraspNet PyTorch Port](https://github.com/elchun/contact_graspnet_pytorch)
- [GPD -- Grasp Pose Detection (GitHub)](https://github.com/atenpas/gpd)
- [GPD Paper (IJRR 2017)](https://arxiv.org/abs/1706.09911)
- [VGN -- Volumetric Grasping Network (GitHub)](https://github.com/ethz-asl/vgn)
- [VGN Paper (CoRL 2020)](https://arxiv.org/abs/2101.01132)
- [GR-ConvNet (GitHub)](https://github.com/skumra/robotic-grasping)
- [GR-ConvNet Paper (IROS 2020)](https://arxiv.org/abs/1909.04810)
- [GR-ConvNet v2 Paper (Sensors 2022)](https://www.mdpi.com/1424-8220/22/16/6208)
- [Dex-Net (Berkeley AUTOLAB)](http://berkeleyautomation.github.io/dex-net/)
- [GQCNN Package (GitHub)](https://github.com/BerkeleyAutomation/gqcnn)
- [Dex-Net 4.0 (Science Robotics)](https://www.science.org/doi/10.1126/scirobotics.aau4984)
- [FC-GQ-CNN Paper (RA-L 2019)](https://goldberg.berkeley.edu/pubs/Fully-Convolutional-Dex-Net-RA-Letters-Journal-Feb-2019.pdf)
- [Graspness Discovery (ICCV 2021)](https://arxiv.org/abs/2406.11142)
- [SuctionNet-1Billion (RA-L)](https://arxiv.org/abs/2103.12311)

### Litter-Specific Research
- [LitterBot: Autonomous Detection and Sorting (Frontiers 2022)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1064853/full)
- [Vision and Tactile Robotic System for Outdoor Litter (JIRS 2023)](https://link.springer.com/article/10.1007/s10846-023-01930-2)
- [Deep Learning Robot for Grass Pickup](https://arxiv.org/pdf/1904.13034)
- [3D-Printed Gripper for Waste Collection (Robotics MDPI 2025)](https://www.mdpi.com/2218-6581/14/7/87)

### Gripper Design
- [Evaluation of Robotic Grippers for Multi-Object Grasping (Frontiers 2024)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1351932/full)
- [Soft Robotic Grippers Review (Frontiers 2025)](https://www.frontiersin.org/journals/materials/articles/10.3389/fmats.2025.1692206/full)
- [Hybrid Vacuum-Actuated Soft Gripper (RoboSoft 2024)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1356692/full)
- [Compact Gripper with Retractable Suction (Frontiers 2023)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2023.1066516/full)

### Deformable Object Manipulation
- [Thin Deformable Object Manipulation with Soft Hand (2024)](https://arxiv.org/html/2411.13952v1)
- [RoTipBot: Thin and Flexible Objects (2024)](https://arxiv.org/html/2406.09332)
- [ShakingBot: Dynamic Bag Manipulation](https://arxiv.org/html/2304.04558v2)
- [Deformable Object Manipulation Workshop (ICRA 2024)](https://deformable-workshop.github.io/icra2024/)

### Grasp Execution and Visual Servoing
- [Closing the Loop for Robotic Grasping (RSS 2018)](https://www.roboticsproceedings.org/rss14/p21.pdf)
- [Visual Servo Control and Grasp Detection](https://www.sciencedirect.com/science/article/abs/pii/S0921889021000427)
- [Robot Closed-Loop Grasping with Visual Servoing (MDPI 2024)](https://www.mdpi.com/2076-0825/14/1/25)

### Edge Deployment
- [Jetson Orin Nano Super Developer Kit (NVIDIA)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
- [Edge AI on NVIDIA Jetson (NVIDIA Blog)](https://developer.nvidia.com/blog/getting-started-with-edge-ai-on-nvidia-jetson-llms-vlms-and-foundation-models-for-robotics/)
- [MoveIt Deep Grasps Integration](https://moveit.picknik.ai/humble/doc/examples/moveit_deep_grasps/moveit_deep_grasps_tutorial.html)
- [Dex-Net 5.0 PyTorch Re-implementation](https://github.com/Apgoldberg1/Dex-Net-5.0)
