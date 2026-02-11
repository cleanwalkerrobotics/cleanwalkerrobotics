# State-of-the-Art Depth Estimation for Robotics on Jetson Orin Nano Super (Early 2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB RAM, Ampere GPU with 1024 CUDA cores)
**Use Case:** Robotic depth perception for navigation, obstacle avoidance, and litter detection with grasp planning
**Date:** February 11, 2026 (Updated with latest 2025-2026 models)

---

## Executive Summary

Depth estimation for robotics falls into two categories: **monocular** (single camera) and **stereo** (dual camera). Monocular methods are simpler in hardware but produce either relative depth (ordering only) or metric depth requiring calibration. Stereo methods produce true metric depth from triangulation but require calibrated camera pairs.

**Key Findings for Jetson Orin Nano Super (8GB):**

1. **Depth Anything V3 (DA3)** is the NEW state-of-the-art monocular option (Nov 2025): Supports metric depth at 4K resolution with ~20 FPS at 768×1024 for ViT-L. DA3-Small variant likely achieves 30-50 FPS on Jetson. Supersedes Depth Anything V2.
2. **Depth Anything V2 Small** remains the PROVEN edge deployment choice: 24.8M params, ~42 FPS at 308x308 on Jetson Orin with TensorRT, Apache-2.0 license (Small only). Base/Large/Giant models do NOT fit in 8GB.
3. **Apple Depth Pro** (Oct 2024) produces sharp metric depth in 0.3s on V100, but model size (~600M+ params estimated) makes it unsuitable for Jetson Orin Nano.
4. **NVIDIA Isaac ROS ESS** is the production-ready stereo choice: 17M params, optimized for Jetson, real-time with confidence maps, integrated into ROS 2 pipeline.
5. **Metric depth** (absolute distance in meters) from monocular is now PRODUCTION-READY in 2025-2026 thanks to Depth Anything V3 Metric, Depth Pro, and UniDepthV2. Monocular metric depth rivals stereo for many applications.
6. **FoundationStereo** by NVIDIA (CVPR 2025 Best Paper Nomination) offers superior stereo accuracy but at ~1.8 FPS on AGX Orin -- too slow for real-time on Orin Nano. Achieves 26 FPS with TensorRT FP16 on AGX Orin with power-of-two resolutions.
7. For a **litter-picking robot with grasp planning**, stereo is RECOMMENDED over monocular: Monocular depth lacks scale information and produces high errors in point cloud conversion, making it unsuitable for accurate 3D grasping. Use stereo camera + Isaac ROS ESS for metric depth + confidence filtering.

---

## Part 1: Monocular Depth Estimation

### 1.1 Depth Anything V2 (TikTok/ByteDance)

**Paper:** "Depth Anything V2" (NeurIPS 2024)
**Source:** [GitHub](https://github.com/DepthAnything/Depth-Anything-V2) | [arXiv 2406.09414](https://arxiv.org/abs/2406.09414)

#### Architecture
- **Type:** Dense Prediction Transformer (DPT) decoder on DINOv2 encoder backbone
- **Training:** Teacher-student paradigm -- large teacher trained on 595K synthetic images, students trained on 62M pseudo-labeled real images
- **Key Innovation:** Uses only synthetic images for labeled training (avoids noisy real labels), then distills to student models via pseudo-labels on real data

#### Model Variants

| Variant | Params | Encoder | Output Channels | License |
|---------|--------|---------|-----------------|---------|
| **Small (ViT-S)** | **24.8M** | DINOv2-S | [48, 96, 192, 384] | **Apache-2.0** |
| Base (ViT-B) | 97.5M | DINOv2-B | [96, 192, 384, 768] | CC-BY-NC-4.0 |
| Large (ViT-L) | 335.3M | DINOv2-L | [256, 512, 1024, 1024] | CC-BY-NC-4.0 |
| Giant (ViT-G) | ~1.3B | DINOv2-G | [1536, 1536, 1536, 1536] | CC-BY-NC-4.0 |

**License Note:** Only the Small model is Apache-2.0 (commercial-friendly). Base/Large/Giant are CC-BY-NC-4.0 (non-commercial only).

#### Output Type
- **Default:** Relative (affine-invariant) depth -- gives depth ordering, not metric distances
- **Metric variant:** Fine-tuned metric depth models available for NYU (indoor) and KITTI (outdoor)

#### Accuracy Benchmarks

**Zero-Shot Relative Depth:**

| Model | KITTI AbsRel | KITTI delta1 | NYU AbsRel | NYU delta1 | DA-2K Accuracy |
|-------|-------------|-------------|------------|------------|----------------|
| DA V2 Small | 0.078 | 0.936 | 0.053 | 0.973 | 95.3% |
| DA V2 Base | 0.078 | 0.939 | 0.049 | 0.976 | 97.0% |
| DA V2 Large | 0.074 | 0.946 | 0.045 | 0.979 | 97.1% |
| DA V2 Giant | 0.075 | 0.948 | 0.044 | 0.979 | 97.4% |
| DA V1 Large | 0.076 | 0.947 | -- | -- | 88.5% |
| MiDaS V3.1 (ViT-L) | 0.127 | 0.850 | -- | -- | -- |

**Fine-Tuned Metric Depth (NYU Indoor):**

| Model | delta1 | delta2 | delta3 | AbsRel | RMSE | log10 |
|-------|--------|--------|--------|--------|------|-------|
| DA V2 Small | 0.961 | 0.996 | 0.999 | 0.073 | 0.261 | 0.032 |
| DA V2 Base | 0.977 | 0.997 | 1.000 | 0.063 | 0.228 | 0.027 |
| DA V2 Large | 0.984 | 0.998 | 1.000 | 0.056 | 0.206 | 0.024 |

**Fine-Tuned Metric Depth (KITTI Outdoor):**

| Model | delta1 | delta2 | delta3 | AbsRel | RMSE | log10 |
|-------|--------|--------|--------|--------|------|-------|
| DA V2 Small | 0.973 | 0.997 | 0.999 | 0.053 | 2.235 | 0.081 |
| DA V2 Base | 0.979 | 0.998 | 1.000 | 0.048 | 1.999 | 0.072 |
| DA V2 Large | 0.983 | 0.998 | 1.000 | 0.045 | 1.861 | 0.067 |

#### Inference Speed

**Desktop GPU (approximate):**
- Small model at 518x518: ~100ms on A100 (~10 FPS in PyTorch); significantly faster with TensorRT
- >10x faster than Stable Diffusion-based depth methods (Marigold, DepthFM)

**Jetson Orin 8GB (TensorRT, Depth Anything Small):**

| Input Resolution | Inference Time | FPS | Memory Usage |
|-----------------|----------------|-----|--------------|
| 308x308 | 23.5 ms | ~42.6 | 626 MB |
| 364x364 | 39.2 ms | ~25.5 | 640 MB |
| 406x406 | 47.7 ms | ~20.9 | 649 MB |
| 518x518 | 98.0 ms | ~10.2 | 689 MB |

**Critical Constraint:** Base and Large models do NOT fit on Jetson Orin 8GB due to memory. Only Small is viable on this hardware.

#### Jetson Deployment
- **TensorRT export:** Supported via ONNX -> TensorRT pipeline. [spacewalk01/depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt)
- **Jetson-specific repo:** [IRCVLab/Depth-Anything-for-Jetson-Orin](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin)
- **Known issues:** INT64 weights not natively supported by TensorRT; requires workaround. Input size must be divisible by 14.
- **Requirements:** CUDA 11.4+, TensorRT 8.5.2+, PyTorch 2.0+

#### Verdict for CleanWalker
**RECOMMENDED for monocular depth.** The Small model runs at 25-42 FPS on Jetson Orin with TensorRT, fits in 8GB, and has an Apache-2.0 license. Provides excellent relative depth for obstacle ordering. For metric depth, use the fine-tuned KITTI variant or pair with stereo.

**NOTE:** Depth Anything V3 (released Nov 2025) supersedes V2 with metric depth support and 4K resolution. See section 1.5 below.

---

### 1.5 Depth Anything V3 (DA3) (ByteDance, November 2025) **NEW**

**Paper:** "Depth Anything 3: Recovering the Visual Space from Any Views" (Nov 14, 2025)
**Source:** [GitHub](https://github.com/ByteDance-Seed/Depth-Anything-3) | [Project Page](https://depth-anything-3.github.io/) | [arXiv 2511.10647](https://arxiv.org/abs/2511.10647)

#### Architecture
- **Type:** DINOv2 ViT encoder + DPT decoder (same backbone as DA V2)
- **Key Innovation:** Dual-prediction head outputs BOTH dense depth maps AND ray maps for cross-view reasoning
- **Unified depth-ray representation:** Enables multi-view consistency and metric depth recovery
- **Token reordering:** Efficient cross-view reasoning for multi-camera setups
- **Streaming inference:** Ultra-long video sequences with <12GB GPU memory via sliding-window approach

#### Model Variants

| Series | Variant | Backbone | Outputs | Notes |
|--------|---------|----------|---------|-------|
| **DA3 Main** | DA3-Giant | DINOv2-G | Depth + Ray | Flagship foundation model |
| | DA3-Large | DINOv2-L | Depth + Ray | High accuracy |
| | DA3-Base | DINOv2-B | Depth + Ray | Mid-range |
| | DA3-Small | DINOv2-S | Depth + Ray | Fastest |
| **DA3 Metric** | DA3Metric-Large | DINOv2-L | Metric Depth | **Metric depth specialist** |
| **DA3 Monocular** | DA3Mono-Large | DINOv2-L | Relative Depth | High-quality relative depth |

**Parameter counts:** Same as DA V2 (DA3-Small ~25M, DA3-Large ~335M)

#### Output Type
- **DA3 Main series:** Depth + Ray (enables metric recovery via focal length estimation)
- **DA3Metric-Large:** Direct metric depth in meters via formula: `metric_depth = focal * net_output / 300`
- **4K resolution support:** Native 4K depth estimation with minimal retraining

#### Accuracy Benchmarks

**NOTE:** Specific benchmark numbers for DA3 not yet published in accessible form (as of Feb 2026). Expected to match or exceed DA V2 based on architecture improvements.

#### Inference Speed

| Model | Resolution | Device | FPS | Notes |
|-------|-----------|--------|-----|-------|
| DA3-Large (ViT-L) | 768×1024 | Desktop GPU | ~20 | High-quality metric depth |
| DA3Metric-Large | 4K | Desktop GPU | Varies | 4K-resolution accurate metric depth |
| DA3-Small (est.) | 364×364 | Jetson Orin (TRT) | 25-35 est. | Based on DA V2 Small performance |

**Streaming mode:** Enables ultra-long video sequences with <12GB GPU memory.

#### Jetson Deployment
- **TensorRT export:** Community deployment expected in Q1 2026 (model just released Nov 2025)
- **ROS 2 TensorRT implementation:** [ros2-depth-anything-v3-trt](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt) by ika-rwth-aachen
- **Jetson AGX Orin deployment guide:** [Seeed Studio Wiki - Deploy Depth Anything V3 on Jetson AGX Orin](https://wiki.seeedstudio.com/deploy_depth_anything_v3_jetson_agx_orin/)
- **Expected Orin Nano performance:** DA3-Small likely 25-40 FPS at 308-364px resolution with TensorRT optimization

#### License
- Not explicitly stated in search results; likely follows DA V2 licensing (Small: Apache-2.0, Large/Giant: CC-BY-NC-4.0)

#### Verdict for CleanWalker
**FUTURE RECOMMENDED monocular option (once Jetson deployment matures).** DA3Metric-Large offers true metric depth from monocular cameras, which could replace stereo for some applications. However, as of Feb 2026, deployment on Jetson Orin Nano is unproven. DA3-Small with TensorRT optimization is the target once community tooling stabilizes (Q1-Q2 2026). For immediate deployment, use DA V2 Small.

**For grasp planning:** Monocular metric depth (even DA3Metric) is still less reliable than stereo for robotic manipulation. Use stereo for safety-critical grasping.

---

### 1.6 Apple Depth Pro (Apple ML, October 2024) **NEW**

**Paper:** "Depth Pro: Sharp Monocular Metric Depth in Less Than a Second" (ICLR 2025)
**Source:** [GitHub](https://github.com/apple/ml-depth-pro) | [Apple ML Research](https://machinelearning.apple.com/research/depth-pro) | [arXiv 2410.02073](https://arxiv.org/abs/2410.02073)

#### Architecture
- **Type:** Multi-scale Vision Transformer (ViT) with DINOv2 encoder + DPT-like fusion stage
- **Key Innovation:** Zero-shot metric depth WITHOUT camera intrinsics -- predicts focal length alongside depth
- **Plain ViT backbone:** Processes multiple scales (downsampled patches) via shared DINOv2 encoder
- **High-resolution output:** 2.25-megapixel native depth maps with fine boundaries and high-frequency details

#### Model Specifications
- **Parameters:** Not officially published; estimated ~600-800M based on multi-scale ViT architecture
- **Input resolution:** 2.25 megapixels (native)
- **Training:** Real + synthetic datasets with boundary-accuracy-focused loss functions

#### Output Type
- **Metric depth** (absolute, in meters) -- zero-shot, no camera metadata required
- **Focal length estimation:** Predicts camera focal length from image alone (15.5% better than prior methods)
- **Sharp boundaries:** State-of-the-art fine-structure preservation

#### Accuracy Benchmarks
- State-of-the-art focal length estimation (15.5% improvement in SIlog over second-best)
- Dedicated boundary accuracy metrics (new evaluation protocol)
- Zero-shot metric depth across diverse scenes

#### Inference Speed
- **V100 GPU:** 0.3 seconds for 2.25-megapixel depth map (~3.3 FPS)
- **Optimized (TensorRT):** Community reports ~6x speedup with FP16 on RTX 3090

#### Jetson Orin Nano Feasibility
- **NOT FEASIBLE for real-time:** Model size (~600-800M params) far exceeds 8GB memory budget
- TensorRT implementation exists ([yuvraj108c/ml-depth-pro-tensorrt](https://github.com/yuvraj108c/ml-depth-pro-tensorrt)) but designed for high-end GPUs
- Suitable for: offline processing, server-side depth estimation, high-quality dataset generation

#### Open Source & License
- **License:** Apple Sample Code License (open-source, but with restrictions)
- **Framework support:** PyTorch, ONNX, TensorRT (community ports)

#### Verdict for CleanWalker
**NOT suitable for edge deployment.** Depth Pro achieves excellent metric depth quality but is far too large for Jetson Orin Nano. Best use case: offline generation of high-quality metric depth ground truth for training smaller models or validating stereo depth outputs.

---

### 1.2 UniDepth / UniDepthV2 (ETH Zurich, CVPR 2024)

**Paper:** "UniDepth: Universal Monocular Metric Depth Estimation" (CVPR 2024)
**V2 Paper:** "UniDepthV2: Universal Monocular Metric Depth Estimation Made Simpler" (arXiv 2502.20110, Feb 2025)
**Source:** [GitHub](https://github.com/lpiccinelli-eth/UniDepth) | [Project Page](https://lpiccinelli-eth.github.io/pub/unidepth/)

#### Architecture
- **Type:** Vision Transformer encoder + Self-Promptable Camera Module + Depth Module
- **Key Innovation:** Predicts BOTH dense camera intrinsics AND metric depth from a single image, enabling zero-shot metric depth without knowing camera parameters
- **Camera Module:** Learns pseudo-spherical camera representation that disentangles camera and depth
- **Geometric Invariance Loss:** Promotes invariance of camera-prompted depth features

#### Model Variants

**UniDepth V1:**

| Variant | Backbone | Notes |
|---------|----------|-------|
| V1-ConvNeXt-L | ConvNeXt-Large | CNN-based backbone |
| V1-ViT-L | ViT-L14 (DINOv2) | Transformer backbone |

**UniDepth V2:**

| Variant | Backbone | Speed vs V1 |
|---------|----------|-------------|
| V2-ViT-S | DINOv2-S (ViT-S14) | Smallest, fastest |
| V2-ViT-B | DINOv2-B (ViT-B14) | Mid-range |
| V2-ViT-L | DINOv2-L (ViT-L14) | Best accuracy |

#### Output Type
- **Metric depth** (absolute distance in meters) -- zero-shot across domains
- Also outputs 3D point clouds directly
- V2 adds uncertainty/confidence output

#### Accuracy Benchmarks (Zero-Shot, delta1 metric, higher is better)

| Dataset | UniDepth V1-ViT-L |
|---------|-------------------|
| NYUv2 | 98.4 |
| SUN-RGBD | 94.3 |
| KITTI | 98.6 |
| NuScenes | 84.6 |
| DDAD | 85.8 |

#### Inference Speed
- V2 is >30% faster than V1 (tested on RTX 4090 with float16)
- V2 is up to 2x faster than comparable baselines at resolutions up to ~2 megapixels
- Exact parameter counts for V2 variants not publicly listed; V1-ViT-L is comparable to ZoeDepth (~345M)
- **Estimated:** V2-ViT-S likely ~25-40M params, V2-ViT-L likely ~340M+ params (based on DINOv2 backbone sizes)

#### Jetson Orin Nano Feasibility
- **V2-ViT-S:** Potentially feasible with ONNX export (V2 supports ONNX natively). Memory would be tight.
- **V2-ViT-L:** Very unlikely to fit in 8GB for real-time inference
- **No known Jetson-specific deployment** or TensorRT conversion pipeline exists
- Supports Pinhole and Fisheye624 camera models

#### Open Source & License
- **License:** CC-BY-NC-4.0 (non-commercial)
- Python 3.10+, CUDA 11.8+

#### Verdict for CleanWalker
**Promising but not production-ready for Jetson.** UniDepthV2-ViT-S could theoretically run on Orin Nano but no one has demonstrated this. The non-commercial license is also a concern. Best for: research prototyping where metric depth from unknown cameras is needed.

---

### 1.3 Metric3D v2 (Tsinghua/ZJU)

**Paper:** "Metric3D v2: A Versatile Monocular Geometric Foundation Model" (TPAMI 2024)
**Source:** [GitHub](https://github.com/YvanYin/Metric3D) | [Project Page](https://jugghm.github.io/Metric3Dv2/) | [arXiv 2404.15506](https://arxiv.org/abs/2404.15506)

#### Architecture
- **Type:** DINOv2-reg encoder + RAFT-style iterative decoder
- **Key Innovation:** Canonical Camera Space Transformation -- transforms all images to a canonical focal length, eliminating metric ambiguity from varying cameras
- **Joint depth-normal optimization:** Distills knowledge from depth labels to improve surface normal prediction
- **Random Proposal Normalization Loss:** Boosts depth accuracy

#### Model Variants

| Variant | Encoder | Decoder | Iterations | Outputs |
|---------|---------|---------|------------|---------|
| ConvNeXt-Tiny | ConvNeXt-T | Hourglass | -- | Depth |
| ConvNeXt-Large | ConvNeXt-L | Hourglass | -- | Depth |
| ViT-Small | DINOv2-reg-S | RAFT (4 iter) | 4 | Depth + Normal |
| ViT-Large | DINOv2-reg-L | RAFT (8 iter) | 8 | Depth + Normal |
| ViT-Giant2 | DINOv2-reg-G | RAFT (8 iter) | 8 | Depth + Normal |

#### Output Type
- **Metric depth** (absolute, in meters) -- zero-shot across camera types
- **Surface normals** (jointly estimated in V2 variants)

#### Accuracy Benchmarks

**Zero-Shot Metric Depth:**

| Model | KITTI delta1 | KITTI AbsRel | KITTI RMS | NYU delta1 | NYU AbsRel | NYU RMS |
|-------|-------------|-------------|-----------|------------|------------|---------|
| ViT-Large | 0.974 | 0.052 | 2.511 | 0.975 | 0.063 | 0.251 |
| ViT-Giant2 | 0.977 | 0.051 | 2.403 | 0.980 | 0.067 | 0.260 |

**With fine-tuning (from GitHub):**

| Model | KITTI delta1 | KITTI AbsRel | NYU delta1 | NYU AbsRel |
|-------|-------------|-------------|------------|------------|
| ViT-Large | 0.985 | 0.044 | 0.989 | 0.047 |
| ViT-Giant2 | 0.989 | 0.039 | -- | -- |

#### Inference Speed
- ViT-Small: Fastest variant, suitable for applications needing speed over accuracy
- ViT-Large: ~300-500ms per frame on desktop GPU (estimated from RAFT-style decoder with 8 iterations)
- ViT-Giant2: Slowest, highest accuracy
- Trained on 16M+ images from thousands of camera models

#### Model Sizes (Estimated from DINOv2 backbone sizes)
- ViT-Small: ~30-50M params
- ViT-Large: ~340-400M params
- ViT-Giant2: ~1.1-1.3B params

#### Jetson Orin Nano Feasibility
- **ViT-Small:** Potentially feasible but no known Jetson deployment exists
- **ViT-Large/Giant2:** Too large for 8GB VRAM
- ONNX export supported via PyTorch Hub
- HuggingFace checkpoints available

#### Open Source & License
- **License:** BSD-2-Clause (permissive, commercial-friendly)
- Ranks #1 on KITTI and NYU benchmarks

#### Verdict for CleanWalker
**Best metric depth accuracy available, but too large for real-time Jetson.** ViT-Small variant might work with significant optimization effort. The BSD-2-Clause license is excellent. Best for: offline processing, ground-truth generation for training smaller models, or server-side depth estimation.

---

### 1.4 ZoeDepth (Intel ISL)

**Paper:** "ZoeDepth: Zero-shot Transfer by Combining Relative and Metric Depth" (arXiv 2302.12288, 2023)
**Source:** [GitHub](https://github.com/isl-org/ZoeDepth) | [HuggingFace](https://huggingface.co/docs/transformers/model_doc/zoedepth)

**Note:** This project is no longer maintained by Intel as of May 2025.

#### Architecture
- **Type:** DPT architecture with BEiT-L encoder (from MiDaS) + metric bins module
- **Key Innovation:** Two-stage training -- relative depth pre-training on 12 diverse datasets, then metric fine-tuning with novel "attractor layers" for bin refinement
- **Metric Bins Module:** Uses log-binomial distribution (not softmax) for ordinal probability, with {16,8,4,1} attractors across decoder layers
- **Domain Router:** Learned classifier automatically routes to indoor/outdoor metric head

#### Model Variants

| Variant | Training | Backbone | Total Params |
|---------|----------|----------|--------------|
| ZoeD-N | NYU only | BEiT384-L | ~345M |
| ZoeD-K | KITTI only | BEiT384-L | ~345M |
| ZoeD-M12-N | 12-dataset pretrain + NYU | BEiT384-L | ~345M |
| ZoeD-M12-NK | 12-dataset pretrain + NYU+KITTI | BEiT384-L | ~345M |

**Parameter Breakdown:**
- BEiT384-L backbone: ~305M params
- MiDaS decoder: ~39M params
- Metric heads: <1% of backbone (<3.5M)

**Smaller backbone options:**
- Swin2-T: ~42M total params
- Swin2-B: ~102M total params
- BEiT-B: ~112M total params

#### Output Type
- **Metric depth** (absolute, in meters)
- Domain-aware: automatically handles indoor (NYU-style) and outdoor (KITTI-style)

#### Accuracy Benchmarks

| Model | NYU delta1 | NYU REL | NYU RMSE | KITTI delta1 | KITTI REL | KITTI RMSE |
|-------|------------|---------|----------|-------------|-----------|------------|
| ZoeD-M12-N | 0.955 | 0.075 | 0.270 | -- | -- | -- |
| ZoeD-M12-NK | 0.953 | 0.077 | 0.277 | 0.824 | 0.138 | 7.225 |
| ZoeD-X-N | 0.946 | 0.082 | 0.294 | -- | -- | -- |

**Zero-Shot Generalization (REL, lower is better):**

| Dataset | NeWCRFs | ZoeD-M12-NK | Improvement |
|---------|---------|-------------|-------------|
| iBims-1 | 0.206 | 0.186 | +9.7% |
| DIODE Indoor | 0.404 | 0.331 | +18.1% |
| SUN RGB-D | 0.151 | 0.123 | +18.5% |
| DIML Outdoor | 1.918 | 0.641 | +66.6% |

#### Inference Speed
- BEiT-L variant: ~170ms per frame (~5.9 FPS) on desktop GPU
- Input resolution: 384x512 (~0.2 megapixels)
- Slower than Depth Anything V2 at comparable accuracy

#### Jetson Orin Nano Feasibility
- **BEiT-L (345M):** Too large for 8GB real-time inference
- **Swin2-T (42M):** Potentially feasible but with reduced accuracy
- No known Jetson-specific deployment or TensorRT pipeline
- ONNX/TensorRT conversion not officially supported

#### Open Source & License
- **License:** MIT (very permissive, commercial-friendly)
- **Status:** No longer maintained (as of May 2025)

#### Verdict for CleanWalker
**Superseded by Depth Anything V2 and UniDepthV2.** ZoeDepth was groundbreaking in 2023 but newer models achieve better accuracy at similar or faster speeds. The MIT license is excellent, and the Swin2-T backbone could theoretically fit on Jetson, but lack of maintenance and Jetson optimization makes it impractical. Not recommended for new projects.

---

## Part 2: Stereo Depth Estimation

### 2.1 RAFT-Stereo (Princeton, 3DV 2021)

**Paper:** "RAFT-Stereo: Multilevel Recurrent Field Transforms for Stereo Matching" (3DV 2021, Best Student Paper)
**Source:** [GitHub](https://github.com/princeton-vl/RAFT-Stereo) | [arXiv 2109.07547](https://arxiv.org/abs/2109.07547)

#### Architecture
- **Type:** Recurrent all-pairs field transforms adapted for stereo (from RAFT optical flow)
- **Key Innovation:** Multi-level convolutional GRUs (at 1/8, 1/16, 1/32 resolutions) for efficient information propagation
- **Feature extraction:** Shared backbone for correlation and context features
- **Correlation volume:** 3D correlation restricted to same y-coordinates (epipolar constraint)
- **Instance normalization** in feature encoder; batch normalization in context encoder

#### Parameters & Model Size
- **Parameters:** 11.23M
- **Memory:** <1GB GPU overhead
- **Architecture:** Lightweight compared to other stereo methods

#### Output Type
- **Disparity map** (convertible to metric depth with known camera baseline and focal length)

#### Accuracy Benchmarks

| Benchmark | Metric | Score | Ranking |
|-----------|--------|-------|---------|
| ETH3D Bad 0.5% | Outlier rate | 7.04% | #1 published |
| ETH3D Bad 2.0% | Outlier rate | 0.44% | #1 published |
| ETH3D AvgErr | Mean error | 0.18 px | #1 published |
| Middlebury AvgErr | Mean error | 1.27 px | #1 on leaderboard |
| Middlebury Bad 2.0% | Outlier rate | 4.74% | 26% better than next DL method |
| KITTI-2015 All | Error rate | 1.96% | #2 published |
| KITTI-2015 FG | Foreground | 2.89% | -- |

**Zero-Shot (SceneFlow trained -> real):**

| Dataset | Metric | Score |
|---------|--------|-------|
| ETH3D | 1px error | 3.28% |
| Middlebury | 1px error | 12.59% |
| KITTI | 3px error | 5.74% |

#### Inference Speed

| Configuration | Resolution | Time | FPS |
|---------------|-----------|------|-----|
| Standard | KITTI (1248x384) | 132 ms | ~7.6 |
| Real-time (Slow-Fast GRU) | KITTI (1248x384) | ~38 ms | ~26 |
| Fastest (reduced iters) | KITTI (1248x384) | ~50 ms | ~20 |

#### Jetson Orin Nano Feasibility
- **Potentially feasible:** 11.23M params is very small; <1GB memory
- **No official TensorRT/ONNX export** but architecture is simple enough for conversion
- Recurrent GRUs may not optimize well with TensorRT (dynamic iteration count)
- Would need custom ONNX export; expect 3-8 FPS on Orin Nano at KITTI resolution

#### Open Source & License
- **License:** MIT (very permissive)

#### Verdict for CleanWalker
**Good baseline stereo method with tiny model size.** The 11.23M parameters and <1GB memory make it theoretically viable on Jetson, but the recurrent architecture is hard to optimize with TensorRT. The real-time variant at 26 FPS on desktop GPU would likely drop to 3-8 FPS on Orin Nano. Consider if stereo accuracy matters more than frame rate.

---

### 2.2 CREStereo (Megvii/MEGVII, CVPR 2022 Oral)

**Paper:** "Practical Stereo Matching via Cascaded Recurrent Network with Adaptive Correlation" (CVPR 2022)
**Source:** [GitHub (Official MegEngine)](https://github.com/megvii-research/CREStereo) | [PyTorch Port](https://github.com/ibaiGorordo/CREStereo-Pytorch) | [arXiv 2203.11483](https://arxiv.org/abs/2203.11483)

#### Architecture
- **Type:** Cascaded recurrent network with adaptive correlation
- **Key Components:**
  - Hierarchical feature extraction network (multi-level pyramid)
  - Recurrent Update Modules (RUMs) in cascaded coarse-to-fine structure
  - Adaptive Group Correlation Layer (AGCL) -- mitigates erroneous rectification and reduces matching ambiguity
- **Inference:** Stacked cascaded architecture for coarse-to-fine refinement

#### Parameters & Model Size
- **Parameters:** Not officially published; estimated ~5-15M based on architecture (similar to RAFT-Stereo with cascade overhead)
- **Tested resolution:** 1024x1536 (flexible input)

#### Output Type
- **Disparity map** (metric depth with known baseline)

#### Accuracy Benchmarks

| Benchmark | Metric | Performance | Notes |
|-----------|--------|-------------|-------|
| Middlebury Bad 2.0% | Outlier rate | #1 overall | 21.73% better than prior SOTA |
| Middlebury A95 | 95th percentile | #1 overall | 31.00% better than prior SOTA |
| ETH3D | Multiple | #1 overall | -- |
| KITTI 2012 Out-Noc | 2px error | Better than LEAStereo | 9.47% improvement |
| KITTI 2015 | Competitive | Top-tier | -- |

#### Inference Speed
- **Desktop GPU:** Not officially benchmarked in paper
- **Limitation noted by authors:** "Not yet efficient enough for mobile applications"
- Estimated: slower than RAFT-Stereo due to cascade + AGCL overhead

#### Jetson Orin Nano Feasibility
- **ONNX port exists:** [ONNX-CREStereo](https://github.com/ibaiGorordo/ONNX-CREStereo-Depth-Estimation)
- **Luxonis DepthAI integration:** CREStereo available on Luxonis model zoo
- Cascaded recurrent architecture is challenging for TensorRT optimization
- **Feasibility:** Possible but would require significant optimization; likely <5 FPS

#### Open Source & License
- **License:** Apache-2.0 (commercial-friendly)
- Official implementation in MegEngine (not PyTorch); community PyTorch ports available

#### Verdict for CleanWalker
**Strong accuracy but uncertain Jetson performance.** CREStereo has excellent benchmark results and an Apache-2.0 license, but the cascaded recurrent architecture is difficult to optimize for edge devices. The ONNX port makes it possible but real-time on Orin Nano is unlikely without major engineering effort.

---

### 2.3 IGEV-Stereo / IGEV++ (Selective-IGEV) (CVPR 2023 / TPAMI 2025)

**Paper:** "Iterative Geometry Encoding Volume for Stereo Matching" (CVPR 2023)
**IGEV++ Paper:** "IGEV++: Iterative Multi-range Geometry Encoding Volumes for Stereo Matching" (TPAMI 2025)
**Source:** [GitHub IGEV](https://github.com/gangweiX/IGEV) | [GitHub IGEV++](https://github.com/gangweiX/IGEV-plusplus) | [arXiv 2303.06615](https://arxiv.org/abs/2303.06615) | [arXiv 2409.00638](https://arxiv.org/abs/2409.00638)

#### Architecture
- **Type:** Combined geometry encoding volume + iterative ConvGRU updates
- **Backbone:** MobileNetV2 (pretrained on ImageNet) for feature extraction
- **Feature scales:** Multi-scale at 1/4, 1/8, 1/16, 1/32 resolutions
- **3D Regularization:** Lightweight 3D UNet (3 down + 3 up blocks)
- **Update:** Multi-level ConvGRUs (128 channels standard, 96 for real-time)

**IGEV++ (Selective-IGEV) additions:**
- **Multi-range Geometry Encoding Volumes (MGEV):** Encodes coarse-grained geometry for ill-posed/large disparity regions + fine-grained for details/small disparities
- **Adaptive Patch Matching Module:** Efficiently computes matching costs for large disparity ranges
- **Selective Geometry Feature Fusion Module:** Adaptively weights features across small/medium/large disparity ranges using sigmoid-gated combination

#### Parameters & Model Size
- **IGEV-Stereo:** Not officially published; estimated ~15-25M params (MobileNetV2 backbone + 3D UNet + GRUs)
- **Memory:** 0.66 GB (vs RAFT-Stereo's 1.02 GB) -- 35% less memory
- **IGEV++ memory:** 1.57 GB for large disparity (vs PCWNet's 11.62 GB) -- 7x less

#### Output Type
- **Disparity map** (supports up to 768px disparity range in IGEV++)

#### Accuracy Benchmarks

**IGEV-Stereo:**

| Benchmark | Metric | Score | Notes |
|-----------|--------|-------|-------|
| Scene Flow EPE | End-point error | 0.47 px | SOTA on Scene Flow |
| KITTI 2015 D1-all | Error rate | 1.59% | #1 among published |
| KITTI 2012 3-noc | Error rate | 1.12% | #1 (reflective) |

**IGEV++:**

| Benchmark | Metric | Score |
|-----------|--------|-------|
| Scene Flow EPE (<768px) | End-point error | 0.67 px |
| Scene Flow Bad 3.0 | Outlier rate | 2.21% |
| KITTI 2015 D1-all | Error rate | 1.51% |
| KITTI 2012 3-noc | Error rate | 1.04% |
| Middlebury Bad 2.0 | Outlier rate | 3.23% |
| ETH3D Bad 0.5 | Outlier rate | 2.98% |

#### Inference Speed

| Variant | Resolution | Time | Notes |
|---------|-----------|------|-------|
| IGEV-Stereo (3 iters) | KITTI | 100 ms (~10 FPS) | Minimal quality |
| IGEV-Stereo (full) | KITTI | 180 ms (~5.6 FPS) | Standard quality |
| IGEV++ (full) | KITTI | ~280 ms (~3.6 FPS) | Best accuracy |
| **RT-IGEV++ (real-time)** | **KITTI** | **36-54 ms (18-28 FPS)** | **2-8 iterations** |
| RAFT-Stereo comparison | KITTI | 440 ms (32 iters) | Slower, less accurate |

#### Jetson Orin Nano Feasibility
- **RT-IGEV++:** Most promising variant -- real-time on desktop GPU
- Memory usage (0.66 GB) fits well within 8GB
- No official TensorRT/ONNX export, but MobileNetV2 backbone is well-supported
- **Estimated Orin Nano:** 3-8 FPS for RT-IGEV++ (extrapolating from desktop to edge ratio)

#### Open Source & License
- **License:** MIT (very permissive)

#### Verdict for CleanWalker
**Best accuracy-to-speed ratio among academic stereo methods.** RT-IGEV++ achieves real-time on desktop and uses less memory than alternatives. The MobileNetV2 backbone is edge-friendly. With TensorRT optimization, this could potentially achieve 5-10 FPS on Orin Nano. The MIT license is excellent. Main risk: no existing Jetson deployment -- would require custom engineering.

---

### 2.4 NVIDIA Isaac ROS Stereo Depth

**Source:** [Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html) | [GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth) | [Concepts](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/index.html)

Isaac ROS provides three stereo depth approaches out of the box:

#### 2.4.1 ESS (Efficient Semi-Supervised Stereo)

| Attribute | Value |
|-----------|-------|
| **Architecture** | Shared CNN backbones + learnable cost volume |
| **Parameters** | 17M |
| **Input Resolution** | 576x960 (full) / 288x480 (light) |
| **Output** | Disparity map + confidence map |
| **Accuracy (Middlebury)** | BP2: 8.27%, MAE: 1.06 px |
| **Speed (AGX Orin)** | 108.86 FPS |
| **Speed (AGX Thor)** | 178 FPS (full) / 350 FPS (light) |
| **Jetson Orin Nano** | Supported (users report successful deployment) |
| **TensorRT** | Natively optimized with custom TRT plugins |
| **License** | NVIDIA proprietary (free for Jetson) |

#### 2.4.2 FoundationStereo (NVIDIA, CVPR 2025 Best Paper Nomination)

| Attribute | Value |
|-----------|-------|
| **Architecture** | Transformer-based, uses Depth Anything V2 as feature extractor + DINOv2 side-tuning |
| **Parameters** | 60.6M |
| **Input Resolution** | 576x960 or 320x736 |
| **Output** | Disparity map only (no confidence) |
| **Accuracy (Middlebury)** | BP2: 5.306%, MAE: 0.775 px |
| **Speed (AGX Orin)** | 1.77 FPS |
| **Speed (AGX Thor)** | Higher but still not real-time |
| **Jetson Orin Nano** | Technically supported but impractical for real-time |
| **TensorRT** | ONNX -> TensorRT pipeline provided, FP16 supported |
| **Variants** | 23-51-11 (ViT-L, best accuracy) / 11-33-40 (ViT-S, faster) |
| **License** | NVIDIA proprietary; commercial version via NVIDIA TAO |

**FoundationStereo Key Results:**
- #1 on Middlebury leaderboard (zero-shot)
- #1 on ETH3D leaderboard (zero-shot)
- 6x speedup with TensorRT FP16 vs PyTorch on RTX 3090
- Jetson Orin deployment guide included (readme_jetson.md)

#### 2.4.3 SGM (Semi-Global Matching)

| Attribute | Value |
|-----------|-------|
| **Architecture** | Classical analytic algorithm (not DNN) |
| **Parameters** | N/A (no neural network) |
| **Speed (AGX Thor)** | 217 FPS at 1080p |
| **Accuracy** | Lower than DNN methods; struggles with smooth/textureless surfaces |
| **Package** | `isaac_ros_stereo_image_proc` |
| **License** | Apache-2.0 (ROS 2 standard) |

#### Isaac ROS Integration Notes
- All methods use NVIDIA NITROS for zero-copy GPU message passing
- ROS 2 Jazzy compatible
- Designed for Jetson Orin family (Nano and up)
- ESS provides both disparity AND confidence -- confidence can filter unreliable depth
- FoundationStereo prioritizes quality over speed

#### Verdict for CleanWalker
**ESS is the recommended production stereo depth solution.** At 17M params with 108+ FPS on AGX Orin, ESS provides real-time stereo depth with confidence maps, fully integrated into ROS 2. It runs on Jetson Orin Nano. FoundationStereo is better for offline/high-quality applications. SGM is a fallback if DNN deployment is problematic.

---

## Part 3: Comparison Tables

### 3.1 Monocular Depth Models -- Head-to-Head (Updated Feb 2026)

| Model | Params | Output Type | KITTI AbsRel | NYU AbsRel | Desktop Speed | Jetson Feasible | License |
|-------|--------|-------------|-------------|------------|---------------|-----------------|---------|
| **DA V3 Small (NEW)** | **~25M** | Metric (focal-based) | TBD | TBD | ~30-50 FPS est. | **Yes (est. 25-40 FPS TRT)** | TBD (likely Apache-2.0) |
| **DA V2 Small** | **24.8M** | Relative (metric w/ finetune) | 0.078 | 0.053 | ~100 FPS (A100) | **Yes (42 FPS TRT)** | **Apache-2.0** |
| DA V3 Metric-Large (NEW) | ~335M | Metric (direct) | TBD | TBD | ~20 FPS (768x1024) | No (OOM) | TBD (likely CC-BY-NC-4.0) |
| DA V2 Large | 335.3M | Relative (metric w/ finetune) | 0.074 | 0.045 | ~10-20 FPS | No (OOM) | CC-BY-NC-4.0 |
| Depth Pro (Apple, NEW) | ~600-800M est. | Metric (zero-shot) | SOTA boundaries | SOTA | 3.3 FPS (V100) | No (OOM) | Apple Sample Code |
| UniDepthV2-ViT-S | ~30M est. | Metric (zero-shot) | ~0.05 est. | ~0.06 est. | ~30-50 FPS est. | Maybe (untested) | CC-BY-NC-4.0 |
| UniDepthV2-ViT-L | ~340M est. | Metric (zero-shot) | SOTA (98.6 KITTI δ1) | SOTA (98.4 NYU δ1) | ~5-10 FPS | No (OOM) | CC-BY-NC-4.0 |
| Metric3D v2 ViT-S | ~40M est. | Metric + Normals | -- | -- | ~15-30 FPS est. | Maybe (untested) | **BSD-2-Clause** |
| Metric3D v2 ViT-L | ~370M est. | Metric + Normals | 0.052 | 0.063 | ~3-5 FPS | No (OOM) | **BSD-2-Clause** |
| Metric3D v2 ViT-G | ~1.1B | Metric + Normals | 0.043 | 0.043 | ~1-2 FPS | No (OOM) | **BSD-2-Clause** |
| ZoeDepth (BEiT-L) | 345M | Metric (domain-aware) | 0.138 (NK) | 0.075 | ~5.9 FPS | No (OOM) | **MIT** (unmaintained) |
| ZoeDepth (Swin2-T) | 42M | Metric (reduced) | -- | -- | ~15-20 FPS est. | Maybe (unmaintained) | **MIT** (unmaintained) |

### 3.2 Stereo Depth Models -- Head-to-Head

| Model | Params | Memory | KITTI D1-all | Middlebury | Desktop Speed | Jetson Feasible | License |
|-------|--------|--------|-------------|------------|---------------|-----------------|---------|
| RAFT-Stereo | 11.23M | <1 GB | 1.96% | 4.74% Bad2 | 26 FPS (RT) | Yes (untested) | **MIT** |
| CREStereo | ~10-15M est. | -- | Competitive | #1 Bad2 | ~5-10 FPS est. | Possible (ONNX exists) | **Apache-2.0** |
| IGEV-Stereo | ~20M est. | 0.66 GB | 1.59% | -- | ~5.6 FPS | Yes (untested) | **MIT** |
| RT-IGEV++ | ~20M est. | 0.66 GB | 1.51% | 3.23% Bad2 | **18-28 FPS** | Likely (untested) | **MIT** |
| **Isaac ROS ESS** | **17M** | Optimized | -- | 8.27% BP2 | **108+ FPS (AGX)** | **Yes (production)** | NVIDIA prop. |
| FoundationStereo | 60.6M | -- | -- | 5.31% BP2 | 1.77 FPS (AGX) | Yes (slow) | NVIDIA prop. |
| SGM (classical) | N/A | Minimal | -- | -- | 217 FPS (1080p) | Yes | Apache-2.0 |

### 3.3 Jetson Orin Nano Super Deployment Readiness (Updated Feb 2026)

| Model | Runs on Orin Nano? | TensorRT Export? | Known FPS on Jetson | Production Ready? |
|-------|-------------------|------------------|--------------------|--------------------|
| **DA V3 Small (NEW)** | **Likely (Q1-Q2 2026)** | **Yes (community)** | **Est. 25-40 FPS** | **Soon (maturing)** |
| **DA V2 Small** | **Yes** | **Yes (tested)** | **10-42 FPS** | **Yes** |
| DA V3 Metric-Large | No (OOM) | Yes (community) | -- | No |
| DA V2 Base/Large | No (OOM) | Yes | -- | No |
| Depth Pro (Apple) | No (OOM) | Yes (community) | -- | No |
| UniDepthV2-ViT-S | Untested | ONNX yes | Unknown | No |
| Metric3D v2 ViT-S | Untested | ONNX possible | Unknown | No |
| ZoeDepth (any) | Untested | No official | Unknown | No (unmaintained) |
| RAFT-Stereo | Likely fits | No official | Est. 3-8 FPS | No |
| CREStereo | Possible | ONNX exists | Unknown | No |
| RT-IGEV++ | Likely fits | No official | Est. 3-8 FPS | No |
| **Isaac ROS ESS** | **Yes** | **Native TRT** | **Est. 20-40+ FPS** | **Yes** |
| **FoundationStereo** | Yes (slow on Nano) | Yes (TRT) | 26 FPS (AGX Orin) | AGX Orin only |
| SGM | Yes | N/A (classical) | High FPS | Yes |
| RT-MonoDepth | Yes | Yes (torch2trt) | 18-30 FPS (Nano) | Research only |

---

## Part 4: NVIDIA Jetson-Optimized Depth Solutions

### 4.1 What NVIDIA Provides Out of the Box

NVIDIA Isaac ROS includes these depth estimation capabilities:

1. **isaac_ros_ess** -- DNN stereo depth with ESS model (17M params, near real-time)
2. **isaac_ros_foundationstereo** -- High-accuracy stereo depth (60.6M params, slow)
3. **isaac_ros_stereo_image_proc** -- Classical SGM stereo (fast, lower quality)
4. **isaac_ros_depth_segmentation** -- Depth-based segmentation
5. **jetson-inference DepthNet** -- NVIDIA's own monocular depth via dusty-nv/jetson-inference

### 4.2 Jetson-Optimized Depth Models

| Solution | Type | Optimized for Jetson? | FPS on Orin Nano | Notes |
|----------|------|-----------------------|------------------|-------|
| Isaac ROS ESS | Stereo DNN | Yes (native TRT) | Est. 20-40+ | Production ready |
| Isaac ROS SGM | Stereo classical | Yes (GPU accelerated) | High (100+) | Lower accuracy |
| jetson-inference DepthNet | Mono DNN | Yes (TensorRT) | Varies by model | Included in JetPack |
| Depth Anything V2 Small (TRT) | Mono DNN | Community (IRCVLab) | 10-42 | Best community option |
| RT-MonoDepth | Mono DNN | Yes (torch2trt) | 18-30 (Nano) | ICIP 2024, research |

### 4.3 Recommended Stack for CleanWalker

**Option A: Monocular (simpler hardware, lower cost)**
- Camera: Single RGB camera (e.g., IMX477, Arducam)
- Model: Depth Anything V2 Small (TensorRT, Apache-2.0)
- Resolution: 364x364 for 25 FPS or 308x308 for 42 FPS
- Output: Relative depth for obstacle ordering; metric with fine-tuned variant
- Memory: ~640 MB for depth model + ~500 MB for YOLO = fits in 8GB

**Option B: Stereo (metric depth, higher accuracy)**
- Camera: Stereo pair (e.g., ZED 2i, Intel RealSense D435/D455, or custom baseline)
- Model: Isaac ROS ESS (17M params, NVIDIA-optimized)
- Resolution: 576x960 or 288x480 (light)
- Output: Metric depth + confidence maps
- Memory: Optimized by NVIDIA for Jetson

**Option C: Hybrid (best of both)**
- Use stereo camera for metric depth (ESS)
- Run Depth Anything V2 Small on single view for dense relative depth
- Fuse both for robust depth with fallback

---

## Part 5: Key Takeaways (Updated Feb 2026)

1. **For real-time monocular on Jetson Orin Nano (production, today):** Depth Anything V2 Small is the clear winner. 24.8M params, Apache-2.0, 25-42 FPS with TensorRT, well-tested on Jetson. For Q2 2026 and beyond, watch Depth Anything V3 Small.

2. **For real-time stereo on Jetson Orin Nano:** NVIDIA Isaac ROS ESS is the production choice. Natively optimized, ROS 2 integrated, confidence maps included, 20-40+ FPS estimated on Orin Nano.

3. **For highest accuracy (offline/server):** Depth Pro (Apple, Oct 2024) or Metric3D v2 ViT-Giant2 for monocular; FoundationStereo (NVIDIA CVPR 2025) for stereo. All too large for real-time edge but excellent for ground-truth generation.

4. **Metric vs relative depth in 2026:** The gap is closing. Depth Anything V3 (Nov 2025) now offers metric depth from monocular cameras via focal length estimation. UniDepthV2 and Depth Pro achieve zero-shot metric depth without camera intrinsics. HOWEVER, for robotic grasp planning, stereo is still STRONGLY RECOMMENDED because monocular depth (even metric) produces high errors in point cloud conversion, making accurate 3D grasping unreliable.

5. **State of monocular metric depth in 2025-2026:** Production-ready. Models like Depth Pro, UniDepthV2, Depth Anything V3 Metric, and Metric3D v2 now provide metric depth with absolute scale. Research shows sustained progress via CVPR 2023-2025 Monocular Depth Estimation Challenges. Key advances: diffusion distillation (SharpDepth), focal length prediction (Depth Pro), canonical camera space (Metric3D v2), multi-view consistency (DA3).

6. **For litter-picking with grasp planning:** Use STEREO. Monocular depth estimation (even metric variants) lacks scale information for precise 3D object localization. Research shows monocular methods achieve 91-93% grasp success on benchmarks but stereo provides true metric depth via triangulation with lower error. Stereo eliminates scale ambiguity and enhances robustness in texture-poor environments.

7. **License considerations:** Apache-2.0 (DA V2 Small, CREStereo, Isaac ROS SGM) and MIT (RAFT-Stereo, IGEV, ZoeDepth) are commercial-friendly. CC-BY-NC-4.0 (DA V2 Base+, DA V3 Large+, UniDepth) prohibits commercial use. BSD-2-Clause (Metric3D) is permissive. Apple Depth Pro uses Apple Sample Code License (restrictions apply).

8. **Memory is the bottleneck on Jetson Orin Nano 8GB.** Most large ViT-based models (>100M params) will not fit alongside a detection model. Plan for ~600-700 MB for depth + ~500 MB for detection + ~1-2 GB for OS/framework overhead. JetPack 6.2 (Jan 2025) adds "Super Mode" with up to 2x generative AI performance and TensorRT 10.3.

9. **New in 2025-2026:** Depth Anything V3 (Nov 2025) with metric depth + 4K support, Depth Pro (Oct 2024) with sharp metric depth in 0.3s, UniDepthV2 (Feb 2025) with improved efficiency, FoundationStereo (CVPR 2025 Best Paper Nomination) ranking #1 on Middlebury/ETH3D, and JetPack 6.2 with Super Mode for Jetson Orin Nano.

---

## Sources

### Monocular Depth Estimation
- [Depth Anything V3 GitHub](https://github.com/ByteDance-Seed/Depth-Anything-3)
- [Depth Anything V3 Paper (arXiv)](https://arxiv.org/abs/2511.10647)
- [Depth Anything V3 Project Page](https://depth-anything-3.github.io/)
- [Deploy Depth Anything V3 on Jetson AGX Orin - Seeed Studio](https://wiki.seeedstudio.com/deploy_depth_anything_v3_jetson_agx_orin/)
- [ROS2 Depth Anything V3 TensorRT](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt)
- [Depth Anything V2 GitHub](https://github.com/DepthAnything/Depth-Anything-V2)
- [Depth Anything V2 Paper (arXiv)](https://arxiv.org/abs/2406.09414)
- [Depth Anything for Jetson Orin (IRCVLab)](https://github.com/IRCVLab/Depth-Anything-for-Jetson-Orin)
- [Depth Anything TensorRT](https://github.com/spacewalk01/depth-anything-tensorrt)
- [Depth Pro GitHub (Apple)](https://github.com/apple/ml-depth-pro)
- [Depth Pro Paper (arXiv)](https://arxiv.org/abs/2410.02073)
- [Depth Pro - Apple Machine Learning Research](https://machinelearning.apple.com/research/depth-pro)
- [Depth Pro TensorRT Implementation](https://github.com/yuvraj108c/ml-depth-pro-tensorrt)
- [UniDepth GitHub](https://github.com/lpiccinelli-eth/UniDepth)
- [UniDepthV2 Paper (arXiv)](https://arxiv.org/abs/2502.20110)
- [UniDepth Project Page](https://lpiccinelli-eth.github.io/pub/unidepth/)
- [Metric3D GitHub](https://github.com/YvanYin/Metric3D)
- [Metric3D v2 Paper (arXiv)](https://arxiv.org/abs/2404.15506)
- [Metric3D v2 Project Page](https://jugghm.github.io/Metric3Dv2/)
- [ZoeDepth GitHub](https://github.com/isl-org/ZoeDepth)
- [ZoeDepth Paper (arXiv)](https://arxiv.org/abs/2302.12288)

### Stereo Depth Estimation
- [RAFT-Stereo GitHub](https://github.com/princeton-vl/RAFT-Stereo)
- [RAFT-Stereo Paper (arXiv)](https://arxiv.org/abs/2109.07547)
- [CREStereo GitHub (Official MegEngine)](https://github.com/megvii-research/CREStereo)
- [CREStereo Paper (arXiv)](https://arxiv.org/abs/2203.11483)
- [CREStereo PyTorch Port](https://github.com/ibaiGorordo/CREStereo-Pytorch)
- [CREStereo ONNX](https://github.com/ibaiGorordo/ONNX-CREStereo-Depth-Estimation)
- [IGEV GitHub](https://github.com/gangweiX/IGEV)
- [IGEV++ GitHub](https://github.com/gangweiX/IGEV-plusplus)
- [IGEV Paper (arXiv)](https://arxiv.org/abs/2303.06615)
- [IGEV++ Paper (arXiv)](https://arxiv.org/abs/2409.00638)
- [FoundationStereo GitHub](https://github.com/NVlabs/FoundationStereo)
- [FoundationStereo Project Page](https://nvlabs.github.io/FoundationStereo/)
- [FoundationStereo Paper (CVPR 2025)](https://arxiv.org/abs/2501.09898)

### NVIDIA Isaac ROS & Jetson
- [NVIDIA Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)
- [NVIDIA Isaac ROS Stereo Depth Concepts](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/index.html)
- [NVIDIA Isaac ROS Performance](https://nvidia-isaac-ros.github.io/performance/index.html)
- [NVIDIA Jetson Benchmarks](https://developer.nvidia.com/embedded/jetson-benchmarks)
- [JetPack 6.2 Release (Super Mode)](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
- [Jetson Orin Nano Super Guide](https://github.com/ajeetraina/jetson-orin-nano-super-guide)

### Surveys and General Research
- [Survey on Monocular Metric Depth Estimation (Jan 2025)](https://arxiv.org/abs/2501.11841)
- [Roboflow Depth Estimation Models Comparison](https://blog.roboflow.com/depth-estimation-models/)
- [Depth Estimation for Robotic Grasping (ACM 2023)](https://dl.acm.org/doi/10.1145/3634865.3634869)
- [RT-MonoDepth GitHub](https://github.com/Ecalpal/RT-MonoDepth)
