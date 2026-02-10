# State-of-the-Art Semantic Segmentation for Outdoor Terrain Classification (Early 2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB RAM)
**Use Case:** Terrain classification for litter-picking robot — grass, pavement, sand, stairs, water, gravel, dirt, obstacles
**Date:** February 10, 2026

---

## Executive Summary

This document evaluates nine semantic segmentation architectures for real-time outdoor terrain classification on the Jetson Orin Nano Super. The use case requires classifying 8 terrain types at sufficient resolution and speed for safe autonomous navigation of a quadruped litter-picking robot.

**Key Findings:**

1. **Best real-time choice:** PIDNet-S or DDRNet-23-slim — both achieve 77-79% mIoU on Cityscapes at 90-100+ FPS on desktop GPU. With TensorRT FP16 on Jetson Orin Nano Super, expect 25-40+ FPS at 1024x512.
2. **Best accuracy/speed balance:** SegFormer-B0 or B1 — transformer-based with strong generalization, NVIDIA-native TensorRT/TAO support, 37-42% mIoU on ADE20K (150 classes). Isaac ROS integration available.
3. **Best accuracy (non-real-time):** Mask2Former or OneFormer with Swin-L — 57+ mIoU on ADE20K but far too heavy for real-time on Jetson.
4. **SAM-family models (FastSAM, EfficientSAM):** Designed for zero-shot segmentation with prompts, not semantic class prediction. Not suitable as primary terrain classifiers but useful for annotation tooling.
5. **Critical path:** Fine-tune on RUGD/RELLIS-3D terrain datasets, then export via ONNX to TensorRT FP16/INT8 for deployment.

**Recommended architecture for CleanWalker:**
- **Primary:** SegFormer-B0 (NVIDIA ecosystem, Isaac ROS native, TAO fine-tuning, TensorRT export)
- **Fallback:** PIDNet-S (lighter, faster, MIT license, but less NVIDIA toolchain support)
- **Dataset:** Fine-tune on RUGD + RELLIS-3D combined, mapped to 8 CleanWalker terrain classes

---

## 1. Hardware Platform: Jetson Orin Nano Super 8GB

| Specification | Value |
|--------------|-------|
| GPU | 1024-core NVIDIA Ampere, 32 Tensor Cores |
| CPU | 6-core Arm Cortex-A78AE v8.2 64-bit |
| AI Performance | 67 TOPS (INT8) / 34 TOPS (FP16) |
| Memory | 8GB 128-bit LPDDR5, 68 GB/s bandwidth |
| Power Mode | 7W / 15W / 25W (Super mode) |
| TensorRT | Yes (JetPack 6.x) |
| DLA | No (only on AGX Orin) |
| Price | ~$249 (developer kit) |

**Performance context:** The Orin Nano Super at 25W mode delivers approximately 1.7x the performance of the original Orin Nano. With TensorRT FP16 optimization, expect 2-3x speedup over native PyTorch inference. INT8 quantization adds another 1.5-2x but may reduce mIoU by 1-3%.

---

## 2. Model-by-Model Analysis

### 2.1 SegFormer (NVIDIA, NeurIPS 2021)

**Paper:** "SegFormer: Simple and Efficient Design for Semantic Segmentation with Transformers" — Xie et al., 2021
**Repository:** https://github.com/NVlabs/SegFormer
**License:** NVIDIA Source Code License (non-commercial research only). Commercial use requires NVIDIA Research License inquiry. HuggingFace checkpoints available under more permissive terms.

**Architecture:** Hierarchical Vision Transformer encoder (Mix Transformer / MiT) with lightweight All-MLP decoder. No positional encoding (uses overlapping patch embeddings), enabling flexible input resolutions. Scales from B0 (tiny) to B5 (large).

#### Performance Table — ADE20K (512x512 input)

| Variant | Params (M) | GFLOPs | mIoU (SS) | mIoU (MS) |
|---------|-----------|--------|-----------|-----------|
| SegFormer-B0 | 3.8 | 8.4 | 37.4 | 38.0 |
| SegFormer-B1 | 13.7 | 15.9 | 42.2 | 43.1 |
| SegFormer-B2 | 27.5 | 62.4 | 46.5 | 47.5 |
| SegFormer-B3 | 47.3 | 79.0 | 49.4 | 50.0 |
| SegFormer-B4 | 64.1 | 95.7 | 50.3 | 51.1 |
| SegFormer-B5 | 84.7 | 183.3 | 51.0 | 51.8 |

*SS = single-scale, MS = multi-scale inference*

#### Performance Table — Cityscapes (1024x1024 input)

| Variant | GFLOPs | mIoU (SS) | mIoU (MS) | FPS (V100) |
|---------|--------|-----------|-----------|------------|
| SegFormer-B0 | 125.5 | 76.2 | 78.1 | ~48 |
| SegFormer-B1 | 243.7 | 78.5 | 80.0 | ~28 |
| SegFormer-B2 | 717.1 | 81.0 | 82.2 | ~14 |
| SegFormer-B3 | 962.9 | 81.7 | 83.3 | ~10 |
| SegFormer-B4 | 1240.6 | 82.3 | 83.9 | ~7 |
| SegFormer-B5 | 1460.3 | 82.4 | 84.0 | ~5 |

*FPS measured on V100 without TensorRT*

#### Jetson Orin Nano Compatibility
- **TensorRT:** Supported via NVIDIA TAO Toolkit. Export: PyTorch -> ONNX -> TensorRT (FP16). INT8 calibration NOT supported for SegFormer in TAO as of 2025.
- **Isaac ROS:** Native `isaac_ros_segformer` package available. PeopleSemSegFormer pre-trained model on NGC.
- **TAO Toolkit:** Full fine-tuning pipeline: data prep, training, evaluation, export, deployment to DeepStream.
- **Estimated Jetson Orin Nano Super FPS (B0, 512x512, TensorRT FP16):** ~30-50 FPS (estimated based on GFLOPs scaling and Ampere tensor core throughput).

#### Pre-trained on Outdoor/Terrain?
Pre-trained models available for ADE20K (150 classes including some outdoor) and Cityscapes (urban driving). No dedicated terrain/off-road pre-training, but ADE20K includes grass, earth/ground, sand, water, path, rock, stairs. Fine-tuning required for the 8 CleanWalker terrain classes.

**Verdict:** Top recommendation for CleanWalker. NVIDIA-native toolchain (TAO, Isaac ROS, TensorRT, DeepStream) makes it the most integrated option. B0 is the clear target variant — 3.8M parameters, 8.4 GFLOPs at 512x512, real-time capable on Jetson.

---

### 2.2 PIDNet (CVPR 2023)

**Paper:** "PIDNet: A Real-time Semantic Segmentation Network Inspired by PID Controllers" — Xu et al., CVPR 2023
**Repository:** https://github.com/XuJiacong/PIDNet
**License:** MIT (fully permissive, commercial use allowed)

**Architecture:** Three-branch network inspired by PID (Proportional-Integral-Derivative) controllers. The Detail branch captures high-resolution spatial details (P), the Context branch extracts semantic context (I), and the Boundary branch provides boundary attention (D) to guide fusion. This addresses the "overshoot" problem where detail and context features interfere during fusion.

#### Performance Table — Cityscapes (2048x1024 input, RTX 3090)

| Variant | Params (M) | GFLOPs | Val mIoU | Test mIoU | FPS |
|---------|-----------|--------|----------|-----------|-----|
| PIDNet-S | 7.6 | 47.6 | 78.8 | 78.6 | 93.2 |
| PIDNet-M | 34.4 | 197.4 | 80.1 | 80.1 | 39.8 |
| PIDNet-L | 36.9 | 275.8 | 80.9 | 80.6 | 31.1 |

*ADE20K results not reported in the paper*

#### CamVid Results
| Variant | mIoU | FPS |
|---------|------|-----|
| PIDNet-S | 80.1 | 153.7 |

#### Jetson Orin Nano Compatibility
- **TensorRT:** No built-in export. PyTorch -> ONNX -> TensorRT pipeline must be done manually. Community implementations exist.
- **Isaac ROS:** Not natively supported. Would require custom DNN inference node.
- **ONNX:** Standard PyTorch export to ONNX should work (standard ops).
- **Estimated Jetson Orin Nano Super FPS (PIDNet-S, 1024x512, TensorRT FP16):** ~25-40 FPS (scaled from RTX 3090 at full res).
- **Qualcomm AI Hub:** PIDNet-S available as optimized model, indicating good portability to edge devices.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on Cityscapes (urban road scenes) and CamVid (driving video). No off-road/terrain pre-training. Would need fine-tuning on RUGD/RELLIS-3D.

**Verdict:** Excellent real-time performance with MIT license. PIDNet-S at 7.6M params is very lightweight. Main drawback: no NVIDIA ecosystem integration (no TAO, no Isaac ROS package). Best choice if license freedom is critical.

---

### 2.3 DDRNet (IEEE T-ITS 2023)

**Paper:** "Deep Dual-resolution Networks for Real-time and Accurate Semantic Segmentation of Road Scenes" — Hong et al., 2021
**Repository:** https://github.com/ydhongHIT/DDRNet
**License:** MIT

**Architecture:** Two parallel branches at different resolutions with bilateral fusion at multiple stages. The high-resolution branch preserves spatial detail while the low-resolution branch captures semantic context. A Deep Aggregation Pyramid Pooling Module (DAPPM) enlarges the effective receptive field on low-res features.

#### Performance Table — Cityscapes

| Variant | Params (M) | Val mIoU | Test mIoU | FPS (2080Ti) |
|---------|-----------|----------|-----------|--------------|
| DDRNet-23-slim | 5.7 | 77.8 | 77.4 | 102 / 230* |
| DDRNet-23 | 20.1 | 79.5 | 79.4 | 62 |
| DDRNet-39 | 32.7 | - | 80.4 | 23 |

*230 FPS reported on CamVid dataset*

#### Jetson Orin Nano Compatibility
- **TensorRT:** The README mentions torch2trt support. DDRNet-23-slim can achieve 130+ FPS with torch2trt on desktop GPU.
- **ONNX:** Standard PyTorch export.
- **Isaac ROS:** Not natively supported.
- **Estimated Jetson Orin Nano Super FPS (DDRNet-23-slim, 1024x512, TensorRT FP16):** ~30-50 FPS.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on Cityscapes and CamVid (urban driving), plus ImageNet backbone pre-training. No dedicated terrain datasets.

**Verdict:** DDRNet-23-slim is the lightest model in this comparison at 5.7M params with strong Cityscapes accuracy. torch2trt compatibility is a plus. MIT license. Strong contender alongside PIDNet-S.

---

### 2.4 BiSeNet V2 (IJCV 2021)

**Paper:** "BiSeNet V2: Bilateral Network with Guided Aggregation for Real-time Semantic Segmentation" — Yu et al., 2021
**Repository:** https://github.com/CoinCheung/BiSeNet
**License:** MIT

**Architecture:** Two-pathway bilateral network. The Detail Branch uses wide channels and shallow layers for high-resolution spatial features. The Semantic Branch uses narrow channels and deep layers with fast downsampling for high-level context. A Guided Aggregation Layer fuses both branches. Booster training strategy improves accuracy at zero extra inference cost.

#### Performance Table — Cityscapes (2048x1024 input)

| Variant | GFLOPs | mIoU (Test) | FPS (GTX 1080Ti) |
|---------|--------|-------------|-------------------|
| BiSeNet V2 | 21.2 | 72.6 | 156 |
| BiSeNet V2-Large | 118.5 | 75.3 | 47.3 |

Community implementation reports on Cityscapes val:
- FP32: 103 FPS
- FP16: 161 FPS
- INT8: 198 FPS

#### Jetson Orin Nano Compatibility
- **TensorRT:** Explicitly supported in the community repository (dedicated tensorrt/ directory).
- **ONNX:** Supported.
- **OpenVINO:** Supported.
- **Triton Inference Server:** Supported.
- **Estimated Jetson Orin Nano Super FPS (BiSeNet V2, 1024x512, TensorRT FP16):** ~40-60 FPS.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on Cityscapes, COCO, and ADE20K. No dedicated off-road/terrain pre-training.

**Verdict:** Extremely fast with the lowest GFLOPs (21.2) in this comparison, but also the lowest accuracy (72.6% Cityscapes). The strong TensorRT/ONNX export ecosystem is a significant advantage. The INT8 acceleration (198 FPS on 1080Ti) suggests excellent Jetson performance. Good choice if speed is the absolute priority over accuracy.

---

### 2.5 PP-LiteSeg (Baidu, 2022)

**Paper:** "PP-LiteSeg: A Superior Real-Time Semantic Segmentation Model" — Peng et al., 2022
**Repository:** https://github.com/PaddlePaddle/PaddleSeg
**License:** Apache 2.0

**Architecture:** Encoder-aggregator-decoder with three key modules: (1) STDCNet encoder for efficient feature extraction, (2) Simple Pyramid Pooling Module (SPPM) for low-cost global context aggregation, (3) Flexible Lightweight Decoder (FLD) for feature fusion, and (4) Unified Attention Fusion Module (UAFM) combining spatial and channel attention.

#### Performance Table — Cityscapes Test Set (GTX 1080Ti)

| Variant | Backbone | Resolution | mIoU (%) | FPS |
|---------|----------|-----------|----------|-----|
| PP-LiteSeg-T1 | STDC1 | 512x1024 | 72.0 | 273.6 |
| PP-LiteSeg-B1 | STDC2 | 512x1024 | 73.9 | 195.3 |
| PP-LiteSeg-T2 | STDC1 | 768x1536 | 74.9 | 143.6 |
| PP-LiteSeg-B2 | STDC2 | 768x1536 | 77.5 | 102.6 |

#### Jetson Orin Nano Compatibility
- **TensorRT:** PaddlePaddle models can be exported via Paddle2ONNX -> ONNX -> TensorRT. Not a native PyTorch model.
- **PaddlePaddle dependency:** Requires PaddlePaddle framework, which has less Jetson community support than PyTorch.
- **ONNX:** Paddle2ONNX converter available.
- **PyTorch port:** Community PyTorch reimplementation exists (zh320/realtime-semantic-segmentation-pytorch).
- **Estimated Jetson Orin Nano Super FPS (T2, 768x512, TensorRT FP16):** ~40-70 FPS.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on Cityscapes only. PaddleSeg offers pre-trained models on various datasets but no dedicated terrain/off-road options.

**Verdict:** Excellent speed-accuracy tradeoff. PP-LiteSeg-T at 512x1024 runs at 273 FPS but only 72% mIoU; PP-LiteSeg-B at 768x1536 hits 77.5% at 102 FPS. Main concern: PaddlePaddle ecosystem is less integrated with NVIDIA Jetson toolchain than PyTorch. Apache 2.0 license is commercially friendly.

---

### 2.6 OneFormer (CVPR 2023)

**Paper:** "OneFormer: One Transformer to Rule Universal Image Segmentation" — Jain et al., CVPR 2023
**Repository:** https://github.com/SHI-Labs/OneFormer
**License:** MIT

**Architecture:** Universal segmentation framework handling semantic, instance, and panoptic tasks with a single model. Uses task-conditioned joint training with task tokens. Builds on Mask2Former architecture with a contrastive query loss. Supports Swin-L, ConvNeXt-L, and DiNAT-L backbones.

#### Performance Table

| Backbone | Dataset | mIoU | PQ | AP | Params (M) |
|----------|---------|------|----|----|-----------|
| Swin-L | ADE20K | 57.0 | 49.8 | 35.9 | 219 |
| DiNAT-L | ADE20K | 57.7 | 51.5 | 37.8 | 219 |
| Swin-L | Cityscapes | 84.4 | 67.2 | 45.6 | 219 |
| ConvNeXt-L | Cityscapes | - | 68.5 | - | 219 |

Training: 8x A100 80GB GPUs, 160K iterations (ADE20K), 90K iterations (Cityscapes).

#### Jetson Orin Nano Compatibility
- **NOT feasible for real-time.** 219M parameters with heavy Swin-L/DiNAT-L backbone.
- **FLOPs:** 1000+ GFLOPs (estimated, comparable to Mask2Former Swin-L at ~1470 GFLOPs).
- **Estimated FPS on Jetson Orin Nano Super:** <1 FPS. Far too heavy.
- **NVIDIA TAO:** OneFormer is supported in NVIDIA TAO Toolkit for fine-tuning.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on ADE20K, Cityscapes, COCO. Strong generalization due to universal training, but no dedicated terrain pre-training.

**Verdict:** State-of-the-art accuracy (57% mIoU on ADE20K, 84.4% on Cityscapes) but completely impractical for real-time on Jetson edge hardware. Useful for: (a) offline annotation / pseudo-label generation for terrain datasets, (b) teacher model for knowledge distillation into a lighter student (e.g., SegFormer-B0).

---

### 2.7 Mask2Former (Meta/FAIR, CVPR 2022)

**Paper:** "Masked-attention Mask Transformer for Universal Image Segmentation" — Cheng et al., CVPR 2022
**Repository:** https://github.com/facebookresearch/Mask2Former
**License:** MIT (majority), Swin-Transformer components under MIT, Deformable-DETR under Apache 2.0

**Architecture:** Universal segmentation with masked attention mechanism in the transformer decoder. Constrains cross-attention to predicted mask regions, achieving 3x faster convergence than standard cross-attention. Backbone + pixel decoder + transformer decoder pipeline. Supports R50, R101, Swin-B, Swin-L backbones.

#### Performance Table — ADE20K Semantic Segmentation

| Backbone | Params (M) | FLOPs (G) | mIoU |
|----------|-----------|-----------|------|
| R50 | 44 | 226 | 47.2 |
| R101 | 63 | 293 | 47.7 |
| Swin-B | 107 | 603 | 53.9 |
| Swin-L | 216 | 868 | 57.7 |

#### Cityscapes Semantic Segmentation
- Swin-L: 83.3% mIoU (val), ~1470 GFLOPs

#### Jetson Orin Nano Compatibility
- **R50 variant:** 44M params, 226 GFLOPs — borderline feasible at low resolution but likely <5 FPS.
- **Swin-L variant:** Not feasible for real-time on Jetson.
- **NVIDIA TAO:** Mask2Former is supported in NVIDIA TAO Toolkit.
- **TensorRT:** Deformable attention operations may require custom TensorRT plugins.

#### Pre-trained on Outdoor/Terrain?
Pre-trained on ADE20K, Cityscapes, COCO. No dedicated terrain datasets.

**Verdict:** Like OneFormer, too heavy for real-time inference on Jetson. Even the R50 variant at 226 GFLOPs is ~27x heavier than SegFormer-B0. Best used as an offline tool for dataset annotation, pseudo-label generation, or knowledge distillation.

---

### 2.8 FastSAM (CASIA, 2023)

**Paper:** "Fast Segment Anything" — Zhao et al., 2023
**Repository:** https://github.com/CASIA-LMC-Lab/FastSAM
**License:** Apache 2.0

**Architecture:** CNN-based approach to the Segment Anything task. Uses YOLOv8-seg as the backbone for all-instance segmentation, then prompt-guided selection to choose the relevant mask. Trained on only 2% of the SA-1B dataset. Two variants: FastSAM-x (YOLOv8x-seg, ~68M params) and FastSAM-s (YOLOv8s-seg, ~12M params).

#### Performance

| Variant | Params (M) | Speed vs SAM | Notes |
|---------|-----------|-------------|-------|
| FastSAM-x | 68 | 50x faster than SAM | YOLOv8x backbone |
| FastSAM-s | 12 | ~100x faster than SAM | YOLOv8s backbone |

#### Jetson Orin Nano Compatibility
- **TensorRT:** Community TensorRT implementation exists (FastSam_Awesome_TensorRT). YOLOv8 TensorRT export is well-supported.
- **ONNX:** Standard YOLO export pipeline.
- **Estimated FPS:** FastSAM-s could run at 15-30 FPS on Jetson with TensorRT.

#### Critical Limitation for Terrain Classification
FastSAM is a **class-agnostic** segmentation model. It segments "everything" in the image into masks but does NOT assign semantic class labels (grass, pavement, water, etc.). It requires text/point/box prompts to identify specific objects. This makes it unsuitable as a standalone terrain classifier.

**Possible use:** Generate mask proposals, then classify each mask with a lightweight classifier head. However, this adds complexity and latency compared to direct semantic segmentation.

**Verdict:** Not suitable for direct terrain classification. Useful for: annotation tooling, mask proposal generation, or combined with CLIP for open-vocabulary segmentation. The prompt-based paradigm does not match the "classify every pixel into terrain type" requirement.

---

### 2.9 EfficientSAM (Meta/FAIR, CVPR 2024)

**Paper:** "EfficientSAM: Leveraged Masked Image Pretraining for Efficient Segment Anything" — Xiong et al., CVPR 2024
**Repository:** https://github.com/yformer/EfficientSAM
**License:** Apache 2.0

**Architecture:** Retains SAM's original encoder-decoder architecture but replaces the heavy ViT-H image encoder with lightweight alternatives (ViT-Tiny or ViT-Small). Uses SAM-based Masked Image (SAMI) pre-training strategy to train efficient encoders.

#### Performance

| Variant | Params (M) | COCO AP (SegAny) | LVIS AP (SegAny) | Speed |
|---------|-----------|-----------------|-----------------|-------|
| SAM ViT-H (baseline) | 632 | - | - | 1x |
| EfficientSAM-Ti | 10 | ~4 AP gain vs FastSAM | ~5.3 AP gain vs FastSAM | ~20 img/s |
| EfficientSAM-S | 25 | ~6.5 AP gain vs FastSAM | ~7.8 AP gain vs FastSAM | ~15 img/s |

**Related:** NanoSAM (NVIDIA) — distilled MobileSAM, runs real-time on Jetson Orin, 5x faster than FastSAM. Available at https://github.com/NVIDIA-AI-IOT/nanosam.

#### Jetson Orin Nano Compatibility
- **EfficientViT-SAM:** 48.9x measured TensorRT speedup on A100 over SAM ViT-H. Jetson deployment feasible.
- **NanoSAM:** Explicitly designed for Jetson Orin. Runs real-time with TensorRT. FP32 precision required for image encoder (FP16 produces errors).

#### Critical Limitation for Terrain Classification
Same as FastSAM: EfficientSAM is a **class-agnostic prompt-based** segmentation model. It segments objects based on prompts (points, boxes, text) but does NOT perform semantic classification of terrain types.

**Verdict:** Not suitable as a standalone terrain classifier. Useful for: real-time mask generation on Jetson (especially NanoSAM), annotation tooling, or as part of a pipeline where masks are classified by a secondary model.

---

## 3. Comparison Tables

### 3.1 Model Size and Compute

| Model | Params (M) | GFLOPs | Input Resolution | Architecture |
|-------|-----------|--------|-----------------|-------------|
| **SegFormer-B0** | **3.8** | **8.4** | 512x512 | Transformer |
| SegFormer-B1 | 13.7 | 15.9 | 512x512 | Transformer |
| SegFormer-B2 | 27.5 | 62.4 | 512x512 | Transformer |
| **DDRNet-23-slim** | **5.7** | **~16** | 1024x512 | CNN |
| DDRNet-23 | 20.1 | ~60 | 2048x1024 | CNN |
| **PIDNet-S** | **7.6** | **47.6** | 2048x1024 | CNN |
| PIDNet-M | 34.4 | 197.4 | 2048x1024 | CNN |
| **BiSeNet V2** | **~5*** | **21.2** | 2048x1024 | CNN |
| PP-LiteSeg-T | ~8** | ~10** | 512x1024 | CNN |
| PP-LiteSeg-B | ~11** | ~20** | 768x1536 | CNN |
| FastSAM-s | 12 | ~30** | 640x640 | CNN (YOLO) |
| EfficientSAM-Ti | 10 | ~25** | 1024x1024 | Transformer |
| Mask2Former (R50) | 44 | 226 | 512x512 | Transformer |
| OneFormer (Swin-L) | 219 | ~1200** | 640x640 | Transformer |

*Params not explicitly reported in paper; estimated from architecture description*
**Estimated; not explicitly reported*

### 3.2 Accuracy Comparison

| Model | ADE20K mIoU | Cityscapes mIoU (val) | Cityscapes mIoU (test) |
|-------|------------|----------------------|----------------------|
| SegFormer-B0 | 37.4 | 76.2 | - |
| SegFormer-B1 | 42.2 | 78.5 | - |
| SegFormer-B2 | 46.5 | 81.0 | - |
| SegFormer-B5 | 51.0 | 82.4 | 84.0 |
| PIDNet-S | N/A | 78.8 | 78.6 |
| PIDNet-M | N/A | 80.1 | 80.1 |
| PIDNet-L | N/A | 80.9 | 80.6 |
| DDRNet-23-slim | N/A | 77.8 | 77.4 |
| DDRNet-23 | N/A | 79.5 | 79.4 |
| DDRNet-39 | N/A | - | 80.4 |
| BiSeNet V2 | N/A | ~74.9 | 72.6 |
| BiSeNet V2-Large | N/A | - | 75.3 |
| PP-LiteSeg-T2 | N/A | - | 74.9 |
| PP-LiteSeg-B2 | N/A | - | 77.5 |
| Mask2Former (Swin-L) | 57.7 | 83.3 | - |
| OneFormer (DiNAT-L) | 57.7 | 84.4 | - |

**Note:** ADE20K has 150 classes (harder task), Cityscapes has 19 classes (urban driving). ADE20K results are more relevant for terrain generalization since it includes diverse outdoor classes.

### 3.3 Speed Comparison (Desktop GPU Baseline)

| Model | GPU | Resolution | FPS | Real-time? |
|-------|-----|-----------|-----|------------|
| SegFormer-B0 | V100 | 1024x1024 | ~48 | Yes |
| PIDNet-S | RTX 3090 | 2048x1024 | 93.2 | Yes |
| DDRNet-23-slim | RTX 2080Ti | 2048x1024 | 102 | Yes |
| BiSeNet V2 | GTX 1080Ti | 2048x1024 | 156 | Yes |
| PP-LiteSeg-T1 | GTX 1080Ti | 512x1024 | 273.6 | Yes |
| PP-LiteSeg-B2 | GTX 1080Ti | 768x1536 | 102.6 | Yes |
| Mask2Former (R50) | - | 512x512 | ~8** | No |
| OneFormer (Swin-L) | - | 640x640 | ~2** | No |

**Estimated based on model complexity*

### 3.4 Jetson Orin Nano Super Estimated Performance

| Model | Est. FPS (FP16 TensorRT) | Est. FPS (INT8 TensorRT) | Resolution | Feasible? |
|-------|--------------------------|--------------------------|-----------|-----------|
| SegFormer-B0 | 30-50 | 40-70 | 512x512 | YES |
| SegFormer-B1 | 15-25 | 20-35 | 512x512 | YES |
| PIDNet-S | 25-40 | 35-55 | 1024x512 | YES |
| DDRNet-23-slim | 30-50 | 45-70 | 1024x512 | YES |
| BiSeNet V2 | 40-60 | 55-80 | 1024x512 | YES |
| PP-LiteSeg-T | 50-80 | 70-100 | 512x1024 | YES |
| FastSAM-s | 15-30 | 20-40 | 640x640 | YES (but no class labels) |
| NanoSAM | 20-40 | N/A (FP32 only) | 1024x1024 | YES (but no class labels) |
| Mask2Former (R50) | 2-5 | 3-7 | 512x512 | NO (too slow) |
| OneFormer (Swin-L) | <1 | <2 | 640x640 | NO |

**Note:** These are estimates based on scaling desktop GPU benchmarks by the Orin Nano Super's relative compute (67 TOPS INT8, 1024 CUDA cores). Actual performance depends heavily on model-specific TensorRT optimization, operator support, and memory bandwidth utilization. Always benchmark on actual hardware.

### 3.5 License Comparison

| Model | License | Commercial Use | Fine-tuning Allowed |
|-------|---------|---------------|-------------------|
| SegFormer | NVIDIA Source Code License | NO (research only)* | Yes (research) |
| PIDNet | MIT | YES | Yes |
| DDRNet | MIT | YES | Yes |
| BiSeNet V2 | MIT | YES | Yes |
| PP-LiteSeg | Apache 2.0 | YES | Yes |
| OneFormer | MIT | YES | Yes |
| Mask2Former | MIT / Apache 2.0 | YES | Yes |
| FastSAM | Apache 2.0 | YES | Yes |
| EfficientSAM | Apache 2.0 | YES | Yes |

*SegFormer weights from NVlabs GitHub are research-only. However, HuggingFace community models fine-tuned from publicly available weights may have different terms. NVIDIA TAO-trained models follow NVIDIA's license terms. For commercial deployment, consider: (a) licensing from NVIDIA, (b) using MIT-licensed alternatives (PIDNet, DDRNet, BiSeNet V2), (c) training from scratch on a permissively-licensed architecture.*

---

## 4. Terrain / Outdoor Segmentation Datasets

### 4.1 Dataset Overview

| Dataset | Year | Images | Classes | Modalities | Focus | License |
|---------|------|--------|---------|-----------|-------|---------|
| **RUGD** | 2019 | 7,456 annotated | 24 | RGB | Off-road robot navigation | Research |
| **RELLIS-3D** | 2021 | 6,235 images + 13,556 LiDAR | 20 | RGB, LiDAR, IMU | Off-road multi-modal | CC BY-SA 4.0 |
| **Freiburg Forest** | 2016 | 366 annotated | 6 | RGB, NIR, NRG, Depth | Forest trails | Research |
| **GOOSE** | 2024 | 10,000 annotated | 64 | RGB, LiDAR, NIR | Unstructured outdoor | CC BY-SA |
| **WildScenes** | 2024 | Large-scale | 15+ | RGB, LiDAR | Australian forests | CC BY |
| **TAS500** | 2020 | 500 | 9 | RGB | Sidewalks/trails | Research |
| **YCOR** | 2006 | 1,076 | 8 | RGB | Off-road CMU/Yamaha | Research |
| **ADE20K** | 2017 | 25,574 | 150 | RGB | General scenes | BSD |
| **Cityscapes** | 2016 | 5,000 fine | 30 (19 eval) | RGB, Stereo | Urban driving | Custom |
| **COCO-Stuff** | 2018 | 164K | 172 | RGB | General scenes+stuff | CC BY 4.0 |

### 4.2 RUGD (Robot Unstructured Ground Driving Dataset)

**Source:** US Army Research Lab / University of Michigan
**URL:** http://rugd.vision/
**Paper:** "A RUGD Dataset for Autonomous Navigation and Visual Perception in Unstructured Outdoor Environments" (IROS 2019)

- 18 video sequences from small unmanned ground robot
- 37,000+ total frames, 7,456 with pixel-level annotations
- 24 classes including: **dirt, grass, gravel, sand, mud, water, asphalt, concrete, rock, log, mulch, barrier, sky, tree, bush, sign, pole, fence, building, person, vehicle, bicycle, void**
- Environments: trails, creeks, parks, villages
- Resolution: 688x550
- **Highly relevant** — contains 6 of 8 CleanWalker target terrain classes directly

### 4.3 RELLIS-3D

**Source:** Texas A&M University
**URL:** https://github.com/unmannedlab/RELLIS-3D
**Paper:** "RELLIS-3D Dataset: Data, Benchmarks and Analysis" (2021)

- Captured with Clearpath Warthog robot
- 6,235 annotated images + 13,556 LiDAR scans
- 20 classes: **void, dirt, grass, tree, pole, water, sky, vehicle, object, asphalt, building, log, person, fence, bush, concrete, barrier, puddle, mud, rubble**
- Multi-modal: RGB camera, LiDAR, GPS/IMU
- Resolution: 1920x1200
- **Highly relevant** — derived from RUGD ontology with additional terrain types

### 4.4 GOOSE (German Outdoor and Offroad Dataset)

**Source:** Fraunhofer IOSB (2024)
**URL:** https://goose-dataset.de/
**Paper:** "The GOOSE Dataset for Perception in Unstructured Environments" (2024)

- 10,000 pixel-wise annotated image-LiDAR pairs
- 64 fine-grained classes (vegetation, terrain, sky categories)
- Includes NIR, surround view, RTK-GNSS localization
- CC BY-SA license
- **Most comprehensive** terrain dataset available as of 2024
- Ideal for pre-training before fine-tuning on CleanWalker-specific classes

### 4.5 GANav (Terrain Navigability)

**Source:** University of Maryland GAMMA Lab
**Paper:** "GANav: Efficient Terrain Segmentation for Robot Navigation in Unstructured Outdoor Environments" (IEEE RA-L 2022)

- Not a dataset but a **terrain segmentation method** with a group-wise attention mechanism
- Classifies terrain by navigability levels (smooth, rough, bumpy, forbidden, obstacle, background)
- Achieves 2-39% mIoU improvement over SOTA on RUGD and 5-19% on RELLIS-3D
- Tested on ClearPath Jackal and Husky robots
- **Directly relevant architecture** — could be applied to CleanWalker for combined terrain + navigability classification

### 4.6 Mapping CleanWalker Classes to Existing Datasets

| CleanWalker Class | RUGD | RELLIS-3D | GOOSE | ADE20K |
|------------------|------|-----------|-------|--------|
| Grass | grass | grass | Yes | Yes |
| Pavement | concrete, asphalt | concrete, asphalt | Yes | Yes (sidewalk, road) |
| Sand | sand | N/A | Yes | Yes |
| Stairs | N/A | N/A | Partial | Yes |
| Water | water | water, puddle | Yes | Yes |
| Gravel | gravel | N/A | Yes | N/A |
| Dirt | dirt | dirt | Yes | Yes (earth) |
| Obstacles | barrier, log, fence | barrier, log, fence | Yes | Yes (various) |

**Gap analysis:** "Stairs" is the least represented class. ADE20K contains stairs but off-road datasets do not. A small supplementary dataset of stairway images with annotations may be needed.

---

## 5. Fine-Tuning Requirements

### 5.1 Recommended Fine-Tuning Pipeline

```
1. Pre-trained backbone (ImageNet or ADE20K)
       |
2. Combine RUGD + RELLIS-3D + GOOSE (merge/remap to 8 classes)
       |
3. Fine-tune on combined dataset (SegFormer-B0 or PIDNet-S)
       |
4. Validate on held-out test split
       |
5. Export: PyTorch -> ONNX -> TensorRT (FP16 or INT8 + calibration)
       |
6. Deploy on Jetson Orin Nano Super
```

### 5.2 Data Requirements

| Aspect | Recommendation |
|--------|---------------|
| Minimum images | ~2,000-5,000 annotated images across all 8 classes |
| Available from RUGD + RELLIS-3D | ~13,000+ annotated images (need class remapping) |
| GOOSE supplement | 10,000 additional images (64 classes, remap to 8) |
| Custom collection | 200-500 images of stairs (gap filling) + local environment samples |
| Annotation tool | Use Mask2Former/OneFormer for pseudo-labeling, then manual correction |
| Augmentation | Random crop, flip, color jitter, scale, Gaussian blur |

### 5.3 Compute Requirements for Fine-Tuning

| Model | GPU | Training Time | Epochs | Batch Size | LR |
|-------|-----|--------------|--------|-----------|-----|
| SegFormer-B0 | 1x RTX 3090/4090 | 4-8 hours | 160K iters (~300 epochs) | 8-16 | 6e-5 (AdamW) |
| PIDNet-S | 1x RTX 3090/4090 | 6-12 hours | 484 epochs (Cityscapes-style) | 12 | 0.01 (SGD) |
| DDRNet-23-slim | 1x RTX 3090/4090 | 4-8 hours | 500 epochs | 12 | 0.01 (SGD) |
| BiSeNet V2 | 1x RTX 3090/4090 | 3-6 hours | 150K iters | 16 | 0.05 (SGD) |

**HuggingFace SegFormer fine-tuning** is well-documented and simple:
- Use `SegformerForSemanticSegmentation` from transformers library
- Load `nvidia/mit-b0` pre-trained backbone
- Replace classification head for 8 classes
- Train with standard cross-entropy loss
- Full tutorial available: https://huggingface.co/blog/fine-tune-segformer

### 5.4 Class Remapping Strategy

**From RUGD (24 classes) to CleanWalker (8 classes):**
```
grass           -> grass
concrete        -> pavement
asphalt         -> pavement
sand            -> sand
water           -> water
gravel          -> gravel
dirt            -> dirt
mud             -> dirt
mulch           -> dirt
rock            -> obstacles
log             -> obstacles
barrier         -> obstacles
fence           -> obstacles
building        -> obstacles
person          -> obstacles
vehicle         -> obstacles
tree            -> (ignore / background)
bush            -> (ignore / background)
sky             -> (ignore / background)
sign            -> obstacles
pole            -> obstacles
void            -> (ignore)
```

---

## 6. NVIDIA Isaac ROS Segmentation

### 6.1 Available Packages

Isaac ROS Image Segmentation provides four ROS 2 packages:

| Package | Model | Use Case |
|---------|-------|----------|
| `isaac_ros_unet` | U-NET | Biomedical-style segmentation |
| `isaac_ros_segformer` | SegFormer | Transformer-based semantic segmentation |
| `isaac_ros_segment_anything` | SAM | Prompt-based zero-shot segmentation |
| `isaac_ros_segment_anything2` | SAM 2 | Video segmentation with tracking |

### 6.2 SegFormer in Isaac ROS

- Pre-trained model: **PeopleSemSegFormer** on NGC (people segmentation)
- Custom models: Train via NVIDIA TAO Toolkit, export to ONNX/TensorRT
- Inference: GPU-accelerated via TensorRT or Triton Inference Server
- Integration: ROS 2 topic-based — subscribes to image topics, publishes segmentation masks

### 6.3 Custom Model Deployment Pipeline

```
NVIDIA TAO Toolkit
├── Data preparation (convert to TFRecord)
├── Training (SegFormer-B0 with custom 8 classes)
├── Evaluation
├── Export to ONNX
└── Convert to TensorRT engine (trtexec or tao-deploy)
       |
Isaac ROS
├── Load TensorRT engine
├── Subscribe to camera topic
├── Run GPU-accelerated inference
└── Publish segmentation mask topic
       |
Navigation Stack
├── Terrain-aware costmap
├── Path planning on safe surfaces
└── Obstacle avoidance
```

### 6.4 Platform Compatibility (as of Isaac ROS 3.x / 2025)

- **Jetson Thor (T5000):** Officially supported with JetPack 7.0
- **Jetson AGX Orin:** Supported
- **Jetson Orin Nano:** Supported via JetPack 6.x (check specific Isaac ROS version compatibility)
- **ROS 2:** Jazzy (primary), Humble (secondary)

**Note:** Isaac ROS 3.x (2025) lists Jetson Thor as the primary Jetson target. Earlier versions (2.x) explicitly support Jetson Orin Nano. Verify compatibility with the specific Isaac ROS release being used.

---

## 7. Deployment Recommendations for CleanWalker

### 7.1 Architecture Selection

**Primary recommendation: SegFormer-B0**
- 3.8M params, 8.4 GFLOPs — well within Jetson Orin Nano Super budget
- NVIDIA ecosystem: TAO fine-tuning, Isaac ROS integration, TensorRT export, DeepStream
- Transformer architecture generalizes better than CNNs to unseen terrain distributions
- ADE20K pre-training covers many outdoor classes
- Risk: NVIDIA Source Code License for research-only use (commercial deployment needs licensing inquiry or alternative weights)

**Alternative: PIDNet-S**
- 7.6M params, 47.6 GFLOPs — slightly heavier but still real-time on Jetson
- MIT license — fully commercially permissive
- CNN architecture is simpler to deploy and optimize
- Strong Cityscapes results (78.8% val mIoU)
- Risk: No Isaac ROS integration, manual ONNX/TensorRT export required

**Budget alternative: DDRNet-23-slim**
- 5.7M params — lightest CNN option
- MIT license
- torch2trt support documented
- Risk: Similar to PIDNet regarding NVIDIA ecosystem integration

### 7.2 Resolution Selection

For a litter-picking robot navigating outdoor environments:
- **Minimum viable:** 512x512 (sufficient for terrain classification at 2-5m range)
- **Recommended:** 640x480 or 768x512 (better obstacle boundary detection)
- **Maximum on Jetson:** 1024x512 (highest detail but may reduce FPS below target)
- **Target FPS:** 15-30 FPS minimum for safe navigation at walking speed (1-2 m/s)

### 7.3 Deployment Pipeline

```
Phase 1: Data Collection & Annotation (2-4 weeks)
├── Collect 500+ terrain images from target deployment environments
├── Auto-annotate with Mask2Former (offline, GPU server)
├── Manual correction of auto-annotations
├── Merge with RUGD + RELLIS-3D (remap to 8 classes)
└── Create train/val/test splits (70/15/15)

Phase 2: Model Training (1-2 weeks)
├── SegFormer-B0 fine-tuning via HuggingFace or TAO
├── PIDNet-S fine-tuning as backup
├── Hyperparameter search (learning rate, augmentation)
├── Evaluate on test set (target: >70% mIoU on 8 classes)
└── Compare models on accuracy vs speed

Phase 3: Edge Deployment (1-2 weeks)
├── Export best model to ONNX
├── Convert to TensorRT FP16 engine on Jetson
├── Benchmark FPS, latency, memory usage
├── INT8 quantization experiment (if FP16 meets FPS target, skip)
├── Integrate with Isaac ROS or custom ROS 2 node
└── End-to-end test: camera -> segmentation -> costmap -> navigation

Phase 4: Field Testing (ongoing)
├── Deploy on robot in target environments
├── Collect failure cases for active learning
├── Retrain periodically with expanded dataset
└── Monitor inference latency in production
```

### 7.4 Memory Budget on Jetson Orin Nano Super (8GB)

| Component | Estimated Memory |
|-----------|-----------------|
| OS + ROS 2 runtime | ~1.5 GB |
| Camera pipeline (1080p) | ~0.3 GB |
| SegFormer-B0 TensorRT (FP16) | ~0.2 GB |
| Detection model (YOLO26n) | ~0.3 GB |
| Navigation stack | ~0.5 GB |
| SLAM (if running) | ~1.0 GB |
| Headroom / buffers | ~1.2 GB |
| **Total** | **~5.0 GB** |
| **Available** | **8.0 GB** |
| **Margin** | **3.0 GB (37.5%)** |

SegFormer-B0 at 3.8M params is extremely memory-efficient. The full perception + navigation stack should fit comfortably within the 8GB budget.

---

## 8. References

1. Xie, E. et al. "SegFormer: Simple and Efficient Design for Semantic Segmentation with Transformers." NeurIPS 2021. https://arxiv.org/abs/2105.15203
2. Xu, J. et al. "PIDNet: A Real-time Semantic Segmentation Network Inspired by PID Controllers." CVPR 2023. https://arxiv.org/abs/2206.02066
3. Hong, Y. et al. "Deep Dual-resolution Networks for Real-time and Accurate Semantic Segmentation of Road Scenes." IEEE T-ITS 2023. https://arxiv.org/abs/2101.06085
4. Yu, C. et al. "BiSeNet V2: Bilateral Network with Guided Aggregation for Real-time Semantic Segmentation." IJCV 2021. https://arxiv.org/abs/2004.02147
5. Peng, J. et al. "PP-LiteSeg: A Superior Real-Time Semantic Segmentation Model." 2022. https://arxiv.org/abs/2204.02681
6. Jain, J. et al. "OneFormer: One Transformer to Rule Universal Image Segmentation." CVPR 2023. https://arxiv.org/abs/2211.06220
7. Cheng, B. et al. "Masked-attention Mask Transformer for Universal Image Segmentation." CVPR 2022. https://arxiv.org/abs/2112.01527
8. Zhao, X. et al. "Fast Segment Anything." 2023. https://arxiv.org/abs/2306.12156
9. Xiong, Y. et al. "EfficientSAM: Leveraged Masked Image Pretraining for Efficient Segment Anything." CVPR 2024. https://arxiv.org/abs/2312.00863
10. Wigness, M. et al. "RUGD Dataset for Autonomous Navigation." IROS 2019. http://rugd.vision/
11. Jiang, P. et al. "RELLIS-3D Dataset: Data, Benchmarks and Analysis." 2021. https://arxiv.org/abs/2011.12954
12. Peter, J. et al. "The GOOSE Dataset for Perception in Unstructured Environments." 2024. https://arxiv.org/abs/2310.16788
13. Guan, R. et al. "GANav: Efficient Terrain Segmentation for Robot Navigation." IEEE RA-L 2022. https://arxiv.org/abs/2103.04233
14. Vidanapathirana, K. et al. "WildScenes: A Benchmark for 2D and 3D Semantic Segmentation in Natural Environments." IJRR 2024.
15. NVIDIA Isaac ROS Image Segmentation. https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_segmentation/
16. NVIDIA TAO Toolkit — SegFormer. https://docs.nvidia.com/tao/tao-toolkit/text/tao_deploy/segformer.html
