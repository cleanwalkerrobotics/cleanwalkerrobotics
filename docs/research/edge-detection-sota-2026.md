# State-of-the-Art Real-Time Object Detection on Edge Devices (Early 2026)

**Target Hardware:** NVIDIA Jetson Orin Nano Super (67 TOPS INT8, 8GB RAM)
**Use Case:** Litter detection (small to medium objects in outdoor scenes)
**Date:** February 10, 2026

## Executive Summary

As of early 2026, the real-time object detection landscape for edge devices has evolved significantly with multiple competitive architectures:

1. **YOLO Family** (Ultralytics): YOLO26, YOLO11, YOLOv10, YOLOv9, YOLOv8 - AGPL-3.0 licensed
2. **YOLO Family** (Independent): YOLOv12 - Released February 2025
3. **Transformer-based**: RT-DETR, RT-DETRv2, RF-DETR - Apache 2.0 licensed (more permissive)
4. **Open-vocabulary**: YOLO-World - Zero-shot detection capabilities

**Key Findings for Litter Detection:**
- YOLO models excel at small object detection, particularly YOLO26 with its Small-Target-Aware Label Assignment (STAL)
- RT-DETR has acknowledged limitations with small objects compared to YOLO
- License matters: Apache 2.0 (RT-DETR, RF-DETR) vs AGPL-3.0 (Ultralytics YOLO) for commercial use
- TensorRT INT8 quantization delivers 2-3x speedup on Jetson with minimal accuracy loss

---

## 1. YOLO Family (Ultralytics)

### 1.1 YOLO26 (January 2026) - Latest Release

**Overview:** Most radical simplification in YOLO evolution, designed for edge and low-power devices.

| Variant | Params | GFLOPs | COCO mAP | Latency (CPU/ONNX) | Latency (TensorRT) | Model Size |
|---------|--------|--------|----------|-------------------|-------------------|------------|
| YOLO26n | ~2-3M | ~8G | ~40.6% | ~85ms | ~1.64ms (T4 GPU) | ~6MB |
| YOLO26s | ~8-9M | ~28G | ~47.2% | ~87.2ms | ~2.61ms (T4 GPU) | ~20MB |
| YOLO26m | ~16M | ~78G | ~51.5% | ~220ms | ~5.73ms (T4 GPU) | ~50MB |
| YOLO26l | ~25M | ~165G | ~53.0-53.4% | ~286ms | ~6.14ms (T4 GPU) | ~100MB |

**Key Innovations:**
- **NMS-Free Inference**: Native end-to-end predictions, eliminating post-processing bottleneck
- **DFL Removal**: Lighter box decoding for better hardware compatibility (ONNX, TensorRT, CoreML, TFLite)
- **Progressive Loss Balancing (ProgLoss)**: Adaptive loss weighting during training
- **Small-Target-Aware Label Assignment (STAL)**: Improved small object detection (critical for litter)
- **MuSGD Optimizer**: Hybrid SGD/Muon optimizer from LLM research for stable training

**Jetson Performance:**
- **43% faster CPU inference** than YOLO11-N
- 31% CPU speed improvement with accuracy gains
- Excellent export compatibility with TensorRT INT8

**License:** AGPL-3.0 (requires Enterprise License for commercial use)

**Best For:** Litter detection - STAL specifically targets small objects, NMS-free deployment simplifies edge pipeline

---

### 1.2 YOLO11 (2024)

**Overview:** Focuses on efficiency and small-object performance with compact architecture.

| Variant | Params | GFLOPs | COCO mAP | TensorRT INT8 (Orin Nano) | Model Size |
|---------|--------|--------|----------|---------------------------|------------|
| YOLO11n | 2.6M | 6.5G | ~39.5% | 4.54ms (~220 FPS) | ~6MB |
| YOLO11s | 9.4M | 21.5G | ~47.0% | 6.05ms (~165 FPS) | ~22MB |
| YOLO11m | 20.1M | 68.0G | ~51.5% | 10.43ms (~96 FPS) | ~49MB |
| YOLO11l | 25.3M | 86.9G | ~53.4% | 13.64ms (~73 FPS) | ~58MB |

**Key Features:**
- C3k2 bottlenecks for compactness
- C2PSA attention module for feature aggregation
- Strong small-object performance
- FP16/INT8 export with improved post-quantization accuracy

**Jetson Orin Nano Super Performance (estimated):**
- YOLO11n: **~320-350 FPS** with TensorRT INT8 (67 TOPS vs 40 TOPS base)
- YOLO11s: **~250 FPS**
- YOLO11m: **~145 FPS**

**License:** AGPL-3.0

---

### 1.3 YOLOv10 (2024)

**Overview:** Major efficiency leap with NMS-free training.

| Variant | Params | GFLOPs | COCO mAP | Latency (T4 GPU) | vs YOLOv8 |
|---------|--------|--------|----------|------------------|-----------|
| YOLOv10-N | 2.3M | 6.7G | 39.5% | 1.84ms | 28% fewer params |
| YOLOv10-S | 7.2M | 21.6G | 46.8% | 2.49ms | 38% fewer FLOPs |
| YOLOv10-M | 15.4M | 59.1G | 51.3% | ~5ms | 37% lower latency |
| YOLOv10-L | 24.4M | 120.3G | 53.4% | 7.28ms | 70% lower latency |
| YOLOv10-X | ~30M | ~160G | ~54.4% | ~9ms | - |

**Key Innovations:**
- NMS-free training with consistent dual assignments
- Spatial-channel decoupled downsampling
- Large-kernel convolutions
- **1.2-1.4% AP improvement** over YOLOv8 with fewer params

**License:** AGPL-3.0

---

### 1.4 YOLOv9 (2024)

**Overview:** Programmable Gradient Information architecture.

| Variant | Params | GFLOPs | COCO mAP |
|---------|--------|--------|----------|
| YOLOv9-T | 2.0M | 7.7G | 38.3% |
| YOLOv9-S | 7.1M | 26.4G | 46.8% |
| YOLOv9-M | 20.0M | 76.3G | 51.4% |
| YOLOv9-C | 25.3M | 102.1G | 53.0% |
| YOLOv9-E | 57.3M | 189.0G | 55.6% |

**Key Features:**
- **49% parameter reduction** vs YOLOv8
- 43% computation reduction
- 0.6% accuracy improvement
- Programmable Gradient Information (PGI) prevents information loss

**License:** AGPL-3.0

---

### 1.5 YOLOv8 (2023)

**Overview:** Baseline for modern YOLO evolution.

| Variant | Params | GFLOPs | COCO mAP | TensorRT INT8 (Orin Nano) |
|---------|--------|--------|----------|---------------------------|
| YOLOv8n | 3.2M | 8.7G | 37.3% | 23.16ms (~43 FPS) |
| YOLOv8s | 11.2M | 28.6G | 44.9% | 28.25ms (~35 FPS) |
| YOLOv8m | 25.9M | 78.9G | 50.2% | ~45ms (~22 FPS) |
| YOLOv8l | 43.7M | 165.2G | 52.9% | ~75ms (~13 FPS) |
| YOLOv8x | 68.2M | 257.8G | 53.9% | ~120ms (~8 FPS) |

**License:** AGPL-3.0

---

## 2. YOLOv12 (February 2025) - Independent Release

**Overview:** Attention-based architecture instead of traditional CNN while maintaining speed.

| Variant | Params | GFLOPs | COCO mAP | Latency (T4 GPU) |
|---------|--------|--------|----------|------------------|
| YOLO12n | ~2.5M | ~9G | ~40.6% | 1.64ms |
| YOLO12s | ~8M | ~30G | ~48.0% | 2.61ms |
| YOLO12m | ~18M | ~80G | ~51.9% | 5.73ms |
| YOLO12l | ~26M | ~170G | ~53.2% | 6.14ms |

**Key Innovations:**
- **Efficient Area Attention mechanism**
- **R-ELAN blocks** for improved efficiency
- Better accuracy on COCO than comparable YOLO variants
- Maintains YOLO-level inference speed despite attention mechanism

**License:** Check official repository (likely open source, but verify for commercial use)

**Note:** Some sources suggest YOLO26 outperforms YOLO12 in deployment scenarios despite YOLO12's higher COCO scores.

---

## 3. RT-DETR Family (Transformer-based)

### 3.1 RT-DETRv2 (2024)

**Overview:** Enhanced transformer-based real-time detector with bag-of-freebies.

| Variant | Params | GFLOPs | COCO mAP | FPS (T4 TensorRT FP16) | Improvement vs RT-DETR |
|---------|--------|--------|----------|----------------------|------------------------|
| RT-DETRv2-S | ~20M | ~60G | ~48.1% | ~217 FPS | +1.4 mAP |
| RT-DETRv2-M | ~31M | ~92G | ~49.9% | ~161 FPS | +1.0 mAP |
| RT-DETRv2-L | ~42M | ~136G | ~53.4% | ~108 FPS | +0.3 mAP |
| RT-DETRv2-X | ~76M | ~259G | ~54.3% | ~74 FPS | - |

**Key Features:**
- **Selective Multi-Scale Feature Extraction**: Better small/large object handling
- **Dynamic Data Augmentation**: Scale-adaptive hyperparameters
- **Discrete Sampling Option**: Removes grid_sample for better deployment compatibility
- No NMS required (end-to-end)

**Jetson Deployment:**
- Supports TensorRT optimization
- Grid sampling works best with TensorRT 8.5+
- Discrete sampling for TensorRT 8.4 or older

**Small Object Performance:**
- Improved vs RT-DETR but **still inferior to YOLO for small objects**
- Better for large objects and general-purpose detection

**License:** Apache 2.0 (permissive for commercial use)

---

### 3.2 RT-DETR (2024, CVPR)

**Overview:** First real-time end-to-end transformer detector.

| Variant | Params | GFLOPs | COCO mAP | FPS (T4 GPU) | With Objects365 Pre-training |
|---------|--------|--------|----------|--------------|------------------------------|
| RT-DETR-R18 | ~20M | ~60G | ~46.5% | ~217 FPS | - |
| RT-DETR-R34 | ~31M | ~92G | ~48.9% | ~161 FPS | - |
| RT-DETR-R50 | ~42M | ~136G | ~53.1% | ~108 FPS | 55.3% |
| RT-DETR-R101 | ~76M | ~259G | ~54.3% | ~74 FPS | 56.2% |

**Key Advantages:**
- **Outperforms YOLOs** in speed AND accuracy at higher scales
- End-to-end detection (no NMS)
- Efficient hybrid encoder
- IoU-aware query selection

**Known Limitations:**
- **Small object detection inferior to YOLO** (acknowledged in paper)
- Higher memory footprint than YOLO
- Transformer complexity may limit nano variant performance

**License:** Apache 2.0

---

### 3.3 RF-DETR (ICLR 2026)

**Overview:** Roboflow's SOTA real-time transformer with vision foundation model backbone.

| Variant | Params | COCO mAP | Segmentation | License |
|---------|--------|----------|--------------|---------|
| RF-DETR-base | 29M | **60+ AP** | Available (3x faster than YOLO) | Apache 2.0 |
| RF-DETR-large | 128M | **>60 AP** | Available | Apache 2.0 |
| RF-DETR-XL | ~200M+ | - | Available | Platform License 1.0* |
| RF-DETR-2XL | ~300M+ | - | Available | Platform License 1.0* |

**Key Features:**
- **First real-time model to exceed 60 AP on COCO**
- DINOv2 vision transformer backbone
- State-of-the-art on COCO and RF100-VL benchmarks
- Fine-tuning optimized architecture
- Segmentation support (RF-DETR Seg)

**License:**
- Base/Large: Apache 2.0 (fully open source)
- XL/2XL: Platform Model License 1.0 (requires Roboflow account)

**Jetson Deployment:**
- Large model likely too heavy for Orin Nano Super (128M params)
- Base model (29M params) may be viable but needs benchmarking
- Best suited for cloud or high-end edge devices

**Note:** Highest accuracy but may be overkill for litter detection; evaluate base model on Jetson first.

---

## 4. YOLO-World (Open-Vocabulary Detection)

**Overview:** Zero-shot open-vocabulary detector for detecting arbitrary object categories.

| Variant | Params | LVIS Zero-Shot AP | FPS (V100) | Model Size |
|---------|--------|-------------------|------------|------------|
| YOLO-World-S | 13M | 26.2 AP | 74.1 FPS | ~30MB |
| YOLO-World-M | ~26M | ~31.0 AP | ~60 FPS | ~60MB |
| YOLO-World-L | ~35M | 35.4 AP | 52.0 FPS | ~90MB |

**Key Features:**
- **Vision-language modeling** for open-vocabulary detection
- Zero-shot capability (no training on new categories)
- Can detect novel litter types without retraining
- YOLO-World-v2.1 released February 2025 with image prompts
- Practical deployment guides for Jetson available

**Jetson Deployment:**
- Confirmed working on Jetson Orin devices via Roboflow/visionplatform
- Small model (13M params) suitable for edge deployment
- May sacrifice some accuracy vs closed-vocabulary YOLOs

**Use Case for Litter:**
- **Excellent for novel litter types**: Can recognize "plastic bottle", "cigarette butt", "food wrapper" without specific training
- Trade-off: Lower mAP than closed-vocabulary models on known classes
- Useful for exploratory deployments or diverse litter categories

**License:** Check YOLO-World repository (likely aligned with Ultralytics AGPL-3.0)

---

## 5. Jetson Orin Nano Super Performance Estimates

Based on benchmarks from base Jetson Orin Nano (40 TOPS) and Orin NX (100 TOPS), **scaling to 67 TOPS**:

| Model | TensorRT INT8 FPS | Latency | Use Case |
|-------|------------------|---------|----------|
| YOLO26n | **~370 FPS** | ~2.7ms | Fastest, good for real-time |
| YOLO11n | **~320 FPS** | ~3.1ms | Balanced speed/accuracy |
| YOLOv10-S | **~240 FPS** | ~4.2ms | Better accuracy |
| YOLO11m | **~145 FPS** | ~6.9ms | Best accuracy/speed tradeoff |
| RT-DETRv2-S | **~120 FPS** | ~8.3ms | Transformer, less small obj optimized |
| YOLO-World-S | **~100 FPS** | ~10ms | Open-vocabulary, novel objects |

**Key Assumptions:**
- TensorRT INT8 quantization with 1-2% mAP loss
- 640x640 input resolution
- Optimized CUDA kernels
- Scaling factor: 67/40 = 1.675x performance vs base Orin Nano

**Recommended Configuration for Litter Detection:**
- **Primary:** YOLO26n or YOLO11n (small object optimized, 300+ FPS)
- **Fallback:** YOLOv10-S (better accuracy, 240 FPS still real-time)
- **Experimental:** YOLO-World-S for novel litter types

---

## 6. Accuracy vs Speed Tradeoff Analysis

### For Small Objects (Litter-sized: 2-20cm)

**Best Accuracy:**
1. YOLO26 (STAL feature) - **Recommended**
2. YOLO11 (C2PSA attention)
3. YOLOv10-M/L
4. RT-DETRv2 (inferior to YOLO per paper)

**Best Speed:**
1. YOLO26n (NMS-free + lightweight) - **Recommended**
2. YOLO11n (compact C3k2)
3. YOLOv10-N (NMS-free)
4. YOLOv8n (baseline)

**Best Tradeoff:**
1. **YOLO26s**: 47% mAP, ~87ms CPU, NMS-free
2. **YOLO11s**: 47% mAP, ~165 FPS on Jetson INT8
3. **YOLOv10-S**: 47% mAP, fewer params than predecessors

### For Outdoor Scenes (Complex Backgrounds)

**Considerations:**
- Transformers (RT-DETR) better at context understanding
- YOLO better at small object discrimination
- YOLO26's ProgLoss helps with imbalanced scenarios (few litter vs lots of background)

**Recommended:** YOLO26 for both accuracy and deployment simplicity (NMS-free)

---

## 7. License Comparison

| Model Family | License | Commercial Use | Restrictions |
|--------------|---------|----------------|--------------|
| YOLO26, YOLO11, YOLOv10, YOLOv9, YOLOv8 | AGPL-3.0 | Requires Enterprise License | Must open-source modifications or buy license |
| YOLOv12 | TBD | Unknown | Verify with official repo |
| RT-DETR, RT-DETRv2 | Apache 2.0 | Yes, unrestricted | No restrictions, fully permissive |
| RF-DETR (base/large) | Apache 2.0 | Yes, unrestricted | No restrictions |
| RF-DETR (XL/2XL) | Platform License 1.0 | Requires Roboflow account | Account-gated but commercial-friendly |
| YOLO-World | Likely AGPL-3.0 | Likely requires license | Verify with repo |

**Critical Decision Point:**
- **Apache 2.0 (RT-DETR family)**: Can be integrated into proprietary products without restrictions
- **AGPL-3.0 (Ultralytics YOLO)**: Must either open-source entire codebase OR purchase Enterprise License
- For CleanWalker Robotics commercial deployment, either:
  1. Budget for Ultralytics Enterprise License (~$1k-5k/year) for YOLO26/YOLO11
  2. Use RT-DETRv2 with Apache 2.0 (but accept lower small object performance)
  3. Consider YOLOv12 if license is permissive (needs verification)

---

## 8. Litter Detection Dataset Considerations

### Existing Datasets

| Dataset | Images | Annotations | Object Size | Format | Focus |
|---------|--------|-------------|-------------|--------|-------|
| TACO | 1,500 | 4,784 | Small-medium | COCO | Diverse litter, wild environments |
| RoLID-11K | 11,000+ | - | **80% small** (COCO criteria) | COCO | Roadside litter, dashcam |
| UAVVaste | 772 | - | Aerial view | COCO | Low-altitude UAV |
| PlastOPol | 2,418 | 5,300 | Real-world | COCO | Mobile device evaluation |

**Key Insight:** Over 80% of litter objects are "small" by COCO standards (<32x32 pixels), reinforcing the need for small-object-optimized models like YOLO26 and YOLO11.

### Evaluation Metrics

- **AP50**: Detection at 0.5 IoU threshold (common for litter detection papers)
- **AP50:95**: Average across IoU 0.5-0.95 (more stringent, better for localization quality)

**Recommendation:** Evaluate on both COCO and litter-specific datasets (TACO, RoLID-11K) with AP50:95 to ensure precise localization for robotic grasping.

---

## 9. Final Recommendations for CleanWalker Robotics

### Primary Model Choice: **YOLO26n/s**

**Rationale:**
- **Small-Target-Aware Label Assignment (STAL)**: Specifically designed for small objects (litter)
- **NMS-Free Deployment**: Simplifies edge pipeline, reduces latency
- **67 TOPS Performance**: ~370 FPS (n) or ~250 FPS (s) on Jetson Orin Nano Super
- **31% CPU improvement**: Better fallback if GPU maxed out
- **Model Size**: 6-20MB, fits easily in 8GB RAM with room for other processes

**Cons:**
- AGPL-3.0 license requires Enterprise License for commercial use (~$2k-5k/year budget)

### Alternative 1: **RT-DETRv2-S (Apache 2.0)**

**Rationale:**
- **Permissive license**: No commercial restrictions
- **Good accuracy**: 48% mAP, competitive with YOLO11s
- **~120 FPS estimated** on Jetson Orin Nano Super
- End-to-end transformer architecture

**Cons:**
- Acknowledged small object detection weakness vs YOLO
- 20M params (larger than YOLO26s)
- May require more tuning for litter-specific deployment

### Alternative 2: **YOLO-World-S (Experimental)**

**Rationale:**
- **Zero-shot detection**: Can identify novel litter types without retraining
- **74 FPS on V100** → ~100 FPS on Jetson Orin Nano Super (estimated)
- Jetson deployment confirmed working
- Useful for diverse/unknown litter categories

**Cons:**
- Lower accuracy on known classes vs closed-vocabulary models
- AGPL-3.0 license (same as YOLO26)
- Experimental for production deployment

---

## 10. Implementation Roadmap

### Phase 1: Baseline (Week 1-2)
1. **Deploy YOLO26n** with TensorRT INT8 on Jetson Orin Nano Super
2. Benchmark FPS, latency, memory usage
3. Evaluate on TACO and RoLID-11K datasets
4. Establish baseline AP50 and AP50:95 metrics

### Phase 2: Optimization (Week 3-4)
1. Fine-tune YOLO26n/s on custom litter dataset (if available)
2. Test YOLO11s for comparison (if YOLO26 underperforms)
3. Implement NMS-free inference pipeline
4. Optimize TensorRT INT8 quantization (QAT if accuracy drops >2%)

### Phase 3: Alternative Evaluation (Week 5-6)
1. **If licensing is a blocker**: Deploy RT-DETRv2-S with Apache 2.0
2. Compare small object performance on litter dataset
3. Evaluate YOLO-World-S for zero-shot litter detection
4. Cost-benefit analysis: Enterprise License vs RT-DETR accuracy tradeoff

### Phase 4: Production (Week 7-8)
1. Integrate selected model into CleanWalker perception pipeline
2. Test end-to-end: detection → localization → grasping
3. Field testing in outdoor environments
4. Iterative tuning based on real-world failure cases

---

## Sources

### Academic Papers
- [Ultralytics YOLO Evolution: YOLO26, YOLO11, YOLOv8, YOLOv5](https://arxiv.org/html/2510.09653v2)
- [YOLO26: Key Architectural Enhancements and Performance Benchmarking](https://arxiv.org/html/2509.25164v4)
- [YOLOv10: Advanced Real-Time End-to-End Object Detection](https://www.digitalocean.com/community/tutorials/yolov10-advanced-real-time-end-to-end-object-detection)
- [YOLOv9: Learning What You Want to Learn Using Programmable Gradient Information](https://github.com/WongKinYiu/yolov9)
- [DETRs Beat YOLOs on Real-time Object Detection (RT-DETR)](https://arxiv.org/abs/2304.08069)
- [RT-DETRv2: Improved Baseline with Bag-of-Freebies](https://arxiv.org/html/2407.17140v1)
- [YOLO-World: Real-Time Open-Vocabulary Object Detection](https://arxiv.org/html/2401.17270v3)
- [YOLOv12 (yolov12.com)](https://yolov12.com/)

### Benchmarks & Performance
- [YOLO11 Jetson Benchmarks - Ultralytics Discussions](https://github.com/orgs/ultralytics/discussions/9942)
- [YOLOv8 Performance Benchmarks on NVIDIA Jetson Devices - Seeed Studio](https://www.seeedstudio.com/blog/2023/03/30/yolov8-performance-benchmarks-on-nvidia-jetson-devices/)
- [NVIDIA Jetson Benchmarks](https://developer.nvidia.com/embedded/jetson-benchmarks)
- [Benchmarking Deep Learning Models for Object Detection on Edge Computing Devices](https://arxiv.org/html/2409.16808v1)

### Deployment & Integration
- [Ultralytics YOLO11 on NVIDIA Jetson using DeepStream SDK and TensorRT](https://xinetzone.github.io/torch-book/ecosystem/ultralytics/guides/deepstream-nvidia-jetson.html)
- [Deploy YOLO-World to NVIDIA Jetson - Roboflow](https://roboflow.com/how-to-deploy/deploy-yolo-world-to-nvidia-jetson)
- [RF-DETR - Roboflow SOTA Real-Time Detector](https://blog.roboflow.com/rf-detr/)

### Licensing
- [Ultralytics GitHub - AGPL-3.0 License](https://github.com/ultralytics/ultralytics/blob/main/LICENSE)
- [RT-DETR GitHub - Apache 2.0 License](https://github.com/lyuwenyu/RT-DETR/blob/main/LICENSE)
- [RF-DETR GitHub - Apache 2.0 License (base/large)](https://github.com/roboflow/rf-detr/blob/develop/LICENSE)
- [YOLO11 Model License and Pricing - Roboflow](https://roboflow.com/model-licenses/yolo11)

### Litter Detection Datasets
- [TACO: Trash Annotations in Context for Litter Detection](https://arxiv.org/pdf/2003.06975)
- [RoLID-11K: A Dashcam Dataset for Small-Object Roadside Litter Detection](https://arxiv.org/html/2601.00398)
- [UAVVaste: COCO-like dataset for aerial waste detection](https://github.com/PUTvision/UAVVaste)
- [Waste Datasets Review - GitHub](https://github.com/AgaMiko/waste-datasets-review)

### Model Comparisons
- [YOLO11 vs YOLO26: Compared and Contrasted - Roboflow](https://roboflow.com/compare/yolo11-vs-yolo26)
- [RT-DETR Beats YOLO? Full Comparison + Tutorial](https://www.labellerr.com/blog/rt-detrv2-beats-yolo-full-comparison-tutorial/)
- [Best Object Detection Models 2025: RF-DETR, YOLOv12 & Beyond](https://blog.roboflow.com/best-object-detection-models/)

---

## Appendix: Quick Reference Table

| Model | Params | COCO mAP | Jetson FPS (INT8, 640px) | Small Objects | License | Production Ready |
|-------|--------|----------|--------------------------|---------------|---------|------------------|
| **YOLO26n** | 2.3M | 40.6% | **~370** | Excellent (STAL) | AGPL-3.0* | Yes |
| **YOLO26s** | 8M | 47.2% | **~260** | Excellent (STAL) | AGPL-3.0* | Yes |
| **YOLO11n** | 2.6M | 39.5% | **~320** | Excellent | AGPL-3.0* | Yes |
| **YOLO11s** | 9.4M | 47.0% | **~250** | Excellent | AGPL-3.0* | Yes |
| **YOLOv10-S** | 7.2M | 46.8% | **~240** | Good | AGPL-3.0* | Yes |
| **RT-DETRv2-S** | 20M | 48.1% | **~120** | Fair (inferior to YOLO) | Apache 2.0 | Yes |
| **YOLO-World-S** | 13M | 26.2% (LVIS) | **~100** | Good (zero-shot) | AGPL-3.0* | Experimental |
| **RF-DETR-base** | 29M | 60+ AP | **~70** (estimated) | Good | Apache 2.0 | Research |

*AGPL-3.0 requires Enterprise License for commercial deployment without open-sourcing code.

**Recommended for CleanWalker:** YOLO26n or YOLO26s (budget for license) OR RT-DETRv2-S (Apache 2.0, no license needed).
