# Perception Demo Options — Real Litter Detection

**Date:** 2026-02-10
**Status:** Research complete, PoC built
**Author:** cw-software team

## Problem

Our current litter detection demo at `/demos/litter-detection` uses mock data with hardcoded bounding boxes. We need a working demo that runs real inference on actual images to show prospects.

## Options Evaluated

### Option 1: Pre-trained YOLO with COCO Classes

**Approach:** Use `ultralytics` YOLOv8/YOLO11 with the default COCO-trained weights. COCO's 80 classes include some litter-adjacent items.

**Relevant COCO classes for litter:**
| COCO ID | Class | Litter Relevance |
|---------|-------|-----------------|
| 39 | bottle | High — beverage bottles |
| 41 | cup | High — disposable cups |
| 40 | wine glass | Medium — glass debris |
| 42–44 | fork/knife/spoon | Low — food waste |
| 45 | bowl | Medium — food containers |

**Pros:**
- Free — no API costs, no GPU needed (CPU inference ~1-3s per image)
- Already a dependency (`ultralytics` in our requirements.txt)
- Works offline, can run on our server or in a Docker container
- Well-understood, battle-tested models

**Cons:**
- Only detects 6-8 litter-relevant classes out of 80
- Misses most real litter: cans, plastic bags, cigarette butts, wrappers, cardboard, food packaging
- No "trash" or "litter" class — only individual object types
- Not impressive for demos — detects a bottle but misses the bag next to it

**Feasibility:** High
**Cost:** Free
**Time to implement:** 2-4 hours
**Demo quality:** Low — too many misses on real litter

---

### Option 2: Fine-tune on TACO Dataset

**Approach:** Use our existing training pipeline (`ml/training/`) to fine-tune YOLOv8n on the TACO dataset with our 10 litter classes.

**Pros:**
- Best detection quality — trained specifically for litter
- Full control over classes, thresholds, model size
- Our pipeline is already built and ready
- Produces a `best.pt` weights file for offline inference

**Cons:**
- No GPU on our server (Hetzner CX32 = CPU only)
- CPU training would take 12-24 hours for 100 epochs
- Alternative: Google Colab free tier (limited sessions, T4 GPU ~2 hours)
- Alternative: Rent cloud GPU ($0.50-2/hr on Lambda, Vast.ai, RunPod)
- TACO dataset is small (~1500 images) — may need augmentation
- Requires downloading ~3GB of images first

**Feasibility:** Medium (blocked by GPU access)
**Cost:** $0 (Colab) to ~$5 (cloud GPU for 2-3 hours)
**Time to implement:** 4-8 hours including dataset prep + training
**Demo quality:** High — but only after training completes

---

### Option 3: Cloud Inference API (Replicate)

**Approach:** Use Replicate's hosted models via HTTP API. Two strong candidates:

#### Option 3a: YOLOv8s-WorldV2 (Open-Vocabulary YOLO) — RECOMMENDED

- **Model:** `ultralytics/yolov8s-worldv2` on Replicate
- **Key feature:** Open-vocabulary — accepts custom class names as free text
- **We can specify:** `"bottle, can, cup, plastic bag, cigarette butt, food wrapper, paper, cardboard, trash, litter"`
- **Returns:** Structured JSON with bounding boxes, class names, confidence scores
- **Cost:** ~$0.0001/sec (CPU), roughly $0.001-0.005 per image
- **Latency:** 2-5 seconds per image (sync mode with `Prefer: wait`)

#### Option 3b: Grounding DINO

- **Model:** `adirik/grounding-dino` on Replicate
- **Key feature:** Natural language queries for detection targets
- **Returns:** Clean JSON with bbox, label, confidence
- **Cost:** Slightly higher (GPU-based)
- **Better for:** Complex scene understanding, but slower

**Pros:**
- Works immediately — no training required
- Open-vocabulary means we detect exactly the litter categories we care about
- Structured JSON output — easy to integrate into web demo
- Cheap at demo scale (~100 images/month = ~$0.50)
- Returns annotated images AND bounding box data

**Cons:**
- Requires internet access and API key
- 2-5 second latency per image (not real-time)
- API dependency — if Replicate goes down, demo breaks
- Cost scales with usage (not a concern for demos, but matters for production)
- Open-vocabulary detection is less accurate than purpose-trained models

**Feasibility:** High
**Cost:** ~$0.001-0.005 per image
**Time to implement:** 2-3 hours
**Demo quality:** Good — detects litter-specific classes with reasonable accuracy

---

### Option 4: Client-Side TensorFlow.js / ONNX

**Approach:** Convert a YOLO model to TensorFlow.js or ONNX format and run inference directly in the browser.

**Pros:**
- Zero server cost — runs entirely in the user's browser
- No API dependency
- Privacy-friendly — images never leave the device
- Impressive technical demo ("runs on your phone!")

**Cons:**
- Need a trained model first (same GPU problem as Option 2)
- Model conversion is finicky (YOLO → ONNX → TF.js)
- Browser inference is slow (~5-15s on desktop, worse on mobile)
- Large model download for users (~10-30MB)
- Limited browser GPU support (WebGL/WebGPU inconsistencies)
- Most complex to implement and debug

**Feasibility:** Low-Medium (blocked by model availability)
**Cost:** Free at runtime
**Time to implement:** 8-16 hours
**Demo quality:** Medium — impressive concept but slow and unreliable

---

## Recommendation

### Immediate (this week): Option 3a — Replicate YOLOv8s-WorldV2

**Why:** Fastest path to a working demo we can show prospects. Open-vocabulary detection means we specify our exact litter classes without training. The PoC script is already built (see `ml/poc/replicate_detect.py`).

**Integration plan:**
1. PoC script takes image URL → returns JSON detections (DONE)
2. Next: Add a Next.js API route that proxies to Replicate
3. Update the web demo to call our API route with uploaded images
4. Display real bounding boxes on the uploaded image

### Medium-term (2-4 weeks): Option 2 — Fine-tune on TACO

**Why:** Best accuracy for our specific use case. Use Replicate demo now, train custom model when we have GPU access (Google Colab or cloud GPU rental for ~$5).

**Plan:**
1. Run `download_taco.sh` to prepare dataset
2. Train on Colab or rented GPU (2-3 hours)
3. Export `best.pt` → replace Replicate API with local inference
4. Optionally export to ONNX for Option 4

### Long-term: Option 4 — Client-side inference

**Why:** Best for production — zero server cost, works offline, impressive demo. But requires Option 2 first to produce the model, plus significant browser integration work.

---

## Proof of Concept

A working PoC script is available at `ml/poc/replicate_detect.py`. It:
- Takes an image URL as input
- Calls Replicate's YOLOv8s-WorldV2 with our 10 litter classes
- Returns structured JSON detections (class name, confidence, bounding box)
- Returns an annotated image URL

Usage:
```bash
export REPLICATE_API_TOKEN="your-token-here"
python ml/poc/replicate_detect.py --image "https://example.com/street-litter.jpg"
python ml/poc/replicate_detect.py --image "https://example.com/photo.jpg" --conf 0.15
```

### Test Results (2026-02-10)

Tested against Pexels stock photos of litter/trash:

| Image | Detections | Classes Found | Notes |
|-------|-----------|---------------|-------|
| Plastic bottles scene | 6 | paper cup (5), glass bottle (1) | Good coverage, low-confidence but valid boxes |
| Cardboard on ground | 2 | cardboard (0.57), paper cup (0.21) | Cardboard detected at high confidence |
| Bottles at bar | 2 | glass bottle (0.11), paper cup (0.11) | Low conf on close-up images |

**Key observations:**
- Open-vocabulary detection works — returns our custom class names ("paper cup", "glass bottle", "cardboard")
- Confidence scores are lower than a purpose-trained model (0.1-0.6 range vs 0.7+ for fine-tuned)
- Works best on clear, well-lit scenes with distinct litter objects
- Struggles with abstract/artistic photos and scenes where litter blends with background
- For a demo, this is good enough — prospects will see real bounding boxes on real images
- For production, fine-tuned model (Option 2) will be significantly better

### Output Format

Each detection contains:
```json
{
  "name": "paper cup",
  "class": 3,
  "confidence": 0.287,
  "box": { "x1": 657.96, "y1": 44.94, "x2": 705.86, "y2": 128.46 }
}
```

## Cost Projections

| Scenario | Monthly Images | Replicate Cost | Notes |
|----------|---------------|----------------|-------|
| Demo/sales | 50-100 | ~$0.50 | Negligible |
| Beta testing | 500-1000 | ~$5 | Still cheap |
| Production (per robot) | 10,000+ | ~$50+ | Switch to local model |

At production scale, we'd use the fine-tuned model (Option 2) running on the robot's edge compute, not a cloud API.
