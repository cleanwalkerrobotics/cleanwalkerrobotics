# Render Pipeline Research — Consistent Photorealistic Robot Renders

**Date:** 2026-02-10
**Status:** Research complete, recommended pipeline defined
**Author:** CW Software Team

---

## Problem Statement

Our current text-to-image pipeline (Seedream 4.5 via Replicate) cannot reliably produce our CW-1 robot to spec:

| Issue | Root Cause |
|-------|-----------|
| Bag roll axis renders front-to-back instead of left-to-right | Text prompts cannot enforce spatial orientation |
| Heads come out dog-like instead of boxy/flush sensor module | Diffusion model bias toward organic dog forms for quadrupeds |
| No precise dimensional control (4:1 body ratio not achievable) | Text-to-image has no geometric grounding |
| Each render is a lottery — 14 attempts for one passable result | No structural conditioning; pure text → pixel inference |

**Core insight:** Text prompts describe *what* but cannot enforce *where* or *how big*. We need geometric conditioning from our existing 3D model.

---

## Approaches Evaluated

### 1. Pure AI Text-to-Image (Current Pipeline)

**How it works:** Descriptive prompt → Seedream 4.5 / Flux → hope for the best.

| Factor | Assessment |
|--------|-----------|
| Setup time | 30 min (already done) |
| Per-render cost | ~$0.02-0.05 |
| Consistency | 3/10 — lottery per render |
| Quality | 8/10 — when it works, looks great |
| Dimensional control | 2/10 — no geometric grounding |
| Bag orientation accuracy | ~20% correct |

**Verdict:** Unsuitable for production renders. Good only for mood boards and initial concepts.

---

### 2. ControlNet Depth + Canny Conditioning

**How it works:** Render depth maps and edge maps from our 3D model → feed as ControlNet conditioning alongside text prompt → AI generates photorealistic image that follows the geometry.

- **Depth maps** preserve 3D spatial relationships, foreground/background, and overall shape
- **Canny edge maps** preserve sharp mechanical details (joints, panels, sensor housings)
- **Combined depth + canny** produces ~94% usable results vs. 68% for depth alone
- Best models: Flux Depth Pro, Z-Image Turbo ControlNet Union 2.0
- Available on Replicate, ComfyUI, fal.ai

| Factor | Assessment |
|--------|-----------|
| Setup time | 2-4 hours |
| Per-render cost | $0.02-0.15 |
| Consistency | 7/10 |
| Quality | 8/10 |
| Dimensional control | 7/10 |
| Bag orientation accuracy | ~85% (depth map enforces it) |

**Verdict:** Major improvement over pure text. Requires generating conditioning maps from 3D model first.

---

### 3. IP-Adapter + ControlNet

**How it works:** IP-Adapter uses CLIP Vision to embed a reference image of the robot → injects identity into diffusion process. Combined with ControlNet for structural conditioning.

- IP-Adapter handles "what" (robot identity, color, surface finish)
- ControlNet handles "where" (pose, structure, geometry)
- Can be combined with InstantStyle to separate style from structure
- Used in production by game studios (InnoGames, etc.)

| Factor | Assessment |
|--------|-----------|
| Setup time | 4-6 hours |
| Per-render cost | $0.05-0.15 |
| Consistency | 7/10 |
| Quality | 8/10 |
| Dimensional control | 7/10 |

**Verdict:** Good for identity preservation but adds complexity. ControlNet depth alone may be sufficient with our detailed 3D model.

---

### 4. LoRA Fine-Tuning + ControlNet Depth

**How it works:** Train a custom LoRA on 15-20 renders of our robot → the model "learns" CleanWalker's design language → combine with ControlNet depth for geometric accuracy.

**Training requirements:**
- 10-20 high-quality images from diverse angles (generate from 3D model)
- ~1,000-1,400 training steps
- ~2 minutes on Replicate (8xH100), cost ~$1.50-2.50
- Output: ~5MB LoRA weights file

**Services:**

| Service | Cost | Time | Notes |
|---------|------|------|-------|
| Replicate (Flux) | ~$1.50/training | 2 min | Easiest setup |
| fal.ai Flux LoRA Fast | $2/1000 steps | Fast | 10x faster |
| fal.ai Kontext Trainer | $2.50/1000 steps | Varies | Newer model |
| Civitai | ~$2 (2,000 Buzz) | Varies | Up to 10,000 steps |

| Factor | Assessment |
|--------|-----------|
| Setup time | 6-8 hours (including training image generation) |
| Per-render cost | $0.02-0.05 + $1.50 one-time training |
| Consistency | 8/10 |
| Quality | 9/10 |
| Dimensional control | 8/10 (with ControlNet) |

**Verdict:** Excellent consistency once trained. The LoRA encodes robot identity; ControlNet depth ensures geometry. Worth the setup investment.

---

### 5. Pure Blender Cycles Rendering

**How it works:** Import our Three.js GLB model into Blender → set up PBR materials, HDRI lighting, photorealistic environments → render with Cycles path tracer.

- Blender Cycles produces genuinely photorealistic results (path-traced global illumination)
- Full dimensional accuracy — the 3D model IS the render
- Fully scriptable via CLI: `blender -b scene.blend --python render_script.py`
- Automation tools: BlenderProc, AutoBlender, native Python API
- Can output depth maps, normal maps, and edge maps simultaneously

| Factor | Assessment |
|--------|-----------|
| Setup time | 8-16 hours (materials, lighting, 14 scene compositions) |
| Per-render cost | Free (GPU time only) |
| Consistency | 10/10 — deterministic |
| Quality | 9/10 — photorealistic with proper setup |
| Dimensional control | 10/10 — perfect by definition |
| Bag orientation accuracy | 100% — it's the 3D model |

**Limitations:**
- Environment/scene creation requires 3D art skill or purchased assets
- Achieving "final-pixel" photorealism for marketing requires significant material and lighting work
- Our current Three.js model uses simple primitives — will need material enhancement in Blender

**Verdict:** Perfect accuracy but significant artistic effort for marketing-quality environments. Best as foundation for hybrid approach.

---

### 6. HYBRID: Blender Base Render + AI Enhancement (RECOMMENDED)

**How it works:** Render the robot in Blender at each camera angle → use the renders as img2img base + depth/canny ControlNet conditioning → AI adds photorealistic detail, environmental integration, and material richness.

```
Three.js GLB ──► Blender Import ──► Cycles Render (14 angles)
                                         │
                                    ┌────┴────┐
                                    │         │
                              RGB render   Depth map + Canny edges
                                    │         │
                                    ▼         ▼
                              img2img     ControlNet
                              (denoise    (depth 0.8 +
                               0.3-0.4)   canny 0.5)
                                    │         │
                                    └────┬────┘
                                         │
                                    Flux Dev / FLUX.2
                                    + optional LoRA
                                         │
                                         ▼
                                  Photorealistic render
                                         │
                                         ▼
                                  AI Upscale to 4K+
                                  (ESRGAN / Krea AI)
```

| Factor | Assessment |
|--------|-----------|
| Setup time | 10-16 hours first time; ~2 hours for subsequent batches |
| Per-render cost | ~$0.10-0.20 (AI enhancement) |
| Total cost for 14 renders | ~$5-15 AI + Blender GPU time |
| Consistency | 9/10 |
| Quality | 10/10 — marketing-ready photorealism |
| Dimensional control | 9/10 |
| Bag orientation accuracy | ~98% (3D model enforces geometry) |

**Verdict:** Best overall. The 3D model guarantees geometric accuracy while AI adds the photorealistic "wow factor" that would take 10x longer in pure 3D.

---

### 7. Consistent Character Tools (Midjourney --cref, DALL-E, etc.)

| Tool | Consistency | Control | API Access | Notes |
|------|------------|---------|------------|-------|
| Midjourney --cref | 5/10 | 2/10 | No (Discord only) | General look only, no dimensions |
| GPT Image 1 | 5/10 | 3/10 | Yes | Better instruction following, still imprecise |
| FLUX.1 Kontext | 7/10 | 4/10 | Yes | Best reference-based consistency without training |
| Seedream 4.5 | 6/10 | 3/10 | Yes (Replicate) | Built-in multi-image coherence |

**Verdict:** These tools cannot enforce geometry. They maintain a general "vibe" but not exact proportions. Insufficient for our spec-critical needs.

---

## Comparison Summary

| Approach | Setup | $/Render | Consistency | Quality | Dim. Control | Overall |
|----------|-------|---------|-------------|---------|--------------|---------|
| Pure AI text-to-image | 30 min | $0.03 | 3/10 | 8/10 | 2/10 | Poor |
| ControlNet depth+canny | 2-4 hrs | $0.08 | 7/10 | 8/10 | 7/10 | Good |
| IP-Adapter + ControlNet | 4-6 hrs | $0.10 | 7/10 | 8/10 | 7/10 | Good |
| LoRA + ControlNet | 6-8 hrs | $0.03 | 8/10 | 9/10 | 8/10 | Very Good |
| Pure Blender Cycles | 8-16 hrs | Free | 10/10 | 9/10 | 10/10 | Great (but slow) |
| **Hybrid: Blender + AI** | **10-16 hrs** | **$0.15** | **9/10** | **10/10** | **9/10** | **Best** |
| Character tools (--cref) | 15 min | $0.01 | 5/10 | 8/10 | 2/10 | Poor |

---

## Recommended Pipeline: Hybrid 3D + AI Enhancement

### Why Hybrid Wins

1. **Dimensional accuracy:** The 3D model is the single source of truth. No AI hallucination of wrong proportions, wrong bag axis, or dog-like heads.
2. **Scene control:** Camera angles, robot orientation, and bag placement are precisely controlled in Blender.
3. **Photorealism:** AI enhancement adds micro-detail, environmental integration, and material realism that would take 10x longer to achieve in pure 3D.
4. **Consistency:** Same 3D model → identical robot in every render. AI only varies the scene/environment.
5. **Scalability:** Once set up, new scenes = position camera + run script. Minutes, not hours.
6. **Cost:** ~$5-15 for 14 renders vs. hiring a 3D artist for full Cycles scenes.

### We Already Have the Hard Part

Our Three.js 3D model (`apps/web/src/app/demos/3d-robot-viewer/`) defines:
- Complete robot geometry (body, legs, arm, gripper, bag system)
- Correct proportions (60×15×12cm body, 4:1 ratio)
- Correct bag roll axis (left-to-right)
- Correct head shape (boxy sensor module, not dog-like)
- Material colors and finishes
- All V2.3 spec details

This model can be exported to GLB and imported into Blender with one command.

---

## Step-by-Step Implementation Plan

### Phase 1: 3D Foundation (One-Time Setup, ~8 hours)

**Step 1.1: Export Three.js model to GLB**
- Add a GLB export script using Three.js `GLTFExporter`
- Export the complete robot (body + bag system + arm) as a single GLB file
- Verify geometry, materials, and hierarchy transfer correctly

**Step 1.2: Import to Blender and enhance materials**
- Import GLB into Blender (native glTF 2.0 support)
- Enhance PBR materials:
  - Matte olive-green body: roughness 0.75, metalness 0.1, subtle surface imperfections via normal map
  - Dark grey joints: roughness 0.6, metalness 0.25
  - Black rubber feet: roughness 0.95
  - Green LEDs: emissive material with bloom
- Add surface detail normal maps for injection-molded plastic appearance

**Step 1.3: Create lighting rig**
- HDRI environment lighting (download free HDRIs from Poly Haven)
- 3-point studio setup as base
- Per-scene environment-specific HDRI (park, city, night, studio)

**Step 1.4: Script 14 camera positions**
- Create a Python script that positions camera for each of the 14 renders:
  1. Hero park — 3/4 front view, golden hour
  2. Hero sidewalk — side profile, dynamic pose
  3. Hero fleet — aerial/drone view of 3 robots
  4. Detail gripper — close-up, macro lens
  5. Detail sensors — front face close-up
  6. Detail side profile — clean studio side view
  7. Detail charging dock — low angle
  8. Lifestyle city worker — medium shot with human
  9. Lifestyle night ops — dramatic low light
  10. Lifestyle before/after — split composition
  11. Tech exploded view — orthographic, white BG
  12. Tech dashboard — 3/4 view with UI overlay
  13. Component actuator — extreme close-up
  14. Component PCB — extreme close-up

**Step 1.5: Test render at low resolution**
- Cycles, 64 samples, 512x512
- Verify geometry, materials, and composition
- Iterate until base renders look correct

### Phase 2: Batch 3D Render (~2 hours)

**Step 2.1: Configure Cycles render settings**
```python
# Blender Python script
bpy.context.scene.render.engine = 'CYCLES'
bpy.context.scene.cycles.samples = 128  # Balance quality/speed
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 576  # 16:9
bpy.context.scene.cycles.device = 'GPU'
```

**Step 2.2: Render all outputs per scene**
```bash
blender -b robot-scenes.blend --python batch_render.py
```

Output per scene (56 total images):
- `{scene}_rgb.png` — Full color Cycles render
- `{scene}_depth.png` — Normalized depth map (for ControlNet)
- `{scene}_normal.png` — Normal map (for ControlNet)
- `{scene}_canny.png` — Canny edge detection (post-process)

**Step 2.3: Install ControlNet Blender addon (optional)**
- [controlnet-render-blender-addon](https://github.com/x6ud/controlnet-render-blender-addon) — one-click depth/normal/edge maps

### Phase 3: LoRA Training (Optional, ~30 minutes)

**Step 3.1: Prepare training images**
- Use the 14 RGB Blender renders as training data
- Add 6-8 additional angles for diversity (20 total)
- Clean white/grey backgrounds for training images

**Step 3.2: Train Flux LoRA on Replicate**
```bash
# Via Replicate API
curl -s -X POST https://api.replicate.com/v1/models/ostris/flux-dev-lora-trainer/versions/.../trainings \
  -H "Authorization: Bearer $REPLICATE_API_TOKEN" \
  -d '{
    "input": {
      "input_images": "<zip_url>",
      "trigger_word": "CLEANWALKER_CW1",
      "steps": 1200,
      "learning_rate": 0.0004
    }
  }'
```
- Cost: ~$1.50
- Time: ~2 minutes
- Output: LoRA weights file (~5MB)

**Step 3.3: Test LoRA**
- Generate 5 test images with trigger word + varied prompts
- Verify robot identity preservation
- Adjust training if needed

### Phase 4: AI Enhancement (~30 minutes for all 14)

**Step 4.1: Set up ComfyUI workflow (or use Replicate API)**

**Option A — Replicate API (simpler):**
```javascript
// Per render
const output = await replicate.run("black-forest-labs/flux-depth-pro", {
  input: {
    prompt: scenePrompt,
    control_image: depthMapUrl,      // Blender depth map
    image: rgbRenderUrl,             // Blender RGB render (img2img)
    strength: 0.35,                  // Low denoise to preserve structure
    control_strength: 0.8,           // Strong depth conditioning
    num_inference_steps: 28
  }
});
```

**Option B — ComfyUI workflow (more control):**
- Self-host on RunPod (~$0.50-1.00/hr for A100)
- Workflow: Load Model → ControlNet Depth (0.8) → ControlNet Canny (0.5) → IP-Adapter (reference image) → LoRA → KSampler → Upscale
- Save workflow as template for all 14 renders

**Step 4.2: Run enhancement pipeline**
- Process all 14 renders through the AI pipeline
- Review each output for quality and accuracy
- Re-run any that drift from spec (with adjusted denoise strength)

### Phase 5: Upscale + Final Polish (~15 minutes)

**Step 5.1: AI upscale to 4K+**
- Use ESRGAN (free, via Replicate) or Krea AI Enhancer
- Target: 3840x2160 (4K 16:9) minimum

**Step 5.2: Quality check**
- Verify bag roll axis orientation in all renders
- Verify head shape (boxy sensor module, not dog-like)
- Verify body proportions (4:1 length:width)
- Verify LED placement (upper legs only)
- Verify arm configuration (single, multi-joint, mechanical gripper)
- Check for any AI-introduced branding, text, or watermarks

**Step 5.3: Export final assets**
- Save to `apps/web/public/renders/` (replacing current AI-only renders)
- Maintain existing filenames for website compatibility

---

## Tools and Services Needed

### Required

| Tool | Purpose | Cost | Notes |
|------|---------|------|-------|
| **Blender 4.x** | 3D rendering, material setup, scene composition | Free | Install via `sudo snap install blender --classic` or download |
| **Replicate API** | AI enhancement (Flux Depth Pro) | ~$0.10-0.20/render | Already have account (Seedream pipeline) |
| **Three.js GLTFExporter** | Export robot model to GLB | Free | npm package, already in ecosystem |

### Recommended

| Tool | Purpose | Cost | Notes |
|------|---------|------|-------|
| **Poly Haven HDRIs** | Free photorealistic environment maps | Free | polyhaven.com, CC0 license |
| **Replicate Flux LoRA Trainer** | Train custom robot LoRA | ~$1.50 one-time | Optional but improves consistency |
| **ESRGAN (via Replicate)** | 4K upscaling | ~$0.01/image | Or use Krea AI |
| **controlnet-render-blender-addon** | Quick depth/normal/canny export from Blender | Free | GitHub addon |

### Optional (for advanced pipeline)

| Tool | Purpose | Cost | Notes |
|------|---------|------|-------|
| **ComfyUI on RunPod** | Full control over AI pipeline | ~$0.50-1.00/hr | More flexible than Replicate API |
| **Krea AI Enhancer** | One-click 3D → photorealistic enhancement | Pay-as-you-go | Up to 22K resolution |
| **fal.ai** | Alternative AI inference API | $0.021/megapixel | Good Flux support |

---

## Fastest Path to 14 Consistent Renders

If time is the priority, here is the minimum viable pipeline:

### Fast Track (~6-8 hours total)

1. **Add GLTFExporter to Three.js app** (1 hour)
   - Export robot model as GLB from browser

2. **Install Blender, import GLB** (30 min)
   - `sudo snap install blender --classic`
   - Import GLB, verify geometry

3. **Quick material + lighting pass** (2 hours)
   - Apply basic PBR materials matching our color spec
   - Download 4-5 HDRIs from Poly Haven for different scenes
   - Set up simple 3-point lighting

4. **Script 14 camera angles + batch render** (2 hours)
   - Python script with camera positions per scene
   - Render RGB + depth maps at 1024x576
   - Output: 28 images (14 RGB + 14 depth)

5. **AI enhance via Replicate API** (1 hour)
   - Script: for each scene, call Flux Depth Pro with RGB + depth map
   - Low denoise (0.3-0.4) to preserve 3D structure
   - Scene-specific prompts for environment details

6. **Upscale + QA** (30 min)
   - ESRGAN upscale to 4K
   - Visual quality check against V2.3 spec
   - Replace renders in `apps/web/public/renders/`

### What This Solves

| Previous Issue | How Hybrid Fixes It |
|---------------|-------------------|
| Bag roll axis wrong | 3D model has correct left-to-right axis → depth map enforces it |
| Dog-like heads | 3D model has boxy sensor module → depth map enforces shape |
| No dimensional control | 3D model is dimensionally exact → AI preserves geometry |
| Render lottery (14 attempts) | 3D base is deterministic → AI enhancement is predictable |

---

## References

- [ControlNet Render Blender Addon](https://github.com/x6ud/controlnet-render-blender-addon)
- [Flux Depth Pro on Replicate](https://replicate.com/black-forest-labs/flux-depth-pro)
- [BlenderProc Procedural Pipeline](https://github.com/DLR-RM/BlenderProc)
- [FLUX.1 Kontext](https://bfl.ai/models/flux-kontext)
- [Replicate Flux LoRA Trainer](https://replicate.com/docs/get-started/fine-tune-with-flux)
- [Poly Haven Free HDRIs](https://polyhaven.com/hdris)
- [Blender CLI Rendering Docs](https://docs.blender.org/manual/en/latest/advanced/command_line/render.html)
- [Three.js GLTFExporter](https://threejs.org/docs/#examples/en/exporters/GLTFExporter)
