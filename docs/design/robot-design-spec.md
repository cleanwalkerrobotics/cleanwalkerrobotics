# CleanWalker Robot — Definitive Visual Design Specification

**Version:** 2.2
**Date:** 2026-02-10
**Purpose:** Single source of truth for all renders, 3D models, and technical descriptions. Every visual asset MUST conform to this spec.

---

## 1. OVERALL FORM & PROPORTIONS

- **Type:** Quadrupedal robot, similar body proportions to Unitree Go2 / Boston Dynamics Spot — but NO competitor branding, NO competitor logos, and custom-attached modules (arm + bag system) that make it distinctly CleanWalker.
- **Aesthetic:** Modern industrial — smooth molded panels with softly rounded/chamfered edges. Think Spot's body quality meets Dyson's surface finish. NOT rough/gritty military, NOT toy-like. Clean, capable, professional.

### Dimensions & Ratios

| Measurement | Value | Ratio Reference |
|---|---|---|
| **Body length (L)** | ~45cm (nose to tail of body) | 1.0× (base unit) |
| **Body width (W)** | ~30cm | 0.67× body length |
| **Body height (H)** | ~15cm (body enclosure only) | 0.33× body length |
| **Total height (standing)** | ~55-60cm at top of body | ~3.7× body height |
| **Leg length** | ~40-45cm (hip to foot) | ~1.0× body length |
| **Upper leg segment** | ~20cm | 0.5× leg length |
| **Lower leg segment** | ~20cm | 0.5× leg length |
| **Arm height (extended)** | ~40cm above body top | ~0.9× body length |
| **Arm reach (forward)** | ~50cm from shoulder to gripper tip | ~1.1× body length |
| **Frame depth** | ~22cm (front-to-back) | 0.5× body length |
| **Frame width** | ~30cm (same as body width) | 1.0× body width |
| **Ground clearance** | ~35cm (bottom of body to ground) | ~0.78× body length |
| **Overall footprint** | ~45cm L × 35cm W (legs spread slightly wider than body) | — |

### Key Proportions (visual guide)
- The body is a wide, flat, horizontal slab — roughly **3:2:1** (L:W:H)
- Legs are approximately the same length as the body — the robot does NOT look stubby or squatted
- The arm is the tallest element, rising ~40cm above the body
- The bag system (frame + bag) extends ~22cm behind the body's rear edge
- Standing height at body top is about 3.5-4× the body thickness

## 2. BODY

- **Shape:** Horizontally elongated rectangular enclosure with softly rounded/chamfered edges and panel seams. Slightly convex top surface. Smooth, clean panel lines.
- **Color:** Dark matte olive-green (hex ~#3B4A3F). NOT pure black — visible green-grey undertone. This color is CORRECT and INTENTIONAL.
- **Finish:** Smooth matte to satin. No gloss, no rough texture, no brick pattern. Think injection-molded ABS plastic.
- **Material look:** Smooth molded ABS/polycarbonate panels over aluminum frame. Clean panel seams visible but not aggressive. Sealed, weatherproof, IP65-rated appearance.
- **Panel style:** Large smooth panels with subtle rounded edges where panels meet. Visible but minimal fasteners (small hex bolts at corners). NOT heavily bolted/tactical.

## 3. HEAD

- **Shape:** A compact rectangular module integrated at the front of the body. The head is slightly narrower than the body and has a **flat front face**. It can have gentle rounding on the top and sides but the FRONT FACE must be flat/vertical.
- **CRITICAL:** The head shape is similar to Spot/Go2's head module — it is NOT a literal dog head with a snout. It's a sensor housing module. Some gentle contouring is fine and expected, but it should NOT look like an organic animal head.
- **Front face:** Two square bright green LED panels (#22c55e) as "eyes" (camera/sensor windows). These are the robot's most recognizable feature — they give it a friendly, identifiable face.
- **Size:** ~12cm wide × 10cm tall × 10cm deep. About 0.27× body length wide.

## 4. LEGS (×4)

- **Configuration:** Four legs, one at each corner of the body. Mammalian stance (knees forward on front legs, knees backward on rear legs, like a dog).
- **Joints per leg:** 3 joints (hip, knee, ankle). Each joint has a visible cylindrical actuator housing (~5cm diameter).
- **Color:** Same dark matte olive-green as body, with dark grey/black actuator housings at joints.
- **LED strips:** Bright green (#22c55e) LED light strips running vertically along the **OUTWARD-FACING flat panel surface** of each UPPER leg segment (thigh/shoulder). One strip per upper leg, ~15cm long, ~1cm wide. NOT ring accents on joints. NOT on lower legs. Always illuminated when operating.
- **Feet:** Rounded black rubber foot pads with textured grip surface. ~5cm diameter. No claws or toes.
- **Stance:** Legs spread slightly wider than the body (~35cm track width vs 30cm body width). This gives a stable, planted stance.

## 5. ARM (×1)

- **Mounting point:** Top of body, front-center, just behind the head module. The shoulder joint sits on the dorsal surface.
- **Type:** Single multi-joint industrial robotic arm. Joints: shoulder (pitch + yaw), elbow (pitch), wrist (pitch + roll), gripper.
- **Segments:** 
  - Upper arm: ~18cm (shoulder to elbow)
  - Forearm: ~18cm (elbow to wrist)  
  - Gripper: ~10cm
- **Reach:** Can reach the ground in front of the robot AND reach back to the bag behind for compression.
- **Color:** Dark grey/black joint housings with olive-green arm segment panels.
- **Gripper:** 2-3 finger mechanical gripper. Dark grey/black. Industrial, NOT soft/silicone.
- **Wrist camera:** Small round camera lens (~2cm) near the gripper wrist for precision pickup.
- **Height:** When extended upward, the arm tip is ~40cm above the body top — making it the tallest point of the robot.
- **Default render pose:** Arm extended forward and slightly downward, actively reaching for or holding a piece of litter.

## 6. BAG SYSTEM (V2 — Folding Frame + Roll Dispenser + Gravity Seal)

This is the MOST DISTINCTIVE and INNOVATIVE feature. It MUST be clearly visible and accurately depicted.

### 6.1 System Overview

Three components mounted on the rear half of the robot's back:
1. **Bag roll dispenser** (center of back)
2. **Folding bag frame** (hinged at rear edge)
3. **Drawstring retention system** (for gravity-powered sealing)

The entire mechanism has only ONE moving part: the folding frame.

### 6.2 Bag Roll Dispenser

- **Position:** Center of the robot's flat back, between the arm mount (front) and the frame hinge (rear).
- **Orientation:** Cylinder axis runs **LEFT-TO-RIGHT** (perpendicular to the robot's front-to-back axis), **parallel to the frame hinge line**.
- **Size:** ~8cm diameter × ~28cm wide (slightly narrower than body width).
- **Mounting:** Sits LOW and CLOSE to the body surface, in a shallow cradle or recess. Not elevated on a tall mount.
- **Appearance:** Black anodized aluminum cylindrical housing with visible roll of black bag material.
- **The front edge of the roll area** serves as the **inner clipping line** where one edge of the bag opening attaches.

### 6.3 Folding Bag Frame

- **Type:** Rectangular rim frame made of dark grey/black metal tubing (~1cm diameter tube).
- **Dimensions:** Width = body width (~30cm). Depth (front-to-back) = half body length (~22cm).
- **Hinge point:** Single-axis hinge at the **REAR EDGE** of the robot body's top surface. Two metal support bars connect the frame to the hinge.
- **Actuator:** Small servo motor at the hinge. This is the ONLY moving part in the entire bag system.

#### Open Position (During Collection)

- Frame extends **backward and upward** from the rear edge of the body.
- The frame makes a **135-degree angle** with the robot's back surface (i.e., 45° past vertical, angling upward and rearward).
- The frame's outer rim is ~22cm behind the body's rear edge and ~22cm above it.
- Looking from the side: the frame angles up and back like an opened laptop lid past vertical.

#### Bag Attachment Geometry

The trash bag opening spans between **TWO PARALLEL CLIPPING LINES** that run left-to-right across the robot:

1. **Inner clip line** (near the bag roll): The bag's front/inner edge attaches here, close to the body center at the roll dispenser area.
2. **Outer clip line** (at the frame rim): The bag's rear/outer edge attaches to the far rim of the folding frame.

When the frame is in the open position, these two clip lines are separated by the frame's depth (~22cm), holding the bag opening wide. The bag hangs down freely between and below these two lines, creating an open receptacle behind the robot's rear.

**NO basket, NO cage, NO mesh, NO rigid walls around the bag.** Just the bag hanging from its two clipped edges.

### 6.4 Bag Swap Cycle (3 Steps)

**Step 1 — FOLD IN:** The frame folds inward (forward and downward), rotating toward the bag roll. The distance between the two clip lines shrinks. The full bag goes slack, its weight pulls it free from the clips. The bag begins to fall.

**Step 2 — CLIP NEW BAG:** While the frame is folded in flat against the body (near the roll), it clips onto the outer edge of the next bag from the roll.

**Step 3 — FOLD OUT:** The frame folds back out to the 135° open position. As it opens, it pulls the new bag's outer edge away from the roll, stretching the bag open and ready for collection.

### 6.5 Gravity-Powered Bag Sealing

Each bag has a **drawstring or plastic cord** threaded through its opening edge (standard drawstring trash bag design).

When the bag detaches from the clips and falls:
- The **drawstring remains attached** to a retention point on the frame or body
- The bag's weight **pulls the drawstring tight** as it falls
- The bag **cinches closed automatically** under its own weight
- The sealed bag drops to the ground behind the robot — ready for curbside collection
- **No additional actuator needed** — gravity does the sealing

### 6.6 Bag Specifications

- **Preferred:** Standard off-the-shelf heavy-duty drawstring trash bags on a roll (30-50L capacity)
- **Customization (if needed):** Reinforced edges for the clip system, pre-scored perforations between bags
- **Goal:** Off-the-shelf bags if possible. Custom bags acceptable but add supply chain complexity.
- **Material:** Heavy-duty LDPE, strong enough for 5-10kg of mixed litter
- **Color:** Black or dark grey, semi-transparent (operators can visually assess fill level)

### 6.7 Compression

The robotic arm doubles as the compressor:
- During collection, the arm periodically reaches back and presses litter down into the open bag
- The arm's gripper or flat wrist surface presses downward
- No separate compaction actuator needed

## 7. SENSORS

- **Front stereo cameras:** Behind the two green LED "eye" panels on the head face
- **LiDAR:** Small dark puck on top of body (subtle, between arm mount and bag roll)
- **Arm camera:** Small lens near the gripper wrist
- **No other external sensors prominently visible**

## 8. OTHER DETAILS

- **Antenna:** Small cellular antenna nub on the body (subtle)
- **No visible wheels or treads** — legs only
- **No visible charging port** (on the underside)
- **No text, no logos, no branding on the robot body.** Clean, unmarked surfaces.
- **NEVER reference any competitor brand name in render prompts** — the AI model will embed their branding as visible text

## 9. WHAT TO AVOID

- ❌ Any text, logos, or branding on the robot (especially "Unitree", "Go2", "Boston Dynamics", "Spot")
- ❌ Referencing competitor brands in prompts — even "similar to X" causes branding to appear
- ❌ Organic/literal dog head with snout — the head is a sensor housing module
- ❌ LED ring accents on joints — LEDs are STRIPS on OUTER UPPER LEGS only
- ❌ X-shaped scissor frame (V1 design, replaced)
- ❌ Basket, cage, or mesh around the bag
- ❌ Rough/textured/brick-pattern surfaces — surfaces are SMOOTH molded plastic
- ❌ Built-in bin/box instead of hanging bag
- ❌ Soft/silicone gripper (must be mechanical)
- ❌ Multiple arms
- ❌ Pure black body (must be dark olive-green)
- ❌ Glossy surfaces (must be matte)
- ❌ Overly futuristic/sci-fi aesthetic
- ❌ Stubby legs (legs should be ~same length as body)
- ❌ Bag roll axis front-to-back (must be LEFT-TO-RIGHT)

## 10. DESIGN EVOLUTION LOG

| Version | Date | Change |
|---------|------|--------|
| V1.0 | 2026-02-09 | Initial spec — X-frame scissor bag system, LED strips on body sides |
| V2.0 | 2026-02-10 | Folding frame + roll dispenser + gravity seal. Boxy head. LED strips on upper legs. |
| V2.1 | 2026-02-10 | Precise frame geometry: 135° angle, frame depth = half body, bag spans roll-to-rim |
| V2.2 | 2026-02-10 | Full dimensional ratios. Smooth modern panel aesthetic. Head clarified as sensor module (Spot-like). Roll axis perpendicular. Comprehensive proportions table. |

---

## RENDER PROMPT TEMPLATE

When generating renders, ALWAYS start with this core description. NEVER reference competitor brands.

```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.
```

Then append scene-specific details (lighting, environment, pose, camera angle).
