# CleanWalker Robot — Definitive Visual Design Specification

**Version:** 2.0
**Date:** 2026-02-10
**Purpose:** This document is the single source of truth for the robot's visual design and mechanical systems. All renders, CAD models, and technical descriptions MUST conform to this spec.

---

## 1. OVERALL FORM

- **Type:** Quadrupedal robot ("robot dog")
- **Scale:** Medium dog size. ~60cm tall at body, ~80cm long (nose to tail), ~100cm total height with bag frame in open position
- **Aesthetic:** Industrial-utilitarian but modern. Chunky weatherproof panels, visible bolts and panel seams. Think military equipment meets Dyson. NOT toy-like, NOT consumer-cute, NOT sci-fi.
- **Silhouette:** Boxy rectangular body on four articulated legs, robotic arm at front, bag system at rear

## 2. BODY

- **Shape:** Horizontally elongated rectangular box with slightly rounded/chamfered edges. Flat top surface providing a platform for the arm mount and bag system.
- **Head:** Compact rectangular box shape at the front of the body. Purely mechanical and boxy — NOT dog-like, NOT organic, NOT a snout. The head is an integrated extension of the body, slightly narrower.
- **Color:** Dark matte olive-green / military sage green (hex ~#3B4A3F). NOT pure black — has visible green-grey undertone.
- **Finish:** Matte to satin. No gloss. Weatherproof appearance.
- **Material look:** Molded ABS/polycarbonate panels over aluminum frame. Visible panel seams, screws, and bolts. Sealed, IP65-rated weatherproof appearance.
- **Front face:** Two square bright green LED panels as "eyes" (camera/sensor windows) on the front face of the head, giving the robot a recognizable face. These are the primary visual identifier.

## 3. LEGS (x4)

- **Configuration:** Four legs, one at each corner of the body. Mammalian stance (knees forward on front legs, knees backward on rear legs).
- **Joints per leg:** 3 visible joints (hip, knee, ankle/foot). Each joint has a visible cylindrical actuator housing.
- **Color:** Same dark matte olive-green as body, with dark grey/black joint housings.
- **LED strips:** Bright green (#22c55e) LED light strips running vertically along the OUTWARD-FACING SIDES of the UPPER portion of each leg (thigh/shoulder segments). NOT ring accents on the joints — the strips run along the outer flat panel surface of the upper leg segment, facing away from the body. Always illuminated when operating.
- **Feet:** Rounded rubber tread foot pads, dark black rubber. Textured for grip. No claws or toes.
- **Proportions:** Legs are roughly the same height as the body depth — robot stands with body at about 60% of total height.

## 4. ARM (x1)

- **Mounting point:** Top of body, front-center. Rises from the front dorsal surface, just behind the head.
- **Type:** Single articulated robotic arm, 3-4 joints (shoulder, elbow, wrist, gripper).
- **Reach:** Long enough to reach the ground in front of the robot AND reach back to the bag frame for compression.
- **Color:** Dark grey/black joint housings with olive-green upper arm segments.
- **Gripper:** 2-3 finger mechanical gripper at the end. Dark grey/black. Industrial look, NOT soft/silicone.
- **Camera/sensor:** Small round camera module mounted near the gripper wrist (for precision pickup).
- **Default pose:** Arm extended forward or slightly upward, actively grabbing or carrying an item.
- **Secondary function:** The arm also performs litter compression by pressing down into the open bag.

## 5. BAG SYSTEM (V2 — Folding Frame + Roll Dispenser)

This is the MOST DISTINCTIVE and INNOVATIVE feature. It MUST be clearly visible and accurately depicted in every render.

### 5.1 Overview

The bag system replaces the previous V1 X-shaped scissor frame design with a simpler, more reliable folding frame mechanism. It consists of three components mounted on the rear top of the robot body:

1. **Bag roll dispenser** (center of back)
2. **Folding bag frame** (rear of back)
3. **Drawstring retention clips** (on frame or body)

### 5.2 Bag Roll Dispenser

- **Position:** Mounted horizontally on the flat top of the robot body, between the arm mount (front) and the bag frame hinge (rear). Sits LOW and CLOSE to the body surface.
- **Form:** A compact black cylindrical tube/spindle housing, oriented perpendicular to the robot's length (left-to-right across the back).
- **Diameter:** ~8-10cm cylinder, same width as the robot body (~30cm).
- **Function:** Holds a standard roll of heavy-duty trash bags. Bags feed out from the roll toward the rear frame. One edge of each bag's opening attaches at a clipping line near the roll.
- **Appearance:** Black anodized aluminum housing with a visible roll of black bag material. Simple, mechanical. Sits flush and low on the robot's back.

### 5.3 Folding Bag Frame

- **Type:** A rectangular metal rim/frame attached to the rear of the robot body via a single hinge axis.
- **Frame dimensions:** Width = same as the robot body width (~30cm). Depth (front-to-back) = approximately HALF the robot body length (~20cm, given ~40cm body length).
- **Material:** Dark grey/black metal tubing, matching the arm's joint color.
- **Hinge point:** At the REAR EDGE of the robot body's top surface. This is the only pivot point — the frame swings outward/backward and inward/forward around this single axis.
- **Actuator:** The frame is motor-actuated (small servo at the hinge) — it folds in and out under electronic control. This is the ONLY moving part in the bag system.

#### Bag Opening Geometry

The trash bag opening spans between TWO attachment lines:
1. **Inner edge:** The bag's inner/front edge clips to a clipping line near the bag roll dispenser (close to the body center).
2. **Outer edge:** The bag's outer/rear edge clips to the OUTER RIM of the folding frame (the far end of the frame).

This means the bag opening stretches across the full depth of the frame — from the roll area to the frame's outer rim. The frame holds the bag open by pulling the outer edge away from the inner edge.

#### Operating Positions:

**OPEN position (during collection):**
- Frame is folded outward and upward, making a **135-degree angle** with the flat back surface of the robot (i.e., 45 degrees above horizontal, angled upward and backward from the rear edge).
- The frame extends BEHIND the robot's rear, past the rear legs.
- The outer rim of the frame pulls the bag's outer edge up and back, while the inner edge stays clipped near the roll.
- The bag hangs freely downward between these two edges, creating an open receptacle behind/below the frame.
- NO basket, NO cage, NO mesh around the bag — just the bag hanging from its two clipped edges.
- The bag is visible hanging loosely, partially filled with collected litter.

**CLOSED/FOLD-IN position (during bag swap):**
- Frame folds inward and downward, rotating toward the bag roll dispenser (closing like a book toward the body).
- As the frame folds in, the distance between the two clipping edges shrinks, the bag goes slack.
- The full bag's weight causes it to detach from the frame clips.
- While folded in near the roll, the frame clips onto the outer edge of the next bag from the roll.
- Frame then folds back out to OPEN position, pulling the new bag open and taut, ready for collection.

### 5.4 Bag Swap Cycle (3 steps)

This is the automated bag change sequence:

**Step 1 — FOLD IN:** The frame folds inward toward the bag roll. As it folds, the full bag's weight causes it to detach from the frame's clips. The bag begins to fall.

**Step 2 — CLIP NEW BAG:** While the frame is folded in near the roll, it clips onto the opening edge of the next bag from the roll. The clip mechanism grabs the bag's edge securely.

**Step 3 — FOLD OUT:** The frame folds back out to its open operating position, pulling the new bag open and taut, ready to receive litter.

### 5.5 Gravity-Powered Bag Sealing

This is a key innovation:

- Each bag has a **drawstring or plastic cord** threaded through its opening edge (like a standard drawstring trash bag).
- When the bag detaches from the frame and begins to fall, the **drawstring remains attached** to a retention point on the frame or robot body.
- As the full bag falls under gravity, its weight **pulls the drawstring tight**, automatically cinching the bag closed.
- The sealed bag drops to the ground behind the robot — ready for curbside collection.
- **No additional actuator or mechanism is needed for sealing** — gravity does the work.

### 5.6 Bag Specifications

- **Preferred:** Standard off-the-shelf heavy-duty drawstring trash bags on a roll (30-50 liter capacity)
- **Customization (if needed):** Reinforced edges for the clip system, pre-scored perforations between bags on the roll, drawstring loop engineered for the retention point
- **Goal:** Use off-the-shelf bags if at all possible. Custom bags are acceptable but add cost and supply chain complexity.
- **Material:** Heavy-duty LDPE or similar, strong enough to hold 5-10kg of mixed litter without tearing
- **Color:** Black or dark grey, semi-transparent (so operators can visually assess fill level)

### 5.7 Compression

- The robotic arm doubles as the compression mechanism.
- During collection, the arm periodically reaches back and presses collected litter down into the open bag.
- No separate compaction actuator needed — the arm's gripper or the flat of the wrist presses downward.

## 6. SENSORS

- **Front stereo cameras:** Two camera lenses behind the green LED "eye" panels on the front face.
- **LiDAR:** Small dark puck on top of body (subtle, between arm mount and bag roll).
- **Arm camera:** Small lens near the gripper wrist for precision pickup.
- **No other external sensors prominently visible.**

## 7. OTHER DETAILS

- **Antenna:** Small cellular antenna nub on the body (subtle, not prominent).
- **No visible wheels, treads, or non-leg locomotion.**
- **No visible charging port (it's on the underside).**
- **No text, no logos, no branding on the robot body.** Clean, unmarked surfaces. NEVER reference any competitor brand in render prompts.

## 8. WHAT TO AVOID (Consistency Rules)

- ❌ Any text, logos, or branding on the robot body (especially "Unitree", "Go2", or any competitor names)
- ❌ Referencing competitor brands in render prompts (e.g., "similar to Unitree Go2") — the AI model will embed their branding
- ❌ Dog-like or organic head shape — the head is BOXY and RECTANGULAR
- ❌ LED ring accents on the joints — LEDs are STRIPS on the UPPER LEGS
- ❌ X-shaped scissor frame (V1 design, replaced by V2 folding frame)
- ❌ Basket, cage, or mesh around the bag — the bag hangs FREELY
- ❌ Built-in bin/box instead of hanging bag
- ❌ Soft/silicone gripper (should be mechanical)
- ❌ Multiple arms
- ❌ Pure black body (should be dark olive-green)
- ❌ Overly futuristic/sci-fi look
- ❌ Glossy surfaces (should be matte)
- ❌ Robot appearing too bulky or tank-like

## 9. DESIGN EVOLUTION LOG

| Version | Date | Change | Reason |
|---------|------|--------|--------|
| V1.0 | 2026-02-09 | Initial spec — X-frame scissor bag system, LED strips on body sides | Original design |
| V2.0 | 2026-02-10 | Folding frame + roll dispenser + gravity seal. Boxy head. LED strips on upper legs. No competitor brand references. | Simpler mechanism, fewer moving parts, self-sealing bags, more distinctive industrial look |

---

## PROMPT TEMPLATE

When generating renders, ALWAYS include this core robot description. NEVER reference any competitor brand.

```
A custom-designed quadrupedal robot with a matte dark olive-green body. The head is a compact rectangular box with two square bright green LED panels as eyes on the front face — purely mechanical and boxy, not organic or dog-like. Bright green LED light strips on the outward-facing sides of the upper legs. Four articulated legs with rubber tread foot pads. A tall multi-joint industrial robotic arm with a mechanical gripper mounted on the front top of the body, rising significantly above the body with visible shoulder, elbow and wrist joints. On the center of the robots flat back, a black cylindrical bag roll dispenser is mounted horizontally, sitting low and close to the body surface. At the very rear edge of the body, a rectangular metal frame is hinged — in the open position it extends backward and upward at a 135-degree angle from the back surface. A heavy-duty black drawstring trash bag hangs freely between two clipping points: the inner edge clips near the bag roll and the outer edge clips to the far rim of the frame. The bag hangs down openly behind the robot with no basket, no cage, no mesh. The frame is the same width as the body and about half the body length in depth. Medium-dog sized, 60cm body. Industrial boxy design, chunky weatherproof panels, visible bolts. No text, no logos, no branding, no watermarks on the robot. Clean unmarked matte surfaces.
```

Then add scene-specific details after this base description.
