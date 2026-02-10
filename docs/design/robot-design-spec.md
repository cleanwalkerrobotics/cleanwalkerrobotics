# CleanWalker Robot — Definitive Visual Design Specification

**Version:** 1.0
**Date:** 2026-02-10
**Reference Image:** Hero Fleet render (hero-fleet.png) — THIS is the canonical design.
**Purpose:** This document is the single source of truth for generating consistent robot renders.

---

## 1. OVERALL FORM

- **Type:** Quadrupedal robot ("robot dog"), similar in silhouette to Unitree Go2 / Boston Dynamics Spot
- **Scale:** Medium dog size. ~60cm tall at body, ~80cm long (nose to tail), ~100cm total height with frame raised
- **Aesthetic:** Utilitarian-industrial but modern. Think military equipment meets Dyson. NOT toy-like or consumer-cute.

## 2. BODY

- **Shape:** Horizontally elongated rectangular box with rounded/chamfered edges. Slightly convex top surface.
- **Color:** Dark matte olive-green / military sage green (hex ~#3B4A3F). NOT pure black — has visible green-grey undertone.
- **Finish:** Matte to satin. No gloss.
- **Material look:** Molded ABS/polycarbonate panels over metal frame. Visible panel seams. Sealed, weatherproof appearance.
- **LED strips:** Two horizontal green LED bars on each side of the body (bright green, #22c55e). One strip per side, running along the mid-line. Always illuminated when operating.
- **Front face:** Two square green LED "eyes" (camera/sensor windows) on the front face, giving the robot a face-like appearance.

## 3. LEGS (x4)

- **Configuration:** Four legs, one at each corner of the body. Mammalian stance (knees forward on front legs, knees backward on rear legs).
- **Joints per leg:** 3 visible joints (hip, knee, ankle/foot). 
- **Color:** Same dark matte olive-green as body.
- **Construction:** Tubular/cylindrical actuator housings at each joint. Visible servo motors at hip and knee.
- **Feet:** Rounded rubber foot pads, dark black rubber. No claws or toes.
- **Proportions:** Legs are roughly the same height as the body depth — robot stands with body at about 60% of total height.

## 4. ARM (x1)

- **Mounting point:** Top of body, front-center. Rises from the front dorsal surface.
- **Type:** Single articulated robotic arm, 3-4 joints (shoulder, elbow, wrist, gripper).
- **Reach:** Long enough to reach the ground in front of the robot AND up to the bag frame above/behind.
- **Color:** Same dark olive-green as body, with dark grey/black joint housings.
- **Gripper:** 2-3 finger mechanical gripper at the end. Dark grey/black. Industrial look, NOT soft/silicone.
- **Camera/sensor:** Small round camera module mounted near the gripper wrist (for precision pickup).
- **Default pose:** Arm extended forward or slightly upward, actively grabbing or carrying an item.

## 5. BAG CASSETTE SYSTEM (on top of body)

This is the MOST DISTINCTIVE feature. It MUST be clearly visible in every render.

- **Frame type:** X-shaped scissor/folding frame, made of metal tubing (dark grey/olive, matching body).
- **Frame base:** Mounts flat on the top rear of the robot body.
- **Frame top:** Square/rectangular rim (~30cm x 30cm) that holds the bag open. The rim has a slightly lighter color (brown/tan/olive accent).
- **Height when raised:** Frame extends ~40cm above the body (total robot height ~100cm).
- **Bag:** Semi-transparent black/dark grey trash bag, draped over the frame. The bag hangs down inside the X-frame.
- **Bag contents:** When in collecting mode, the bag shows colorful litter items visible through the semi-transparent material.
- **Frame appearance:** The X-cross is clearly visible — two diagonal tubes crossing in the middle, with the square rim on top. Simple, structural, functional.

## 6. SENSORS

- **Front stereo cameras:** Two camera lenses on the front face (the green "eye" squares).
- **LiDAR:** Small dark puck on top of body (if visible — often obscured by the bag frame).
- **Arm camera:** Small lens near the gripper wrist.
- **No other external sensors prominently visible.**

## 7. OTHER DETAILS

- **Antenna:** Small cellular antenna nub on the body (subtle, not prominent).
- **No visible wheels, treads, or non-leg locomotion.**
- **No visible charging port (it's on the underside).**
- **No text, logos, or branding on the robot body in renders.**

## 8. WHAT TO AVOID (Consistency Errors in Previous Renders)

- ❌ Robot appearing too bulky or tank-like
- ❌ Built-in bin/box instead of bag frame
- ❌ Soft/silicone gripper (should be mechanical)
- ❌ Multiple arms
- ❌ Frame that looks like a basket instead of X-shaped scissor frame
- ❌ Pure black body (should be dark olive-green)
- ❌ Missing LED strips
- ❌ Overly futuristic/sci-fi look
- ❌ Bag frame missing or collapsed in active collection scenes

---

## PROMPT TEMPLATE

When generating renders, ALWAYS include this core robot description:

```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic.
```

Then add the scene-specific details after this base description.
