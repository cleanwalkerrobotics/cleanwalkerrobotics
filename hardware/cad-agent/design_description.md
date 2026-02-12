# Design Description: CW-1 Bag System Assembly

## Overview
The bag dispensing and collection system for the CleanWalker CW-1 quadrupedal litter robot. This is the most innovative subsystem — a folding frame with bag roll dispenser and gravity-powered bag sealing. Only ONE moving part (the folding frame).

**Source:** Full robot spec at `docs/design/robot-design-spec.md`, Section 6.

## Context
This mounts on the rear half of the robot's flat back surface. The robot body is ~600mm long × 150mm wide × 120mm tall. The arm turret occupies the front-center of the back. This system occupies the rear ~300mm of the back surface.

## Components

### 1. Bag Roll Dispenser
- **Position:** Center of robot's back, behind the arm mount area
- **Orientation:** Cylinder axis runs LEFT-TO-RIGHT (orthogonal to robot's front-to-back axis)
- **Roll cylinder:** Ø80mm × 130mm wide (slightly narrower than 150mm body width)
- **Housing:** Shallow cradle/recess, sits LOW and CLOSE to the body surface (not elevated)
- **Material appearance:** Black anodized aluminum cylindrical housing
- **Mounting:** Two side brackets (3mm thick aluminum) supporting the roll axis, bolted to the body top surface
- **The front edge of the roll area** = inner clip line where bag opening front edge attaches

### 2. Folding Bag Frame
- **Type:** Rectangular rim frame made of Ø10mm circular cross-section metal tubing
- **Frame dimensions:** 150mm wide (matches body) × 220mm deep (front-to-back)
- **Material:** Dark grey/black metal (aluminum tube, anodized)
- **Hinge point:** Single-axis hinge at the REAR EDGE of the body top surface
- **Two support bars:** Connect frame to hinge point, ~10mm diameter, 80mm long each
- **Hinge mechanism:** Pin hinge with 2mm diameter pin, housed in a 15mm wide bracket on each side

#### Open Position (model this as default)
- Frame extends backward and upward from rear edge at 135° angle to body surface
- i.e., 45° past vertical, angling upward and rearward
- Frame outer rim is approximately 220mm behind and 220mm above the body rear edge

#### Clip System
- **Inner clip line:** Along the front edge of the roll dispenser area — a 130mm wide rail with spring clips (4× small spring clips, 20mm wide each, spaced evenly)
- **Outer clip line:** Along the far rim of the folding frame — matching 4× spring clips
- Clips are simple leaf-spring type, 2mm thick spring steel, 20mm × 8mm each

### 3. Hinge Assembly
- **Location:** Rear edge of body top surface, centered
- **Type:** Single-axis pin hinge
- **Pin:** Ø4mm × 160mm (spanning body width)
- **Brackets:** 2× L-shaped mounting brackets, 3mm aluminum, 25mm × 20mm footprint
- **Actuator space:** 15mm × 15mm × 30mm void on left side for micro servo (not modeled, just leave clearance)

## Assembly Relationships
- Roll dispenser is ~150mm from the body's rear edge (centered front-to-back in the rear half)
- Hinge brackets are at the rear edge, 30mm from each side
- When frame is open (135°), bag hangs between inner clip line (at roll) and outer clip line (at frame rim)
- The ~220mm separation between clip lines holds the bag opening wide

## Dimensions Summary
| Parameter | Value |
|---|---|
| Overall width | 150mm |
| Overall depth (roll to frame tip, open) | ~370mm |
| Overall height (frame open) | ~280mm above body surface |
| Roll diameter | 80mm |
| Roll width | 130mm |
| Frame tube diameter | 10mm |
| Frame depth | 220mm |
| Frame open angle | 135° from body surface |
| Clip count | 4× inner + 4× outer |
| Hinge pin diameter | 4mm |
| Material thickness (brackets) | 3mm |

## Notes
- All dimensions in millimeters
- Model in the OPEN position (135° frame angle) as default
- Suitable for CNC machining — no undercuts in bracket/housing parts
- The bag itself is NOT modeled — just the mechanical hardware
- Leave clearance void for servo actuator at hinge (don't model the servo)
- The body surface is NOT part of this model — assume flat mounting plane at Z=0
