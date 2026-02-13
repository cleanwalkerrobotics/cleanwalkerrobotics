# Body Chassis + Head Module Design Description — CW-1

## Overview

The main body chassis and integrated head/sensor module for the CW-1 quadrupedal robot. This is the structural core that all other subsystems mount to: 4 legs, 1 arm, bag system, electronics, and battery. The body is a rectangular aluminum frame with 3D-printed enclosure panels. The head is an integrated forward extension of the body (not a separate block).

## Reference Data (from Robot Design Spec V2.3 + Component Decisions)

### Coordinate Convention
- X = lateral (positive = left), Y = longitudinal (positive = forward), Z = vertical (positive = up)
- Body center at origin (0, 0, 0) — this model is self-referenced, not relative to another component

### External Dimensions

| Dimension | Value | Notes |
|-----------|-------|-------|
| Body length | 600mm | Y-axis, from rear edge (Y=-300) to front edge (Y=+300) |
| Body width | 150mm | X-axis, from X=-75 to X=+75 |
| Body height | 120mm | Z-axis, from Z=-60 (bottom) to Z=+60 (top) |
| Head depth | 80mm | Extension beyond front edge, Y=300 to Y=380 |
| Head width | 150mm | Same as body (tapers slightly at front face) |
| Head height | 80mm | Z=-40 to Z=+40 (lower profile than body) |

### Mass Budget

| Component | Mass |
|-----------|------|
| Chassis frame (6061-T6 aluminum) | 2.0 kg |
| Enclosure panels (ASA 3D-printed) | 0.5 kg |
| Head/sensor housing | 0.5 kg |
| Battery (48V 20Ah) | 5.0 kg |
| Electronics (Jetson + PCB + adapters) | 0.5 kg |
| **Total body** | **8.0 kg** |

## Detailed Geometry

### 1. Main Chassis Frame
- **Purpose:** Structural backbone, all subsystems mount to this
- **Construction:** Rectangular frame of 20×20mm square aluminum tubes, welded or bolted
- **Outer dimensions:** 600×150×120mm
- **Frame members:**
  - 4× longitudinal rails (600mm, along Y) at the 4 bottom corners: (±75, Y, -60)
  - 4× lateral cross-members (150mm, along X) connecting bottom rails at Y=-300, -100, +100, +300
  - 4× vertical posts (120mm, along Z) at corners: (±75, ±300, Z)
  - 4× lateral cross-members (150mm, along X) connecting top rails at Y=-300, +300
  - 2× longitudinal top rails (600mm, along Y) at top edges: (±75, Y, +60)
- **Material:** 6061-T6 aluminum, 20×20mm square tube, 2mm wall
- **Color:** Medium grey (0.5, 0.5, 0.5)

### 2. Top Panel
- **Purpose:** Flat top surface for arm mounting and protection
- **Shape:** Rectangular plate, 600×150×3mm
- **Position:** Z=+60 (top of frame)
- **Cutouts:**
  - Arm turret mount: circular hole, 85mm diameter, centered at Y=+150 (front third)
  - LiDAR mount: circular hole, 50mm diameter, centered at Y=0 (body center)
  - Battery access: rectangular, 200×100mm, centered at Y=-100 (rear half)
- **Color:** Olive green (0.23, 0.29, 0.25)

### 3. Bottom Panel
- **Purpose:** Structural floor, battery support
- **Shape:** Rectangular plate, 600×150×3mm
- **Position:** Z=-60 (bottom of frame)
- **Cutouts:**
  - Charging port: circular, 25mm diameter, at Y=-200 (rear underside)
- **Color:** Dark grey (0.2, 0.2, 0.2)

### 4. Leg Mount Plates (×4)
- **Purpose:** Interface between chassis and leg hip mounting plates
- **Shape:** Rectangular reinforcement plates, 80×60×5mm each
- **Positions (bottom corners):**
  - Front-left: X=+75, Y=+250, Z=-60
  - Front-right: X=-75, Y=+250, Z=-60
  - Rear-left: X=+75, Y=-250, Z=-60
  - Rear-right: X=-75, Y=-250, Z=-60
- **Bolt pattern:** 4× M5 holes at corners (65×45mm pattern), matching leg hip mounting plate
- **Note:** Plates extend downward 5mm below frame bottom
- **Color:** Dark grey (0.3, 0.3, 0.3)

### 5. Arm Turret Mount
- **Purpose:** Reinforced platform for arm turret base
- **Shape:** Circular reinforcement ring, OD=90mm, ID=60mm, 5mm thick
- **Position:** Z=+60, centered at Y=+150 (front-center top)
- **Bolt pattern:** 4× M4 holes on 70mm bolt circle (matching turret base)
- **Color:** Dark grey (0.3, 0.3, 0.3)

### 6. Bag System Hinge Mount
- **Purpose:** Hinge bracket mounting for folding bag frame
- **Shape:** 2× reinforcement plates, 30×20×5mm
- **Positions:** X=±(75-30)=±45, Y=-300 (rear edge), Z=+60 (top)
- **Bolt pattern:** 2× M5 holes per plate for hinge bracket
- **Color:** Dark grey (0.3, 0.3, 0.3)

### 7. Head Module (Integrated Sensor Housing)
- **Purpose:** Houses front stereo cameras, LED "eyes", forward-facing sensors
- **Shape:** Tapered rectangular box extending from body front face
  - Rear face (at Y=+300): same size as body cross-section (150×120mm)
  - Front face (at Y=+380): narrower and lower (120×80mm) — slight taper
  - Use linear loft or simple box with chamfered top edge
- **Front face features:**
  - 2× LED "eye" panels: square cutouts, 30×30mm each, centered at X=±25, Z=+10
  - These are camera windows (OAK-D Pro stereo pair behind them)
- **Material appearance:** Same olive green as body panels
- **Color:** Olive green (0.23, 0.29, 0.25) with bright green (0.13, 0.77, 0.37) LED panel inserts

### 8. LiDAR Mount
- **Purpose:** Mounting point for LiDAR sensor puck on body top
- **Shape:** Short cylinder, OD=60mm, height=15mm
- **Position:** Z=+60+3=+63 (on top panel surface), Y=0 (body center)
- **Color:** Dark grey (0.15, 0.15, 0.15)

### 9. Antenna Nub
- **Purpose:** 4G cellular antenna
- **Shape:** Small cylinder, OD=10mm, height=20mm
- **Position:** Z=+63, Y=-50, X=+60 (rear-right top)
- **Color:** Black (0.1, 0.1, 0.1)

### 10. Side Panels (×2)
- **Purpose:** Weatherproof enclosure walls
- **Shape:** Rectangular plates, 600×120×3mm each
- **Positions:** X=+75 (left) and X=-75 (right), spanning Y=-300 to Y=+300, Z=-60 to Z=+60
- **Color:** Olive green (0.23, 0.29, 0.25)

## Assembly Dimensions Summary

| Measurement | Value |
|-------------|-------|
| Total length (with head) | 680mm (300 rear + 300 front body + 80 head) |
| Total width | 150mm |
| Total height (with LiDAR) | 138mm (120 body + 3 top panel + 15 LiDAR) |
| Chassis tube size | 20×20mm square, 2mm wall |
| Top/bottom panel thickness | 3mm |
| Side panel thickness | 3mm |
| Leg mount plate size | 80×60×5mm |
| Head taper | 150→120mm width, 120→80mm height over 80mm depth |

## Default Orientation

Body horizontal, Y-axis pointing forward, Z-axis up. No rotation. This is the world reference frame for full robot assembly.

## Simplification Notes for CadQuery

- Frame members can be modeled as solid rectangular boxes (20×20mm cross-section, full length) — hollow tubes are optional refinement
- Head taper: if CadQuery loft is unreliable, use a simple box (150×120×80mm) attached to the front face
- Panel cutouts: use boolean cut operations on the flat panels
- LED panels: simple colored boxes inset into the head front face
- Focus on correct dimensions and mount point positions over surface detail
