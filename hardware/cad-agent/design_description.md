# Leg Module Design Description — Front-Left (FL) Leg

## Overview

One complete front-left leg assembly for the CW-1 quadrupedal robot. 3 degrees of freedom: hip abduction/adduction (yaw), hip flexion/extension (pitch), and knee flexion/extension (pitch). The front-left leg is the reference design; all other legs are mirrored/rotated copies.

## Reference Data (from URDF: cleanwalker_cw1.urdf)

### Coordinate Convention
- X = forward, Y = left, Z = up
- Leg mounts at body position: X=+250mm, Y=+75mm, Z=0mm (front-left corner)

### Joint Specifications

| Joint | Type | Axis | Lower Limit | Upper Limit | Effort Limit | Actuator |
|-------|------|------|-------------|-------------|--------------|----------|
| Hip yaw (abduction) | Revolute | Z (vertical) | -28.6° (-0.4993 rad) | +28.6° (+0.4993 rad) | 30 Nm | CubeMars AK60-6 |
| Hip pitch (swing) | Revolute | Y (lateral) | -90° (-1.5708 rad) | +90° (+1.5708 rad) | 30 Nm | CubeMars AK70-10 |
| Knee pitch (bend) | Revolute | Y (lateral) | -5.7° (-0.0995 rad) | +149° (+2.6005 rad) | 30 Nm | CubeMars AK70-10 |

### Link Specifications

| Link | Mass | Geometry | Dimensions | Material Color |
|------|------|----------|------------|----------------|
| Hip (actuator housing) | 0.4 kg | Cylinder | R=25mm, L=50mm, axis along Y | Dark grey |
| Thigh (upper leg) | 0.5 kg | Box (URDF) / Structural tube (CAD) | 40×40×200mm cross-section, 200mm long along Z | Olive green |
| Calf (lower leg) | 0.2 kg | Cylinder | R=15mm, L=200mm, along Z | Olive green |
| Foot | 0.1 kg | Sphere | R=25mm | Black rubber |

**Total leg mass: 1.2 kg**

## Actuator Specifications

### CubeMars AK70-10 (Hip pitch, Knee pitch) — 2 per leg
- Outer diameter: ~98mm (housing)
- Length: ~48mm (axial)
- Mass: ~500g
- Peak torque: 24.8 Nm
- Voltage: 48V
- Interface: CAN bus
- Mounting: 4× M4 bolts on 76mm bolt circle

### CubeMars AK60-6 (Hip yaw) — 1 per leg
- Outer diameter: ~76mm (housing)
- Length: ~38mm (axial)
- Mass: ~305g
- Peak torque: 9 Nm
- Voltage: 24V
- Interface: CAN bus
- Mounting: 4× M3 bolts on 58mm bolt circle

## Detailed Geometry

### 1. Hip Mounting Plate
- **Purpose:** Attaches leg assembly to body chassis at front-left corner
- **Shape:** Rectangular plate, 80mm wide × 60mm deep × 5mm thick
- **Material:** 6061-T6 aluminum
- **Mounting holes:** 4× M5 clearance holes at corners (6mm diameter), 65mm × 45mm bolt pattern
- **Center hole:** 30mm diameter for cable pass-through (power + CAN bus)
- **Position:** Centered at body attachment point (X=250, Y=75, Z=0 from body center)

### 2. Hip Yaw Assembly
- **Actuator:** AK60-6 mounted vertically on the hip mounting plate
- **Housing:** Cylindrical shell, OD=82mm, ID=77mm, height=45mm (encloses AK60-6)
- **Axis:** Z (vertical) — rotates the entire leg left/right for abduction/adduction
- **Rotation range:** ±28.6°
- **Output:** The actuator output shaft connects to the hip pitch bracket below

### 3. Hip Pitch Assembly
- **Actuator:** AK70-10 mounted horizontally, axis along Y (lateral)
- **Housing:** Cylindrical shell, OD=104mm, ID=99mm, height=55mm
- **Position:** Offset 35mm outward (Y direction) from hip yaw axis
- **Axis:** Y (lateral) — swings upper leg forward/backward
- **Rotation range:** ±90°
- **Bracket:** U-shaped bracket connecting hip yaw output to hip pitch motor, 3mm thick aluminum

### 4. Upper Leg (Thigh)
- **Structure:** Two parallel structural tubes (TUBE METHOD), 15mm OD × 12mm ID, 200mm long
- **Tube spacing:** 30mm center-to-center (straddle the actuator output)
- **Cross braces:** 2× horizontal tubes at 1/3 and 2/3 length, same diameter, connecting the parallel tubes
- **LED strip channel:** Flat face on the outward side, 15mm wide × 2mm deep groove for LED strip
- **Top end:** Connects to hip pitch output shaft
- **Bottom end:** Connects to knee pitch actuator housing
- **Direction:** Extends downward (-Z) from hip pitch joint, 200mm total

### 5. Knee Pitch Assembly
- **Actuator:** AK70-10 mounted horizontally, axis along Y (lateral)
- **Housing:** Cylindrical shell, OD=104mm, ID=99mm, height=55mm (same as hip pitch)
- **Position:** At bottom of upper leg, 200mm below hip pitch axis
- **Axis:** Y (lateral) — bends the knee
- **Rotation range:** -5.7° to +149° (front leg: knee bends backward)

### 6. Lower Leg (Calf)
- **Structure:** Single structural tube, 30mm OD × 25mm ID, 200mm long
- **Direction:** Extends downward (-Z) from knee joint
- **Cable routing:** Internal bore (25mm ID) serves as cable routing channel for foot sensors

### 7. Foot
- **Shape:** Hemisphere + short cylinder, 25mm radius
- **Material:** Black rubber (Shore A 60-70)
- **Attachment:** Press-fit or M4 bolt into lower leg tube end
- **Position:** At bottom of lower leg, 200mm below knee axis

### 8. Cable Routing
- **Path:** Cables enter at hip mounting plate center hole (30mm), route through hip yaw housing, along hip pitch bracket, through upper leg tube interior, through knee housing, down lower leg bore
- **Cable types:** 48V power (2× 14AWG), CAN bus (2× twisted pair), encoder signals
- **Channel dimensions:** Minimum 10mm × 10mm cross-section throughout path

## Assembly Dimensions Summary

| Measurement | Value |
|-------------|-------|
| Hip to knee (upper leg length) | 200mm |
| Knee to foot (lower leg length) | 200mm |
| Total leg length (hip to foot) | ~400mm + actuator housings |
| Hip yaw housing diameter | 82mm |
| Hip pitch housing diameter | 104mm |
| Knee housing diameter | 104mm |
| Mounting plate size | 80mm × 60mm × 5mm |
| Upper leg cross-section | 30mm wide (tube spacing) × 40mm deep |
| Lower leg diameter | 30mm OD |
| Foot radius | 25mm |
| Total leg mass | 1.2 kg |

## Default Pose (for CAD model)

Leg in nominal standing position:
- Hip yaw: 0° (centered)
- Hip pitch: 0° (vertical)
- Knee pitch: 0° (straight — thigh and calf aligned vertically)

This gives a straight-leg pose where the total height from mounting plate to foot = ~455mm (5mm plate + 45mm hip yaw + 55mm hip pitch + 200mm thigh + 55mm knee + 200mm calf + 25mm foot radius).
