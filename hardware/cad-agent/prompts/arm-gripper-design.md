# Arm + Gripper Assembly Design Description — CW-1

## Overview

Complete 5-DOF robotic arm assembly for the CW-1 quadrupedal litter-collecting robot. Mounts to the front-center of the body via a turret base. The arm reaches forward/down to pick up litter and back to compress it into the bag. Includes turret, upper arm, forearm, wrist, and 2-finger mechanical gripper with silicone fingertip pads.

## Reference Data (from URDF: cleanwalker_cw1.urdf)

### Coordinate Convention
- X = forward, Y = left, Z = up
- Arm turret mounts at body position: X=+150mm, Y=0, Z=+60mm (front-center, top of body)

### Joint Specifications

| Joint | Type | Axis | Lower Limit | Upper Limit | Effort Limit | Actuator |
|-------|------|------|-------------|-------------|--------------|----------|
| Turret yaw | Revolute | Z (vertical) | -180° (-3.14159 rad) | +180° (+3.14159 rad) | 15 Nm | Dynamixel XM430-W350-T |
| Shoulder pitch | Revolute | Y (lateral) | -45° (-0.7854 rad) | +180° (+3.14159 rad) | 15 Nm | Dynamixel XM430-W350-T |
| Elbow pitch | Revolute | Y (lateral) | 0° | +150° (+2.618 rad) | 15 Nm | Dynamixel XM430-W350-T |
| Wrist pitch | Revolute | Y (lateral) | -90° (-1.5708 rad) | +90° (+1.5708 rad) | 10 Nm | Dynamixel XL430-W250-T |
| Gripper open/close | Revolute | Y (lateral) | 0° | +60° (+1.0472 rad) | 5 Nm | Dynamixel XL430-W250-T |

### Link Specifications

| Link | Mass | Geometry | Dimensions | Material Color |
|------|------|----------|------------|----------------|
| Turret base | 0.3 kg | Cylinder | R=40mm, H=50mm, axis along Z | Dark grey |
| Upper arm | 0.3 kg | Structural tube | 30×30mm cross-section, 180mm long along Z | Olive green panels + dark grey joints |
| Forearm | 0.3 kg | Structural tube | 30×30mm cross-section, 180mm long along Z | Olive green panels + dark grey joints |
| Wrist | 0.15 kg | Cylinder | R=20mm, H=50mm | Dark grey |
| Gripper (body + fingers) | 0.15 kg | 2-finger mechanism | 100mm total length, 60mm max opening | Dark grey + silicone tips |

**Total arm mass: 1.2 kg (excluding payload)**

## Actuator Specifications

### Dynamixel XM430-W350-T (Turret, Shoulder, Elbow) — 3 per arm
- Form factor: ~46.5mm × 36mm × 34mm (rectangular, integrated gearbox)
- Mass: ~82g
- Stall torque: 4.1 Nm (at 12V)
- Voltage: 12V (nominal)
- Interface: TTL (half-duplex UART, daisy-chain capable)
- Mounting: 4× M2.5 tapped holes on both faces

### Dynamixel XL430-W250-T (Wrist, Gripper) — 2 per arm
- Form factor: ~28.5mm × 46.5mm × 34mm
- Mass: ~57.2g
- Stall torque: 1.5 Nm (at 11.1V)
- Voltage: 11.1V (nominal)
- Interface: TTL (same bus as XM430)
- Mounting: 4× M2 tapped holes on both faces

## Detailed Geometry

### 1. Turret Base
- **Purpose:** Provides 360° yaw rotation for the entire arm, mounts to body dorsal surface
- **Shape:** Cylinder, OD=80mm, height=50mm
- **Internal:** Hollow (ID=60mm) for cable routing (TTL bus + power)
- **Mounting:** 4× M4 holes on bottom face at 60mm bolt circle diameter for body attachment
- **Top:** Flat face with 4× M2.5 holes at 40mm bolt circle for shoulder bracket
- **Actuator:** XM430-W350-T mounted internally (output shaft exits top center)
- **Position:** Centered at X=150, Y=0, Z=60 (body coordinates) — sits on top of body
- **Color:** Dark grey (0.2, 0.2, 0.2)

### 2. Shoulder Joint Bracket
- **Purpose:** Connects turret output to upper arm, provides shoulder pitch rotation
- **Shape:** U-bracket (fork end), 50mm wide × 40mm tall × 30mm deep, 3mm wall thickness
- **Material:** 6061-T6 aluminum
- **Actuator mount:** XM430-W350-T mounted on one side of the U, output shaft through the opposite side
- **Position:** On top of turret, rotating with turret yaw
- **Axis:** Y (lateral) — pitches the upper arm forward/backward
- **Color:** Dark grey

### 3. Upper Arm (Shoulder to Elbow)
- **Structure:** Rectangular structural tube, 30mm wide × 30mm deep × 180mm long
- **Wall thickness:** 2mm (aluminum or PA-CF 3D printed)
- **Cross braces:** 1× horizontal stiffener at midpoint (internal web)
- **LED strip channel:** Flat groove on outward-facing side, 15mm wide × 2mm deep
- **Top end:** Bolts to shoulder bracket via 4× M2.5 through the XM430 output horn
- **Bottom end:** Houses elbow pitch actuator bracket
- **Direction:** Extends upward (+Z) from shoulder joint in default pose, 180mm total
- **Color:** Olive green panels (0.23, 0.29, 0.25), dark grey end caps

### 4. Elbow Joint
- **Actuator:** XM430-W350-T mounted at bottom of upper arm
- **Housing:** Integrated into the upper/forearm junction — motor body in upper arm, output shaft drives forearm
- **Position:** 180mm above shoulder joint (along arm axis)
- **Axis:** Y (lateral) — bends the forearm
- **Rotation range:** 0° to +150° (forearm can fold back against upper arm)
- **Bracket:** Same U-bracket style as shoulder, 50mm wide × 35mm tall × 25mm deep

### 5. Forearm (Elbow to Wrist)
- **Structure:** Rectangular structural tube, 30mm wide × 30mm deep × 180mm long (same as upper arm)
- **Wall thickness:** 2mm
- **Top end:** Bolts to elbow bracket
- **Bottom end:** Houses wrist pitch actuator
- **Internal:** Cable routing channel for gripper motor power + TTL
- **Direction:** Extends from elbow joint, 180mm total
- **Color:** Olive green panels (0.23, 0.29, 0.25), dark grey end caps

### 6. Wrist Joint
- **Actuator:** XL430-W250-T mounted at bottom of forearm
- **Housing:** Cylindrical shell, OD=40mm, height=50mm
- **Camera mount:** Small cylinder (20mm dia, 10mm deep) on the dorsal face for wrist camera (OAK-D or similar)
- **Position:** 180mm below elbow joint (along forearm axis)
- **Axis:** Y (lateral) — pitches the gripper for approach angle
- **Rotation range:** ±90°
- **Color:** Dark grey

### 7. Gripper Assembly
- **Type:** 2-finger parallel/radial gripper, mechanically actuated
- **Actuator:** XL430-W250-T (drives linkage mechanism)
- **Gripper body:** Rectangular housing, 40mm wide × 40mm deep × 30mm tall, dark grey
- **Fingers:**
  - 2× CNC aluminum fingers, each 70mm long × 15mm wide × 5mm thick
  - Curved inward profile (slight concavity for cylindrical objects)
  - Silicone grip pads at tips: 15mm × 10mm × 3mm, Shore A 20-30
- **Max opening:** 60mm (handles bottles, cans, most litter items)
- **Grip force:** ~50N max (XL430 stall torque through linkage)
- **Actuation:** Simple pivot linkage — motor rotates center cam, fingers open/close symmetrically
- **Position:** At bottom of wrist, extending 100mm from wrist axis
- **Color:** Dark grey body, silver/grey fingers, black silicone tips

### 8. Cable Routing
- **Path:** Cables enter turret base bottom (60mm ID bore), route up through turret, into shoulder bracket, down through upper arm tube interior, through elbow bracket, down forearm tube interior, into wrist housing, to gripper motor
- **Cable types:** 12V power (2× 18AWG), TTL bus (3-wire), wrist camera USB
- **Daisy chain:** All 5 Dynamixel motors share a single TTL bus (3-pin: VCC, GND, DATA)
- **Cable management:** Zip-tie anchors every 50mm inside structural tubes

## Assembly Dimensions Summary

| Measurement | Value |
|-------------|-------|
| Turret diameter | 80mm |
| Turret height | 50mm |
| Upper arm length | 180mm |
| Forearm length | 180mm |
| Wrist height | 50mm |
| Gripper length | 100mm |
| Total arm reach (horizontal) | ~460mm (shoulder to gripper tip) |
| Total arm height (vertical, extended up) | ~560mm above body (50 turret + 180 upper + 180 forearm + 50 wrist + 100 gripper) |
| Arm cross-section (tubes) | 30mm × 30mm |
| Gripper max opening | 60mm |
| Total arm mass | 1.2 kg |

## Default Pose (for CAD model)

Arm in "reaching forward and slightly down" pose (the signature render pose):
- Turret yaw: 0° (forward)
- Shoulder pitch: +90° (arm horizontal, pointing forward)
- Elbow pitch: +30° (slight bend downward)
- Wrist pitch: -30° (angled down for pickup)
- Gripper: 30° open (half-open, ready to grab)

This gives a reaching pose where the gripper is approximately at body height, extended forward ~400mm, as if approaching litter on the ground.

**Alternative pose for standing render:** All joints at 0° — arm pointing straight up (compact stow position).

## Body Plate Context

Include a simplified body plate (600×150×5mm at Z=-5 to Z=0) below the turret for visual reference, similar to the bag system model. The turret center should be at X=150mm from body front edge (X=0), Y=0 (centered), Z=60mm (top of body — but for simplified context, place turret at Z=0 on the plate and note the body height offset).

For simplified CAD context: place the turret directly on the body plate surface at Y=100mm (front third of the 600mm plate), centered at X=0.
