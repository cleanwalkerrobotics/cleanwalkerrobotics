# CleanWalker Robotics -- Hardware BOM & Cost Research

**Date:** 2026-02-09
**Author:** Hardware Research Lead
**Revision:** 1.0

---

## Executive Summary

This document provides a comprehensive Bill of Materials (BOM) cost breakdown for the CleanWalker autonomous litter-collecting quadruped robot. All prices are sourced from real suppliers as of early 2026 and reflect actual market pricing. The report covers prototype (1 unit), low-volume (10 units), mid-volume (100 units), and production (1,000 units) cost projections.

**Headline numbers:**
- Prototype unit (1 unit): **~$10,200 -- $14,600** (depending on actuator and sensor choices)
- Production unit at 100 units: **~$6,300 -- $8,800**
- Production unit at 1,000 units: **~$4,500 -- $6,200**

---

## Table of Contents

1. [Locomotion -- Quadrupedal Walking Platform](#1-locomotion--quadrupedal-walking-platform)
2. [Manipulation -- Gripper Arm](#2-manipulation--gripper-arm)
3. [Perception -- Vision + LiDAR](#3-perception--vision--lidar)
4. [Compute -- Edge AI Board](#4-compute--edge-ai-board)
5. [Power -- Battery System](#5-power--battery-system)
6. [Storage -- Compacting Bin](#6-storage--compacting-bin)
7. [Communications](#7-communications)
8. [PCB -- Build vs Buy](#8-pcb--build-vs-buy)
9. [Frame + Enclosure](#9-frame--enclosure)
10. [Total BOM Summary](#10-total-bom-summary)

---

## 1. Locomotion -- Quadrupedal Walking Platform

### 1.1 Background and Architecture

A quadruped requires a minimum of 12 degrees of freedom (3 DOF per leg: hip abduction/adduction, hip flexion/extension, knee flexion/extension). The CleanWalker will carry a payload (bin + arm + electronics) of approximately 5--10 kg and weigh roughly 15--25 kg total. This dictates mid-range actuator torque requirements (8--25 Nm peak per joint).

**Reference platforms studied:**
- **MIT Mini Cheetah** -- Custom quasi-direct-drive (QDD) actuators, ~$75 motor + custom gearbox per joint
- **Unitree Go2** -- Custom 36-slot/42-pole BLDC outrunner with ~6:1 planetary gearbox, 45 Nm peak, 28.8V system ([Simplexity teardown](https://www.simplexitypd.com/blog/unitree-go2-motor-teardown/))
- **ODRI Solo12** -- T-Motor Antigravity 4004 (300KV) + 9:1 timing belt transmission, total BOM ~$3,000--$5,000 ([ODRI Forum](https://odri.discourse.group/t/rough-cost-to-build-the-solo-12-robot/221))
- **Stanford Pupper** -- JX-Servo CLS6336HV servos, total BOM ~$900--$1,000 ([Stanford Robotics Club](https://stanfordstudentrobotics.org/pupper))
- **SpotMicro** -- Hobby servos, total BOM ~$250--$500, insufficient for outdoor use

### 1.2 Actuator Options

#### Option A: CubeMars AK Series (Recommended for Prototype)

| Model | Rated Torque | Peak Torque | Weight | Voltage | Price (1 unit) | Source |
|-------|-------------|-------------|--------|---------|---------------|--------|
| AK60-6 V1.1 KV80 | 6 Nm | 9 Nm | 305 g | 24V | $299 | [CubeMars Store](https://store.cubemars.com/products/ak60-6-v1-1-kv80) |
| AK70-10 KV100 (with driver) | 10 Nm | 24.8 Nm | 500 g | 48V | $499 | [CubeMars Store](https://store.cubemars.com/products/ak70-10) |
| AK80-9 V3.0 KV100 | 9 Nm | 22 Nm | 485 g | 48V | $580 | [CubeMars Store](https://store.cubemars.com/collections/ak-series-robotic-actuation-module) |

**Recommendation:** AK70-10 for hip and knee joints (high torque needed), AK60-6 for hip abduction/adduction (lower torque needed).

- 12x actuators: 8x AK70-10 ($499 ea) + 4x AK60-6 ($299 ea) = **$5,188** for prototype
- MOQ: 1 unit; bulk pricing (50+ units) typically 10--15% discount via CubeMars sales team
- At 100 units: estimated ~$420/AK70-10, ~$250/AK60-6 = **$4,360** per robot

#### Option B: MyActuator RMD-X Series

| Model | Rated Torque | Price (1 unit) | Source |
|-------|-------------|---------------|--------|
| RMD-X6-H 1:6 | ~6 Nm | $255 | [RobotShop](https://www.robotshop.com/products/myactuator-rmd-x6-v3-can-bus-18-mc-x-300-o-brushless-servo-driver) |
| RMD-X8 1:6 | ~18 Nm | $450 | [Amazon](https://www.amazon.com/MyActuator-Servo-Actuator-System-RMD-X8/dp/B0CD22Q6GV) |

- 12x actuators: 8x RMD-X8 ($450) + 4x RMD-X6-H ($255) = **$4,620**
- At 100 units: estimated ~$380/RMD-X8, ~$210/RMD-X6-H = **$3,880** per robot

#### Option C: Dynamixel XM540 Series (Simpler but Lower Performance)

| Model | Stall Torque | Price (1 unit) | Source |
|-------|-------------|---------------|--------|
| XM540-W270-R | 10.6 Nm | $494 | [ROBOTIS US](https://robotis.us/dynamixel-xm540-w270-r/) |
| XM540-W270-T | 10.6 Nm | $483 | [ROBOTIS US](https://www.robotis.us/dynamixel-xm540-w270-t/) |

- 12x XM540-W270-T = **$5,796**
- At 100 units: ~$420 each (ROBOTIS offers academic/volume discounts) = **$5,040**
- Pro: Excellent SDK/ecosystem, daisy-chain wiring. Con: Not designed for dynamic locomotion, may overheat under continuous load.

### 1.3 Actuator Recommendation

**Prototype phase: Option A (CubeMars AK60-6 + AK70-10 mix)** -- Best balance of performance, price, and community documentation for quadruped use. The AK series is used by multiple research labs and has CAN bus communication, integrated driver boards, and field-oriented control built in.

**Production phase: Option B (MyActuator RMD-X series)** -- Better volume pricing and wider gear ratio options. The X8 and X6 lines have good CAN bus support and are increasingly used in commercial quadrupeds.

### 1.4 Actuator Cost Summary

| Phase | Cost (12 actuators) |
|-------|-------------------|
| Prototype (1 unit) | $5,188 |
| 10 units | $4,800 per robot |
| 100 units | $4,360 per robot |
| 1,000 units | $3,500 per robot (est. direct OEM negotiation) |

---

## 2. Manipulation -- Gripper Arm

### 2.1 Requirements

- 2-DOF arm with gripper end effector
- Must pick up litter items ranging from cigarette butts to plastic bottles (~5g to ~500g)
- Soft-touch end effector to handle irregular shapes without crushing
- Reach: ~30--40 cm from robot body

### 2.2 Options

#### Option A: Custom Servo-Driven Arm + Soft Gripper (Recommended)

| Component | Specification | Price | Source |
|-----------|--------------|-------|--------|
| 2x Dynamixel XM430-W350-T (arm joints) | 4.1 Nm stall torque | $270 each | [ROBOTIS US](https://robotis.us/xm/) |
| Aluminum arm linkages (custom CNC) | 6061-T6 aluminum | $80--150 | [Xometry](https://www.xometry.com/capabilities/cnc-machining-service/) |
| Mechanical gripper with silicone-tipped fingers | Custom CNC + silicone tips | $40--80 | Steel fingers + silicone grip pads (Smooth-On) |
| Gripper servo (1x MG996R or Dynamixel XL430) | Gripper open/close | $45 (XL430) | [ROBOTIS US](https://robotis.us/) |

**Total per unit (prototype):** ~$705
**Total per unit (100 units):** ~$480 (volume pricing on Dynamixels, batch CNC)

#### Option B: LOBOT 2DOF Metal Arm + Custom Soft End Effector

| Component | Price | Source |
|-----------|-------|--------|
| LOBOT 2DOF Metal RC Robot Arm with servos | $36 | [Banggood](https://usa.banggood.com/LOBOT-2DOF-Metal-RC-Robot-Arm-Gripper-With-Digital-Servo-p-1499365.html) |
| Upgraded servos (2x MG996R) | $20 | Amazon |
| Custom soft gripper fingers | $40--80 | Custom |

**Total per unit (prototype):** ~$130
- Pro: Very cheap. Con: Insufficient for reliable outdoor use; hobby-grade servos will fail under continuous operation.

#### Option C: Off-Shelf Industrial Gripper (Cost Prohibitive)

| Product | Price | Note |
|---------|-------|------|
| Robotiq 2F-85 | ~$5,000--$6,000 | Massively overspecced for litter |
| DH Robotics AG-105-145 | ~$3,000 | [Devonics comparison](https://www.devonics.com/post/robotiq-alternative) |
| OnRobot Soft Gripper | ~$3,500 | Food-grade, not needed |

These are designed for industrial cobots and are cost-prohibitive for our application.

### 2.3 Soft Gripper Design Notes

For litter picking, a custom soft gripper is optimal:
- Silicone fingers (Shore A 20--30) can conform to irregular shapes
- 3-finger radial design grasps bottles, cans, wrappers
- Material cost: ~$5--15 per set using Smooth-On Ecoflex or Dragon Skin silicone
- Mold: 3D printed, $10--20 per mold
- Replaceable fingers reduce maintenance cost

### 2.4 Gripper Arm Cost Summary

| Phase | Cost |
|-------|------|
| Prototype (1 unit) | $705 |
| 10 units | $600 per robot |
| 100 units | $480 per robot |
| 1,000 units | $350 per robot |

---

## 3. Perception -- Vision + LiDAR

### 3.1 Requirements

- Object detection (litter classification): YOLO-based, requires color + depth
- SLAM / path planning: depth data for obstacle avoidance
- Outdoor operation: sun-resilient, IP54+ desirable
- Detection range: 0.3--10 m

### 3.2 Stereo Camera Options

| Camera | Resolution | Depth Range | IMU | Price | Source |
|--------|-----------|-------------|-----|-------|--------|
| Intel RealSense D435i | 1280x720 stereo | 0.2--10 m | Yes (BMI055) | $334 | [Intel Store](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435i.html) |
| Luxonis OAK-D Lite | 640x480 stereo | 0.2--9 m | No | $149 | [Luxonis Shop](https://shop.luxonis.com/products/oak-d-lite-1) |
| Luxonis OAK-D Pro | 1280x800 stereo | 0.2--15 m | Yes | $399 | [Luxonis Shop](https://shop.luxonis.com/products/oak-d-pro) |
| Stereolabs ZED 2i | 2208x1242 stereo | 0.3--20 m | Yes (ICM-42688) | $499 | [Stereolabs Store](https://store.stereolabs.com/products/zed-2i) |

**Recommendation for v1 prototype:** **OAK-D Pro ($399)** -- On-device neural network inference (4 TOPS) offloads compute from the Jetson, active stereo with IR projectors for outdoor use, ROS2 support. The built-in AI capability means the camera can run lightweight litter detection models independently.

**Budget alternative:** **OAK-D Lite ($149)** -- Adequate for initial prototyping. Lacks IR projector (struggles in direct sunlight). Good for indoor lab testing.

### 3.3 LiDAR Options

| LiDAR | FOV | Range | Points/sec | Weight | Price | Source |
|-------|-----|-------|-----------|--------|-------|--------|
| Livox Mid-360 | 360deg H / 59deg V | 40 m | 200,000 | 265 g | $979 | [DJI Store](https://store.dji.com/product/livox-mid-360) |
| RPLidar A1 (2D) | 360deg H | 12 m | 8,000 | 170 g | $99 | Various |
| RPLidar S2E (2D) | 360deg H | 30 m | 32,000 | 185 g | $249 | Various |

### 3.4 Can We Skip LiDAR for v1?

**Yes, for the initial prototype.** Rationale:
- The OAK-D Pro provides usable depth data to 15 m, sufficient for park/sidewalk navigation
- SLAM can be done with visual-inertial odometry (VIO) using the stereo camera + IMU
- Adding a 2D RPLidar A1 ($99) as a safety layer for 360-degree obstacle detection is a sensible compromise
- Full 3D LiDAR (Livox Mid-360) should be added in v2 for robust all-weather operation

### 3.5 Perception Cost Summary

| Configuration | Prototype | 100 units |
|--------------|-----------|-----------|
| **v1 Minimum:** OAK-D Lite + RPLidar A1 | $248 | $200 |
| **v1 Recommended:** OAK-D Pro + RPLidar A1 | $498 | $400 |
| **v2 Full:** OAK-D Pro + Livox Mid-360 | $1,378 | $1,150 |

**Recommendation:** v1 Recommended configuration ($498 prototype)

---

## 4. Compute -- Edge AI Board

### 4.1 Requirements

- Real-time YOLO inference (YOLOv8n/v10n) at 15+ FPS on 640x640 input
- ROS2 Humble/Jazzy support
- Path planning (Nav2 stack) concurrently with perception
- Motor control communication (CAN bus)
- Camera and sensor I/O (USB 3.0, CSI)
- Power efficiency: <15W total compute power budget

### 4.2 Options

| Board | AI Performance | GPU | CPU | RAM | Power | Price | Source |
|-------|---------------|-----|-----|-----|-------|-------|--------|
| Jetson Orin Nano Super (8GB) | 67 TOPS | 1024-core Ampere | 6-core A78AE | 8 GB | 7--25W | $249 | [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) |
| Jetson Orin NX 16GB (module) | 100 TOPS | 1024-core Ampere | 8-core A78AE | 16 GB | 10--25W | ~$599 (module) | [Arrow](https://www.arrow.com/en/products/900-13767-0000-000/nvidia), [CDW](https://www.cdw.com/product/nvidia-jetson-orin-nx-16gb-module/7340982) |
| Raspberry Pi 5 (8GB) + Hailo-8 | 26 TOPS (Hailo) | VideoCore VII | 4-core A76 | 8 GB | 5--12W | $85 + $200 = $285 | [RPi](https://www.raspberrypi.com/products/raspberry-pi-5/), [Waveshare](https://www.waveshare.com/hailo-8.htm) |
| Raspberry Pi 5 + Coral USB | 4 TOPS (Coral) | VideoCore VII | 4-core A76 | 8 GB | 5--10W | $85 + $75 = $160 | [RPi](https://www.raspberrypi.com/products/raspberry-pi-5/), [Amazon](https://www.amazon.com/Google-G950-01456-01-Coral-USB-Accelerator/dp/B07S214S5Y) |

### 4.3 Performance Analysis

**Minimum viable compute for our workload:**
- YOLOv8n at 640x640: requires ~6--8 TOPS for 15 FPS
- Nav2 path planning: moderate CPU load, ~2 cores
- Motor control loop (1kHz): 1 dedicated core
- Sensor fusion: 1 core

The **Jetson Orin Nano Super** at $249 is the sweet spot:
- 67 TOPS handles YOLOv8n at 30+ FPS easily
- Full CUDA/TensorRT support for model optimization
- Native ROS2 support via JetPack SDK
- 8 GB RAM is sufficient for our stack
- Carrier board included in the dev kit

The **Raspberry Pi 5 + Hailo-8** ($285) is a viable budget alternative but requires more software integration work and has no CUDA support for custom model development.

### 4.4 Carrier Board for Production

For production, the Jetson module needs a carrier board:
- **Seeed Studio reComputer J4012** (Orin NX 16GB + carrier + SSD): ~$699 ([Seeed Studio](https://www.seeedstudio.com/reComputer-J4012-p-5586.html))
- **Connect Tech carrier boards**: $200--$400 ([Connect Tech](https://connecttech.com/product/nvidia-jetson-orin-nx-16gb-module/))
- **Custom carrier board** (JLCPCB fabrication): ~$50--100 per board in volume

### 4.5 Compute Cost Summary

| Phase | Cost |
|-------|------|
| Prototype (1 unit) -- Orin Nano Super Dev Kit | $249 |
| 10 units -- Orin Nano Super Dev Kit | $249 per robot |
| 100 units -- Orin Nano module + custom carrier | $200 (module) + $80 (carrier) = $280 |
| 1,000 units -- Orin Nano module + custom carrier | $150 (module, volume) + $40 (carrier) = $190 |

---

## 5. Power -- Battery System

### 5.1 Power Budget Estimation

| Subsystem | Typical Draw | Peak Draw |
|-----------|-------------|-----------|
| 12x Actuators (walking) | 120W avg | 400W peak (jumping/recovery) |
| Compute (Jetson Orin Nano) | 15W | 25W |
| Sensors (cameras, LiDAR, IMU) | 10W | 15W |
| Gripper arm (2 servos) | 5W | 20W |
| Compaction mechanism | 0W (intermittent) | 30W |
| Communications (4G + WiFi) | 3W | 5W |
| Misc (fans, LEDs, status) | 5W | 10W |
| **Total** | **~158W average** | **~505W peak** |

**Target runtime:** 4--6 hours continuous operation
**Energy required:** 158W x 5h = 790 Wh (with 20% margin = ~950 Wh)

Note: The Unitree Go2 uses a 236.8 Wh battery (8000 mAh @ 28.8V) for 1--2 hours of runtime at roughly similar motor load but lighter weight. Our robot will be heavier and carry more payload, requiring proportionally more energy.

### 5.2 Battery Options

| Option | Voltage | Capacity | Energy | Weight | Price | Source |
|--------|---------|----------|--------|--------|-------|--------|
| 48V 20Ah Li-ion pack (e-bike type) | 48V | 20 Ah | 960 Wh | ~5.5 kg | $200--350 | [Amazon](https://www.amazon.com/48v-20ah-lithium-battery/s?k=48v+20ah+lithium+battery), [Aegis Battery](https://www.aegisbattery.com/products/aegis-48v-20ah-lithium-ion-battery-pack-nmc-48v-lithium-battery) |
| Custom 48V 21Ah 18650 pack (13S7P) | 48V | 21 Ah | 1008 Wh | ~5 kg | $350--500 (custom) | [CM Batteries](https://cmbatteries.com/project/48v-21ah-18650-battery-pack/) |
| 2x 24V 20Ah packs in series | 48V | 20 Ah | 960 Wh | ~6 kg | $300--400 | Various |
| Unitree Go2 battery (for reference) | 28.8V | 8 Ah | 236.8 Wh | 0.8 kg | $250 | [Unitree Shop](https://shop.unitree.com/products/go2-battery) |

### 5.3 Realistic Runtime Estimate with 48V 20Ah Pack

- Available energy: 960 Wh x 0.85 (depth of discharge) = 816 Wh usable
- At 158W average: 816 / 158 = **~5.2 hours**
- At 200W average (hilly terrain, frequent picking): 816 / 200 = **~4.1 hours**

This meets our 4--6 hour target.

### 5.4 Charging Dock Design

**Contact-based charging (recommended for v1):**
- Spring-loaded pogo pins on robot base, matching contacts on dock
- 48V 5A charger: ~$30--50
- Full charge time: 20 Ah / 5A = ~4 hours
- Dock mechanics: simple ramp + alignment guides, sheet metal or 3D printed
- Dock total BOM: ~$80--150

**Wireless charging (v2 consideration):**
- WiBotic systems: $2,000--5,000 per station ([WiBotic](https://www.wibotic.com/))
- Too expensive for v1

### 5.5 Power Cost Summary

| Phase | Cost (battery + BMS + charger) |
|-------|------|
| Prototype (1 unit) | $450 (off-shelf 48V 20Ah + charger) |
| 10 units | $380 per robot |
| 100 units | $280 per robot (custom pack from CM Batteries or similar) |
| 1,000 units | $180 per robot (direct cell sourcing + custom BMS) |

---

## 6. Storage -- Compacting Bin

### 6.1 Design Specifications

- Volume: ~20L (equivalent to a small kitchen trash bin)
- Compaction ratio: 3:1 target (reduces to ~7L equivalent per fill)
- Material: HDPE or aluminum inner bin, weatherproof
- Compaction mechanism: linear actuator + compression plate

### 6.2 Components

| Component | Specification | Price (1 unit) | Price (100 units) | Source |
|-----------|--------------|---------------|-------------------|--------|
| 12V linear actuator (100mm stroke, 1500N) | For compaction plate | $35--60 | $20--30 | [Robotistan](https://www.robotistan.com/12v-dc-100mm-linear-actuator-7mm/s-1500n), [Amazon](https://www.amazon.com/s?k=12v+linear+actuator+100mm) |
| HDPE bin shell (20L, custom) | Thermoformed or CNC'd | $40--80 | $15--25 | Custom fabrication |
| Compression plate (aluminum) | 6061-T6, CNC machined | $25--40 | $10--15 | [Xometry](https://www.xometry.com/capabilities/cnc-machining-service/) |
| Guide rails (2x linear rails) | MGN12H 100mm | $15--25 | $8--12 | Amazon / AliExpress |
| Bin door mechanism (servo + latch) | SG90 or similar | $5--10 | $3--5 | Amazon |
| Weatherproof gasket/seal | Silicone | $5--10 | $2--4 | McMaster-Carr |
| Fasteners, brackets, misc | | $15--25 | $8--12 | McMaster-Carr |

### 6.3 Bin Cost Summary

| Phase | Cost |
|-------|------|
| Prototype (1 unit) | $200 |
| 10 units | $160 per robot |
| 100 units | $100 per robot |
| 1,000 units | $65 per robot |

---

## 7. Communications

### 7.1 Requirements

- Cellular connectivity for fleet management, telemetry, and remote monitoring
- Local connectivity for debug/development (WiFi)
- Optional mesh networking for multi-robot coordination

### 7.2 Options

#### Cellular (Required)

| Module | Network | Interface | Price (1 unit) | Source |
|--------|---------|-----------|---------------|--------|
| SIM7600G-H (4G LTE Cat-4) | 4G LTE / 3G / 2G | UART/USB | $35--50 | [Amazon](https://www.amazon.com/SIM7600G-H-4G-HAT-Communication-Positioning/dp/B08ZY2FV22) |
| Waveshare SIM7600G-H HAT | 4G LTE + GNSS | USB/UART (RPi HAT) | $55--75 | [Waveshare](https://www.waveshare.com/sim7600x-h-m2.htm) |
| Quectel RM500Q-GL (5G) | 5G Sub-6GHz | M.2 | $150--200 (module) | [4GLTEMall](https://www.4gltemall.com/quectel-rm500q.html) |

**Recommendation:** SIM7600G-H for v1 -- 4G LTE is sufficient for telemetry data and low-res video streaming. 5G is overkill and adds cost.

- Antenna: external 4G antenna, $5--15
- SIM card slot: integrated on HAT/module
- Data plan: ~$10--15/month per robot (IoT plan)

#### Mesh Networking (Optional for v2)

| Module | Protocol | Range | Price | Source |
|--------|----------|-------|-------|--------|
| RAK811 LoRa module | LoRaWAN | 3+ km | $15--25 | [RAKwireless](https://store.rakwireless.com/products/rak811-lpwan-breakout-module) |
| ESP32 with 802.11s WiFi mesh | WiFi mesh | 50--100 m | $5--10 | Various |

### 7.3 Communications Cost Summary

| Phase | Cost |
|-------|------|
| Prototype (1 unit) | $80 (SIM7600G-H HAT + antenna) |
| 10 units | $65 per robot |
| 100 units | $45 per robot |
| 1,000 units | $30 per robot |

---

## 8. PCB -- Build vs Buy

### 8.1 PCB Requirements

The robot needs electronics for:
1. **Motor control**: CAN bus communication with 12 actuators
2. **Power distribution**: 48V main bus, regulated 24V/12V/5V rails
3. **Sensor integration**: USB hub, CSI camera interfaces
4. **Safety**: emergency stop, overcurrent protection, thermal monitoring
5. **I/O expansion**: GPIO for compaction actuator, bin sensors, status LEDs

### 8.2 Option A: Custom PCB (KiCad + JLCPCB)

| Item | Cost (5 boards) | Cost (100 boards) | Source |
|------|-----------------|-------------------|--------|
| PCB fabrication (4-layer, 100x150mm) | $7--15 per board | $3--5 per board | [JLCPCB](https://jlcpcb.com/) |
| SMT assembly (JLCPCB) | $50--100 per board | $15--25 per board | [JLCPCB Assembly](https://jlcpcb.com/pcb-assembly) |
| Components (BOM) | $40--80 per board | $25--40 per board | JLCPCB parts library |
| Engineering/design time | 80--120 hours | Amortized | Internal |

**Total per board (prototype run of 5):** $100--200
**Total per board (production run of 100):** $45--70

Pros: Compact, integrated, optimized for our specific needs
Cons: Engineering time, risk of design errors, 2--4 week lead time

### 8.3 Option B: Open-Source Motor Controller PCBs

| Board | Channels | Price | Source |
|-------|----------|-------|--------|
| moteus r4.11 | 1 motor | $79 (1--9 qty), $75 (10--99) | [mjbots](https://mjbots.com/products/moteus-r4-11) |
| ODrive S1 | 1 motor | ~$149 | [ODrive Shop](https://shop.odriverobotics.com/products/odrive-s1) |
| SimpleFOC Shield V2.0.4 | 1 motor | $9--15 | [eBay](https://www.ebay.com/itm/388173960952), [Amazon](https://www.amazon.com/SimpleFOC-Shield-V2-0-4-Controller-Arduino/dp/B0FDBRKCNV) |

**Note:** If using CubeMars AK series actuators, the driver is already integrated into the actuator module. No separate motor controller PCBs are needed -- just a CAN bus interface board.

For CubeMars-based design, the PCB need is reduced to:
- **Power distribution board** (48V to 12V/5V converters, CAN bus hub)
- **Interface board** (Jetson carrier adapter, sensor connectors)

### 8.4 Option C: Off-Shelf Dev Boards (Prototype Only)

| Board | Function | Price | Source |
|-------|----------|-------|--------|
| Jetson Orin Nano Super Dev Kit | Compute + carrier | $249 | Included in compute |
| USB CAN adapter (Canable) | CAN bus interface | $25--40 | [Canable](https://canable.io/) |
| 48V to 12V DC-DC converter | Power regulation | $15--25 | Amazon |
| 48V to 5V DC-DC converter | Power regulation | $10--15 | Amazon |
| USB 3.0 hub (powered) | Sensor connectivity | $15--25 | Amazon |
| Proto board + connectors | Misc wiring | $20--30 | Amazon |

**Total (dev board approach):** ~$85--135 for additional boards

### 8.5 PCB Recommendation

| Phase | Approach | Cost per Robot |
|-------|----------|---------------|
| Prototype (1--5 units) | **Option C** (dev boards) + CAN adapter | $120 |
| 10 units | **Option A** (custom PCB, small run) | $180 |
| 100 units | **Option A** (custom PCB, production) | $65 |
| 1,000 units | **Option A** (custom PCB, volume) | $35 |

---

## 9. Frame + Enclosure

### 9.1 Frame Options

The frame includes the main body chassis, leg structures, and mounting points for all subsystems.

#### Option A: CNC Machined Aluminum (Recommended for legs + structural)

| Component | Specification | Prototype Cost | 100-unit Cost |
|-----------|--------------|---------------|---------------|
| Main body frame | 6061-T6, CNC machined | $300--500 | $100--150 |
| 4x leg assemblies (upper + lower links) | 6061-T6, CNC machined | $400--600 (set) | $150--200 (set) |
| Motor mounting brackets (12x) | 6061-T6, CNC machined | $150--250 (set) | $50--80 (set) |
| Fasteners, bearings, pins | Standard | $50--80 | $25--40 |

**Source:** [Xometry](https://www.xometry.com/capabilities/cnc-machining-service/), [PCBWay CNC](https://www.pcbway.com/rapid-prototyping/manufacture/?type=1)
**Total prototype:** $900--1,430
**Total at 100 units:** $325--470

#### Option B: 3D Printed (Prototype/Testing Only)

| Component | Material | Prototype Cost |
|-----------|----------|---------------|
| Main body frame | PA12 Nylon (MJF/SLS) | $150--300 |
| Leg assemblies | PA12 Nylon | $200--400 |
| Motor brackets | PA12-CF (carbon-filled nylon) | $100--200 |

**Total prototype:** $450--900
- Pro: Fast iteration (3--5 day turnaround), cheaper for 1--5 units
- Con: Lower strength/stiffness than aluminum, UV degradation outdoors, not suitable for production

#### Option C: Carbon Fiber (Premium)

| Component | Prototype Cost |
|-----------|---------------|
| CF plates (CNC cut) + aluminum joints | $800--1,500 |
| CF tubes for legs | $200--400 |

**Total prototype:** $1,000--1,900
- Pro: Lightest option, highest stiffness-to-weight ratio
- Con: Expensive, difficult to repair, complex manufacturing

### 9.2 Weatherproof Enclosure (IP54)

The enclosure covers the electronics bay (compute, PCBs, battery) and the compacting bin.

| Approach | Prototype Cost | 100-unit Cost | Notes |
|----------|---------------|---------------|-------|
| 3D printed (PETG/ASA) + gaskets | $80--150 | N/A | Good for prototype only |
| Sheet metal (powder-coated aluminum) | $200--400 | $80--120 | [KDM Steel](https://www.kdmsteel.com/ip54-enclosure/) |
| Injection molded (ABS/PC) | $8,000--15,000 tooling + $5--10/part | $5--10/part | Only viable at 500+ units |

**Recommendation:**
- Prototype: 3D printed ASA enclosure with silicone gaskets ($120)
- 10 units: Sheet metal enclosure ($250 each)
- 100+ units: Sheet metal enclosure ($100 each)
- 1,000+ units: Injection molded ($10 each + amortized tooling = ~$25 effective)

### 9.3 Foot Pads / Terrain Contact

| Component | Cost (set of 4) |
|-----------|----------------|
| Rubber foot pads (custom molded) | $20--40 prototype, $8--15 at volume |
| Optional: point-contact feet (3D printed + rubber tip) | $10--20 |

### 9.4 Frame + Enclosure Cost Summary

| Phase | Frame | Enclosure | Feet | Total |
|-------|-------|-----------|------|-------|
| Prototype (1 unit) | $1,100 (CNC aluminum) | $120 (3D printed) | $30 | $1,250 |
| 10 units | $700 | $250 | $25 | $975 |
| 100 units | $400 | $100 | $12 | $512 |
| 1,000 units | $250 | $25 | $8 | $283 |

---

## 10. Total BOM Summary

### 10.1 Prototype Configuration (v1 Recommended)

| # | Component | Specific Parts | Cost (1 unit) |
|---|-----------|---------------|---------------|
| 1 | **Actuators (12x)** | 8x CubeMars AK70-10, 4x AK60-6 | $5,188 |
| 2 | **Gripper Arm** | 2x Dynamixel XM430 + custom soft gripper | $705 |
| 3 | **Perception** | OAK-D Pro + RPLidar A1 | $498 |
| 4 | **Compute** | Jetson Orin Nano Super Dev Kit | $249 |
| 5 | **Battery + Power** | 48V 20Ah Li-ion + charger + BMS | $450 |
| 6 | **Compacting Bin** | Linear actuator + HDPE bin + mechanism | $200 |
| 7 | **Communications** | SIM7600G-H HAT + antenna | $80 |
| 8 | **PCB / Electronics** | Dev boards + CAN adapter + DC-DC converters | $120 |
| 9 | **Frame + Enclosure** | CNC aluminum frame + 3D printed enclosure | $1,250 |
| | **Wiring, connectors, misc** | Cables, connectors, heat shrink, etc. | $150 |
| | | | |
| | **SUBTOTAL** | | **$8,890** |
| | **Contingency (15%)** | Unexpected costs, rework, shipping | **$1,334** |
| | **TOTAL PROTOTYPE** | | **$10,224** |

### 10.2 Scaling Cost Table

| Component | 1 unit | 10 units | 100 units | 1,000 units |
|-----------|--------|----------|-----------|-------------|
| Actuators (12x) | $5,188 | $4,800 | $4,360 | $3,500 |
| Gripper Arm | $705 | $600 | $480 | $350 |
| Perception | $498 | $470 | $400 | $320 |
| Compute | $249 | $249 | $280 | $190 |
| Battery + Power | $450 | $380 | $280 | $180 |
| Compacting Bin | $200 | $160 | $100 | $65 |
| Communications | $80 | $65 | $45 | $30 |
| PCB / Electronics | $120 | $180 | $65 | $35 |
| Frame + Enclosure | $1,250 | $975 | $512 | $283 |
| Wiring, connectors, misc | $150 | $120 | $80 | $50 |
| **Subtotal** | **$8,890** | **$7,999** | **$6,602** | **$5,003** |
| Contingency (15% proto, 10% production) | $1,334 | $800 | $660 | $500 |
| **Total per Unit** | **$10,224** | **$8,799** | **$7,262** | **$5,503** |

### 10.3 Cost Reduction Roadmap

| Strategy | Estimated Savings | Timeline |
|----------|------------------|----------|
| Custom actuator design (in-house BLDC + gearbox) | 40--50% on actuators (-$1,500--2,000/unit) | 12--18 months |
| Volume OEM pricing on actuators (1000+ qty) | 25--30% (-$800--1,200/unit) | 6 months |
| Injection molded enclosure (amortized at 1000 units) | 75% on enclosure (-$75/unit) | 6--9 months |
| Custom Jetson carrier board | 50% on compute electronics (-$30--50/unit) | 3--6 months |
| Direct cell sourcing for battery packs | 30--40% on battery (-$80--120/unit) | 3 months |

### 10.4 Long-Term Production Target

With the cost reduction strategies above fully realized at 5,000+ unit volumes, the target BOM should reach **$3,500--$4,000 per unit**, with actuators remaining the single largest cost driver at approximately 50--60% of BOM.

---

## Appendix A: Supplier Contact List

| Supplier | Components | Website | Notes |
|----------|-----------|---------|-------|
| CubeMars (T-Motor) | AK series actuators | [cubemars.com](https://store.cubemars.com/) | Volume pricing via sales@cubemars.com |
| MyActuator | RMD-X series actuators | [myactuator.com](https://www.myactuator.com/) | Alternative actuator supplier |
| ROBOTIS | Dynamixel servos | [robotis.us](https://robotis.us/) | Gripper arm servos |
| Luxonis | OAK-D cameras | [shop.luxonis.com](https://shop.luxonis.com/) | Stereo depth + on-device AI |
| NVIDIA | Jetson modules | [developer.nvidia.com](https://developer.nvidia.com/buy-jetson) | Compute platform |
| JLCPCB | PCB fabrication + assembly | [jlcpcb.com](https://jlcpcb.com/) | Custom PCBs at low cost |
| Xometry | CNC machining | [xometry.com](https://www.xometry.com/) | Frame parts, instant quoting |
| PCBWay | CNC machining + PCBs | [pcbway.com](https://www.pcbway.com/) | Alternative to Xometry |
| Aegis Battery | Custom battery packs | [aegisbattery.com](https://www.aegisbattery.com/) | 48V Li-ion packs |
| SIMCom | SIM7600 modules | [simcom.com](https://www.simcom.com/) | 4G LTE cellular |
| RAKwireless | LoRa modules | [store.rakwireless.com](https://store.rakwireless.com/) | Mesh networking |
| Livox (DJI) | Mid-360 LiDAR | [livoxtech.com](https://www.livoxtech.com/) | 3D LiDAR for v2 |

## Appendix B: Key Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Actuator overheating during sustained outdoor operation | Motor derating, reduced runtime | Thermal modeling, duty cycle limits, aluminum heat sinks on motor housings |
| Battery degradation in high temperatures | Reduced capacity over time | Temperature-controlled battery compartment, LiFePO4 cells for hot climates |
| IP54 seal failure | Water ingress, electronics damage | Rigorous environmental testing, double gasket design, conformal coating on PCBs |
| CAN bus reliability with 12+ nodes | Communication dropouts | Redundant CAN channels, watchdog timers, graceful degradation mode |
| Soft gripper wear | Frequent replacement of silicone fingers | Design for quick replacement (snap-fit), stock replacement sets ($5--15 per set) |

---

*This document is a living research artifact and will be updated as vendor quotes are received and design decisions are finalized. All prices are in USD and were researched in February 2026. Actual costs may vary based on exchange rates, shipping, import duties, and vendor negotiations.*
