# CleanWalker Robotics -- Assembly, Iteration & Total Prototype Cost Report

**Date:** 2026-02-09
**Author:** Research Team
**Revision:** 1.0
**BOM Reference:** ~$10,200 per prototype unit (components only, from hardware-bom-research.md)

---

## Executive Summary

The component BOM of ~$10,200 represents only **40-55% of the true cost** of getting a pilot-ready prototype. Assembly labor, tooling, workspace setup, engineering time, iteration cycles, and pre-certification testing add $8,000-$15,000+ per unit for the first few prototypes, declining sharply as designs stabilize.

**Headline total cost estimates to reach pilot-ready (v1.0):**

| Approach | Total Spend (all-in, through pilot-ready) |
|----------|-------------------------------------------|
| Scrappy (founder-led, AI-assisted) | $85,000 - $120,000 |
| Professional (contract engineers + CM) | $250,000 - $400,000 |
| Recommended (hybrid) | $130,000 - $200,000 |

These figures include all prototype units built (estimated 5-8 units across iterations), all engineering time, all tooling, workspace, and pre-certification testing.

---

## Table of Contents

1. [Assembly Labor Costs](#1-assembly-labor-costs)
2. [Prototype Iteration Costs](#2-prototype-iteration-costs)
3. [Tooling & Equipment Costs](#3-tooling--equipment-costs)
4. [Engineering Time Costs](#4-engineering-time-costs)
5. [Testing & Validation Costs](#5-testing--validation-costs)
6. [Summary Cost Table](#6-summary-cost-table)

---

## 1. Assembly Labor Costs

### 1.1 Assembly Time Estimate for CleanWalker

Based on analysis of open-source quadruped builds (ODRI Solo-12, Stanford Pupper, SpotMicro community) and the Unitree Go2 teardown by [Simplexity Product Development](https://www.simplexitypd.com/blog/unitree-go2-quadruped-robot-dog-teardown/), a quadrupedal robot of CleanWalker's complexity involves the following assembly stages:

| Assembly Phase | Estimated Hours | Notes |
|----------------|----------------|-------|
| **Mechanical assembly** (frame, leg linkages, motor mounting, bearings, fasteners) | 16-24 hrs | CNC aluminum frame with 12 actuator mounts, 4 leg assemblies, bin mechanism. Reference: Solo-12 mechanical assembly is documented at ~8-12 hrs for a smaller/simpler robot. |
| **Electrical assembly** (wiring harness, power distribution, CAN bus connections, sensor mounting) | 12-20 hrs | 12 actuators on CAN bus, power distribution from 48V battery, DC-DC converters, Jetson mounting, camera/LiDAR mounting, cellular module. |
| **Gripper arm assembly** | 4-6 hrs | 2-DOF arm with Dynamixel servos, soft gripper, cabling. |
| **Firmware flashing & software deployment** | 4-8 hrs | Jetson OS/JetPack installation, ROS2 stack, motor controller configuration, sensor driver setup, network configuration. |
| **Initial calibration** | 4-8 hrs | Joint zero calibration, IMU calibration, camera intrinsics/extrinsics, CAN bus ID assignment, motor parameter tuning. |
| **QA & functional testing** | 8-16 hrs | Motor spin-up tests, CAN communication verification, sensor data validation, power draw measurement, basic locomotion test, gripper test, thermal monitoring. |
| **Total per unit** | **48-82 hrs** | **Estimated midpoint: ~60 hours** |

For subsequent units (once the first unit has been debugged and procedures documented), assembly time drops to approximately **40-55 hours** per unit.

### 1.2 Labor Rate Benchmarks

| Worker Type | US Hourly Rate | Source |
|-------------|---------------|--------|
| Assembly technician (entry-level) | $22-28/hr | [ZipRecruiter](https://www.ziprecruiter.com/Salaries/Robot-Technician-Salary) |
| Skilled robotics technician | $29-36/hr | [Salary.com](https://www.salary.com/research/salary/posting/robotics-technician-hourly-wages) |
| Senior robotics technician / lead | $34-40/hr | [PayScale](https://www.payscale.com/research/US/Job=Robotics_Technician/Hourly_Rate) |
| Contract manufacturer burdened rate (US) | ~$40/hr | Industry average, includes overhead |
| System integrator rate | $100-200/hr | Professional integration services |

**Note:** The "burdened rate" from a contract manufacturer includes facility overhead, tooling amortization, insurance, QA overhead, and management costs on top of the direct labor wage.

### 1.3 Assembly Cost Per Unit

| Approach | Hours | Rate | Cost per Unit |
|----------|-------|------|---------------|
| **Self-assembly (founder)** | 60 hrs | $0 direct (opportunity cost: ~$75/hr) | $0 cash / $4,500 opportunity cost |
| **Hire a technician** | 60 hrs | $30-36/hr | $1,800 - $2,160 |
| **Contract manufacturer (US)** | 60 hrs | $40/hr burdened | $2,400 |
| **Contract manufacturer (specialized robotics, e.g., Simplexity, PEKO, Mack)** | 55 hrs | $60-80/hr (includes engineering support) | $3,300 - $4,400 |

### 1.4 Contract Manufacturer Options

**US-Based Robotics Contract Manufacturers:**

| Company | Specialization | Min Volume | Notes |
|---------|---------------|-----------|-------|
| [Mack Molding](https://www.mack.com/markets/robotics/) | Full electromechanical assembly, plastic + metal + electronics integration | 5+ units | Vermont-based; handles volumes from 5 to 5M units |
| [PEKO Precision](https://www.pekoprecision.com/) | Electromechanical assembly, prototype to low-volume | 1+ units | Flexible, scalable; vertically integrated |
| [Fathom Manufacturing](https://fathommfg.com/robotics-industry) | Low-volume/high-mix robotics builds | 1+ units | Hands-on DFM approach, good for new platforms |
| [Simplexity Product Development](https://www.simplexitypd.com/) | Full product development + NPI | 1+ units | Premium: engineering + manufacturing, San Diego |
| [Applied Engineering](https://www.appliedengineering.com/) | Robotic system assembly | 5+ units | Experienced technicians, scalable builds |

**Overseas (China/Shenzhen):**
- Typical burdened labor rate: $12-20/hr
- Relevant for production (100+ units), not ideal for initial prototypes due to communication overhead, iteration speed, and IP concerns
- Lead time: 4-8 weeks per iteration vs. 1-2 weeks US-based

### 1.5 Assembly Cost Recommendation

**For prototypes 1-5: Self-assemble (founder + hired technician).**
- Reason: You need to deeply understand every subsystem. Contract manufacturers cannot debug novel designs -- that requires engineering judgment. Hiring one skilled technician ($30-36/hr) to assist the founder is the best use of capital.
- Cost per unit: ~$1,800-$2,200 in labor (technician hours only)
- Founder time: ~30-40 hrs per unit (unpaid but critical for learning)

**For units 6-20: Transition to contract manufacturer.**
- Once assembly procedures are documented and the design is stable, hand off to a US CM like Mack Molding or PEKO.
- Cost per unit: $2,400-$4,400 in assembly labor

---

## 2. Prototype Iteration Costs

### 2.1 How Many Iterations to Pilot-Ready?

Hardware development follows the standard EVT-DVT-PVT pipeline. Based on real-world robotics company histories:

**Reference company iteration counts:**

| Company | Product | Prototype Generations | Years to Production | Source |
|---------|---------|----------------------|--------------------|----|
| Unitree | Quadruped series | 6+ major versions (A1 -> Go1 -> Go2 -> B2 -> Go2-W -> H1) | 2016-2023 (~7 years from founding to mature product line) | [IEEE Spectrum](https://spectrum.ieee.org/quadruped-robot-unitree-go2) |
| Agility Robotics | Digit | 5+ versions (ATRIAS -> Cassie -> Digit v1 -> v2 -> v3 -> v4) | 2015-2023 (~8 years from founding) | [Wikipedia](https://en.wikipedia.org/wiki/Agility_Robotics) |
| Starship Technologies | Delivery robot | Multiple named versions (6C -> 6E, 6-wheel -> 8-wheel, continuous iteration) | 2014-2017 (3 years to commercial service) | [Starship About](https://www.starship.xyz/about/) |
| Boston Dynamics | Spot | 10+ years of BigDog -> LS3 -> Spot Classic -> Spot (commercial) | 2005-2020 (~15 years, but they were also inventing the field) | Public knowledge |

**Realistic estimate for CleanWalker (starting with known actuators, proven compute, existing open-source locomotion algorithms):**

| Phase | Versions | Units Built | Focus |
|-------|----------|-------------|-------|
| **POC / v0.1** | 1 | 1 | Basic walking. Prove 12-DOF locomotion works with selected actuators. No arm, no perception. 3D-printed frame. |
| **EVT / v0.2-v0.4** | 2-3 | 2-3 | Add perception, arm, bin. CNC aluminum frame. Iterate on mechanical fit, wiring, thermal management. Each sub-version addresses failures found in testing. |
| **DVT / v0.5-v0.7** | 2-3 | 3-5 | Design for outdoor use: IP54+ sealing, ruggedized connectors, proper cable routing. Weatherproofing. Field trials. Software maturation. |
| **PVT / v0.8-v1.0** | 1-2 | 3-5 | Production-intent design. Assembly procedure documentation. Final DFM. Pilot units for municipality testing. |
| **Total** | **6-9 versions** | **9-14 units** | **Estimated 12-18 months** |

### 2.2 Cost Per Iteration Type

Not every iteration requires a full new build. Here is the cost breakdown by iteration type:

#### A. Frame / Mechanical Redesign

| Item | Prototype Cost | Notes |
|------|---------------|-------|
| New CNC aluminum frame (Xometry/PCBWay) | $800-1,400 | Major body redesign; 5-7 day turnaround from Xometry |
| New CNC leg linkages (set of 8 parts) | $300-600 | Leg geometry change |
| New 3D-printed parts (enclosure, brackets) | $50-200 | In-house with Bambu Lab P1S; material cost only |
| New motor mounting brackets | $100-250 | If actuator mounting points change |
| Fasteners, bearings, hardware | $30-80 | New sizes/types needed |
| **Typical mechanical iteration** | **$500 - $1,500** | Partial redesign (most common) |
| **Full mechanical redesign** | **$1,300 - $2,500** | New frame + new legs + new brackets |

**Turnaround time:** 5-10 business days for CNC parts (Xometry expedited), 1-2 days for 3D prints.

#### B. PCB / Electronics Revision

| Item | Cost (5-board run) | Notes |
|------|-------------------|-------|
| PCB fabrication (4-layer, JLCPCB) | $7-15/board | [JLCPCB](https://jlcpcb.com/) -- as low as $2 for simple 2-layer |
| SMT assembly (JLCPCB) | $50-100/board | Setup fee + per-joint cost |
| Components (BOM per board) | $40-80/board | DC-DC converters, CAN transceivers, connectors, passives |
| Stencil | $8-15 | One-time per revision |
| Shipping (DHL from China) | $20-40 | 5-7 days |
| **Total per PCB revision (5 boards)** | **$250 - $500** | Including all assembly |
| **Design engineering time** | 20-40 hrs | KiCad schematic + layout revision |

**Turnaround time:** 7-14 days (fab + assembly + shipping from JLCPCB).

#### C. Actuator / Sensor Swap

| Scenario | Cost | Notes |
|----------|------|-------|
| Try different actuator model (e.g., swap AK70-10 for RMD-X8) | $450-580 per actuator x quantity changed | May require new mounting brackets (+$100-250) |
| Swap camera (e.g., OAK-D Lite -> OAK-D Pro) | $150-400 | Usually plug-compatible, minimal rework |
| Add/swap LiDAR | $99-979 | RPLidar A1 ($99) to Livox Mid-360 ($979) |
| Swap battery pack | $200-500 | May require new mounting, new BMS wiring |
| **Typical sensor/actuator iteration** | **$200 - $2,000** | Depends on what changes |

#### D. Software-Only Iteration

| Item | Hardware Cost | Notes |
|------|-------------|-------|
| Firmware update | $0 | Flash over USB/network |
| ROS2 stack update | $0 | Deploy via container/OTA |
| ML model update | $0 | New YOLO weights, new gait policy |
| Locomotion policy change | $0 | New RL-trained policy from simulation |
| **Total** | **$0** | Engineering time only |

### 2.3 Estimated Total Iteration Spend (v0.1 to v1.0)

| Phase | Units Built | Hardware per Unit | Iteration Costs | Phase Total |
|-------|-------------|------------------|----------------|-------------|
| **POC (v0.1)** | 1 | $8,000 (reduced BOM: 3D-printed frame, no arm, basic sensors) | -- | $8,000 |
| **EVT (v0.2-v0.4)** | 2-3 | $10,200 each | 2x mechanical iterations ($2,000), 2x PCB revisions ($800), 1x sensor swap ($500) | $24,000 - $34,000 |
| **DVT (v0.5-v0.7)** | 3-4 | $10,200 each | 2x mechanical iterations ($2,500), 1x PCB revision ($400), weatherproofing materials ($500) | $34,000 - $44,000 |
| **PVT (v0.8-v1.0)** | 2-3 | $10,200 each | 1x mechanical iteration ($1,000), minor refinements ($500) | $22,000 - $32,000 |
| **Total hardware spend** | **8-11 units** | | | **$88,000 - $118,000** |

This includes all BOM costs for all units plus all iteration-specific costs (new CNC parts, new PCBs, sensor swaps).

**Cost-saving approach:** Build fewer physical units by spending more time in simulation (MuJoCo, Isaac Sim) before cutting metal. A strong sim-to-real pipeline can reduce the number of physical iterations by 30-50%.

---

## 3. Tooling & Equipment Costs

### 3.1 Electronics Workbench

| Tool | Model (Recommended) | Price | Notes |
|------|---------------------|-------|-------|
| Soldering station | Hakko FX-888D or Pinecil V2 | $100-120 | Temperature-controlled, essential for connectors and rework |
| Hot air rework station | Quick 957DW+ or equivalent | $80-150 | For SMD rework, heat shrink |
| Oscilloscope (100MHz, 4-ch) | Rigol DHO814 or Siglent SDS1104X-E | $400-500 | 12-bit resolution, CAN bus decode capable. [Rigol DHO800](https://www.rigolna.com/products/rigol-digital-oscilloscopes/dho800/) |
| Digital multimeter | Fluke 117 or Brymen BM257s | $80-200 | True RMS, essential for power debugging |
| CAN bus analyzer | PCAN-USB (Peak Systems) or Canable 2.0 | $25-250 | Canable ($25-40) for basic use; PCAN-USB ($250) for professional CAN FD support. [Peak Systems](https://www.peak-system.com/PCAN-USB.199.0.html) |
| Logic analyzer | Saleae Logic 8 | $400 | Decode SPI, I2C, UART, CAN simultaneously |
| Bench power supply (0-60V, 0-10A) | Rigol DP832 or Korad KA6005P | $200-400 | For testing subsystems without battery |
| Wire crimping tool set | Engineer PA-09 + assorted connectors | $50-100 | JST, Molex, XT60, Deutsch connectors |
| Helping hands / PCB holder | Generic + magnifying lamp | $30-50 | For precision soldering work |
| ESD mat + wrist strap | Generic | $25-40 | Protect sensitive electronics |
| **Subtotal (electronics)** | | **$1,400 - $2,300** | |

### 3.2 Mechanical Tools

| Tool | Price | Notes |
|------|-------|-------|
| Metric hex key set (ball-end) | $25-40 | Wera or Bondhus |
| Torque wrench set (small, 1-25 Nm) | $60-120 | Critical for motor mounting bolts |
| Calipers (digital, 150mm) | $25-40 | Mitutoyo or iGaging |
| Thread-locking compound (Loctite 243) | $10 | Medium-strength, essential for vibrating joints |
| Tap and die set (metric) | $30-60 | For threading aluminum parts |
| Deburring tools | $15-25 | Clean up CNC edges |
| Files, sandpaper, hand tools | $50-80 | General workshop supplies |
| Drill press (benchtop) | $150-300 | For precision holes in brackets |
| Dremel rotary tool | $50-80 | Quick modifications |
| Safety equipment (goggles, gloves) | $30-50 | |
| **Subtotal (mechanical)** | | **$450 - $900** | |

### 3.3 3D Printer

| Option | Price | Notes |
|--------|-------|-------|
| Bambu Lab P1S | $399-699 | Enclosed, high-speed, prints PETG/ABS/ASA/PA. Currently on sale at [$399](https://us.store.bambulab.com/products/p1s). Best value for prototyping enclosures, brackets, gripper molds. |
| Bambu Lab P1S Combo (with AMS) | $599-900 | Multi-material capability for multi-color prints |
| Filament stock (initial) | $100-200 | PETG, ASA, PA-CF assortment |
| **Subtotal (3D printing)** | | **$500 - $1,100** | |

### 3.4 Programming & Flashing Equipment

| Item | Price | Notes |
|------|-------|-------|
| Jetson Orin Nano Super Dev Kit | $249 | Included in BOM; also serves as dev/flash station |
| USB-to-Serial adapters (FTDI) | $15-25 (x2) | For MCU debugging |
| ST-Link V2 or J-Link EDU Mini | $20-70 | ARM MCU programmer/debugger (if using STM32 safety MCU) |
| MicroSD cards (high-endurance, 64GB) | $15-20 (x5) | For Jetson boot images, spares |
| USB hub (powered, 7-port) | $25-35 | Development connectivity |
| Ethernet cables, USB cables, assorted | $30-50 | |
| **Subtotal (programming)** | | **$110 - $200** | |

### 3.5 Workbench & Workspace

| Item | Price | Notes |
|------|-------|-------|
| Workbench (heavy-duty, 60"x30") | $200-500 | Must support 50+ kg robot weight |
| Anti-static bench mat | $40-60 | Full bench coverage |
| Storage bins / parts organizer | $50-100 | For BOM components, fasteners |
| Adequate lighting (LED task lights) | $40-80 | |
| Shelving / storage rack | $80-150 | For completed subassemblies and spare parts |
| Fire extinguisher (LiPo-rated) | $30-50 | Required when working with large Li-ion batteries |
| Fume extractor (for soldering) | $40-80 | Health and safety |
| **Subtotal (workspace)** | | **$480 - $1,020** | |

### 3.6 Total Tooling & Equipment

| Category | Low | High |
|----------|-----|------|
| Electronics workbench | $1,400 | $2,300 |
| Mechanical tools | $450 | $900 |
| 3D printer + materials | $500 | $1,100 |
| Programming equipment | $110 | $200 |
| Workspace setup | $480 | $1,020 |
| **Total** | **$2,940** | **$5,520** |

**Note:** This is a one-time investment that supports all prototype iterations. Amortized across 10 prototype units, it adds $294-$552 per unit.

---

## 4. Engineering Time Costs

### 4.1 Engineering Effort Estimate: Zero Hardware to Pilot-Ready

This is the single largest cost category. The estimate below assumes starting from zero physical hardware but leveraging existing open-source locomotion algorithms (MIT Cheetah, ODRI), existing ML models (YOLO), and existing frameworks (ROS2, Nav2).

| Engineering Discipline | Effort (months) | Key Tasks | Can AI (Claude) Assist? |
|----------------------|-----------------|-----------|------------------------|
| **Mechanical Engineering** | 6-10 months | Frame design (CAD), leg kinematics, actuator integration, thermal management, IP65 sealing, DFM for CNC, gripper arm design, bin mechanism | Partially: parametric design scripts, FEA setup, documentation. Physical design requires human judgment. |
| **Electrical Engineering** | 4-8 months | Power distribution board, CAN bus architecture, wiring harness design, PCB design (KiCad), EMC considerations, battery management integration, connector selection | Partially: schematic review, component selection research, KiCad scripting. PCB layout requires human skill. |
| **Firmware Engineering** | 4-8 months | Motor controller interface (CAN), safety MCU firmware, sensor drivers, Jetson BSP customization, OTA update system, real-time control loop (1kHz) | Significantly: Claude can write and review firmware, generate CAN protocol handlers, write test harnesses, debug serial output. |
| **ML / Perception Engineering** | 4-8 months | Litter detection model (YOLO fine-tuning), depth estimation, SLAM integration, grasp planning, sim-to-real locomotion policy, data collection pipeline | Significantly: model architecture selection, training pipeline scripts, data augmentation, deployment optimization (TensorRT). |
| **Software / Integration** | 4-8 months | ROS2 system architecture, Nav2 integration, fleet management backend, telemetry, remote monitoring dashboard, safety state machine | Heavily: Claude excels at ROS2 nodes, API development, system integration, infrastructure code, testing. |
| **Total** | **22-42 person-months** | | |

**Accounting for parallelism:** With overlapping work (mechanical + electrical in parallel, then firmware + ML + software in parallel), the calendar time is **12-18 months** with a team of 2-3 engineers.

### 4.2 Engineer Cost Rates

| Engineer Type | Full-Time Salary (US) | Freelance/Contract Rate | Monthly Cost (Contract) |
|---------------|----------------------|------------------------|------------------------|
| Mechanical engineer (robotics) | $90,000-$140,000/yr | $60-100/hr | $10,000-$17,000 |
| Electrical engineer (PCB/power) | $95,000-$145,000/yr | $65-110/hr | $11,000-$18,000 |
| Firmware engineer (embedded) | $100,000-$150,000/yr | $70-120/hr | $12,000-$20,000 |
| ML engineer (perception/RL) | $120,000-$180,000/yr | $80-150/hr | $13,000-$25,000 |
| Robotics generalist (full-stack) | $110,000-$160,000/yr | $75-130/hr | $12,000-$22,000 |

Sources: [Salary.com](https://www.salary.com/research/salary/recruiting/robotics-engineer-salary), [ZipRecruiter](https://www.ziprecruiter.com/Salaries/Robotics-Developer-Salary), [Thirdwork](https://www.thirdwork.xyz/rate-guides/Hourly-rate-for-robotics-engineer-freelancers)

### 4.3 Engineering Cost Scenarios

#### Scenario A: Scrappy / Founder-Led + AI

- **Team:** 1 founder (full-stack robotics, unpaid equity), 1 part-time contract ME ($70/hr, 20 hrs/week for 12 months), AI tools (Claude Code for software/firmware/ML)
- **Assumptions:** Founder covers firmware, ML, software, integration. Contract ME handles CAD, mechanical analysis, DFM. Claude handles ~40-60% of software/firmware development tasks.
- **Timeline:** 15-18 months

| Line Item | Cost |
|-----------|------|
| Contract mechanical engineer (20 hrs/wk x 50 wks) | $70,000 |
| Founder salary | $0 (equity) |
| Claude Code / AI tools subscription | $2,400-$6,000 ($200-500/mo) |
| Occasional EE consultant (PCB review, 40 hrs) | $4,000-$6,000 |
| **Total engineering** | **$76,000 - $82,000** |

**How much can the founder + AI actually do?**

Based on real-world reports from AI-assisted development:
- Software (ROS2 nodes, fleet management, dashboard): Claude can produce 70-80% of first-draft code, with the founder reviewing, testing, and integrating. This is a 3-5x productivity multiplier for software tasks.
- Firmware (CAN protocol, motor control): Claude can generate CAN frame parsers, PID controllers, state machines. Founder must validate on real hardware. 2-3x multiplier.
- ML (YOLO fine-tuning, training pipelines): Claude can write the training scripts, data augmentation, TensorRT export. Founder manages data collection and model evaluation. 2-3x multiplier.
- Mechanical/Electrical: AI assistance is limited to documentation, calculation scripts, and research. Physical design requires hands-on experience. 1.2-1.5x multiplier.

This makes a solo-founder + AI approach viable for software-heavy tasks but still requires at least one experienced mechanical engineer for the physical design.

#### Scenario B: Small Professional Team

- **Team:** 2 full-time engineers (ME + robotics generalist) + 1 part-time EE consultant + 1 part-time ML contractor
- **Timeline:** 12-15 months

| Line Item | Monthly Cost | Duration | Total |
|-----------|-------------|----------|-------|
| Mechanical engineer (full-time) | $10,000-$12,000 | 12 months | $120,000-$144,000 |
| Robotics generalist (full-time, firmware + software) | $12,000-$15,000 | 12 months | $144,000-$180,000 |
| EE consultant (part-time, 10 hrs/wk) | $3,000-$5,000 | 8 months | $24,000-$40,000 |
| ML contractor (part-time, 15 hrs/wk) | $5,000-$8,000 | 6 months | $30,000-$48,000 |
| **Total engineering** | | | **$318,000 - $412,000** |

#### Scenario C: Recommended Hybrid

- **Team:** 1 founder (full-time, equity-compensated, handles software/firmware/ML with AI assistance), 1 full-time contract ME, occasional EE consultant
- **Timeline:** 14-18 months

| Line Item | Cost |
|-----------|------|
| Contract mechanical engineer (full-time, 12 months) | $100,000-$130,000 |
| EE consultant (PCB design + review, ~60 hrs total) | $6,000-$9,000 |
| ML consultant (model training guidance, ~40 hrs) | $4,000-$8,000 |
| Founder salary | $0 (equity; or $3,000-$5,000/mo if taking minimum living wage from seed funding) |
| AI tools (Claude Code Pro) | $3,000-$6,000 |
| **Total engineering** | **$113,000 - $153,000** |

### 4.4 Can It Be Done by a Founder + AI?

**What works well with AI assistance:**
- All ROS2 node development, system integration, API design
- Fleet management backend (TypeScript/Node.js -- Claude excels here)
- ML training pipelines, data processing, model optimization
- Firmware for non-safety-critical systems
- Documentation, test generation, CI/CD pipelines
- Research synthesis (exactly what this document is)

**What still requires a human expert:**
- Physical mechanical design (CAD: SolidWorks/Fusion 360 -- AI cannot operate these tools effectively yet)
- PCB layout (KiCad layout requires spatial reasoning + EMC intuition)
- Hands-on assembly, debugging, and testing
- Safety-critical firmware (e-stop, motor overcurrent protection)
- Thermal analysis and vibration analysis
- IP65 sealing design
- Navigating certifications (requires domain-specific human expertise)

**Bottom line:** A technical founder with robotics experience can handle 60-70% of the engineering work with AI assistance, but needs at minimum one experienced ME and occasional EE consulting. Attempting to do all mechanical + electrical design without a dedicated ME will result in costly redesign cycles and likely 6-12 months of delays.

---

## 5. Testing & Validation Costs (Pre-Certification)

These costs are for internal testing and pre-compliance work before formal certification (which is covered in the product-spec-research.md at $195,000-$380,000).

### 5.1 Internal Testing Per Unit

| Test Type | Hours Per Unit | Equipment Needed | Notes |
|-----------|---------------|-----------------|-------|
| **Electronics burn-in** | 8-48 hrs | Power supply, thermal monitoring | Run all electronics at operating temperature for 24-48 hrs. Monitor for failures. |
| **Motor break-in & characterization** | 4-8 hrs | Oscilloscope, CAN analyzer | Run each motor through full range, measure torque curves, check for cogging/noise |
| **Locomotion testing (indoor)** | 8-16 hrs | Flat floor, motion capture (optional) | Basic walking gaits, turning, inclines (ramp) |
| **Locomotion testing (outdoor)** | 20-40 hrs | GPS, various terrain | Grass, gravel, concrete, curbs, slopes. Iterative with software tuning. |
| **Gripper/arm testing** | 8-12 hrs | Assorted litter samples | Test grasp success rate across 50+ object types |
| **Battery runtime test** | 8-12 hrs | Data logger | Full discharge test under realistic load profile |
| **Weatherproofing validation** | 2-4 hrs | Garden hose, spray bottle | DIY IP54/65 spray test before sending to lab |
| **Integration testing** | 8-16 hrs | Full system | End-to-end: detect litter -> navigate -> pick up -> deposit -> return to dock |
| **Total per unit** | **66-156 hrs** | | **Midpoint: ~100 hrs per DVT/PVT unit** |

At a technician rate of $30-36/hr, internal testing costs **$2,000-$5,600 per unit**.

### 5.2 Environmental Testing (External Lab)

These are typically done once or twice (on DVT and PVT builds) rather than on every unit.

| Test | Typical Cost | Duration | Lab Notes |
|------|-------------|----------|-----------|
| **IP65 water jet test** (IEC 60529) | $500-$2,000 per test | 2-4 hrs | Size-dependent. [JJR Lab](https://www.jjrlab.com/news/how-much-does-ip-waterproof-testing-cost.html) offers competitive rates. Larger products (our robot is ~800mm) cost more. |
| **IP6X dust test** (IEC 60529) | $500-$1,500 per test | 4-8 hrs | Dust chamber rental + technician time |
| **Vibration testing** (IEC 60068-2-6) | $1,000-$3,000 per day | 1-2 days | Shaker table rental. Labs like [Crystal Instruments](https://www.crystalinstruments.com/environmental-testing-lab) and [Elite Electronic](https://www.elitetest.com/) offer services. |
| **Temperature cycling** (-10C to +45C, IEC 60068-2-14) | $1,000-$2,500 per day | 1-3 days | Climate chamber; multiple cycles required |
| **HALT (Highly Accelerated Life Test)** | $3,000-$8,000 per session | 2-5 days | Combined vibration + temperature stress. [Element](https://www.element.com/product-qualification-testing-services/halt-testing-and-hass-testing) and similar labs. Strongly recommended -- finds design weaknesses before field deployment. |
| **Total environmental test package** | **$6,000 - $17,000** | | Per design iteration that needs environmental validation (typically 1-2 times) |

### 5.3 FCC Pre-Compliance Testing

| Test | Cost | Duration | Notes |
|------|------|----------|-------|
| **EMC pre-compliance scan** (in-house or friendly lab) | $500-$2,000 | 0.5-1 day | Using pre-certified modules (4G, WiFi, BLE) reduces risk. [EMC FastPass](https://emcfastpass.com/pre-compliance-testing-guide/) recommends building an in-house pre-compliance kit ($2,000-$5,000 one-time). |
| **FCC Part 15 formal testing** | $3,000-$10,000 | 1-3 days | For unintentional radiators (the robot as a whole). If using pre-certified radio modules, cost is on the lower end. [Compliance Testing](https://compliancetesting.com/fcc-certification-faqs/fcc-certification-cost/) |
| **Pre-compliance equipment (own)** | $2,000-$5,000 one-time | -- | Near-field probe set + spectrum analyzer rental. Useful for catching issues early. Alternative: rent time at a pre-compliance lab ($1,250/hr per [Tektronix](https://www.tek.com/en/blog/financial-case-emi-emc-pre-compliance-test-solution)). |

### 5.4 Total Testing & Validation Cost

| Test Category | Scrappy | Professional |
|---------------|---------|-------------|
| Internal testing (across all prototype units, ~6-8 units at DVT/PVT stage) | $8,000 | $25,000 |
| Environmental testing (1-2 rounds) | $6,000 | $17,000 |
| FCC pre-compliance | $1,000 | $10,000 |
| Pre-compliance equipment (one-time) | $0 (rent) | $5,000 (own) |
| **Total** | **$15,000** | **$57,000** |

---

## 6. Summary Cost Table

### 6.1 Per-Unit Cost Breakdown (First Prototype)

| Cost Category | Amount |
|---------------|--------|
| Component BOM | $10,200 |
| Assembly labor (60 hrs) | $1,800 - $4,400 |
| **Total first unit** | **$12,000 - $14,600** |

### 6.2 Total Program Cost: Zero to Pilot-Ready (v1.0)

| Cost Category | Scrappy (Founder-Led) | Professional (Team) | Recommended (Hybrid) |
|---------------|----------------------|--------------------|--------------------|
| **Component BOM** (8-11 units across all iterations) | $72,000 | $112,000 | $88,000 |
| **Assembly labor** | $6,000 | $30,000 | $15,000 |
| **Iteration costs** (new CNC parts, PCB revisions, sensor swaps) | $8,000 | $18,000 | $12,000 |
| **Tooling & equipment** (one-time) | $3,000 | $5,500 | $4,000 |
| **Engineering time** | $76,000 | $350,000 | $130,000 |
| **Testing & validation** | $15,000 | $57,000 | $30,000 |
| **Miscellaneous** (shipping, consumables, travel, cloud compute for ML training) | $5,000 | $15,000 | $8,000 |
| | | | |
| **TOTAL** | **$185,000** | **$587,500** | **$287,000** |

### 6.3 Simplified Summary Table

| Cost Category | Scrappy | Professional | Recommended |
|---------------|---------|-------------|-------------|
| Hardware (BOM + iterations, all units) | $80,000 | $130,000 | $100,000 |
| Assembly labor | $6,000 | $30,000 | $15,000 |
| Tooling & equipment | $3,000 | $5,500 | $4,000 |
| Engineering time | $76,000 | $350,000 | $130,000 |
| Testing & validation | $15,000 | $57,000 | $30,000 |
| Miscellaneous | $5,000 | $15,000 | $8,000 |
| **TOTAL** | **$185,000** | **$587,500** | **$287,000** |

### 6.4 Key Assumptions & Notes

1. **Scrappy approach** assumes: 1 founder working full-time (unpaid equity), 1 part-time contract ME, heavy use of AI tools, minimal external testing, 8 prototype units built, 15-18 month timeline.

2. **Professional approach** assumes: 2-3 full-time engineers + consultants, contract manufacturer for assembly, comprehensive environmental testing, 11+ prototype units, 12-15 month timeline.

3. **Recommended approach** assumes: 1 founder + 1 full-time contract ME + occasional EE/ML consultants, self-assembly transitioning to CM, moderate testing, 9-10 prototype units, 14-18 month timeline.

4. **Not included in these estimates:**
   - Formal certification (FCC, UL 3300, CE/ISO 13482): $195,000-$380,000 additional (see product-spec-research.md)
   - Office/workshop rent (if not working from home/garage)
   - Legal, insurance, incorporation costs
   - Pilot deployment operational costs
   - Marketing, sales, travel for municipality pilots

5. **Cost optimization levers:**
   - Invest heavily in simulation (MuJoCo, Isaac Sim) before building physical prototypes -- each avoided physical iteration saves $10,000-$15,000
   - Use 3D-printed frames for v0.1-v0.2 before committing to CNC aluminum
   - Leverage pre-certified radio modules to minimize FCC testing scope
   - Use JLCPCB for PCB fab+assembly at very low cost ($50-200 per board)
   - Consider open-source hardware designs (ODRI Solo-12, MIT Mini Cheetah) as starting points rather than designing from scratch

### 6.5 Cash Flow Timeline (Recommended Approach)

| Month | Spend | Cumulative | Activity |
|-------|-------|-----------|----------|
| 1-3 | $35,000 | $35,000 | Tooling purchase, ME onboarded, v0.1 POC build (3D printed), first BOM order |
| 4-6 | $45,000 | $80,000 | v0.2-v0.3 EVT builds, CNC frame, PCB v1, initial locomotion |
| 7-9 | $55,000 | $135,000 | v0.4-v0.5 DVT builds, arm integration, perception, outdoor testing begins |
| 10-12 | $50,000 | $185,000 | v0.6-v0.7 weatherproofing, environmental testing, field trials |
| 13-15 | $55,000 | $240,000 | v0.8-v1.0 PVT builds, pilot unit assembly, pre-compliance testing |
| 16-18 | $47,000 | $287,000 | Final pilot units, documentation, handoff to CM for future builds |

---

## Appendix A: Reference Robotics Company Timelines

| Company | Founded | First Prototype | Commercial Product | Duration | Funding Before Revenue |
|---------|---------|----------------|-------------------|----------|----------------------|
| Starship Technologies | 2014 | Aug 2014 | 2017 (commercial service) | 3 years | ~$17M pre-revenue |
| Unitree Robotics | 2016 | 2017 (first quad, 35kg, $30K) | 2020 (Go1 at $2,700) | 4 years | ~$10M+ |
| Agility Robotics | 2015 | 2016 (Cassie) | 2023 (Digit commercial) | 8 years | ~$180M+ total |
| Boston Dynamics | 1992 | 2005 (BigDog) | 2020 (Spot commercial) | 15 years | $2B+ (Alphabet/Softbank/Hyundai) |

Sources: [Starship Technologies](https://www.starship.xyz/about/), [IEEE Spectrum - Unitree Go2](https://spectrum.ieee.org/quadruped-robot-unitree-go2), [Agility Robotics Wikipedia](https://en.wikipedia.org/wiki/Agility_Robotics)

**Key takeaway:** Starship (wheeled, simpler) took 3 years. Unitree (quadruped, consumer) took 4 years. These are relevant benchmarks for CleanWalker. Agility and Boston Dynamics are higher-complexity platforms with longer timelines.

## Appendix B: Hardware Development Phase Definitions

| Phase | Purpose | Typical Units | Key Exit Criteria |
|-------|---------|--------------|-------------------|
| **POC** (Proof of Concept) | Prove core technology works | 1 | Basic locomotion demonstrated |
| **EVT** (Engineering Validation Test) | Validate all functional requirements | 3-50 (typically 5-12) | All subsystems functional; major design issues resolved |
| **DVT** (Design Validation Test) | Validate production-intent design | 5-50 | Environmental testing passed; DFM complete; assembly procedures documented |
| **PVT** (Production Validation Test) | Validate manufacturing process | 50-500 (5-10% of production run) | Yield targets met; quality acceptable; supply chain validated |

Sources: [Instrumental EVT/DVT/PVT Guide](https://instrumental.com/build-better-handbook/evt-dvt-pvt), [EnCata Hardware Stages](https://www.encata.net/blog/overview-of-the-hardware-product-development-stages-explained-poc-evt-dvt-pvt)

For CleanWalker at pre-revenue stage, PVT quantities will be much smaller (3-5 units) than traditional consumer electronics.

## Appendix C: Tool & Equipment Quick-Buy List

**Day 1 essentials (before first prototype build):**

| Item | Price | Where to Buy |
|------|-------|-------------|
| Bambu Lab P1S | $399 | [Bambu Lab Store](https://us.store.bambulab.com/products/p1s) |
| Hakko FX-888D soldering station | $110 | Amazon |
| Rigol DHO814 oscilloscope | $489 | [Saelig](https://www.saelig.com/category/dho800.htm) |
| Fluke 117 multimeter | $180 | Amazon |
| Canable 2.0 (CAN bus interface) | $40 | [Canable.io](https://canable.io/) |
| Wire crimping tools + connectors | $80 | Amazon |
| Metric hex key set (Wera) | $35 | Amazon |
| Digital calipers | $25 | Amazon |
| Bench power supply (Korad KA6005P) | $200 | Amazon |
| Workbench + storage | $300 | Home Depot / IKEA |
| ESD mat + safety gear | $60 | Amazon |
| **Total Day 1 kit** | **~$1,920** | |

---

*This document is a living research artifact. All prices are in USD and were researched in February 2026. Engineering time estimates assume leveraging existing open-source robotics frameworks and AI coding assistance. Actual costs will vary based on founder experience, design complexity, number of required iterations, and market conditions.*
