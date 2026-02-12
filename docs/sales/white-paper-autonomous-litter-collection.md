# Autonomous Quadrupedal Litter Collection: A Technical White Paper

**CleanWalker Robotics**
**February 2026**

*For technical evaluation by municipal operations, property management, and procurement teams.*

---

## Executive Summary

Urban litter collection remains one of the most labor-intensive and costly components of municipal operations, consuming $8--12 billion annually in the United States alone and accounting for 55--70% of street cleaning budgets through manual labor. Current solutions --- primarily diesel-powered sweeper trucks and manual pickup crews --- are expensive, carbon-intensive, and unable to access the parks, plazas, and pedestrian zones where litter accumulates most visibly.

CleanWalker Robotics has developed the CW-1, an autonomous quadrupedal robot purpose-built for outdoor litter collection in public spaces. The CW-1 combines terrain-adaptive legged locomotion, an AI-powered perception pipeline capable of detecting 10 categories of litter in real time, and a robotic arm with class-specific grasping strategies. Deployed as a Robot-as-a-Service (RaaS) fleet, the CW-1 operates autonomously for 4--5 hours per charge across grass, gravel, paved paths, and uneven terrain that wheeled platforms cannot traverse.

Key differentiators include sub-75-millisecond detection-to-grasp perception latency, full EU Machinery Regulation 2023/1230 compliance architecture, and a total cost of ownership 40--60% below equivalent manual labor at fleet scale. This paper presents the technical architecture, operational model, safety framework, and economic analysis underlying the CW-1 platform.

---

## 1. The Urban Litter Challenge

### 1.1 Scale of the Problem

Municipal street and public space cleaning represents a substantial and growing fiscal burden. In the United States, municipalities spend an estimated $8--12 billion annually on street cleaning operations. The European Union allocates a comparable EUR 9--14 billion per year across member states. Individual cities bear disproportionate costs: San Francisco allocates $47.8 million annually ($59 per capita), while New York City's Department of Sanitation devotes approximately $890 million to collection and street cleaning --- 46.8% of the department's $1.9 billion budget.

These figures continue to rise. San Francisco's street cleaning budget increased 63% between 2018 and 2025, yet the city simultaneously cut 20 full-time positions due to broader fiscal pressures. This pattern --- increasing demand coupled with decreasing labor availability --- is replicated across municipalities worldwide. Germany allocated EUR 120 million in 2024 specifically for modernizing urban cleaning systems. The Netherlands invested EUR 50 million in smart city cleaning technology in the same period.

### 1.2 Why Current Solutions Are Insufficient

The municipal cleaning industry relies on two primary methods, both with fundamental limitations:

**Mechanized street sweepers** are large diesel- or electric-powered vehicles designed for paved roads. They cannot access parks, pedestrian plazas, waterfronts, campus grounds, or any area without vehicle-width paved access. They generate significant noise (typically 85--95 dB), restricting operation to daytime hours, and their $250,000--$450,000 unit cost limits fleet size. Most critically, they sweep debris from road surfaces but cannot identify and selectively collect individual litter items from grass, gravel, or garden beds.

**Manual labor crews** remain the primary method for litter collection in parks and pedestrian areas. A fully loaded municipal cleaning worker costs $55,000--$85,000 annually when accounting for salary, benefits, insurance, and equipment. Labor represents 55--70% of total cleaning budgets. These crews face recruitment challenges, seasonal variability, safety risks from traffic and contaminated waste, and limited coverage --- a single worker can effectively patrol only 2--4 hectares per shift depending on litter density.

Neither approach can operate during early morning or late evening hours without disturbing residents, and neither provides real-time data on litter accumulation patterns to inform resource allocation.

### 1.3 The Automation Gap

The autonomous cleaning robot market, valued at $592 million in 2025 and projected to reach $832 million by 2034, has focused almost exclusively on indoor environments. Commercially certified autonomous cleaners --- including products from major manufacturers --- are designed for airports, shopping centers, and warehouse floors. These wheeled platforms require flat, predictable indoor surfaces and are incapable of operating on grass, navigating curbs, or handling the variability of outdoor public spaces.

No commercially available autonomous robot currently addresses outdoor litter collection in parks, plazas, and pedestrian zones. This represents both the largest gap in the municipal cleaning market and the segment where labor costs are highest per unit of area covered.

---

## 2. Technical Approach

### 2.1 Quadrupedal Locomotion

The CW-1 employs a four-legged locomotion platform rather than a wheeled or tracked chassis. This design choice is driven by the operational environment: urban parks and public spaces present grass, gravel paths, tree roots, curbs, slopes up to 15 degrees, and narrow gaps between benches and planters that wheeled platforms cannot traverse.

Each leg has three degrees of freedom (hip abduction, hip flexion, knee flexion) driven by quasi-direct-drive actuators providing up to 24.8 Nm peak torque per joint. The resulting 12-DOF system enables multiple gait patterns --- trot, walk, and amble --- selected dynamically based on terrain classification and required stability margins. Ground clearance of approximately 35 cm allows the robot to step over low obstacles, and the mammalian stance configuration provides inherent stability on uneven ground.

Locomotion control uses a model predictive control (MPC) architecture for the initial deployment, with a reinforcement learning policy trained in NVIDIA Isaac Lab as the production path. The RL policy, trained across 4,096 parallel simulated environments with domain randomization on friction, mass distribution, and terrain geometry, demonstrates superior adaptability on novel terrain compared to model-based approaches.

### 2.2 AI Perception Pipeline

The CW-1's perception stack runs three concurrent pipelines on a single NVIDIA Jetson Orin Nano Super (67 TOPS INT8 performance):

**Litter Detection.** A YOLO26s model, optimized via TensorRT INT8 quantization, detects 10 categories of outdoor litter --- plastic bottles, aluminum cans, cigarette butts, paper, plastic bags, food wrappers, glass bottles, cardboard, styrofoam, and general trash --- at approximately 260 frames per second with 5--10 ms inference latency. The model incorporates Small-Target-Aware Label Assignment (STAL) to address the challenge of detecting small items such as cigarette butts and candy wrappers at distances of 0.3--5 meters. Training data is sourced from a combined dataset of over 30,000 annotated images drawn from public litter datasets (TACO, RoLID-11K) augmented with field-collected imagery.

**Stereo Depth Estimation.** The Luxonis OAK-D Pro stereo camera provides hardware-computed metric depth at 30 FPS across a 0.7--15 meter range using active infrared structured light projection. The infrared projector solves the low-texture problem inherent to outdoor pavement and concrete surfaces. Depth computation runs entirely on the camera's onboard vision processing unit (4 TOPS), consuming zero GPU resources on the main compute module. For production units, Isaac ROS ESS provides enhanced stereo depth with per-pixel confidence maps for grasp quality filtering.

**Terrain Segmentation.** A SegFormer-B0 model classifies ground surfaces into eight categories (paved path, grass, gravel, stairs/curb, water/mud, vegetation, road, unknown) at 5 Hz. The resulting terrain mask feeds directly into the navigation costmap, enabling speed modulation --- full speed on pavement, reduced speed on grass, avoidance of water and steep curbs.

**SLAM and Localization.** GPU-accelerated visual odometry (cuVSLAM at 30 Hz, consuming only 9% of GPU resources) is fused with RTAB-Map for loop-closure-corrected mapping. A Livox Mid-360 3D LiDAR (360-degree by 59-degree field of view, IP67 rated, 70-meter range) provides point cloud data for scan matching and obstacle mapping. State estimation fuses visual odometry, inertial measurement data (400 Hz), wheel/leg odometry, and optional RTK-GPS through an Extended Kalman Filter.

The complete perception pipeline --- from camera frame acquisition through litter detection, depth estimation, and grasp pose computation --- operates within a 50--75 millisecond latency budget. Total GPU utilization across all concurrent models is approximately 52%, providing substantial headroom for future capability additions.

### 2.3 Grasping System

A five-degree-of-freedom robotic arm (turret yaw, shoulder pitch, elbow pitch, wrist pitch, and gripper actuation) with 50 cm reach handles the physical collection of detected litter. The gripper employs a 2--3 finger mechanical design with silicone-tipped steel fingers, providing conformable grip on irregular shapes without crushing deformable items.

Grasp planning uses a two-stage approach. The GR-ConvNet v2 network (1.9 million parameters, 5--10 ms inference on TensorRT INT8) generates grasp pose candidates from RGB-D input, achieving a 95.4% grasp success rate on benchmark objects. Class-specific grasp strategies adapt the approach angle and grip force based on the detected litter category: enveloping grasps for cylindrical objects (bottles, cans), pinch grasps for small items (cigarette butts), and scoop grasps for flat deformable items (wrappers, paper).

A wrist-mounted camera provides close-range visual feedback for the final approach phase, using image-based visual servoing to center the target item and verify grasp success. Failed grasps (three consecutive attempts) trigger automatic logging of the item's GPS location and a photograph for human review, ensuring the robot does not stall on difficult items.

### 2.4 Autonomous Navigation

The CW-1 uses the ROS 2 Navigation Stack (Nav2) for path planning and obstacle avoidance, integrated with a Boustrophedon Cell Decomposition coverage planner that generates systematic patrol patterns across the assigned operating area. The coverage planner decomposes the mapped free space into cells, plans back-and-forth sweeps within each cell, and connects cells via an optimized tour --- ensuring complete area coverage.

Pedestrian-aware navigation is a core safety feature. The perception system detects people, children, and pets using the same YOLO model (with COCO-pretrained person/animal classes). Detected pedestrians generate inflation zones in the navigation costmap: 1.5-meter radius for adults, 2.0 meters for children and pets. Speed modulates dynamically based on proximity: normal operating speed (0.5 m/s) at distance, 0.3 m/s within 5--10 meters, 0.2 m/s within 2--5 meters, and full stop below 2 meters.

---

## 3. System Architecture

### 3.1 CW-1 Platform Specifications

| Parameter | Specification |
|---|---|
| Mass | 15 kg (fully loaded) |
| Dimensions (L x W x H) | 60 x 20 x 55 cm (standing) |
| Locomotion | 12-DOF quadrupedal, 3 joints per leg |
| Arm | 5-DOF with mechanical gripper, 50 cm reach |
| Compute | NVIDIA Jetson Orin Nano Super (8 GB, 67 TOPS) |
| Primary Camera | Luxonis OAK-D Pro (12 MP RGB, stereo depth, 9-axis IMU) |
| LiDAR | Livox Mid-360 (360 x 59 deg FOV, 70 m range, IP67) |
| Battery | 48V 20Ah Li-ion NMC (960 Wh) |
| Runtime | 4--5 hours (terrain dependent) |
| Operating Speed | 0.5 m/s patrol, 1.5 m/s transit |
| Litter Categories | 10 classes detected |
| Detection Latency | 5--10 ms (YOLO26s, TensorRT INT8) |
| Perception Pipeline Latency | 50--75 ms (detection through grasp pose) |
| Connectivity | 4G LTE (global quad-band) + WiFi |
| Weather Rating | IP65 (dust-tight, water jet resistant) |
| Noise Level | < 55 dB at 1 m (quieter than normal conversation) |
| Operating Temperature | -10 C to 45 C |
| Charging | Autonomous contact-based docking, 2--4 hour charge |
| Collection System | Roll-dispensed bag with auto-seal, 30--50 L capacity |
| Safety | Physical + wireless e-stop, independent safety MCU |

### 3.2 Software Stack

The CW-1 software architecture is built on ROS 2 Humble Hawksbill (Long-Term Support through May 2027) as the middleware layer. Key software components include:

- **Perception:** YOLO26s (litter detection), SegFormer-B0 (terrain), Isaac ROS ESS (depth), GR-ConvNet v2 (grasping) --- all optimized via TensorRT
- **Localization:** cuVSLAM (GPU visual odometry) + RTAB-Map (mapping and loop closure) + EKF sensor fusion
- **Navigation:** Nav2 with Boustrophedon Cell Decomposition coverage planning
- **Behavior:** BehaviorTree.CPP v4 state machine managing patrol, detect, approach, pick, deposit, dock, and yield behaviors
- **Locomotion:** MPC-based gait controller (Phase 1), RL policy via Isaac Lab (production)
- **Fleet Management:** Cloud-based dashboard for remote monitoring, mission assignment, telemetry, and OTA model updates

The full software stack consumes approximately 52% of GPU resources and 3.6 GB of the 8 GB unified memory, leaving substantial headroom for future capabilities.

### 3.3 Fleet Management

Each CW-1 unit reports real-time telemetry --- position, battery state, collection count, litter heatmaps, and system health --- to a cloud-based fleet management dashboard accessible to operations staff via web browser. Fleet operators can define operating zones via geofencing, schedule patrol shifts, review collection analytics, and receive automated alerts for maintenance needs. Over-the-air (OTA) updates enable continuous improvement of detection models and navigation behavior without physical access to deployed units.

---

## 4. Operational Model

### 4.1 Robot-as-a-Service (RaaS)

CleanWalker deploys the CW-1 exclusively through a Robot-as-a-Service model. Customers subscribe to a monthly per-unit rate that includes the robot hardware, software, remote monitoring, maintenance, and model updates. This structure eliminates capital expenditure for municipal customers, aligns cost with usage, and ensures CleanWalker maintains and improves the fleet continuously.

The RaaS model is particularly suited to municipal procurement, where operating expenditure budgets are typically more flexible than capital budgets. Customers do not own the hardware, removing end-of-life disposal obligations and technology obsolescence risk.

### 4.2 Deployment Process

A typical deployment proceeds in four phases:

1. **Site Assessment (Week 1--2).** CleanWalker engineers survey the deployment area using satellite imagery and on-site inspection, defining operating zones, dock locations, and charging infrastructure requirements.

2. **Infrastructure Installation (Week 2--4).** Charging docks are installed at designated locations with outdoor GFCI power connections. Each dock includes a weatherproof shelter, contact-based charging with pogo pins, and an alignment ramp for autonomous docking.

3. **Fleet Deployment and Mapping (Week 4--5).** Robots are deployed on-site, perform initial mapping runs to build the operating area map, and begin supervised autonomous operation. Staff training is provided for the fleet management dashboard.

4. **Autonomous Operations (Week 5+).** Robots operate autonomously according to configured schedules. CleanWalker monitors fleet health remotely and dispatches maintenance as needed. Monthly performance reports provide collection statistics, coverage analytics, and litter density heatmaps.

### 4.3 Maintenance

Scheduled maintenance includes quarterly gripper pad replacement, semi-annual actuator inspection, and annual battery health assessment. CleanWalker performs all maintenance either on-site (through regional technicians) or through modular component swap --- robots are designed for rapid field-serviceable replacement of the gripper assembly, bag system, and foot pads. Remote diagnostics identify potential issues before they cause downtime.

---

## 5. Safety and Regulatory Compliance

### 5.1 EU Machinery Regulation 2023/1230

The CW-1 is designed from the ground up for compliance with the EU Machinery Regulation 2023/1230, which replaces Directive 2006/42/EC effective January 20, 2027. As an autonomous mobile machine with ML-based safety functions (pedestrian detection and avoidance), the CW-1 falls under Annex I, Part A, requiring mandatory Notified Body involvement for CE certification.

The compliance architecture addresses the regulation's specific provisions for autonomous mobile machinery (Annex III, Section 3.5.4), including supervisor function (remote monitoring, stop, restart), working area safety (geofencing, obstacle detection), and movement functions (people/animal/obstacle detection with immediate stop capability).

### 5.2 Safety Architecture

Safety is implemented through an independent dual-channel architecture:

**Software safety layer.** The behavior tree enforces pedestrian clearance distances, speed modulation, geofence compliance, and battery management. If the perception system detects a collision risk, the robot stops immediately.

**Hardware safety layer.** A dedicated safety microcontroller (STM32), operating independently from the main compute system, monitors physical e-stop input, wireless kill switch, motor overcurrent, Jetson watchdog heartbeat, and tilt/rollover. If any safety condition is violated, the MCU cuts motor power via a hardware relay within 10 ms. This independent channel satisfies ISO 13849 Performance Level d requirements.

The physical e-stop button is accessible on the robot body. A wireless kill switch provides 100-meter range remote stop capability. Both trigger immediate motor power disconnection without any software in the critical path.

### 5.3 Certification Pathway

| Standard | Scope | Application to CW-1 |
|---|---|---|
| EN ISO 12100:2010 | Risk assessment | Foundation of CE process; full hazard analysis including AI behavior |
| ISO 13482 (revised) | Service robot safety | Primary safety standard for professional service robots |
| IEC 63327 | Autonomous mobile robot safety | Used by certified autonomous cleaners; directly applicable |
| ISO 13849-1 | Safety control systems | Performance Level determination for safety controls |
| IEC 62061 | Electronic safety systems | SIL determination for electronic safety |
| IEC 62443 | Cybersecurity | Industrial cybersecurity for connected machinery |
| EU AI Act 2024/1689 | High-risk AI systems | ML perception as safety component triggers high-risk classification |

CleanWalker has engaged in pre-consultation with Notified Bodies experienced in autonomous mobile robot certification. The target is CE marking under Module B+C (EU Type-Examination + Conformity to Type) prior to the January 2027 application date. No competitor has publicly achieved CE certification for an autonomous outdoor litter-collecting robot, positioning CleanWalker to establish a first-mover regulatory advantage.

### 5.4 Public Space Safety Measures

- Noise output below 55 dB at 1 meter (below normal conversation level), enabling operation during any hours
- IP65 environmental protection for all-weather operation
- Maximum operating speed of 0.5 m/s in pedestrian areas (below typical walking pace)
- Automatic stop and yield when any person is within 2 meters
- 360-degree LiDAR obstacle detection with 70-meter range
- Visual and LED status indicators (moving, stopped, collecting)
- Defined operating zones via geofencing --- the robot cannot leave its assigned area
- Full operational logging for incident investigation and regulatory audit

---

## 6. Economic Analysis

### 6.1 Total Cost of Ownership Comparison

The economic case for autonomous litter collection rests on the labor cost structure of manual cleaning. With fully loaded labor costs of $55,000--$85,000 per FTE annually (salary, benefits, insurance, equipment), even modest labor offset generates significant savings.

| Metric | Manual Labor (per FTE) | CW-1 (per unit, at fleet scale) |
|---|---|---|
| Annual cost | $55,000--$85,000 | Contact Sales for pricing |
| Daily operating hours | 6--8 (single shift) | 16--20 (with mid-day recharge) |
| Area coverage per day | 2--4 hectares | 4--8 hectares |
| Night/early morning operation | Limited (overtime rates) | Standard (no labor premium) |
| Weather downtime | Rain, extreme heat | Operates in rain (IP65), reduced in extremes |
| Data collection | None | Continuous (litter heatmaps, coverage analytics) |
| Consistency | Variable (fatigue, motivation) | Consistent (algorithmic coverage) |

Each CW-1 unit provides coverage equivalent to 1.5--2.5 FTEs of manual labor, depending on litter density and terrain complexity. At fleet scale (100+ units), the hardware cost per unit drops below $8,500, with annual operating costs of approximately $2,500--$4,000 per unit. The resulting total cost of ownership represents a 40--60% reduction compared to equivalent manual labor deployment.

### 6.2 ROI Timeline

The RaaS model eliminates upfront capital expenditure for customers. Return on investment is measured against the labor cost displaced from day one of deployment. Customers deploying a 10-unit fleet in a typical urban park environment can expect to offset 15--25 FTEs of manual collection labor over the course of the fleet's operation, with measurable cost savings within the first operational quarter. Detailed pricing is available through CleanWalker's sales team on a per-deployment basis.

### 6.3 Unit Economics at Scale

At production volumes of 1,000 units, CleanWalker achieves hardware costs of approximately $6,200 per unit (down from $11,600 at prototype scale), with hardware payback in 3--4 months of RaaS revenue per unit. Gross margins at scale reach 70--79%, driven by the high proportion of software and service value in the subscription model.

---

## 7. Pilot Program

### 7.1 Program Structure

CleanWalker offers a structured pilot program designed to demonstrate operational viability in the customer's specific environment. A standard pilot deploys 5--10 CW-1 units over a 6-month period in a defined operating zone (park, downtown corridor, campus, or waterfront).

### 7.2 Pilot Timeline

| Phase | Duration | Activities |
|---|---|---|
| Site Assessment | 2 weeks | Survey operating area, plan dock placement, define zones |
| Infrastructure Setup | 2 weeks | Install charging docks, outdoor power, network connectivity |
| Deployment and Mapping | 1 week | Deploy fleet, initial mapping, supervised autonomy |
| Autonomous Operations | 5 months | Full autonomous operation with remote monitoring |
| Review and Reporting | 2 weeks | Performance analysis, ROI assessment, expansion planning |

### 7.3 Success Metrics

Pilots are evaluated against quantitative key performance indicators agreed upon with the customer prior to deployment:

- **Collection rate:** Pieces of litter collected per hour per unit
- **Area coverage:** Hectares covered per day per unit
- **Uptime:** Percentage of scheduled operating hours with robot operational
- **Detection accuracy:** Percentage of visible litter items successfully detected
- **Grasp success rate:** Percentage of detected items successfully collected
- **Safety incidents:** Target of zero contact incidents with pedestrians
- **Customer satisfaction:** Operator feedback on dashboard usability and fleet performance

Historical pilot data is available upon request for qualified prospects. Contact CleanWalker's sales team to discuss pilot eligibility and deployment timelines.

---

## 8. About CleanWalker Robotics

CleanWalker Robotics develops autonomous robotic systems for outdoor litter collection in public spaces. Founded with the mission of making cities cleaner, quieter, and more sustainable, the company combines expertise in quadrupedal robotics, computer vision, and fleet operations to address the growing gap between municipal cleanliness demands and available labor.

The company's technology stack is built on open-source foundations (ROS 2, NVIDIA Isaac, Ultralytics YOLO) and designed for the European regulatory environment from day one. CleanWalker operates under a Robot-as-a-Service model, ensuring continuous improvement of deployed fleets through over-the-air updates and proactive maintenance.

For technical inquiries, pilot program information, or partnership discussions, visit [cleanwalkerrobotics.com](https://cleanwalkerrobotics.com) or contact the sales team at walker@cleanwalkerrobotics.com.

---

*This document contains forward-looking statements about product capabilities and regulatory compliance timelines. Specifications are subject to change as the product progresses through development and certification. All market data is sourced from publicly available municipal budget documents, industry reports, and CleanWalker's internal research as of February 2026.*

*Copyright 2026 MB Software Studio LLC. All rights reserved.*
