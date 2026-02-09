# CleanWalker Robotics -- Product Specification Research

**Document Version:** 1.0
**Date:** 2026-02-09
**Status:** Draft -- Internal Research Document

---

## Table of Contents

1. [Target Product Specifications](#1-target-product-specifications)
2. [Safety & Compliance Certification Roadmap](#2-safety--compliance-certification-roadmap)
3. [Industrial Design Guidelines](#3-industrial-design-guidelines)
4. [Operational Specifications](#4-operational-specifications)
5. [Open Source Reference Designs Comparison](#5-open-source-reference-designs-comparison)
6. [Key Risks and Mitigations](#6-key-risks-and-mitigations)

---

## 1. Target Product Specifications

### 1.1 Core Specifications Table

| Parameter | Target Spec | Rationale |
|---|---|---|
| **Weight** | <= 50 kg (110 lbs) | Below most US state sidewalk robot weight limits (80--550 lbs depending on jurisdiction). Light enough for 2-person lift during maintenance. Comparable to a mid-size dog, less intimidating than heavier platforms. |
| **Dimensions (LxWxH)** | 800 x 500 x 700 mm (stowed) | Must fit standard sidewalks (min 1.2m / 4ft wide) while leaving pedestrian clearance. Comparable to a medium-large dog footprint. |
| **Max Speed** | 1.5 m/s (3.4 mph) walking; 0.5 m/s operating | Below most state-mandated sidewalk speed limits (4--10 mph). Slow operating speed for safe litter detection and grasping. |
| **Coverage Rate** | 2,000--4,000 m^2/hr | Based on 0.5 m/s operating speed with 1m effective pickup width and time for detection/grasp cycles. A single unit covers a mid-size park area per shift. |
| **Battery Life** | >= 4 hours continuous operation | Enables a full morning or afternoon shift. Assumes 48V LiFePO4 pack, approximately 2 kWh capacity. |
| **Charge Time** | <= 2 hours (0--80%) | Fast-charge at dock station between shifts. Enables 2 operating shifts per day. |
| **Litter Bin Capacity** | 15--20 liters | Sufficient for 2--4 hours of typical park litter collection before requiring emptying. |
| **Payload (litter + arm)** | 10 kg manipulation arm + 5 kg bin contents | Must carry manipulation arm, bin, sensors, and accumulated litter. |
| **IP Rating** | IP65 (target), IP54 (minimum viable) | IP65: Dust-tight and protected against water jets from any direction. Required for rain, sprinkler, and pressure-wash cleaning. IP54 is the Boston Dynamics Spot baseline. |
| **Operating Temperature** | -10 degC to +45 degC | Covers most US and EU urban climates. Excludes extreme arctic conditions (specialized market). |
| **Storage Temperature** | -20 degC to +60 degC | Allows unheated outdoor storage in docking stations year-round in temperate climates. |
| **Noise Level** | <= 55 dB at 1m (operating), <= 65 dB at 1m (peak) | Quieter than normal conversation (60 dB). Critical for residential areas and early-morning operation. Comparable to a household refrigerator. |
| **MTBF** | >= 2,000 hours | Industry target for municipal equipment. At 8 hours/day operation, this yields approximately 250 days between failures -- roughly annual maintenance cycle. |
| **MTTR** | <= 4 hours (field-swappable modules) | Modular design with quick-swap legs, compute unit, battery, and bin. Minimizes downtime for municipalities without specialist technicians. |
| **Terrain Capability** | Paved surfaces, grass, gravel, packed earth, 15-deg slopes, 10cm obstacles | Must handle park paths, sidewalks, grass verges, and curb transitions. |
| **Degrees of Freedom** | 12 DOF (3 per leg) + 6 DOF manipulation arm | Standard quadruped configuration. Arm for litter grasping with wrist rotation. |
| **Communication** | 4G LTE (primary), Wi-Fi (dock), BLE (local service), LoRa mesh (fleet) | Redundant connectivity. 4G for cloud/remote ops, Wi-Fi for high-bandwidth updates at dock, BLE for local diagnostics, LoRa for low-power fleet coordination. |
| **Localization** | RTK-GPS (2cm outdoor), LiDAR SLAM, visual odometry | Multi-modal for reliable outdoor positioning in GPS-challenged areas (tree cover, urban canyons). |

### 1.2 Detection & Manipulation Specifications

| Parameter | Target Spec | Rationale |
|---|---|---|
| **Litter Detection Range** | 3--5 meters | Sufficient lookahead at walking speed for path planning. |
| **Detection Accuracy** | >= 95% recall, >= 90% precision | High recall avoids missed litter; moderate precision acceptable (false pickups waste time but are not harmful). |
| **Object Size Range** | 1 cm -- 30 cm (longest dimension) | Cigarette butts to beverage bottles. Excludes large debris (bags of trash, furniture). |
| **Grasp Success Rate** | >= 85% first attempt | Industry target for unstructured manipulation. Retry logic improves effective rate to >= 95%. |
| **Manipulation Arm Reach** | 500 mm from body center | Allows pickup without repositioning the full body for most cases. |
| **Camera Resolution** | 2x stereo 1080p (navigation), 1x 4K (litter detection) | Stereo for depth mapping, high-res for small litter identification. |
| **LiDAR** | 360-deg, 20m range, 0.2-deg resolution | Obstacle detection, mapping, and localization. Solid-state preferred for durability. |

### 1.3 Human Interaction Specifications

| Parameter | Target Spec | Rationale |
|---|---|---|
| **Person Detection Range** | >= 10 meters | Early detection allows smooth path adjustment rather than abrupt stops. |
| **Stopping Distance** | <= 0.3m from walking speed | Safe stopping distance at max 1.5 m/s. |
| **Minimum Clearance from Persons** | >= 1.5 meters (normal), >= 2.0 meters (children/pets detected) | Comfortable social distance. Wider buffer for vulnerable populations per UL 3300. |
| **Emergency Stop (E-Stop)** | Physical button (top-mounted, red mushroom-head), remote software e-stop, autonomous e-stop | Triple-redundant per ISO 13850. Physical button accessible without bending. |
| **Collision Force Limit** | <= 50 N (any contact surface) | Below pain threshold per ISO/TS 15066 collaborative robot guidelines. |
| **Audio Feedback** | Gentle proximity tone (40 dB), status chimes, voice alerts for close encounters | Non-startling. No sirens. Soft tonal language rather than harsh beeps. |
| **Visual Indicators** | LED status ring (360-deg visible), headlights, tail indicators, reflective striping | Visible intent signaling (direction, state). Meets nighttime visibility requirements. |

---

## 2. Safety & Compliance Certification Roadmap

### 2.1 Applicable Standards

| Standard | Scope | Market | Priority | Status |
|---|---|---|---|---|
| **ISO 13482** (being revised as ISO/DIS 13482:2024) | Safety requirements for personal care / service robots operating near people | Global (basis for CE) | Critical | Standard is being revised; new version expected 2025--2026. Plan for revised version. |
| **ISO 18646** (Parts 1--3) | Performance criteria and test methods for service robots: locomotion, navigation, manipulation | Global | High | Use Part 2 (Navigation) and Part 3 (Manipulation) as performance benchmarks. |
| **EU Machinery Directive 2006/42/EC** / **Regulation (EU) 2023/1230** | Essential health and safety requirements for machinery sold in the EU | EU / EEA | Critical (EU market) | Directive applies until Jan 2027; new Regulation (with explicit autonomous mobile machinery provisions) applies from Jan 20, 2027. Design for the new Regulation from the start. |
| **CE Marking** | Conformity marking for EU market access | EU / EEA | Critical (EU market) | Required. Encompasses Machinery Directive, EMC Directive, Radio Equipment Directive (RED). |
| **UL 3300** (ANSI/CAN/UL 3300:2024) | Safety for SCIEE (Service, Communication, Information, Education, Entertainment) robots | US / Canada | Critical (US market) | Added to OSHA NRTL program Dec 2025. Covers operational safety, fire/shock hazards, vulnerable population protection. |
| **FCC Part 15** | RF emissions limits for unintentional and intentional radiators | US | Required | Applies to all electronics; 4G/LTE/Wi-Fi/BLE modules require intentional radiator certification. |
| **ISO 13850** | Emergency stop function design and placement | Global | Required | Defines e-stop button design (red mushroom-head, yellow background), placement, and fail-safe behavior. |
| **ISO 12100** | Risk assessment methodology for machinery | Global | Required | Foundational risk assessment standard; basis for ISO 13482 compliance. |
| **IEC 62061 / ISO 13849** | Functional safety of safety-related control systems | Global | Required | Safety integrity levels (SIL) / Performance levels (PL) for e-stop and collision avoidance systems. |
| **RED 2014/53/EU** | Radio Equipment Directive (EU) for wireless devices | EU | Required (EU) | For 4G, Wi-Fi, BLE, LoRa modules. |
| **R&TTE / ETSI** | Telecom terminal equipment (EU harmonized standards) | EU | Required (EU) | Harmonized standards under RED for specific radio technologies. |
| **ANSI/RIA R15.08** | Safety standard for autonomous mobile robots (industrial) | US | Informational | Industrial-focused but informs best practices for obstacle detection, collision avoidance, speed control. |
| **ISO 3691-4** | Safety for driverless industrial trucks | Global | Informational | Relevant collision avoidance and e-stop requirements transferable to outdoor service robots. |

### 2.2 Certification Roadmap & Cost Estimates

| Phase | Activity | Timeline | Estimated Cost (USD) | Notes |
|---|---|---|---|---|
| **Phase 0: Risk Assessment** | Full ISO 12100 risk assessment; hazard identification and risk reduction | Months 1--3 | $30,000--$50,000 | Use certified safety consultancy (e.g., TUV, Pilz). Foundational for all subsequent certifications. |
| **Phase 1: Design for Safety** | Integrate safety requirements into mechanical, electrical, and software design | Months 2--8 | Internal engineering cost | Concurrent with product development. Safety-rated e-stop circuits, collision sensors, force-limiting design. |
| **Phase 2: FCC Part 15 Testing** | EMC pre-compliance testing, then formal FCC certification for all RF modules | Months 8--10 | $5,000--$15,000 | If using pre-certified modules (4G, Wi-Fi, BLE), cost is lower. Custom antenna designs increase cost. |
| **Phase 3: UL 3300 Certification** | Testing at UL-recognized NRTL lab; covers electrical safety, fire, operational safety, vulnerable population interaction | Months 10--16 | $50,000--$100,000 | Complex product with autonomous mobility, manipulation arm, and battery system. Allow 6+ months. |
| **Phase 4: ISO 13482 / CE Marking** | EU type-examination by Notified Body (TUV Rheinland, TUV SUD, or similar); includes Machinery Directive, EMC, RED | Months 12--18 | $80,000--$150,000 | Most expensive certification. Requires technical file, risk assessment, and physical testing. Plan for the new Machinery Regulation (2023/1230) requirements for autonomous mobile machinery. |
| **Phase 5: Field Validation** | Controlled field trials with municipality partners; collect real-world safety data | Months 14--20 | $20,000--$40,000 | Data supports certification submissions and builds customer confidence. |
| **Phase 6: Ongoing Compliance** | Annual factory audits, follow-up testing for design changes, standards updates | Annually | $10,000--$25,000/year | Required to maintain UL mark and CE marking. OTA software updates may require re-evaluation. |
| **Total Estimated Certification Budget** | | **18--24 months** | **$195,000--$380,000** | Excludes internal engineering time. Costs vary significantly with product complexity and number of design iterations. |

### 2.3 Key Safety Design Requirements

**Emergency Stop System:**
- Physical red mushroom-head e-stop button on top of robot (ISO 13850 compliant)
- Hardwired to motor power cutoff -- not software-dependent
- Remote e-stop via fleet management interface (software-triggered but with hardware failsafe)
- Autonomous e-stop triggered by safety-rated sensors detecting imminent collision
- All e-stop modes must bring robot to a stable, stationary state within 0.5 seconds

**Collision Avoidance:**
- Minimum 2 independent sensor modalities (LiDAR + stereo camera + ultrasonic backup)
- Safety-rated processing pipeline separate from main compute (dedicated safety PLC or safety-rated MCU)
- Performance Level d (PL d) per ISO 13849 for obstacle detection and stop functions
- Tested stopping distances documented for all speeds and surface conditions

**Person/Child/Pet Encounter Protocol:**
1. Detection at >= 10m: Adjust path to maintain >= 1.5m clearance
2. Detection at 5m (child or pet): Widen clearance to >= 2.0m, reduce speed to 0.3 m/s
3. Detection at 2m (unexpected): Full stop, activate proximity tone, wait for path to clear
4. Any contact detected: Immediate e-stop, alert fleet operator, log incident
5. Contact force never to exceed 50 N on any surface

---

## 3. Industrial Design Guidelines

### 3.1 Design Principles

**Principle 1: Friendly, Non-Threatening Appearance**

Research consistently shows that robot acceptance correlates with:
- **Rounded forms** over angular/sharp edges (reduces perception of threat)
- **Soft, light color palettes** (whites, light blues, soft greens) signal helpfulness
- **Moderate anthropomorphism** -- avoid the uncanny valley. Do NOT make it look human. Animal-like features (dog-like proportions) can increase approachability
- **Visible "eyes"** (sensor housing styled as friendly face) increases perceived intentionality and trustworthiness
- **Size proportional to a medium dog** -- familiar, non-threatening scale

Research from PMC (2025) confirms that shape and decoration significantly affect user experience, with rounded, decorated forms scoring higher on attractiveness and stimulation dimensions.

**Principle 2: Legibility of Intent**

The public must instantly understand what the robot is doing and where it is going:
- **360-degree LED status ring** showing state: green (operating), blue (idle/returning), amber (obstacle detected/slowing), red (stopped/error)
- **Directional indicators** (like turn signals) showing intended movement direction
- **Visible branding** identifying the robot as a municipal cleaning service
- **"I am cleaning your park" messaging** on body panels -- text or simple iconography

**Principle 3: Quiet and Unobtrusive**

- Target <= 55 dB operating noise (quieter than conversation)
- No sudden mechanical sounds -- servo whine must be dampened
- Soft, melodic proximity tones rather than beeps or alarms
- Electric actuators only -- no hydraulics or pneumatics
- Vibration-isolated foot pads to reduce ground-transmitted noise

### 3.2 Form Factor Guidelines

| Aspect | Specification | Notes |
|---|---|---|
| **Overall Silhouette** | Dog-like proportions, rounded body shell | Familiar quadruped shape reduces anxiety. Body shell conceals mechanical internals. |
| **Body Shell Material** | Recycled HDPE or ASA, matte finish | Durable, UV-resistant, sustainable messaging. Matte reduces glare and aggressive appearance. |
| **Primary Color** | Soft white / light grey (#E8E8E8 to #F5F5F5) | Research: light colors perceived as friendly and clean. White signals cleanliness. |
| **Accent Color** | Leaf green (#6BBF59) or sky blue (#5BA4CF) | Green: environmental / eco association. Blue: trust and calm. Used for branding stripe and LED ring. |
| **Reflective Elements** | 3M Scotchlite reflective strips on all sides and legs | Nighttime visibility for pedestrians and cyclists. Required for dawn/dusk operation. |
| **Headlights** | 2x forward-facing LED (warm white, 3000K, dimmable) | Illuminates path, signals direction of travel. Warm tone is less harsh/aggressive than cool white. |
| **Tail Light** | 1x rear LED bar (red) | Signals robot presence from behind. |
| **Branding Zone** | Both flanks, 200x100mm area | Municipal partner logo + CleanWalker logo. Swappable magnetic panels for different deployments. |
| **Sensor Housing ("Face")** | Rounded, slightly protruding housing with subtle "eye" styling | Houses stereo cameras and front LiDAR. Styled to suggest awareness without being creepy. |
| **Manipulation Arm** | Compact, folding design; stows flush against body when not in use | Avoids the "grabbing claw" look when idle. Extends only during pickup operations. |
| **Feet/Pads** | Soft rubber compound, rounded profile | Gentle on surfaces, quiet footfalls, no damage to grass or paving. |

### 3.3 Competitive Design Analysis

| Robot | Design Approach | Lessons for CleanWalker |
|---|---|---|
| **Starship Technologies** (delivery) | Small, boxy, 6-wheeled, white with blue flag. Cute and non-threatening. Normalized sidewalk robot presence on university campuses. | Prove that simple, small, friendly design achieves public acceptance. White color works. Flag/antenna aids visibility. |
| **Nuro R3** (delivery) | Rounded, car-sized, road-going. Smooth surfaces, no sharp edges. Large LED display for communication. | LED communication display concept is transferable. Rounded design language works at larger scale too. |
| **Serve Robotics** (delivery) | Compact, white, friendly "face" with LED eyes. Purposefully cute. | Direct inspiration for sensor housing as "face." LED eyes convey awareness and intent. |
| **Boston Dynamics Spot** (industrial) | Aggressive, industrial, exposed mechanical joints. Highly capable but intimidating to general public. | ANTI-pattern for public-facing design. Must conceal mechanics behind shell. Demonstrates why body panels matter. |
| **Unitree Go2** (consumer/research) | Exposed mechanical dog form. Capable but not designed for public acceptance. | Hardware platform reference. Shows what's achievable at consumer price points. |

### 3.4 What Makes People NOT Scared of a Walking Robot

Based on research synthesis:

1. **Predictable behavior** -- People fear what they cannot predict. Clear movement patterns, visible intent signals, and slow speed reduce anxiety.
2. **Familiar scale** -- Dog-sized is comfortable. Human-sized or larger triggers threat response.
3. **Visible purpose** -- A robot with an obvious job (cleaning, delivering) is less threatening than one with unclear intent. Branding and bin visibility communicate purpose.
4. **Soft materials and sounds** -- Hard metal surfaces and mechanical grinding sounds trigger wariness. Soft shells, rubber feet, and quiet operation communicate harmlessness.
5. **Non-humanoid form** -- The uncanny valley applies strongly. Four-legged animal forms avoid it entirely while still being relatable.
6. **Community introduction** -- Municipalities should introduce robots through community events, naming contests, and school demonstrations before deployment.
7. **Retreat behavior** -- A robot that stops and moves away when approached closely signals deference to human priority.
8. **No sudden movements** -- Smooth acceleration/deceleration curves. No jerky motions.

---

## 4. Operational Specifications

### 4.1 Docking Station

| Parameter | Specification | Notes |
|---|---|---|
| **Dimensions** | 1500 x 1000 x 1200 mm (LxWxD) | Accommodates robot + door clearance + bin emptying mechanism. |
| **Power Supply** | 240V AC, 3kW dedicated circuit | Supports fast charging and station electronics. |
| **Charging Method** | Automatic docking with spring-loaded contacts (robot-initiated) | No manual plug-in. Robot backs into dock and self-aligns. |
| **Bin Emptying** | Semi-automatic: Robot ejects bin cartridge at dock; large collection bin (120L) at station | Avoids complex mechanical transfer. Operator empties collection bin 1--2x daily depending on litter volume. |
| **Weather Protection** | Covered canopy, IP44 station minimum | Protects robot during charging from rain, UV. |
| **Placement** | Near park maintenance buildings, parking areas, or utility points | Requires power, cellular signal, and vehicle access for collection bin servicing. |
| **Network** | Hardwired Ethernet or cellular backhaul | Station serves as fleet gateway. Firmware updates downloaded over high-bandwidth connection. |
| **Status Display** | External LED panel showing robot status, charge level, next deployment time | Visible to maintenance staff and curious public. Transparency builds trust. |

### 4.2 Fleet Communication Architecture

```
                    +------------------+
                    |   Cloud Backend  |
                    |  (AWS/GCP/Azure) |
                    +--------+---------+
                             |
                     HTTPS / MQTT / gRPC
                             |
                +------------+------------+
                |                         |
     +----------+----------+   +----------+----------+
     | Fleet Management    |   | Remote Monitoring   |
     | Dashboard (Web)     |   | & Override Console  |
     +----------+----------+   +----------+----------+
                |                         |
                +------------+------------+
                             |
                    4G LTE (primary)
                    LoRa mesh (backup)
                             |
              +--------------+--------------+
              |              |              |
         +----+----+   +----+----+   +----+----+
         | Robot 1 |   | Robot 2 |   | Robot N |
         +----+----+   +----+----+   +----+----+
              |              |              |
         Wi-Fi (dock)   Wi-Fi (dock)   Wi-Fi (dock)
              |              |              |
         +----+----+   +----+----+   +----+----+
         | Dock 1  |   | Dock 2  |   | Dock N  |
         +---------+   +---------+   +---------+
```

**Communication Modes:**
- **4G LTE** (primary): Real-time telemetry, remote monitoring, alerts, and remote override. Always-on when operational.
- **LoRa Mesh** (secondary): Low-power fleet coordination between robots. Heartbeat signals, simple task coordination. Functions as backup if cellular fails.
- **Wi-Fi** (dock only): High-bandwidth for OTA firmware/ML model updates, log uploads, map sync. Only active when docked.
- **BLE** (service): Local diagnostics and configuration via technician tablet/phone.

### 4.3 Remote Monitoring & Override

| Feature | Description |
|---|---|
| **Real-Time Dashboard** | Map view showing all robots, status, battery, bin level, current task. Per-robot telemetry (speed, motor temps, sensor health). |
| **Alert System** | Automated alerts for: low battery, bin full, obstacle timeout, e-stop triggered, sensor degradation, geofence breach, person interaction anomaly. |
| **Remote Override** | Operator can: pause/resume robot, send to dock, adjust operating zone, trigger e-stop, initiate manual teleop mode. |
| **Teleop Mode** | Low-latency video stream + remote joystick control over 4G. For edge cases: stuck robot, unusual obstacle, or incident investigation. Requires operator authentication. |
| **Incident Replay** | All sensor data (cameras, LiDAR, IMU) buffered in rolling 5-minute window. On any incident trigger, full sensor snapshot is saved and uploaded for review. |

### 4.4 OTA Update Mechanism

| Component | Update Method | Frequency |
|---|---|---|
| **Application Software** | Container-based (Docker/Podman) A/B partition update over Wi-Fi at dock | Weekly to monthly |
| **ML Models** | Model file replacement with version management and rollback capability | Monthly (or as improved models are trained) |
| **Firmware (MCU/safety)** | Signed firmware images, written to flash with hardware verification. Requires physical dock connection. | Quarterly or as needed |
| **OS / Kernel** | Full system image A/B update (like Android OTA). Automatic rollback on boot failure. | Quarterly |
| **Maps / Geofences** | Lightweight data file update over 4G | As needed (new deployment areas, seasonal changes) |

**OTA Safety Requirements:**
- All updates cryptographically signed (code signing certificates)
- A/B partition scheme: update goes to inactive partition; verified before switching
- Automatic rollback if updated system fails health check within 5 minutes of boot
- Safety-critical firmware (e-stop, motor control) update requires dock connection + operator confirmation
- Update scheduling: Only during non-operational hours, or when docked and idle

### 4.5 Data Privacy

**Challenge:** CleanWalker robots carry cameras in public spaces. This raises significant privacy concerns under GDPR (EU), CCPA/CPRA (California), and other state privacy laws.

**Privacy-by-Design Approach:**

| Requirement | Implementation |
|---|---|
| **No persistent person imagery** | Onboard face detection runs locally; all detected faces are blurred in real-time before any image is stored or transmitted. Raw camera feeds are never uploaded to cloud. |
| **Rolling buffer only** | Camera data kept in 5-minute rolling buffer (RAM only). Only saved to persistent storage on incident trigger (e-stop, collision, anomaly). |
| **Incident data handling** | Incident footage is encrypted at rest, access-controlled, and automatically deleted after 30 days unless flagged for investigation. |
| **No biometric processing** | System detects "person" as a category for safety. No facial recognition, no individual identification, no tracking of specific persons. |
| **Data minimization** | Only collect data necessary for navigation, litter detection, and safety. No microphones (no audio capture). |
| **GDPR compliance** | Data Processing Agreement (DPA) with municipality customers. Privacy Impact Assessment (PIA) completed before each new deployment. Designated Data Protection Officer. Signage at deployment sites informing public of camera presence. |
| **US state privacy laws** | CCPA/CPRA compliance: no sale of personal data, consumer rights honored. Track evolving state laws (Colorado, Connecticut, Virginia, etc.). |
| **Transparency** | Public-facing privacy policy. QR code on robot body linking to privacy information page. |

---

## 5. Open Source Reference Designs Comparison

### 5.1 Comparison Table

| Feature | MIT Mini Cheetah | Stanford Pupper v3 | ODRI Solo-12 | SpotMicro |
|---|---|---|---|---|
| **Weight** | 9 kg (20 lbs) | 2.1 kg (4.6 lbs) | ~2.5 kg | ~1.5--2 kg |
| **DOF** | 12 (3 per leg) | 12 (3 per leg) | 12 (3 per leg) | 12 (3 per leg) |
| **Actuators** | Custom brushless motors with proprietary drivers | Brushless motors (torque-controllable) | Custom brushless with encoder feedback | Hobby servos (standard or high-voltage) |
| **Max Speed** | ~3.7 m/s (8.3 mph) | Moderate trotting | Fast walk / trot | Very slow walk |
| **Dynamic Capability** | Backflips, high-speed running, terrain adaptation | Hopping, trotting, running | Jumping (65--100cm), impedance-controlled landing | Basic walk, sit, stand |
| **Open Source** | Software open source (MIT license); hardware documented but not fully open | Fully open source (hardware + software) | Fully open source (BSD 3-clause) | Fully open source (community) |
| **Build Cost** | $10,000--$20,000+ (estimate; custom motors) | $600--$2,000 (self-sourced to kit) | ~$4,000+ (EUR) for Solo-8; Solo-12 higher | $60--$300 |
| **Commercially Available** | No (research platform) | Kit available (~$900--$1,500) | Yes, from PAL Robotics (assembled or kit) | No (DIY community project) |
| **Controller** | Custom compute (UP Board / Mini ITX) | Raspberry Pi / Jetson | Custom ODRI master board + Micro Driver boards | Raspberry Pi 3B / Jetson Nano |
| **Software Stack** | MIT Cheetah Software (C++, LCM) | Python, sim-to-real RL | Python, C++, ROS integration | ROS (C++ and Python nodes) |
| **Key Strength** | Unmatched dynamic performance; torque control quality | Lowest barrier to entry; educational; large community | Research-grade torque control; professional build quality; modular | Extremely low cost; huge maker community; easy to build |
| **Key Limitation** | Expensive custom actuators; not commercially available | Small scale; limited payload; hobby-grade components | Expensive for hobbyists; limited community outside academia | Very weak actuators; limited dynamic capability; servo-based control lacks torque feedback |

### 5.2 Lessons for CleanWalker

**From MIT Mini Cheetah:**
- Proprietary actuator design (quasi-direct-drive) is the gold standard for dynamic legged locomotion. CleanWalker should adopt a similar high-torque-density, backdrivable actuator architecture.
- The modular leg design (identical motors, identical leg assemblies) dramatically simplifies manufacturing and maintenance. CleanWalker should use identical leg modules.
- Published control algorithms (MIT Cheetah Software, Model Predictive Control) provide a strong baseline for our locomotion controller.

**From Stanford Pupper:**
- Sim-to-real reinforcement learning is viable even on low-cost hardware. CleanWalker's ML pipeline should include sim-to-real transfer for locomotion policies.
- The educational / open community approach builds ecosystem and attracts talent. Consider open-sourcing non-safety-critical components.
- Demonstrates that 12-DOF is sufficient for dynamic quadruped locomotion.

**From ODRI Solo-12:**
- BSD 3-clause license allows commercial derivative works. The actuator module design and leg kinematics could directly inform CleanWalker's mechanical design.
- PAL Robotics' commercialization path (research platform sold to universities) validates market for advanced quadrupeds.
- Impedance control (soft landings, terrain adaptation) is critical for outdoor operation. Solo-12's approach is directly applicable.

**From SpotMicro:**
- Proves that quadrupeds can be built extremely cheaply, but hobby servos are fundamentally inadequate for outdoor, payload-bearing operation. CleanWalker must use brushless motors with torque sensing.
- The massive community (hundreds of builds worldwide) demonstrates public fascination with quadruped robots. There is inherent market interest.
- ROS integration is expected in the robotics community. CleanWalker should support ROS 2 for developer ecosystem compatibility.

### 5.3 Commercial Reference: Boston Dynamics Spot

While not open source, Spot is the commercial benchmark:

| Parameter | Boston Dynamics Spot | CleanWalker Target | Notes |
|---|---|---|---|
| **Weight** | 32 kg (73 kg with arm) | <= 50 kg (with arm and bin) | Lighter than Spot+arm to stay under weight limits. |
| **IP Rating** | IP54 | IP65 | We target higher: must operate in rain, not just splash. |
| **Temperature** | -20 to +45 degC | -10 to +45 degC | Narrower cold range acceptable for urban deployment. |
| **Battery** | ~90 min runtime | >= 4 hours | Critical differentiator. Spot's runtime is insufficient for shift-based municipal work. |
| **Price** | ~$75,000 (base) | Target $25,000--$40,000 (at scale) | Must be significantly cheaper for municipal fleet economics. |
| **Payload** | 14 kg | 15 kg (arm + bin + litter) | Comparable. |
| **Speed** | 1.6 m/s max | 1.5 m/s max | Similar. Our operating speed is much lower (0.5 m/s). |
| **Autonomy** | Autonomous navigation (Autowalk) | Full autonomy (navigation + litter detection + manipulation) | Spot requires mission programming. CleanWalker operates fully autonomously. |

---

## 6. Key Risks and Mitigations

### 6.1 Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| 1 | **Regulatory rejection / delayed certification** -- ISO 13482 revision changes requirements; new EU Machinery Regulation (2027) introduces unforeseen demands | Medium | Critical | Engage certification consultancy (TUV) from concept phase. Design with maximum safety margins. Maintain active standards committee participation. Budget for redesign iterations. |
| 2 | **Public fear / rejection of walking robot** -- Media amplifies negative incidents; "robot dogs" associated with surveillance/policing | High | Critical | Invest heavily in friendly industrial design. Community engagement before deployment. Transparent privacy practices. Municipal partnership (city endorsement). Naming the robots (individual identities). Clear "cleaning helper" branding. |
| 3 | **Injury to person/child/pet** -- Collision, trip hazard, or falling robot causes harm | Low | Critical | Triple-redundant safety systems. Force-limited design (50N max). Comprehensive insurance. Controlled pilot deployments with safety observers. Immediate incident response protocol. |
| 4 | **Weather durability failures** -- Rain ingress, temperature extremes, UV degradation damage electronics or mechanics | Medium | High | IP65 design from start. Extended environmental testing (HALT/HASS). UV-stabilized materials. Conformal coating on PCBs. Heated/cooled enclosures for critical electronics. |
| 5 | **Insufficient battery life for viable operations** -- Real-world power consumption exceeds lab testing due to terrain, temperature, manipulation arm use | Medium | High | Conservative power budget with 30% margin. LiFePO4 chemistry for longevity and thermal stability. Regenerative braking on leg joints. Sleep modes when idle. Multiple dock stations for large deployments. |
| 6 | **Manipulation reliability too low** -- Diverse litter types (wet, flat, oddly shaped) cause frequent grasp failures, reducing effective cleaning rate | High | High | Invest in adaptive gripper design (soft robotics / underactuated). Large and diverse training dataset for litter detection. Sim-to-real grasp training. Accept "skip and flag" behavior for difficult items. Continuous ML model improvement via OTA. |
| 7 | **Municipal procurement cycles are slow** -- Government purchasing processes take 12--24 months; budget cycles create long sales delays | High | Medium | Offer pilot/lease programs to reduce procurement friction. Target grant-funded sustainability budgets. Build relationships with early-adopter municipalities. Demonstrate ROI with pilot data. |
| 8 | **Privacy backlash -- cameras in parks** -- Public or advocacy groups object to camera-equipped robots in parks/playgrounds | Medium | High | Privacy-by-design architecture (no persistent imagery, no facial recognition). Public signage. Privacy Impact Assessments. No microphones. Open privacy policy. Offer camera-blinding mode for sensitive areas. |
| 9 | **Actuator / mechanical wear in outdoor conditions** -- Dust, water, vibration degrade motors and joints faster than expected | Medium | High | IP65 sealed actuator housings. Modular quick-swap leg assemblies (field-replaceable in < 30 min). Predictive maintenance via motor current monitoring. Planned 1,000-hour actuator replacement schedule. |
| 10 | **Competition from wheeled alternatives** -- Wheeled autonomous cleaners (Trombia, VIGGO, etc.) are simpler, cheaper, and proven | Medium | Medium | Differentiate on terrain capability (grass, curbs, steps, uneven ground). Quadruped access to areas wheels cannot reach. Position as complementary to large sweepers, not competing. Focus on parks and trails, not streets. |
| 11 | **Software reliability / autonomy failures** -- Navigation errors, false detections, getting stuck in complex environments | High | Medium | Extensive simulation testing before field deployment. Geofenced operating areas. Remote teleop fallback. Conservative autonomy (stop and call for help rather than take risky actions). Continuous improvement via fleet learning. |
| 12 | **Supply chain risk for custom actuators** -- Custom quasi-direct-drive motors have long lead times and single-source dependencies | Medium | High | Dual-source actuator supply from design phase. Maintain 3-month buffer stock. Design for compatibility with multiple motor vendors. Consider partnership with actuator manufacturer (e.g., T-Motor, Maxon, or similar). |

### 6.2 Critical Path Items

1. **Actuator selection and testing** -- The most important technical decision. Must balance torque density, backdrivability, cost, and durability. Begin evaluation immediately with ODRI-style and MIT-style actuator architectures.

2. **Safety certification engagement** -- Engage TUV or equivalent Notified Body at concept stage (not after prototype). Their early input prevents expensive redesigns.

3. **Municipal pilot partner** -- Secure at least one forward-thinking municipality as pilot partner within 6 months. Their requirements and feedback shape the product. Their endorsement de-risks sales to other cities.

4. **Litter detection and manipulation R&D** -- The robot's unique value is picking up litter. Detection accuracy and grasp reliability are the key differentiators. Dedicated ML and manipulation engineering team from day one.

5. **Industrial design investment** -- Do not treat design as an afterthought. The robot's appearance IS the product in the eyes of the public and municipal buyers. Engage an industrial design firm with experience in public-facing robotics or consumer electronics.

---

## Appendix A: Regulatory Quick Reference by Market

| Requirement | United States | European Union | United Kingdom |
|---|---|---|---|
| **Safety Standard** | UL 3300 | ISO 13482 + Machinery Regulation | UKCA marking + ISO 13482 |
| **RF/Comms** | FCC Part 15 | RED 2014/53/EU | UK Radio Equipment Regulations 2017 |
| **EMC** | FCC Part 15 (Subpart B) | EMC Directive 2014/30/EU | EMC Regulations 2016 |
| **Privacy** | State laws (CCPA, etc.) -- no federal robot privacy law | GDPR | UK GDPR + Data Protection Act 2018 |
| **Sidewalk Operation** | State-by-state PDD laws (weight 80--550 lbs, speed 4--12 mph) | No EU-wide framework; member state regulations | No specific framework; local authority approval |
| **Marking** | UL mark (voluntary but expected by municipalities) | CE marking (mandatory) | UKCA marking (mandatory) |

## Appendix B: Key Standards Documents

- ISO 13482:2014 (revision in progress as ISO/DIS 13482:2024) -- Safety requirements for personal care robots
- ISO 18646-1:2016 -- Locomotion performance for wheeled robots
- ISO 18646-2:2024 -- Navigation performance for service robots
- ISO 18646-3:2021 -- Manipulation performance for service robots
- ISO 12100:2010 -- Safety of machinery -- Risk assessment
- ISO 13849-1:2023 -- Safety-related parts of control systems
- ISO 13850:2015 -- Emergency stop function
- IEC 62061:2021 -- Safety of machinery -- Functional safety
- ANSI/CAN/UL 3300:2024 -- SCIEE Robots
- EU Machinery Directive 2006/42/EC (until Jan 2027)
- EU Machinery Regulation 2023/1230 (from Jan 2027)
- FCC 47 CFR Part 15

## Appendix C: Sources

- [ANSI/A3 R15.06-2025 Robot Safety Standards](https://blog.ansi.org/ansi/ansi-a3-r15-06-2025-robot-safety/)
- [ISO 13482:2014 Official Page](https://www.iso.org/standard/53820.html)
- [ISO/FDIS 13482 Revision (Service Robots)](https://www.iso.org/standard/83498.html)
- [TUV Rheinland Personal Care Robots](https://www.tuv.com/content-media-files/master-content/global-landingpages/pdfs/robotics/tuv_rheinland_20_p04_rob-personal-care-robots-brochure-global-a4.pdf)
- [UL 3300 Outline of Investigation](https://www.ul.com/news/ul-3300-outline-investigation-helps-advance-safety-consumer-service-and-education-robots)
- [UL Standards Making Robots Safe for Public Spaces](https://ulse.org/insight/ul-standards-engagement-standards-matter-how-standards-are-making-robots-safe-public-and-commercial/)
- [UL 3300 Accessibility](https://ulse.org/insight/ul-standards-engagement-accessibility-ul-3300-helps-account-safety-people-disabilities-human-robot/)
- [UL Certification Fees and Timelines](https://asiandavinci.com/ul-certification-2024-fees-timelines-and-money-saving-tips/)
- [CE Certificate Costs 2025](https://www.sertifike.com/en/ce-certificate-costs-2025-what-factors-determine-the-price/)
- [EU Machinery Regulation 2023/1230 Changes](https://lewisbass.com/upcoming-changes-to-machinery-directive-requirements-for-ce-marking/)
- [New EU Machinery Regulation Safety Requirements](https://www.intertek.com/blog/2025/07-03-new-eu-machinery-regulation/)
- [FCC Part 15 Certification](https://keystonecompliance.com/fcc-part-15/)
- [FCC Part 15 Testing Cost](https://www.jjrlab.com/news/how-much-does-fcc-part-15-testing-cost.html)
- [ISO 3691-4 Mobile Robot Safety](https://jlcrobotics.com/iso-3691-4/)
- [R15.08 Safety Standard for AMR](https://www.agvnetwork.com/r15-08-safety-amr)
- [ANSI/CAN/UL 3300:2024](https://webstore.ansi.org/standards/ul/ansiul33002024)
- [ISO 18646-2:2024 Navigation Performance](https://www.iso.org/standard/82643.html)
- [Starship Delivery Robot Operations](https://roboticsandautomationnews.com/2025/09/26/last-mile-delivery-robots-navigating-sidewalks-and-urban-landscapes/94758/)
- [Nuro Business Breakdown](https://research.contrary.com/company/nuro)
- [Robot Appearance and Acceptance Research (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12028283/)
- [Robot Anthropomorphism and Trust](https://www.tandfonline.com/doi/full/10.1080/10447318.2025.2504198)
- [Sidewalk Robot Regulation](https://www.tandfonline.com/doi/full/10.1080/02723638.2023.2275426)
- [Municipal Robot Regulation (MRSC)](https://mrsc.org/stay-informed/mrsc-insight/september-2021/robot-delivery-devices-coming-soon)
- [Pennsylvania PDD Law](https://www.pghcitypaper.com/news-2/pennsylvania-legalizes-autonomous-delivery-robots-classifies-them-as-pedestrians-18482040/)
- [GDPR and Robot Data (Brain Corp)](https://www.braincorp.com/resources/whitepaper-the-gdpr-applied-to-brain-corporation-robot-data---brainos-r-clean-suite)
- [European Data Protection Supervisor -- Robotics](https://www.edps.europa.eu/data-protection/our-work/subjects/robotics_en)
- [Robot Fleet Management (NVIDIA)](https://developer.nvidia.com/blog/open-source-fleet-management-tools-for-autonomous-mobile-robots/)
- [Boston Dynamics Orbit Fleet Management](https://bostondynamics.com/products/orbit/)
- [Boston Dynamics Spot Specifications](https://bostondynamics.com/wp-content/uploads/2020/10/spot-specifications.pdf)
- [Unitree Go2 Specifications](https://www.unitree.com/go2/)
- [Unitree Go2 Review](https://blog.robozaps.com/b/unitree-go2-review)
- [Stanford Pupper Paper](https://arxiv.org/abs/2110.00736)
- [Pupper v3 Build Info](https://robotsthatexist.com/robots/pupper-v3)
- [MIT Mini Cheetah (MIT News)](https://news.mit.edu/2019/mit-mini-cheetah-first-four-legged-robot-to-backflip-0304)
- [MIT Mini Cheetah Platform Paper](https://www.semanticscholar.org/paper/Mini-Cheetah:-A-Platform-for-Pushing-the-Limits-of-Katz-Carlo/bb7e50d5d25ebf46f04c6d8cabdaac72a0e9d297)
- [ODRI Open Dynamic Robot Initiative](https://open-dynamic-robot-initiative.github.io/)
- [SOLO 12 Platform (PAL Robotics)](https://solo.pal-robotics.com/solo)
- [SpotMicro Quadruped Project](https://github.com/mike4192/spotMicro)
- [VIGGO Outdoor Cleaning Robot](https://www.viggorobot.com/autonomous-cleaning-machines/viggo-s100-n.html)
- [Angsa Robotics Litter Collection](https://www.weforum.org/stories/2021/07/ai-robot-cleaning-litter-beach/)
- [AI in Waste Management Market Size](https://datarootlabs.com/blog/future-wast-ai-solutions)
- [Autonomous Street Sweeping Research](https://nap.nationalacademies.org/read/27903/chapter/15)
- [Baryl Trash Can Robot](https://www.roboticgizmos.com/baryl-trash-robot/)
