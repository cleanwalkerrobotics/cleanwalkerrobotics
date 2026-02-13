# WBSO Application Draft — CleanWalker Robotics

**Program:** WBSO (Wet Bevordering Speur- en Ontwikkelingswerk)
**Applicant:** MB Software Studio LLC (NL-based, treated as BV equivalent)
**Prepared:** 2026-02-13
**Status:** DRAFT — requires review by Dutch tax advisor before submission

---

## 1. Project Information

### Project Title

**Autonomous Quadrupedal Litter Collection Robot: Perception, Locomotion, and Grasp Planning for Unstructured Outdoor Environments**

### Project Period

Application for Period 2, 2026 (starting earliest possible date after submission)

### Applicant Classification

- **Entity:** MB Software Studio LLC — US LLC treated as Dutch BV equivalent for Dutch corporate/payroll tax since January 1, 2025
- **Starter status:** YES — first WBSO application, company < 5 years old
- **Type:** Inhoudingsplichtige (if salary is drawn) OR IB-ondernemer (self-employed)

> **IMPORTANT:** The founder must clarify their tax classification with a Dutch tax advisor:
> - If the founder draws a salary from the LLC → apply as inhoudingsplichtige → 50% starter rate on first €391,020 of R&D wages
> - If the founder is classified as IB-ondernemer → apply as zelfstandige → €15,979 deduction + €7,996 starter supplement = €23,975 total deduction
> - The LLC-as-BV treatment means the entity IS subject to Dutch payroll tax if it has employees/DGA salary

---

## 2. R&D Project Description (S&O-werkzaamheden)

### 2.1 Project Overview

CleanWalker is developing an autonomous quadrupedal robot for outdoor litter collection in public spaces (parks, campuses, waterfronts). The robot must navigate unstructured outdoor terrain, detect diverse litter items, and execute robotic grasping — all on an edge computing platform (NVIDIA Jetson Orin Nano Super, 8GB, 67 TOPS).

The CW-1 platform is a custom 18-DOF quadrupedal robot (15 kg, 12 leg joints + 5 arm joints + 1 bag hinge) with 15 integrated subsystems spanning compute, perception, actuation, power, and environmental hardening. Transitioning from a consumer-grade research prototype (~€10,000 BOM) to an industrial-grade outdoor-deployable system (~€48,000 BOM) involves fundamental R&D challenges across every subsystem.

This project involves five interrelated R&D work packages, each addressing specific **technical uncertainties** that cannot be resolved through standard engineering or application of existing knowledge.

### 2.2 Work Package 1: Multi-Modal Perception Pipeline

**Objective:** Develop a real-time litter detection and depth estimation system that runs three concurrent neural network inference pipelines on a single edge GPU within strict power (25W) and memory (8GB shared) constraints.

**Technical Uncertainties:**

1. **Small-object detection in variable outdoor lighting:** Litter items (cigarette butts at 5-15mm, candy wrappers, bottle caps) require detection at 0.3-5m range under variable outdoor illumination (shade, direct sunlight, rain, dawn/dusk). It is technically uncertain whether a YOLO26s model (8M parameters), optimized to TensorRT INT8, can achieve >85% mAP50 on these small outdoor objects while maintaining >30 FPS on the target hardware. State-of-the-art litter detection datasets (TACO, RoLID-11K) were collected under controlled conditions and do not represent the full range of outdoor scenarios.

2. **Multi-model GPU contention:** Running 5 neural network models concurrently (YOLO26s detection, SegFormer-B0 terrain segmentation, Isaac ROS ESS stereo depth, GR-ConvNet v2 grasp planning, cuVSLAM visual odometry) on a single 1024-core GPU with 8GB shared memory is technically novel. It is uncertain whether CUDA stream scheduling can prevent inference latency spikes when all models compete for GPU resources, particularly during the critical detection-to-grasp pipeline where end-to-end latency must stay below 200ms.

3. **Sensor fusion for outdoor depth estimation:** Combining hardware stereo depth (75mm baseline stereo camera with active IR structured light) for outdoor pavement (low-texture surfaces where passive stereo fails) while simultaneously feeding visual SLAM. It is uncertain whether the IR projector maintains depth accuracy under direct sunlight (>100 klux), and whether the 0.7m minimum depth range is sufficient for close-range grasp verification. The camera module lacks any IP rating and requires custom weatherproof enclosure design that maintains optical path quality — an unsolved integration challenge.

**Research Activities:**
- Custom litter detection dataset creation: merging TACO (5,200 images), RoLID-11K, and Drinking Waste datasets with domain-specific augmentation (copy-paste onto outdoor park backgrounds, lighting variation)
- YOLO26s training with STAL (Small-Target-Aware Label Assignment) for small litter items
- TensorRT INT8 calibration and quantization for all 5 models
- Multi-model GPU profiling with `nsys` to characterize contention patterns and optimize CUDA stream scheduling
- Stereo depth validation under varying outdoor lighting conditions (simulated and real)
- Camera weatherproof enclosure prototyping with optical window material testing (polycarbonate vs glass)

**What is NOT standard engineering:** Standard application of pre-trained object detection models is not S&O. Our research specifically addresses the novel technical challenge of (a) training for an object domain not well-represented in existing datasets (outdoor litter in variable conditions), (b) multi-model GPU contention optimization on edge hardware, and (c) outdoor stereo depth reliability assessment.

### 2.3 Work Package 2: Reinforcement Learning Locomotion for Terrain Adaptation

**Objective:** Develop a sim-to-real reinforcement learning locomotion policy that enables a custom 18-DOF quadrupedal robot (12 leg joints + 5 arm joints + 1 bag hinge) to walk stably on diverse outdoor terrains while carrying a 2-3kg payload (collected litter + arm).

**Technical Uncertainties:**

1. **Sim-to-real transfer for custom morphology:** Training RL locomotion policies in NVIDIA Isaac Lab simulation and deploying to real hardware involves a significant sim-to-real gap. Our custom CW-1 morphology (18 DOF, asymmetric front/rear knee configuration, offset center of mass due to arm and bag system) has never been simulated or trained. It is technically uncertain whether domain randomization of friction, mass, motor dynamics, and terrain parameters can bridge the sim-to-real gap for this novel configuration.

2. **Terrain-adaptive gait with variable payload:** The robot must adapt its gait to grass, gravel, paved paths, slopes (up to 15°), and curb transitions. The payload changes dynamically as litter is collected (0-3kg), shifting the center of mass. It is uncertain whether a single RL policy can handle the combined variability of terrain type AND payload mass, or whether multiple specialized policies with a meta-controller are needed.

3. **Locomotion stability during arm manipulation:** The robot must maintain balance while a 3-DOF arm extends to grasp litter at ground level. The arm's motion creates dynamic disturbance torques on the body. It is technically uncertain whether the locomotion controller can compensate for these disturbances in real-time, particularly on uneven terrain.

4. **Actuator characterization for sim-to-real fidelity:** The selected quasi-direct-drive (QDD) actuators (24.8 Nm peak torque for hip/knee, 9 Nm for hip yaw) operate with tight margins — knee joint margin is only 1.13× above dynamic peak loads. It is technically uncertain whether the motor dynamics modeled in simulation (friction, backlash, torque saturation, thermal derating) accurately represent real actuator behavior, particularly under sustained outdoor walking loads where thermal derating may reduce available torque by 15-30%.

**Research Activities:**
- URDF model development and physics parameter tuning for NVIDIA Isaac Lab simulation
- PPO/SAC reinforcement learning training with 4096 parallel environments
- Domain randomization protocol design (friction 0.4-1.2, mass ±20%, motor strength ±15%)
- Terrain curriculum design (flat → rough → slopes → stairs → mixed)
- Sim-to-real transfer validation methodology development
- Disturbance rejection testing with dynamic arm motion
- Actuator thermal characterization: bench testing motors under sustained walking load profiles (2+ hours continuous) to measure torque derating curves and validate simulation motor models

**What is NOT standard engineering:** Applying an existing commercial quadrupedal locomotion controller to a standard research platform is NOT S&O. Our research develops a novel locomotion policy for a custom morphology with unique challenges (arm disturbance, variable payload, asymmetric configuration, tight actuator margins) that have no existing solution.

### 2.4 Work Package 3: Grasp Planning for Diverse Outdoor Litter

**Objective:** Develop a grasp planning system that can reliably pick up 10 categories of outdoor litter (plastic bottles, cans, cigarette butts, paper, plastic bags, food wrappers, glass bottles, cardboard, styrofoam, other) from ground-level in unstructured outdoor environments.

**Technical Uncertainties:**

1. **Grasp generalization to deformable outdoor objects:** State-of-the-art grasp planning models (GR-ConvNet v2, AnyGrasp) are trained on rigid tabletop objects in controlled indoor settings (Cornell Grasping Dataset, Jacquard). It is technically uncertain whether these models generalize to outdoor litter — which includes deformable items (crumpled wrappers, wet paper), very small items (cigarette butts), and items partially embedded in grass or soil. The gap between indoor tabletop grasping (95%+ success) and outdoor ground-level grasping of diverse litter is an open research question.

2. **Weatherproof grasping reliability:** The gripper must function in rain, humidity, and temperature variations (-5°C to 45°C). Silicone-tipped fingers (Shore A 20-30 durometer) may lose grip on wet surfaces. It is uncertain whether mechanical gripper designs with compliant fingertips can maintain >80% grasp success rate across weather conditions, and what sensor feedback (tactile, force/torque) is needed for closed-loop grasp adjustment. Material testing across multiple silicone compounds and rubber formulations is required.

3. **Ground-level grasp approach geometry:** Grasping objects lying flat on the ground from a quadruped platform (~40cm ground clearance) imposes severe kinematic constraints. The arm must reach below the body to ground level with sufficient force for scraping/scooping flat items. Our selected arm actuators (4.1 Nm stall torque at shoulder) provide only 0.65× safety margin for horizontal hold with full 500g payload — it is uncertain whether this is sufficient for ground-level reach, or whether higher-torque actuators or additional degrees of freedom are required.

**Research Activities:**
- GR-ConvNet v2 fine-tuning on outdoor litter images with depth data
- Grasp primitive library development (top-down, scoop, pinch, edge) with class-specific selection
- Outdoor grasp success rate benchmarking (controlled tests with 10 litter categories × 5 surface types × 3 weather conditions)
- Compliant gripper fingertip material testing (silicone durometer variants, rubber compounds)
- Arm workspace analysis and kinematic constraint evaluation
- Shoulder actuator torque evaluation: testing selected servo at limits to determine if upgrade to higher-torque motor is necessary

### 2.5 Work Package 4: Autonomous Outdoor Navigation and Coverage

**Objective:** Develop a robust autonomous navigation system for a quadrupedal robot operating in GPS-degraded outdoor environments (tree canopy, building shadows) with moving pedestrians.

**Technical Uncertainties:**

1. **Visual SLAM robustness in repetitive outdoor environments:** Parks and public spaces contain repetitive visual features (grass, trees, benches) that cause false loop closures in visual SLAM. It is technically uncertain whether the cuVSLAM + RTAB-Map hybrid architecture can maintain localization accuracy below 0.5m over 4+ hour operational shifts in these challenging environments, particularly during seasonal appearance changes.

2. **Social navigation with pedestrian prediction:** The robot must navigate around pedestrians (1.5m clearance for adults, 2.0m for children/pets) in unpredictable public spaces. It is uncertain whether a costmap-based social navigation approach with distance-based speed modulation is sufficient for safe operation, or whether trajectory prediction models are needed to handle fast-moving pedestrians, cyclists, and unpredictable children/pet behavior.

3. **Coverage path planning with dynamic obstacle re-planning:** Boustrophedon cell decomposition provides complete coverage in static environments, but parks have dynamic obstacles (people, vehicles, temporary structures). It is uncertain how frequently the coverage plan must be re-computed and what computational overhead this adds to the navigation stack on the edge platform.

**Research Activities:**
- RTAB-Map + cuVSLAM hybrid configuration and outdoor benchmarking
- Loop closure reliability testing in repetitive park environments
- Social costmap layer development with person/child/pet detection integration
- Coverage planner implementation using Fields2Cover library with dynamic re-planning
- End-to-end navigation stack integration testing in Gazebo simulation

### 2.6 Work Package 5: Industrial Hardening for Outdoor Deployment

**Objective:** Develop engineering solutions to transition a consumer-grade research prototype to an industrial-grade system capable of sustained outdoor deployment (8-12 hours/day, 365 days/year) in rain, wind, temperature extremes (-5°C to 45°C), and coastal environments.

This work package addresses the fundamental engineering gap between a working lab prototype and an outdoor-deployable system. Our cost analysis shows the industrial transition increases BOM cost from ~€10,000 to ~€48,000 — a 4.8× multiplier driven primarily by actuator sealing, environmental protection, and safety-rated components. Every subsystem requires R&D to validate industrial-grade alternatives.

**Technical Uncertainties:**

1. **Dynamic joint sealing for IP67 protection:** The CW-1 has 18 actuated joints, each requiring dynamic sealing against rotating shafts. Stationary IP67 enclosures are straightforward engineering, but IP67-rated rotary shaft seals on a walking robot's joints — experiencing continuous oscillation, shock loads, and environmental contamination — are technically uncertain. Lip seals create friction (affecting gait control), labyrinth seals allow slow ingress under pressure washing, and magnetic fluid seals are unproven at this scale. It is uncertain which sealing approach (or combination) can maintain IP67 integrity over 15,000+ operating hours while adding acceptable friction and weight to each joint.

2. **Consumer-to-industrial actuator transition:** The selected consumer-grade quasi-direct-drive (QDD) actuators have no IP rating and are designed for indoor lab use. Industrial-grade sealed actuators (IP67, 15,000-30,000 hour rated lifetime) cost 4-7× more per joint, fundamentally changing the system economics. It is technically uncertain whether mid-tier industrial actuators can match the backdrivability and torque density of consumer QDD motors — a property critical for safe locomotion and compliant gait control. The R&D challenge is evaluating multiple actuator families (sealed QDD, servo + harmonic drive, custom sealed designs) for the specific torque, backdrivability, and environmental requirements of a walking litter-collection robot.

3. **Marine-grade materials compatibility and corrosion testing:** Transitioning from standard 6061-T6 aluminum to 5083-H321 marine alloy with Type III hard anodize changes machining parameters, thermal conductivity, and fastener compatibility. Using 316L stainless steel fasteners in contact with aluminum creates galvanic corrosion risk. It is uncertain whether the complete material system (marine aluminum + stainless fasteners + EPDM gaskets + conformal-coated PCBs) maintains structural integrity over multi-year outdoor deployment, particularly in salt-air coastal environments.

4. **Battery chemistry selection for lifetime and safety:** The prototype uses Li-ion NMC (500-1,000 cycle life, thermal runaway at ~210°C). Industrial deployment requires LiFePO4 chemistry (3,000-5,000 cycles, thermal runaway at ~270°C) but at a 50-70% weight penalty (~5.5 kg NMC vs ~10 kg LFP for equivalent capacity). It is technically uncertain whether the heavier LFP pack can be integrated within the 15 kg mass budget without degrading locomotion stability, and whether the cold-weather performance (-5°C to 0°C) is adequate without active heating systems that reduce runtime.

5. **Functional safety system integration:** EU Machinery Regulation 2023/1230 mandates safety-rated obstacle detection for autonomous robots in public spaces. Integrating a safety-rated LiDAR scanner (SIL 2 / PL d rated) into the existing perception stack adds ~€4,500-€7,500 per unit and creates technical uncertainty around latency — the safety system must guarantee <100ms reaction time independent of the main compute pipeline. It is uncertain whether the safety scanner can be architecturally separated from the NVIDIA Jetson (which is not safety-rated) while sharing spatial awareness, or whether a fully independent safety controller is required.

**Research Activities:**
- Rotary shaft seal evaluation: testing lip seal, labyrinth seal, and hybrid approaches across 10,000+ cycle fatigue tests on a single-joint test rig
- Industrial actuator benchmarking: procuring and testing 2-3 actuator families (sealed QDD, servo + harmonic drive) for backdrivability, torque density, thermal performance, and IP67 integrity
- Galvanic corrosion accelerated testing: 5083 aluminum + 316L stainless + EPDM gasket assemblies in salt spray chamber (ASTM B117, 500+ hours)
- LiFePO4 battery integration: mass budget analysis, cold-weather bench testing (-5°C to 0°C), thermal management system design (heater pads + aerogel insulation)
- Safety controller architecture design: evaluating dedicated safety MCU (STM32 SIL 2) with independent E-stop chain vs. safety-rated PLC integration
- IP67 validation: full-system ingress protection testing per IEC 60529

**What is NOT standard engineering:** Purchasing and installing off-the-shelf IP67 components in a stationary enclosure is standard engineering. Our R&D addresses the novel challenge of achieving IP67 protection across 18 dynamically actuated joints on a walking robot — a problem with no commercially available solution. Additionally, transitioning actuator technology while preserving locomotion control performance, and integrating functional safety systems into a novel quadrupedal platform, are open R&D problems specific to this class of outdoor mobile manipulators.

---

## 3. Innovation Beyond State of the Art

### 3.1 Current State of the Art

**No commercial quadrupedal litter-collecting robot exists.** The closest references are:

| System | Organization | Status | Limitation |
|--------|-------------|--------|------------|
| VERO | IIT Genoa (Italy) | Academic research prototype | Indoor environments only, no outdoor capability |
| "Scoop Doggy Dog" | Veolia ANZ (Bondi Beach) | Proof-of-concept trial | Not autonomous, teleoperated, single trial |
| Autonomous sweepers | Various commercial vendors | Commercial wheeled vehicles | No manipulation capability, cannot pick up individual litter items |
| Commercial quadruped platforms | Major robotics OEMs | Research/inspection platform | Used for inspection, not litter collection; no integrated perception-grasp pipeline for litter; prohibitive cost for municipal cleaning operations |

### 3.2 Novel Contributions

Our R&D advances the state of the art in five specific areas:

1. **Integrated perception-grasp pipeline on edge hardware:** No existing system combines litter detection, terrain segmentation, stereo depth, grasp planning, and visual SLAM in a single 25W edge device for outdoor litter collection.

2. **Sim-to-real RL locomotion for litter collection:** Existing quadruped RL policies (walk-these-ways, legged_gym) target locomotion only. We develop policies that maintain stability during active arm manipulation and variable payload — a novel combination.

3. **Outdoor ground-level grasping of diverse litter:** Current grasp planning research focuses on tabletop scenarios. Outdoor ground-level grasping of deformable, wet, and partially embedded litter is an unsolved problem.

4. **Social-aware autonomous coverage in public spaces:** Existing coverage planners assume static environments. We develop dynamic re-planning with pedestrian-aware social navigation for quadrupedal platforms.

5. **Industrial hardening of a walking mobile manipulator:** Existing IP67-rated robots are either stationary industrial arms or wheeled platforms with sealed enclosures. Achieving IP67 protection across 18 dynamically actuated joints on a walking robot while preserving gait control quality is a novel engineering challenge with no existing commercial solution.

---

## 4. Technology Readiness Level (TRL) Progression

### 4.1 Current Status: TRL 2-3

| Element | Current TRL | Evidence |
|---------|-------------|---------|
| Perception pipeline (YOLO, depth, SLAM) | TRL 3 | Models selected and benchmarked in simulation; no hardware integration |
| RL locomotion | TRL 2 | URDF model complete, simulation environment selected; no training runs completed |
| Grasp planning | TRL 2 | Architecture designed, GR-ConvNet v2 selected; no fine-tuning on litter dataset |
| Navigation & coverage | TRL 2 | Nav2 + Fields2Cover architecture designed; no outdoor testing |
| Hardware platform | TRL 2-3 | Design specification complete, 15 subsystem component decisions made, detailed URDF model validated; no physical prototype built |
| Industrial hardening | TRL 1-2 | Cost analysis complete (consumer vs. industrial BOM), material options identified; no prototyping or testing |

### 4.2 Target with WBSO Funding: TRL 5-6

| Target TRL | Description | Milestone |
|------------|-------------|-----------|
| TRL 4 | Technology validated in laboratory | Consumer-grade prototype walking, detecting litter, and executing grasps in controlled indoor/outdoor environment |
| TRL 5 | Technology validated in relevant environment | Industrial-grade prototype operating autonomously in real park environment for multi-hour sessions |
| TRL 6 | Technology demonstrated in relevant environment | Extended outdoor deployment (weeks-months) demonstrating perception, locomotion, grasping, and navigation in real operational conditions |

### 4.3 Key TRL Transitions Requiring R&D

| Transition | Primary Technical Risk | Work Package |
|------------|----------------------|--------------|
| TRL 2→3 (Perception) | Small-object detection accuracy on real outdoor data | WP1 |
| TRL 2→4 (Locomotion) | Sim-to-real transfer gap for custom morphology | WP2 |
| TRL 2→4 (Grasping) | Ground-level grasp success on deformable litter | WP3 |
| TRL 3→5 (Integration) | Multi-model GPU contention under real-time constraints | WP1 + WP4 |
| TRL 2→5 (Industrial) | IP67 dynamic joint sealing, actuator transition | WP5 |

Each transition involves specific technical uncertainties described in Section 2 that cannot be resolved through standard engineering.

---

## 5. Expected R&D Hours and Costs

### 5.1 R&D Hours (Annual Estimate)

| Work Package | Hours/Year | Description |
|-------------|-----------|-------------|
| WP1: Perception Pipeline | 500 | Dataset creation, model training, TensorRT optimization, multi-model profiling, camera weatherproofing |
| WP2: RL Locomotion | 450 | Simulation setup, RL training, domain randomization, sim-to-real validation, actuator characterization |
| WP3: Grasp Planning | 350 | Model fine-tuning, primitive library, outdoor benchmarking, gripper material R&D |
| WP4: Navigation & Coverage | 250 | SLAM configuration, social nav, coverage planner, integration testing |
| WP5: Industrial Hardening | 250 | IP67 sealing R&D, actuator evaluation, materials testing, battery integration, safety system design |
| Cross-cutting: Integration & Testing | 200 | End-to-end stack integration, field testing, debugging |
| **Total** | **2,000** | Well above 500-hour minimum for starters |

### 5.2 R&D Cost Categories

#### A. R&D Wages (Loonkosten S&O)

| Item | Annual Estimate | Notes |
|------|----------------|-------|
| Founder salary (DGA loon) | €48,000-€96,000 | Depends on salary level drawn from LLC |

#### B. Other S&O Costs (Kosten en Uitgaven)

R&D prototype hardware represents a significant portion of eligible costs. Building and testing physical prototypes is essential to resolve the technical uncertainties described in Section 2.

**Prototype hardware costs (R&D materials):**

| R&D Build | Estimated Cost (EUR) | Purpose |
|-----------|---------------------|---------|
| Consumer-grade prototype (TRL 2→3) | €10,000 | Initial proof-of-concept: validate basic walking, perception, grasping. 15 subsystems at consumer-grade pricing. |
| Iterative redesigns (2-3 iterations) | €6,000-€8,000 | Targeted component replacements based on testing failures: actuator spares, gripper redesign, revised PCB, frame bracket modifications |
| Industrial-grade prototype (TRL 3→5) | €48,000 | Full industrial rebuild with sealed actuators (~€28,000 for 17 joints), ruggedized compute (~€900), marine-grade frame (~€3,000), IP67 weatherproofing (~€1,200), industrial sensors (~€2,000), LiFePO4 battery system (~€1,400), safety-rated components (~€5,000), industrial connectors/wiring (~€500) |
| **Total prototype hardware** | **€64,000-€66,000** | |

**Actuator cost breakdown (dominant R&D material cost):**

Actuators represent 55-61% of prototype hardware cost. The transition from consumer quasi-direct-drive motors (~€355/joint average, no IP rating, 3,000-10,000 hour rated life) to industrial sealed actuators (~€1,800/joint average, IP67, 15,000-30,000 hour rated life) is the single largest cost driver and a primary R&D challenge. Evaluating multiple actuator families requires procuring test units from 2-3 manufacturers.

| Actuator Category | Consumer (prototype 1) | Industrial (prototype 2) | R&D Purpose |
|-------------------|----------------------|-------------------------|-------------|
| 12 leg joints | €5,200 | €24,000-€26,000 | Torque margin validation, thermal derating characterization, IP67 seal friction measurement |
| 5 arm joints | €800 | €4,000 | Shoulder torque sufficiency testing, weatherproofing validation |
| Evaluation units (2-3 alternatives) | — | €4,000-€8,000 | Comparative testing of sealed QDD vs servo + harmonic drive approaches |
| **Total actuators** | **€6,000** | **€32,000-€38,000** | |

**Other R&D material costs:**

| Item | Estimated Cost (EUR) | Notes |
|------|---------------------|-------|
| ML training compute | €500 | Cloud GPU for YOLO26s, GR-ConvNet, RL training (10-15 runs) |
| Test equipment | €5,000-€8,000 | CAN bus analyzer, oscilloscope, bench power supply, environmental test setup |
| IP67 seal prototyping materials | €2,000-€3,000 | Rotary lip seals, labyrinth seal prototypes, gasket materials (EPDM, silicone), Gore-Tex vents, cable glands |
| Materials testing | €2,000-€4,000 | Salt spray corrosion testing (ASTM B117), silicone durometer samples, marine-grade fastener evaluation |
| Gripper iterations | €500-€1,000 | CNC finger prototypes, silicone mold iterations, material samples |
| Environmental testing (IP verification) | €3,000-€5,000 | Third-party IP65/IP67 ingress testing per IEC 60529 |
| Data collection equipment | €500 | Mobile annotation setup, outdoor test fixtures |
| **Total other R&D materials** | **€13,500-€22,000** | |

#### C. Total Eligible R&D Costs

| Category | Conservative | Realistic |
|----------|-------------|-----------|
| R&D wages | €48,000 | €96,000 |
| Prototype hardware (R&D materials) | €64,000 | €66,000 |
| Other R&D materials | €13,500 | €22,000 |
| **Total eligible S&O costs** | **€125,500** | **€184,000** |

> **Note:** Hardware prototype costs are eligible as "kosten en uitgaven" (costs and expenditures) under WBSO when the prototypes are built specifically to resolve technical uncertainties described in the S&O application. These are not production costs — they are R&D material costs consumed in the research process. Confirm eligibility of specific cost categories with RVO or tax advisor.

### 5.3 Expected WBSO Benefit

**Scenario A: Inhoudingsplichtige (employer, with salary)**

WBSO benefit is calculated on the total of R&D wages + eligible other S&O costs:

| Item | Conservative | Realistic |
|------|-------------|-----------|
| R&D wage costs (founder salary) | €48,000 | €96,000 |
| Eligible other S&O costs (kosten en uitgaven) | €77,500 | €88,000 |
| **Total eligible base** | **€125,500** | **€184,000** |
| First bracket (50% starter rate on ≤ €391,020) | 50% × €125,500 = **€62,750** | 50% × €184,000 = **€92,000** |
| **Total annual WBSO benefit** | **€62,750** | **€92,000** |

> **Note:** The 50% starter rate applies to the first €391,020 of total S&O costs. Our total eligible costs fall well within this first bracket. Verify calculation methodology with tax advisor — the interaction between wage costs and other S&O costs in the benefit calculation may differ from this simplified estimate.

**Scenario B: IB-ondernemer (self-employed)**

| Item | Amount |
|------|--------|
| S&O deduction | €15,979 |
| Starter supplement | €7,996 |
| **Total deduction from taxable profit** | **€23,975** |
| Tax saving (at ~37% IB rate) | **~€8,870** |

> **Note:** Scenario A provides significantly more benefit. The substantial R&D material costs (prototype hardware) make the inhoudingsplichtige route even more advantageous. Consult tax advisor on optimal structure.

---

## 6. R&D Hour Administration Plan

### 6.1 Hour Tracking Method

All R&D hours will be tracked using a dedicated time tracking system with:

- **Daily entries** specifying: date, hours worked, work package (WP1-WP5), activity description
- **Weekly summaries** linking activities to the technical uncertainties described in this application
- **Monthly totals** reconciled against the approved S&O hours

### 6.2 Documentation Standards

For each work package, the following documentation will be maintained:

- **Technical logbook:** Dated entries describing research activities, experiments, results, and decisions
- **Code commits:** Git history with commit messages linking to work packages (monorepo: `ml/`, `firmware/`, `hardware/`)
- **Training logs:** ML model training runs with hyperparameters, datasets, and results
- **Test reports:** Benchmark results for perception accuracy, locomotion stability, grasp success rates
- **Hardware test logs:** Actuator characterization data, seal fatigue test results, corrosion test photographs, IP67 test reports
- **Component evaluation records:** Comparative test results for actuator families, battery chemistries, sealing approaches

### 6.3 Administration Location

All documentation stored in:
- Git repository (version-controlled code, design specs, and research notes)
- Time tracking system (hours)
- Cloud storage (training data, model weights, test results, hardware test photographs)

---

## 7. Project Timeline

| Period | Activities |
|--------|-----------|
| **Q1-Q2 2026** | WP1: Dataset creation, YOLO26s training, TensorRT optimization. WP2: URDF development, Isaac Lab simulation setup, initial RL training, actuator thermal characterization. WP5: Consumer-grade prototype build and initial testing. |
| **Q3 2026** | WP1: Multi-model GPU profiling, perception integration, camera enclosure prototyping. WP2: RL policy refinement, terrain curriculum. WP3: GR-ConvNet v2 fine-tuning, grasp primitive library, gripper material testing. WP5: Industrial actuator evaluation, IP67 seal prototyping. |
| **Q4 2026** | WP3: Outdoor grasp benchmarking. WP4: SLAM configuration, social nav, coverage planner. WP5: Materials corrosion testing, LiFePO4 battery integration, safety system architecture. Integration testing on consumer-grade prototype. |
| **Q1-Q2 2027** | WP5: Industrial-grade prototype build. Full hardware integration and outdoor field testing. Model iteration based on real-world data. IP67 validation testing. Safety system integration and testing. |

---

## 8. Application Checklist

Before submitting the WBSO application:

- [ ] **Confirm tax classification** with Dutch tax advisor (inhoudingsplichtige vs IB-ondernemer)
- [ ] **Obtain eHerkenning** (Level 3, required for RVO portal access)
- [ ] **Verify LLC-as-BV status** with Belastingdienst (tax authority)
- [ ] **Apply BEFORE starting R&D work** for the application period
- [ ] **Set up hour tracking system** before first R&D hour
- [ ] **Verify "kosten en uitgaven" eligibility** for prototype hardware costs with RVO or tax advisor
- [ ] **Budget:** €1.817 billion total WBSO budget for 2026, first-come-first-served
- [ ] **Deadline:** Apply by end of month for period starting 1st of following month. Last application: September 30, 2026 for Q4 period.

### Application Periods 2026

| Application Deadline | R&D Period Starts |
|---------------------|-------------------|
| December 20, 2025 | January 1, 2026 |
| January 31, 2026 | February 1, 2026 |
| February 28, 2026 | March 1, 2026 |
| ... (monthly rolling) | ... |
| September 30, 2026 | October 1, 2026 (runs through December 31) |

> **URGENT:** If R&D work has already started in January/February 2026, apply IMMEDIATELY for the next available period. WBSO cannot be applied retroactively to work performed before the approved period.

---

## 9. Key Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| LLC not accepted as WBSO-eligible entity | Consult tax advisor; convert to BV if needed (cost: €500-2,000) |
| Founder tax classification unclear | Get ruling from Belastingdienst before applying |
| R&D hours insufficient (< 500/year) | Documented work plan targets 2,000 hours — well above minimum |
| RVO rejects project as "not technically uncertain enough" | Emphasize novel combinations and outdoor/edge constraints; provide literature references showing these are open problems; WP5 (industrial hardening of walking robot joints) is particularly novel |
| Prototype hardware costs not accepted as "kosten en uitgaven" | Verify with RVO before including; prepare detailed justification linking each prototype component to specific technical uncertainties |
| WBSO budget exhausted | Apply as early as possible; total budget is €1.817B (rarely exhausted) |
| Industrial prototype cost higher than estimated | €48,000 estimate includes 10% contingency; actuator costs (55-61% of BOM) are based on manufacturer quotes; cost reduction possible through custom actuator development at volume |
| TRL progression slower than planned | Focus first on consumer-grade prototype (TRL 2→4) before committing to industrial build; each WP can demonstrate progress independently |

---

## Sources

- [RVO.nl — WBSO main page](https://www.rvo.nl/subsidies-financiering/wbso)
- [RVO.nl — WBSO conditions (English)](https://english.rvo.nl/subsidies-financing/wbso/conditions)
- [RVO.nl — WBSO tax credit benefit](https://english.rvo.nl/subsidies-financing/wbso/tax-credit-benefit)
- [RVO.nl — WBSO application process](https://english.rvo.nl/subsidies-financing/wbso/apply)
- [RVO.nl — WBSO calendar](https://www.rvo.nl/subsidies-financiering/wbso/kalender)
- [Business.gov.nl — Dutch R&D tax credit scheme](https://business.gov.nl/subsidies-and-schemes/wbso/)
- [Hezelburcht — WBSO 2026 deadlines](https://www.hezelburcht.com/nieuws/wbso-2025-gemist-dit-is-de-planning-met-bijbehorende-deadlines-voor-wbso-2026/)
- [Fiscount — WBSO 2026 deadlines and budget](https://www.fiscount.nl/publicaties/subsidie/wbso-2026-deadlines-en-budget-ongewijzigd/)
- [Subsidium — WBSO 2026](https://subsidium.nl/wbso-2026/)

### Internal References

Cost data in this application is derived from the following internal research documents:

- `docs/research/industrial-grade-cost-analysis.md` — Industrial-grade BOM: €48,000 prototype, €22,000 at 1,000 units (15 subsystem breakdown)
- `docs/design/component-decisions.md` — Detailed selection rationale for all 15 subsystems with torque analysis, alternatives evaluated, and risk assessment
- `docs/pilot-financial-model.md` — Development cost model and pilot economics
- `hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf` — 15 kg mass model with validated link masses, joint limits, and geometry

---

*This is an internal draft for review. The actual WBSO application is submitted through the RVO eLoket portal and follows the structured form format. This document provides the project description content and supporting analysis for that submission.*
