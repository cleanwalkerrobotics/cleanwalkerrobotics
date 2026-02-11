# WBSO Application Draft — CleanWalker Robotics

**Program:** WBSO (Wet Bevordering Speur- en Ontwikkelingswerk)
**Applicant:** MB Software Studio LLC (NL-based, treated as BV equivalent)
**Prepared:** 2026-02-11
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

This project involves four interrelated R&D work packages, each addressing specific **technical uncertainties** that cannot be resolved through standard engineering or application of existing knowledge.

### 2.2 Work Package 1: Multi-Modal Perception Pipeline

**Objective:** Develop a real-time litter detection and depth estimation system that runs three concurrent neural network inference pipelines on a single edge GPU within strict power (25W) and memory (8GB shared) constraints.

**Technical Uncertainties:**

1. **Small-object detection in variable outdoor lighting:** Litter items (cigarette butts at 5-15mm, candy wrappers, bottle caps) require detection at 0.3-5m range under variable outdoor illumination (shade, direct sunlight, rain, dawn/dusk). It is technically uncertain whether a YOLO26s model (8M parameters), optimized to TensorRT INT8, can achieve >85% mAP50 on these small outdoor objects while maintaining >30 FPS on the target hardware. State-of-the-art litter detection datasets (TACO, RoLID-11K) were collected under controlled conditions and do not represent the full range of outdoor scenarios.

2. **Multi-model GPU contention:** Running 5 neural network models concurrently (YOLO26s detection, SegFormer-B0 terrain segmentation, Isaac ROS ESS stereo depth, GR-ConvNet v2 grasp planning, cuVSLAM visual odometry) on a single 1024-core GPU with 8GB shared memory is technically novel. It is uncertain whether CUDA stream scheduling can prevent inference latency spikes when all models compete for GPU resources, particularly during the critical detection-to-grasp pipeline where end-to-end latency must stay below 200ms.

3. **Sensor fusion for outdoor depth estimation:** Combining hardware stereo depth (OAK-D Pro, 75mm baseline) with active IR structured light for outdoor pavement (low-texture surfaces where passive stereo fails) while simultaneously feeding visual SLAM. It is uncertain whether the IR projector maintains depth accuracy under direct sunlight (>100 klux), and whether the 0.7m minimum depth range is sufficient for close-range grasp verification.

**Research Activities:**
- Custom litter detection dataset creation: merging TACO (5,200 images), RoLID-11K, and Drinking Waste datasets with domain-specific augmentation (copy-paste onto outdoor park backgrounds, lighting variation)
- YOLO26s training with STAL (Small-Target-Aware Label Assignment) for small litter items
- TensorRT INT8 calibration and quantization for all 5 models
- Multi-model GPU profiling with `nsys` to characterize contention patterns and optimize CUDA stream scheduling
- Stereo depth validation under varying outdoor lighting conditions (simulated and real)

**What is NOT standard engineering:** Standard application of pre-trained object detection models is not S&O. Our research specifically addresses the novel technical challenge of (a) training for an object domain not well-represented in existing datasets (outdoor litter in variable conditions), (b) multi-model GPU contention optimization on edge hardware, and (c) outdoor stereo depth reliability assessment.

### 2.3 Work Package 2: Reinforcement Learning Locomotion for Terrain Adaptation

**Objective:** Develop a sim-to-real reinforcement learning locomotion policy that enables a custom 18-DOF quadrupedal robot (12 leg joints + 5 arm joints + 1 bag hinge) to walk stably on diverse outdoor terrains while carrying a 2-3kg payload (collected litter + arm).

**Technical Uncertainties:**

1. **Sim-to-real transfer for custom morphology:** Training RL locomotion policies in NVIDIA Isaac Lab simulation and deploying to real hardware involves a significant sim-to-real gap. Our custom CW-1 morphology (18 DOF, asymmetric front/rear knee configuration, offset center of mass due to arm and bag system) has never been simulated or trained. It is technically uncertain whether domain randomization of friction, mass, motor dynamics, and terrain parameters can bridge the sim-to-real gap for this novel configuration.

2. **Terrain-adaptive gait with variable payload:** The robot must adapt its gait to grass, gravel, paved paths, slopes (up to 15°), and curb transitions. The payload changes dynamically as litter is collected (0-3kg), shifting the center of mass. It is uncertain whether a single RL policy can handle the combined variability of terrain type AND payload mass, or whether multiple specialized policies with a meta-controller are needed.

3. **Locomotion stability during arm manipulation:** The robot must maintain balance while a 3-DOF arm extends to grasp litter at ground level. The arm's motion creates dynamic disturbance torques on the body. It is technically uncertain whether the locomotion controller can compensate for these disturbances in real-time, particularly on uneven terrain.

**Research Activities:**
- URDF model development and physics parameter tuning for NVIDIA Isaac Lab simulation
- PPO/SAC reinforcement learning training with 4096 parallel environments
- Domain randomization protocol design (friction 0.4-1.2, mass ±20%, motor strength ±15%)
- Terrain curriculum design (flat → rough → slopes → stairs → mixed)
- Sim-to-real transfer validation methodology development
- Disturbance rejection testing with dynamic arm motion

**What is NOT standard engineering:** Applying an existing locomotion controller (e.g., Unitree Go2 firmware) to a standard platform is NOT S&O. Our research develops a novel locomotion policy for a custom morphology with unique challenges (arm disturbance, variable payload, asymmetric configuration) that have no existing solution.

### 2.4 Work Package 3: Grasp Planning for Diverse Outdoor Litter

**Objective:** Develop a grasp planning system that can reliably pick up 10 categories of outdoor litter (plastic bottles, cans, cigarette butts, paper, plastic bags, food wrappers, glass bottles, cardboard, styrofoam, other) from ground-level in unstructured outdoor environments.

**Technical Uncertainties:**

1. **Grasp generalization to deformable outdoor objects:** State-of-the-art grasp planning models (GR-ConvNet v2, AnyGrasp) are trained on rigid tabletop objects in controlled indoor settings (Cornell Grasping Dataset, Jacquard). It is technically uncertain whether these models generalize to outdoor litter — which includes deformable items (crumpled wrappers, wet paper), very small items (cigarette butts), and items partially embedded in grass or soil. The gap between indoor tabletop grasping (95%+ success) and outdoor ground-level grasping of diverse litter is an open research question.

2. **Weatherproof grasping reliability:** The gripper must function in rain, humidity, and temperature variations (-5°C to 45°C). Silicone-tipped fingers may lose grip on wet surfaces. It is uncertain whether mechanical gripper designs with compliant fingertips can maintain >80% grasp success rate across weather conditions, and what sensor feedback (tactile, force/torque) is needed for closed-loop grasp adjustment.

3. **Ground-level grasp approach geometry:** Grasping objects lying flat on the ground from a quadruped platform (~40cm ground clearance) imposes severe kinematic constraints. The arm must reach below the body to ground level with sufficient force for scraping/scooping flat items. It is uncertain whether a 3-DOF arm (shoulder pitch, elbow pitch, wrist roll) provides sufficient dexterity, or whether additional degrees of freedom are required.

**Research Activities:**
- GR-ConvNet v2 fine-tuning on outdoor litter images with depth data
- Grasp primitive library development (top-down, scoop, pinch, edge) with class-specific selection
- Outdoor grasp success rate benchmarking (controlled tests with 10 litter categories × 5 surface types × 3 weather conditions)
- Compliant gripper fingertip material testing (silicone durometer variants, rubber compounds)
- Arm workspace analysis and kinematic constraint evaluation

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

---

## 3. Innovation Beyond State of the Art

### 3.1 Current State of the Art

**No commercial quadrupedal litter-collecting robot exists.** The closest references are:

| System | Organization | Status | Limitation |
|--------|-------------|--------|------------|
| VERO | IIT Genoa (Italy) | Academic research prototype | Indoor environments only, no outdoor capability |
| "Scoop Doggy Dog" | Veolia ANZ (Bondi Beach) | Proof-of-concept trial | Not autonomous, teleoperated, single trial |
| Autonomous sweepers | Various (Trombia, Enway) | Commercial wheeled vehicles | No manipulation capability, cannot pick up individual litter items |
| Spot (Boston Dynamics) | Research platform | Used for inspection, not litter collection | No integrated perception-grasp pipeline for litter, $75K+ price point |

### 3.2 Novel Contributions

Our R&D advances the state of the art in four specific areas:

1. **Integrated perception-grasp pipeline on edge hardware:** No existing system combines litter detection, terrain segmentation, stereo depth, grasp planning, and visual SLAM in a single 25W edge device for outdoor litter collection.

2. **Sim-to-real RL locomotion for litter collection:** Existing quadruped RL policies (walk-these-ways, legged_gym) target locomotion only. We develop policies that maintain stability during active arm manipulation and variable payload — a novel combination.

3. **Outdoor ground-level grasping of diverse litter:** Current grasp planning research focuses on tabletop scenarios. Outdoor ground-level grasping of deformable, wet, and partially embedded litter is an unsolved problem.

4. **Social-aware autonomous coverage in public spaces:** Existing coverage planners assume static environments. We develop dynamic re-planning with pedestrian-aware social navigation for quadrupedal platforms.

---

## 4. Expected R&D Hours and Costs

### 4.1 R&D Hours (Annual Estimate)

| Work Package | Hours/Year | Description |
|-------------|-----------|-------------|
| WP1: Perception Pipeline | 600 | Dataset creation, model training, TensorRT optimization, multi-model profiling |
| WP2: RL Locomotion | 500 | Simulation setup, training, domain randomization, sim-to-real validation |
| WP3: Grasp Planning | 400 | Model fine-tuning, primitive library, outdoor benchmarking, gripper R&D |
| WP4: Navigation & Coverage | 300 | SLAM configuration, social nav, coverage planner, integration testing |
| Cross-cutting: Integration & Testing | 200 | End-to-end stack integration, field testing, debugging |
| **Total** | **2,000** | Well above 500-hour minimum for starters |

### 4.2 R&D Cost Categories

| Category | Annual Estimate | Notes |
|----------|----------------|-------|
| R&D wages (founder salary, if applicable) | €48,000-€96,000 | Depends on salary level drawn from LLC |
| Other S&O costs | €5,000-€15,000 | Cloud compute for ML training, test equipment, materials |
| **Total eligible costs** | **€53,000-€111,000** | |

### 4.3 Expected WBSO Benefit

**Scenario A: Inhoudingsplichtige (employer, with salary)**

| Item | Calculation | Amount |
|------|-----------|--------|
| R&D wage costs (founder salary) | €96,000/year | €96,000 |
| First bracket (50% starter rate) | 50% × €96,000 | **€48,000** |
| Total annual WBSO benefit | | **€48,000** |

**Scenario B: IB-ondernemer (self-employed)**

| Item | Amount |
|------|--------|
| S&O deduction | €15,979 |
| Starter supplement | €7,996 |
| **Total deduction from taxable profit** | **€23,975** |
| Tax saving (at ~37% IB rate) | **~€8,870** |

> **Note:** Scenario A provides significantly more benefit if the founder draws a salary. Consult tax advisor on optimal structure.

---

## 5. R&D Hour Administration Plan

### 5.1 Hour Tracking Method

All R&D hours will be tracked using a dedicated time tracking system with:

- **Daily entries** specifying: date, hours worked, work package (WP1-WP4), activity description
- **Weekly summaries** linking activities to the technical uncertainties described in this application
- **Monthly totals** reconciled against the approved S&O hours

### 5.2 Documentation Standards

For each work package, the following documentation will be maintained:

- **Technical logbook:** Dated entries describing research activities, experiments, results, and decisions
- **Code commits:** Git history with commit messages linking to work packages
- **Training logs:** ML model training runs with hyperparameters, datasets, and results
- **Test reports:** Benchmark results for perception accuracy, locomotion stability, grasp success rates

### 5.3 Administration Location

All documentation stored in:
- Git repository (version-controlled code and research notes)
- Time tracking system (hours)
- Cloud storage (training data, model weights, test results)

---

## 6. Project Timeline

| Period | Activities |
|--------|-----------|
| **Q1-Q2 2026** | WP1: Dataset creation, YOLO26s training, TensorRT optimization. WP2: URDF development, Isaac Lab simulation setup, initial RL training. |
| **Q3 2026** | WP1: Multi-model GPU profiling, perception integration. WP2: RL policy refinement, terrain curriculum. WP3: GR-ConvNet v2 fine-tuning, grasp primitive library. |
| **Q4 2026** | WP3: Outdoor grasp benchmarking. WP4: SLAM configuration, social nav, coverage planner. Integration testing. |
| **Q1-Q2 2027** | Hardware integration, field testing, model iteration based on real-world data. |

---

## 7. Application Checklist

Before submitting the WBSO application:

- [ ] **Confirm tax classification** with Dutch tax advisor (inhoudingsplichtige vs IB-ondernemer)
- [ ] **Obtain eHerkenning** (Level 3, required for RVO portal access)
- [ ] **Verify LLC-as-BV status** with Belastingdienst (tax authority)
- [ ] **Apply BEFORE starting R&D work** for the application period
- [ ] **Set up hour tracking system** before first R&D hour
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

## 8. Key Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| LLC not accepted as WBSO-eligible entity | Consult tax advisor; convert to BV if needed (cost: €500-2,000) |
| Founder tax classification unclear | Get ruling from Belastingdienst before applying |
| R&D hours insufficient (< 500/year) | Documented work plan targets 2,000 hours — well above minimum |
| RVO rejects project as "not technically uncertain enough" | Emphasize novel combinations and outdoor/edge constraints; provide literature references showing these are open problems |
| WBSO budget exhausted | Apply as early as possible; total budget is €1.817B (rarely exhausted) |

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

---

*This is an internal draft for review. The actual WBSO application is submitted through the RVO eLoket portal and follows the structured form format. This document provides the project description content and supporting analysis for that submission.*
