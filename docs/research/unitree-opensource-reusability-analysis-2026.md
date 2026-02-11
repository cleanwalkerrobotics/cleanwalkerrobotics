# Unitree Open Source Repositories: Reusability Analysis for Custom Quadruped Development

**Date:** 2026-02-11
**Purpose:** Comprehensive analysis of Unitree's open-source repositories to determine what is reusable for a custom quadruped robot (non-Go2) versus Go2-specific implementations.
**Scope:** Five key repositories covering RL training, SDK architecture, world models, and URDF structures.

---

## Executive Summary

Unitree has released substantial open-source infrastructure that spans reinforcement learning training, low-level motor control, simulation environments, and world models. **Most of the RL training infrastructure is highly reusable** for custom quadrupeds with proper URDF configuration. The **SDK2 patterns are Go2-specific** but provide excellent reference architecture. The **URDF structure demonstrates best practices** that are directly applicable to custom robot development.

### Key Findings:
- ✅ **unitree_rl_gym** and **unitree_rl_lab**: ~70% reusable with custom URDF
- ⚠️ **unitree_sdk2**: Hardware-specific DDS communication, but excellent patterns
- ✅ **UnifoLM World Model**: Theoretically generalizable, practically Unitree-focused
- ✅ **URDF structures**: Highly reusable organization patterns and best practices

---

## 1. unitree_rl_gym

**Repository:** [github.com/unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
**License:** BSD 3-Clause (permissive, allows commercial use)
**Stars:** 2.9k | **Forks:** 472 | **Language:** Python (90.8%), C++ (8.2%)

### Architecture Overview

The repository implements a **four-stage RL workflow**: Train → Play → Sim2Sim → Sim2Real

```
unitree_rl_gym/
├── legged_gym/          # Core training environment (ROBOT-AGNOSTIC BASE)
│   ├── envs/
│   │   ├── base/        # legged_robot_config.py - base environment
│   │   ├── go2/         # Go2-specific config (inherits from base)
│   │   ├── h1/          # H1 humanoid config
│   │   └── g1/          # G1 humanoid config
│   └── scripts/
│       ├── train.py     # Training entry point
│       └── play.py      # Policy visualization
├── deploy/
│   ├── deploy_mujoco/   # Sim2sim validation
│   ├── deploy_real/     # Physical robot deployment (HARDWARE-SPECIFIC)
│   └── pre_train/       # Pre-trained models
└── resources/robots/    # URDF/MJCF files (REPLACEABLE)
```

### Key Components

#### 1. Training Framework: **PPO (Proximal Policy Optimization)**
- Algorithm: Uses [rsl_rl](https://github.com/leggedrobotics/rsl_rl) (ETH Zurich)
- **Hardware-agnostic implementation** from Robotic Systems Lab
- Supports MLP and LSTM policy networks
- GPU-optimized for large-batch training (Isaac Gym)

#### 2. Base Environment: `legged_gym/envs/base/legged_robot_config.py`
- **Robot-agnostic locomotion task** for rough terrain
- No specific robot asset (URDF/MJCF) defined in base
- Zero reward scales by default (must be configured per robot)

#### 3. Reward Function Structure
**Modular reward system** where each reward component can be scaled independently:

```python
# Example from documentation:
# Each non-zero reward scale adds a function to the reward sum
# To disable a reward, set its scale to zero
rewards:
  - base_height_reward: scale = 0.5
  - orientation_reward: scale = 0.3
  - linear_velocity_reward: scale = 1.0
  - energy_penalty: scale = -0.02
```

**Reusability:** ✅ Fully customizable for different morphologies

#### 4. Observation and Action Spaces
- **Configurable via robot config files**
- Observations: joint positions, velocities, base orientation, IMU data
- Actions: target joint positions/velocities for actuators
- Can be adapted to different DoF configurations

### What is Reusable vs Go2-Specific

| Component | Reusability | Notes |
|-----------|-------------|-------|
| **Base environment** | ✅ 100% | Robot-agnostic, designed for inheritance |
| **Reward functions** | ✅ 100% | Modular, fully customizable |
| **PPO training pipeline** | ✅ 100% | Uses rsl_rl (generic) |
| **Isaac Gym integration** | ✅ 100% | Simulator-level, not robot-specific |
| **Observation/action spaces** | ✅ 90% | Configurable, assumes standard quadruped structure |
| **Go2 config files** | ❌ 10% | Go2 joint names, DoF, mass properties |
| **Deployment to real robot** | ❌ 0% | Requires unitree_sdk2, hardware-specific |
| **Pre-trained models** | ❌ 0% | Trained for Go2/H1/G1 morphologies |

### Dependencies

**Training:**
- NVIDIA Isaac Gym (proprietary but free)
- legged_gym (included)
- rsl_rl (open source, BSD-3)
- PyTorch

**Deployment:**
- MuJoCo (open source, Apache 2.0)
- unitree_sdk2_python (Go2-specific hardware communication)
- LibTorch (C++ inference)

### How to Adapt for Custom Quadruped

**Step-by-step process** (from [unitree_rl_gym documentation](https://github.com/unitreerobotics/unitree_rl_gym)):

1. **Create new environment folder:** `legged_gym/envs/my_robot/`
2. **Inherit from base config:**
   ```python
   from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg

   class MyRobotCfg(LeggedRobotCfg):
       class env(LeggedRobotCfg.env):
           num_envs = 4096
           num_observations = 48
           num_actions = 12  # For quadruped with 3 DoF per leg
   ```
3. **Set robot asset:** Point to custom URDF/MJCF in `resources/robots/my_robot/`
4. **Define body names:** Joint names must match URDF
5. **Set default joint positions:** Neutral standing pose
6. **Configure PD gains:** Kp/Kd for position control
7. **Tune reward scales:** Customize for desired gait/behavior
8. **Train:** `python legged_gym/scripts/train.py --task=my_robot`

### Quality Assessment

**Strengths:**
- ✅ Clean separation of base/robot-specific code
- ✅ Well-documented training pipeline
- ✅ Active community (2.9k stars)
- ✅ Proven sim2real transfer (deployed on real Go2/H1)

**Weaknesses:**
- ⚠️ Limited documentation on custom robot integration
- ⚠️ Isaac Gym dependency (NVIDIA GPUs required)
- ⚠️ Deployment code is tightly coupled to unitree_sdk2

**Overall:** 4/5 stars. Excellent foundation for custom quadruped RL, but requires understanding of legged_gym architecture.

---

## 2. unitree_rl_lab

**Repository:** [github.com/unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)
**License:** Apache 2.0 (permissive, allows commercial use)
**Built on:** NVIDIA Isaac Lab 2.3.0+ / IsaacSim 5.1.0+

### Architecture Overview

```
unitree_rl_lab/
├── source/unitree_rl_lab/  # Main package
│   ├── tasks/              # Task definitions
│   ├── assets/             # Robot configurations (ArticulationCfg)
│   └── common_scene/       # Reusable scene components
├── scripts/                # Training/inference scripts
├── deploy/                 # Sim2sim (MuJoCo) and sim2real
└── docker/                 # Containerized setup
```

### Key Differences from unitree_rl_gym

| Feature | unitree_rl_gym | unitree_rl_lab |
|---------|----------------|----------------|
| **Simulator** | Isaac Gym (legacy) | Isaac Lab (modern, 2024+) |
| **Architecture** | Custom gym wrappers | Native Isaac Lab API |
| **Modularity** | Good (base + inheritance) | Excellent (standardized task API) |
| **Asset formats** | URDF/MJCF | URDF/MJCF/USD (native USD support) |
| **Extensibility** | Manual subclassing | Plugin-based task composition |
| **Multi-simulation** | MuJoCo for sim2sim | Native MuJoCo + USD interop |
| **Deployment** | Python → C++ (manual) | Integrated C++ compilation pipeline |

### Isaac Lab Integration: Why It Matters

Isaac Lab provides a **higher-level abstraction** than Isaac Gym:

1. **ArticulationCfg:** Declarative robot configuration
   - Directly accepts URDF/MJCF/USD
   - No manual physics parameter tuning
   - Automatic collision/visual mesh handling

2. **Standardized Task API:**
   ```python
   from omni.isaac.lab.envs import DirectRLEnv

   class MyQuadrupedTask(DirectRLEnv):
       def _setup_scene(self):
           # Add robot via ArticulationCfg
           self.robot = Articulation(cfg=MyRobotCfg())

       def _compute_rewards(self):
           # Modular reward components
           pass
   ```

3. **Multi-Physics Support:**
   - PhysX (default, GPU-accelerated)
   - USD native format (editable in Omniverse)
   - MuJoCo export for validation

### Custom Robot Integration

**From Isaac Lab documentation** ([Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)):

```python
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

MY_ROBOT_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="path/to/my_robot.usd",  # Or URDF path
        rigid_props=sim_utils.RigidBodyPropertiesCfg(...),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(...)
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip", ".*_thigh", ".*_calf"],
            stiffness=20.0,
            damping=0.5,
        )
    }
)
```

**Key insight:** No need to manually configure collision meshes, inertia, etc. Isaac Lab extracts from URDF automatically.

### Reusability Analysis

| Component | Reusability | Notes |
|-----------|-------------|-------|
| **Task structure** | ✅ 100% | Isaac Lab's DirectRLEnv is robot-agnostic |
| **Asset configs** | ✅ 95% | ArticulationCfg works with any URDF/USD |
| **Scene composition** | ✅ 100% | Modular terrain, objects, sensors |
| **Training scripts** | ✅ 100% | Uses rsl_rl (same as unitree_rl_gym) |
| **MuJoCo sim2sim** | ✅ 90% | Requires MuJoCo XML export from URDF |
| **Real robot deploy** | ❌ 0% | unitree_sdk2 integration (hardware-specific) |

### Quality Assessment

**Strengths:**
- ✅ **Modern architecture** (Isaac Lab is actively developed by NVIDIA)
- ✅ **Better modularity** than unitree_rl_gym
- ✅ **Native USD support** (interop with Omniverse, Blender, etc.)
- ✅ **Easier custom robot integration** via ArticulationCfg

**Weaknesses:**
- ⚠️ **Newer** (less community content than Isaac Gym)
- ⚠️ **Higher dependencies** (IsaacSim 5.1.0+ requires more system resources)
- ⚠️ Limited non-Unitree robot examples

**Recommendation:** Use **unitree_rl_lab** for new projects (Isaac Lab is the future). Use **unitree_rl_gym** for reference/legacy compatibility.

**Overall:** 4.5/5 stars. Best-in-class modularity and extensibility.

---

## 3. unitree_sdk2

**Repository:** [github.com/unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
**License:** BSD 3-Clause
**Stars:** 885 | **Forks:** 288 | **Language:** C++ (99.2%)
**Latest Release:** v2.0.2 (July 2025)

### Architecture Overview

```
unitree_sdk2/
├── include/unitree/      # Header files
│   ├── robot/           # Robot state/command interfaces
│   ├── common/          # DDS communication layer
│   └── idl/             # DDS message definitions (IDL)
├── lib/                 # Precompiled libraries (aarch64, x86_64)
├── example/
│   ├── go2/            # Go2-specific examples
│   ├── wireless/       # DDS topic examples
│   └── advanced_gamepad/
└── thirdparty/         # CycloneDDS, spdlog, etc.
```

### Core Architecture: DDS Communication

**CycloneDDS-based pub/sub system:**

```cpp
// Example: Subscribe to robot state
#include <unitree/robot/channel/channel_subscriber.hpp>

unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> sub;
sub.InitChannel("rt/lowstate");  // Topic name

// Callback on state update
sub.RegisterOnMessageCallback([](const LowState& state) {
    // Process state.motorState[0-11]
});
```

**Key Topics:**
- `rt/lowstate`: Motor angles, velocities, IMU, foot contact
- `rt/lowcmd`: Motor commands (position, velocity, torque, Kp, Kd)

### Motor Control Patterns

**From documentation** ([Basic Motion Control](https://support.unitree.com/home/en/developer/Basic_motion_control)):

```cpp
// Low-level motor command structure
struct MotorCmd {
    uint8_t mode;        // 0: Disable, 1: Enable
    float q;             // Target position (rad)
    float dq;            // Target velocity (rad/s)
    float tau;           // Feedforward torque (Nm)
    float kp;            // Position stiffness
    float kd;            // Damping coefficient
};

// Example: Hold joint at 0° with moderate stiffness
cmd.mode = 1;
cmd.q = 0.0;
cmd.dq = 0.0;
cmd.tau = 0.0;
cmd.kp = 10.0;  // Safety: low stiffness during testing
cmd.kd = 1.0;
```

**Control modes:**
1. **Position control:** High Kp, zero Kd
2. **Impedance control:** Moderate Kp/Kd (compliant)
3. **Force control:** Zero Kp/Kd, pure torque (tau)

### State Machine Patterns

**No explicit state machine in SDK examples**, but typical pattern:

```cpp
enum RobotMode {
    IDLE,
    STANDING,
    WALKING,
    EMERGENCY_STOP
};

class QuadrupedController {
    RobotMode mode_;

    void Update() {
        switch (mode_) {
            case IDLE:
                // Send zero commands
                break;
            case STANDING:
                // PD control to standing pose
                break;
            case WALKING:
                // Execute gait trajectory
                break;
        }
    }
};
```

**Reference implementation:** [unitree_guide](https://deepwiki.com/unitreerobotics/unitree_guide) (closed-source, but documented)

### Safety and Fault Handling

**From SDK examples:**
- **Watchdog timer:** If no command for >1s, motors disable
- **Joint limits:** Hardware enforced (cannot be overridden)
- **Temperature monitoring:** `state.motorState[i].temperature`
- **Overcurrent detection:** Automatic shutdown if `current > threshold`

**Best practices:**
```cpp
// Always check connection before sending commands
if (!dds_channel.IsConnected()) {
    // Fallback to safe state
    return;
}

// Gradual gain ramping (avoid sudden stiffness changes)
cmd.kp = std::min(cmd.kp + 1.0 * dt, target_kp);
```

### Reusability for Custom Quadruped

| Component | Reusability | Adaptation Required |
|-----------|-------------|---------------------|
| **DDS communication architecture** | ✅ 80% | Can use CycloneDDS for custom robot |
| **Motor command structure** | ✅ 70% | `MotorCmd` is generic (q, dq, tau, Kp, Kd) |
| **Low-level control patterns** | ✅ 90% | PD control, impedance, force - universal |
| **Topic names (`rt/lowstate`)** | ❌ 0% | Go2-specific, would need custom topics |
| **Message definitions (IDL)** | ⚠️ 50% | Can define custom DDS messages |
| **Example code** | ⚠️ 60% | Reference architecture, not plug-and-play |
| **Hardware drivers** | ❌ 0% | Proprietary actuator drivers (T-Motor, etc.) |

### What Can Be Learned for Custom Development

**1. DDS as Communication Layer:**
- Decouples high-level control from low-level firmware
- Real-time pub/sub (1kHz+ update rates possible)
- Cross-platform (Linux, QNX, embedded)

**2. Motor Control Interface Design:**
- Unified `MotorCmd` struct for all control modes
- Separate Kp/Kd from target position (tunability)
- Feedforward torque for gravity compensation

**3. Software Architecture:**
- Clean separation: communication layer → control layer → planning layer
- Thread-safe DDS subscribers with callbacks
- Synchronous state reads (blocking vs async)

**4. Safety by Design:**
- Watchdog timers at multiple levels
- Graceful degradation (high-level fails → low-level continues)
- Explicit enable/disable per motor

### Dependencies

**Required:**
- CycloneDDS (included in thirdparty/)
- CMake 3.10+
- GCC 9.4.0+
- Eigen3, Boost, spdlog, yaml-cpp

**Optional:**
- unitree_sdk2_python (pybind11 wrapper)
- ROS2 bridge (separate package)

### Quality Assessment

**Strengths:**
- ✅ **Well-structured C++ API** (modern practices)
- ✅ **Real-time capable** (DDS provides <1ms latency)
- ✅ **Examples cover common use cases** (standing, walking)
- ✅ **Active maintenance** (4 releases in 2024-2025)

**Weaknesses:**
- ❌ **Hardware-locked** (cannot run without Unitree robot)
- ⚠️ **Limited documentation** (most info in Chinese docs)
- ⚠️ **Proprietary dependencies** (actual motor drivers are closed-source)

**Overall:** 3.5/5 stars. Excellent reference architecture, but not directly portable to custom hardware.

---

## 4. UnifoLM World Model

**Repository:** [github.com/unitreerobotics/unifolm-world-model-action](https://github.com/unitreerobotics/unifolm-world-model-action)
**License:** Not explicitly stated (inherits from DynamiCrafter, Diffusion Policy, ACT, HPT)
**Project Page:** [unigen-x.github.io/unifolm-world-model-action.github.io](https://unigen-x.github.io/unifolm-world-model-action.github.io/)

### Architecture Overview

**UnifoLM-WMA-0 (World-Model-Action)** is a video generation world model for robotic manipulation.

```
Core Components:
┌─────────────────────────────────────────────────────────┐
│  Image Observation + Text Instruction                   │
└────────────┬────────────────────────────────────────────┘
             │
             v
┌─────────────────────────────────────────────────────────┐
│  World Model (Video Diffusion Transformer)              │
│  - Predicts future 16 frames (~1 second)                │
│  - Conditioned on robot actions                         │
└────────────┬────────────────────────────────────────────┘
             │
             v
┌─────────────────────────────────────────────────────────┐
│  Two Operating Modes:                                   │
│  1. Decision-Making: predict future → assist policy     │
│  2. Simulation: generate synthetic training data        │
└─────────────────────────────────────────────────────────┘
```

### Technical Details

**Model Architecture:**
- Based on video generation models (DynamiCrafter)
- Diffusion-based transformer
- Inputs: RGB image (single camera), robot state (16 DoF max)
- Outputs: Future video frames + predicted actions

**Training Pipeline (3 stages):**
1. **Pre-training:** Fine-tune on Open-X dataset (multi-robot data)
2. **Decision-making post-training:** Task-specific datasets
3. **Simulation post-training:** Same tasks, but optimize for simulation quality

**Model Checkpoints:**
- **UnifoLM-WMA-0 Base:** General capability (Open-X dataset)
- **UnifoLM-WMA-0 Dual:** Unitree-specific (5 Unitree datasets)

### Data Format: LeRobot V2.1

**Compatible with LeRobot dataset standard:**
```python
# Example episode structure
{
    "observation.images.main_view": [H, W, 3],  # RGB camera
    "observation.state": [16],                   # Joint positions
    "action": [16],                              # Target joint positions
    "episode_index": int,
    "frame_index": int
}
```

**Implication:** Theoretically supports any robot with LeRobot-compatible data.

### Hardware Requirements

**From deployment documentation:**
- **Training:** Multi-GPU (A100/H100 recommended)
- **Inference:** Single GPU (RTX 4090 sufficient)
- **Deployment architecture:** Server-client model
  - Server: Runs inference on GPU
  - Robot: Sends observations via network, receives actions
  - Control frequency: 15 Hz (limited by video generation latency)

### Generalization to Custom Robots

| Aspect | Generalizability | Notes |
|--------|------------------|-------|
| **Model architecture** | ✅ 100% | Not Unitree-specific |
| **Data format** | ✅ 100% | LeRobot V2.1 is robot-agnostic |
| **DoF support** | ✅ 90% | Default 16 DoF, configurable |
| **Pre-trained weights** | ⚠️ 50% | Base model trained on multi-robot data, but... |
| **Deployment code** | ⚠️ 40% | Examples use unitree_sdk2 |
| **Practical use** | ⚠️ 30% | All published results on Unitree robots |

**Reality check:** While the architecture is generalizable, **no evidence of successful deployment on non-Unitree robots**. Transfer learning from Unitree models to custom quadrupeds would require:
1. Collecting custom robot dataset in LeRobot format
2. Fine-tuning UnifoLM-WMA-0 Base on custom data
3. Validating sim-to-real transfer

### Relevance to Perception/Navigation

**Limited applicability for CleanWalker's use case:**
- ❌ **Not a SLAM/navigation model** (focused on manipulation)
- ❌ **Single camera view** (no multi-view geometric understanding)
- ⚠️ **Manipulation-centric:** Trained on pick-and-place tasks (not locomotion)

**Potential future use:** If CleanWalker adds manipulation (trash pickup), UnifoLM could model object interactions.

### Quality Assessment

**Strengths:**
- ✅ **State-of-the-art world model** architecture
- ✅ **Open-source implementation** (rare for world models)
- ✅ **LeRobot compatibility** (standardized data pipeline)

**Weaknesses:**
- ❌ **Compute-intensive** (video generation is slow)
- ❌ **No locomotion examples** (all demos are manipulation)
- ⚠️ **Unproven on non-Unitree robots**

**Overall:** 3/5 stars for CleanWalker use case. Interesting research direction, but not immediately applicable.

---

## 5. URDF Structure and Best Practices

**Repositories:**
- [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (ROS1, URDF/xacro)
- [unitree_model](https://github.com/unitreerobotics/unitree_model) (USD format, deprecated)

### Repository Overview: unitree_ros

```
unitree_ros/
├── robots/                  # 18+ robot variants
│   ├── a1_description/
│   │   ├── urdf/           # a1.urdf (assembled)
│   │   ├── xacro/          # a1.xacro (modular)
│   │   └── meshes/         # visual and collision
│   ├── go2_description/
│   └── ...
├── unitree_controller/      # Quadruped joint controllers
├── unitree_legged_control/  # Gazebo plugins
└── unitree_gazebo/          # Simulation worlds
```

### Joint Naming Convention

**Standardized across all Unitree quadrupeds:**

```
Body structure (viewed from above):
    FL (Front Left)      FR (Front Right)
         \                /
          \--------------/
          |    Body     |
          /--------------\
         /                \
    RL (Rear Left)      RR (Rear Right)

Per-leg joints (3 DoF each):
- {FL,FR,RL,RR}_hip_joint       (abduction/adduction)
- {FL,FR,RL,RR}_thigh_joint     (forward/back upper leg)
- {FL,FR,RL,RR}_calf_joint      (knee)

Example: Front-left leg:
- FL_hip_joint
- FL_thigh_joint
- FL_calf_joint
```

**Implication for custom quadruped:** Follow this convention for compatibility with existing controllers and RL code.

### URDF Organization Patterns

**1. Modular xacro files:**
```xml
<!-- go2.xacro (main file) -->
<robot name="go2" xmlns:xacro="http://ros.org/wiki/xacro">
    <!-- Include macros -->
    <xacro:include filename="go2_const.xacro"/>  <!-- Constants -->
    <xacro:include filename="leg.xacro"/>        <!-- Leg macro -->

    <!-- Body link -->
    <link name="base_link">
        <visual>
            <geometry>
                <mesh filename="meshes/body.dae"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <mesh filename="meshes/body_collision.stl"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="6.5"/>
            <inertia ixx="0.0377" ixy="0" ixz="0" ... />
        </inertial>
    </link>

    <!-- Instantiate 4 legs using macro -->
    <xacro:leg prefix="FL" parent="base_link" .../>
    <xacro:leg prefix="FR" parent="base_link" .../>
    <xacro:leg prefix="RL" parent="base_link" .../>
    <xacro:leg prefix="RR" parent="base_link" .../>
</robot>
```

**Benefits:**
- DRY principle (Don't Repeat Yourself)
- Easy to modify leg parameters
- Maintainable across robot variants

**2. Collision vs Visual Meshes:**
```
meshes/
├── visual/
│   ├── body.dae          # High-poly, textured (for visualization)
│   ├── thigh.dae
│   └── calf.dae
└── collision/
    ├── body.stl          # Low-poly, convex (for physics)
    ├── thigh_simplified.stl
    └── calf_cylinder.stl  # Often simple primitives (cylinder, box)
```

**Best practice:** Collision meshes should be 10-100x fewer polygons than visual meshes. From [ROS documentation](https://abedgnu.github.io/Notes-ROS/chapters/ROS/10_robot_modeling/urdf.html):
> "A coarse collision mesh can save processing time while performing collision checks, and the collision geometry can be built slightly larger than the visual geometry to guarantee safe zones."

**3. Inertial Properties:**
```xml
<inertial>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- Center of mass offset -->
    <mass value="1.2"/>
    <inertia ixx="0.0045" ixy="0" ixz="0"
             iyy="0.0052" iyz="0"
             izz="0.0028"/>
</inertial>
```

**Critical for Isaac Sim/Lab:** From [Isaac Lab import tutorial](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html):
> "Ensure every link has nonzero mass and realistic inertia, as the URDF must specify these; otherwise, import tools will estimate or use defaults, which can cause erratic movement."

**Tool for calculating inertia:** SolidWorks, FreeCAD, or `meshlab` + inertia scripts.

### Gazebo Integration Patterns

**Gazebo-specific tags (strip for Isaac Sim):**
```xml
<!-- ros_control plugin -->
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/go2</robotNamespace>
    </plugin>
</gazebo>

<!-- Joint transmission -->
<transmission name="FL_hip_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="FL_hip_joint">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="FL_hip_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

**Isaac Lab does NOT support:**
- `<gazebo>` tags
- `<transmission>` tags

**Solution:** Strip these before import, or use Isaac Lab's `ArticulationCfg` which handles actuators natively.

### Key Learnings for Custom URDF

**1. File Structure:**
```
my_robot_description/
├── urdf/
│   └── my_robot.urdf          # Assembled (generated from xacro)
├── xacro/
│   ├── my_robot.xacro         # Main file
│   ├── const.xacro            # Constants (dimensions, masses)
│   ├── leg.xacro              # Reusable leg macro
│   └── materials.xacro        # Visual colors/textures
├── meshes/
│   ├── visual/                # DAE or OBJ (with textures)
│   └── collision/             # STL (simple geometry)
├── launch/
│   └── display.launch         # RViz visualization
└── config/
    └── joint_limits.yaml      # Safety limits
```

**2. Joint Limits and Dynamics:**
```xml
<joint name="FL_hip_joint" type="revolute">
    <origin xyz="0.2 0.15 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="FL_hip"/>
    <axis xyz="1 0 0"/>
    <!-- Critical: realistic limits -->
    <limit effort="30.0" velocity="15.0" lower="-0.8" upper="0.8"/>
    <!-- Dynamics for simulation -->
    <dynamics damping="0.1" friction="0.01"/>
</joint>
```

**3. Self-Collision Detection:**
```xml
<!-- In Isaac Sim URDF import settings -->
<isaac:selfCollide>true</isaac:selfCollide>
```

**In Isaac Lab:**
```python
ArticulationCfg(
    spawn=UsdFileCfg(
        articulation_props=ArticulationRootPropertiesCfg(
            enable_self_collisions=True
        )
    )
)
```

**4. Parameterization for Variants:**
```xml
<!-- const.xacro -->
<xacro:property name="body_length" value="0.45"/>
<xacro:property name="body_width" value="0.30"/>
<xacro:property name="leg_length" value="0.25"/>

<!-- Create multiple robot sizes by changing constants -->
<xacro:arg name="robot_scale" default="1.0"/>
<xacro:property name="body_length" value="${0.45 * robot_scale}"/>
```

### Quality Assessment

**Strengths:**
- ✅ **Production-quality URDFs** (used in real Go2/H1 robots)
- ✅ **Modular xacro design** (easy to adapt)
- ✅ **Clean mesh organization** (visual vs collision separation)
- ✅ **Realistic inertial properties** (measured from hardware)

**Weaknesses:**
- ⚠️ **Gazebo-specific tags** (need cleanup for Isaac Sim)
- ⚠️ **Limited documentation** on URDF conventions
- ⚠️ **No parametric design** (fixed dimensions, not configurable)

**Overall:** 4.5/5 stars. Excellent reference implementation for custom quadruped URDFs.

---

## Summary Table: Reusability Assessment

| Repository | Primary Purpose | Reusability | License | Best For |
|------------|----------------|-------------|---------|----------|
| **unitree_rl_gym** | RL training (Isaac Gym) | ⭐⭐⭐⭐☆ (4/5) | BSD-3 | Custom quadruped RL training |
| **unitree_rl_lab** | RL training (Isaac Lab) | ⭐⭐⭐⭐⭐ (5/5) | Apache-2.0 | Modern RL projects (recommended) |
| **unitree_sdk2** | Low-level motor control | ⭐⭐⭐☆☆ (3/5) | BSD-3 | Reference architecture only |
| **UnifoLM** | World model (manipulation) | ⭐⭐☆☆☆ (2/5) | Mixed | Research/future manipulation |
| **unitree_ros (URDF)** | Robot description | ⭐⭐⭐⭐⭐ (5/5) | BSD-3 | URDF best practices |

### Reusability Breakdown by Component

```
┌─────────────────────────────────────────────────────────────────┐
│ Component               │ Direct Use │ Reference │ Not Applicable │
├─────────────────────────┼────────────┼───────────┼────────────────┤
│ RL Training Pipeline    │     ✅     │           │                │
│ Base Environment (gym)  │     ✅     │           │                │
│ Reward Functions        │     ✅     │           │                │
│ URDF Structure          │     ✅     │           │                │
│ Isaac Lab Integration   │     ✅     │           │                │
│ PPO Algorithm (rsl_rl)  │     ✅     │           │                │
├─────────────────────────┼────────────┼───────────┼────────────────┤
│ DDS Architecture        │            │     ✅    │                │
│ Motor Control Patterns  │            │     ✅    │                │
│ Safety/Fault Handling   │            │     ✅    │                │
│ Deployment Scripts      │            │     ✅    │                │
├─────────────────────────┼────────────┼───────────┼────────────────┤
│ unitree_sdk2 Hardware   │            │           │       ❌       │
│ Pre-trained Models      │            │           │       ❌       │
│ Go2-Specific Configs    │            │           │       ❌       │
│ UnifoLM (manipulation)  │            │           │       ❌       │
└─────────────────────────────────────────────────────────────────┘
```

---

## Recommendations for CleanWalker Development

### Immediate Actions (High Priority)

1. **Fork unitree_rl_lab** as RL training foundation
   - **Why:** Best modularity, Isaac Lab is actively maintained by NVIDIA
   - **Action:** Create custom task inheriting from base environment
   - **Timeline:** 1-2 weeks to set up basic training pipeline

2. **Adapt URDF structure** from unitree_ros
   - **Why:** Proven organization, compatible with Isaac Lab
   - **Action:** Create `cleanwalker_description` package following Unitree conventions
   - **Key files:**
     ```
     cleanwalker_description/
     ├── xacro/cleanwalker.xacro
     ├── meshes/visual/*.dae
     └── meshes/collision/*.stl
     ```
   - **Timeline:** 1 week (assuming CAD models exist)

3. **Set up rsl_rl training** with Isaac Lab
   - **Why:** GPU-accelerated, proven on legged robots
   - **Action:** Install Isaac Sim 5.1.0+ → Isaac Lab → unitree_rl_lab
   - **Timeline:** 2-3 days setup + iterative tuning

### Medium-Term (Reference Architecture)

4. **Study unitree_sdk2** for control architecture
   - **Why:** Best practices for real-time motor control
   - **Action:** Design custom DDS interface OR use ROS2 Control
   - **Note:** Cannot directly use SDK (Go2-specific), but patterns are transferable
   - **Timeline:** 1 week design + 2 weeks implementation

5. **Implement sim2sim validation** with MuJoCo
   - **Why:** Isaac Lab policies may overfit to PhysX
   - **Action:** Use unitree_rl_lab's MuJoCo deployment as template
   - **Timeline:** 1 week after initial training works

### Long-Term (Research)

6. **Explore UnifoLM** if adding manipulation
   - **Why:** State-of-the-art world model for pick-and-place
   - **Condition:** Only if CleanWalker needs trash pickup/placement
   - **Timeline:** 4-6 weeks (data collection + fine-tuning)

7. **Contribute custom robot examples** back to Isaac Lab
   - **Why:** Community support, validation of approach
   - **Action:** Publish CleanWalker training environment as Isaac Lab extension
   - **Timeline:** After successful sim2real transfer

---

## Technical Debt and Risks

### Dependency Hell

**Risk:** Isaac Lab 2.3.0+ requires IsaacSim 5.1.0+, which is resource-intensive.

**Mitigation:**
- Allocate 32GB+ GPU for training (RTX 4090 / A100)
- Use cloud GPUs if local resources insufficient (Lambda Labs, RunPod)

### Sim2Real Gap

**Risk:** Policies trained in Isaac Lab may not transfer to real hardware.

**Mitigation (from Unitree's approach):**
1. **Domain randomization:** Friction, mass, motor delays
2. **Noisy observations:** Add Gaussian noise to sensor readings
3. **Actuator network:** Model motor response lag
4. **Terrain randomization:** Train on rough terrain, test on flat
5. **Sim2sim validation:** MuJoCo → PhysX consistency check

### Custom Hardware Integration

**Risk:** unitree_sdk2 patterns assume high-quality actuators (CAN bus, 1kHz control).

**Mitigation:**
- Validate actuator specs (bandwidth, latency) before committing to hardware
- Consider off-the-shelf options: T-Motor AK series, ODrive, Moteus

---

## Licensing Summary

| Repository | License | Commercial Use | Attribution Required | Copyleft |
|------------|---------|----------------|---------------------|----------|
| unitree_rl_gym | BSD-3 | ✅ Yes | ✅ Yes | ❌ No |
| unitree_rl_lab | Apache-2.0 | ✅ Yes | ✅ Yes | ❌ No |
| unitree_sdk2 | BSD-3 | ✅ Yes | ✅ Yes | ❌ No |
| rsl_rl | BSD-3 | ✅ Yes | ✅ Yes | ❌ No |
| unitree_ros | BSD-3 | ✅ Yes | ✅ Yes | ❌ No |
| Isaac Lab | Apache-2.0 | ✅ Yes | ✅ Yes | ❌ No |

**Key takeaway:** All licenses are permissive (no GPL contamination). CleanWalker can use these codebases commercially with proper attribution.

---

## Conclusion

Unitree's open-source ecosystem provides **substantial reusable infrastructure** for custom quadruped development:

1. **RL Training:** unitree_rl_lab (Isaac Lab) is production-ready and highly modular
2. **URDF Design:** unitree_ros provides excellent organizational patterns
3. **Control Architecture:** unitree_sdk2 demonstrates real-time motor control best practices
4. **Algorithm Libraries:** rsl_rl (PPO) is hardware-agnostic and GPU-optimized

**Recommended stack for CleanWalker:**
```
┌─────────────────────────────────────┐
│  Training: Isaac Lab + rsl_rl       │  ← Fork unitree_rl_lab
├─────────────────────────────────────┤
│  Simulation: IsaacSim + MuJoCo      │  ← Sim2sim validation
├─────────────────────────────────────┤
│  Robot Model: Custom URDF           │  ← Follow unitree_ros patterns
├─────────────────────────────────────┤
│  Control Interface: ROS2 Control    │  ← Inspired by unitree_sdk2
│  OR Custom DDS (CycloneDDS)         │
├─────────────────────────────────────┤
│  Hardware: Custom Actuators         │  ← Not Unitree-specific
└─────────────────────────────────────┘
```

**Next Steps:**
1. Set up Isaac Lab + unitree_rl_lab development environment
2. Create CleanWalker URDF following unitree_ros conventions
3. Begin RL training with basic locomotion task
4. Iterate on reward functions and domain randomization

---

## Sources

- [unitree_rl_gym GitHub Repository](https://github.com/unitreerobotics/unitree_rl_gym)
- [unitree_rl_lab GitHub Repository](https://github.com/unitreerobotics/unitree_rl_lab)
- [unitree_sdk2 GitHub Repository](https://github.com/unitreerobotics/unitree_sdk2)
- [UnifoLM World Model Repository](https://github.com/unitreerobotics/unifolm-world-model-action)
- [UnifoLM Project Page](https://unigen-x.github.io/unifolm-world-model-action.github.io/)
- [unitree_ros GitHub Repository](https://github.com/unitreerobotics/unitree_ros)
- [unitree_model GitHub Repository](https://github.com/unitreerobotics/unitree_model)
- [RSL-RL GitHub Repository](https://github.com/leggedrobotics/rsl_rl)
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/)
- [Isaac Lab: Importing a New Asset Tutorial](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [Unitree Robotics Documentation Center](https://support.unitree.com/home/en/developer)
- [Legged Gym GitHub Repository](https://github.com/leggedrobotics/legged_gym)
- [ROS URDF Tutorial](https://abedgnu.github.io/Notes-ROS/chapters/ROS/10_robot_modeling/urdf.html)

---

**Document Version:** 1.0
**Last Updated:** 2026-02-11
**Author:** Research Team
**Status:** Complete
