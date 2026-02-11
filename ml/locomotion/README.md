# CleanWalker CW-1 — Locomotion Training

RL-based locomotion training for the CleanWalker CW-1 quadruped robot using **NVIDIA IsaacLab** and **rsl_rl** PPO.

## Architecture

```
ml/locomotion/
├── __init__.py           # Gymnasium environment registration
├── cleanwalker_env.py    # IsaacLab DirectRLEnv + ArticulationCfg
├── train_config.py       # rsl_rl PPO hyperparameters
├── terrain_config.py     # Procedural terrain generation
└── README.md             # This file
```

**Robot:** 18 DOF total, 12 leg joints actuated for locomotion.
**Policy:** MLP [512, 256, 128] with ELU activation, trained via PPO.
**Sim:** 200 Hz physics, 50 Hz policy (decimation=4), 4096 parallel envs.

## Prerequisites

### Hardware
- NVIDIA GPU with 8+ GB VRAM (RTX 3080+, RTX 4090, A100 recommended)
- 32 GB+ system RAM
- Ubuntu 22.04 or 24.04

### Software Stack

1. **NVIDIA Isaac Sim 4.5+**
   ```bash
   # Install via Omniverse Launcher:
   # https://developer.nvidia.com/isaac-sim
   # Or via pip (Isaac Sim 4.5+):
   pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk
   ```

2. **IsaacLab**
   ```bash
   git clone https://github.com/isaac-sim/IsaacLab.git
   cd IsaacLab
   ./isaaclab.sh --install  # or: pip install -e .
   ```

3. **rsl_rl** (included with IsaacLab, or install separately)
   ```bash
   pip install rsl-rl
   ```

4. **This package**
   ```bash
   cd <repo_root>/ml/locomotion
   pip install -e .  # If setup.py exists, or add to PYTHONPATH
   # OR: export PYTHONPATH="$PYTHONPATH:<repo_root>/ml"
   ```

## URDF to USD Conversion

IsaacLab works best with USD assets. Convert the URDF before training:

```bash
cd <IsaacLab_root>

./isaaclab.sh -p scripts/tools/convert_urdf.py \
    <repo_root>/hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf \
    <repo_root>/hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.usd \
    --merge-joints \
    --make-instanceable \
    --joint-stiffness 0.0 \
    --joint-damping 0.0 \
    --joint-target-type none
```

After conversion, update `CLEANWALKER_URDF_PATH` in `cleanwalker_env.py` to point to the `.usd` file (and change `UrdfFileCfg` to `UsdFileCfg`).

**Note:** The environment also supports direct URDF loading via `UrdfFileCfg`. This is slower but works without pre-conversion.

## Training

### Phase 1: Flat Terrain

```bash
# From IsaacLab root:
./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Flat-v0

# With custom num_envs:
./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Flat-v0 --num_envs 2048

# Headless (no GUI, faster):
./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Flat-v0 --headless
```

Expected: ~1500 iterations, ~2-4 hours on RTX 4090.

### Phase 2: Rough Terrain (Curriculum)

```bash
# Resume from flat-terrain checkpoint:
./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Rough-v0 \
    --load_run <flat_run_name> --checkpoint model_1500.pt
```

Expected: ~3000 iterations, ~6-10 hours on RTX 4090.

### Visualize Trained Policy

```bash
./isaaclab.sh -p scripts/play.py --task CleanWalker-CW1-Flat-v0 \
    --load_run <run_name> --checkpoint model_1500.pt
```

### Monitor Training

```bash
tensorboard --logdir logs/rsl_rl/cleanwalker_cw1_flat
```

## Environment Details

### Observation Space (48 dims)

| Index | Quantity | Dims | Description |
|-------|----------|------|-------------|
| 0-2 | Base linear velocity | 3 | Body-frame velocity (m/s) |
| 3-5 | Base angular velocity | 3 | Body-frame angular velocity (rad/s) |
| 6-8 | Projected gravity | 3 | Gravity vector in body frame |
| 9-11 | Velocity commands | 3 | Target vx, vy, yaw_rate |
| 12-23 | Joint positions (relative) | 12 | Current - default joint positions |
| 24-35 | Joint velocities | 12 | Joint angular velocities |
| 36-47 | Previous actions | 12 | Last policy output |

### Action Space (12 dims)

Position targets for the 12 leg joints, scaled by `action_scale` (0.25) and added to default standing pose:

```
FL: hip_yaw, hip_pitch, knee_pitch
FR: hip_yaw, hip_pitch, knee_pitch
RL: hip_yaw, hip_pitch, knee_pitch
RR: hip_yaw, hip_pitch, knee_pitch
```

### Reward Structure

| Reward | Weight | Type | Purpose |
|--------|--------|------|---------|
| Track linear vel XY | +1.5 | Exponential | Follow velocity commands |
| Track angular vel Z | +0.75 | Exponential | Follow yaw rate commands |
| Feet air time | +0.5 | Linear | Encourage alternating gait |
| Vertical velocity | -2.0 | L2 | Minimize bouncing |
| Roll/pitch angular vel | -0.05 | L2 | Smooth body rotation |
| Joint torques | -2.5e-5 | L2 | Energy efficiency |
| Joint accelerations | -2.5e-7 | L2 | Smoothness |
| Action rate | -0.01 | L2 | Smooth actions |
| Flat orientation | -5.0 | L2 | Keep body level |
| Undesired contacts | -1.0 | Binary | Penalize non-foot contacts |
| Joint position limits | -10.0 | Soft | Stay away from joint limits |
| Stumble | -0.5 | Heuristic | Penalize tripping |

### Domain Randomization

| Parameter | Range | Type |
|-----------|-------|------|
| Ground friction | 0.5 - 1.5 | Multiplicative |
| Base mass | -1.0 to +3.0 kg | Additive |
| Motor strength | 0.8 - 1.2x | Multiplicative |

### Termination Conditions

- Body link contacts the ground (force > 1 N)
- Body orientation exceeds 60 degrees from upright
- Episode timeout (20 seconds)

## CW-1 Joint Reference

```
18 DOF total:
  4 legs x 3 DOF = 12 (actuated for locomotion)
    hip_yaw:   Z-axis rotation, +/- 28.6 deg
    hip_pitch:  Y-axis rotation, +/- 90 deg
    knee_pitch: Y-axis rotation
      Front: [-5.7, 149] deg (bends forward)
      Rear:  [-149, 5.7] deg (bends backward)

  1 arm x 5 DOF = 5 (locked during locomotion)
    turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, gripper

  1 bag hinge = 1 (locked during locomotion)
```

## Customization

### Tuning Reward Weights

Edit the `rew_*` fields in `CleanWalkerFlatEnvCfg` (in `cleanwalker_env.py`). Start with the defaults and adjust based on training behavior:

- **Robot falls over:** Increase `rew_flat_orientation` (more negative)
- **Jerky motion:** Increase `rew_action_rate` and `rew_joint_accel` (more negative)
- **Doesn't follow commands:** Increase `rew_track_lin_vel_xy` and `rew_track_ang_vel_z`
- **Drags feet:** Increase `rew_feet_air_time`
- **Uses too much energy:** Increase `rew_joint_torques` (more negative)

### Changing PD Gains

Edit the `actuators` section in `CLEANWALKER_CW1_CFG`:
```python
"legs": ImplicitActuatorCfg(
    stiffness=25.0,   # Kp — increase for stiffer position tracking
    damping=0.5,      # Kd — increase for more damping
)
```

### Adding Height Scanner (for rough terrain)

For rough terrain, add a `RayCasterCfg` height scanner to the environment config. See the ANYmal-C rough environment in IsaacLab for reference.

## References

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab/main/)
- [IsaacLab: Creating a Direct RL Environment](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_direct_rl_env.html)
- [IsaacLab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [rsl_rl (ETH Zurich)](https://github.com/leggedrobotics/rsl_rl)
- [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) — Reference implementation for Go2
- [ANYmal-C Direct Environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — API reference
