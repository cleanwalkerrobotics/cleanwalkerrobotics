# CleanWalker CW-1 — Locomotion & Litter Collection Training

RL-based locomotion and litter collection training for the CleanWalker CW-1 quadruped robot using **NVIDIA IsaacLab** and **rsl_rl** PPO.

## Architecture

```
ml/locomotion/
├── __init__.py           # Gymnasium environment registration (3 envs)
├── cleanwalker_env.py    # IsaacLab DirectRLEnv + ArticulationCfg
├── train_config.py       # rsl_rl PPO hyperparameters (flat/rough/litter)
├── terrain_config.py     # Procedural terrain generation
├── rewards.py            # Litter collection reward functions
├── train.py              # Main training entry point
├── play.py               # Visualize trained policy
├── export_policy.py      # Export to ONNX for Jetson deployment
├── convert_urdf.py       # URDF → USD conversion for Isaac Sim
├── Dockerfile            # Isaac Sim GPU training container
├── docker-compose.yml    # Docker services (train/export/tensorboard)
├── assets/               # Converted USD assets + exported ONNX models
│   └── .gitkeep
└── README.md             # This file
```

**Robot:** 18 DOF total. 12 leg joints for locomotion, 5 arm joints for litter pickup, 1 bag hinge.
**Policy:** MLP [512, 256, 128] with ELU activation, trained via PPO.
**Sim:** 200 Hz physics, 50 Hz policy (decimation=4), 4096 parallel envs.

## Training Pipeline

Three-phase curriculum:

| Phase | Task ID | Iterations | Time (4090) | Description |
|-------|---------|-----------|-------------|-------------|
| 1 | `CleanWalker-CW1-Flat-v0` | 1,500 | ~3h | Flat terrain locomotion |
| 2 | `CleanWalker-CW1-Rough-v0` | 3,000 | ~8h | Rough terrain curriculum |
| 3 | `CleanWalker-CW1-Litter-v0` | 2,000 | ~5h | Locomotion + arm control |

## Prerequisites

### Hardware
- NVIDIA GPU with 8+ GB VRAM (RTX 3080+, RTX 4090, A100 recommended)
- 32 GB+ system RAM
- Ubuntu 22.04 or 24.04

### Software Stack

1. **NVIDIA Isaac Sim 4.5+**
   ```bash
   pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk
   ```

2. **IsaacLab**
   ```bash
   git clone https://github.com/isaac-sim/IsaacLab.git
   cd IsaacLab && ./isaaclab.sh --install
   ```

3. **rsl_rl** (included with IsaacLab, or install separately)
   ```bash
   pip install rsl-rl
   ```

4. **This package**
   ```bash
   export PYTHONPATH="$PYTHONPATH:<repo_root>/ml"
   ```

## Quick Start

### 1. Convert URDF to USD (optional, improves load time)

```bash
python ml/locomotion/convert_urdf.py
```

### 2. Train Phase 1: Flat Terrain

```bash
python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0 --headless
```

### 3. Train Phase 2: Rough Terrain

```bash
python ml/locomotion/train.py --task CleanWalker-CW1-Rough-v0 \
    --load_run <flat_run_name> --checkpoint model_1500.pt --headless
```

### 4. Train Phase 3: Litter Collection

```bash
python ml/locomotion/train.py --task CleanWalker-CW1-Litter-v0 \
    --load_run <rough_run_name> --checkpoint model_3000.pt --headless
```

### 5. Visualize Trained Policy

```bash
python ml/locomotion/play.py --task CleanWalker-CW1-Flat-v0 \
    --load_run <run_name> --checkpoint model_1500.pt
```

### 6. Export to ONNX for Jetson

```bash
python ml/locomotion/export_policy.py \
    --checkpoint logs/rsl_rl/cleanwalker_cw1_rough/<run>/model_3000.pt \
    --output ml/locomotion/assets/cleanwalker_cw1_policy.onnx --fp16
```

### 7. Monitor Training

```bash
tensorboard --logdir logs/rsl_rl/
```

## Docker Training (Cloud GPU)

### Build and Run

```bash
# Build container
docker compose -f ml/locomotion/docker-compose.yml build

# Phase 1
docker compose -f ml/locomotion/docker-compose.yml up train-flat

# Phase 2
docker compose -f ml/locomotion/docker-compose.yml run train-rough

# Phase 3
docker compose -f ml/locomotion/docker-compose.yml run train-litter

# Export
docker compose -f ml/locomotion/docker-compose.yml run export-policy

# TensorBoard (http://localhost:6006)
docker compose -f ml/locomotion/docker-compose.yml up tensorboard
```

### Cloud GPU Options (24h Training Cost)

| Provider | GPU | $/hr | 24h Cost | Notes |
|----------|-----|------|----------|-------|
| Lambda Labs | A100 80GB | $1.29 | ~$31 | Best value, often sold out |
| Lambda Labs | H100 80GB | $2.49 | ~$60 | Fastest training |
| RunPod | A100 80GB | $1.64 | ~$39 | Good availability |
| **RunPod** | **RTX 4090** | **$0.44** | **~$11** | **Recommended for dev** |
| Vast.ai | RTX 4090 | $0.25 | ~$6 | Cheapest, variable reliability |
| Vast.ai | A100 80GB | $0.90 | ~$22 | Good value if available |

**Recommended:** RunPod RTX 4090 ($0.44/hr) for iteration, Lambda A100 ($1.29/hr) for final runs.

24h budget covers all 3 phases (~16h total) with margin for experimentation.

## Litter Collection Reward Function

Phase 3 adds rewards specific to autonomous litter pickup. This is what differentiates CleanWalker from standard quadruped locomotion.

### Base Rewards (always active)

| Reward | Weight | Purpose |
|--------|--------|---------|
| Velocity tracking (XY) | +1.5 | Follow movement commands |
| Yaw rate tracking | +0.75 | Follow turn commands |
| Feet air time | +0.5 | Encourage alternating gait |
| Vertical velocity | -2.0 | Minimize bouncing |
| Body orientation | -5.0 | Keep body level |
| Joint torques | -2.5e-5 | Energy efficiency |
| Joint accelerations | -2.5e-7 | Smooth motion |
| Action rate | -0.01 | Smooth actions |
| Jerk (2nd derivative) | -0.005 | Ultra-smooth motion |
| Mechanical power | -1.0e-5 | Energy penalty |
| Undesired contacts | -1.0 | Only feet touch ground |
| Joint limits | -10.0 | Stay away from limits |
| Stumble | -0.5 | Penalize tripping |

### Litter Pickup Rewards (active during pick command)

| Reward | Weight | Purpose |
|--------|--------|---------|
| Stop and stabilize | +2.0 | Zero velocity + stable body |
| Arm reach | +3.0 | Arm joints reach pickup config |
| Grasp | +5.0 | Gripper closes when arm at target |
| Smooth deceleration | -2.0 | No slamming to a stop |

### How It Works

1. During 40% of episodes, a **pick command** activates after 3-8 seconds
2. When active, the robot must **decelerate smoothly** (deceleration penalty)
3. Once stopped, it earns reward for **body stability** (tighter than walking)
4. The **arm extends** toward a ground-level target (pre-computed IK)
5. **Gripper closes** only when arm is near target (staged reward)
6. The policy learns to coordinate all 17 joints for the full walk→stop→pick cycle

## Environment Details

### Locomotion (48 obs, 12 act)

| Index | Quantity | Dims |
|-------|----------|------|
| 0-2 | Base linear velocity | 3 |
| 3-5 | Base angular velocity | 3 |
| 6-8 | Projected gravity | 3 |
| 9-11 | Velocity commands | 3 |
| 12-23 | Joint positions (relative) | 12 |
| 24-35 | Joint velocities | 12 |
| 36-47 | Previous actions | 12 |

### Litter Collection (58 obs, 17 act)

Adds to locomotion:

| Index | Quantity | Dims |
|-------|----------|------|
| 48-52 | Arm joint positions (relative) | 5 |
| 53-57 | Arm joint velocities | 5 |

Actions 12-16: turret_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, gripper.

### Domain Randomization

| Parameter | Range | Type |
|-----------|-------|------|
| Ground friction | 0.5 - 1.5 | Multiplicative |
| Base mass | -1.0 to +3.0 kg | Additive |
| Motor strength | 0.8 - 1.2x | Multiplicative |

## ONNX Deployment on Jetson

After exporting to ONNX:

```bash
# On Jetson Orin Nano Super — convert to TensorRT
/usr/src/tensorrt/bin/trtexec \
    --onnx=cleanwalker_cw1_policy.onnx \
    --saveEngine=cleanwalker_cw1_policy.engine \
    --fp16 --workspace=256

# Expected: <1ms inference at 50 Hz
# Input:  float32[1, 48] or [1, 58]
# Output: float32[1, 12] or [1, 17]
```

## References

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab/main/)
- [IsaacLab: Creating a Direct RL Environment](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_direct_rl_env.html)
- [IsaacLab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [rsl_rl (ETH Zurich)](https://github.com/leggedrobotics/rsl_rl)
- [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) — Reference implementation for Go2
- [ANYmal-C Direct Environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — API reference
