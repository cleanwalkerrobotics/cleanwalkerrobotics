# CW-1 Locomotion Training — macOS Apple Silicon

Train the CleanWalker CW-1 quadruped locomotion policy on your Mac using MuJoCo + PyTorch MPS.

**Target hardware:** Apple M4 Max (40-core GPU, 32GB unified RAM)
**Expected training time:** ~30-60 min for 2M steps

## Quick Start

```bash
# 1. Install Python 3.11+ (if not already installed)
brew install python@3.11

# 2. Create virtual environment
cd ml/locomotion/macos
python3.11 -m venv .venv
source .venv/bin/activate

# 3. Install dependencies
pip install -r requirements-macos.txt

# 4. Verify MPS backend
python -c "import torch; print('MPS available:', torch.backends.mps.is_available())"

# 5. Convert URDF to MuJoCo format
python convert_urdf_to_mjcf.py

# 6. Train (2M steps, ~30-60 min on M4 Max)
python train.py

# 7. Watch the trained policy walk
python play.py

# 8. Export for deployment
python export_policy.py

# 9. Render a polished demo video
python render_demo.py
```

## What's in This Package

| File | Purpose |
|------|---------|
| `convert_urdf_to_mjcf.py` | Convert CW-1 URDF to MuJoCo MJCF XML |
| `cw1_env.py` | Gymnasium environment (48-dim obs, 12-dim action) |
| `train.py` | PPO training with stable-baselines3 |
| `play.py` | Interactive MuJoCo viewer with trained policy |
| `export_policy.py` | Export trained policy to ONNX |
| `render_demo.py` | Render polished 1080p demo video |

## Environment Details

Matches our IsaacLab configuration:

- **Observation space (48 dims):** base linear velocity (3), base angular velocity (3), projected gravity (3), velocity commands (3), joint positions (12), joint velocities (12), previous actions (12)
- **Action space (12 dims):** leg joint torques (hip yaw, hip pitch, knee pitch x 4 legs)
- **Reward function:** forward velocity tracking, energy penalty, orientation penalty, joint limit penalty, alive bonus, smoothness penalty, contact penalty
- **Physics:** 200 Hz simulation, 50 Hz control (decimation=4)
- **Episode:** 1000 steps (20 seconds), terminates on fall

## MPS Backend Notes

PyTorch's MPS (Metal Performance Shaders) backend accelerates tensor operations on Apple GPU.

- MPS works for policy inference and gradient computation
- Some operations may fall back to CPU automatically — this is normal
- If you see MPS errors, set `PYTORCH_MPS_FALLBACK=1` in your environment:
  ```bash
  export PYTORCH_ENABLE_MPS_FALLBACK=1
  ```
- stable-baselines3 uses CPU for some internal operations; the policy network runs on MPS

## Training Tips

- **Shorter runs first:** Try `python train.py --timesteps 100000` to verify everything works
- **Monitor:** Run `tensorboard --logdir logs/` in another terminal to watch training curves
- **Resume:** Training auto-saves checkpoints; use `python train.py --resume` to continue
- **Adjust reward:** Edit reward weights in `cw1_env.py` if the robot doesn't walk well

## Directory Structure After Training

```
macos/
├── cw1_scene.xml          # Generated MuJoCo scene (after convert)
├── logs/                  # TensorBoard logs
│   └── cw1_ppo_*/
├── checkpoints/           # Saved models
│   ├── cw1_ppo_50000_steps.zip
│   ├── cw1_ppo_100000_steps.zip
│   └── ...
├── cw1_policy_final.zip   # Final trained policy
├── cw1_policy.onnx        # Exported ONNX model
└── renders/               # Demo videos
    └── cw1-walking-demo.mp4
```

## Troubleshooting

**"MPS not available"** — Requires macOS 12.3+ and PyTorch 2.0+. Update both.

**MuJoCo viewer won't open** — Install via `brew install glfw` if you get OpenGL errors.

**Training is slow** — Check that MPS is being used: look for "Device: mps" in training output. If it says "cpu", your PyTorch install may not support MPS.

**Robot falls immediately** — Normal for the first ~100k steps. The policy needs time to learn balance. If it still falls after 500k steps, check the MJCF conversion.
