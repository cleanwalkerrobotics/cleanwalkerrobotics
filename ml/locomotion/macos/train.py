#!/usr/bin/env python3
"""Train CW-1 locomotion policy using PPO on macOS Apple Silicon.

Uses stable-baselines3 with PyTorch MPS backend for GPU acceleration.
Policy architecture matches IsaacLab config: MLP [512, 256, 128].

Usage:
    python train.py                          # Train 2M steps
    python train.py --timesteps 500000       # Shorter run
    python train.py --resume                 # Resume from latest checkpoint
    python train.py --device cpu             # Force CPU training

Monitor training:
    tensorboard --logdir logs/
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

# Enable MPS fallback for unsupported operations
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()


def get_device(requested: str = "auto") -> str:
    """Select the best available device."""
    if requested != "auto":
        return requested

    if torch.backends.mps.is_available():
        print("Using Apple MPS (Metal Performance Shaders) GPU")
        return "mps"
    elif torch.cuda.is_available():
        print("Using CUDA GPU")
        return "cuda"
    else:
        print("Using CPU (MPS not available — check macOS 12.3+ and PyTorch 2.0+)")
        return "cpu"


def find_latest_checkpoint(checkpoint_dir: Path) -> Path | None:
    """Find the most recent checkpoint file."""
    if not checkpoint_dir.exists():
        return None

    checkpoints = sorted(
        checkpoint_dir.glob("cw1_ppo_*_steps.zip"),
        key=lambda p: int(p.stem.split("_")[-2]),
    )
    return checkpoints[-1] if checkpoints else None


def make_env(render_mode=None):
    """Create the CW-1 locomotion environment."""
    # Import here to ensure cw1_env is on the path
    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    return CW1LocomotionEnv(render_mode=render_mode)


def main():
    parser = argparse.ArgumentParser(
        description="Train CW-1 locomotion policy with PPO"
    )
    parser.add_argument(
        "--timesteps", type=int, default=2_000_000,
        help="Total training timesteps (default: 2M, ~30-60 min on M4 Max)"
    )
    parser.add_argument(
        "--device", type=str, default="auto",
        choices=["auto", "mps", "cuda", "cpu"],
        help="Training device"
    )
    parser.add_argument(
        "--resume", action="store_true",
        help="Resume from latest checkpoint"
    )
    parser.add_argument(
        "--checkpoint-freq", type=int, default=50_000,
        help="Save checkpoint every N steps"
    )
    parser.add_argument(
        "--log-dir", type=Path, default=SCRIPT_DIR / "logs",
        help="TensorBoard log directory"
    )
    parser.add_argument(
        "--checkpoint-dir", type=Path, default=SCRIPT_DIR / "checkpoints",
        help="Checkpoint save directory"
    )
    parser.add_argument(
        "--lr", type=float, default=1e-3,
        help="Learning rate (default: 1e-3, matching Phase 1)"
    )
    parser.add_argument(
        "--batch-size", type=int, default=64,
        help="Mini-batch size"
    )
    parser.add_argument(
        "--n-steps", type=int, default=2048,
        help="Steps per rollout buffer collection"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("CW-1 Locomotion Training — macOS Apple Silicon")
    print("=" * 60)

    # Select device
    device = get_device(args.device)

    # stable-baselines3 PPO has some operations that don't support MPS yet.
    # Use CPU for SB3 but let PyTorch handle MPS internally where possible.
    sb3_device = "cpu" if device == "mps" else device
    if device == "mps":
        print("Note: stable-baselines3 will use CPU (MPS partial support).")
        print("      PyTorch MPS is used for tensor operations via fallback.")

    # Create directories
    args.log_dir.mkdir(parents=True, exist_ok=True)
    args.checkpoint_dir.mkdir(parents=True, exist_ok=True)

    # Check that MJCF exists
    mjcf_path = SCRIPT_DIR / "cw1_scene.xml"
    if not mjcf_path.exists():
        print(f"Error: MJCF not found at {mjcf_path}")
        print("Run: python convert_urdf_to_mjcf.py")
        sys.exit(1)

    print(f"\nDevice: {device} (SB3 device: {sb3_device})")
    print(f"Timesteps: {args.timesteps:,}")
    print(f"Learning rate: {args.lr}")
    print(f"Batch size: {args.batch_size}")
    print(f"Rollout steps: {args.n_steps}")
    print(f"Checkpoint frequency: {args.checkpoint_freq:,}")
    print(f"Log directory: {args.log_dir}")
    print()

    # Import SB3 after torch setup
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import (
        BaseCallback,
        CheckpointCallback,
    )

    # Custom progress callback
    class ProgressCallback(BaseCallback):
        """Print training progress with ETA."""

        def __init__(self, total_timesteps: int, print_freq: int = 10_000):
            super().__init__()
            self.total_timesteps = total_timesteps
            self.print_freq = print_freq
            self.start_time = None

        def _on_training_start(self):
            self.start_time = time.time()

        def _on_step(self) -> bool:
            if self.num_timesteps % self.print_freq == 0:
                elapsed = time.time() - self.start_time
                progress = self.num_timesteps / self.total_timesteps
                if progress > 0:
                    eta = elapsed / progress - elapsed
                    eta_min = int(eta // 60)
                    eta_sec = int(eta % 60)
                    steps_per_sec = self.num_timesteps / elapsed
                    print(
                        f"  Step {self.num_timesteps:>9,} / {self.total_timesteps:,} "
                        f"({progress * 100:.1f}%) | "
                        f"{steps_per_sec:.0f} steps/s | "
                        f"ETA: {eta_min}m {eta_sec}s"
                    )
            return True

    # Create environment
    print("Creating CW-1 environment...")
    env = make_env()
    print(f"  Observation space: {env.observation_space.shape}")
    print(f"  Action space: {env.action_space.shape}")

    # Create or load model
    if args.resume:
        checkpoint = find_latest_checkpoint(args.checkpoint_dir)
        if checkpoint:
            print(f"\nResuming from: {checkpoint}")
            model = PPO.load(checkpoint, env=env, device=sb3_device)
            remaining = args.timesteps - model.num_timesteps
            print(f"  Previously trained: {model.num_timesteps:,} steps")
            print(f"  Remaining: {remaining:,} steps")
        else:
            print("No checkpoint found. Starting fresh.")
            args.resume = False

    if not args.resume:
        print("\nCreating PPO model...")
        # Policy architecture matching IsaacLab: [512, 256, 128] with ELU
        policy_kwargs = dict(
            net_arch=dict(pi=[512, 256, 128], vf=[512, 256, 128]),
            activation_fn=torch.nn.ELU,
        )

        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=args.lr,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=5,           # PPO epochs per iteration
            gamma=0.99,           # Discount factor
            gae_lambda=0.95,      # GAE lambda
            clip_range=0.2,       # PPO clip
            ent_coef=0.01,        # Entropy bonus
            vf_coef=1.0,          # Value function coefficient
            max_grad_norm=1.0,    # Gradient clipping
            policy_kwargs=policy_kwargs,
            tensorboard_log=str(args.log_dir),
            verbose=1,
            device=sb3_device,
        )
        print(f"  Policy params: {sum(p.numel() for p in model.policy.parameters()):,}")

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=args.checkpoint_freq,
        save_path=str(args.checkpoint_dir),
        name_prefix="cw1_ppo",
        verbose=1,
    )
    progress_callback = ProgressCallback(args.timesteps)

    # Train
    print(f"\nStarting training for {args.timesteps:,} timesteps...")
    print("Monitor: tensorboard --logdir logs/")
    print("-" * 60)

    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=[checkpoint_callback, progress_callback],
            tb_log_name="cw1_ppo",
            reset_num_timesteps=not args.resume,
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")

    # Save final model
    final_path = SCRIPT_DIR / "cw1_policy_final"
    model.save(str(final_path))
    print(f"\nFinal model saved to: {final_path}.zip")

    # Print summary
    elapsed = time.time() - progress_callback.start_time
    print(f"\nTraining complete!")
    print(f"  Total time: {int(elapsed // 60)}m {int(elapsed % 60)}s")
    print(f"  Steps: {model.num_timesteps:,}")
    print(f"  Avg speed: {model.num_timesteps / elapsed:.0f} steps/s")
    print(f"\nNext steps:")
    print(f"  python play.py                    # Watch the policy")
    print(f"  python export_policy.py           # Export to ONNX")
    print(f"  python render_demo.py             # Render demo video")

    env.close()


if __name__ == "__main__":
    main()
