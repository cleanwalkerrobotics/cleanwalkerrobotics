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


def make_env(render_mode=None, rank=0, terrain_types=None, refine=False):
    """Create a factory function for the CW-1 locomotion environment."""
    def _init():
        sys.path.insert(0, str(SCRIPT_DIR))
        from cw1_env import CW1LocomotionEnv
        # Phase 2 shaped reward weights (only when refining an existing walker)
        refine_kwargs = {}
        if refine:
            refine_kwargs = dict(
                rew_gait_force=1.0,
                rew_gait_vel=1.0,
                rew_clearance_target=-30.0,
                rew_action_smooth_1=-0.1,
                rew_action_smooth_2=-0.1,
                rew_foot_slip=-0.25,
                rew_air_time_var=-1.0,
                rew_hip_vel=-1e-2,
                rew_torques=-1e-4,   # 10x stronger during refinement
            )
        env = CW1LocomotionEnv(
            render_mode=render_mode,
            terrain_types=terrain_types,
            **refine_kwargs,
        )
        env.reset(seed=rank)
        return env
    return _init


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
        "--resume-path", type=Path, default=None,
        help="Resume from a specific checkpoint file"
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
        "--lr", type=float, default=3e-4,
        help="Learning rate (default: 3e-4, adapts via KL if enabled)"
    )
    parser.add_argument(
        "--batch-size", type=int, default=64,
        help="Mini-batch size"
    )
    parser.add_argument(
        "--n-envs", type=int, default=8,
        help="Number of parallel environments"
    )
    parser.add_argument(
        "--n-steps", type=int, default=2048,
        help="Steps per rollout buffer collection (per env)"
    )
    parser.add_argument(
        "--adaptive-lr", action="store_true", default=True,
        help="Use KL-divergence adaptive learning rate (default: on)"
    )
    parser.add_argument(
        "--no-adaptive-lr", action="store_false", dest="adaptive_lr",
        help="Disable adaptive learning rate"
    )
    parser.add_argument(
        "--terrain", type=str, nargs="+", default=["flat"],
        help="Terrain types for training (e.g. --terrain flat rough obstacles)"
    )
    parser.add_argument(
        "--refine", action="store_true",
        help="Enable Phase 2 shaped rewards (clearance target, smoothness, slip)"
    )
    parser.add_argument(
        "--ent-coef", type=float, default=0.01,
        help="Entropy coefficient (default: 0.01, higher = more exploration)"
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
    print(f"Parallel envs: {args.n_envs}")
    print(f"Timesteps: {args.timesteps:,}")
    print(f"Learning rate: {args.lr}")
    print(f"Batch size: {args.batch_size}")
    print(f"Rollout steps: {args.n_steps}")
    print(f"Terrain types: {args.terrain}")
    if args.refine:
        print("Phase 2 REFINE mode: shaped rewards enabled (clearance, smoothness, slip)")
    print(f"Checkpoint frequency: {args.checkpoint_freq:,}")
    print(f"Log directory: {args.log_dir}")
    print()

    # Import SB3 after torch setup
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import (
        BaseCallback,
        CheckpointCallback,
    )
    from stable_baselines3.common.vec_env import SubprocVecEnv

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

    # KL-adaptive learning rate callback (from rsl_rl / IsaacLab)
    class AdaptiveLRCallback(BaseCallback):
        """Adjust learning rate based on approximate KL divergence.

        If KL > 2 * target: halve LR (policy changing too fast)
        If KL < target / 2: double LR (policy not learning enough)
        Bounds: [1e-5, 1e-2]

        This is the industry standard from rsl_rl used by Unitree, ANYmal, etc.
        Prevents PPO policy collapse during extended training.
        """

        def __init__(self, kl_target: float = 0.01, lr_min: float = 1e-5,
                     lr_max: float = 1e-2, verbose: int = 0):
            super().__init__(verbose)
            self.kl_target = kl_target
            self.lr_min = lr_min
            self.lr_max = lr_max
            self._last_lr = None

        def _on_step(self) -> bool:
            return True

        def _on_rollout_end(self):
            # SB3 logs approx_kl after each PPO update
            if len(self.model.logger.name_to_value) > 0:
                approx_kl = self.model.logger.name_to_value.get(
                    "train/approx_kl", None
                )
                if approx_kl is not None:
                    current_lr = self.model.learning_rate
                    if callable(current_lr):
                        current_lr = current_lr(1.0)

                    new_lr = current_lr
                    if approx_kl > 2.0 * self.kl_target:
                        new_lr = max(current_lr * 0.5, self.lr_min)
                    elif approx_kl < self.kl_target / 2.0:
                        new_lr = min(current_lr * 2.0, self.lr_max)

                    if new_lr != current_lr:
                        from stable_baselines3.common.utils import get_schedule_fn
                        self.model.learning_rate = new_lr
                        self.model.lr_schedule = get_schedule_fn(new_lr)
                        if self.verbose > 0 or (self._last_lr != new_lr):
                            direction = "↑" if new_lr > current_lr else "↓"
                            print(
                                f"  [AdaptiveLR] KL={approx_kl:.4f} "
                                f"(target={self.kl_target}) → "
                                f"LR {current_lr:.2e} {direction} {new_lr:.2e}"
                            )
                        self._last_lr = new_lr

    # Create parallel environments
    print(f"Creating {args.n_envs} parallel CW-1 environments...")
    env = SubprocVecEnv([
        make_env(rank=i, terrain_types=args.terrain, refine=args.refine)
        for i in range(args.n_envs)
    ])
    print(f"  Observation space: {env.observation_space.shape}")
    print(f"  Action space: {env.action_space.shape}")
    print(f"  Parallel envs: {args.n_envs}")

    # Create or load model
    loaded = False
    if args.resume_path:
        print(f"\nLoading from: {args.resume_path}")
        model = PPO.load(str(args.resume_path), env=env, device=sb3_device)
        print(f"  Previously trained: {model.num_timesteps:,} steps")
        # Reset num_timesteps for fresh training count
        loaded = True
    elif args.resume:
        checkpoint = find_latest_checkpoint(args.checkpoint_dir)
        if checkpoint:
            print(f"\nResuming from: {checkpoint}")
            model = PPO.load(checkpoint, env=env, device=sb3_device)
            remaining = args.timesteps - model.num_timesteps
            print(f"  Previously trained: {model.num_timesteps:,} steps")
            print(f"  Remaining: {remaining:,} steps")
            loaded = True
        else:
            print("No checkpoint found. Starting fresh.")

    if not loaded:
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
            ent_coef=args.ent_coef,  # Entropy bonus
            vf_coef=1.0,          # Value function coefficient
            max_grad_norm=1.0,    # Gradient clipping
            policy_kwargs=policy_kwargs,
            tensorboard_log=str(args.log_dir),
            verbose=1,
            device=sb3_device,
        )
        print(f"  Policy params: {sum(p.numel() for p in model.policy.parameters()):,}")

    # Setup callbacks
    # save_freq is per-env steps; divide by n_envs for total-timestep frequency
    effective_save_freq = max(1, args.checkpoint_freq // args.n_envs)
    checkpoint_callback = CheckpointCallback(
        save_freq=effective_save_freq,
        save_path=str(args.checkpoint_dir),
        name_prefix="cw1_ppo",
        verbose=1,
    )
    progress_callback = ProgressCallback(args.timesteps)

    callbacks = [checkpoint_callback, progress_callback]

    if args.adaptive_lr:
        adaptive_lr_callback = AdaptiveLRCallback(
            kl_target=0.02, lr_min=1e-5, lr_max=1e-2
        )
        callbacks.append(adaptive_lr_callback)
        print("Adaptive LR: ON (KL target=0.02, range=[1e-5, 1e-2])")

    # Train
    print(f"\nStarting training for {args.timesteps:,} timesteps...")
    print("Monitor: tensorboard --logdir logs/")
    print("-" * 60)

    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            tb_log_name="cw1_ppo",
            reset_num_timesteps=not args.resume or bool(args.resume_path),
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
