#!/usr/bin/env python3
"""Visualize trained CW-1 locomotion policy in MuJoCo viewer.

Usage:
    python play.py                              # Interactive viewer
    python play.py --record output.mp4          # Record video
    python play.py --checkpoint checkpoints/cw1_ppo_100000_steps.zip
    python play.py --episodes 5                 # Run 5 episodes
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()


def find_best_model() -> Path | None:
    """Find the best available model file."""
    candidates = [
        SCRIPT_DIR / "cw1_policy_final.zip",
    ]

    # Also check checkpoints for latest
    checkpoint_dir = SCRIPT_DIR / "checkpoints"
    if checkpoint_dir.exists():
        checkpoints = sorted(
            checkpoint_dir.glob("cw1_ppo_*_steps.zip"),
            key=lambda p: int(p.stem.split("_")[-2]),
        )
        if checkpoints:
            candidates.insert(0, checkpoints[-1])

    for path in candidates:
        if path.exists():
            return path
    return None


def main():
    parser = argparse.ArgumentParser(description="Visualize CW-1 locomotion policy")
    parser.add_argument(
        "--checkpoint", type=Path, default=None,
        help="Path to trained model (.zip)"
    )
    parser.add_argument(
        "--record", type=str, default=None,
        help="Record video to file (e.g., output.mp4)"
    )
    parser.add_argument(
        "--episodes", type=int, default=3,
        help="Number of episodes to run"
    )
    parser.add_argument(
        "--deterministic", action="store_true", default=True,
        help="Use deterministic actions (default: True)"
    )
    parser.add_argument(
        "--cmd-vx", type=float, default=0.8,
        help="Forward velocity command (m/s)"
    )
    args = parser.parse_args()

    # Find model
    model_path = args.checkpoint or find_best_model()
    if model_path is None or not model_path.exists():
        print("No trained model found.")
        print("Run: python train.py")
        print("Or specify: python play.py --checkpoint path/to/model.zip")
        sys.exit(1)

    print(f"Loading model: {model_path}")

    # Import after path setup
    sys.path.insert(0, str(SCRIPT_DIR))
    import mujoco
    import mujoco.viewer
    from stable_baselines3 import PPO

    from cw1_env import CW1LocomotionEnv

    model = PPO.load(str(model_path))
    print(f"Model loaded ({sum(p.numel() for p in model.policy.parameters()):,} params)")

    if args.record:
        _run_recorded(model, args)
    else:
        _run_interactive(model, args)


def _run_interactive(model, args):
    """Run with interactive MuJoCo viewer."""
    import mujoco
    import mujoco.viewer

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    env = CW1LocomotionEnv()
    print(f"\nRunning {args.episodes} episodes in interactive viewer...")
    print("Controls: Mouse drag to rotate, scroll to zoom, Space to pause")
    print(f"Velocity command: vx={args.cmd_vx} m/s")

    for ep in range(args.episodes):
        obs, info = env.reset()
        # Override velocity command
        env._velocity_cmd = np.array([args.cmd_vx, 0.0, 0.0], dtype=np.float32)

        total_reward = 0.0
        step = 0

        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            while viewer.is_running():
                action, _ = model.predict(obs, deterministic=args.deterministic)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                step += 1

                viewer.sync()
                time.sleep(env.control_dt)  # Real-time playback

                if terminated or truncated:
                    break

        print(f"  Episode {ep + 1}: {step} steps, reward={total_reward:.2f}, "
              f"{'terminated' if terminated else 'truncated'}")

    env.close()
    print("\nDone!")


def _run_recorded(model, args):
    """Run and record video."""
    import imageio

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    env = CW1LocomotionEnv(render_mode="rgb_array")
    frames = []

    print(f"\nRecording {args.episodes} episodes to: {args.record}")
    print(f"Velocity command: vx={args.cmd_vx} m/s")

    for ep in range(args.episodes):
        obs, info = env.reset()
        env._velocity_cmd = np.array([args.cmd_vx, 0.0, 0.0], dtype=np.float32)

        total_reward = 0.0
        step = 0

        while True:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step += 1

            frame = env.render()
            if frame is not None:
                frames.append(frame)

            if terminated or truncated:
                break

        print(f"  Episode {ep + 1}: {step} steps, reward={total_reward:.2f}")

    env.close()

    # Save video
    output_path = Path(args.record)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"\nSaving {len(frames)} frames to {output_path}...")
    imageio.mimwrite(str(output_path), frames, fps=50, quality=8)
    print(f"Video saved: {output_path}")


if __name__ == "__main__":
    main()
