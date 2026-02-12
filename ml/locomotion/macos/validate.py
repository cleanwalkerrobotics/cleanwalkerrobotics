#!/usr/bin/env python3
"""Validate trained CW-1 locomotion policy against a random baseline.

Runs 100 episodes with the trained policy and 100 with random actions,
collecting metrics for comparison: success rate, distance traveled,
average reward, and episode length.

Usage:
    python validate.py                          # Auto-find latest model
    python validate.py --checkpoint path/to.zip # Specific checkpoint
    python validate.py --episodes 200           # More episodes
    python validate.py --output results.json    # Custom output path
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()


def find_latest_model() -> Path | None:
    """Find the best available model: final > latest checkpoint."""
    final = SCRIPT_DIR / "cw1_policy_final.zip"
    if final.exists():
        return final

    checkpoint_dir = SCRIPT_DIR / "checkpoints"
    if checkpoint_dir.exists():
        checkpoints = sorted(
            checkpoint_dir.glob("cw1_ppo_*_steps.zip"),
            key=lambda p: int(p.stem.split("_")[-2]),
        )
        if checkpoints:
            return checkpoints[-1]

    return None


def run_episodes(env, predict_fn, n_episodes: int, label: str) -> dict:
    """Run n episodes and collect metrics.

    Args:
        env: Gymnasium environment.
        predict_fn: Callable(obs) -> action.
        n_episodes: Number of episodes to run.
        label: Label for progress display.

    Returns:
        Dict with collected metrics.
    """
    rewards = []
    distances = []
    episode_lengths = []
    successes = []  # survived full episode = success

    for ep in range(n_episodes):
        obs, info = env.reset()
        ep_reward = 0.0
        start_x = env.data.xpos[env.body_id][0]

        for step in range(env.max_episode_steps):
            action = predict_fn(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward

            if terminated or truncated:
                break

        end_x = env.data.xpos[env.body_id][0]
        distance = float(end_x - start_x)
        length = step + 1
        survived = not terminated  # truncated (time limit) = survived

        rewards.append(ep_reward)
        distances.append(distance)
        episode_lengths.append(length)
        successes.append(survived)

        if (ep + 1) % 25 == 0 or ep == 0:
            print(
                f"  [{label}] Episode {ep + 1:>3}/{n_episodes} | "
                f"reward: {ep_reward:>7.2f} | "
                f"dist: {distance:>5.2f}m | "
                f"steps: {length:>4} | "
                f"survived: {'✓' if survived else '✗'}"
            )

    return {
        "episodes": n_episodes,
        "success_rate": float(np.mean(successes)),
        "avg_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards)),
        "avg_distance": float(np.mean(distances)),
        "std_distance": float(np.std(distances)),
        "avg_episode_length": float(np.mean(episode_lengths)),
        "std_episode_length": float(np.std(episode_lengths)),
        "min_reward": float(np.min(rewards)),
        "max_reward": float(np.max(rewards)),
        "min_distance": float(np.min(distances)),
        "max_distance": float(np.max(distances)),
    }


def print_comparison(trained: dict, random: dict):
    """Print a formatted comparison table."""
    print("\n" + "=" * 64)
    print(f"{'VALIDATION RESULTS':^64}")
    print("=" * 64)
    print(f"{'Metric':<26} {'Trained':>16} {'Random':>16}")
    print("-" * 64)

    rows = [
        ("Success rate",     f"{trained['success_rate']:.1%}",         f"{random['success_rate']:.1%}"),
        ("Avg reward",       f"{trained['avg_reward']:.2f} ± {trained['std_reward']:.2f}",
                              f"{random['avg_reward']:.2f} ± {random['std_reward']:.2f}"),
        ("Avg distance (m)", f"{trained['avg_distance']:.2f} ± {trained['std_distance']:.2f}",
                              f"{random['avg_distance']:.2f} ± {random['std_distance']:.2f}"),
        ("Avg ep. length",   f"{trained['avg_episode_length']:.0f} ± {trained['std_episode_length']:.0f}",
                              f"{random['avg_episode_length']:.0f} ± {random['std_episode_length']:.0f}"),
        ("Reward range",     f"[{trained['min_reward']:.1f}, {trained['max_reward']:.1f}]",
                              f"[{random['min_reward']:.1f}, {random['max_reward']:.1f}]"),
        ("Distance range (m)", f"[{trained['min_distance']:.2f}, {trained['max_distance']:.2f}]",
                                f"[{random['min_distance']:.2f}, {random['max_distance']:.2f}]"),
    ]

    for label, t_val, r_val in rows:
        print(f"  {label:<24} {t_val:>16} {r_val:>16}")

    print("=" * 64)

    # Improvement summary
    if random["avg_reward"] != 0:
        reward_imp = (trained["avg_reward"] - random["avg_reward"]) / abs(random["avg_reward"]) * 100
    else:
        reward_imp = float("inf") if trained["avg_reward"] > 0 else 0
    dist_imp = trained["avg_distance"] - random["avg_distance"]
    surv_imp = trained["success_rate"] - random["success_rate"]

    print(f"\n  📊 Improvement over random baseline:")
    print(f"     Reward:   {'+' if reward_imp >= 0 else ''}{reward_imp:.1f}%")
    print(f"     Distance: {'+' if dist_imp >= 0 else ''}{dist_imp:.2f} m")
    print(f"     Survival: {'+' if surv_imp >= 0 else ''}{surv_imp:.1%}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Validate CW-1 trained policy vs random baseline"
    )
    parser.add_argument(
        "--checkpoint", type=Path, default=None,
        help="Path to trained model (.zip)"
    )
    parser.add_argument(
        "--episodes", type=int, default=100,
        help="Number of evaluation episodes per policy (default: 100)"
    )
    parser.add_argument(
        "--output", type=Path, default=SCRIPT_DIR / "validation_results.json",
        help="Output JSON path (default: validation_results.json)"
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Random seed for reproducibility"
    )
    args = parser.parse_args()

    # Find model
    model_path = args.checkpoint or find_latest_model()
    if model_path is None or not model_path.exists():
        print("❌ No trained model found. Run train.py first.")
        sys.exit(1)

    print("=" * 64)
    print("CW-1 Policy Validation")
    print("=" * 64)
    print(f"  Model:    {model_path}")
    print(f"  Episodes: {args.episodes} per policy")
    print(f"  Seed:     {args.seed}")
    print()

    # Import heavy deps after arg parse
    from stable_baselines3 import PPO

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    # Load model
    print("Loading trained model...")
    model = PPO.load(str(model_path), device="cpu")
    print(f"  Loaded: {model_path.name}")
    print(f"  Timesteps trained: {model.num_timesteps:,}")
    print()

    # Create env (no rendering needed)
    env = CW1LocomotionEnv()

    # Trained policy predict function
    def trained_predict(obs):
        action, _ = model.predict(obs, deterministic=True)
        return action

    # Random policy predict function
    def random_predict(obs):
        return env.action_space.sample()

    # Run trained policy
    print("▶ Running trained policy...")
    t0 = time.time()
    trained_results = run_episodes(env, trained_predict, args.episodes, "trained")
    trained_time = time.time() - t0
    print(f"  Completed in {trained_time:.1f}s")
    print()

    # Run random baseline
    print("▶ Running random baseline...")
    t0 = time.time()
    random_results = run_episodes(env, random_predict, args.episodes, "random")
    random_time = time.time() - t0
    print(f"  Completed in {random_time:.1f}s")

    env.close()

    # Print comparison
    print_comparison(trained_results, random_results)

    # Save results
    output = {
        "model_path": str(model_path),
        "model_name": model_path.name,
        "timesteps_trained": model.num_timesteps,
        "episodes_per_policy": args.episodes,
        "seed": args.seed,
        "trained": trained_results,
        "random_baseline": random_results,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(output, f, indent=2)

    print(f"✅ Results saved to: {args.output}")


if __name__ == "__main__":
    main()
