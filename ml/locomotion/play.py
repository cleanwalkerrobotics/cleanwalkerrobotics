# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
CleanWalker CW-1 — Play (visualize) a trained policy.

Loads a trained PPO checkpoint and runs the robot in simulation with
rendering enabled so you can see it walk.

Usage:
    # Visualize flat terrain policy
    python ml/locomotion/play.py --task CleanWalker-CW1-Flat-v0 \\
        --load_run <run_name> --checkpoint model_1500.pt

    # Visualize rough terrain policy
    python ml/locomotion/play.py --task CleanWalker-CW1-Rough-v0 \\
        --load_run <run_name> --checkpoint model_3000.pt

    # Visualize litter collection policy
    python ml/locomotion/play.py --task CleanWalker-CW1-Litter-v0 \\
        --load_run <run_name> --checkpoint model_2000.pt

    # Fewer envs for cleaner visualization
    python ml/locomotion/play.py --task CleanWalker-CW1-Flat-v0 \\
        --load_run <run_name> --checkpoint model_1500.pt --num_envs 16

Via IsaacLab:
    ./isaaclab.sh -p ml/locomotion/play.py --task CleanWalker-CW1-Flat-v0 \\
        --load_run <run_name> --checkpoint model_1500.pt
"""

from __future__ import annotations

import argparse
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ML_DIR = os.path.dirname(_SCRIPT_DIR)
if _ML_DIR not in sys.path:
    sys.path.insert(0, _ML_DIR)


def main():
    parser = argparse.ArgumentParser(description="Visualize trained CleanWalker policy")
    parser.add_argument(
        "--task",
        type=str,
        default="CleanWalker-CW1-Flat-v0",
        help="Task/environment",
    )
    parser.add_argument("--load_run", type=str, required=True, help="Run name to load")
    parser.add_argument("--checkpoint", type=str, required=True, help="Checkpoint file (e.g. model_1500.pt)")
    parser.add_argument("--num_envs", type=int, default=16, help="Number of envs to visualize (default: 16)")
    parser.add_argument("--num_steps", type=int, default=2000, help="Steps to run (default: 2000 = 40s)")
    parser.add_argument("--device", type=str, default="cuda", help="Compute device")
    args = parser.parse_args()

    # --- Launch Isaac Sim with rendering ---
    from isaaclab.app import AppLauncher

    launcher_args = argparse.Namespace(
        headless=False,
        enable_cameras=True,
    )
    app_launcher = AppLauncher(launcher_args)
    simulation_app = app_launcher.app

    # --- Imports ---
    import gymnasium as gym
    import torch

    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    import locomotion  # noqa: F401

    # --- Create environment ---
    env = gym.make(args.task, **{"scene.num_envs": args.num_envs})
    env = RslRlVecEnvWrapper(env)

    # --- Find checkpoint ---
    repo_root = os.path.dirname(_ML_DIR)
    log_root = os.path.join(repo_root, "logs", "rsl_rl")

    resume_path = None
    for exp_dir in os.listdir(log_root) if os.path.isdir(log_root) else []:
        exp_path = os.path.join(log_root, exp_dir)
        if os.path.isdir(exp_path):
            for run_dir in sorted(os.listdir(exp_path)):
                if args.load_run in run_dir:
                    candidate = os.path.join(exp_path, run_dir, args.checkpoint)
                    if os.path.isfile(candidate):
                        resume_path = candidate
                        break

    if resume_path is None:
        print(f"ERROR: Checkpoint not found: run={args.load_run}, file={args.checkpoint}")
        print(f"Searched in: {log_root}")
        env.close()
        simulation_app.close()
        sys.exit(1)

    print(f"[play] Loading checkpoint: {resume_path}")

    # --- Load policy ---
    runner_cfg: RslRlOnPolicyRunnerCfg = env.unwrapped.cfg.runner_cfg
    runner_cfg.device = args.device

    runner = OnPolicyRunner(env, runner_cfg, log_dir=None, device=runner_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=runner_cfg.device)

    # --- Run simulation ---
    print(f"[play] Running {args.num_steps} steps ({args.num_steps * 0.02:.0f}s sim time)...")
    print("[play] Close the Isaac Sim window or press Ctrl+C to stop.")

    obs, _ = env.get_observations()

    for step in range(args.num_steps):
        with torch.no_grad():
            actions = policy(obs)
        obs, _, dones, _, _ = env.step(actions)

        if step % 500 == 0:
            print(f"[play] Step {step}/{args.num_steps}")

    print("[play] Done.")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
