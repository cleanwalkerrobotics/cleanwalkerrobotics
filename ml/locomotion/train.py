# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
CleanWalker CW-1 — Training entry point.

Trains locomotion or litter-collection policies using PPO (rsl_rl)
on NVIDIA IsaacLab.

Usage:
    # Phase 1: Flat terrain locomotion
    python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0

    # Phase 2: Rough terrain (resume from flat)
    python ml/locomotion/train.py --task CleanWalker-CW1-Rough-v0 \\
        --load_run <flat_run_name> --checkpoint model_1500.pt

    # Litter collection (resume from rough)
    python ml/locomotion/train.py --task CleanWalker-CW1-Litter-v0 \\
        --load_run <rough_run_name> --checkpoint model_3000.pt

    # Headless (no GUI — faster, use for cloud GPU)
    python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0 --headless

    # Custom env count (reduce for low-VRAM GPUs)
    python ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0 --num_envs 2048

Via IsaacLab:
    ./isaaclab.sh -p ml/locomotion/train.py --task CleanWalker-CW1-Flat-v0
"""

from __future__ import annotations

import argparse
import os
import sys

# Ensure the locomotion package is importable
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ML_DIR = os.path.dirname(_SCRIPT_DIR)
if _ML_DIR not in sys.path:
    sys.path.insert(0, _ML_DIR)


def main():
    parser = argparse.ArgumentParser(description="Train CleanWalker CW-1 locomotion policy")
    parser.add_argument(
        "--task",
        type=str,
        default="CleanWalker-CW1-Flat-v0",
        choices=[
            "CleanWalker-CW1-Flat-v0",
            "CleanWalker-CW1-Rough-v0",
            "CleanWalker-CW1-Litter-v0",
        ],
        help="Task/environment to train (default: CleanWalker-CW1-Flat-v0)",
    )
    parser.add_argument("--num_envs", type=int, default=None, help="Override number of parallel envs")
    parser.add_argument("--headless", action="store_true", help="Run without GUI (faster)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument("--max_iterations", type=int, default=None, help="Override max training iterations")
    parser.add_argument("--load_run", type=str, default=None, help="Run name to load checkpoint from")
    parser.add_argument("--checkpoint", type=str, default=None, help="Checkpoint file to load (e.g. model_1500.pt)")
    parser.add_argument("--log_dir", type=str, default=None, help="Override log directory")
    parser.add_argument("--device", type=str, default="cuda", choices=["cuda", "cpu"], help="Compute device")
    args = parser.parse_args()

    # --- Launch Isaac Sim ---
    from isaaclab.app import AppLauncher

    launcher_args = argparse.Namespace(
        headless=args.headless,
        enable_cameras=False,
    )
    app_launcher = AppLauncher(launcher_args)
    simulation_app = app_launcher.app

    # --- Imports after Isaac Sim is running ---
    import gymnasium as gym
    import torch
    from datetime import datetime

    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    # Register CleanWalker environments
    import locomotion  # noqa: F401 — triggers gym.register()

    # --- Resolve USD asset ---
    # If a converted USD exists in assets/, patch the env config to use it
    repo_root = os.path.dirname(_ML_DIR)
    usd_path = os.path.join(_SCRIPT_DIR, "assets", "cleanwalker_cw1.usd")
    use_usd = os.path.isfile(usd_path)

    if use_usd:
        print(f"[train] Using pre-converted USD asset: {usd_path}")
    else:
        print("[train] No USD asset found in assets/. Using URDF directly (slower).")
        print("[train] Run 'python convert_urdf.py' to convert for faster loading.")

    # --- Create environment ---
    env_cfg_kwargs = {}
    if args.num_envs is not None:
        env_cfg_kwargs["scene.num_envs"] = args.num_envs

    env = gym.make(args.task, **env_cfg_kwargs)

    # Wrap for rsl_rl
    env = RslRlVecEnvWrapper(env)

    # --- Load runner config ---
    runner_cfg: RslRlOnPolicyRunnerCfg = env.unwrapped.cfg.runner_cfg
    if args.seed is not None:
        runner_cfg.seed = args.seed
    if args.max_iterations is not None:
        runner_cfg.max_iterations = args.max_iterations
    if args.device:
        runner_cfg.device = args.device

    # --- Set up logging ---
    log_root = args.log_dir or os.path.join(repo_root, "logs", "rsl_rl")
    log_dir = os.path.join(log_root, runner_cfg.experiment_name)
    if runner_cfg.run_name:
        log_dir = os.path.join(log_dir, runner_cfg.run_name)
    else:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = os.path.join(log_dir, timestamp)

    os.makedirs(log_dir, exist_ok=True)
    print(f"[train] Logging to: {log_dir}")

    # --- Resume from checkpoint ---
    resume_path = None
    if args.load_run and args.checkpoint:
        # Search for the checkpoint in the log directory
        search_dir = os.path.join(log_root, runner_cfg.experiment_name)
        if os.path.isdir(search_dir):
            for run_dir in sorted(os.listdir(search_dir)):
                if args.load_run in run_dir:
                    candidate = os.path.join(search_dir, run_dir, args.checkpoint)
                    if os.path.isfile(candidate):
                        resume_path = candidate
                        break
        # Also search parent experiment dirs (for cross-phase loading)
        if resume_path is None:
            for exp_dir in os.listdir(log_root):
                exp_path = os.path.join(log_root, exp_dir)
                if os.path.isdir(exp_path):
                    for run_dir in sorted(os.listdir(exp_path)):
                        if args.load_run in run_dir:
                            candidate = os.path.join(exp_path, run_dir, args.checkpoint)
                            if os.path.isfile(candidate):
                                resume_path = candidate
                                break

        if resume_path:
            print(f"[train] Resuming from checkpoint: {resume_path}")
        else:
            print(f"[train] WARNING: Checkpoint not found for run={args.load_run}, file={args.checkpoint}")
            print("[train] Starting from scratch.")

    # --- Create PPO runner ---
    runner = OnPolicyRunner(env, runner_cfg, log_dir=log_dir, device=runner_cfg.device)

    if resume_path:
        runner.load(resume_path)
        print(f"[train] Loaded checkpoint: {resume_path}")

    # --- Train ---
    print(f"[train] Starting training: {args.task}")
    print(f"[train] Num envs: {env.num_envs}")
    print(f"[train] Max iterations: {runner_cfg.max_iterations}")
    print(f"[train] Device: {runner_cfg.device}")
    print(f"[train] Policy: {runner_cfg.policy.actor_hidden_dims}")

    runner.learn(num_learning_iterations=runner_cfg.max_iterations, init_at_random_ep_len=True)

    # --- Cleanup ---
    print(f"\n[train] Training complete. Logs saved to: {log_dir}")
    print(f"[train] Final checkpoint: {os.path.join(log_dir, f'model_{runner_cfg.max_iterations}.pt')}")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
