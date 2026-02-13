#!/usr/bin/env python3
"""Optuna hyperparameter optimization for CW-1 locomotion policy.

Uses 800K-step quick-eval cycles with 500K triage gate.
Optimizes reward weights for PPO training on macOS Apple Silicon.

Protocol:
    - 800K training steps per trial (~3.7 min on M4 Max)
    - Evaluate every 200K steps (10 episodes, deterministic)
    - 500K triage gate: kill if survival <90% or tilt >6°
    - MedianPruner kills underperforming trials
    - Objective: speed * min(1, survival/95)^2

Usage:
    python hpo.py                           # Run 50 trials (~4 hours)
    python hpo.py --n-trials 100            # Run 100 trials
    python hpo.py --resume                  # Resume previous study

Monitor:
    optuna-dashboard sqlite:///hpo_study.db
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

SCRIPT_DIR = Path(__file__).parent.resolve()


def evaluate_policy(model, env_kwargs, n_episodes=10, n_steps=500, cmd_vx=0.4):
    """Evaluate a trained policy and return speed/survival/tilt metrics."""
    import mujoco

    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    eval_kwargs = dict(env_kwargs)
    eval_kwargs["randomize_friction"] = False
    eval_kwargs["randomize_mass"] = False

    env = CW1LocomotionEnv(**eval_kwargs)

    speeds = []
    survivals = []
    tilts = []

    for ep in range(n_episodes):
        obs, info = env.reset()
        env._velocity_cmd = np.array([cmd_vx, 0.0, 0.0], dtype=np.float32)

        start_x = env.data.xpos[env.body_id][0]
        ep_tilts = []
        survived_steps = 0

        for step in range(n_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            body_quat = env.data.xquat[env.body_id]
            body_rot = np.zeros(9)
            mujoco.mju_quat2Mat(body_rot, body_quat)
            body_rot = body_rot.reshape(3, 3)
            up = body_rot.T @ np.array([0, 0, 1.0])
            tilt = np.degrees(np.arccos(np.clip(up[2], -1, 1)))
            ep_tilts.append(tilt)
            survived_steps = step + 1

            if terminated or truncated:
                break

        end_x = env.data.xpos[env.body_id][0]
        distance = end_x - start_x
        duration = survived_steps * 0.02

        speeds.append(distance / duration if duration > 0 else 0)
        survivals.append(survived_steps / n_steps * 100)
        tilts.append(np.mean(ep_tilts) if ep_tilts else 90.0)

    env.close()

    return {
        "avg_speed": float(np.mean(speeds)),
        "avg_survival": float(np.mean(survivals)),
        "avg_tilt": float(np.mean(tilts)),
    }


def compute_objective(metrics):
    """Score = speed * min(1, survival/95)^2.

    Heavily penalizes survival below 95% while maximizing speed.
    """
    speed = metrics["avg_speed"]
    survival = metrics["avg_survival"]
    survival_factor = min(1.0, survival / 95.0) ** 2
    return speed * survival_factor


def objective(trial):
    """Single HPO trial: train PPO with suggested params, evaluate, return score."""
    import optuna
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    trial_start = time.time()

    # --- Sample Tier 1: Reward weights ---
    rew_track_lin_vel = trial.suggest_float("rew_track_lin_vel", 1.0, 6.0)
    rew_alive = trial.suggest_float("rew_alive", 0.1, 1.0)
    rew_orientation = trial.suggest_float("rew_orientation", -2.0, -0.1)
    tracking_sigma = trial.suggest_float("tracking_sigma", 0.05, 0.25)
    fall_penalty = trial.suggest_float("fall_penalty", 1.0, 10.0)
    rew_gait_clock = trial.suggest_float("rew_gait_clock", 0.1, 1.0)

    env_kwargs = dict(
        cmd_vx_range=(0.0, 0.7),
        cmd_vy_range=(-0.1, 0.1),
        cmd_yaw_range=(-0.3, 0.3),
        rew_track_lin_vel=rew_track_lin_vel,
        rew_alive=rew_alive,
        rew_orientation=rew_orientation,
        tracking_sigma=tracking_sigma,
        fall_penalty=fall_penalty,
        rew_gait_clock=rew_gait_clock,
    )

    # --- Create parallel environments ---
    def make_env(rank=0):
        def _init():
            sys.path.insert(0, str(SCRIPT_DIR))
            from cw1_env import CW1LocomotionEnv
            env = CW1LocomotionEnv(**env_kwargs)
            env.reset(seed=rank)
            return env
        return _init

    n_envs = 8
    env = SubprocVecEnv([make_env(rank=i) for i in range(n_envs)])

    # --- Create PPO model (fixed algo params from Run 10) ---
    policy_kwargs = dict(
        net_arch=dict(pi=[512, 256, 128], vf=[512, 256, 128]),
        activation_fn=torch.nn.ELU,
    )

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=5,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=1.0,
        max_grad_norm=1.0,
        policy_kwargs=policy_kwargs,
        verbose=0,
        device="cpu",
    )

    # --- Adaptive LR callback ---
    class AdaptiveLRCallback(BaseCallback):
        def __init__(self, kl_target=0.02):
            super().__init__()
            self.kl_target = kl_target

        def _on_step(self):
            return True

        def _on_rollout_end(self):
            if len(self.model.logger.name_to_value) > 0:
                approx_kl = self.model.logger.name_to_value.get(
                    "train/approx_kl"
                )
                if approx_kl is not None:
                    current_lr = self.model.learning_rate
                    if callable(current_lr):
                        current_lr = current_lr(1.0)
                    new_lr = current_lr
                    if approx_kl > 2.0 * self.kl_target:
                        new_lr = max(current_lr * 0.5, 1e-5)
                    elif approx_kl < self.kl_target / 2.0:
                        new_lr = min(current_lr * 2.0, 1e-2)
                    if new_lr != current_lr:
                        import warnings
                        with warnings.catch_warnings():
                            warnings.simplefilter("ignore", DeprecationWarning)
                            from stable_baselines3.common.utils import (
                                get_schedule_fn,
                            )
                            self.model.learning_rate = new_lr
                            self.model.lr_schedule = get_schedule_fn(new_lr)

    # --- Eval + pruning callback ---
    class OptunaEvalCallback(BaseCallback):
        def __init__(self, trial, env_kwargs, eval_freq=200_000):
            super().__init__()
            self.trial = trial
            self.env_kwargs = env_kwargs
            self.eval_freq = eval_freq
            self.last_eval_step = 0
            self.best_score = -1.0
            self.best_metrics = None

        def _on_step(self):
            if self.num_timesteps - self.last_eval_step >= self.eval_freq:
                self.last_eval_step = self.num_timesteps
                step_k = self.num_timesteps // 1000

                metrics = evaluate_policy(
                    self.model, self.env_kwargs,
                    n_episodes=10, n_steps=500, cmd_vx=0.4,
                )
                score = compute_objective(metrics)

                if score > self.best_score:
                    self.best_score = score
                    self.best_metrics = metrics

                step_idx = self.num_timesteps // self.eval_freq
                self.trial.report(score, step_idx)

                elapsed = time.time() - trial_start
                print(
                    f"  [{step_k}K {elapsed:.0f}s] "
                    f"speed={metrics['avg_speed']:.3f} "
                    f"surv={metrics['avg_survival']:.0f}% "
                    f"tilt={metrics['avg_tilt']:.1f}° "
                    f"score={score:.4f}"
                )

                # 500K triage gate
                if self.num_timesteps >= 400_000:
                    surv = metrics["avg_survival"]
                    tlt = metrics["avg_tilt"]
                    if surv < 90 or tlt > 6:
                        print(
                            f"  TRIAGE: surv={surv:.0f}% tilt={tlt:.1f}° → pruned"
                        )
                        raise optuna.TrialPruned()

                # Optuna MedianPruner
                if self.trial.should_prune():
                    print(f"  PRUNED at {step_k}K (below median)")
                    raise optuna.TrialPruned()

            return True

    # --- Train ---
    eval_cb = OptunaEvalCallback(trial, env_kwargs, eval_freq=200_000)
    adaptive_lr = AdaptiveLRCallback(kl_target=0.02)

    try:
        model.learn(
            total_timesteps=800_000,
            callback=[eval_cb, adaptive_lr],
            progress_bar=False,
        )
    except optuna.TrialPruned:
        env.close()
        raise
    except Exception as e:
        print(f"  ERROR: {e}")
        env.close()
        return -1.0

    env.close()

    elapsed = time.time() - trial_start
    if eval_cb.best_metrics:
        m = eval_cb.best_metrics
        print(
            f"  DONE ({elapsed:.0f}s): speed={m['avg_speed']:.3f} "
            f"surv={m['avg_survival']:.0f}% "
            f"tilt={m['avg_tilt']:.1f}° "
            f"score={eval_cb.best_score:.4f}"
        )
    return eval_cb.best_score


def main():
    import optuna

    parser = argparse.ArgumentParser(description="HPO for CW-1 locomotion")
    parser.add_argument("--n-trials", type=int, default=50)
    parser.add_argument("--study-name", type=str, default="cw1_locomotion_v1")
    parser.add_argument(
        "--resume", action="store_true", help="Resume a previous study"
    )
    parser.add_argument(
        "--db", type=str, default=str(SCRIPT_DIR / "hpo_study.db"),
    )
    args = parser.parse_args()

    print("=" * 60)
    print("CW-1 Locomotion HPO — Optuna + 800K Quick Eval")
    print("=" * 60)
    print(f"Trials: {args.n_trials}")
    print(f"Study: {args.study_name}")
    print(f"Database: {args.db}")
    print(f"Protocol: 800K steps, eval@200K/400K/600K/800K, 500K triage")
    est_min = args.n_trials * 2.5  # ~2.5 min avg with pruning
    print(f"Estimated time: ~{est_min:.0f} min ({est_min / 60:.1f} hours)")
    print()

    storage = f"sqlite:///{args.db}"

    sampler = optuna.samplers.TPESampler(
        n_startup_trials=10,
        multivariate=True,
    )

    pruner = optuna.pruners.MedianPruner(
        n_startup_trials=5,
        n_warmup_steps=1,  # Don't prune at first eval (200K)
    )

    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        sampler=sampler,
        pruner=pruner,
        direction="maximize",
        load_if_exists=True,
    )

    existing = len(study.trials)
    if existing > 0:
        print(f"Resuming: {existing} previous trials")
        completed = len([
            t for t in study.trials
            if t.state == optuna.trial.TrialState.COMPLETE
        ])
        pruned = len([
            t for t in study.trials
            if t.state == optuna.trial.TrialState.PRUNED
        ])
        print(f"  Completed: {completed}, Pruned: {pruned}")
        if completed > 0:
            try:
                print(f"  Best so far: score={study.best_trial.value:.4f}")
            except ValueError:
                pass

    print(f"\nMonitor: optuna-dashboard {storage}")
    print("-" * 60)

    study.optimize(
        objective,
        n_trials=args.n_trials,
        show_progress_bar=True,
    )

    # --- Results ---
    print("\n" + "=" * 60)
    print("OPTIMIZATION COMPLETE")
    print("=" * 60)

    completed = [
        t for t in study.trials
        if t.state == optuna.trial.TrialState.COMPLETE
    ]
    pruned_trials = [
        t for t in study.trials
        if t.state == optuna.trial.TrialState.PRUNED
    ]

    print(f"\nTrials: {len(study.trials)} total")
    print(f"  Completed: {len(completed)}")
    print(f"  Pruned: {len(pruned_trials)}")

    if not completed:
        print("\nNo completed trials. All pruned.")
        return

    best = study.best_trial
    print(f"\nBest trial #{best.number}:")
    print(f"  Score: {best.value:.4f}")
    print(f"  Params:")
    for key, value in best.params.items():
        print(f"    {key}: {value:.4f}")

    # Parameter importance
    try:
        importances = optuna.importance.get_param_importances(study)
        print(f"\nParameter importance:")
        for key, value in sorted(
            importances.items(), key=lambda x: -x[1]
        ):
            bar = "#" * int(value * 40)
            print(f"  {key:>20s}: {value:.3f} {bar}")
    except Exception:
        pass

    # Top 5 trials
    top = sorted(completed, key=lambda t: t.value, reverse=True)[:5]
    print(f"\nTop 5 trials:")
    for t in top:
        print(f"  #{t.number}: score={t.value:.4f}", end="")
        for k in ["rew_track_lin_vel", "rew_alive", "rew_orientation"]:
            if k in t.params:
                print(f"  {k.split('_')[-1]}={t.params[k]:.2f}", end="")
        print()

    # Retrain command
    print(f"\nRetrain best config for 1M steps:")
    p = best.params
    print(f"  .venv/bin/python train.py --timesteps 1000000 \\")
    print(f"    --ent-coef 0.0 --adaptive-lr --n-envs 8 --terrain flat")
    print(f"  # Set in cw1_env.py constructor defaults or pass as kwargs:")
    for key, value in p.items():
        print(f"  #   {key}={value:.4f}")


if __name__ == "__main__":
    main()
