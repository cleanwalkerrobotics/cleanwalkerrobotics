#!/usr/bin/env python3
"""CW-1 Locomotion HPO v4 — Multi-Terrain Reward Optimization.

Designed for 12+ hour overnight runs. Each trial:
1. Loads the Stage 2 terrain checkpoint (good flat/rough/obstacles walker)
2. Fine-tunes 2M steps on ALL 5 terrain types
3. Evaluates across all 5 terrains
4. Objective: weighted multi-terrain score (speed * survival across terrains)

The goal: find reward weights that maintain walking speed while improving
terrain robustness (stairs/slopes currently 80/60% survival).

Budget: 12h at ~3900 steps/s ≈ 168M steps.
  - 2M per trial ≈ 8.5 min
  - ~84 completed trials (more with pruning)
  - 12 search parameters focused on terrain-relevant rewards

Usage:
    python hpo_v4.py                    # 12h run (~80 trials)
    python hpo_v4.py --n-trials 20      # Quick test
    python hpo_v4.py --resume           # Resume study
"""

import argparse
import gc
import os
import resource
import sys
import time
import warnings
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
warnings.filterwarnings("ignore", category=DeprecationWarning)

soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
resource.setrlimit(resource.RLIMIT_NOFILE, (min(8192, hard), hard))

SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_POLICY = SCRIPT_DIR / "cw1_policy_terrain_s2.zip"

# Training config
TOTAL_STEPS = 2_000_000
EVAL_FREQ = 500_000  # Eval at 500K, 1M, 1.5M, 2M
N_ENVS = 8

# Terrain types and evaluation weights
TERRAIN_TYPES = ["flat", "rough", "obstacles", "stairs_up", "slope_30"]
TERRAIN_WEIGHTS = {
    "flat": 0.20,       # baseline — must not regress
    "rough": 0.15,
    "obstacles": 0.20,
    "stairs_up": 0.25,  # highest weight — biggest room for improvement
    "slope_30": 0.20,
}


def evaluate_multi_terrain(model, env_kwargs, n_episodes=5, n_steps=500, cmd_vx=0.4):
    """Evaluate policy across all terrain types.

    Returns dict of per-terrain scores and a weighted composite.
    """
    sys.path.insert(0, str(SCRIPT_DIR))
    from cw1_env import CW1LocomotionEnv

    results = {}

    for terrain in TERRAIN_TYPES:
        eval_kwargs = dict(env_kwargs)
        eval_kwargs["randomize_friction"] = False
        eval_kwargs["randomize_mass"] = False
        eval_kwargs["terrain_types"] = [terrain]

        env = CW1LocomotionEnv(**eval_kwargs)
        speeds, survivals, tilts = [], [], []

        for ep in range(n_episodes):
            obs, _ = env.reset(seed=ep + 500)
            env._velocity_cmd = np.array([cmd_vx, 0.0, 0.0], dtype=np.float32)

            steps = 0
            for _ in range(n_steps):
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                steps += 1
                if terminated or truncated:
                    break

            body_pos = env.data.qpos[:3]
            speed = body_pos[0] / (steps * env.control_dt) if steps > 0 else 0.0
            survived = 1.0 if steps >= n_steps else 0.0

            quat = env.data.qpos[3:7]
            w, x, y, z = quat
            grav_proj_z = 1.0 - 2.0 * (x * x + y * y)
            tilt = np.degrees(np.arccos(np.clip(grav_proj_z, -1, 1)))

            speeds.append(speed)
            survivals.append(survived)
            tilts.append(tilt)

        env.close()

        avg_speed = float(np.mean(speeds))
        avg_survival = float(np.mean(survivals))
        avg_tilt = float(np.mean(tilts))

        # Per-terrain score: speed * survival_gate
        # survival_gate: quadratic penalty below 90%, capped at 1.0
        survival_gate = min(1.0, avg_survival / 0.90) ** 2
        # Speed score: linear, capped at 0.5 m/s (diminishing returns beyond)
        speed_score = min(1.0, max(0.0, avg_speed / 0.5))
        # Tilt penalty: reduce score if body tilt > 10°
        tilt_factor = max(0.0, 1.0 - max(0.0, avg_tilt - 10.0) / 30.0)

        terrain_score = speed_score * survival_gate * tilt_factor

        results[terrain] = {
            "speed": avg_speed,
            "survival": avg_survival,
            "tilt": avg_tilt,
            "score": terrain_score,
        }

    # Weighted composite
    composite = sum(
        results[t]["score"] * TERRAIN_WEIGHTS[t] for t in TERRAIN_TYPES
    )

    results["composite"] = composite
    return results


def objective(trial):
    """Single HPO trial: load Stage 2, fine-tune with sampled params, eval multi-terrain."""
    import optuna
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    trial_start = time.time()

    # ===== SAMPLE PARAMETERS =====
    # Focus on terrain-relevant rewards (12 params)

    # Speed vs. safety balance
    rew_track_lin_vel = trial.suggest_float("rew_track_lin_vel", 3.0, 8.0)
    rew_alive = trial.suggest_float("rew_alive", 0.1, 1.0)
    rew_orientation = trial.suggest_float("rew_orientation", -0.5, -0.05)
    fall_penalty = trial.suggest_float("fall_penalty", 1.0, 10.0)
    tracking_sigma = trial.suggest_float("tracking_sigma", 0.15, 0.40)

    # Terrain-relevant
    rew_base_height = trial.suggest_float("rew_base_height", 0.1, 2.0)
    rew_collision = trial.suggest_float("rew_collision", -3.0, -0.1)
    action_scale = trial.suggest_float("action_scale", 0.25, 0.50)

    # Drift + heading control
    rew_lat_vel = trial.suggest_float("rew_lat_vel", -10.0, -0.5)
    rew_track_ang_vel = trial.suggest_float("rew_track_ang_vel", 0.5, 4.0)

    # Gait quality
    rew_action_smooth_1 = trial.suggest_float("rew_action_smooth_1", -1.0, -0.05)
    rew_gait_clock = trial.suggest_float("rew_gait_clock", 0.3, 2.0)

    env_kwargs = dict(
        cmd_vx_range=(0.0, 0.7),
        cmd_vy_range=(-0.1, 0.1),
        cmd_yaw_range=(-0.3, 0.3),
        action_scale=action_scale,
        use_foot_contacts=True,
        only_positive_rewards=False,
        rew_track_lin_vel=rew_track_lin_vel,
        rew_track_ang_vel=rew_track_ang_vel,
        rew_alive=rew_alive,
        rew_orientation=rew_orientation,
        tracking_sigma=tracking_sigma,
        fall_penalty=fall_penalty,
        rew_gait_clock=rew_gait_clock,
        rew_action_smooth_1=rew_action_smooth_1,
        rew_base_height=rew_base_height,
        rew_collision=rew_collision,
        rew_lat_vel=rew_lat_vel,
        # Keep HPO v3 Trial #35 values for non-searched params
        rew_action_smooth_2=-0.083,
        rew_gait_force=2.835,
        rew_gait_vel=1.004,
        rew_clearance_target=-24.765,
        rew_foot_slip=-0.096,
        rew_air_time_var=-0.836,
        rew_hip_vel=-0.001,
        gait_period=0.458,
        swing_height_target=0.111,
        feet_air_time_threshold=0.186,
        # Train on all terrains
        terrain_types=TERRAIN_TYPES,
    )

    print(f"\n{'='*60}")
    print(f"Trial #{trial.number}")
    print(f"  track={rew_track_lin_vel:.2f}  alive={rew_alive:.2f}  orient={rew_orientation:.3f}")
    print(f"  fall={fall_penalty:.1f}  sigma={tracking_sigma:.3f}  act_scale={action_scale:.3f}")
    print(f"  base_h={rew_base_height:.2f}  collision={rew_collision:.2f}")
    print(f"  lat_vel={rew_lat_vel:.2f}  ang_vel={rew_track_ang_vel:.2f}")
    print(f"  smooth1={rew_action_smooth_1:.3f}  gait_clk={rew_gait_clock:.2f}")
    print(f"{'='*60}")

    # Create environments
    def make_env(rank=0):
        def _init():
            sys.path.insert(0, str(SCRIPT_DIR))
            from cw1_env import CW1LocomotionEnv
            env = CW1LocomotionEnv(**env_kwargs)
            env.reset(seed=rank)
            return env
        return _init

    try:
        env = SubprocVecEnv([make_env(rank=i) for i in range(N_ENVS)])
    except Exception as e:
        print(f"  ENV ERROR: {e}")
        gc.collect()
        return -1.0

    # Load from Stage 2 checkpoint and re-attach to new env
    try:
        model = PPO.load(str(BASE_POLICY), env=env, device="cpu")
    except Exception as e:
        print(f"  LOAD ERROR: {e}")
        env.close()
        gc.collect()
        return -1.0

    # Adaptive LR callback
    class AdaptiveLRCallback(BaseCallback):
        def __init__(self, kl_target_val=0.021):
            super().__init__()
            self.kl_target = kl_target_val

        def _on_step(self):
            return True

        def _on_rollout_end(self):
            if len(self.model.logger.name_to_value) > 0:
                approx_kl = self.model.logger.name_to_value.get("train/approx_kl")
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
                        from stable_baselines3.common.utils import get_schedule_fn
                        self.model.learning_rate = new_lr
                        self.model.lr_schedule = get_schedule_fn(new_lr)

    # Eval + pruning callback
    class OptunaEvalCallback(BaseCallback):
        def __init__(self, trial_obj, eval_env_kwargs, eval_freq=EVAL_FREQ):
            super().__init__()
            self.trial = trial_obj
            self.eval_env_kwargs = eval_env_kwargs
            self.eval_freq = eval_freq
            self.last_eval_step = 0
            self.best_score = -1.0
            self.best_results = None
            self.last_score = 0.0

        def _on_step(self):
            if self.num_timesteps - self.last_eval_step >= self.eval_freq:
                self.last_eval_step = self.num_timesteps
                step_k = self.num_timesteps // 1000

                results = evaluate_multi_terrain(
                    self.model, self.eval_env_kwargs,
                    n_episodes=5, n_steps=500, cmd_vx=0.4,
                )
                score = results["composite"]

                self.last_score = score
                if score > self.best_score:
                    self.best_score = score
                    self.best_results = results

                step_idx = self.num_timesteps // self.eval_freq
                self.trial.report(score, step_idx)

                elapsed = time.time() - trial_start
                parts = []
                for t in TERRAIN_TYPES:
                    r = results[t]
                    parts.append(f"{t[:5]}={r['speed']:.2f}/{r['survival']*100:.0f}%")
                terrain_str = " | ".join(parts)
                print(
                    f"  [{step_k}K {elapsed:.0f}s] {terrain_str}"
                    f" | composite={score:.4f}"
                )

                # Triage at 500K: flat survival must be 100%, composite > 0.1
                if self.num_timesteps >= 500_000 and self.num_timesteps < 750_000:
                    flat_surv = results["flat"]["survival"]
                    if flat_surv < 0.80 or score < 0.10:
                        print(f"  TRIAGE: flat_surv={flat_surv:.0%} composite={score:.3f} → pruned")
                        raise optuna.TrialPruned()

                if self.trial.should_prune():
                    print(f"  PRUNED at {step_k}K (below median)")
                    raise optuna.TrialPruned()

            return True

    # Train
    eval_cb = OptunaEvalCallback(trial, env_kwargs, eval_freq=EVAL_FREQ)
    adaptive_lr = AdaptiveLRCallback(kl_target_val=0.021)

    try:
        model.learn(
            total_timesteps=TOTAL_STEPS,
            callback=[eval_cb, adaptive_lr],
            progress_bar=False,
            reset_num_timesteps=True,
        )
    except optuna.TrialPruned:
        env.close()
        del model, env
        gc.collect()
        raise
    except Exception as e:
        print(f"  ERROR: {e}")
        env.close()
        del model, env
        gc.collect()
        return -1.0

    env.close()
    del model, env
    gc.collect()

    elapsed = time.time() - trial_start

    # Stability-aware scoring
    best_s = max(eval_cb.best_score, 1e-8)
    final_s = max(eval_cb.last_score, 0.0)
    stability_ratio = min(final_s / best_s, 1.0)
    stable_score = best_s * (stability_ratio ** 0.5)

    if eval_cb.best_results:
        r = eval_cb.best_results
        print(
            f"  DONE ({elapsed:.0f}s): "
            f"best={eval_cb.best_score:.4f} final={final_s:.4f} "
            f"stable={stable_score:.4f}"
        )
        for t in TERRAIN_TYPES:
            tr = r[t]
            print(f"    {t:12s}: speed={tr['speed']:.3f} surv={tr['survival']*100:.0f}% tilt={tr['tilt']:.1f}°")

    return stable_score


def main():
    import optuna

    parser = argparse.ArgumentParser(description="HPO v4 — Multi-Terrain Optimization")
    parser.add_argument("--n-trials", type=int, default=100)
    parser.add_argument("--study-name", type=str, default="cw1_terrain_v4")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--db", type=str, default=str(SCRIPT_DIR / "hpo_v4_study.db"))
    args = parser.parse_args()

    if not BASE_POLICY.exists():
        print(f"ERROR: Base policy not found: {BASE_POLICY}")
        print("Run terrain Stages 1-2 first to generate cw1_policy_terrain_s2.zip")
        sys.exit(1)

    print("=" * 60)
    print("CW-1 Locomotion HPO v4 — Multi-Terrain Optimization")
    print("=" * 60)
    print(f"Base policy: {BASE_POLICY}")
    print(f"Trials: {args.n_trials}")
    print(f"Steps/trial: {TOTAL_STEPS:,}")
    print(f"Envs: {N_ENVS}")
    print(f"Terrains: {TERRAIN_TYPES}")
    print(f"Search params: 12 (terrain-relevant rewards)")
    print(f"Objective: weighted multi-terrain (speed * survival * tilt)")
    est_min = args.n_trials * 10  # ~10 min per trial with eval overhead
    print(f"Estimated time: ~{est_min:.0f} min ({est_min / 60:.1f} hours)")
    print()

    storage = f"sqlite:///{args.db}"

    sampler = optuna.samplers.TPESampler(
        n_startup_trials=15,
        multivariate=True,
    )

    pruner = optuna.pruners.MedianPruner(
        n_startup_trials=10,
        n_warmup_steps=1,
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
        completed = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
        pruned = [t for t in study.trials if t.state == optuna.trial.TrialState.PRUNED]
        print(f"  Completed: {len(completed)}, Pruned: {len(pruned)}")
        if completed:
            try:
                print(f"  Best so far: score={study.best_trial.value:.4f}")
            except ValueError:
                pass
    else:
        # Seed 1: Current Run 16d defaults (proven good on flat)
        study.enqueue_trial({
            "rew_track_lin_vel": 5.587,
            "rew_alive": 0.366,
            "rew_orientation": -0.208,
            "fall_penalty": 3.502,
            "tracking_sigma": 0.271,
            "rew_base_height": 0.5,
            "rew_collision": -1.0,
            "action_scale": 0.330,
            "rew_lat_vel": -5.0,
            "rew_track_ang_vel": 2.0,
            "rew_action_smooth_1": -0.454,
            "rew_gait_clock": 1.018,
        })
        # Seed 2: Higher speed + stronger survival for terrain
        study.enqueue_trial({
            "rew_track_lin_vel": 7.0,
            "rew_alive": 0.8,
            "rew_orientation": -0.3,
            "fall_penalty": 8.0,
            "tracking_sigma": 0.25,
            "rew_base_height": 1.5,
            "rew_collision": -2.0,
            "action_scale": 0.40,
            "rew_lat_vel": -3.0,
            "rew_track_ang_vel": 1.5,
            "rew_action_smooth_1": -0.3,
            "rew_gait_clock": 1.5,
        })
        # Seed 3: Conservative (strong balance, moderate speed)
        study.enqueue_trial({
            "rew_track_lin_vel": 4.0,
            "rew_alive": 0.6,
            "rew_orientation": -0.4,
            "fall_penalty": 6.0,
            "tracking_sigma": 0.30,
            "rew_base_height": 1.0,
            "rew_collision": -1.5,
            "action_scale": 0.35,
            "rew_lat_vel": -7.0,
            "rew_track_ang_vel": 3.0,
            "rew_action_smooth_1": -0.6,
            "rew_gait_clock": 0.8,
        })
        print("Seeded 3 trials: current-defaults, high-speed-terrain, conservative")

    print(f"\nDatabase: {args.db}")
    print("-" * 60)

    study.optimize(
        objective,
        n_trials=args.n_trials,
        show_progress_bar=True,
    )

    # Results
    print("\n" + "=" * 60)
    print("OPTIMIZATION COMPLETE")
    print("=" * 60)

    completed = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
    pruned = [t for t in study.trials if t.state == optuna.trial.TrialState.PRUNED]

    print(f"\nTrials: {len(study.trials)} total")
    print(f"  Completed: {len(completed)}")
    print(f"  Pruned: {len(pruned)}")

    if not completed:
        print("\nNo completed trials.")
        return

    best = study.best_trial
    print(f"\nBest trial #{best.number}:")
    print(f"  Score: {best.value:.4f}")
    print(f"  Params:")
    for key, value in best.params.items():
        print(f"    {key}: {value:.4f}" if isinstance(value, float) else f"    {key}: {value}")

    try:
        importances = optuna.importance.get_param_importances(study)
        print(f"\nParameter importance:")
        for key, value in sorted(importances.items(), key=lambda x: -x[1]):
            bar = "#" * int(value * 40)
            print(f"  {key:>25s}: {value:.3f} {bar}")
    except Exception:
        pass

    top = sorted(completed, key=lambda t: t.value, reverse=True)[:5]
    print(f"\nTop 5 trials:")
    for t in top:
        p = t.params
        print(
            f"  #{t.number}: score={t.value:.4f}"
            f"  track={p.get('rew_track_lin_vel', '?'):.1f}"
            f"  alive={p.get('rew_alive', '?'):.2f}"
            f"  fall={p.get('fall_penalty', '?'):.1f}"
            f"  act_scale={p.get('action_scale', '?'):.3f}"
        )


if __name__ == "__main__":
    main()
