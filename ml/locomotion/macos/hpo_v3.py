#!/usr/bin/env python3
"""CW-1 Locomotion HPO v3 — Gait Quality Objective.

Key change from v2: the objective function now measures GAIT QUALITY,
not just speed*survival. This prevents the vibrating gait local optimum
where the policy achieves forward motion through high-frequency oscillation
instead of actual stepping.

New composite objective:
    speed^0.3 * survival^0.2 * step_quality^0.3 * smoothness^0.1 * energy^0.1

Gait quality metrics measured during evaluation:
    - action_smoothness: RMS of consecutive action differences (lower = better)
    - avg_foot_clearance: average foot height during swing phase (higher = better)
    - contact_regularity: variance of contact/air durations (lower = better)
    - cost_of_transport: energy per unit distance (lower = better)
    - avg_air_time: mean foot air time per step (should be ~gait_period/2)

Search space (20 params):
    Fixed: use_foot_contacts=True, n_envs=8 (proven by v2)
    Removed: only_positive_rewards (never helped in v2)
    Expanded: action_smooth_1 [-5.0, -0.1], action_smooth_2 [-3.0, 0.0]
    New: gait_force, gait_vel, foot_slip, air_time_var, gait_period,
         swing_height_target, feet_air_time_threshold

Usage:
    python hpo_v3.py                    # Run 200 trials
    python hpo_v3.py --n-trials 50      # Run 50 trials
    python hpo_v3.py --resume           # Resume previous study
"""

import argparse
import gc
import math
import os
import resource
import sys
import time
import warnings
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Fix macOS file descriptor limit (SubprocVecEnv leaks fds)
soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
resource.setrlimit(resource.RLIMIT_NOFILE, (min(8192, hard), hard))

SCRIPT_DIR = Path(__file__).parent.resolve()


def evaluate_policy_v3(model, env_kwargs, n_episodes=10, n_steps=500, cmd_vx=0.4):
    """Evaluate policy with gait quality metrics (not just speed/survival).

    Returns:
        dict with: avg_speed, avg_survival, avg_tilt,
                   action_smoothness, avg_foot_clearance, contact_regularity,
                   cost_of_transport, avg_step_count
    """
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
    action_smoothnesses = []
    foot_clearances = []
    contact_regularities = []
    costs_of_transport = []
    step_counts = []

    for ep in range(n_episodes):
        obs, info = env.reset()
        env._velocity_cmd = np.array([cmd_vx, 0.0, 0.0], dtype=np.float32)

        start_x = env.data.xpos[env.body_id][0]
        ep_tilts = []
        survived_steps = 0
        prev_action = np.zeros(env.n_act, dtype=np.float32)
        action_diffs = []
        ep_clearances = []
        ep_torque_sq_sum = 0.0
        contact_durations = []  # list of (foot_idx, duration) for contact periods
        air_durations = []  # list of (foot_idx, duration) for air periods
        foot_contact_state = np.zeros(4, dtype=bool)
        foot_state_duration = np.zeros(4, dtype=np.float32)
        foot_step_count = np.zeros(4, dtype=int)  # count contact→air transitions

        for step in range(n_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            # Body tilt
            body_quat = env.data.xquat[env.body_id]
            body_rot = np.zeros(9)
            mujoco.mju_quat2Mat(body_rot, body_quat)
            body_rot = body_rot.reshape(3, 3)
            up = body_rot.T @ np.array([0, 0, 1.0])
            tilt = np.degrees(np.arccos(np.clip(up[2], -1, 1)))
            ep_tilts.append(tilt)

            # Action smoothness (RMS of consecutive action diffs)
            action_diff = np.sqrt(np.mean((action - prev_action) ** 2))
            action_diffs.append(action_diff)
            prev_action = action.copy()

            # Foot clearance during swing
            for j in range(4):
                foot_phase = env._gait_phase + env._gait_phase_offsets[j]
                if math.sin(foot_phase) < 0:  # swing phase
                    foot_z = env.data.xpos[env.foot_body_ids[j]][2]
                    terrain_z = env._terrain_gen.get_height_at(
                        env.data.xpos[env.foot_body_ids[j]][0],
                        env.data.xpos[env.foot_body_ids[j]][1],
                    )
                    ep_clearances.append(foot_z - terrain_z)

            # Torques for cost of transport
            torques = env.data.actuator_force[env.leg_actuator_ids]
            joint_vel = env.data.qvel[env.leg_qvel_ids]
            ep_torque_sq_sum += np.sum(np.abs(torques * joint_vel)) * env.control_dt

            # Contact tracking for regularity
            in_contact = np.zeros(4, dtype=bool)
            for ci in range(env.data.ncon):
                contact = env.data.contact[ci]
                g1, g2 = contact.geom1, contact.geom2
                for j, foot_id in enumerate(env.foot_geom_ids):
                    if (g1 == foot_id and g2 == env.floor_geom_id) or \
                       (g2 == foot_id and g1 == env.floor_geom_id):
                        in_contact[j] = True

            for j in range(4):
                foot_state_duration[j] += env.control_dt
                if in_contact[j] != foot_contact_state[j]:
                    # State transition
                    if foot_contact_state[j]:  # was in contact, now in air
                        contact_durations.append(foot_state_duration[j])
                    else:  # was in air, now in contact
                        air_durations.append(foot_state_duration[j])
                        foot_step_count[j] += 1
                    foot_state_duration[j] = 0.0
                    foot_contact_state[j] = in_contact[j]

            survived_steps = step + 1
            if terminated or truncated:
                break

        # Compute episode metrics
        end_x = env.data.xpos[env.body_id][0]
        distance = end_x - start_x
        duration = survived_steps * 0.02

        speeds.append(distance / duration if duration > 0 else 0)
        survivals.append(survived_steps / n_steps * 100)
        tilts.append(np.mean(ep_tilts) if ep_tilts else 90.0)

        # Smoothness: mean RMS of action diffs (lower = smoother)
        action_smoothnesses.append(np.mean(action_diffs) if action_diffs else 1.0)

        # Foot clearance: mean clearance during swing (higher = better stepping)
        foot_clearances.append(np.mean(ep_clearances) if ep_clearances else 0.0)

        # Contact regularity: CV of contact+air durations (lower = more regular)
        all_durations = contact_durations + air_durations
        if len(all_durations) >= 4:
            dur_mean = np.mean(all_durations)
            dur_std = np.std(all_durations)
            contact_regularities.append(dur_std / max(dur_mean, 0.01))
        else:
            contact_regularities.append(2.0)  # bad score if no stepping

        # Cost of transport: mechanical energy / (mass * g * distance)
        mass = np.sum(env.model.body_mass)
        if distance > 0.1:
            cot = ep_torque_sq_sum / (mass * 9.81 * distance)
        else:
            cot = 10.0  # high CoT for no movement
        costs_of_transport.append(cot)

        # Step count: total foot-ground transitions / 4 (steps per leg)
        step_counts.append(np.mean(foot_step_count))

    env.close()

    return {
        "avg_speed": float(np.mean(speeds)),
        "avg_survival": float(np.mean(survivals)),
        "avg_tilt": float(np.mean(tilts)),
        "action_smoothness": float(np.mean(action_smoothnesses)),
        "avg_foot_clearance": float(np.mean(foot_clearances)),
        "contact_regularity": float(np.mean(contact_regularities)),
        "cost_of_transport": float(np.mean(costs_of_transport)),
        "avg_step_count": float(np.mean(step_counts)),
    }


def compute_objective_v3(metrics):
    """Composite gait quality objective.

    Components (all normalized to 0-1 range):
      speed_score^0.3      — forward velocity (cap at 0.6 m/s)
      survival_score^0.2   — episode survival
      step_quality^0.3     — foot clearance + stepping regularity
      smoothness^0.1       — action smoothness (anti-vibration)
      energy^0.1           — cost of transport efficiency

    This prevents the vibrating gait optimum because:
    1. Smoothness directly penalizes high-frequency oscillation
    2. Step quality requires actual foot lifting (clearance > 0)
    3. Contact regularity requires periodic stepping (not random vibration)
    4. Speed is only 30% weight (not 100% like v2)
    """
    speed = metrics["avg_speed"]
    survival = metrics["avg_survival"]
    smoothness_raw = metrics["action_smoothness"]
    clearance = metrics["avg_foot_clearance"]
    regularity = metrics["contact_regularity"]
    cot = metrics["cost_of_transport"]
    step_count = metrics["avg_step_count"]

    # Speed score: linear up to 0.6 m/s, then capped
    speed_score = min(1.0, max(0.0, speed / 0.6))

    # Survival score: quadratic penalty below 95%
    survival_score = min(1.0, survival / 95.0) ** 2

    # Step quality: combination of clearance, regularity, and step count
    # Clearance: reward 3-8cm range, penalize <1cm or >15cm
    clearance_score = np.exp(-((clearance - 0.06) ** 2) / 0.003)  # peak at 6cm
    # Regularity: lower CV is better (CV of 0.3 is good, >1.5 is bad)
    regularity_score = max(0.0, 1.0 - regularity / 2.0)
    # Step count: should have ~10+ steps per leg in 500 steps at 50Hz (10s)
    step_score = min(1.0, step_count / 8.0)
    # Combined step quality
    step_quality = clearance_score * 0.4 + regularity_score * 0.3 + step_score * 0.3

    # Smoothness: action RMS < 0.1 is excellent, > 0.5 is vibrating
    smoothness_score = max(0.0, 1.0 - smoothness_raw / 0.5)

    # Energy: CoT < 2 is efficient, > 10 is wasteful
    energy_score = max(0.0, 1.0 - cot / 10.0)

    # Composite: geometric weighted mean (all components must be nonzero)
    # Clamp to prevent log(0)
    eps = 1e-6
    composite = (
        max(eps, speed_score) ** 0.30
        * max(eps, survival_score) ** 0.20
        * max(eps, step_quality) ** 0.30
        * max(eps, smoothness_score) ** 0.10
        * max(eps, energy_score) ** 0.10
    )

    return composite


def objective(trial):
    """Single HPO trial: sample params, train, evaluate with gait quality."""
    import optuna
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    trial_start = time.time()

    # ===== SAMPLE PARAMETERS =====

    # Tier 1: Training hyperparams (narrow ranges from v2 findings)
    action_scale = trial.suggest_float("action_scale", 0.3, 0.8)
    ent_coef = trial.suggest_float("ent_coef", 0.0, 0.005)
    kl_target = trial.suggest_float("kl_target", 0.005, 0.03)

    # Tier 2: Core reward weights
    rew_track_lin_vel = trial.suggest_float("rew_track_lin_vel", 1.0, 6.0)
    rew_alive = trial.suggest_float("rew_alive", 0.1, 1.0)
    rew_orientation = trial.suggest_float("rew_orientation", -2.0, -0.1)
    tracking_sigma = trial.suggest_float("tracking_sigma", 0.1, 0.4)
    fall_penalty = trial.suggest_float("fall_penalty", 1.0, 10.0)
    rew_gait_clock = trial.suggest_float("rew_gait_clock", 0.1, 1.5)

    # Tier 3: Anti-vibration rewards (THIS IS THE KEY CHANGE)
    # NOTE: Our env uses L1 norm over 12 joints, scaled by dt=0.02.
    # At smooth1=-0.5, typical per-step penalty ≈ 0.024 (vs tracking reward ~0.09).
    # At smooth1=-1.5, penalty ≈ 0.072 (approaching tracking reward — upper bound).
    rew_action_smooth_1 = trial.suggest_float("rew_action_smooth_1", -1.5, -0.05)
    rew_action_smooth_2 = trial.suggest_float("rew_action_smooth_2", -1.0, 0.0)

    # Tier 4: Shaped gait rewards
    rew_gait_force = trial.suggest_float("rew_gait_force", 0.0, 3.0)
    rew_gait_vel = trial.suggest_float("rew_gait_vel", 0.0, 2.0)
    rew_foot_slip = trial.suggest_float("rew_foot_slip", -0.3, 0.0)
    rew_clearance_target = trial.suggest_float("rew_clearance_target", -25.0, -1.0)
    rew_air_time_var = trial.suggest_float("rew_air_time_var", -1.5, 0.0)

    # Tier 5: Gait geometry
    gait_period = trial.suggest_float("gait_period", 0.4, 0.7)
    swing_height_target = trial.suggest_float("swing_height_target", 0.04, 0.12)
    # Air time threshold must be < gait_period/2 (max possible air time per step)
    # Use a fraction of half the gait period
    air_time_fraction = trial.suggest_float("air_time_fraction", 0.4, 0.9)
    feet_air_time_threshold = air_time_fraction * gait_period / 2.0

    # --- Build env kwargs ---
    env_kwargs = dict(
        cmd_vx_range=(0.0, 0.7),
        cmd_vy_range=(-0.1, 0.1),
        cmd_yaw_range=(-0.3, 0.3),
        action_scale=action_scale,
        use_foot_contacts=True,       # proven by v2
        only_positive_rewards=False,  # proven by v2
        rew_track_lin_vel=rew_track_lin_vel,
        rew_alive=rew_alive,
        rew_orientation=rew_orientation,
        tracking_sigma=tracking_sigma,
        fall_penalty=fall_penalty,
        rew_gait_clock=rew_gait_clock,
        rew_action_smooth_1=rew_action_smooth_1,
        rew_action_smooth_2=rew_action_smooth_2,
        rew_gait_force=rew_gait_force,
        rew_gait_vel=rew_gait_vel,
        rew_foot_slip=rew_foot_slip,
        rew_clearance_target=rew_clearance_target,
        rew_air_time_var=rew_air_time_var,
        gait_period=gait_period,
        swing_height_target=swing_height_target,
        feet_air_time_threshold=feet_air_time_threshold,
    )

    print(f"\n{'='*60}")
    print(f"Trial #{trial.number}")
    print(f"  act_scale={action_scale:.2f}  ent={ent_coef:.4f}  kl={kl_target:.3f}")
    print(f"  track={rew_track_lin_vel:.2f}  alive={rew_alive:.2f}  orient={rew_orientation:.2f}")
    print(f"  sigma={tracking_sigma:.3f}  fall={fall_penalty:.1f}  gait_clk={rew_gait_clock:.2f}")
    print(f"  smooth1={rew_action_smooth_1:.2f}  smooth2={rew_action_smooth_2:.2f}")
    print(f"  gait_f={rew_gait_force:.2f}  gait_v={rew_gait_vel:.2f}  slip={rew_foot_slip:.2f}")
    print(f"  clearance={rew_clearance_target:.1f}  air_var={rew_air_time_var:.2f}")
    print(f"  period={gait_period:.2f}  swing_h={swing_height_target:.3f}")
    print(f"  air_frac={air_time_fraction:.2f}  air_thresh={feet_air_time_threshold:.3f}s (period/2={gait_period/2:.2f})")
    print(f"{'='*60}")

    # --- Create parallel environments (12 of 14 cores, leave 2 for system) ---
    n_envs = 12

    def make_env(rank=0):
        def _init():
            sys.path.insert(0, str(SCRIPT_DIR))
            from cw1_env import CW1LocomotionEnv
            env = CW1LocomotionEnv(**env_kwargs)
            env.reset(seed=rank)
            return env
        return _init

    try:
        env = SubprocVecEnv([make_env(rank=i) for i in range(n_envs)])
    except Exception as e:
        print(f"  ENV ERROR: {e}")
        gc.collect()
        return -1.0

    # --- Create PPO model ---
    policy_kwargs = dict(
        net_arch=dict(pi=[512, 256, 128], vf=[512, 256, 128]),
        activation_fn=torch.nn.ELU,
    )

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=1360,  # buffer = 12*1360 = 16320 (divisible by 64)
        batch_size=64,
        n_epochs=5,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=ent_coef,
        vf_coef=1.0,
        max_grad_norm=1.0,
        policy_kwargs=policy_kwargs,
        verbose=0,
        device="cpu",
    )

    # --- Adaptive LR callback ---
    class AdaptiveLRCallback(BaseCallback):
        def __init__(self, kl_target_val=0.02):
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

    # --- Eval + pruning callback ---
    TOTAL_STEPS = 1_500_000
    EVAL_FREQ = 250_000

    class OptunaEvalCallback(BaseCallback):
        def __init__(self, trial, env_kwargs, eval_freq=EVAL_FREQ):
            super().__init__()
            self.trial = trial
            self.env_kwargs = env_kwargs
            self.eval_freq = eval_freq
            self.last_eval_step = 0
            self.best_score = -1.0
            self.best_metrics = None
            self.last_score = 0.0

        def _on_step(self):
            if self.num_timesteps - self.last_eval_step >= self.eval_freq:
                self.last_eval_step = self.num_timesteps
                step_k = self.num_timesteps // 1000

                metrics = evaluate_policy_v3(
                    self.model, self.env_kwargs,
                    n_episodes=10, n_steps=500, cmd_vx=0.4,
                )
                score = compute_objective_v3(metrics)

                self.last_score = score
                if score > self.best_score:
                    self.best_score = score
                    self.best_metrics = metrics

                step_idx = self.num_timesteps // self.eval_freq
                self.trial.report(score, step_idx)

                elapsed = time.time() - trial_start
                print(
                    f"  [{step_k}K {elapsed:.0f}s] "
                    f"spd={metrics['avg_speed']:.3f} "
                    f"surv={metrics['avg_survival']:.0f}% "
                    f"smooth={metrics['action_smoothness']:.3f} "
                    f"clear={metrics['avg_foot_clearance']:.3f} "
                    f"steps={metrics['avg_step_count']:.0f} "
                    f"score={score:.4f}"
                )

                # 500K triage gate (lenient: only kill clearly hopeless trials)
                if self.num_timesteps >= 500_000 and self.num_timesteps < 750_000:
                    surv = metrics["avg_survival"]
                    tlt = metrics["avg_tilt"]
                    if surv < 30 or tlt > 20:
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
    eval_cb = OptunaEvalCallback(trial, env_kwargs, eval_freq=EVAL_FREQ)
    adaptive_lr = AdaptiveLRCallback(kl_target_val=kl_target)

    try:
        model.learn(
            total_timesteps=TOTAL_STEPS,
            callback=[eval_cb, adaptive_lr],
            progress_bar=False,
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
    # Stability-aware score: penalize policies that peak then collapse.
    # stable_score = best * sqrt(final/best). If final ≈ best, no penalty.
    # If final << best (collapse), score is heavily reduced.
    best_s = max(eval_cb.best_score, 1e-8)
    final_s = max(eval_cb.last_score, 0.0)
    stability_ratio = min(final_s / best_s, 1.0)
    stable_score = best_s * (stability_ratio ** 0.5)

    if eval_cb.best_metrics:
        m = eval_cb.best_metrics
        print(
            f"  DONE ({elapsed:.0f}s): spd={m['avg_speed']:.3f} "
            f"surv={m['avg_survival']:.0f}% "
            f"smooth={m['action_smoothness']:.3f} "
            f"clear={m['avg_foot_clearance']:.3f} "
            f"steps={m['avg_step_count']:.0f} "
            f"best={eval_cb.best_score:.4f} final={final_s:.4f} "
            f"stable_score={stable_score:.4f}"
        )
    return stable_score


def main():
    import optuna

    parser = argparse.ArgumentParser(description="HPO v3 — Gait Quality Objective")
    parser.add_argument("--n-trials", type=int, default=200)
    parser.add_argument("--study-name", type=str, default="cw1_locomotion_v3")
    parser.add_argument(
        "--resume", action="store_true", help="Resume a previous study"
    )
    parser.add_argument(
        "--db", type=str, default=str(SCRIPT_DIR / "hpo_v3_study.db"),
    )
    args = parser.parse_args()

    print("=" * 60)
    print("CW-1 Locomotion HPO v3 — Gait Quality Objective")
    print("=" * 60)
    print(f"Trials: {args.n_trials}")
    print(f"Study: {args.study_name}")
    print(f"Database: {args.db}")
    print(f"Protocol: 1.5M steps, eval@250K, 500K triage")
    print(f"Search: 20 params (anti-vibration + shaped gait + geometry)")
    print(f"Objective: speed^0.3 * survival^0.2 * step_quality^0.3 * smooth^0.1 * energy^0.1")
    print(f"Key change: v2 objective = speed*survival (caused vibration)")
    print(f"           v3 objective = composite gait quality (rewards actual stepping)")
    est_min = args.n_trials * 8
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
    else:
        # Seed 1: Trial #21 baseline with mild smoothness
        study.enqueue_trial({
            "action_scale": 0.574,
            "ent_coef": 0.0017,
            "kl_target": 0.015,
            "rew_track_lin_vel": 4.49,
            "rew_alive": 0.14,
            "rew_orientation": -1.71,
            "tracking_sigma": 0.145,
            "fall_penalty": 9.08,
            "rew_gait_clock": 0.67,
            "rew_action_smooth_1": -0.1,
            "rew_action_smooth_2": 0.0,
            "rew_gait_force": 0.0,
            "rew_gait_vel": 0.0,
            "rew_foot_slip": 0.0,
            "rew_clearance_target": -1.69,
            "rew_air_time_var": 0.0,
            "gait_period": 0.5,
            "swing_height_target": 0.06,
            "air_time_fraction": 0.8,  # 0.8 * 0.25 = 0.2s threshold
        })
        # Seed 2: Anti-vibration (moderate smoothness + shaped rewards)
        study.enqueue_trial({
            "action_scale": 0.5,
            "ent_coef": 0.001,
            "kl_target": 0.015,
            "rew_track_lin_vel": 3.0,
            "rew_alive": 0.5,
            "rew_orientation": -1.0,
            "tracking_sigma": 0.25,
            "fall_penalty": 5.0,
            "rew_gait_clock": 0.8,
            "rew_action_smooth_1": -0.5,
            "rew_action_smooth_2": -0.3,
            "rew_gait_force": 1.5,
            "rew_gait_vel": 1.0,
            "rew_foot_slip": -0.1,
            "rew_clearance_target": -10.0,
            "rew_air_time_var": -0.5,
            "gait_period": 0.5,
            "swing_height_target": 0.08,
            "air_time_fraction": 0.7,  # 0.7 * 0.25 = 0.175s threshold
        })
        # Seed 3: Conservative walker (wide sigma, strong gait clock)
        study.enqueue_trial({
            "action_scale": 0.4,
            "ent_coef": 0.0,
            "kl_target": 0.02,
            "rew_track_lin_vel": 2.0,
            "rew_alive": 0.5,
            "rew_orientation": -0.5,
            "tracking_sigma": 0.3,
            "fall_penalty": 5.0,
            "rew_gait_clock": 1.2,
            "rew_action_smooth_1": -0.8,
            "rew_action_smooth_2": -0.5,
            "rew_gait_force": 2.0,
            "rew_gait_vel": 1.0,
            "rew_foot_slip": -0.1,
            "rew_clearance_target": -15.0,
            "rew_air_time_var": -0.5,
            "gait_period": 0.6,
            "swing_height_target": 0.08,
            "air_time_fraction": 0.6,  # 0.6 * 0.30 = 0.18s threshold
        })
        print("Seeded with 3 trials: v2-best-mild, anti-vibration, conservative-walker")

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
        if isinstance(value, float):
            print(f"    {key}: {value:.4f}")
        else:
            print(f"    {key}: {value}")

    # Parameter importance
    try:
        importances = optuna.importance.get_param_importances(study)
        print(f"\nParameter importance:")
        for key, value in sorted(
            importances.items(), key=lambda x: -x[1]
        ):
            bar = "#" * int(value * 40)
            print(f"  {key:>25s}: {value:.3f} {bar}")
    except Exception:
        pass

    # Top 5 trials
    top = sorted(completed, key=lambda t: t.value, reverse=True)[:5]
    print(f"\nTop 5 trials:")
    for t in top:
        p = t.params
        print(
            f"  #{t.number}: score={t.value:.4f}"
            f"  smooth1={p.get('rew_action_smooth_1', '?'):.2f}"
            f"  smooth2={p.get('rew_action_smooth_2', '?'):.2f}"
            f"  gait_f={p.get('rew_gait_force', '?'):.1f}"
            f"  period={p.get('gait_period', '?'):.2f}"
        )


if __name__ == "__main__":
    main()
