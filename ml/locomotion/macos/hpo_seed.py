#!/usr/bin/env python3
"""Seed the HPO study with known-good Run 10 params."""
import optuna
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
db = str(SCRIPT_DIR / "hpo_study.db")
storage = f"sqlite:///{db}"

study = optuna.create_study(
    study_name="cw1_locomotion_v1",
    storage=storage,
    direction="maximize",
    load_if_exists=True,
)

# Seed with Run 10 winning config (score ~0.375)
study.enqueue_trial({
    "rew_track_lin_vel": 3.0,
    "rew_alive": 0.5,
    "rew_orientation": -1.0,
    "tracking_sigma": 0.15,
    "fall_penalty": 5.0,
    "rew_gait_clock": 0.5,
})

print(f"Seeded study with Run 10 baseline params. Total queued: {len(study.trials)}")
