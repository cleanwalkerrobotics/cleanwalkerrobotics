# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
PPO training configuration for CleanWalker CW-1 locomotion.

Uses rsl_rl (ETH Zurich Robotic Systems Lab) PPO implementation,
the same algorithm used by Unitree for Go2 locomotion and by
ANYbotics for ANYmal.

Training pipeline:
  1. Flat terrain (CleanWalkerFlatPPORunnerCfg) — ~1500 iterations
  2. Rough terrain (CleanWalkerRoughPPORunnerCfg) — ~3000 iterations
     Load checkpoint from flat training as starting point.
  3. Litter collection (CleanWalkerLitterPPORunnerCfg) — ~2000 iterations
     Load checkpoint from rough training. Adds arm control.

Usage:
    # Phase 1: Flat terrain
    ./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Flat-v0

    # Phase 2: Rough terrain (resume from flat)
    ./isaaclab.sh -p scripts/train.py --task CleanWalker-CW1-Rough-v0 \\
        --load_run <flat_run_name> --checkpoint model_1500.pt

    # Phase 3: Litter collection (resume from rough)
    python ml/locomotion/train.py --task CleanWalker-CW1-Litter-v0 \\
        --load_run <rough_run_name> --checkpoint model_3000.pt
"""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class CleanWalkerFlatPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO runner config for flat terrain locomotion training."""

    seed: int = 42
    device: str = "cuda"
    num_steps_per_env: int = 24
    max_iterations: int = 1500
    save_interval: int = 100
    experiment_name: str = "cleanwalker_cw1_flat"
    run_name: str = ""
    logger: str = "tensorboard"
    resume: bool = False
    load_run: str = ".*"
    load_checkpoint: str = "model_.*.pt"

    policy: RslRlPpoActorCriticCfg = RslRlPpoActorCriticCfg(
        class_name="ActorCritic",
        init_noise_std=1.0,
        noise_std_type="scalar",
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )

    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class CleanWalkerRoughPPORunnerCfg(CleanWalkerFlatPPORunnerCfg):
    """PPO runner config for rough terrain locomotion training.

    Inherits from flat config with adjustments for harder terrain:
      - More iterations (curriculum needs longer to converge)
      - Slightly lower learning rate for stability
      - Resume from flat-trained checkpoint
    """

    max_iterations: int = 3000
    experiment_name: str = "cleanwalker_cw1_rough"

    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,          # Lower entropy for fine-tuning
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5.0e-4,        # Half the flat LR for stability
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.008,            # Tighter KL for rough terrain
        max_grad_norm=1.0,
    )


@configclass
class CleanWalkerLitterPPORunnerCfg(CleanWalkerRoughPPORunnerCfg):
    """PPO runner config for litter collection training.

    Extends rough terrain config with adjustments for combined
    locomotion + arm control:
      - Wider policy network (handles 17 actions, 58 observations)
      - More iterations for the dual-task learning
      - Lower learning rate (fine-tuning on top of locomotion)
      - Higher entropy to encourage exploration of arm behaviors
    """

    max_iterations: int = 2000
    experiment_name: str = "cleanwalker_cw1_litter"

    policy: RslRlPpoActorCriticCfg = RslRlPpoActorCriticCfg(
        class_name="ActorCritic",
        init_noise_std=0.5,          # Lower noise — fine-tuning, not from scratch
        noise_std_type="scalar",
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )

    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.008,          # Slightly higher for arm exploration
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,        # Lower LR for stability
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.008,
        max_grad_norm=1.0,
    )
