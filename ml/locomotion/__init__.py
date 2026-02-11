# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""CleanWalker CW-1 locomotion training package for IsaacLab + rsl_rl."""

import gymnasium as gym

gym.register(
    id="CleanWalker-CW1-Flat-v0",
    entry_point="isaaclab.envs:DirectRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cleanwalker_env:CleanWalkerFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.train_config:CleanWalkerFlatPPORunnerCfg",
    },
)

gym.register(
    id="CleanWalker-CW1-Rough-v0",
    entry_point="isaaclab.envs:DirectRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cleanwalker_env:CleanWalkerRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.train_config:CleanWalkerRoughPPORunnerCfg",
    },
)
