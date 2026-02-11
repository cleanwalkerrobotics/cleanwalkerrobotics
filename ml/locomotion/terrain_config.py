# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
Terrain configurations for CleanWalker CW-1 locomotion training.

Provides procedural terrain generation for curriculum-based training:
  - Phase 1: Flat ground (use CleanWalkerFlatEnvCfg)
  - Phase 2: Mixed rough terrain (use CleanWalkerRoughEnvCfg)

Terrain types are weighted by proportion and difficulty scales with
the curriculum system in IsaacLab.
"""

from __future__ import annotations

import isaaclab.terrains as terrain_gen
from isaaclab.utils import configclass


# ===========================================================================
# Rough terrain generator — mixed outdoor surfaces
# ===========================================================================
#
# Designed for urban/park environments where CW-1 will operate:
#   - Flat: sidewalks, paved paths
#   - Rough: grass, gravel, dirt
#   - Slopes: ramps, gentle hills
#   - Stairs: park steps, curbs
#
# Difficulty scales from 0.0 (easy) to 1.0 (hard) via curriculum.
# ===========================================================================

ROUGH_TERRAIN_CFG = terrain_gen.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    curriculum=True,
    sub_terrains={
        # --- Flat surfaces (sidewalks, paved areas) ---
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.15,
        ),

        # --- Random rough terrain (grass, gravel, dirt) ---
        "rough_low": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.20,
            noise_range=(0.01, 0.06),
            noise_step=0.01,
            border_width=0.25,
        ),
        "rough_high": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.10,
            noise_range=(0.04, 0.10),
            noise_step=0.02,
            border_width=0.25,
        ),

        # --- Slopes (ramps, gentle hills) ---
        "slopes": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.15,
            slope_range=(0.0, 0.4),
            platform_width=2.0,
            border_width=0.25,
        ),

        # --- Stairs ascending (park steps, curbs) ---
        "stairs_up": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.15,
            step_height_range=(0.03, 0.15),
            step_width=0.3,
            platform_width=3.0,
        ),

        # --- Stairs descending ---
        "stairs_down": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.15,
            step_height_range=(0.03, 0.15),
            step_width=0.3,
            platform_width=3.0,
        ),

        # --- Stepping stones (irregular surfaces) ---
        "stepping_stones": terrain_gen.HfSteppingStonesTerrainCfg(
            proportion=0.10,
            stone_height_max=0.04,
            stone_width_range=(0.3, 0.8),
            stone_distance_range=(0.04, 0.20),
            holes_depth=-3.0,
            platform_width=2.0,
        ),
    },
)
