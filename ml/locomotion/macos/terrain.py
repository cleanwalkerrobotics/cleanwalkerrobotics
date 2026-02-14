"""Heightfield terrain generator for CW-1 locomotion training.

Generates MuJoCo-compatible heightfield data for various terrain types.
Used by cw1_env.py to randomize terrain on each episode reset.

Heightfield parameters (must match cw1_scene.xml):
  nrow=80, ncol=400, size="20 2 6.0 0.0"
  Resolution: 0.05m per cell (40m / 400 cols, 4m / 80 rows)
  Height range: 0-6m (mapped to uint16 via MuJoCo)

Terrain types:
  flat        — baseline, all zeros
  rough       — random bumps (0.01-0.05m amplitude)
  obstacles   — rectangular blocks (0.02-0.08m height)
  stairs_up   — ascending steps (0.05-0.15m height, 0.3m depth)
  slope_30    — 30-degree incline
  slope_debris — 30-degree slope with random noise
"""

import numpy as np


# Heightfield dimensions (must match MJCF hfield asset)
HFIELD_NROW = 80
HFIELD_NCOL = 400
HFIELD_SIZE_X = 20.0   # half-size in X (total 40m)
HFIELD_SIZE_Y = 2.0    # half-size in Y (total 4m)
HFIELD_SIZE_Z = 6.0    # max height
HFIELD_SIZE_W = 0.01   # min height offset (must be >0 for MuJoCo)

# Derived resolution
CELL_SIZE_X = (2 * HFIELD_SIZE_X) / HFIELD_NCOL  # 0.1m
CELL_SIZE_Y = (2 * HFIELD_SIZE_Y) / HFIELD_NROW  # 0.05m

TERRAIN_TYPES = ["flat", "rough", "obstacles", "stairs_up", "slope_30", "slope_debris"]


class TerrainGenerator:
    """Generate heightfield terrain data for MuJoCo simulation."""

    def __init__(self, rng: np.random.Generator | None = None):
        self.rng = rng or np.random.default_rng()
        self._current_heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)
        self._current_type = "flat"

    def generate(self, terrain_type: str = "flat") -> np.ndarray:
        """Generate heightfield data for the given terrain type.

        Args:
            terrain_type: One of TERRAIN_TYPES.

        Returns:
            Float array (nrow, ncol) of heights in meters, normalized to [0, 1]
            for MuJoCo hfield_data (which scales by hfield size_z).
        """
        if terrain_type not in TERRAIN_TYPES:
            raise ValueError(f"Unknown terrain: {terrain_type}. Use one of {TERRAIN_TYPES}")

        self._current_type = terrain_type

        if terrain_type == "flat":
            heights = self._gen_flat()
        elif terrain_type == "rough":
            heights = self._gen_rough()
        elif terrain_type == "obstacles":
            heights = self._gen_obstacles()
        elif terrain_type == "stairs_up":
            heights = self._gen_stairs_up()
        elif terrain_type == "slope_30":
            heights = self._gen_slope_30()
        elif terrain_type == "slope_debris":
            heights = self._gen_slope_debris()

        self._current_heights = heights.copy()

        # Normalize to [0, 1] for MuJoCo (hfield_data stores normalized values)
        if HFIELD_SIZE_Z > 0:
            normalized = heights / HFIELD_SIZE_Z
        else:
            normalized = np.zeros_like(heights)

        return normalized.astype(np.float32).ravel()

    def get_height_at(self, x: float, y: float) -> float:
        """Get terrain height at world coordinates (x, y).

        Args:
            x: World X position (robot forward direction)
            y: World Y position (lateral)

        Returns:
            Height in meters at that position.
        """
        # World coords to heightfield indices
        # hfield centered at origin: x in [-size_x, size_x], y in [-size_y, size_y]
        col = int((x + HFIELD_SIZE_X) / (2 * HFIELD_SIZE_X) * HFIELD_NCOL)
        row = int((y + HFIELD_SIZE_Y) / (2 * HFIELD_SIZE_Y) * HFIELD_NROW)

        col = np.clip(col, 0, HFIELD_NCOL - 1)
        row = np.clip(row, 0, HFIELD_NROW - 1)

        return float(self._current_heights[row, col])

    def get_heights_at(self, xy: np.ndarray) -> np.ndarray:
        """Batch terrain height query. xy shape: (N, 2). Returns (N,) heights."""
        cols = ((xy[:, 0] + HFIELD_SIZE_X) / (2 * HFIELD_SIZE_X) * HFIELD_NCOL).astype(int)
        rows = ((xy[:, 1] + HFIELD_SIZE_Y) / (2 * HFIELD_SIZE_Y) * HFIELD_NROW).astype(int)
        np.clip(cols, 0, HFIELD_NCOL - 1, out=cols)
        np.clip(rows, 0, HFIELD_NROW - 1, out=rows)
        return self._current_heights[rows, cols].astype(np.float32)

    def _gen_flat(self) -> np.ndarray:
        """Flat terrain — all zeros."""
        return np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)

    def _gen_rough(self) -> np.ndarray:
        """Random bumps with 0.01-0.05m amplitude."""
        amplitude = self.rng.uniform(0.01, 0.05)
        # Use smoothed noise for natural-looking bumps
        raw = self.rng.standard_normal((HFIELD_NROW, HFIELD_NCOL))
        # Simple box filter for smoothing (3x3 kernel)
        kernel_size = 3
        from scipy.ndimage import uniform_filter
        smoothed = uniform_filter(raw, size=kernel_size)
        # Scale to desired amplitude
        smoothed = smoothed / (np.abs(smoothed).max() + 1e-8) * amplitude
        # Shift to non-negative
        smoothed -= smoothed.min()
        return smoothed.astype(np.float32)

    def _gen_obstacles(self) -> np.ndarray:
        """Rectangular blocks scattered on flat ground."""
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)

        n_obstacles = self.rng.integers(15, 30)
        for _ in range(n_obstacles):
            h = self.rng.uniform(0.02, 0.08)
            # Block size: 2-6 cells wide (0.1-0.3m), 2-10 cells long (0.2-1.0m)
            w = self.rng.integers(2, 7)
            l = self.rng.integers(2, 11)
            # Random position (leave 5m buffer at start for spawn)
            col_start = self.rng.integers(50, HFIELD_NCOL - l)
            row_start = self.rng.integers(0, HFIELD_NROW - w)
            heights[row_start:row_start + w, col_start:col_start + l] = h

        return heights

    def _gen_stairs_up(self) -> np.ndarray:
        """Ascending staircase with configurable step height/depth."""
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)

        step_height = self.rng.uniform(0.05, 0.15)
        step_depth_cells = int(0.3 / CELL_SIZE_X)  # 0.3m depth = 3 cells

        # Start stairs after 2m buffer (20 cells from center)
        start_col = HFIELD_NCOL // 2 + 20
        current_height = 0.0

        col = start_col
        while col < HFIELD_NCOL:
            end_col = min(col + step_depth_cells, HFIELD_NCOL)
            heights[:, col:end_col] = current_height
            current_height += step_height
            col = end_col

        return heights

    def _gen_slope_30(self) -> np.ndarray:
        """30-degree incline starting from center."""
        heights = np.zeros((HFIELD_NROW, HFIELD_NCOL), dtype=np.float32)

        slope = np.tan(np.radians(30))  # rise per meter = 0.577
        start_col = HFIELD_NCOL // 2 + 20  # 2m buffer from center

        for col in range(start_col, HFIELD_NCOL):
            x_dist = (col - start_col) * CELL_SIZE_X
            heights[:, col] = slope * x_dist

        return heights

    def _gen_slope_debris(self) -> np.ndarray:
        """30-degree slope with random noise (debris)."""
        heights = self._gen_slope_30()

        # Add noise only on the slope portion
        start_col = HFIELD_NCOL // 2 + 20
        noise_amp = self.rng.uniform(0.02, 0.05)
        noise = self.rng.standard_normal((HFIELD_NROW, HFIELD_NCOL - start_col))
        from scipy.ndimage import uniform_filter
        noise = uniform_filter(noise, size=2) * noise_amp
        heights[:, start_col:] += noise
        heights = np.maximum(heights, 0)  # no negative heights

        return heights
