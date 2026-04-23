import numpy as np


def divergence_to_tac(divergence_grid: np.ndarray) -> np.ndarray:
    """
    Convert divergence grid to time-to-contact grid.
    tau = 1/divergence; NaN where divergence <= ~0 (not approaching).
    Accepts NaN entries (e.g. unpopulated cells) and preserves them.
    """
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.where(divergence_grid > 1e-6, 1.0 / divergence_grid, np.nan)
