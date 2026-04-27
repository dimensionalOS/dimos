# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Outlier rejection and signal cleaning.

Two-stage filter applied to derived body-frame velocities:

  1. Hampel filter (rolling-MAD outlier replacement). Window 7, k=3 by
     default — flags samples whose deviation from the local median
     exceeds k × MAD and replaces them with the local median.

  2. Physics bound. Reject samples implying acceleration |a| > 8 m/s² or
     |α| > 20 rad/s². The Go2 hardware can't actually achieve these;
     anything past these is sensor / odometry noise.

**The raw arrays are never mutated.** Functions return cleaned copies
plus a stats dict reporting what was rejected. Persistence is the
caller's choice — typically processed.json next to the run.

These cleaned signals are appropriate for *plotting* and for
*long-window steady-state estimation*. They are NOT appropriate for
edge / timing measurements (rise time, deadtime onset, FOPDT fitting),
which should use the raw signal — see ``README.md`` "Processing rule".
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np


_DEFAULT_HAMPEL_WINDOW = 7
_DEFAULT_HAMPEL_K = 3.0

# Physics bounds (per-channel). Go2 specs:
#   max linear accel ≈ 5 m/s² (sustained); we use 8 to give headroom for
#   transients without flagging real maneuvers.
#   max angular accel ≈ 15 rad/s²; we use 20.
_DEFAULT_AX_MAX = 8.0
_DEFAULT_ALPHA_MAX = 20.0


@dataclass
class CleaningStats:
    n_input: int
    n_hampel_rejected: int
    n_physics_rejected: int
    n_total_rejected: int

    def asdict(self) -> dict[str, int]:
        return asdict(self)


def hampel(x: np.ndarray, *, window: int = _DEFAULT_HAMPEL_WINDOW, k: float = _DEFAULT_HAMPEL_K
           ) -> tuple[np.ndarray, np.ndarray]:
    """Replace outlier samples with the local median.

    Returns ``(cleaned, mask)`` where ``mask[i] == True`` indicates
    sample ``i`` was flagged as an outlier and replaced.
    """
    if x.size == 0:
        return x.copy(), np.zeros(0, dtype=bool)
    half = window // 2
    cleaned = x.copy()
    mask = np.zeros_like(x, dtype=bool)
    for i in range(x.size):
        lo = max(0, i - half)
        hi = min(x.size, i + half + 1)
        local = x[lo:hi]
        med = np.median(local)
        mad = np.median(np.abs(local - med))
        # 1.4826 = consistency constant for normal distribution
        threshold = k * 1.4826 * mad
        if threshold > 0 and abs(x[i] - med) > threshold:
            cleaned[i] = med
            mask[i] = True
    return cleaned, mask


def physics_bound(
    ts: np.ndarray, x: np.ndarray, *, max_rate: float
) -> tuple[np.ndarray, np.ndarray]:
    """Reject samples implying ``|d x / d t| > max_rate``.

    Returns ``(cleaned, mask)``. Rejected samples are replaced by the
    previous valid value (forward-fill).
    """
    if x.size < 2:
        return x.copy(), np.zeros_like(x, dtype=bool)
    cleaned = x.copy()
    mask = np.zeros_like(x, dtype=bool)
    for i in range(1, x.size):
        dt = ts[i] - ts[i - 1]
        if dt <= 0:
            continue
        rate = abs(x[i] - cleaned[i - 1]) / dt
        if rate > max_rate:
            cleaned[i] = cleaned[i - 1]
            mask[i] = True
    return cleaned, mask


def clean_channel(
    ts: np.ndarray,
    x: np.ndarray,
    *,
    max_rate: float,
    hampel_window: int = _DEFAULT_HAMPEL_WINDOW,
    hampel_k: float = _DEFAULT_HAMPEL_K,
) -> tuple[np.ndarray, CleaningStats]:
    """Apply Hampel + physics filter in sequence to a single channel."""
    h_cleaned, h_mask = hampel(x, window=hampel_window, k=hampel_k)
    p_cleaned, p_mask = physics_bound(ts, h_cleaned, max_rate=max_rate)
    n_total = int(np.sum(h_mask | p_mask))
    return p_cleaned, CleaningStats(
        n_input=int(x.size),
        n_hampel_rejected=int(np.sum(h_mask)),
        n_physics_rejected=int(np.sum(p_mask)),
        n_total_rejected=n_total,
    )


def clean_run_velocities(
    ts: np.ndarray,
    vx_raw: np.ndarray,
    vy_raw: np.ndarray,
    wz_raw: np.ndarray,
    *,
    ax_max: float = _DEFAULT_AX_MAX,
    alpha_max: float = _DEFAULT_ALPHA_MAX,
    hampel_window: int = _DEFAULT_HAMPEL_WINDOW,
    hampel_k: float = _DEFAULT_HAMPEL_K,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    """Apply the full cleaning pipeline to body-frame velocities.

    The 'physics rate' for vx/vy is acceleration; for wz it's angular
    accel. So ``max_rate`` for the velocity-channel filter is in m/s²
    (not m/s) and rad/s² for wz.
    """
    vx_clean, vx_stats = clean_channel(
        ts, vx_raw, max_rate=ax_max, hampel_window=hampel_window, hampel_k=hampel_k,
    )
    vy_clean, vy_stats = clean_channel(
        ts, vy_raw, max_rate=ax_max, hampel_window=hampel_window, hampel_k=hampel_k,
    )
    wz_clean, wz_stats = clean_channel(
        ts, wz_raw, max_rate=alpha_max, hampel_window=hampel_window, hampel_k=hampel_k,
    )
    return vx_clean, vy_clean, wz_clean, {
        "vx": vx_stats.asdict(),
        "vy": vy_stats.asdict(),
        "wz": wz_stats.asdict(),
        "params": {
            "hampel_window": hampel_window,
            "hampel_k": hampel_k,
            "ax_max": ax_max,
            "alpha_max": alpha_max,
        },
    }


def write_processed_json(run_dir: Path, stats: dict[str, Any]) -> None:
    """Persist cleaning stats next to run.json (separate file)."""
    p = Path(run_dir) / "processed.json"
    p.write_text(json.dumps(stats, indent=2) + "\n")


__all__ = [
    "CleaningStats",
    "clean_channel",
    "clean_run_velocities",
    "hampel",
    "physics_bound",
    "write_processed_json",
]
