# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Reference-signal builders for the closed-loop sim.

Each builder returns a callable ``r(t) -> float`` that the closed-loop
sim evaluates at every control tick. Stateless / pure functions; the
sim is responsible for advancing time.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable, Sequence

import numpy as np


def step(amplitude: float, t_start: float = 0.5) -> Callable[[float], float]:
    """Single step from 0 to ``amplitude`` at ``t_start`` and hold."""
    def r(t: float) -> float:
        return amplitude if t >= t_start else 0.0
    return r


def staircase(
    amplitudes: Sequence[float],
    dwell_s: float,
    t_start: float = 0.5,
) -> Callable[[float], float]:
    """Step-and-hold sequence. ``amplitudes[0]`` becomes active at
    ``t_start``; each subsequent level holds for ``dwell_s``.

    Returns 0 before ``t_start`` and the last amplitude after the
    sequence completes.
    """
    levels = list(amplitudes)
    edges = [t_start + i * dwell_s for i in range(len(levels))]

    def r(t: float) -> float:
        if t < t_start:
            return 0.0
        for i in range(len(levels) - 1, -1, -1):
            if t >= edges[i]:
                return float(levels[i])
        return 0.0
    return r


def ramp(
    slope: float, duration: float, t_start: float = 0.5,
    final_hold: bool = True,
) -> Callable[[float], float]:
    """Linear ramp starting at ``t_start``, climbing at ``slope`` for
    ``duration`` seconds. If ``final_hold`` is True, holds the final
    value after the ramp ends; otherwise drops back to zero.
    """
    final_value = slope * duration

    def r(t: float) -> float:
        if t < t_start:
            return 0.0
        elapsed = t - t_start
        if elapsed >= duration:
            return float(final_value) if final_hold else 0.0
        return float(slope * elapsed)
    return r


def sinusoid(
    amplitude: float, freq_hz: float, offset: float = 0.0,
    t_start: float = 0.0, phase_rad: float = 0.0,
) -> Callable[[float], float]:
    """``offset + amplitude * sin(2π·f·(t - t_start) + φ)`` for
    ``t >= t_start``, ``offset`` before. Useful for tracking-bandwidth
    probes.
    """
    omega = 2.0 * math.pi * freq_hz

    def r(t: float) -> float:
        if t < t_start:
            return float(offset)
        return float(offset + amplitude * math.sin(omega * (t - t_start) + phase_rad))
    return r


def from_array(
    t_array: np.ndarray, r_array: np.ndarray,
) -> Callable[[float], float]:
    """ZOH lookup against (t_array, r_array). Use this to replay a
    recorded velocity profile from a real session.
    """
    t_array = np.asarray(t_array, dtype=float)
    r_array = np.asarray(r_array, dtype=float)
    if t_array.shape != r_array.shape:
        raise ValueError("t_array and r_array must have the same shape")
    if t_array.size == 0:
        raise ValueError("t_array must be non-empty")

    def r(t: float) -> float:
        # ZOH: most recent sample at or before t.
        if t < t_array[0]:
            return float(r_array[0])
        if t >= t_array[-1]:
            return float(r_array[-1])
        idx = int(np.searchsorted(t_array, t, side="right") - 1)
        idx = max(0, min(idx, t_array.size - 1))
        return float(r_array[idx])
    return r


def realistic_path_velocity(profile_path: Path) -> Callable[[float], float]:
    """Load a recorded path-following velocity profile from disk.

    Expected format: a 2-column .npy or .csv with columns ``(t, v)``.
    Raises ``FileNotFoundError`` if the file doesn't exist; callers can
    catch and fall back to ``staircase`` / ``step``.
    """
    profile_path = Path(profile_path).expanduser()
    if not profile_path.exists():
        raise FileNotFoundError(f"velocity profile not found: {profile_path}")
    if profile_path.suffix == ".npy":
        arr = np.load(profile_path)
    else:
        arr = np.loadtxt(profile_path, delimiter=",")
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError(f"expected (N, 2) array; got shape {arr.shape}")
    return from_array(arr[:, 0], arr[:, 1])


__all__ = [
    "step", "staircase", "ramp", "sinusoid",
    "from_array", "realistic_path_velocity",
]
