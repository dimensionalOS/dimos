# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Plant properties beyond the per-axis FOPDT, read from the same step data.

These are the asymmetry/nonlinear-shape and deadzone characteristics the user
asked for. Autotune *reports* them; it does NOT auto-select a controller from
them (that choice stays with the user). Intentionally not measured (marginal
benefit, see design.md): isolated measurement latency, hysteresis.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DirectionAsymmetry:
    """Forward vs reverse steady-state gain on one axis. ``asymmetric`` is True
    when the two differ by more than ``rel_threshold``; when False the axis is
    pooled to a single K."""

    K_forward: float
    K_reverse: float
    rel_difference: float
    asymmetric: bool


def direction_asymmetry(
    K_forward: float, K_reverse: float, *, rel_threshold: float = 0.15
) -> DirectionAsymmetry:
    """Compare per-direction gains. ``rel_difference`` is normalized by the mean
    magnitude so it is sign- and scale-robust."""
    denom = 0.5 * (abs(K_forward) + abs(K_reverse))
    rel = abs(abs(K_forward) - abs(K_reverse)) / denom if denom > 1e-9 else 0.0
    return DirectionAsymmetry(
        K_forward=K_forward,
        K_reverse=K_reverse,
        rel_difference=float(rel),
        asymmetric=bool(rel > rel_threshold),
    )


@dataclass(frozen=True)
class GainSchedule:
    """Steady-state gain as a function of commanded amplitude. ``nonlinear`` is
    True when K varies across amplitude by more than ``rel_threshold`` - the
    signal the user uses to decide whether a linear PI suffices or a
    gain-scheduled/nonlinear controller is warranted (their call, not ours)."""

    amplitudes: tuple[float, ...]
    gains: tuple[float, ...]
    rel_span: float
    nonlinear: bool


def gain_schedule(
    amplitudes: list[float], gains: list[float], *, rel_threshold: float = 0.15
) -> GainSchedule:
    """Assess gain-vs-amplitude nonlinearity. ``rel_span`` is the peak-to-peak K
    spread normalized by the mean |K|."""
    if len(amplitudes) != len(gains) or not gains:
        raise ValueError("amplitudes and gains must be equal-length and non-empty")
    g = np.asarray(gains, dtype=float)
    mean_mag = float(np.mean(np.abs(g)))
    rel_span = float((g.max() - g.min()) / mean_mag) if mean_mag > 1e-9 else 0.0
    return GainSchedule(
        amplitudes=tuple(amplitudes),
        gains=tuple(gains),
        rel_span=rel_span,
        nonlinear=bool(rel_span > rel_threshold),
    )


def cross_axis_coupling(on_axis_response: np.ndarray, off_axis_response: np.ndarray) -> float:
    """Coupling ratio when one axis is excited: RMS off-axis response over RMS
    on-axis response. 0 = decoupled; larger = more bleed into the other axis."""
    on = float(np.sqrt(np.mean(np.asarray(on_axis_response, dtype=float) ** 2)))
    off = float(np.sqrt(np.mean(np.asarray(off_axis_response, dtype=float) ** 2)))
    if on < 1e-9:
        return 0.0
    return off / on


@dataclass(frozen=True)
class Deadzone:
    """Smallest command magnitude that produces sustained motion."""

    threshold: float
    found: bool


def estimate_deadzone(
    commands: np.ndarray,
    body_velocity: np.ndarray,
    *,
    motion_threshold: float = 0.02,
    fractional_threshold: float = 0.05,
    sustained_samples: int = 5,
) -> Deadzone:
    """Find the deadzone from a slow ramp.

    A sample "moves" when ``|v| > motion_threshold`` AND ``|v| > fractional *
    |cmd|`` (the AND-test rejects both sensor noise and tiny commands that
    coincidentally exceed the floor). The deadzone is the smallest |command| at
    which motion is sustained for ``sustained_samples`` consecutive samples.
    ``commands`` should be monotonically increasing in magnitude (a ramp)."""
    cmd = np.abs(np.asarray(commands, dtype=float))
    vel = np.abs(np.asarray(body_velocity, dtype=float))
    if cmd.size != vel.size or cmd.size == 0:
        raise ValueError("commands and body_velocity must be equal-length, non-empty")

    moving = (vel > motion_threshold) & (vel > fractional_threshold * cmd)
    run = 0
    for i in range(cmd.size):
        if moving[i]:
            run += 1
            if run >= sustained_samples:
                # Deadzone is the command where the sustained run began.
                return Deadzone(threshold=float(cmd[i - sustained_samples + 1]), found=True)
        else:
            run = 0
    return Deadzone(threshold=float("nan"), found=False)
