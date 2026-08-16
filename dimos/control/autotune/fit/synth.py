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

"""Synthetic ground-truth generator for pinning the FOPDT fitters.

Given a known ``(K, tau, L)`` plant and a measurement model (sample rate +
noise), produce the exact velocity and pose traces the plant would emit for a
step. The fitters must recover the known parameters from these traces - this is
how we validate the fitters offline, with no robot or sim. Distilled from the
characterization ground-truth harness, dropping its DB I/O and Go2 grids.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MeasurementModel:
    """How a continuous true signal becomes a measured stream.

    ``rate_hz`` is the odom sample rate; ``noise_std`` is per-sample Gaussian
    sigma on the measured channel; ``drift`` is a constant velocity slip added to
    pose (the odom-slip nuisance the pose fitter absorbs)."""

    rate_hz: float
    noise_std: float = 0.0
    drift: float = 0.0


def true_velocity(t: np.ndarray, K: float, tau: float, L: float, amp: float) -> np.ndarray:
    """Continuous FOPDT velocity step response, ``t`` relative to the step edge."""
    t = np.asarray(t, dtype=float)
    v = np.zeros_like(t)
    mask = t >= L
    v[mask] = K * amp * (1.0 - np.exp(-(t[mask] - L) / tau))
    return v


def true_pose(t: np.ndarray, K: float, tau: float, L: float, amp: float) -> np.ndarray:
    """Closed-form integral of :func:`true_velocity` (pose from rest, p0=0)."""
    t = np.asarray(t, dtype=float)
    p = np.zeros_like(t)
    mask = t >= L
    s = t[mask] - L
    p[mask] = K * amp * (s - tau * (1.0 - np.exp(-s / tau)))
    return p


def synth_step(
    K: float,
    tau: float,
    L: float,
    amp: float,
    duration_s: float,
    model: MeasurementModel,
    *,
    seed: int = 0,
    p0: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Synthesize one measured step at the model's sample rate.

    Returns ``(t_rel, velocity_meas, pose_meas)`` sampled at ``rate_hz``, with
    Gaussian noise and constant drift applied. ``t_rel`` is relative to the step
    edge (step at t=0). Noise is seeded for reproducibility; vary ``seed`` per
    call to get independent realizations."""
    n = max(4, round(duration_s * model.rate_hz))
    t = np.arange(n) / model.rate_hz
    v = true_velocity(t, K, tau, L, amp)
    p = p0 + model.drift * t + true_pose(t, K, tau, L, amp)
    if model.noise_std > 0:
        rng = np.random.default_rng(seed)
        v = v + rng.normal(0.0, model.noise_std, size=n)
        # Pose noise is smaller-magnitude than velocity noise in practice; scale
        # it so the pose SNR is comparable rather than identical sigma.
        p = p + rng.normal(0.0, model.noise_std / model.rate_hz, size=n)
    return t, v, p
