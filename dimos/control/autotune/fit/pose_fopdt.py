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

"""Pose-domain (output-error) FOPDT fitter.

Fits the FOPDT against raw pose without ever differentiating the measurement.
At low odom rate the velocity-domain fitter's smoothing window is longer than
tau, which flattens the step and inflates tau, and co-fitting L with tau is
degenerate. This fitter forward-models pose a candidate (K, tau, L) would
produce and fits to raw pose (simulation-error identification).

Forward model for one step segment (axis excited alone, from rest, signed
amplitude ``amp``, ``t`` relative to the step edge):

    velocity:  v(t) = K*amp*(1 - exp(-(t-L)/tau))            for t >= L, else 0
    position:  p(t) = p0 + drift*t
                      + K*amp*[(t-L) - tau*(1 - exp(-(t-L)/tau))]   for t >= L

``p0`` (start offset) and ``drift`` (constant-velocity odom slip) are fit as
nuisance parameters so they cannot bias K/tau. L is fixed per fit;
:func:`estimate_deadtime` chooses it by profiling, decoupling it from tau.

Ported from the characterization harness; bounds default to values suited to a
legged base with slow odom but are parameters, not baked-in constants.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

K_ABS_MAX = 5.0
# Defaults tuned for a slow-odom legged base; override per robot.
DEFAULT_TAU_BOUNDS = (0.03, 0.6)
DEFAULT_L_BOUNDS = (0.05, 0.30)


@dataclass
class PoseFopdtParams:
    """Result of a pose-domain FOPDT fit. ``r_squared``/``rmse`` are on POSE
    (the fitted quantity), not a differentiated velocity. ``valid`` is the
    plausibility gate: ``False`` means the fit landed outside physical bounds
    and must not be tuned from."""

    K: float
    tau: float
    L: float  # fixed input, echoed for the record
    p0: float
    drift: float
    rmse: float
    r_squared: float
    n_samples: int
    valid: bool
    converged: bool
    reason: str | None = None
    bounds: dict[str, tuple[float, float]] = field(default_factory=dict)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def pose_step_response(t_rel: np.ndarray, K: float, tau: float, L: float, amp: float) -> np.ndarray:
    """Position of a from-rest FOPDT step, ``t_rel`` relative to the step edge.

    Closed-form integral of the FOPDT velocity step - no differentiation, no
    discretization. Excludes ``p0``/``drift`` (added by the fit)."""
    t_rel = np.asarray(t_rel, dtype=float)
    out = np.zeros_like(t_rel)
    if tau <= 0.0:
        return out
    mask = t_rel >= L
    s = t_rel[mask] - L
    out[mask] = K * amp * (s - tau * (1.0 - np.exp(-s / tau)))
    return out


def _nan_result(n: int, reason: str, l_fixed: float) -> PoseFopdtParams:
    nan = float("nan")
    return PoseFopdtParams(
        K=nan,
        tau=nan,
        L=l_fixed,
        p0=nan,
        drift=nan,
        rmse=nan,
        r_squared=nan,
        n_samples=n,
        valid=False,
        converged=False,
        reason=reason,
    )


def fit_pose_fopdt(
    t_rel: np.ndarray,
    p_meas: np.ndarray,
    amp: float,
    l_fixed: float,
    *,
    tau_bounds: tuple[float, float] = DEFAULT_TAU_BOUNDS,
    l_bounds: tuple[float, float] = DEFAULT_L_BOUNDS,
) -> PoseFopdtParams:
    """Fit (K, tau, p0, drift) to a pose step segment with L held at ``l_fixed``.

    ``t_rel`` is time relative to the step edge; ``p_meas`` is the measured pose
    channel (body-x displacement for a vx step, yaw for a wz step); ``amp`` is
    the signed commanded amplitude. Least-squares on the pose residual."""
    from scipy.optimize import least_squares

    t_rel = np.asarray(t_rel, dtype=float)
    p_meas = np.asarray(p_meas, dtype=float)
    n = int(t_rel.size)
    if n < 4:
        return _nan_result(n, "fewer than 4 samples in segment", l_fixed)
    if abs(amp) < 1e-9:
        return _nan_result(n, "amp is zero - cannot identify K", l_fixed)

    span = float(p_meas[-1] - p_meas[0])
    duration = float(t_rel[-1] - t_rel[0]) or 1.0
    k_guess = float(np.clip(span / (amp * duration) if duration else 1.0, -K_ABS_MAX, K_ABS_MAX))
    tau_guess = float(np.clip(0.2, *tau_bounds))
    p0_guess = float(p_meas[0])

    def residual(params: np.ndarray) -> np.ndarray:
        k, tau, p0, drift = params
        pred = p0 + drift * t_rel + pose_step_response(t_rel, k, tau, l_fixed, amp)
        return np.asarray(pred - p_meas, dtype=float)

    lo = [-K_ABS_MAX, tau_bounds[0], -np.inf, -np.inf]
    hi = [K_ABS_MAX, tau_bounds[1], np.inf, np.inf]
    try:
        sol = least_squares(
            residual, x0=[k_guess, tau_guess, p0_guess, 0.0], bounds=(lo, hi), max_nfev=5000
        )
    except Exception as e:
        return _nan_result(n, f"least_squares failed: {type(e).__name__}: {e}", l_fixed)

    k, tau, p0, drift = (float(v) for v in sol.x)
    resid = sol.fun
    rmse = float(np.sqrt(np.mean(resid**2)))
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((p_meas - float(np.mean(p_meas))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    valid = (
        bool(sol.success)
        and tau_bounds[0] * 1.001 <= tau <= tau_bounds[1] * 0.999
        and l_bounds[0] <= l_fixed <= l_bounds[1]
        and abs(k) < K_ABS_MAX * 0.999
    )
    return PoseFopdtParams(
        K=k,
        tau=tau,
        L=l_fixed,
        p0=p0,
        drift=drift,
        rmse=rmse,
        r_squared=r2,
        n_samples=n,
        valid=valid,
        converged=bool(sol.success),
        reason=None if valid else "fit outside physical plausibility bounds",
        bounds={"tau": tau_bounds, "L": l_bounds},
    )


def fit_pose_fopdt_multi(
    segments: list[tuple[np.ndarray, np.ndarray, float]],
    l_fixed: float,
    *,
    tau_bounds: tuple[float, float] = DEFAULT_TAU_BOUNDS,
    l_bounds: tuple[float, float] = DEFAULT_L_BOUNDS,
) -> PoseFopdtParams:
    """Joint fit of one axis across several amplitudes: shared (K, tau, drift).

    A single step can't separate the steady-state slope (K*amp) from a constant
    drift slope - they're collinear - which biases K low. Fitting several
    amplitudes together with a SHARED drift but amplitude-scaled response breaks
    that collinearity. Each segment keeps its own ``p0``. ``segments`` is a list
    of ``(t_rel, p_meas, amp)``."""
    from scipy.optimize import least_squares

    segs = [
        (np.asarray(t, dtype=float), np.asarray(p, dtype=float), float(amp))
        for t, p, amp in segments
        if np.asarray(t).size >= 4 and abs(amp) > 1e-9
    ]
    n_total = sum(t.size for t, _, _ in segs)
    if not segs:
        return _nan_result(n_total, "no usable segments for joint fit", l_fixed)

    n_seg = len(segs)
    spans = [float(p[-1] - p[0]) for _, p, _ in segs]
    durations = [float(t[-1] - t[0]) or 1.0 for t, _, _ in segs]
    k_guesses = [s / (a * d) for s, (_, _, a), d in zip(spans, segs, durations, strict=True)]
    k_guess = float(np.clip(np.median(k_guesses), -K_ABS_MAX, K_ABS_MAX))
    tau_guess = float(np.clip(0.2, *tau_bounds))
    p0_guesses = [float(p[0]) for _, p, _ in segs]

    def residual(x: np.ndarray) -> np.ndarray:
        k, tau, drift = x[0], x[1], x[2]
        p0s = x[3:]
        parts = []
        for (t, p, amp), p0 in zip(segs, p0s, strict=True):
            pred = p0 + drift * t + pose_step_response(t, k, tau, l_fixed, amp)
            parts.append(pred - p)
        return np.concatenate(parts)

    lo = [-K_ABS_MAX, tau_bounds[0], -np.inf, *([-np.inf] * n_seg)]
    hi = [K_ABS_MAX, tau_bounds[1], np.inf, *([np.inf] * n_seg)]
    x0 = [k_guess, tau_guess, 0.0, *p0_guesses]
    try:
        sol = least_squares(residual, x0=x0, bounds=(lo, hi), max_nfev=5000)
    except Exception as e:
        return _nan_result(n_total, f"joint least_squares failed: {type(e).__name__}: {e}", l_fixed)

    k, tau, drift = float(sol.x[0]), float(sol.x[1]), float(sol.x[2])
    resid = sol.fun
    rmse = float(np.sqrt(np.mean(resid**2)))
    p_all = np.concatenate([p for _, p, _ in segs])
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((p_all - float(np.mean(p_all))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    valid = (
        bool(sol.success)
        and tau_bounds[0] * 1.001 <= tau <= tau_bounds[1] * 0.999
        and l_bounds[0] <= l_fixed <= l_bounds[1]
        and abs(k) < K_ABS_MAX * 0.999
    )
    return PoseFopdtParams(
        K=k,
        tau=tau,
        L=l_fixed,
        p0=float(sol.x[3]),
        drift=drift,
        rmse=rmse,
        r_squared=r2,
        n_samples=n_total,
        valid=valid,
        converged=bool(sol.success),
        reason=None if valid else "fit outside physical plausibility bounds",
        bounds={"tau": tau_bounds, "L": l_bounds},
    )


def _parabolic_vertex(x: np.ndarray, y: np.ndarray, i: int) -> float:
    """Sub-grid minimum of ``y`` near index ``i`` by 3-point parabola fit."""
    if i == 0 or i == x.size - 1:
        return float(x[i])
    y0, y1, y2 = y[i - 1], y[i], y[i + 1]
    denom = y0 - 2.0 * y1 + y2
    if denom <= 0.0:
        return float(x[i])
    offset = 0.5 * (y0 - y2) / denom
    return float(x[i] + offset * (x[i + 1] - x[i]))


def estimate_deadtime(
    t_rel: np.ndarray,
    p_meas: np.ndarray,
    amp: float,
    *,
    l_candidates: np.ndarray | None = None,
    l_bounds: tuple[float, float] = DEFAULT_L_BOUNDS,
    tau_bounds: tuple[float, float] = DEFAULT_TAU_BOUNDS,
    grid_dt: float = (1.0 / 18.0) / 4.0,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Estimate deadtime L by profiling, decoupled from tau.

    For each candidate L, fit (K, tau, p0, drift) on the pose and record RMSE;
    the L with the lowest RMSE (refined by a parabolic vertex) is returned. This
    is the profile-likelihood approach: L is chosen on an outer 1-D search rather
    than co-fit in the same gradient step as tau, which is degenerate.

    Returns ``(best_L, l_candidates, rmse_per_candidate)``."""
    if l_candidates is None:
        n_steps = max(3, round((l_bounds[1] - l_bounds[0]) / grid_dt) + 1)
        l_candidates = np.linspace(l_bounds[0], l_bounds[1], n_steps)
    rmses = np.full(l_candidates.size, np.inf)
    for i, candidate in enumerate(l_candidates):
        fit = fit_pose_fopdt(
            t_rel, p_meas, amp, float(candidate), tau_bounds=tau_bounds, l_bounds=l_bounds
        )
        if fit.converged and np.isfinite(fit.rmse):
            rmses[i] = fit.rmse
    best_i = int(np.argmin(rmses))
    best_l = _parabolic_vertex(l_candidates, rmses, best_i)
    best_l = float(np.clip(best_l, l_bounds[0], l_bounds[1]))
    return best_l, l_candidates, rmses


__all__ = [
    "PoseFopdtParams",
    "estimate_deadtime",
    "fit_pose_fopdt",
    "fit_pose_fopdt_multi",
    "pose_step_response",
]
