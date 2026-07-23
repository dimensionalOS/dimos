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

"""Velocity-domain FOPDT fitter.

Fits the FOPDT step response directly to a (reconstructed) body-velocity trace.
Ported from the characterization modeling harness; the bounds are exposed as
parameters rather than baked-in Go2 constants so the fitter is robot-agnostic.

  K   - steady-state gain (output per unit command)
  tau - first-order time constant (s)
  L   - lumped dead-time (s)
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

# Generic plausibility bounds. Override per robot via fit_fopdt(..., bounds=...).
K_ABS_MAX = 5.0
TAU_MIN = 1e-3
TAU_MAX = 5.0
L_MIN = 0.0
L_MAX = 1.0


@dataclass
class FopdtParams:
    """Result of a single velocity-domain FOPDT fit. ``converged=False`` means
    the optimizer failed or the input was degenerate; numeric fields are then
    NaN and ``reason`` explains why. ``plausible`` is the gate: a converged fit
    that lands on a bound is flagged rather than silently trusted."""

    K: float
    tau: float
    L: float
    K_ci: tuple[float, float]
    tau_ci: tuple[float, float]
    L_ci: tuple[float, float]
    rmse: float
    r_squared: float
    n_samples: int
    fit_window_s: tuple[float, float]
    degenerate: bool
    converged: bool
    plausible: bool = True
    reason: str | None = None
    initial_guess: dict[str, float] = field(default_factory=dict)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def fopdt_step_response(t: np.ndarray, K: float, tau: float, L: float, u_step: float) -> np.ndarray:
    """Vectorized FOPDT step response. ``t`` is time relative to the step edge."""
    t = np.asarray(t, dtype=float)
    out = np.zeros_like(t)
    if tau <= 0.0:
        return out
    mask = t >= L
    out[mask] = K * u_step * (1.0 - np.exp(-(t[mask] - L) / tau))
    return out


def _nan_params(n: int, window: tuple[float, float], reason: str) -> FopdtParams:
    nan = float("nan")
    return FopdtParams(
        K=nan,
        tau=nan,
        L=nan,
        K_ci=(nan, nan),
        tau_ci=(nan, nan),
        L_ci=(nan, nan),
        rmse=nan,
        r_squared=nan,
        n_samples=n,
        fit_window_s=window,
        degenerate=True,
        converged=False,
        plausible=False,
        reason=reason,
    )


def _initial_guess(
    t: np.ndarray,
    y: np.ndarray,
    u_step: float,
    noise_std: float | None,
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> tuple[float, float, float]:
    (_, tau_min, l_min), (k_abs, tau_max, l_max) = bounds
    if t.size < 4:
        return (1.0, 0.2, 0.05)
    n = t.size
    tail_n = max(1, int(n * 0.2))
    y_tail = float(np.mean(y[-tail_n:]))
    K_init = y_tail / u_step if abs(u_step) > 1e-9 else 1.0
    K_init = float(np.clip(K_init, -k_abs * 0.99, k_abs * 0.99))
    if abs(K_init) < 1e-3:
        K_init = 0.5 if u_step >= 0 else -0.5

    band = 3.0 * (noise_std if noise_std and noise_std > 0 else 1e-3)
    above = np.flatnonzero(np.abs(y) > band)
    L_init = float(t[above[0]]) if above.size else 0.05
    L_init = float(np.clip(L_init, l_min, l_max * 0.99))

    target = 0.63 * K_init * u_step
    if abs(target) > 1e-6:
        crossed = (
            np.flatnonzero(y >= target) if K_init * u_step > 0 else np.flatnonzero(y <= target)
        )
        tau_init = float(t[crossed[0]] - L_init) if crossed.size else 0.3
    else:
        tau_init = 0.3
    tau_init = float(np.clip(tau_init, tau_min * 10, tau_max * 0.99))
    return (K_init, tau_init, L_init)


def fit_fopdt(
    t: np.ndarray,
    y: np.ndarray,
    u_step: float,
    *,
    noise_std: float | None = None,
    fit_window_s: tuple[float, float] | None = None,
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
) -> FopdtParams:
    """Fit FOPDT to a step-response trace.

    ``t`` is time relative to the step edge (step at t=0). ``y`` is the measured
    response with the pre-step baseline subtracted. ``u_step`` is the signed
    commanded amplitude. ``bounds`` is ``((lo_K, lo_tau, lo_L), (hi_K, hi_tau,
    hi_L))``; defaults to the generic plausibility box.
    """
    from scipy.optimize import curve_fit

    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    window = (
        fit_window_s
        if fit_window_s is not None
        else ((float(t[0]), float(t[-1])) if t.size else (0.0, 0.0))
    )
    if bounds is None:
        bounds = ((-K_ABS_MAX, TAU_MIN, L_MIN), (K_ABS_MAX, TAU_MAX, L_MAX))
    lo, hi = bounds

    if t.size < 4:
        return _nan_params(int(t.size), window, "fewer than 4 samples in fit window")
    if abs(u_step) < 1e-9:
        return _nan_params(int(t.size), window, "u_step is zero - cannot identify K")

    K0, tau0, L0 = _initial_guess(t, y, u_step, noise_std, bounds)
    sigma = np.full_like(y, float(noise_std)) if noise_std and noise_std > 0 else None

    def _model(t_, K, tau, L):
        return fopdt_step_response(t_, K, tau, L, u_step)

    try:
        popt, pcov = curve_fit(
            _model,
            t,
            y,
            p0=(K0, tau0, L0),
            bounds=(lo, hi),
            sigma=sigma,
            absolute_sigma=False,
            maxfev=5000,
        )
    except Exception as e:
        out = _nan_params(int(t.size), window, f"curve_fit failed: {type(e).__name__}: {e}")
        out.initial_guess = {"K": K0, "tau": tau0, "L": L0}
        return out

    K, tau, L = float(popt[0]), float(popt[1]), float(popt[2])
    y_hat = _model(t, K, tau, L)
    resid = y - y_hat
    rmse = float(np.sqrt(np.mean(resid**2))) if resid.size else float("nan")
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((y - float(np.mean(y))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    diag = np.diag(pcov)
    degenerate = bool(np.any(~np.isfinite(diag)) or np.any(diag <= 0))
    if degenerate:
        K_ci = tau_ci = L_ci = (float("nan"), float("nan"))
    else:
        s = np.sqrt(diag)
        K_ci = (K - 1.96 * float(s[0]), K + 1.96 * float(s[0]))
        tau_ci = (tau - 1.96 * float(s[1]), tau + 1.96 * float(s[1]))
        L_ci = (L - 1.96 * float(s[2]), L + 1.96 * float(s[2]))

    # Plausibility gate: flag fits that hit a bound (likely unidentified).
    on_bound = (
        abs(K) >= hi[0] * 0.999
        or tau <= lo[1] * 1.001
        or tau >= hi[1] * 0.999
        or L >= hi[2] * 0.999
    )
    return FopdtParams(
        K=K,
        tau=tau,
        L=L,
        K_ci=K_ci,
        tau_ci=tau_ci,
        L_ci=L_ci,
        rmse=rmse,
        r_squared=r2,
        n_samples=int(t.size),
        fit_window_s=window,
        degenerate=degenerate,
        converged=True,
        plausible=not on_bound,
        reason=None if not on_bound else "fit landed on a plausibility bound",
        initial_guess={"K": K0, "tau": tau0, "L": L0},
    )


__all__ = ["FopdtParams", "fit_fopdt", "fopdt_step_response"]
