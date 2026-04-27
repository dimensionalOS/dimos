# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""First-Order Plus Deadtime (FOPDT) model + fitter.

Step response (cmd 0 -> ``u_step`` at t=0, sample times in ``t``):

    y(t) = 0                                    for t < L
    y(t) = K * u_step * (1 - exp(-(t - L) / tau))   for t >= L

Three parameters per channel:

    K    steady-state gain
    tau  time constant (~63% of K * u_step reached at t = L + tau)
    L    deadtime / pure delay before any response begins

Fit uses ``scipy.optimize.curve_fit`` (Levenberg-Marquardt with bounds).
Initial guesses are derived from the trace itself (steady-state span,
time-to-63%, first-sample-above-noise-floor) — bad initial guesses send
the optimizer to bad local minima for nonlinear fits.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

# --- Bounds. Channel-aware bounds would be slightly tighter but these are
# generous enough to avoid clipping good fits while still preventing the
# optimizer from running off into the weeds.
_K_ABS_MAX = 5.0
_TAU_MIN = 1e-3
_TAU_MAX = 5.0
_L_MIN = 0.0
_L_MAX = 1.0


@dataclass
class FopdtParams:
    """Result of a single FOPDT fit. ``converged=False`` means the optimizer
    reported failure or the input was degenerate; in that case all numeric
    fields are NaN and ``reason`` explains why.
    """

    K: float
    tau: float
    L: float
    K_ci: tuple[float, float]      # 95% CI as (low, high); (nan, nan) if degenerate
    tau_ci: tuple[float, float]
    L_ci: tuple[float, float]
    rmse: float
    r_squared: float
    n_samples: int
    fit_window_s: tuple[float, float]   # (t_start, t_end) relative to step edge
    degenerate: bool                    # singular covariance => point estimates only
    converged: bool
    reason: str | None = None
    initial_guess: dict[str, float] = field(default_factory=dict)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def fopdt_step_response(
    t: np.ndarray, K: float, tau: float, L: float, u_step: float
) -> np.ndarray:
    """Vectorized FOPDT step response. ``t`` is time relative to step edge."""
    t = np.asarray(t, dtype=float)
    out = np.zeros_like(t)
    mask = t >= L
    if tau <= 0.0:
        return out
    out[mask] = K * u_step * (1.0 - np.exp(-(t[mask] - L) / tau))
    return out


def _initial_guess(
    t: np.ndarray, y: np.ndarray, u_step: float, noise_std: float | None
) -> tuple[float, float, float]:
    """Derive (K, tau, L) initial guesses from the data.

    K_init: steady-state span (mean of last 20% of post-step samples)
            divided by ``u_step``.
    L_init: first time the response leaves the noise band ``3 * noise_std``
            (or ``1e-3`` fallback). Pulled in slightly from where rise
            actually begins so curve_fit doesn't have to climb out of a
            zero-gradient region.
    tau_init: first time the response crosses ``0.63 * (K * u_step)``,
              minus L. Falls back to a sensible default if the trace
              never makes it that far.
    """
    if t.size < 4:
        # Caller should have rejected this; provide a non-zero guess so we
        # don't hit divide-by-zero before the fit fails cleanly.
        return (1.0, 0.2, 0.05)

    n = t.size
    tail_n = max(1, int(n * 0.2))
    y_tail = float(np.mean(y[-tail_n:]))
    K_init = y_tail / u_step if abs(u_step) > 1e-9 else 1.0
    K_init = float(np.clip(K_init, -_K_ABS_MAX * 0.99, _K_ABS_MAX * 0.99))
    if abs(K_init) < 1e-3:
        K_init = 0.5 if u_step >= 0 else -0.5

    band = 3.0 * (noise_std if noise_std and noise_std > 0 else 1e-3)
    above = np.flatnonzero(np.abs(y) > band)
    if above.size:
        L_init = float(t[above[0]])
    else:
        L_init = 0.05
    L_init = float(np.clip(L_init, _L_MIN, _L_MAX * 0.99))

    target = 0.63 * K_init * u_step
    if abs(target) > 1e-6:
        if K_init * u_step > 0:
            crossed = np.flatnonzero(y >= target)
        else:
            crossed = np.flatnonzero(y <= target)
        if crossed.size:
            tau_init = float(t[crossed[0]] - L_init)
        else:
            tau_init = 0.3
    else:
        tau_init = 0.3
    tau_init = float(np.clip(tau_init, _TAU_MIN * 10, _TAU_MAX * 0.99))

    return (K_init, tau_init, L_init)


def _bounds_for(u_step: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Bounds tuple ((lo_K, lo_tau, lo_L), (hi_K, hi_tau, hi_L)).

    K is unsigned-bounded: the fit recovers the signed gain from the data,
    independent of ``u_step``'s sign. Bounding K to one sign would
    actually rule out reasonable fits where the plant inverts.
    """
    return (
        (-_K_ABS_MAX, _TAU_MIN, _L_MIN),
        (_K_ABS_MAX, _TAU_MAX, _L_MAX),
    )


def fit_fopdt(
    t: np.ndarray,
    y: np.ndarray,
    u_step: float,
    *,
    noise_std: float | None = None,
    fit_window_s: tuple[float, float] | None = None,
) -> FopdtParams:
    """Fit FOPDT to a step-response trace.

    ``t`` is time relative to the step edge (so the step happens at t=0).
    ``y`` is the measured response with pre-step baseline already
    subtracted. ``u_step`` is the commanded step amplitude (signed).

    ``noise_std`` (optional) is per-sample sigma for weighted least
    squares. ``fit_window_s`` is recorded into the result for traceability.
    """
    from scipy.optimize import curve_fit

    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)

    fit_window = fit_window_s if fit_window_s is not None else (
        (float(t[0]), float(t[-1])) if t.size else (0.0, 0.0)
    )

    if t.size < 4:
        return FopdtParams(
            K=float("nan"), tau=float("nan"), L=float("nan"),
            K_ci=(float("nan"), float("nan")),
            tau_ci=(float("nan"), float("nan")),
            L_ci=(float("nan"), float("nan")),
            rmse=float("nan"), r_squared=float("nan"),
            n_samples=int(t.size),
            fit_window_s=fit_window,
            degenerate=True, converged=False,
            reason="fewer than 4 samples in fit window",
        )

    if abs(u_step) < 1e-9:
        return FopdtParams(
            K=float("nan"), tau=float("nan"), L=float("nan"),
            K_ci=(float("nan"), float("nan")),
            tau_ci=(float("nan"), float("nan")),
            L_ci=(float("nan"), float("nan")),
            rmse=float("nan"), r_squared=float("nan"),
            n_samples=int(t.size),
            fit_window_s=fit_window,
            degenerate=True, converged=False,
            reason="u_step is zero — cannot identify K",
        )

    K0, tau0, L0 = _initial_guess(t, y, u_step, noise_std)
    p0 = (K0, tau0, L0)
    lo, hi = _bounds_for(u_step)

    sigma = None
    if noise_std is not None and noise_std > 0:
        sigma = np.full_like(y, float(noise_std))

    def _model(t_, K, tau, L):
        return fopdt_step_response(t_, K, tau, L, u_step)

    try:
        popt, pcov = curve_fit(
            _model, t, y,
            p0=p0,
            bounds=(lo, hi),
            sigma=sigma,
            absolute_sigma=False,
            maxfev=5000,
        )
    except Exception as e:
        return FopdtParams(
            K=float("nan"), tau=float("nan"), L=float("nan"),
            K_ci=(float("nan"), float("nan")),
            tau_ci=(float("nan"), float("nan")),
            L_ci=(float("nan"), float("nan")),
            rmse=float("nan"), r_squared=float("nan"),
            n_samples=int(t.size),
            fit_window_s=fit_window,
            degenerate=True, converged=False,
            reason=f"curve_fit failed: {type(e).__name__}: {e}",
            initial_guess={"K": K0, "tau": tau0, "L": L0},
        )

    K, tau, L = (float(popt[0]), float(popt[1]), float(popt[2]))
    y_hat = _model(t, K, tau, L)
    resid = y - y_hat
    rmse = float(np.sqrt(np.mean(resid ** 2))) if resid.size else float("nan")
    ss_res = float(np.sum(resid ** 2))
    y_mean = float(np.mean(y))
    ss_tot = float(np.sum((y - y_mean) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    diag = np.diag(pcov)
    degenerate = bool(np.any(~np.isfinite(diag)) or np.any(diag <= 0))
    if degenerate:
        K_ci = (float("nan"), float("nan"))
        tau_ci = (float("nan"), float("nan"))
        L_ci = (float("nan"), float("nan"))
    else:
        sigmas = np.sqrt(diag)
        K_ci = (K - 1.96 * float(sigmas[0]), K + 1.96 * float(sigmas[0]))
        tau_ci = (tau - 1.96 * float(sigmas[1]), tau + 1.96 * float(sigmas[1]))
        L_ci = (L - 1.96 * float(sigmas[2]), L + 1.96 * float(sigmas[2]))

    return FopdtParams(
        K=K, tau=tau, L=L,
        K_ci=K_ci, tau_ci=tau_ci, L_ci=L_ci,
        rmse=rmse, r_squared=r2,
        n_samples=int(t.size),
        fit_window_s=fit_window,
        degenerate=degenerate,
        converged=True,
        reason=None,
        initial_guess={"K": K0, "tau": tau0, "L": L0},
    )


__all__ = ["FopdtParams", "fit_fopdt", "fopdt_step_response"]
