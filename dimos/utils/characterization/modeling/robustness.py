# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Robustness sweeps — does the controller survive plant uncertainty?

Two checks before deploy:

  - **gain_sweep**: hold the controller fixed, sweep K across the
    run-to-run variance band Session 2 measured (default ±15%). For
    every K, run the closed-loop sim and confirm tracking error stays
    bounded and the loop stays stable.
  - **param_sweep**: same shape but for τ and L (model uncertainty
    rather than plant variance).

A controller passes if every point in the sweep settles within the
horizon, doesn't go unstable (no diverging y), and stays inside the
saturation budget.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Callable

import numpy as np

from dimos.utils.characterization.modeling.closed_loop import (
    PIController, simulate_closed_loop,
)
from dimos.utils.characterization.modeling.fopdt import FopdtParams


@dataclass
class SweepPoint:
    label: str                 # e.g. "K=0.85*nominal" or "tau=1.20*nominal"
    plant: dict[str, float]
    metrics: dict[str, float]
    stable: bool
    saturation_fraction: float

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SweepResult:
    swept: str                 # "K", "tau", "L"
    nominal: dict[str, float]
    points: list[SweepPoint] = field(default_factory=list)
    all_stable: bool = True
    worst_iae: float = 0.0

    def asdict(self) -> dict[str, Any]:
        return {
            "swept": self.swept,
            "nominal": self.nominal,
            "all_stable": self.all_stable,
            "worst_iae": self.worst_iae,
            "points": [p.asdict() for p in self.points],
        }


def _is_stable(y: np.ndarray, max_y: float) -> bool:
    """Stable if y stays bounded within ±10× the saturation magnitude.

    This catches obvious instability (oscillation that diverges or NaN)
    without flagging legitimate transients.
    """
    if not np.all(np.isfinite(y)):
        return False
    bound = max(10.0 * max_y, 1.0)
    return bool(np.max(np.abs(y)) < bound)


def _scaled_plant(nominal: FopdtParams, *, which: str, factor: float) -> FopdtParams:
    K, tau, L = nominal.K, nominal.tau, nominal.L
    if which == "K":
        K = K * factor
    elif which == "tau":
        tau = tau * factor
    elif which == "L":
        L = L * factor
    else:
        raise ValueError(f"unknown parameter to scale: {which!r}")
    return FopdtParams(
        K=K, tau=tau, L=L,
        K_ci=(K, K), tau_ci=(tau, tau), L_ci=(L, L),
        rmse=0.0, r_squared=1.0, n_samples=0,
        fit_window_s=(0.0, 0.0),
        degenerate=False, converged=True,
    )


def gain_sweep(
    plant: FopdtParams,
    controller: PIController,
    reference_fn: Callable[[float], float],
    *,
    k_range: tuple[float, float] = (0.85, 1.15),
    n: int = 7,
    duration_s: float = 4.0,
    control_dt_s: float = 0.02,
) -> SweepResult:
    """Sweep K across ``[k_range[0]*K_nom, k_range[1]*K_nom]``.

    Use the run-to-run variance band Session 2 measured as the default
    (±15%). The controller is held fixed; only the plant's K changes.
    """
    factors = list(np.linspace(k_range[0], k_range[1], n))
    sat_max = max(abs(controller.u_min), abs(controller.u_max))
    nominal = {"K": plant.K, "tau": plant.tau, "L": plant.L}
    result = SweepResult(swept="K", nominal=nominal)
    for f in factors:
        scaled = _scaled_plant(plant, which="K", factor=float(f))
        # Reset controller integrator between sweeps.
        controller.reset()
        sim = simulate_closed_loop(
            scaled, controller, reference_fn,
            duration_s=duration_s, control_dt_s=control_dt_s,
        )
        stable = _is_stable(sim.y, max_y=sat_max)
        sat_frac = float(sim.metrics.get("saturation_fraction", 0.0))
        result.points.append(SweepPoint(
            label=f"K={f:.2f}*nom",
            plant={"K": scaled.K, "tau": scaled.tau, "L": scaled.L},
            metrics=sim.metrics,
            stable=stable,
            saturation_fraction=sat_frac,
        ))
        if not stable:
            result.all_stable = False
        iae = float(sim.metrics.get("iae", 0.0))
        if np.isfinite(iae) and iae > result.worst_iae:
            result.worst_iae = iae
    return result


def param_sweep(
    plant: FopdtParams,
    controller: PIController,
    reference_fn: Callable[[float], float],
    *,
    which: str,
    factor_range: tuple[float, float] = (0.80, 1.20),
    n: int = 7,
    duration_s: float = 4.0,
    control_dt_s: float = 0.02,
) -> SweepResult:
    """Sweep τ or L over ``factor_range`` to cover model uncertainty."""
    if which not in ("tau", "L"):
        raise ValueError(f"param_sweep accepts 'tau' or 'L', got {which!r}")
    factors = list(np.linspace(factor_range[0], factor_range[1], n))
    sat_max = max(abs(controller.u_min), abs(controller.u_max))
    nominal = {"K": plant.K, "tau": plant.tau, "L": plant.L}
    result = SweepResult(swept=which, nominal=nominal)
    for f in factors:
        scaled = _scaled_plant(plant, which=which, factor=float(f))
        controller.reset()
        sim = simulate_closed_loop(
            scaled, controller, reference_fn,
            duration_s=duration_s, control_dt_s=control_dt_s,
        )
        stable = _is_stable(sim.y, max_y=sat_max)
        result.points.append(SweepPoint(
            label=f"{which}={f:.2f}*nom",
            plant={"K": scaled.K, "tau": scaled.tau, "L": scaled.L},
            metrics=sim.metrics,
            stable=stable,
            saturation_fraction=float(sim.metrics.get("saturation_fraction", 0.0)),
        ))
        if not stable:
            result.all_stable = False
        iae = float(sim.metrics.get("iae", 0.0))
        if np.isfinite(iae) and iae > result.worst_iae:
            result.worst_iae = iae
    return result


__all__ = ["SweepPoint", "SweepResult", "gain_sweep", "param_sweep"]
