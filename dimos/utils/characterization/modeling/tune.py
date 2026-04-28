# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Lambda-tuning recipe + multiplier sweep.

Lambda tuning (SIMC / IMC for FOPDT plants) gives PI gains as a closed
form of (K, τ, L) plus a designer choice ``λ`` (closed-loop time
constant).

  λ  = max(τ, L) * multiplier      # multiplier ∈ [1, 3] in this module
  Kp = τ / (K * (λ + L))
  Ki = Kp / τ
  Kt = 1 / τ                       # back-calculation anti-windup gain

``multiplier=1`` is aggressive; ``multiplier=3`` is conservative. The
sweep runs each candidate through a battery of reference signals,
scores via IAE + overshoot + saturation penalties, and picks the lowest
total cost.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Callable

import numpy as np

from dimos.utils.characterization.modeling.closed_loop import (
    PIController, SimResult, simulate_closed_loop,
)
from dimos.utils.characterization.modeling.fopdt import FopdtParams
from dimos.utils.characterization.modeling import references as refs


@dataclass
class Gains:
    Kp: float
    Ki: float
    Kt: float
    multiplier: float
    lambda_s: float

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TuningCandidate:
    gains: Gains
    per_reference: list[dict[str, Any]]   # one entry per scenario
    cost: float                           # weighted sum across references
    cost_breakdown: dict[str, float]      # average IAE, overshoot, sat across refs

    def asdict(self) -> dict[str, Any]:
        return {
            "gains": self.gains.asdict(),
            "per_reference": self.per_reference,
            "cost": self.cost,
            "cost_breakdown": self.cost_breakdown,
        }


@dataclass
class TuningResult:
    plant: dict[str, float]
    saturation_limits: tuple[float, float]
    multipliers: list[float]
    candidates: list[TuningCandidate]
    best_index: int

    @property
    def best(self) -> TuningCandidate:
        return self.candidates[self.best_index]

    def asdict(self) -> dict[str, Any]:
        return {
            "plant": self.plant,
            "saturation_limits": list(self.saturation_limits),
            "multipliers": self.multipliers,
            "best_index": self.best_index,
            "best_gains": self.best.gains.asdict(),
            "candidates": [c.asdict() for c in self.candidates],
        }


# --------------------------------------------------------------------------- recipe

def lambda_tune(
    K: float, tau: float, L: float, multiplier: float = 1.0,
) -> Gains:
    """Standard lambda-tuning for FOPDT → PI."""
    lam = max(tau, L) * multiplier
    Kp = tau / (K * (lam + L))
    Ki = Kp / tau
    Kt = 1.0 / tau
    return Gains(Kp=Kp, Ki=Ki, Kt=Kt, multiplier=multiplier, lambda_s=lam)


# --------------------------------------------------------------------------- scoring

# Cost weights. Tuned so each term contributes roughly the same order of
# magnitude on a typical step response.
_W_IAE = 1.0
_W_OVERSHOOT = 2.0          # 1.0 overshoot fraction adds 2 units of cost
_W_SATURATION = 0.5         # 100% saturated time adds 0.5 units of cost
_W_SETTLE = 0.2             # 1 s of settle time adds 0.2 units of cost


def _scenario_cost(metrics: dict[str, float]) -> float:
    """Per-scenario cost. Higher = worse."""
    iae = float(metrics.get("iae", 0.0))
    over = float(metrics.get("overshoot", 0.0))
    if not np.isfinite(over):
        over = 0.0
    sat = float(metrics.get("saturation_fraction", 0.0))
    settle = float(metrics.get("settle_time_s", 0.0))
    if not np.isfinite(settle):
        # Ran out of horizon without settling — penalise heavily.
        settle = 5.0
    return (
        _W_IAE * iae
        + _W_OVERSHOOT * over
        + _W_SATURATION * sat
        + _W_SETTLE * settle
    )


# --------------------------------------------------------------------------- scenarios

def default_scenarios(
    saturation_limits: tuple[float, float],
) -> list[tuple[str, Callable[[float], float], float]]:
    """Battery of reference signals to score each candidate against.

    Returns (label, reference_fn, duration_s). Amplitudes scale to the
    actuator range so the same battery works for vx (~1.0 m/s) and wz
    (~1.5 rad/s) without manual tweaks.
    """
    u_max = float(saturation_limits[1])
    return [
        # Modest step (50% of saturation): clean transient.
        ("step_50pct", refs.step(amplitude=0.5 * u_max, t_start=0.5), 4.0),
        # Aggressive step (90% of saturation): probes saturation handling.
        ("step_90pct", refs.step(amplitude=0.9 * u_max, t_start=0.5), 4.0),
        # Reverse step.
        ("step_neg_70pct", refs.step(amplitude=-0.7 * u_max, t_start=0.5), 4.0),
        # Staircase (forward, then back). Tests sequential setpoints.
        ("staircase", refs.staircase(
            amplitudes=[0.3 * u_max, 0.6 * u_max, 0.9 * u_max,
                        0.3 * u_max, 0.0],
            dwell_s=1.5, t_start=0.5,
        ), 8.5),
        # Ramp: tests slow-tracking behaviour without harsh transients.
        ("ramp", refs.ramp(slope=0.5 * u_max, duration=1.5, t_start=0.5), 4.0),
    ]


# --------------------------------------------------------------------------- tune

def tune_channel(
    plant: FopdtParams,
    *,
    saturation_limits: tuple[float, float],
    multiplier_range: tuple[float, float] = (1.0, 3.0),
    n_multipliers: int = 5,
    control_dt_s: float = 0.02,
    scenarios: list[tuple[str, Callable[[float], float], float]] | None = None,
) -> TuningResult:
    """Sweep λ-multiplier in ``multiplier_range``, score each candidate
    against the scenario battery, return the winner + full table."""
    if scenarios is None:
        scenarios = default_scenarios(saturation_limits)

    multipliers = list(np.linspace(
        multiplier_range[0], multiplier_range[1], n_multipliers,
    ))

    candidates: list[TuningCandidate] = []
    for m in multipliers:
        gains = lambda_tune(plant.K, plant.tau, plant.L, multiplier=float(m))
        per_ref: list[dict[str, Any]] = []
        cost_sum = 0.0
        breakdown_sum = {"iae": 0.0, "overshoot": 0.0,
                         "saturation_fraction": 0.0, "settle_time_s": 0.0}
        for label, ref_fn, dur in scenarios:
            ctrl = PIController(
                Kp=gains.Kp, Ki=gains.Ki, Kt=gains.Kt,
                u_min=saturation_limits[0], u_max=saturation_limits[1],
            )
            sim = simulate_closed_loop(
                plant, ctrl, ref_fn, duration_s=dur, control_dt_s=control_dt_s,
            )
            sc_cost = _scenario_cost(sim.metrics)
            per_ref.append({
                "scenario": label,
                "duration_s": dur,
                "metrics": sim.metrics,
                "cost": sc_cost,
            })
            cost_sum += sc_cost
            for k in breakdown_sum:
                v = float(sim.metrics.get(k, 0.0))
                if np.isfinite(v):
                    breakdown_sum[k] += v
        n_scenarios = max(1, len(scenarios))
        breakdown = {k: v / n_scenarios for k, v in breakdown_sum.items()}
        candidates.append(TuningCandidate(
            gains=gains, per_reference=per_ref,
            cost=cost_sum, cost_breakdown=breakdown,
        ))

    best_index = int(np.argmin([c.cost for c in candidates]))
    return TuningResult(
        plant={"K": plant.K, "tau": plant.tau, "L": plant.L},
        saturation_limits=saturation_limits,
        multipliers=multipliers,
        candidates=candidates,
        best_index=best_index,
    )


__all__ = [
    "Gains", "TuningCandidate", "TuningResult",
    "default_scenarios", "lambda_tune", "tune_channel",
]
