# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Closed-loop sim: PI controller + FOPDT plant.

The plant model is Session 2's ``simulate_fopdt`` — we don't reimplement
the FOPDT math, we wrap it. At each control tick the controller computes
``u`` from ``r - y``, the plant simulator extends its output one step
based on the entire ``u`` history (deadtime handled by the plant).

Anti-windup is back-calculation: when the saturated output differs from
the unsaturated demand, the integrator is bled by ``(u_sat - u_raw) *
Kt * dt``. With ``Kt ≈ 1/τ`` (rule of thumb), saturation excursions
unwind on the same timescale as the plant.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from dimos.utils.characterization.modeling.fopdt import FopdtParams
from dimos.utils.characterization.modeling.simulate import simulate_fopdt


@dataclass
class PIController:
    """PI controller with output saturation + back-calculation anti-windup.

    ``Kt`` is the anti-windup gain (typically ``1/τ`` for an FOPDT plant).
    ``u_min`` / ``u_max`` are absolute bounds on the actuator output.
    """
    Kp: float
    Ki: float
    Kt: float = 0.0
    u_min: float = -np.inf
    u_max: float = np.inf
    integrator: float = field(default=0.0)

    def reset(self) -> None:
        self.integrator = 0.0

    def step(self, ref: float, meas: float, dt: float) -> tuple[float, float]:
        """Advance one control tick.

        Returns ``(u_raw, u_sat)`` so the caller can log both — saturation
        events show up as ``u_raw != u_sat``.
        """
        e = ref - meas
        u_raw = self.Kp * e + self.Ki * self.integrator
        u_sat = float(np.clip(u_raw, self.u_min, self.u_max))
        # Standard back-calculation: when not saturated the correction is
        # zero (u_sat == u_raw); when saturated, integrator gets bled.
        self.integrator += (e + self.Kt * (u_sat - u_raw)) * dt
        return float(u_raw), u_sat


@dataclass
class SimResult:
    t: np.ndarray
    r: np.ndarray
    u: np.ndarray
    u_raw: np.ndarray              # pre-saturation demand
    y: np.ndarray
    metrics: dict[str, float] = field(default_factory=dict)
    plant: dict[str, float] = field(default_factory=dict)
    controller: dict[str, float] = field(default_factory=dict)

    def asdict(self) -> dict[str, Any]:
        return {
            "t": self.t.tolist(),
            "r": self.r.tolist(),
            "u": self.u.tolist(),
            "u_raw": self.u_raw.tolist(),
            "y": self.y.tolist(),
            "metrics": self.metrics,
            "plant": self.plant,
            "controller": self.controller,
        }


def _compute_metrics(
    t: np.ndarray, r: np.ndarray, u: np.ndarray, u_raw: np.ndarray, y: np.ndarray,
    *, settle_band: float = 0.02,
) -> dict[str, float]:
    """Standard step-response metrics + closed-loop control effort.

    Settle time is computed against the *final* reference value, which
    is what matters for staircase / single-step inputs.
    """
    if t.size < 2:
        return {}
    e = r - y
    dt = float(np.mean(np.diff(t)))
    iae = float(np.sum(np.abs(e)) * dt)
    itae = float(np.sum(t * np.abs(e)) * dt)
    rmse = float(np.sqrt(np.mean(e ** 2)))
    max_u = float(np.max(np.abs(u)))
    saturation_fraction = float(np.mean(np.abs(u_raw - u) > 1e-9))

    # Overshoot + settle time relative to the final reference value.
    r_final = float(r[-1])
    if abs(r_final) > 1e-9 and r_final > 0:
        overshoot = float(max(0.0, np.max(y) - r_final) / abs(r_final))
    elif abs(r_final) > 1e-9 and r_final < 0:
        overshoot = float(max(0.0, abs(np.min(y)) - abs(r_final)) / abs(r_final))
    else:
        overshoot = float("nan")

    if abs(r_final) > 1e-9:
        outside = np.abs(y - r_final) > settle_band * abs(r_final)
        if outside.any():
            last_outside = int(np.where(outside)[0][-1])
            settle_time = float(t[last_outside]) if last_outside + 1 < t.size else float("inf")
        else:
            settle_time = 0.0
    else:
        settle_time = float("nan")

    return {
        "iae": iae,
        "itae": itae,
        "rmse": rmse,
        "overshoot": overshoot,
        "settle_time_s": settle_time,
        "max_abs_u": max_u,
        "saturation_fraction": saturation_fraction,
    }


def simulate_closed_loop(
    plant: FopdtParams,
    controller: PIController,
    reference_fn: Callable[[float], float],
    *,
    duration_s: float,
    control_dt_s: float = 0.02,
    initial_y: float = 0.0,
    settle_band: float = 0.02,
) -> SimResult:
    """Simulate a closed-loop response.

    Parameters
    ----------
    plant : FopdtParams
        FOPDT plant parameters. Used as the model fed to ``simulate_fopdt``.
    controller : PIController
        Reset before the run; integrator state will be modified.
    reference_fn : Callable[[float], float]
        ``reference_fn(t) -> r(t)``. See ``references.py`` for builders.
    duration_s : float
        Total simulation horizon.
    control_dt_s : float, default 0.02
        Control / discretization period (50 Hz default — matches harness).
    initial_y : float, default 0.0
        Initial plant output. Pass nonzero when starting from a non-rest
        operating point.
    settle_band : float, default 0.02
        ±2% band used by the settle-time metric.
    """
    if duration_s <= 0 or control_dt_s <= 0:
        raise ValueError("duration_s and control_dt_s must be positive")

    n = int(np.floor(duration_s / control_dt_s)) + 1
    t = np.arange(n, dtype=float) * control_dt_s
    r = np.array([float(reference_fn(float(ti))) for ti in t])
    u = np.zeros(n)
    u_raw = np.zeros(n)
    y = np.zeros(n)
    y[0] = initial_y

    controller.reset()

    # Step k uses y[k-1] (the latest measurement available at time t[k-1])
    # to compute u[k]. The plant simulator then advances y[k] using the
    # full u[:k+1] history (with the FOPDT deadtime handled internally).
    for k in range(1, n):
        ur, us = controller.step(float(r[k - 1]), float(y[k - 1]), control_dt_s)
        u_raw[k] = ur
        u[k] = us
        y_traj = simulate_fopdt(t[: k + 1], u[: k + 1], plant, initial=initial_y)
        y[k] = float(y_traj[k])

    metrics = _compute_metrics(t, r, u, u_raw, y, settle_band=settle_band)
    return SimResult(
        t=t, r=r, u=u, u_raw=u_raw, y=y,
        metrics=metrics,
        plant={"K": plant.K, "tau": plant.tau, "L": plant.L},
        controller={
            "Kp": controller.Kp, "Ki": controller.Ki, "Kt": controller.Kt,
            "u_min": controller.u_min, "u_max": controller.u_max,
        },
    )


__all__ = ["PIController", "SimResult", "simulate_closed_loop"]
