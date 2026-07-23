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

"""Lambda-tune PI gains from a fitted FOPDT, scored in closed-loop simulation.

Per channel: sweep a lambda (closed-loop time-constant) multiplier, compute PI
gains analytically for each, score every candidate against a reference battery
through a closed-loop FOPDT simulation, pick the lowest-cost candidate, then run
a robustness sweep (K +/-15%, tau/L +/-20%) and emit a pass/marginal/fail
verdict.

Ported from the characterization tuning harness and made robot-agnostic: all
Go2-specific constants (saturation limits, control rate, cost weights, verdict
thresholds) are parameters or named module constants, not baked-in values. The
dual-regime ("fall_params") FOPDT path is dropped - tuning never used it.

Lambda (IMC) PI tuning for a FOPDT plant:

    lambda_s = max(tau, L) * multiplier
    Kp = tau / (K * (lambda_s + L))
    Ki = Kp / tau
    Kt = 1 / tau                      # back-calculation anti-windup gain
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, field
import math
from typing import Any, Literal

import numpy as np

# Candidate-scoring cost weights (empirical; named so a robot with different
# priorities can rebalance). cost = w_iae*iae + w_over*overshoot
#                                 + w_sat*saturation_fraction + w_settle*settle.
W_IAE = 1.0
W_OVERSHOOT = 2.0
W_SATURATION = 0.5
W_SETTLE = 0.2
# Penalty substituted when settle time is non-finite (never settled in horizon).
SETTLE_PENALTY_S = 5.0

# Verdict thresholds (design margins; surfaced as constants, not hidden).
VERDICT_OVERSHOOT_MAX = 0.05
VERDICT_SATURATION_MAX = 0.50
# Stability bound: |y| must stay below max(STABILITY_GAIN*sat_max, STABILITY_FLOOR).
STABILITY_GAIN = 10.0
STABILITY_FLOOR = 1.0

DEFAULT_CONTROL_DT_S = 0.02  # 50 Hz; override from the robot's actual control tick
DEFAULT_SETTLE_BAND = 0.02

ReferenceFn = Callable[[float], float]
Verdict = Literal["pass", "marginal", "fail"]


# ── plant propagator (single-regime ZOH FOPDT) ─────────────────────────────


def _zoh_at(t_query: np.ndarray, t_grid: np.ndarray, values: np.ndarray, left: float) -> np.ndarray:
    """Zero-order-hold sample of ``values`` (on ``t_grid``) at ``t_query``. Times
    before ``t_grid[0]`` return ``left`` (the pre-trace value)."""
    idx = np.searchsorted(t_grid, t_query, side="right") - 1
    out = np.where(idx < 0, left, values[np.clip(idx, 0, values.size - 1)])
    return np.asarray(out, dtype=float)


def simulate_fopdt(
    t: np.ndarray,
    cmd: np.ndarray,
    K: float,
    tau: float,
    L: float,
    *,
    initial: float = 0.0,
    pre_cmd: float = 0.0,
) -> np.ndarray:
    """Forward-simulate a FOPDT plant for a commanded waveform.

        cmd_delayed(t) = cmd(t - L)            (zero-order hold, pre-trace = pre_cmd)
        target[k-1]    = K * cmd_delayed[k-1]
        alpha          = exp(-dt / tau)
        y[k]           = alpha*y[k-1] + (1-alpha)*target[k-1]

    Using ``target[k-1]`` (held over the previous interval) is the exact ZOH
    discretization of a first-order ODE. ``tau <= 0`` snaps to target."""
    t = np.asarray(t, dtype=float)
    cmd = np.asarray(cmd, dtype=float)
    if t.shape != cmd.shape:
        raise ValueError(f"t and cmd must have the same shape; got {t.shape} vs {cmd.shape}")
    if t.size == 0:
        return np.zeros(0, dtype=float)
    if t.size == 1:
        return np.full(1, initial, dtype=float)

    cmd_delayed = _zoh_at(t - L, t, cmd, left=pre_cmd)
    y = np.empty_like(t)
    y[0] = initial
    for k in range(1, t.size):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            y[k] = y[k - 1]
            continue
        target = K * cmd_delayed[k - 1]
        if tau <= 0:
            y[k] = target
        else:
            alpha = float(np.exp(-dt / tau))
            y[k] = alpha * y[k - 1] + (1.0 - alpha) * target
    return y


# ── PI controller with back-calculation anti-windup ────────────────────────


@dataclass
class PIController:
    """Velocity PI with back-calculation anti-windup. When ``u_sat`` differs from
    ``u_raw`` the integrator is bled by ``(u_sat - u_raw) * Kt * dt``; with
    ``Kt ~ 1/tau`` saturation excursions unwind on the plant timescale."""

    Kp: float
    Ki: float
    Kt: float = 0.0
    u_min: float = -math.inf
    u_max: float = math.inf
    integrator: float = 0.0

    def reset(self) -> None:
        self.integrator = 0.0

    def step(self, ref: float, meas: float, dt: float) -> tuple[float, float]:
        """Advance one control tick. Returns (u_raw, u_sat). The integrator is
        updated AFTER computing u_raw, using the pre-update integrator."""
        e = ref - meas
        u_raw = self.Kp * e + self.Ki * self.integrator
        u_sat = float(np.clip(u_raw, self.u_min, self.u_max))
        self.integrator += (e + self.Kt * (u_sat - u_raw)) * dt
        return float(u_raw), u_sat


@dataclass
class Gains:
    Kp: float
    Ki: float
    Kt: float
    multiplier: float
    lambda_s: float

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def lambda_tune(K: float, tau: float, L: float, multiplier: float) -> Gains:
    """Analytic IMC/lambda PI gains for a FOPDT plant. ``multiplier`` scales the
    closed-loop time constant relative to the dominant plant timescale."""
    if abs(K) < 1e-9:
        raise ValueError("cannot tune a channel with ~zero gain K")
    if tau <= 0:
        raise ValueError("cannot tune a channel with non-positive tau")
    lambda_s = max(tau, L) * multiplier
    Kp = tau / (K * (lambda_s + L))
    Ki = Kp / tau
    Kt = 1.0 / tau
    return Gains(Kp=Kp, Ki=Ki, Kt=Kt, multiplier=multiplier, lambda_s=lambda_s)


# ── reference-signal builders (pure (t) -> value) ──────────────────────────


def step(amplitude: float, t_start: float = 0.5) -> ReferenceFn:
    def r(t: float) -> float:
        return amplitude if t >= t_start else 0.0

    return r


def staircase(levels: Sequence[float], dwell_s: float, t_start: float = 0.5) -> ReferenceFn:
    levels = list(levels)

    def r(t: float) -> float:
        if t < t_start:
            return 0.0
        i = int((t - t_start) // dwell_s)
        if 0 <= i < len(levels):
            return float(levels[i])
        return 0.0

    return r


def ramp(
    slope: float, duration: float, t_start: float = 0.5, *, final_hold: bool = True
) -> ReferenceFn:
    final_value = slope * duration

    def r(t: float) -> float:
        if t < t_start:
            return 0.0
        elapsed = t - t_start
        if elapsed >= duration:
            return float(final_value) if final_hold else 0.0
        return float(slope * elapsed)

    return r


@dataclass(frozen=True)
class Scenario:
    label: str
    reference: ReferenceFn
    duration_s: float


def default_scenarios(saturation_limits: tuple[float, float]) -> list[Scenario]:
    """Reference battery scaled to a channel's command envelope. ``u_max`` is the
    upper saturation limit; amplitudes are fractions of it so the battery is
    channel-agnostic."""
    u_max = saturation_limits[1]
    return [
        Scenario("step_50pct", step(0.5 * u_max), 4.0),
        Scenario("step_90pct", step(0.9 * u_max), 4.0),
        Scenario("step_neg_70pct", step(-0.7 * u_max), 4.0),
        Scenario("staircase", staircase([l * u_max for l in (0.3, 0.6, 0.9, 0.3, 0.0)], 1.5), 8.5),
        Scenario("ramp", ramp(0.5 * u_max, 1.5), 4.0),
    ]


# ── closed-loop simulation + metrics ───────────────────────────────────────


def _compute_step_metrics(
    t: np.ndarray,
    r: np.ndarray,
    u: np.ndarray,
    u_raw: np.ndarray,
    y: np.ndarray,
    *,
    settle_band: float,
) -> dict[str, float]:
    e = r - y
    dt = float(np.mean(np.diff(t))) if t.size > 1 else 0.0
    iae = float(np.sum(np.abs(e)) * dt)
    itae = float(np.sum(t * np.abs(e)) * dt)
    rmse = float(np.sqrt(np.mean(e**2))) if e.size else float("nan")
    max_abs_u = float(np.max(np.abs(u))) if u.size else 0.0
    saturation_fraction = float(np.mean(np.abs(u_raw - u) > 1e-9)) if u.size else 0.0

    r_final = float(r[-1]) if r.size else 0.0
    if r_final > 0:
        overshoot = max(0.0, float(np.max(y)) - r_final) / abs(r_final)
    elif r_final < 0:
        overshoot = max(0.0, abs(float(np.min(y))) - abs(r_final)) / abs(r_final)
    else:
        overshoot = float("nan")

    if abs(r_final) < 1e-9:
        settle_time = float("nan")
    else:
        outside = np.abs(y - r_final) > settle_band * abs(r_final)
        if not outside.any():
            settle_time = 0.0
        elif outside[-1]:
            settle_time = float("inf")
        else:
            settle_time = float(t[np.flatnonzero(outside)[-1]])

    return {
        "iae": iae,
        "itae": itae,
        "rmse": rmse,
        "max_abs_u": max_abs_u,
        "saturation_fraction": saturation_fraction,
        "overshoot": overshoot,
        "settle_time_s": settle_time,
    }


@dataclass
class SimResult:
    t: np.ndarray
    r: np.ndarray
    y: np.ndarray
    u: np.ndarray
    u_raw: np.ndarray
    metrics: dict[str, float]


def simulate_closed_loop(
    K: float,
    tau: float,
    L: float,
    controller: PIController,
    reference_fn: ReferenceFn,
    *,
    duration_s: float,
    control_dt_s: float = DEFAULT_CONTROL_DT_S,
    initial_y: float = 0.0,
    settle_band: float = DEFAULT_SETTLE_BAND,
) -> SimResult:
    """Close the PI loop around the FOPDT plant. The controller acts on the
    PREVIOUS measurement ``y[k-1]``; the plant is re-simulated over the full
    command history each tick (exact, O(n^2) - preserved from the source).
    Resets the controller integrator before running."""
    n = int(duration_s / control_dt_s) + 1
    t = np.arange(n) * control_dt_s
    r = np.array([reference_fn(float(ti)) for ti in t])
    y = np.empty(n)
    u = np.zeros(n)
    u_raw = np.zeros(n)
    y[0] = initial_y
    controller.reset()
    for k in range(1, n):
        ur, us = controller.step(float(r[k - 1]), float(y[k - 1]), control_dt_s)
        u_raw[k] = ur
        u[k] = us
        y_traj = simulate_fopdt(t[: k + 1], u[: k + 1], K, tau, L, initial=initial_y)
        y[k] = y_traj[k]
    metrics = _compute_step_metrics(t, r, u, u_raw, y, settle_band=settle_band)
    return SimResult(t=t, r=r, y=y, u=u, u_raw=u_raw, metrics=metrics)


def _scenario_cost(m: dict[str, float]) -> float:
    over = m["overshoot"] if math.isfinite(m["overshoot"]) else 0.0
    settle = m["settle_time_s"] if math.isfinite(m["settle_time_s"]) else SETTLE_PENALTY_S
    return (
        W_IAE * m["iae"]
        + W_OVERSHOOT * over
        + W_SATURATION * m["saturation_fraction"]
        + W_SETTLE * settle
    )


@dataclass
class TuningCandidate:
    gains: Gains
    per_reference: list[dict[str, Any]]
    cost: float
    cost_breakdown: dict[str, float]


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


def _mean_finite(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return float(np.mean(finite)) if finite else float("nan")


def tune_channel(
    K: float,
    tau: float,
    L: float,
    *,
    saturation_limits: tuple[float, float],
    multiplier_range: tuple[float, float] = (1.0, 3.0),
    n_multipliers: int = 5,
    control_dt_s: float = DEFAULT_CONTROL_DT_S,
    scenarios: list[Scenario] | None = None,
) -> TuningResult:
    """Sweep the lambda multiplier, score each candidate over the scenario
    battery, return all candidates with the best (lowest-cost) index."""
    u_min, u_max = saturation_limits
    scen = scenarios if scenarios is not None else default_scenarios(saturation_limits)
    multipliers = list(np.linspace(multiplier_range[0], multiplier_range[1], n_multipliers))

    candidates: list[TuningCandidate] = []
    for mult in multipliers:
        gains = lambda_tune(K, tau, L, mult)
        per_ref: list[dict[str, Any]] = []
        total = 0.0
        breakdown_acc: dict[str, list[float]] = {
            "iae": [],
            "overshoot": [],
            "saturation_fraction": [],
            "settle_time_s": [],
        }
        for s in scen:
            ctrl = PIController(Kp=gains.Kp, Ki=gains.Ki, Kt=gains.Kt, u_min=u_min, u_max=u_max)
            sim = simulate_closed_loop(
                K, tau, L, ctrl, s.reference, duration_s=s.duration_s, control_dt_s=control_dt_s
            )
            cost = _scenario_cost(sim.metrics)
            total += cost
            per_ref.append({"label": s.label, "cost": cost, **sim.metrics})
            for key in breakdown_acc:
                breakdown_acc[key].append(sim.metrics[key])
        breakdown = {k: _mean_finite(v) for k, v in breakdown_acc.items()}
        candidates.append(
            TuningCandidate(
                gains=gains, per_reference=per_ref, cost=total, cost_breakdown=breakdown
            )
        )

    costs = [c.cost for c in candidates]
    best_index = int(np.argmin(costs))
    return TuningResult(
        plant={"K": K, "tau": tau, "L": L},
        saturation_limits=saturation_limits,
        multipliers=multipliers,
        candidates=candidates,
        best_index=best_index,
    )


# ── robustness sweeps + verdict ────────────────────────────────────────────


@dataclass
class SweepResult:
    which: str
    factors: list[float]
    iaes: list[float]
    stable: list[bool]

    @property
    def all_stable(self) -> bool:
        return all(self.stable)

    @property
    def worst_iae(self) -> float:
        finite = [v for v in self.iaes if math.isfinite(v)]
        return max(finite) if finite else float("inf")


def _is_stable(y: np.ndarray, sat_max: float) -> bool:
    if not np.all(np.isfinite(y)):
        return False
    return bool(np.max(np.abs(y)) < max(STABILITY_GAIN * sat_max, STABILITY_FLOOR))


def _robustness_reference(saturation_limits: tuple[float, float]) -> tuple[ReferenceFn, float]:
    u_max = saturation_limits[1]
    return step(0.7 * u_max), 4.0


def _sweep(
    gains: Gains,
    K: float,
    tau: float,
    L: float,
    saturation_limits: tuple[float, float],
    which: str,
    factors: np.ndarray,
    *,
    control_dt_s: float,
) -> SweepResult:
    u_min, u_max = saturation_limits
    sat_max = max(abs(u_min), abs(u_max))
    ref, duration = _robustness_reference(saturation_limits)
    iaes: list[float] = []
    stable: list[bool] = []
    for f in factors:
        Kp_, tau_, L_ = K, tau, L
        if which == "K":
            Kp_ = K * f
        elif which == "tau":
            tau_ = tau * f
        elif which == "L":
            L_ = L * f
        ctrl = PIController(Kp=gains.Kp, Ki=gains.Ki, Kt=gains.Kt, u_min=u_min, u_max=u_max)
        sim = simulate_closed_loop(
            Kp_, tau_, L_, ctrl, ref, duration_s=duration, control_dt_s=control_dt_s
        )
        iaes.append(sim.metrics["iae"])
        stable.append(_is_stable(sim.y, sat_max))
    return SweepResult(which=which, factors=list(factors), iaes=iaes, stable=stable)


@dataclass
class RobustnessReport:
    gain_sweep: SweepResult
    tau_sweep: SweepResult
    l_sweep: SweepResult

    @property
    def all_stable(self) -> bool:
        return self.gain_sweep.all_stable and self.tau_sweep.all_stable and self.l_sweep.all_stable


def robustness_sweep(
    gains: Gains,
    K: float,
    tau: float,
    L: float,
    saturation_limits: tuple[float, float],
    *,
    control_dt_s: float = DEFAULT_CONTROL_DT_S,
) -> RobustnessReport:
    """Three sweeps against a 70%-amplitude step: K +/-15% (run-to-run variance)
    and tau/L +/-20% (model uncertainty)."""
    gain_factors = np.linspace(0.85, 1.15, 7)
    param_factors = np.linspace(0.80, 1.20, 7)
    return RobustnessReport(
        gain_sweep=_sweep(
            gains, K, tau, L, saturation_limits, "K", gain_factors, control_dt_s=control_dt_s
        ),
        tau_sweep=_sweep(
            gains, K, tau, L, saturation_limits, "tau", param_factors, control_dt_s=control_dt_s
        ),
        l_sweep=_sweep(
            gains, K, tau, L, saturation_limits, "L", param_factors, control_dt_s=control_dt_s
        ),
    )


def channel_verdict(best: TuningCandidate, robustness: RobustnessReport) -> Verdict:
    """pass/marginal/fail from robustness stability + the winner's overshoot and
    saturation behavior."""
    if not robustness.all_stable:
        return "fail"
    overshoot = best.cost_breakdown.get("overshoot", 0.0)
    if not math.isfinite(overshoot):
        overshoot = 0.0
    saturation = best.cost_breakdown.get("saturation_fraction", 0.0)
    if overshoot <= VERDICT_OVERSHOOT_MAX and saturation <= VERDICT_SATURATION_MAX:
        return "pass"
    return "marginal"


@dataclass
class ChannelTuning:
    """Full tuning record for one channel: winning gains, robustness, verdict."""

    channel: str
    plant: dict[str, float]
    gains: Gains
    verdict: Verdict
    cost_breakdown: dict[str, float]
    robustness: RobustnessReport = field(repr=False)


def tune_channel_full(
    channel: str,
    K: float,
    tau: float,
    L: float,
    *,
    saturation_limits: tuple[float, float],
    control_dt_s: float = DEFAULT_CONTROL_DT_S,
    **tune_kwargs: Any,
) -> ChannelTuning:
    """End-to-end for one channel: sweep -> pick winner -> robustness -> verdict."""
    result = tune_channel(
        K, tau, L, saturation_limits=saturation_limits, control_dt_s=control_dt_s, **tune_kwargs
    )
    best = result.best
    robustness = robustness_sweep(
        best.gains, K, tau, L, saturation_limits, control_dt_s=control_dt_s
    )
    verdict = channel_verdict(best, robustness)
    return ChannelTuning(
        channel=channel,
        plant=result.plant,
        gains=best.gains,
        verdict=verdict,
        cost_breakdown=best.cost_breakdown,
        robustness=robustness,
    )


__all__ = [
    "ChannelTuning",
    "Gains",
    "PIController",
    "Scenario",
    "TuningCandidate",
    "TuningResult",
    "channel_verdict",
    "default_scenarios",
    "lambda_tune",
    "ramp",
    "robustness_sweep",
    "simulate_closed_loop",
    "simulate_fopdt",
    "staircase",
    "step",
    "tune_channel",
    "tune_channel_full",
]
