# Copyright 2026 Dimensional Inc.
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


import math

import numpy as np
import pytest

from dimos.control.calibration.step_analysis import (
    RestNoise,
    StepEvent,
    StepMetrics,
    analyze_step,
    propose_gains,
    rest_limit_deg,
    rest_noise,
    sweep_friction_nm,
)

RATE_HZ = 100.0
INERTIA = 0.02
ANALYSIS = {"max_overshoot_pct": 20.0, "rest_limit_deg": 0.1, "stuck_fraction": 0.5}
PICK = {"settle_s": 0.5, "kd_limit": 5.0, "max_overshoot_pct": 20.0}


def simulate(
    kp: float, kd: float, delta: float = 2.0, hold: float = 2.0, friction: float = 0.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A spring-damper stepping ``delta`` deg from rest at t = 0, sampled at RATE_HZ; past
    ``friction`` (N m) of spring torque it moves, short of it it sticks."""
    dt = 1e-4
    q, dq, samples = 0.0, 0.0, []
    for k in range(round(hold / dt) + 1):
        if k % round(1 / (RATE_HZ * dt)) == 0:
            samples.append((k * dt, q, dq))
        torque = kp * math.radians(delta - q) - kd * math.radians(dq)
        if dq == 0.0 and abs(torque) <= friction:
            continue
        ddq = math.degrees(torque - math.copysign(friction, dq or torque)) / INERTIA
        stopped = dq != 0.0 and (dq + ddq * dt) * dq < 0.0
        dq = 0.0 if stopped else dq + ddq * dt
        q += dq * dt
    t, q_, dq_ = (np.asarray(column) for column in zip(*samples, strict=True))
    return t, q_, dq_


def noisy(rng: np.random.Generator, q: np.ndarray, dq: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return q + rng.normal(0.0, 0.005, q.size), dq + rng.normal(0.0, 0.2, dq.size)


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(0)


@pytest.fixture
def noise(rng: np.random.Generator) -> RestNoise:
    t = np.arange(50) / RATE_HZ
    q, dq = noisy(rng, np.zeros(t.size), np.zeros(t.size))
    return rest_noise(t, q, dq)


def event(kp: float, kd: float, delta: float = 2.0, hold: float = 2.0) -> StepEvent:
    return StepEvent("j", delta, kp, kd, 0.0, hold, 0.0, delta)


def test_rest_noise_measures_the_window(noise: RestNoise) -> None:
    assert noise.window_s == pytest.approx(0.49)
    assert 0.005 < noise.band_deg < 0.03
    assert 0.2 < noise.speed_peak_deg_s < 1.0


def test_a_well_damped_step_passes(rng: np.random.Generator, noise: RestNoise) -> None:
    kp, kd = 30.0, 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(kp, kd)
    metrics = analyze_step(event(kp, kd), t, *noisy(rng, q, dq), noise, **ANALYSIS)
    assert metrics.passed, metrics.failures
    assert metrics.first_motion_s is not None and metrics.first_motion_s < 0.05
    assert abs(metrics.steady_state_error_deg) < 0.01 and metrics.moved_deg > 1.9
    assert metrics.settle_time_s is not None and metrics.settle_time_s < 0.5


def test_an_underdamped_step_fails_only_on_overshoot(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kp, kd = 30.0, 0.2 * 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(kp, kd, hold=3.0)
    metrics = analyze_step(event(kp, kd, hold=3.0), t, *noisy(rng, q, dq), noise, **ANALYSIS)
    assert metrics.overshoot_pct == pytest.approx(52.7, abs=2.0)
    assert len(metrics.failures) == 1 and "overshot" in metrics.failures[0]


def test_a_joint_resting_short_on_friction_settles_to_its_rest_within_its_limit(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kp, kd, friction = 30.0, 2.0 * math.sqrt(30.0 * INERTIA), 0.2
    t, q, dq = simulate(kp, kd, friction=friction)
    limit = rest_limit_deg(friction, kp, smallest_deg=0.1, friction_factor=1.5)
    assert limit == pytest.approx(math.degrees(1.5 * friction / kp))
    metrics = analyze_step(
        event(kp, kd), t, *noisy(rng, q, dq), noise, **{**ANALYSIS, "rest_limit_deg": limit}
    )
    assert metrics.passed, metrics.failures
    assert metrics.steady_state_error_deg == pytest.approx(-math.degrees(friction / kp), abs=0.03)
    assert metrics.rest_limit_deg == limit and metrics.settle_time_s is not None
    assert rest_limit_deg(0.001, kp, smallest_deg=0.1, friction_factor=1.5) == 0.1


def test_a_step_that_barely_moves_is_stuck_and_skips_overshoot_and_settle(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kp, kd = 30.0, 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(kp, kd, friction=kp * math.radians(1.2))
    metrics = analyze_step(event(kp, kd), t, *noisy(rng, q, dq), noise, **ANALYSIS)
    assert metrics.stuck and not metrics.passed and metrics.failures == ()
    assert metrics.moved_deg == pytest.approx(0.8, abs=0.05)
    assert metrics.settle_time_s is None and metrics.overshoot_pct is None
    t = np.arange(201) / RATE_HZ
    still, speed = (np.tile(v, 5)[: t.size] for v in noisy(rng, np.zeros(50), np.zeros(50)))
    never = analyze_step(
        event(kp, kd), t, still, speed, rest_noise(t[:50], still[:50], speed[:50]), **ANALYSIS
    )
    assert never.stuck and never.first_motion_s is None


def test_a_buzzing_rest_fails(rng: np.random.Generator, noise: RestNoise) -> None:
    kp, kd = 30.0, 1.55
    t, q, dq = simulate(kp, kd)
    buzz = 0.05 * np.sin(2 * math.pi * 5 * t)
    q, dq = noisy(rng, q + buzz, dq + 0.05 * 2 * math.pi * 5 * np.cos(2 * math.pi * 5 * t))
    metrics = analyze_step(event(kp, kd), t, q, dq, noise, **ANALYSIS)
    assert metrics.buzzing and any("buzzes" in reason for reason in metrics.failures)


def test_samples_that_stop_short_or_are_not_finite_raise(noise: RestNoise) -> None:
    t, q, dq = simulate(30.0, 1.55)
    with pytest.raises(ValueError, match="stop"):
        analyze_step(event(30.0, 1.55), t[:150], q[:150], dq[:150], noise, **ANALYSIS)
    with pytest.raises(ValueError, match="outlast"):
        analyze_step(event(30.0, 1.55, hold=0.4), t, q, dq, noise, **ANALYSIS)
    q[10] = math.nan
    with pytest.raises(ValueError, match="NaN"):
        analyze_step(event(30.0, 1.55), t, q, dq, noise, **ANALYSIS)


def metrics(
    kp: float,
    kd: float,
    delta: float,
    *failures: str,
    settle: float = 0.2,
    error: float = 0.05,
    limit: float = 0.1,
    stuck: bool = False,
    overshoot: float = 0.0,
) -> StepMetrics:
    return StepMetrics(
        "j",
        delta,
        kp,
        kd,
        100,
        0.01,
        0.01,
        0.1 if stuck else delta,
        stuck,
        None if stuck else settle,
        None if stuck else overshoot,
        error,
        limit,
        0.1,
        False,
        failures,
    )


def test_propose_gains_picks_the_lowest_rest_and_settle_score_and_says_why_the_rest_lost() -> None:
    steps = [
        *(metrics(10.0, 1.0, d, error=0.09, settle=0.1) for d in (1.0, -1.0)),
        *(metrics(20.0, 1.0, d, error=0.02, settle=0.3) for d in (1.0, -1.0)),
        *(metrics(20.0, 2.0, d, error=0.02, settle=0.1) for d in (1.0, -1.0)),
        metrics(30.0, 1.0, 1.0),
        metrics(30.0, 1.0, -1.0, "overshot"),
    ]
    proposal = propose_gains(steps, **PICK)["j"]
    assert (proposal.kp, proposal.kd) == (20.0, 2.0)
    assert proposal.rejected["kp=10 kd=1"] == "passes, but scores 1.10 against 0.40"
    assert proposal.rejected["kp=20 kd=1"] == "passes, but scores 0.80 against 0.40"
    assert proposal.rejected["kp=30 kd=1"] == "overshot"


def test_a_stuck_small_step_drops_out_of_the_score_and_a_stuck_largest_one_rejects() -> None:
    steps = [
        metrics(10.0, 1.0, 1.0, stuck=True),
        *(metrics(10.0, 1.0, d, settle=0.1) for d in (-1.0, 3.0, -3.0)),
        *(metrics(20.0, 1.0, d) for d in (1.0, -1.0, 3.0)),
        metrics(20.0, 1.0, -3.0, stuck=True),
    ]
    proposal = propose_gains(steps, candidates=[(10.0, 1.0), (20.0, 1.0), (30.0, 1.0)], **PICK)
    assert (proposal["j"].kp, proposal["j"].kd) == (10.0, 1.0)
    assert "its +1 deg steps stuck and do not count" in proposal["j"].reason
    assert proposal["j"].rejected["kp=20 kd=1"].startswith("-3 deg step is stuck")
    assert proposal["j"].rejected["kp=30 kd=1"] == "not tested"


def test_a_candidate_stopped_in_the_sweep_or_without_a_step_each_way_is_rejected() -> None:
    steps = [
        *(metrics(10.0, 1.0, d) for d in (1.0, -1.0)),
        metrics(20.0, 1.0, 1.0),
        *(metrics(30.0, 1.0, d) for d in (1.0, -1.0)),
    ]
    stopped = {"j": {(30.0, 1.0): "stopped at its -1 deg step, which buzzed"}}
    proposal = propose_gains(steps, stopped=stopped, **PICK)["j"]
    assert (proposal.kp, proposal.kd) == (10.0, 1.0)
    assert proposal.rejected["kp=20 kd=1"] == "no negative step"
    assert proposal.rejected["kp=30 kd=1"] == "stopped at its -1 deg step, which buzzed"


def test_overshooting_with_kd_at_the_driver_limit_says_to_lower_kp() -> None:
    steps = [
        metrics(kp, kd, d, "overshot", overshoot=40.0)
        for kp in (80.0, 120.0)
        for kd in (2.0, 5.0)
        for d in (1.0, -1.0)
    ]
    proposal = propose_gains(steps, **PICK)["j"]
    assert proposal.kp is None
    assert proposal.reason.endswith("kd at the driver limit 5; lower kp")
    softer = propose_gains([s for s in steps if s.kd == 2.0], **PICK)["j"]
    assert "lower kp" not in softer.reason


def test_friction_is_kp_times_half_the_rest_band_or_the_largest_rest_error() -> None:
    kp = 40.0
    band = [metrics(kp, 1.0, d, error=-0.3 * math.copysign(1.0, d)) for d in (1.0, -1.0, 3.0, -3.0)]
    assert sweep_friction_nm(band) == {"j": pytest.approx(kp * math.radians(0.3))}
    offset = [metrics(kp, 1.0, d, error=0.5) for d in (3.0, -3.0)]
    assert sweep_friction_nm(offset) == {"j": pytest.approx(kp * math.radians(0.5))}
    stuck = [*band[2:], metrics(kp, 1.0, 1.0, error=-0.9, stuck=True)]
    assert sweep_friction_nm(stuck) == {"j": pytest.approx(kp * math.radians(0.3))}
