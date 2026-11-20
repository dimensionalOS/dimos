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
    rest_noise,
)

RATE_HZ = 100.0
INERTIA = 0.02


def simulate(
    kp: float, kd: float, delta: float = 2.0, hold: float = 2.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """An ideal spring-damper stepping ``delta`` deg from rest at t = 0, sampled at RATE_HZ."""
    dt = 1e-4
    q, dq, samples = 0.0, 0.0, []
    for k in range(round(hold / dt) + 1):
        if k % round(1 / (RATE_HZ * dt)) == 0:
            samples.append((k * dt, q, dq))
        ddq = (kp * (delta - q) - kd * dq) / INERTIA
        dq += ddq * dt
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


def test_a_well_damped_step_passes_alone_and_against_itself(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kp, kd = 30.0, 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(kp, kd)
    baseline = analyze_step(
        event(kp, kd), t, *noisy(rng, q, dq), noise, max_overshoot_pct=20.0, baseline=None
    )
    assert baseline.passed, baseline.failures
    assert baseline.first_motion_s is not None and baseline.first_motion_s < 0.05
    assert abs(baseline.steady_state_error_deg) < 0.01
    t, q, dq = simulate(2 * kp, 1.5 * kd)
    stiffer = analyze_step(
        event(2 * kp, 1.5 * kd),
        t,
        *noisy(rng, q, dq),
        noise,
        max_overshoot_pct=20.0,
        baseline=baseline,
    )
    assert stiffer.baseline_settle_s == baseline.settle_time_s and stiffer.passed, stiffer.failures


def test_an_underdamped_baseline_fails_only_on_overshoot(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kp, kd = 30.0, 0.2 * 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(kp, kd, hold=3.0)
    metrics = analyze_step(
        event(kp, kd, hold=3.0), t, *noisy(rng, q, dq), noise, max_overshoot_pct=20.0, baseline=None
    )
    assert metrics.overshoot_pct == pytest.approx(52.7, abs=2.0)
    assert len(metrics.failures) == 1 and "overshot" in metrics.failures[0]


def test_a_candidate_slower_than_its_baseline_fails(
    rng: np.random.Generator, noise: RestNoise
) -> None:
    kd = 2.0 * math.sqrt(30.0 * INERTIA)
    t, q, dq = simulate(30.0, kd)
    baseline = analyze_step(
        event(30.0, kd), t, *noisy(rng, q, dq), noise, max_overshoot_pct=100.0, baseline=None
    )
    t, q, dq = simulate(10.0, 4 * kd)
    slow = analyze_step(
        event(10.0, 4 * kd),
        t,
        *noisy(rng, q, dq),
        noise,
        max_overshoot_pct=100.0,
        baseline=baseline,
    )
    assert slow.settle_time_s > baseline.settle_time_s
    assert slow.failures == (
        f"+2 deg step settled in {slow.settle_time_s:.2f} s, slower than the baseline's "
        f"{baseline.settle_time_s:.2f} s",
    )
    with pytest.raises(ValueError, match="baseline"):
        analyze_step(
            event(10.0, kd, delta=-2.0),
            t,
            -q,
            -dq,
            noise,
            max_overshoot_pct=100.0,
            baseline=baseline,
        )


def test_a_buzzing_rest_fails(rng: np.random.Generator, noise: RestNoise) -> None:
    kp, kd = 30.0, 1.55
    t, q, dq = simulate(kp, kd)
    buzz = 0.05 * np.sin(2 * math.pi * 5 * t)
    q, dq = noisy(rng, q + buzz, dq + 0.05 * 2 * math.pi * 5 * np.cos(2 * math.pi * 5 * t))
    metrics = analyze_step(event(kp, kd), t, q, dq, noise, max_overshoot_pct=20.0, baseline=None)
    assert any("buzzes" in reason for reason in metrics.failures)


def test_a_step_that_never_moves_fails(rng: np.random.Generator) -> None:
    t = np.arange(201) / RATE_HZ
    q, dq = (np.tile(v, 5)[: t.size] for v in noisy(rng, np.zeros(50), np.zeros(50)))
    noise = rest_noise(t[:50], q[:50], dq[:50])
    metrics = analyze_step(event(30.0, 1.0), t, q, dq, noise, max_overshoot_pct=20.0, baseline=None)
    assert metrics.first_motion_s is None and "never moved" in metrics.failures[0]


def test_samples_that_stop_short_or_are_not_finite_raise(noise: RestNoise) -> None:
    t, q, dq = simulate(30.0, 1.55)
    with pytest.raises(ValueError, match="stop"):
        analyze_step(
            event(30.0, 1.55),
            t[:150],
            q[:150],
            dq[:150],
            noise,
            max_overshoot_pct=20.0,
            baseline=None,
        )
    with pytest.raises(ValueError, match="outlast"):
        analyze_step(
            event(30.0, 1.55, hold=0.4), t, q, dq, noise, max_overshoot_pct=20.0, baseline=None
        )
    q[10] = math.nan
    with pytest.raises(ValueError, match="NaN"):
        analyze_step(event(30.0, 1.55), t, q, dq, noise, max_overshoot_pct=20.0, baseline=None)


def metrics(kp: float, kd: float, delta: float, *failures: str, settle: float = 0.2) -> StepMetrics:
    return StepMetrics("j", delta, kp, kd, 100, 0.01, 0.01, settle, None, 0.0, 0.0, 0.1, failures)


def test_propose_gains_picks_the_stiffest_pair_that_passes_both_ways() -> None:
    steps = [
        *(metrics(kp, 1.0, d) for kp in (10.0, 20.0) for d in (1.0, -1.0)),
        metrics(30.0, 1.0, 1.0),
        metrics(30.0, 1.0, -1.0, "overshot"),
        metrics(20.0, 2.0, 1.0, settle=0.1),
        metrics(20.0, 2.0, -1.0, settle=0.1),
    ]
    proposal = propose_gains(steps)["j"]
    assert (proposal.kp, proposal.kd) == (20.0, 2.0)
    assert proposal.rejected["kp=30 kd=1"] == "overshot"


def test_propose_gains_refuses_a_stiff_pass_above_a_failing_softer_pair() -> None:
    steps = [
        metrics(10.0, 1.0, 1.0),
        metrics(10.0, 1.0, -1.0, "buzzes"),
        metrics(20.0, 1.0, 1.0),
        metrics(20.0, 1.0, -1.0),
        metrics(30.0, 1.0, 1.0),
    ]
    proposal = propose_gains(
        steps, candidates=[(10.0, 1.0), (20.0, 1.0), (30.0, 1.0), (40.0, 1.0)]
    )["j"]
    assert proposal.kp is None
    assert "below it failed" in proposal.rejected["kp=20 kd=1"]
    assert proposal.rejected["kp=30 kd=1"] == "no negative step"
    assert proposal.rejected["kp=40 kd=1"] == "not tested"
