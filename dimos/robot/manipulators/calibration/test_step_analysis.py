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

from dimos.robot.manipulators.calibration.step_analysis import (
    SETTLE_BAND_DEG,
    StepEvent,
    StepMetrics,
    analyze_step,
    format_proposals,
    format_table,
    propose_gains,
    segment_steps,
)

DT = 0.005  # 200 Hz samples
HOLD_S = 2.0


def _event(delta_deg: float = 1.0, **overrides: object) -> StepEvent:
    fields = {
        "joint": "left_joint1",
        "delta_deg": delta_deg,
        "kp": 80.0,
        "kd": 5.0,
        "t0": 100.0,
        "hold_s": HOLD_S,
        "target_before_deg": 10.0,
        "target_after_deg": 10.0 + delta_deg,
    }
    fields.update(overrides)
    return StepEvent(**fields)  # type: ignore[arg-type]


def _first_order(delta: float, tau: float, dead_time: float = 0.0) -> tuple[np.ndarray, ...]:
    t = np.arange(0.0, HOLD_S, DT)
    active = np.clip(t - dead_time, 0.0, None)
    y = delta * (1.0 - np.exp(-active / tau))
    dy = np.where(t >= dead_time, delta / tau * np.exp(-active / tau), 0.0)
    return t, 10.0 + y, dy


def _second_order(delta: float, wn: float, zeta: float) -> tuple[np.ndarray, ...]:
    t = np.arange(0.0, HOLD_S, DT)
    wd = wn * math.sqrt(1.0 - zeta**2)
    phi = math.atan2(math.sqrt(1.0 - zeta**2), zeta)
    y = delta * (1.0 - np.exp(-zeta * wn * t) / math.sqrt(1.0 - zeta**2) * np.sin(wd * t + phi))
    dy = np.gradient(y, DT)
    return t, 10.0 + y, dy


def test_first_order_settle_time_matches_the_analytic_value() -> None:
    tau = 0.1
    t, y, dy = _first_order(1.0, tau)
    metrics = analyze_step(_event(1.0), t, y, dy)
    expected = tau * math.log(1.0 / SETTLE_BAND_DEG)
    assert metrics.settle_time_s is not None
    assert metrics.settle_time_s == pytest.approx(expected, abs=2 * DT)
    assert metrics.overshoot_pct == 0.0
    assert abs(metrics.steady_state_error_deg) < 1e-3
    assert metrics.buzz_rms_deg_s < 0.01
    assert metrics.passed


def test_dead_time_shows_up_as_first_motion() -> None:
    t, y, dy = _first_order(1.0, 0.05, dead_time=0.4)
    metrics = analyze_step(_event(1.0), t, y, dy)
    # First sample beyond 0.05 deg: 1 - exp(-x/0.05) > 0.05 -> x > 2.6 ms.
    assert metrics.first_motion_s is not None
    assert metrics.first_motion_s == pytest.approx(0.4, abs=2 * DT)
    assert metrics.settle_time_s is not None
    assert metrics.settle_time_s > 0.4


def test_underdamped_overshoot_matches_the_analytic_value() -> None:
    zeta = 0.3
    t, y, dy = _second_order(3.0, wn=30.0, zeta=zeta)
    metrics = analyze_step(_event(3.0), t, y, dy)
    expected = 100.0 * math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta**2))
    assert metrics.overshoot_pct == pytest.approx(expected, rel=0.02)
    assert metrics.settle_time_s is not None
    assert not metrics.passed  # 37 % overshoot


def test_negative_step_overshoot_is_measured_past_the_target() -> None:
    zeta = 0.5
    t, y_pos, dy_pos = _second_order(1.0, wn=40.0, zeta=zeta)
    y = 10.0 - (y_pos - 10.0)
    metrics = analyze_step(_event(-1.0), t, y, -dy_pos)
    expected = 100.0 * math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta**2))
    assert metrics.overshoot_pct == pytest.approx(expected, rel=0.02)
    assert metrics.passed


def test_stuck_joint_never_settles_and_reports_the_offset() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    y = y - 0.3  # stiction leaves it 0.3 deg short
    metrics = analyze_step(_event(1.0), t, y, dy)
    assert metrics.settle_time_s is None
    assert metrics.steady_state_error_deg == pytest.approx(-0.3, abs=1e-3)
    assert not metrics.passed


def test_buzz_is_the_velocity_rms_at_rest() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    buzz = 2.0 * np.sin(2 * math.pi * 50.0 * t)  # 2 deg/s amplitude
    metrics = analyze_step(_event(1.0), t, y, dy + buzz)
    assert metrics.buzz_rms_deg_s == pytest.approx(2.0 / math.sqrt(2.0), rel=0.05)
    assert not metrics.passed


def test_segment_steps_cuts_a_hold_short_at_the_next_step() -> None:
    first = _event(1.0, t0=100.0)
    second = _event(-1.0, t0=101.0)
    windows = segment_steps([second, first], t_end=102.4)
    assert [(e.delta_deg, start, stop) for e, start, stop in windows] == [
        (1.0, 100.0, 101.0),
        (-1.0, 101.0, 102.4),
    ]


def test_step_event_round_trips_through_json() -> None:
    event = _event(3.0, kp=120.0)
    assert StepEvent.from_json(event.to_json()) == event


def test_table_lists_every_step_with_a_verdict() -> None:
    t, y, dy = _first_order(1.0, 0.1)
    rows = [analyze_step(_event(d), t, y, dy) for d in (1.0, -1.0)]
    table = format_table(rows)
    assert table.count("| left_joint1 |") == 2
    assert "| ok |" in table
    assert "| FAIL |" in table  # the -1 step evaluated against a +1 response never settles


# Gain proposal


def _sweep_step(joint: str, kp: float, kd: float, delta: float) -> StepMetrics:
    """A step whose response stiffens with kp: overdamped low, ringing high."""
    if kp <= 40.0:
        t, y, dy = _first_order(delta, tau=0.3)  # too soft: settles in about 0.7 s
    elif kp <= 120.0:
        t, y, dy = _first_order(delta, tau=0.05)
    else:
        t, y, dy = _second_order(delta, wn=60.0, zeta=0.25)  # rings: ~44 % overshoot
    return analyze_step(_event(delta, joint=joint, kp=kp, kd=kd), t, y, dy)


def test_propose_gains_picks_the_highest_passing_kp_and_explains_the_rest() -> None:
    candidates = [(40.0, 2.0), (80.0, 5.0), (120.0, 5.0), (160.0, 5.0), (200.0, 5.0)]
    steps = [
        _sweep_step("arm/joint2", kp, kd, delta)
        for kp, kd in candidates[:4]
        for delta in (1.0, -1.0, 3.0, -3.0)
    ]
    proposal = propose_gains(steps, candidates)["arm/joint2"]
    assert proposal.gains == (120.0, 5.0)
    assert "highest kp" in proposal.reason and "4 step(s) all pass" in proposal.reason
    assert set(proposal.rejected) == {"kp=160 kd=5", "kp=200 kd=5"}
    assert "overshot" in proposal.rejected["kp=160 kd=5"]
    assert proposal.rejected["kp=200 kd=5"] == "not tested"
    # Without explicit candidates every tested pair counts.
    assert propose_gains(steps)["arm/joint2"].gains == (120.0, 5.0)
    text = format_proposals(propose_gains(steps))
    assert "arm/joint2: kp=120 kd=5" in text and "kp=160 kd=5 rejected" in text


def test_propose_gains_needs_every_step_to_pass() -> None:
    good = _sweep_step("j", 80.0, 5.0, 1.0)
    t, y, dy = _first_order(1.0, 0.05)
    stuck = analyze_step(_event(-1.0, joint="j", kp=120.0, kd=5.0), t, 10.0 - (y - 10.0) + 0.3, -dy)
    fine = analyze_step(_event(1.0, joint="j", kp=120.0, kd=5.0), t, y, dy)
    proposal = propose_gains([good, stuck, fine])["j"]
    assert proposal.gains == (80.0, 5.0)
    assert "off target" in proposal.rejected["kp=120 kd=5"]


def test_propose_gains_prefers_the_faster_kd_at_equal_kp_and_reports_none_passing() -> None:
    t = np.arange(0.0, HOLD_S, DT)
    slow = _first_order(1.0, 0.08)
    fast = _first_order(1.0, 0.04)
    steps = [
        analyze_step(_event(1.0, joint="j", kp=80.0, kd=2.0), t, slow[1], slow[2]),
        analyze_step(_event(1.0, joint="j", kp=80.0, kd=5.0), t, fast[1], fast[2]),
    ]
    assert propose_gains(steps)["j"].gains == (80.0, 5.0)
    ringing = _second_order(1.0, wn=60.0, zeta=0.2)
    failing = [analyze_step(_event(1.0, joint="k", kp=300.0, kd=1.0), t, ringing[1], ringing[2])]
    none = propose_gains(failing)["k"]
    assert none.gains is None and none.kp is None
    assert "no candidate passed" in none.reason and "kp=300 kd=1" in none.rejected
