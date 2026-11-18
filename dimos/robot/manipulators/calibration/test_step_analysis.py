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

from dataclasses import replace
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
from dimos.robot.manipulators.calibration.step_report import (
    CommandTrace,
    JointTrace,
    Recording,
    extract_steps,
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
    candidates = [(80.0, 5.0), (120.0, 5.0), (160.0, 5.0), (200.0, 5.0)]
    steps = [
        _sweep_step("arm/joint2", kp, kd, delta)
        for kp, kd in candidates[:3]
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
    # A softer scale that fails (too slow) blocks every stiffer one, whatever its kd.
    soft = [_sweep_step("arm/joint2", 40.0, 2.0, delta) for delta in (1.0, -1.0)]
    blocked = propose_gains([*soft, *steps])["arm/joint2"]
    assert blocked.gains is None
    assert "settled in" in blocked.rejected["kp=40 kd=2"]
    assert blocked.rejected["kp=120 kd=5"] == "its steps pass, but kp=40 kd=2 below it failed"


def test_propose_gains_needs_every_step_to_pass() -> None:
    good = [_sweep_step("j", 80.0, 5.0, delta) for delta in (1.0, -1.0)]
    t, y, dy = _first_order(1.0, 0.05)
    stuck = analyze_step(_event(-1.0, joint="j", kp=120.0, kd=5.0), t, 10.0 - (y - 10.0) + 0.3, -dy)
    fine = analyze_step(_event(1.0, joint="j", kp=120.0, kd=5.0), t, y, dy)
    proposal = propose_gains([*good, stuck, fine])["j"]
    assert proposal.gains == (80.0, 5.0)
    assert "off target" in proposal.rejected["kp=120 kd=5"]


def test_propose_gains_prefers_the_faster_kd_at_equal_kp_and_reports_none_passing() -> None:
    t = np.arange(0.0, HOLD_S, DT)
    steps = [
        analyze_step(_event(delta, joint="j", kp=80.0, kd=kd), t, y, dy)
        for kd, tau in ((2.0, 0.08), (5.0, 0.04))
        for delta in (1.0, -1.0)
        for _, y, dy in [_first_order(delta, tau)]
    ]
    assert propose_gains(steps)["j"].gains == (80.0, 5.0)
    ringing = _second_order(1.0, wn=60.0, zeta=0.2)
    failing = [analyze_step(_event(1.0, joint="k", kp=300.0, kd=1.0), t, ringing[1], ringing[2])]
    none = propose_gains(failing)["k"]
    assert none.gains is None and none.kp is None
    assert "no candidate passed" in none.reason and "kp=300 kd=1" in none.rejected


def test_a_non_finite_metric_fails_the_step_and_its_gains() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    dy = dy.copy()
    dy[-3] = np.nan  # one velocity sample lost in the rest window
    metrics = analyze_step(_event(1.0, joint="j"), t, y, dy)
    assert math.isnan(metrics.buzz_rms_deg_s)
    assert not metrics.passed
    assert metrics.failures() == ["+1 deg step metric buzz_rms_deg_s is not finite"]
    assert metrics.as_row()["passed"] is False
    t_neg, y_neg, dy_neg = _first_order(-1.0, 0.05)
    other_direction = analyze_step(_event(-1.0, joint="j"), t_neg, y_neg, dy_neg)
    assert other_direction.passed
    proposal = propose_gains([metrics, other_direction])["j"]
    assert proposal.gains is None
    assert "not finite" in proposal.rejected["kp=80 kd=5"]
    overshoot = replace(other_direction, overshoot_pct=math.inf)
    assert overshoot.failures() == ["-1 deg step metric overshoot_pct is not finite"]


def test_a_hold_cut_short_fails_the_step() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    keep = t < 0.8  # the next step (or the recording's end) came after 0.8 of 2 s
    metrics = analyze_step(_event(1.0), t[keep], y[keep], dy[keep])
    assert metrics.hold_s == HOLD_S and metrics.recorded_s == pytest.approx(0.795)
    assert not metrics.hold_complete and not metrics.passed
    assert metrics.failures() == ["+1 deg step hold was cut short: 0.80 of 2 s recorded"]
    assert analyze_step(_event(1.0), t, y, dy).hold_complete  # the last sample is one dt short


def test_propose_gains_needs_a_step_each_way() -> None:
    up = [_sweep_step("j", kp, 5.0, 1.0) for kp in (80.0, 120.0)]
    down = _sweep_step("j", 80.0, 5.0, -1.0)
    proposal = propose_gains([*up, down])["j"]
    assert proposal.gains == (80.0, 5.0)
    assert proposal.rejected == {"kp=120 kd=5": "no negative step recorded"}
    only_up = propose_gains(up)["j"]
    assert only_up.gains is None
    assert only_up.rejected["kp=80 kd=5"] == "no negative step recorded"


def test_propose_gains_does_not_pick_a_stiff_pass_above_a_failing_softer_kp() -> None:
    def steps(kp: float, tau: float, kd: float = 5.0) -> list[StepMetrics]:
        return [
            analyze_step(_event(delta, joint="j", kp=kp, kd=kd), *_first_order(delta, tau))
            for delta in (1.0, -1.0)
        ]

    # kp 120 settles too slowly (a bad run); kp 200 happens to pass above it.
    results = [*steps(80.0, 0.05), *steps(120.0, 0.3), *steps(200.0, 0.04)]
    proposal = propose_gains(results)["j"]
    assert proposal.gains == (80.0, 5.0)
    assert "settled in" in proposal.rejected["kp=120 kd=5"]
    assert proposal.rejected["kp=200 kd=5"] == ("its steps pass, but kp=120 kd=5 below it failed")
    # The sweep scales kd with the kp scale, so the rule holds whatever the kd:
    # a failing kp 120 at kd 3.67 blocks kp 200 at kd 4.24 ...
    scaled = propose_gains(
        [*steps(80.0, 0.05, kd=3.0), *steps(120.0, 0.3, kd=3.67), *steps(200.0, 0.04, kd=4.24)]
    )
    assert scaled["j"].gains == (80.0, 3.0)
    assert scaled["j"].rejected["kp=200 kd=4.24"] == (
        "its steps pass, but kp=120 kd=3.67 below it failed"
    )
    # ... and with every softer scale passing, the stiffest is chosen.
    clean = propose_gains([*steps(80.0, 0.05, kd=3.0), *steps(200.0, 0.04, kd=4.24)])
    assert clean["j"].gains == (200.0, 4.24)


def _recorded_step(commands: dict[str, CommandTrace]) -> Recording:
    """A +1 deg step of left_joint1 marked at 100 s whose target went out at 100.012 s."""
    t = 100.0 - 0.1 + np.arange(0.0, HOLD_S + 0.2, DT)
    start = 100.012
    q = np.where(
        t >= start, 10.0 + 1.0 * (1.0 - np.exp(-np.clip(t - start, 0.0, None) / 0.05)), 10.0
    )
    trace = JointTrace(t=t, q_deg=q, dq_deg_s=np.gradient(q, t))
    return Recording(
        traces={"arm/left_joint1": trace}, events=[_event()], t_end=float(t[-1]), commands=commands
    )


def test_a_step_window_starts_at_the_first_recorded_command_of_its_target() -> None:
    sent_t = 100.0 + np.arange(-0.05, 0.2, 0.01) + 0.002
    sent_q = np.where(sent_t >= 100.0115, 11.0, 10.0)
    commands = {"arm/left_joint1": CommandTrace(t=sent_t, q_deg=sent_q)}

    [aligned] = extract_steps(_recorded_step(commands))
    [unaligned] = extract_steps(_recorded_step({}))

    assert aligned.t[0] >= 0.0 and aligned.t[0] < DT
    assert aligned.metrics.first_motion_s is not None and aligned.metrics.first_motion_s < 0.01
    # Without commands the window starts at the marker, 12 ms before the target went out.
    assert unaligned.metrics.first_motion_s is not None
    assert unaligned.metrics.first_motion_s >= 0.012
    # A command that never carries the target leaves the marker as the start.
    stray = {"arm/left_joint1": CommandTrace(t=sent_t, q_deg=np.full(sent_t.shape, 10.5))}
    [fallback] = extract_steps(_recorded_step(stray))
    assert fallback.metrics.first_motion_s == unaligned.metrics.first_motion_s
