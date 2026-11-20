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

from dimos.control.calibration.step_analysis import (
    KD_LIMIT,
    SETTLE_BAND_DEG,
    StepEvent,
    StepMetrics,
    analyze_step,
    format_proposals,
    format_table,
    gain_candidates,
    propose_gains,
    rest_limit_deg,
    segment_steps,
    sweep_friction_nm,
)
from dimos.control.calibration.step_report import (
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


def test_a_joint_resting_short_settles_to_its_rest_and_is_scored_not_failed() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    y = y - 0.3  # friction leaves it 0.3 deg short
    metrics = analyze_step(_event(1.0), t, y, dy)
    assert metrics.steady_state_error_deg == pytest.approx(-0.3, abs=1e-3)
    assert metrics.moved_deg == pytest.approx(0.7, abs=1e-3) and not metrics.stuck
    assert metrics.settle_time_s == pytest.approx(0.05 * math.log(10.0), abs=2 * DT)
    assert metrics.passed and metrics.rest_limit_deg == SETTLE_BAND_DEG
    rubbing = analyze_step(_event(1.0), t, y, dy, friction_nm=0.3)
    limit = math.degrees(1.5 * 0.3 / 80.0)
    assert rubbing.rest_limit_deg == pytest.approx(limit) == rest_limit_deg(0.3, 80.0)
    assert rubbing.settle_time_s == pytest.approx(0.05 * math.log(1.0 / limit), abs=2 * DT)
    assert rest_limit_deg(0.01, 80.0) == SETTLE_BAND_DEG


def test_a_step_that_barely_moves_is_stuck_and_skips_overshoot_and_settle() -> None:
    t, y, dy = _first_order(-0.2, 0.05)  # commanded -1 deg, moved 0.2
    metrics = analyze_step(_event(-1.0), t, y, dy)
    assert metrics.stuck and metrics.moved_deg == pytest.approx(0.2, abs=1e-3)
    assert metrics.overshoot_pct is None and metrics.settle_time_s is None
    assert metrics.failures() == [
        "-1 deg step is stuck: moved +0.20 deg (less than 50% of the step)"
    ]
    assert metrics.failures(stuck=False) == [] and not metrics.passed
    half = analyze_step(_event(-1.0), *_first_order(-0.55, 0.05))
    assert not half.stuck and half.passed


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
    assert "| stuck |" in table  # the -1 step evaluated against a +1 response moved backwards


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


def test_propose_gains_picks_the_lowest_score_and_explains_the_rest() -> None:
    candidates = [(40.0, 5.0), (80.0, 5.0), (120.0, 5.0), (160.0, 5.0), (200.0, 5.0)]
    steps = [
        _sweep_step("arm/joint2", kp, kd, delta)
        for kp, kd in candidates[:4]
        for delta in (1.0, -1.0, 3.0, -3.0)
    ]
    proposal = propose_gains(steps, candidates)["arm/joint2"]
    # kp 80 and 120 respond alike: the tie goes to the stiffer one.
    assert proposal.gains == (120.0, 5.0)
    assert proposal.reason.startswith("kp=120 kd=5: lowest score 0.3")
    assert "of 3 passing candidate(s)" in proposal.reason and "overshoot 0 %" in proposal.reason
    assert set(proposal.rejected) == {"kp=40 kd=5", "kp=80 kd=5", "kp=160 kd=5", "kp=200 kd=5"}
    assert "overshot" in proposal.rejected["kp=160 kd=5"]
    assert proposal.rejected["kp=200 kd=5"] == "not tested"
    # Slow is scored, not failed: kp 40 settles in about a second.
    assert proposal.rejected["kp=40 kd=5"].startswith("passes, but scores 2.")
    assert proposal.rejected["kp=80 kd=5"].startswith("passes, but scores 0.3")
    # Without explicit candidates every tested pair counts.
    assert propose_gains(steps)["arm/joint2"].gains == (120.0, 5.0)
    text = format_proposals(propose_gains(steps))
    assert "arm/joint2 (friction 0.00 Nm): kp=120 kd=5" in text
    assert "kp=160 kd=5 rejected" in text


def test_a_stuck_step_rejects_its_gains() -> None:
    good = [_sweep_step("j", 80.0, 5.0, delta) for delta in (1.0, -1.0)]
    stuck = analyze_step(_event(-1.0, joint="j", kp=120.0, kd=5.0), *_first_order(-0.2, 0.05))
    fine = analyze_step(_event(1.0, joint="j", kp=120.0, kd=5.0), *_first_order(1.0, 0.05))
    proposal = propose_gains([*good, stuck, fine])["j"]
    assert proposal.gains == (80.0, 5.0)
    assert proposal.rejected["kp=120 kd=5"] == (
        "-1 deg step is stuck: moved +0.20 deg (less than 50% of the step)"
    )


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


def test_a_non_finite_sample_or_metric_fails_the_step_and_its_gains() -> None:
    t, y, dy = _first_order(1.0, 0.05)
    dy = dy.copy()
    dy[-3] = np.nan  # one velocity sample lost in the rest window
    metrics = analyze_step(_event(1.0, joint="j"), t, y, dy)
    assert math.isfinite(metrics.buzz_rms_deg_s)
    assert metrics.samples == t.size - 1 and metrics.nonfinite_samples == 1
    assert not metrics.passed
    assert metrics.failures() == ["+1 deg step has 1 non-finite sample(s)"]
    assert metrics.as_row()["passed"] is False
    t_neg, y_neg, dy_neg = _first_order(-1.0, 0.05)
    other_direction = analyze_step(_event(-1.0, joint="j"), t_neg, y_neg, dy_neg)
    assert other_direction.passed
    proposal = propose_gains([metrics, other_direction])["j"]
    assert proposal.gains is None
    assert "non-finite sample" in proposal.rejected["kp=80 kd=5"]
    overshoot = replace(other_direction, overshoot_pct=math.inf)
    assert overshoot.failures() == ["-1 deg step metric overshoot_pct is not finite"]


def test_a_non_finite_sample_cannot_hide_an_overshoot() -> None:
    t, y, dy = _second_order(3.0, wn=30.0, zeta=0.3)
    clean = analyze_step(_event(3.0), t, y, dy)
    y = y.copy()
    y[5] = np.nan
    metrics = analyze_step(_event(3.0), t, y, dy)
    assert metrics.overshoot_pct == pytest.approx(clean.overshoot_pct) and clean.overshoot_pct > 30
    assert metrics.settle_time_s == clean.settle_time_s
    failures = metrics.failures()
    assert failures[0] == "+3 deg step has 1 non-finite sample(s)"
    assert any("overshot" in reason for reason in failures)


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


def test_a_stuck_small_step_drops_out_of_the_score_and_a_stuck_large_one_rejects() -> None:
    def step(kp: float, delta: float, moved: float, tau: float = 0.05) -> StepMetrics:
        return analyze_step(_event(delta, joint="j", kp=kp, kd=1.0), *_first_order(moved, tau))

    # The 1 deg steps stick at kp 20 and kp 40; the 3 deg steps move, slowly at kp 20.
    soft = [step(20.0, 1.0, 0.2), step(20.0, -1.0, -0.2)]
    soft += [step(20.0, 3.0, 3.0, 0.1), step(20.0, -3.0, -3.0, 0.1)]
    stiff = [step(40.0, 1.0, 0.3), step(40.0, -1.0, -0.3)]
    stiff += [step(40.0, 3.0, 3.0), step(40.0, -3.0, -1.0)]
    proposal = propose_gains([*soft, *stiff])["j"]
    assert proposal.gains == (20.0, 1.0)
    assert proposal.reason.endswith("; the +1, -1 deg step(s) stuck and do not count")
    # Over its 3 deg steps alone: rests on target, settles in about 0.34 s.
    assert "rests up to 0.00 deg off target" in proposal.reason
    assert proposal.rejected["kp=40 kd=1"].startswith("-3 deg step is stuck: moved +1.00 deg")
    assert "+1 deg step is stuck" not in proposal.rejected["kp=40 kd=1"]


def test_a_rest_error_counts_against_what_friction_leaves_at_that_kp() -> None:
    def steps(kp: float, short_deg: float) -> list[StepMetrics]:
        out = []
        for delta in (1.0, -1.0):
            t, y, dy = _first_order(delta, 0.05)
            event = _event(delta, joint="j", kp=kp, kd=1.0)
            out.append(analyze_step(event, t, y - delta * short_deg, dy, friction_nm=0.2))
        return out

    # 0.2 N m of friction leaves 0.86 deg at kp 20 and 0.21 deg at kp 80: kp 20
    # resting 0.4 deg short beats kp 80 resting 0.3 deg short.
    proposal = propose_gains([*steps(20.0, 0.4), *steps(80.0, 0.3)])["j"]
    assert proposal.gains == (20.0, 1.0) and proposal.friction_nm == 0.2
    assert "rests up to 0.40 deg off target (limit 0.86 deg, friction 0.20 Nm)" in (proposal.reason)
    assert proposal.rejected["kp=80 kd=1"].startswith("passes, but scores 1.")


def test_overshoot_at_the_kd_limit_says_to_lower_kp() -> None:
    def ringing(kp: float, kd: float) -> list[StepMetrics]:
        return [
            analyze_step(_event(d, joint="j", kp=kp, kd=kd), *_second_order(d, 60.0, 0.2))
            for d in (3.0, -3.0)
        ]

    capped = propose_gains([*ringing(80.0, 2.0), *ringing(80.0, KD_LIMIT), *ringing(120.0, 5.0)])
    assert capped["j"].gains is None
    assert capped["j"].reason == (
        "overshoots more than 20 % at every kp; kd at the driver limit 5; lower kp"
    )
    assert "overshot" in capped["j"].rejected["kp=120 kd=5"]
    below = propose_gains([*ringing(80.0, 2.0), *ringing(80.0, 3.5)])["j"]
    assert below.reason == "no candidate passed every step"


def test_gain_candidates_are_a_kp_by_kd_grid_with_the_robot_gains_first() -> None:
    pairs = gain_candidates(80.0, 1.0, (0.6, 1.0, 1.5), (1.0, 2.0, 3.5))
    assert pairs[0] == (80.0, 1.0) and len(pairs) == 9
    assert set(pairs) == {(kp, kd) for kp in (48.0, 80.0, 120.0) for kd in (1.0, 2.0, 3.5)}
    wrist = gain_candidates(10.0, 1.5, (0.6, 1.0, 1.5), (1.0, 2.0, 3.5))
    assert {kd for _, kd in wrist} == {1.5, 3.0, KD_LIMIT}
    clamped = gain_candidates(80.0, 5.0, (0.6, 1.0, 1.5), (1.0, 2.0, 3.5))
    assert clamped == [(80.0, 5.0), (48.0, 5.0), (120.0, 5.0)]


def test_friction_comes_from_the_largest_steps_at_the_first_gains() -> None:
    def step(kp: float, delta: float, error: float) -> StepMetrics:
        t, y, dy = _first_order(delta, 0.05)
        event = _event(delta, joint="j", kp=kp, kd=1.0)
        return analyze_step(event, t, y + error, dy)

    # Friction stops +3 short by 0.6 deg and -3 short by 0.4 deg (a 0.1 deg sag):
    # kp 80 x 0.5 deg is 0.70 N m; the second gains do not count.
    swept = [step(80.0, 1.0, -0.3), step(80.0, 3.0, -0.6), step(80.0, -3.0, 0.4)]
    swept += [step(120.0, 3.0, -2.0), step(120.0, -3.0, 2.0)]
    assert sweep_friction_nm(swept) == {"j": pytest.approx(80.0 * math.radians(0.5), abs=1e-3)}
    pushed = [step(80.0, 3.0, 0.2), step(80.0, -3.0, -0.2)]
    assert sweep_friction_nm(pushed) == {"j": 0.0}
    assert sweep_friction_nm([step(80.0, 3.0, -0.6)]) == {}


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


def test_a_gain_pair_lost_to_a_recording_gap_is_rejected_and_the_rest_compete() -> None:
    hold, tau = HOLD_S - 2 * DT, 0.05
    events = [
        _event(1.0, kp=kp, t0=t0, hold_s=hold)
        if delta > 0
        else _event(-1.0, kp=kp, t0=t0, hold_s=hold, target_before_deg=11.0, target_after_deg=10.0)
        for kp, delta, t0 in (
            (80.0, 1.0, 100.0),
            (80.0, -1.0, 102.0),
            (200.0, 1.0, 104.0),
            (200.0, -1.0, 106.0),
        )
    ]
    t = 104.0 + np.arange(0.0, 2 * HOLD_S, DT)
    rising = t < 106.0
    since = np.where(rising, t - 104.0, t - 106.0)
    decay = np.exp(-since / tau)
    q = np.where(rising, 11.0 - decay, 10.0 + decay)
    dq = np.where(rising, decay / tau, -decay / tau)
    trace = JointTrace(t=t, q_deg=q, dq_deg_s=dq)
    recording = Recording(traces={"left_joint1": trace}, events=events, t_end=float(t[-1]))

    records = extract_steps(recording)

    assert [r.metrics.samples == 0 for r in records] == [True, True, False, False]
    assert records[0].metrics.failures() == ["+1 deg step has no recorded samples"]
    assert records[0].t.size == 0 and not records[0].metrics.passed
    assert records[2].metrics.passed and records[3].metrics.passed
    proposal = propose_gains(r.metrics for r in records)["left_joint1"]
    assert proposal.gains == (200.0, 5.0)
    assert "has no recorded samples" in proposal.rejected["kp=80 kd=5"]


def test_a_step_takes_its_joints_dither_friction_when_the_sweep_shows_less() -> None:
    [alone] = extract_steps(_recorded_step({}))
    assert alone.metrics.friction_nm == 0.0 and alone.metrics.rest_limit_deg == SETTLE_BAND_DEG
    [record] = extract_steps(_recorded_step({}), dither_friction_nm={"left_joint1": 0.4})
    assert record.metrics.friction_nm == 0.4
    assert record.metrics.rest_limit_deg == pytest.approx(math.degrees(1.5 * 0.4 / 80.0))


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
