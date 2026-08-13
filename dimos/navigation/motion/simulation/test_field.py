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

"""The field-pair loader's pure parts: windowing, alignment, anchoring, bins."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.navigation.motion.adapter.diagnose import parse_instant, stamp_dialect
from dimos.navigation.motion.simulation import metrics
from dimos.navigation.motion.simulation.field import (
    REGIME_BINS,
    Bin,
    _bins,
    anchor,
    choose_lag,
    clock_offset,
    dead_reckon,
    direct_lag,
    held,
    joint_fault,
    joint_limits,
    regime,
    relative_window,
    sidecar_for,
)


def _yaw_quat(yaw: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [np.cos(yaw / 2), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2)]
    )


# ------------------------------------------------------------------ pairing --


def test_sidecar_is_the_same_stem_without_the_zenoh_infix():
    assert sidecar_for("data/x/20260806-063428.zenoh.mcap").name == "20260806-063428.mcap"


def test_a_non_pair_recording_says_so_instead_of_guessing():
    with pytest.raises(ValueError, match="not a .*zenoh.mcap"):
        sidecar_for("data/x/plain.mcap")


# ---------------------------------------------------------------- windowing --


def test_relative_seconds_come_back_as_seconds_from_the_start():
    w = relative_window(parse_instant("6.9"), parse_instant("9.0"), t0=1785969268.18)
    assert w.lo == pytest.approx(6.9)
    assert w.hi == pytest.approx(9.0)


def test_a_utc_time_of_day_resolves_against_the_recording_start():
    """`--from 22:34:35.4` on a run that began at 22:34:28.18 is 7.22 s in."""
    t0 = 1785969268.18  # 22:34:28.18 UTC, the 063428 field pair's first command
    w = relative_window(parse_instant("22:34:35.400"), None, t0)
    assert w.lo == pytest.approx(7.22, abs=1e-3)
    assert w.hi == math.inf


def test_no_bounds_keeps_every_sample():
    w = relative_window(None, None, t0=100.0)
    assert not w.bounded
    assert w.mask(np.array([-5.0, 0.0, 1e6])).all()


def test_the_window_selects_only_the_samples_inside_it():
    w = relative_window(parse_instant("2"), parse_instant("4"), t0=0.0)
    np.testing.assert_array_equal(
        w.mask(np.array([1.0, 2.0, 3.0, 4.0, 5.0])), [False, True, True, True, False]
    )


# ---------------------------------------------------------------- alignment --


def test_cross_correlation_recovers_a_known_shift():
    t = np.arange(0.0, 20.0, 0.01)
    x = np.sin(2 * np.pi * 0.7 * t) + 0.3 * np.sin(2 * np.pi * 0.11 * t)
    # a is the same signal stamped 170 ms late
    assert clock_offset(t + 0.17, x, t, x) == pytest.approx(0.17, abs=0.011)


def test_a_stream_stamped_early_reports_a_negative_lag():
    t = np.arange(0.0, 20.0, 0.01)
    x = np.sin(2 * np.pi * 0.7 * t)
    assert clock_offset(t - 0.08, x, t, x) == pytest.approx(-0.08, abs=0.011)


def test_a_constant_stream_has_no_lag_to_find():
    t = np.arange(0.0, 20.0, 0.01)
    assert clock_offset(t, np.ones_like(t), t, np.sin(t)) == 0.0


def test_too_short_an_overlap_refuses_to_guess():
    t = np.arange(0.0, 1.0, 0.01)
    assert clock_offset(t, np.sin(t), t, np.sin(t), max_lag=1.0) == 0.0


# ------------------------------------------------- which lag times the track --


def _odom_dialect(age, n=200, t0=1_785_969_268.0):
    ts = t0 + np.arange(n) / 30.0
    return stamp_dialect(ts, ts - age)


def test_an_old_recording_has_only_the_correlation():
    foreign = _odom_dialect(1_785_861_881.0)
    assert direct_lag(foreign) is None
    assert choose_lag(0.17, None, None) == (0.17, "correlation")


def test_a_correlation_inside_the_grid_resolution_is_not_applied():
    assert choose_lag(0.02, None, None) == (0.0, "correlation")


def test_sensor_time_stamps_time_the_track_when_they_agree():
    d = _odom_dialect(0.16)
    assert direct_lag(d) == pytest.approx(0.16)
    lag, source = choose_lag(0.17, direct_lag(d), None)
    assert (lag, source) == (pytest.approx(0.16), "stamps")


def test_stamps_far_off_the_correlation_lose_to_it():
    # the correlation is anchored to motion the robot actually made
    lag, source = choose_lag(0.17, 0.02, None)
    assert lag == 0.17
    assert source.startswith("correlation")


def test_an_explicit_lag_beats_both():
    assert choose_lag(0.17, 0.16, 0.0) == (0.0, "override")


def test_stamps_that_do_not_advance_are_not_a_clock():
    ts = 1_785_969_268.0 + np.arange(50) / 30.0
    jumbled = ts - 0.1
    jumbled[20] += 5.0  # one stamp out of order
    assert direct_lag(stamp_dialect(ts, jumbled)) is None


# ---------------------------------------------------------------- anchoring --


def test_anchoring_puts_the_anchor_sample_at_the_origin_facing_x():
    pos = np.array([[5.0, -3.0, 1.2], [6.0, -3.0, 1.25]])
    quat = _yaw_quat(np.array([np.pi / 2, np.pi / 2]))
    p, q = anchor(pos, quat, at=0, height=0.32)
    np.testing.assert_allclose(p[0], [0.0, 0.0, 0.32], atol=1e-9)
    assert metrics.yaw_of(q)[0] == pytest.approx(0.0, abs=1e-9)


def test_anchoring_rotates_travel_into_the_body_frame_of_the_anchor():
    """Facing +y and walking +y is walking straight ahead once anchored."""
    pos = np.array([[0.0, 0.0, 0.3], [0.0, 1.0, 0.3]])
    quat = _yaw_quat(np.array([np.pi / 2, np.pi / 2]))
    p, _ = anchor(pos, quat, at=0)
    np.testing.assert_allclose(p[1, :2], [1.0, 0.0], atol=1e-9)


def test_anchoring_keeps_relative_heading_changes():
    quat = _yaw_quat(np.array([1.0, 1.4]))
    _, q = anchor(np.zeros((2, 3)), quat, at=0)
    assert metrics.yaw_of(q)[1] == pytest.approx(0.4, abs=1e-9)


def test_anchoring_at_a_later_sample_still_zeroes_that_one():
    pos = np.array([[0.0, 0.0, 0.3], [1.0, 0.0, 0.3], [2.0, 0.0, 0.3]])
    quat = _yaw_quat(np.zeros(3))
    p, _ = anchor(pos, quat, at=1, height=0.32)
    np.testing.assert_allclose(p[1], [0.0, 0.0, 0.32], atol=1e-9)
    np.testing.assert_allclose(p[2, 0], 1.0, atol=1e-9)


# -------------------------------------------------------------- zero-order --


def test_a_command_holds_until_the_next_one_arrives():
    cmd_t = np.array([0.0, 1.0, 2.0])
    cmd = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    out = held(np.array([0.5, 1.0, 1.9, 5.0]), cmd_t, cmd)
    np.testing.assert_array_equal(out, [cmd[0], cmd[1], cmd[1], cmd[2]])


def test_samples_before_the_first_command_take_the_first_one():
    cmd_t = np.array([2.0, 3.0])
    cmd = np.array([[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
    np.testing.assert_array_equal(held(np.array([0.0]), cmd_t, cmd), [cmd[0]])


# ------------------------------------------------------------------- bands --


def test_bands_split_at_the_declared_bounds():
    out = regime(np.array([0.0, 0.39, 0.4, 0.89, 0.9, 3.0]))
    assert list(out) == ["gentle", "gentle", "moderate", "moderate", "aggressive", "aggressive"]


def test_every_sample_lands_in_exactly_one_band():
    out = regime(np.abs(np.random.default_rng(0).normal(0, 1.0, 500)))
    assert set(out) <= {name for name, _ in REGIME_BINS}
    assert None not in set(out)


# ----------------------------------------------------------- dead reckoning --


def test_a_pure_spin_goes_nowhere():
    t = np.arange(0.0, 2.0, 0.01)
    cmd = np.tile([0.0, 0.0, 1.5], (len(t), 1))
    path = dead_reckon(t, cmd)
    np.testing.assert_allclose(path[-1, :2], [0.0, 0.0], atol=1e-9)
    assert path[-1, 2] == pytest.approx(1.5 * (t[-1] - t[0]), abs=0.02)


def test_a_pure_forward_command_is_a_straight_line():
    t = np.arange(0.0, 2.0, 0.01)
    path = dead_reckon(t, np.tile([0.5, 0.0, 0.0], (len(t), 1)))
    assert path[-1, 0] == pytest.approx(0.5 * t[-1], abs=0.01)
    np.testing.assert_allclose(path[:, 1], 0.0, atol=1e-12)


def test_strafing_moves_sideways_in_the_body_frame():
    t = np.arange(0.0, 1.0, 0.01)
    path = dead_reckon(t, np.tile([0.0, 0.4, 0.0], (len(t), 1)))
    assert path[-1, 1] == pytest.approx(0.4 * t[-1], abs=0.01)


def test_forward_plus_yaw_closes_a_circle_of_the_right_radius():
    """vx / wz is the turn radius; a full 2*pi returns to the start."""
    vx, wz = 0.5, 1.0
    t = np.arange(0.0, 2 * np.pi / wz, 0.002)
    path = dead_reckon(t, np.tile([vx, 0.0, wz], (len(t), 1)))
    assert np.abs(path[:, :2]).max() == pytest.approx(2 * vx / wz, abs=0.02)
    np.testing.assert_allclose(path[-1, :2], [0.0, 0.0], atol=0.02)


# ----------------------------------------------------------- joint validity --


def test_a_plausible_standing_posture_is_accepted():
    q = np.tile([0.0, 0.9, -1.8] * 4, (100, 1))
    dq = np.full_like(q, 0.01)
    tau = np.full_like(q, 1.0)
    assert joint_fault(q, dq, tau, joint_limits()) is None


def test_a_misframed_decode_is_rejected_rather_than_scored():
    """A four-byte-late CDR frame reads q off dq: nonzero angles, no dq, no torque.

    A presence check passes it and the resulting joint RMS reads exactly like a
    wrong motor permutation, so the check is physical, not statistical.
    """
    rng = np.random.default_rng(0)
    q = rng.normal(0.0, 2.0, (2000, 12))
    zero = np.zeros_like(q)
    fault = joint_fault(q, zero, zero, joint_limits())
    assert fault is not None
    assert "no joint velocity or torque" in fault
    assert "CDR frame" in fault


def test_a_real_standing_recording_passes_the_guard():
    """The 063428 sidecar's actual stance, once the CDR frame is right."""
    q = np.tile([0.033, 0.717, -1.566] * 4, (100, 1))
    alive = np.tile([0.03, -0.01, 0.05] * 4, (100, 1))
    tau = np.tile([-0.89, -0.05, 7.44] * 4, (100, 1))
    assert joint_fault(q, alive, tau, joint_limits()) is None


def test_out_of_travel_angles_are_rejected_even_with_telemetry_beside_them():
    q = np.tile([3.0, 0.9, -1.8] * 4, (100, 1))  # hip at 3 rad, travel is +-1.05
    live = np.full_like(q, 0.5)
    fault = joint_fault(q, live, live, joint_limits())
    assert fault is not None
    assert "outside the Go2 joint travel" in fault


# -------------------------------------------------------------------- bins --


def _binned(cmd_grid: np.ndarray, real_yr: np.ndarray, sim_yr: np.ndarray) -> list[Bin]:
    n = len(cmd_grid)
    grid = np.arange(n) * 0.02
    zero = np.zeros((n, 2))
    keep = np.ones(n, bool)
    bins, _stand = _bins(grid, cmd_grid, zero, real_yr, zero, sim_yr, np.zeros(n), keep)
    return [b for b in bins if b.axis == "wz"]


def test_a_band_only_scores_the_samples_commanded_into_it():
    cmd = np.zeros((200, 3))
    cmd[:100, 2] = 0.2  # gentle
    cmd[100:, 2] = 1.2  # aggressive
    out = {b.name: b for b in _binned(cmd, np.zeros(200), np.zeros(200))}
    assert out["gentle"].n == 100
    assert out["aggressive"].n == 100
    assert "moderate" not in out


def test_a_sim_that_misses_twice_as_badly_shows_a_ratio_of_two():
    cmd = np.zeros((100, 3))
    cmd[:, 2] = 1.2
    real = np.full(100, 1.0)  # 0.2 rad/s short
    sim = np.full(100, 0.8)  # 0.4 rad/s short
    (agg,) = _binned(cmd, real, sim)
    assert agg.real_err == pytest.approx(0.2)
    assert agg.sim_err == pytest.approx(0.4)
    assert agg.ratio == pytest.approx(2.0)


def test_standing_samples_are_kept_out_of_the_moving_bands():
    cmd = np.zeros((200, 3))
    cmd[100:, 2] = 1.2
    grid = np.arange(200) * 0.02
    zero = np.zeros((200, 2))
    bins, stand = _bins(
        grid, cmd, zero, np.zeros(200), zero, np.zeros(200), np.zeros(200), np.ones(200, bool)
    )
    assert stand is not None and stand.n == 100
    assert sum(b.n for b in bins if b.axis == "wz") == 100


def test_the_window_keeps_bins_off_the_samples_it_excludes():
    cmd = np.zeros((200, 3))
    cmd[:, 2] = 1.2
    grid = np.arange(200) * 0.02
    zero = np.zeros((200, 2))
    keep = grid >= 2.0
    bins, _ = _bins(grid, cmd, zero, np.zeros(200), zero, np.zeros(200), np.zeros(200), keep)
    assert sum(b.n for b in bins if b.axis == "wz") == int(keep.sum())
