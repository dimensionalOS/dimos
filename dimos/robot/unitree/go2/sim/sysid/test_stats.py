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

"""The eleven chaos-tolerant statistics, on signals with known answers."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.stats import (
    NOT_COMPARABLE,
    _gain,
    gait_frequency,
    pitch_roll_of,
    spread_of,
    summarize,
    velocity,
    yaw_of,
)


def test_velocity_survives_sample_jitter():
    """A fixed-sample window on jittered 250 Hz data once read 3.9 m/s from a
    walking Go2; a window in seconds on a uniform grid does not."""
    rng = np.random.default_rng(0)
    t = np.sort(np.cumsum(rng.uniform(0.001, 0.008, 2000)))
    pos = np.stack([0.5 * t, np.zeros_like(t), np.zeros_like(t)], axis=1)
    _grid, vel = velocity(t, pos)
    speed = np.linalg.norm(vel[:, :2], axis=1)
    assert np.median(speed) == pytest.approx(0.5, abs=0.02)


def test_gait_frequency_finds_the_stride_not_a_harmonic():
    t = np.arange(0, 30, 0.01)
    z = 0.30 + 0.005 * np.sin(2 * np.pi * 2.0 * t) + 0.002 * np.sin(2 * np.pi * 4.0 * t)
    assert gait_frequency(z) == pytest.approx(2.0, abs=0.15)


def test_gait_frequency_does_not_lock_onto_a_noise_ripple():
    """The bimodality fix: a weak first local maximum (a noise ripple near the
    step harmonic) must yield to a much stronger stride peak behind it, so a
    probe's gait_hz is a measurement rather than a draw. This seed made the
    pre-fix estimator read ~3 Hz on a clean 1.5 Hz gait."""
    rng = np.random.default_rng(4)
    t = np.arange(0, 20, 0.01)
    z = 0.01 * np.sin(2 * np.pi * 1.5 * t) + 0.004 * rng.normal(size=len(t))
    assert gait_frequency(z) == pytest.approx(1.5, abs=0.1)


def test_gain_recovers_slope_and_lag():
    rng = np.random.default_rng(0)
    t = np.arange(0, 40, 0.01)
    cmd = np.where(np.sin(2 * np.pi * 0.11 * t) > 0, 0.6, 0.0) + rng.normal(0, 0.01, len(t))
    lag_s = 0.2
    achieved = 0.8 * np.roll(cmd, int(lag_s * 100))
    gain, lag = _gain(achieved, cmd, 0.25)
    assert gain == pytest.approx(0.8, abs=0.05)
    assert lag == pytest.approx(lag_s, abs=0.03)


def test_yaw_and_tilt_conventions():
    # 90 deg yaw, wxyz
    q = np.array([[np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]])
    assert yaw_of(q)[0] == pytest.approx(np.pi / 2)
    p, r = pitch_roll_of(q)
    assert p[0] == pytest.approx(0.0, abs=1e-9) and r[0] == pytest.approx(0.0, abs=1e-9)


def test_summarize_reads_a_synthetic_walk_correctly():
    t = np.arange(0, 20, 0.02)
    pos = np.stack([0.5 * t, np.zeros_like(t), 0.30 + 0.004 * np.sin(2 * np.pi * 2.0 * t)], axis=1)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))
    s = summarize(t, pos, quat, cmd)
    assert s.speed == pytest.approx(0.5, abs=0.03)
    assert s.speed_gain == pytest.approx(1.0, abs=0.06)
    assert s.gait_hz == pytest.approx(2.0, abs=0.15)
    assert s.height_mean == pytest.approx(0.30, abs=0.01)
    assert s.tilt_p99 == pytest.approx(0.0, abs=1e-6)
    assert len(s.as_dict()) == 13
    assert "height_mean" in NOT_COMPARABLE  # room frame: mean is not comparable


def test_summarize_attitude_may_ride_its_own_timeline():
    """The instrument split: position at t, attitude at t_att (a different
    sensor, its own clock and rate). Passing the same timeline explicitly
    must change nothing; a genuine second timeline must be read correctly."""
    t = np.arange(0, 20, 0.02)
    pos = np.stack([0.5 * t, np.zeros_like(t), np.full_like(t, 0.30)], axis=1)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))
    assert summarize(t, pos, quat, cmd) == summarize(t, pos, quat, cmd, t_att=t.copy())

    # attitude at 200 Hz: a 2 Hz roll wobble of 0.05 rad amplitude
    ta = np.arange(0, 20, 0.005)
    phi = 0.05 * np.sin(2 * np.pi * 2.0 * ta)
    quat_a = np.stack([np.cos(phi / 2), np.sin(phi / 2), np.zeros_like(ta), np.zeros_like(ta)], 1)
    s = summarize(t, pos, quat_a, cmd, t_att=ta)
    assert s.speed == pytest.approx(0.5, abs=0.03)  # position: untouched by the wobble
    assert s.roll_std == pytest.approx(0.05 / np.sqrt(2), rel=0.1)
    assert s.pitch_std == pytest.approx(0.0, abs=1e-6)


def test_summarize_without_position_scores_attitude_only():
    """No tracker: position-derived statistics are NaN (not comparable), the
    attitude block still measures."""
    t = np.arange(0, 20, 0.005)
    phi = 0.05 * np.sin(2 * np.pi * 2.0 * t)
    quat = np.stack([np.cos(phi / 2), np.sin(phi / 2), np.zeros_like(t), np.zeros_like(t)], 1)
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))
    s = summarize(t, None, quat, cmd)
    for k in ("speed", "speed_gain", "speed_lag", "height_mean", "height_std", "gait_hz"):
        assert np.isnan(s.as_dict()[k]), k
    assert s.roll_std == pytest.approx(0.05 / np.sqrt(2), rel=0.1)
    assert np.isfinite(s.tilt_p99) and np.isfinite(s.yaw_rate_gain)


def test_spread_of_is_peak_to_peak_per_statistic():
    t = np.arange(0, 10, 0.02)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))

    def run(speed: float):
        pos = np.stack([speed * t, np.zeros_like(t), np.full_like(t, 0.3)], axis=1)
        return summarize(t, pos, quat, cmd)

    spread = spread_of([run(0.4), run(0.5), run(0.6)])
    assert spread["speed"] == pytest.approx(0.2, abs=0.03)
    assert spread["tilt_p99"] == pytest.approx(0.0, abs=1e-9)
