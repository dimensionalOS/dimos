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

"""Filtering and the chaos-tolerant statistics built on it."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.motion.simulation.metrics import (
    Summary,
    _gain,
    chaos_spread,
    resample,
    summarize,
    velocity,
    yaw_of,
)


def test_resample_lands_on_a_uniform_grid():
    t = np.array([0.0, 0.1, 0.35, 0.4, 0.9])  # deliberately jittery
    grid, x = resample(t, t * 2.0, rate=100.0)
    np.testing.assert_allclose(np.diff(grid), 0.01, atol=1e-12)
    np.testing.assert_allclose(x, grid * 2.0, atol=1e-9)


def test_resample_handles_multiple_columns():
    t = np.linspace(0, 1, 37)
    grid, x = resample(t, np.stack([t, -t, 2 * t], 1), rate=50.0)
    assert x.shape == (len(grid), 3)
    np.testing.assert_allclose(x[:, 1], -grid, atol=1e-9)


def test_velocity_recovers_a_constant_speed():
    t = np.linspace(0, 4, 400)
    pos = np.stack([0.5 * t, np.zeros_like(t), np.zeros_like(t)], 1)
    _grid, v = velocity(t, pos)
    mid = slice(len(v) // 4, 3 * len(v) // 4)  # ignore edge effects of the window
    np.testing.assert_allclose(v[mid, 0], 0.5, atol=1e-3)


def test_velocity_is_insensitive_to_sample_rate():
    """A 253 Hz recording and a 50 Hz rollout must give the same answer.

    Using a fixed *sample* window instead of a time window is what made an
    earlier pass report a Go2 walking at 3.9 m/s.
    """
    speeds = []
    for n in (200, 1012):
        t = np.linspace(0, 4, n)
        pos = np.stack([0.4 * t, np.zeros_like(t), np.zeros_like(t)], 1)
        _g, v = velocity(t, pos)
        mid = slice(len(v) // 4, 3 * len(v) // 4)
        speeds.append(v[mid, 0].mean())
    assert abs(speeds[0] - speeds[1]) < 1e-3


def test_velocity_rejects_jitter_noise():
    """Irregular sampling must not inflate the speed estimate."""
    rng = np.random.default_rng(0)
    t = np.cumsum(rng.uniform(0.001, 0.008, 2000))
    pos = np.stack([0.4 * t, np.zeros_like(t), np.zeros_like(t)], 1)
    pos += rng.normal(0, 1e-4, pos.shape)  # tracker noise
    _g, v = velocity(t, pos)
    mid = slice(len(v) // 4, 3 * len(v) // 4)
    assert np.linalg.norm(v[mid, :2], axis=1).mean() == pytest.approx(0.4, abs=0.05)


def test_yaw_of_reads_rotation_about_z():
    half = np.pi / 4  # 90 deg yaw
    q = np.array([[np.cos(half), 0.0, 0.0, np.sin(half)], [1.0, 0.0, 0.0, 0.0]])
    np.testing.assert_allclose(yaw_of(q), [np.pi / 2, 0.0], atol=1e-9)


def test_gain_is_a_slope_not_a_mean_of_ratios():
    """Sign-flipping commands must not cancel — the bug this replaced."""
    cmd = np.tile([1.0, -1.0, 0.8, -0.8], 50)
    gain, lag = _gain(0.6 * cmd, cmd, 0.2)
    assert gain == pytest.approx(0.6)
    assert lag == pytest.approx(0.0)


def test_gain_ignores_commands_below_the_threshold():
    """Near-zero commands are where a ratio estimator explodes."""
    rng = np.random.default_rng(3)
    cmd = np.repeat(rng.choice([-1.0, 1.0, 0.01], 80), 15).astype(float)
    achieved = 0.5 * cmd
    achieved[np.abs(cmd) < 0.2] = 4.0  # nonsense where nothing was commanded
    assert _gain(achieved, cmd, 0.2)[0] == pytest.approx(0.5, abs=0.03)


def test_gain_returns_zero_when_nothing_is_commanded():
    assert _gain(np.ones(5), np.zeros(5), 0.2) == (0.0, 0.0)


def test_summarize_on_a_synthetic_walk():
    t = np.linspace(0, 20, 2000)
    pos = np.stack([0.4 * t, np.zeros_like(t), 0.30 + 0.02 * np.sin(2 * np.pi * 2.0 * t)], 1)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))
    s = summarize(t, pos, quat, cmd)

    assert s.speed == pytest.approx(0.4, abs=0.02)
    assert s.speed_gain == pytest.approx(0.8, abs=0.05)
    assert s.height_mean == pytest.approx(0.30, abs=0.01)
    assert s.height_std == pytest.approx(0.02 / np.sqrt(2), abs=0.005)
    assert s.gait_hz == pytest.approx(2.0, abs=0.2)


def test_summarize_reads_turn_rate_with_the_right_sign():
    t = np.linspace(0, 10, 1000)
    rate = 0.5
    yaw = rate * t
    quat = np.stack([np.cos(yaw / 2), np.zeros_like(t), np.zeros_like(t), np.sin(yaw / 2)], axis=1)
    pos = np.stack([np.zeros_like(t), np.zeros_like(t), np.full_like(t, 0.3)], 1)
    cmd = np.tile([0.0, 0.0, 1.0], (len(t), 1))
    assert summarize(t, pos, quat, cmd).yaw_rate_gain == pytest.approx(rate, abs=0.05)


def test_height_std_ignores_a_slow_drift():
    """A sag, a tilted room frame, or a gait-height ramp must not read as bob."""
    t = np.linspace(0, 20, 2000)
    bob = 0.02 * np.sin(2 * np.pi * 2.0 * t)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    cmd = np.tile([0.5, 0.0, 0.0], (len(t), 1))

    flat = summarize(t, np.stack([0.4 * t, np.zeros_like(t), 0.30 + bob], 1), quat, cmd)
    drifting = summarize(
        t, np.stack([0.4 * t, np.zeros_like(t), 0.30 + bob + 0.005 * t], 1), quat, cmd
    )
    assert drifting.height_std == pytest.approx(flat.height_std, abs=0.002)


def test_pitch_roll_std_reads_the_oscillation_not_the_mount_bias():
    """A constant tilt (mount or room error) must not count; the wobble must."""
    t = np.linspace(0, 20, 2000)
    amp, bias = 0.05, 0.3
    pitch = bias + amp * np.sin(2 * np.pi * 2.0 * t)
    quat = np.stack(
        [np.cos(pitch / 2), np.zeros_like(t), np.sin(pitch / 2), np.zeros_like(t)], axis=1
    )
    pos = np.stack([np.zeros_like(t), np.zeros_like(t), np.full_like(t, 0.3)], 1)
    s = summarize(t, pos, quat, np.tile([0.5, 0.0, 0.0], (len(t), 1)))
    assert s.pitch_std == pytest.approx(amp / np.sqrt(2), abs=0.005)
    assert s.roll_std == pytest.approx(0.0, abs=1e-6)


def test_chaos_spread_is_peak_to_peak_per_statistic():
    def mk(speed):
        return Summary(
            speed=speed,
            speed_gain=1.0,
            yaw_rate_gain=0.0,
            height_mean=0.3,
            height_std=0.01,
            gait_hz=2.0,
            speed_lag=0.1,
            yaw_lag=0.2,
            pitch_std=0.02,
            roll_std=0.02,
            tilt_p99=0.06,
        )

    spread = chaos_spread([mk(0.40), mk(0.44), mk(0.42)])
    assert spread["speed"] == pytest.approx(0.04)
    assert spread["gait_hz"] == 0.0


def test_walk_rejects_a_start_past_the_end_of_the_commands():
    """--start beyond the recording used to die with an IndexError on an empty run."""
    from dimos.navigation.motion.simulation.walk import walk

    class _Stub:
        hist, obs_per_frame, act_dim = 1, 45, 12
        default_pose = np.zeros(12)

    schedule = (np.array([0.0, 1.0, 2.0]), np.zeros((3, 3)))
    with pytest.raises(ValueError, match="past the end of the commands"):
        walk(_Stub(), schedule=schedule, start=5.0)  # type: ignore[arg-type]


def test_walk_requires_exactly_one_command_source():
    from dimos.navigation.motion.simulation.walk import walk

    class _Stub:
        hist, obs_per_frame, act_dim = 1, 45, 12
        default_pose = np.zeros(12)

    with pytest.raises(ValueError, match="exactly one"):
        walk(_Stub())  # type: ignore[arg-type]


def test_start_offset_reaches_the_command_schedule():
    """Regression: --start must shift the commands, not only the ghost.

    A failed patch once left cmd_at() reading from t while the ghost read from
    t + start, so every comparison drove the simulator with the first seconds
    of a run and scored it against a ghost six seconds later. It looked
    plausible -- the robot walked -- and silently invalidated every number.
    """
    from dimos.navigation.motion.simulation import walk as walk_mod

    class FakePolicy:
        hist, act_dim, obs_per_frame = 1, 12, 45
        default_pose = np.zeros(12)
        kp, kd = np.full(12, 40.0), np.full(12, 1.0)

        def normalize(self, raw):
            return raw

        def act(self, _p_obs, _cmd):
            return np.zeros(12), self.default_pose

    t = np.arange(0.0, 10.0, 0.1)
    cmd = np.stack([np.zeros_like(t), np.zeros_like(t), t], 1)  # vyaw ramps with time
    track = walk_mod.walk(FakePolicy(), schedule=(t, cmd), seconds=2.0, start=5.0)

    # at start=5 the commanded vyaw must begin near 5.0, not near 0.0
    assert track.cmd[0, 2] == pytest.approx(5.0, abs=0.2)


def test_gain_recovers_a_delayed_response():
    """A lagged response must still read its true gain."""
    rng = np.random.default_rng(0)
    cmd = np.repeat(rng.choice([-0.8, 0.8], 60), 20).astype(float)
    lag_samples = 25
    achieved = np.concatenate([np.zeros(lag_samples), 0.7 * cmd])[: len(cmd)]
    gain, lag = _gain(achieved, cmd, 0.2, rate=100.0)
    assert gain == pytest.approx(0.7, abs=0.05)
    assert lag == pytest.approx(lag_samples / 100.0, abs=0.02)


def test_command_slew_ramps_a_step_at_the_hardware_rate():
    """The policy must see the robot's ramped command, not the operator step:
    vyaw moves at most 0.10 rad/s per 20 ms tick (go2web VEL_DV_VYAW)."""
    from dimos.navigation.motion.simulation import walk as walk_mod

    class FakePolicy:
        hist, act_dim, obs_per_frame = 1, 12, 45
        default_pose = np.zeros(12)
        kp, kd = np.full(12, 40.0), np.full(12, 1.0)

        def normalize(self, raw):
            return raw

        def act(self, _p_obs, _cmd):
            return np.zeros(12), self.default_pose

    t = np.arange(0.0, 4.0, 0.1)
    cmd = np.stack([np.zeros_like(t), np.zeros_like(t), np.where(t < 2.0, -1.0, 1.0)], 1)
    track = walk_mod.walk(FakePolicy(), schedule=(t, cmd), seconds=4.0)

    applied = track.cmd[:, 2]
    assert applied[0] == pytest.approx(-1.0)
    assert np.abs(np.diff(applied)).max() == pytest.approx(0.10, abs=1e-9)
    # -1 -> +1 takes 20 ticks = 0.4 s of ramping
    i = int(np.searchsorted(track.t, 2.0))
    assert applied[i + 9] == pytest.approx(0.0, abs=0.11)
    assert applied[i + 25] == pytest.approx(1.0)

    unslewed = walk_mod.walk(FakePolicy(), schedule=(t, cmd), seconds=4.0, slew=False)
    assert np.abs(np.diff(unslewed.cmd[:, 2])).max() == pytest.approx(2.0)


def test_gait_height_schedule_reaches_obs_channel_45():
    """A 46-channel policy must see the commanded height at index 45, held at
    nominal before the first entry; a 45-channel policy must see nothing --
    the frame stays 45 wide."""
    from dimos.navigation.motion.simulation import walk as walk_mod

    class FakePolicy:
        hist, act_dim, obs_per_frame = 1, 12, 46
        default_pose = np.zeros(12)
        kp, kd = np.full(12, 40.0), np.full(12, 1.0)

        def __init__(self):
            self.frames = []

        def normalize(self, raw):
            self.frames.append(raw.copy())
            return raw

        def act(self, _p_obs, _cmd):
            return np.zeros(12), self.default_pose

    t = np.arange(0.0, 4.0, 0.1)
    cmd = np.stack([np.zeros_like(t)] * 3, 1)
    heights = (np.array([2.0]), np.array([0.15]))
    policy = FakePolicy()
    track = walk_mod.walk(policy, schedule=(t, cmd), heights=heights, seconds=4.0)

    # frames = hist warm-up, then one per control tick past the settle window
    stamped = np.array([f[45] for f in policy.frames[policy.hist :]])
    grid = track.t[track.t >= 0.5][: len(stamped)]
    assert len(stamped) == len(grid)
    assert np.all(stamped[grid < 1.9] == pytest.approx(walk_mod.NOMINAL_GAIT_HEIGHT))
    assert np.all(stamped[grid > 2.1] == pytest.approx(0.15))

    narrow = FakePolicy()
    narrow.obs_per_frame = 45
    walk_mod.walk(narrow, schedule=(t, cmd), heights=heights, seconds=1.0)
    assert narrow.frames and all(f.size == 45 for f in narrow.frames)


def test_actuator_step_is_a_pass_through_when_ideal():
    from dimos.navigation.motion.simulation.walk import actuator_step

    req = np.array([1.0, -2.0, 3.0])
    out = actuator_step(np.zeros(3), req, 0.002, 0.0)
    np.testing.assert_allclose(out, req)


def test_actuator_step_reaches_63_percent_after_one_time_constant():
    from dimos.navigation.motion.simulation.walk import actuator_step

    dt, tau = 0.0005, 0.005
    applied, req = np.zeros(1), np.ones(1)
    for _ in range(int(tau / dt)):
        applied = actuator_step(applied, req, dt, tau)
    assert applied[0] == pytest.approx(1 - 1 / np.e, abs=0.03)


def test_actuator_step_converges_and_never_overshoots():
    from dimos.navigation.motion.simulation.walk import actuator_step

    applied, req = np.zeros(1), np.full(1, 5.0)
    for _ in range(4000):
        applied = actuator_step(applied, req, 0.002, 0.01)
        assert 0.0 <= applied[0] <= 5.0
    assert applied[0] == pytest.approx(5.0, abs=1e-6)
