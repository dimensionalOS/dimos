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

"""The loop measurements: transport leg, control timing, sensor noise, sweep.

The recording-gated tests pin the 2026-08-16 freewalk session's measured
values — they are MEASUREMENTS, so a drift means either the instrument
changed or the file did, and both deserve a loud failure.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ground import ObsNoise
from dimos.robot.unitree.go2.sim.sysid.loop import (
    ControlTiming,
    SensorNoise,
    match_commands,
)

FREEWALK = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
HANGING = Path.home() / "recordings/hard_floor/20260816-185220_sport-hanging_novive.mcap"
FREEWALK_BIN = Path(__file__).parents[6] / "data/ml-trajectory-research/freewalk_mcf.bin"

needs_freewalk = pytest.mark.skipif(not FREEWALK.is_file(), reason=f"no recording at {FREEWALK}")
needs_hanging = pytest.mark.skipif(not HANGING.is_file(), reason=f"no recording at {HANGING}")


# ------------------------------------------------------------ pure functions


def test_match_commands_takes_the_first_occurrence_at_or_after():
    src_t = np.array([0.0, 1.0])
    dst_t = np.array([0.5, 1.2, 3.0])
    d, unmatched = match_commands(src_t, [b"a", b"b"], dst_t, [b"a", b"b", b"a"])
    assert unmatched == 0
    assert d == pytest.approx([0.5, 0.2])


def test_match_commands_repeated_payloads_cannot_steal_an_earlier_match():
    # the same payload sent twice: the second emission must match the LATER echo
    src_t = np.array([0.0, 2.0])
    dst_t = np.array([0.1, 2.1])
    d, unmatched = match_commands(src_t, [b"x", b"x"], dst_t, [b"x", b"x"])
    assert unmatched == 0
    assert d == pytest.approx([0.1, 0.1])


def test_match_commands_counts_the_unmatched_instead_of_guessing():
    d, unmatched = match_commands(np.array([0.0, 1.0]), [b"a", b"z"], np.array([0.2]), [b"a"])
    assert unmatched == 1
    assert d == pytest.approx([0.2])


def test_control_timing_statistics_and_dropouts():
    ct = ControlTiming(intervals=np.array([0.020, 0.022, 0.024, 0.089]))
    assert ct.mean_ms == pytest.approx(38.75)
    assert ct.median_ms == pytest.approx(23.0)
    assert ct.n_dropouts == 1
    assert "1 dropouts" in ct.describe()


def test_sensor_noise_obs_levels_are_rms_times_sqrt3():
    sn = SensorNoise(
        dq=np.full(12, 0.3), gyro=np.full(3, 0.07), q=np.full(12, 0.003), gravity=np.zeros(3)
    )
    n = sn.obs_noise()
    assert n.dq == pytest.approx(0.3 * np.sqrt(3))
    assert n.gyro == pytest.approx(0.07 * np.sqrt(3))
    assert n.q == pytest.approx(0.003 * np.sqrt(3))
    assert n.gravity == 0.0
    # the point of measuring: well under the training levels on every channel
    train = ObsNoise()
    assert n.dq < train.dq and n.gyro < train.gyro


def test_sensor_noise_recovers_a_known_floor_and_ignores_the_gait_band():
    from dimos.robot.unitree.go2.sim.sysid.loop import sensor_noise
    from dimos.robot.unitree.go2.sim.sysid.test_real import _streams

    rng = np.random.default_rng(0)
    t = np.arange(0, 10, 1 / 500)
    gait = 0.5 * np.sin(2 * np.pi * 2.5 * t)  # in-band signal, must not count
    white = rng.normal(0.0, 0.1, (len(t), 12))
    st = _streams(np.zeros(0), np.zeros((0, 3)))
    st.lt = t
    st.ldq = gait[:, None] + white
    st.lq = np.zeros((len(t), 12))
    st.lgyro = np.zeros((len(t), 3))
    st.lquat = np.tile([1.0, 0, 0, 0], (len(t), 1))
    sn = sensor_noise(st, t0=0.0, t1=10.0)
    # white noise at 0.1 RMS: the >20 Hz band holds sqrt(1 - 20/250) of it
    assert sn.dq_rms == pytest.approx(0.1 * np.sqrt(1 - 20 / 250), rel=0.05)
    assert sn.q_rms == 0.0


def test_timing_of_windows_the_ingested_command_clock():
    from dimos.robot.unitree.go2.sim.sysid.loop import timing_of
    from dimos.robot.unitree.go2.sim.sysid.test_real import _streams

    st = _streams(np.zeros(0), np.zeros((0, 3)))
    st.ct = np.arange(0.0, 2.0, 0.022)
    ct = timing_of(st, t0=0.5, t1=1.5)
    assert ct.mean_ms == pytest.approx(22.0)
    assert len(ct.intervals) < len(st.ct) - 1


def test_timing_of_refuses_a_sport_command_clock():
    """ct at 500 Hz is rt/lowcmd — the builtin controller's clock, not ours."""
    from dimos.robot.unitree.go2.sim.sysid.loop import timing_of
    from dimos.robot.unitree.go2.sim.sysid.test_real import _streams

    st = _streams(np.zeros(0), np.zeros((0, 3)))
    st.ct = np.arange(0.0, 2.0, 0.002)
    with pytest.raises(ValueError, match="sport"):
        timing_of(st)


def test_sensor_noise_refuses_a_starved_window():
    from dimos.robot.unitree.go2.sim.sysid.loop import sensor_noise
    from dimos.robot.unitree.go2.sim.sysid.test_real import _streams

    st = _streams(np.zeros(0), np.zeros((0, 3)))
    with pytest.raises(ValueError, match="samples"):
        sensor_noise(st, t0=0.0, t1=1.0)


# ------------------------------------------------- against the real recording


@needs_freewalk
def test_the_transport_leg_measures_1_3_ms_on_the_freewalk_session():
    from dimos.robot.unitree.go2.sim.sysid.loop import transport_leg

    leg = transport_leg(FREEWALK)
    assert leg.n_commands == 3539
    assert len(leg.delta) == 3539 and leg.n_unmatched == 0
    assert leg.median_ms == pytest.approx(1.338, abs=0.01)
    assert leg.percentile_ms(10) == pytest.approx(1.286, abs=0.01)
    assert leg.percentile_ms(90) == pytest.approx(1.475, abs=0.01)


@needs_freewalk
def test_the_executor_runs_at_44_hz_not_50():
    """The headline timing measurement: the '50 Hz' loop's real mean interval
    is 22.6 ms, with a dropout tail — not jitter around 20 ms."""
    from dimos.robot.unitree.go2.sim.sysid.loop import control_timing

    ct = control_timing(FREEWALK)
    assert ct.mean_ms == pytest.approx(22.56, abs=0.05)
    assert ct.median_ms == pytest.approx(22.27, abs=0.05)
    assert ct.rate_hz == pytest.approx(44.3, abs=0.2)
    assert ct.n_dropouts == 71


@needs_hanging
def test_the_loop_instruments_refuse_a_sport_recording():
    """No policy/lowcmd -> no transport pair and no executor timing: refuse,
    never guess."""
    from dimos.robot.unitree.go2.sim.sysid.loop import control_timing, transport_leg

    with pytest.raises(ValueError, match="BOTH"):
        transport_leg(HANGING)
    with pytest.raises(ValueError, match="policy/lowcmd"):
        control_timing(HANGING)


@needs_freewalk
def test_the_measured_sensor_floor_sits_under_the_training_levels():
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
    from dimos.robot.unitree.go2.sim.sysid.loop import sensor_noise

    sn = sensor_noise(read_streams(FREEWALK), t0=20.0, t1=50.0)
    assert sn.dq_rms == pytest.approx(0.288, abs=0.005)
    assert sn.gyro_rms == pytest.approx(0.0726, abs=0.001)
    assert sn.q_rms == pytest.approx(0.00318, abs=0.0002)
    assert sn.gravity_rms < 0.001  # the attitude filter smooths gravity clean
    n, train = sn.obs_noise(), ObsNoise()
    assert n.dq < 0.4 * train.dq
    assert n.gyro < 0.7 * train.gyro


# ------------------------------------------------------------ needs MuJoCo


@pytest.mark.go2sim
@needs_freewalk
def test_the_command_shift_sweep_reproduces_and_prefers_zero_over_30ms():
    """The open-loop latency probe: deterministic to full precision, and the
    recorded timeline beats a +30 ms delay — the target->plant leg is ~0."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
    from dimos.robot.unitree.go2.sim.sysid.loop import command_shift_sweep

    # The FROZEN acceptance plant, not the live preset: the pinned numbers
    # check determinism of the sweep, and must not move when draw selection
    # re-ships the plant (test_replay.acceptance_plant says why).
    from dimos.robot.unitree.go2.sim.sysid.test_replay import acceptance_plant

    st = read_streams(FREEWALK)
    preset = acceptance_plant()
    backend = MujocoBackend(envelope=TORQUE_ENVELOPES[preset.envelope])
    p0, p30 = command_shift_sweep(st, backend, (0.0, 30.0), t0=20.0, duration=10.0, preset=preset)
    # Re-based 2026-08-17 (consolidation, envelope honoured): from
    # 0.024606277165766802 / 2.2214191211627377 — friction correction ~+1.8%,
    # envelope ~+6% on this hard-floor window. The CLAIM is the ordering below.
    assert p0.joint_mean == pytest.approx(0.026612264870808103, rel=1e-9)
    assert p0.accel_rms == pytest.approx(2.264508044181515, rel=1e-9)
    assert p30.joint_mean > p0.joint_mean * 1.15
    assert p30.accel_rms > p0.accel_rms


@pytest.mark.go2sim
@needs_freewalk
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_measured_control_timing_is_default_off_and_deterministic():
    """An exact 20 ms interval sequence reproduces the default grid
    bit-for-bit; the measured sequence changes the physics, repeatably."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import CONTROL_DT, rollout_policy
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
    from dimos.robot.unitree.go2.sim.sysid.loop import control_timing

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    be = MujocoBackend()
    kw: dict[str, float] = {"start": 6.0, "seconds": 3.0}
    base = rollout_policy(st, policy, preset, be, **kw)  # type: ignore[arg-type]
    grid = rollout_policy(
        st,
        policy,
        preset,
        be,
        control_intervals=np.full(400, CONTROL_DT),
        **kw,  # type: ignore[arg-type]
    )
    assert np.array_equal(base.pos, grid.pos) and np.array_equal(base.target, grid.target)

    iv = control_timing(FREEWALK).intervals
    a = rollout_policy(st, policy, preset, be, control_intervals=iv, **kw)  # type: ignore[arg-type]
    b = rollout_policy(st, policy, preset, be, control_intervals=iv, **kw)  # type: ignore[arg-type]
    assert not np.array_equal(base.pos, a.pos)
    assert np.array_equal(a.pos, b.pos)
    # the measured rate is ~44 Hz: fewer policy ticks over the same span
    assert len(a.t) < len(base.t)
