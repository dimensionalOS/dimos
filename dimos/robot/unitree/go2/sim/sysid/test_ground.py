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

"""Loop 2: floors, SNRs, and the closed-loop rollout against a recording."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ground import (
    ObsNoise,
    Report,
    usable_floor,
)
from dimos.robot.unitree.go2.sim.sysid.stats import Summary

FREEWALK = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
FREEWALK_BIN = Path(__file__).parents[6] / "data/ml-trajectory-research/freewalk_mcf.bin"


def _summary(**over: float) -> Summary:
    base = dict.fromkeys(Summary.__dataclass_fields__, 0.0)
    del base["source"]  # provenance, not a statistic
    base.update(speed=0.5, speed_gain=0.9, gait_hz=1.7, height_mean=0.3)
    base.update(over)
    return Summary(**base)  # type: ignore[arg-type]


def test_usable_floor_lifts_a_collapsed_noise_floor():
    """A well-stabilised policy can drive a floor to ~0, sending its SNR to
    infinity and letting one term dominate — clamp it."""
    floor = usable_floor({"speed": 1e-9, "gait_hz": 0.5}, {"speed": 0.5, "gait_hz": 1.7})
    assert floor["speed"] == pytest.approx(0.025)  # 5% of the real value
    assert floor["gait_hz"] == 0.5  # an honest floor is untouched


def test_usable_floor_cross_clamps_against_another_recording():
    floor = usable_floor({"speed": 1e-9}, {"speed": 0.0}, {"speed": 0.03})
    assert floor["speed"] == 0.03


def test_snr_leaves_out_statistics_that_are_nan_on_either_side():
    """NaN means NOT COMPARABLE on this recording — neither scored nor counted."""
    nan = float("nan")
    rep = _report(
        _summary(speed=0.6),
        _summary(speed=nan, speed_gain=nan, speed_lag=nan, height_std=nan, gait_hz=nan),
        dict.fromkeys(Summary.__dataclass_fields__, 0.05),
    )
    snr = rep.snr()
    assert len(snr) == 7  # 11 comparable - 4 NaN pairs (gait_hz was never scored)
    assert "speed" not in snr and "roll_std" in snr
    assert np.isfinite(rep.loss())
    _n, of = rep.n_matched()
    assert of == 7


def test_obs_noise_scales_every_level_together():
    n = ObsNoise().scaled(2.0)
    assert (n.gyro, n.gravity, n.q, n.dq) == (0.4, 0.1, 0.02, 3.0)
    assert ObsNoise().scaled(0.0) == ObsNoise(0.0, 0.0, 0.0, 0.0)


def _report(sim: Summary, real: Summary, noise: dict[str, float]) -> Report:
    return Report(
        preset="test", sim=sim, real=real, noise=noise, floor_source="test", start=0.0, seconds=1.0
    )


def test_snr_is_the_difference_over_the_floor_and_height_mean_is_excluded():
    rep = _report(
        _summary(speed=0.6, height_mean=0.5),
        _summary(speed=0.5, height_mean=0.2),
        dict.fromkeys(Summary.__dataclass_fields__, 0.05),
    )
    snr = rep.snr()
    assert snr["speed"] == pytest.approx(2.0)
    assert "height_mean" not in snr  # room frame: no honest comparison exists
    assert "gait_hz" not in snr  # retired: it measured its own estimator (README 5g)
    assert len(snr) == 11


def test_loss_is_rms_and_n_matched_counts_within_floor():
    noise = dict.fromkeys(Summary.__dataclass_fields__, 0.1)
    rep = _report(_summary(speed=0.7), _summary(speed=0.5), noise)  # speed SNR 2, rest 0
    assert rep.loss() == pytest.approx(np.sqrt(4.0 / 11.0))
    n, of = rep.n_matched()
    assert (n, of) == (10, 11)
    assert "10 of 11" in rep.table()


# ------------------------------------------------- against the real recording


def _menagerie_available() -> bool:
    from dimos.robot.unitree.go2.sim.model import scene_path

    try:
        scene_path()
    except FileNotFoundError:
        return False
    return True


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_the_closed_loop_rollout_is_deterministic_and_summarisable():
    if not _menagerie_available():
        pytest.skip("no mujoco_menagerie checkout")
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy, sim_summary
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    a = rollout_policy(st, policy, preset, start=6.0, seconds=4.0)
    b = rollout_policy(st, policy, preset, start=6.0, seconds=4.0)
    assert np.array_equal(a.pos, b.pos) and np.array_equal(a.target, b.target)
    assert len(a.t) == pytest.approx(4.0 / 0.02, abs=2)
    # it walks rather than falls: the summary is computable and upright.
    # height_mean is the VIRTUAL TRACKER's height (base + 0.207 m lever arm)
    # — sensor space, matching the raw tracker height on the real side.
    s = sim_summary(a)
    assert 0.35 < s.height_mean < 0.65
    assert s.tilt_p99 < 1.0


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_loop_mechanisms_are_off_by_default_and_deterministic_when_on():
    """Latency and obs noise must change the physics when on, change NOTHING
    when off (every existing grounding number reproduces), and be exactly
    repeatable so a probe result is a measurement, not a draw."""
    if not _menagerie_available():
        pytest.skip("no mujoco_menagerie checkout")
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    kw: dict[str, float] = {"start": 6.0, "seconds": 3.0}
    base = rollout_policy(st, policy, preset, **kw)  # type: ignore[arg-type]
    explicit_off = rollout_policy(
        st,
        policy,
        preset,
        action_latency=0.0,
        obs_noise=None,
        **kw,  # type: ignore[arg-type]
    )
    assert np.array_equal(base.pos, explicit_off.pos)

    lat = rollout_policy(st, policy, preset, action_latency=0.01, **kw)  # type: ignore[arg-type]
    lat2 = rollout_policy(st, policy, preset, action_latency=0.01, **kw)  # type: ignore[arg-type]
    assert not np.array_equal(base.pos, lat.pos)
    assert np.array_equal(lat.pos, lat2.pos)

    nz = rollout_policy(st, policy, preset, obs_noise=ObsNoise(), **kw)  # type: ignore[arg-type]
    nz2 = rollout_policy(st, policy, preset, obs_noise=ObsNoise(), **kw)  # type: ignore[arg-type]
    assert not np.array_equal(base.pos, nz.pos)
    assert np.array_equal(nz.pos, nz2.pos)


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_a_perturbed_start_diverges_but_the_statistics_survive():
    """The whole premise of loop 2 in one assertion pair: position separates,
    the summary statistics stay within a sensible band."""
    if not _menagerie_available():
        pytest.skip("no mujoco_menagerie checkout")
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy, sim_summary
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    a = rollout_policy(st, policy, preset, start=6.0, seconds=8.0)
    b = rollout_policy(st, policy, preset, start=6.0, seconds=8.0, perturb=np.full(12, 0.02))
    # chaos: the trajectories separate ...
    assert np.abs(a.pos - b.pos).max() > 0.01
    # ... the distributional statistics do not
    sa, sb = sim_summary(a), sim_summary(b)
    assert abs(sa.speed - sb.speed) < 0.15
    assert abs(sa.gait_hz - sb.gait_hz) < 0.6
