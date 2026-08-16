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
    Report,
    cmd_at,
    usable_floor,
)
from dimos.robot.unitree.go2.sim.sysid.ingest import Streams
from dimos.robot.unitree.go2.sim.sysid.stats import Summary

FREEWALK = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
FREEWALK_BIN = Path(__file__).parents[6] / "data/ml-trajectory-research/freewalk_mcf.bin"


def _summary(**over: float) -> Summary:
    base = dict.fromkeys(Summary.__dataclass_fields__, 0.0)
    base.update(speed=0.5, speed_gain=0.9, gait_hz=1.7, height_mean=0.3)
    base.update(over)
    return Summary(**base)  # type: ignore[arg-type]


def _streams(wt: np.ndarray, wcmd: np.ndarray) -> Streams:
    z = np.zeros(0)
    return Streams(
        lt=z,
        lq=np.zeros((0, 12)),
        ldq=np.zeros((0, 12)),
        ltau=np.zeros((0, 12)),
        lquat=np.zeros((0, 4)),
        lgyro=np.zeros((0, 3)),
        lacc=np.zeros((0, 3)),
        ct=z,
        cq=np.zeros((0, 12)),
        ckp=np.zeros((0, 12)),
        ckd=np.zeros((0, 12)),
        ctau=np.zeros((0, 12)),
        cdq=np.zeros((0, 12)),
        wt=wt,
        wcmd=wcmd,
    )


def test_cmd_at_holds_zero_order():
    st = _streams(np.array([0.0, 1.0, 2.0]), np.array([[0.1, 0, 0], [0.5, 0, 0], [0.0, 0, 0]]))
    got = cmd_at(st, np.array([-0.5, 0.5, 1.0, 1.9, 5.0]))
    assert got[:, 0] == pytest.approx([0.1, 0.1, 0.5, 0.5, 0.0])


def test_usable_floor_lifts_a_collapsed_noise_floor():
    """A well-stabilised policy can drive a floor to ~0, sending its SNR to
    infinity and letting one term dominate — clamp it."""
    floor = usable_floor({"speed": 1e-9, "gait_hz": 0.5}, {"speed": 0.5, "gait_hz": 1.7})
    assert floor["speed"] == pytest.approx(0.025)  # 5% of the real value
    assert floor["gait_hz"] == 0.5  # an honest floor is untouched


def test_usable_floor_cross_clamps_against_another_recording():
    floor = usable_floor({"speed": 1e-9}, {"speed": 0.0}, {"speed": 0.03})
    assert floor["speed"] == 0.03


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
    assert len(snr) == 10


def test_loss_is_rms_and_n_matched_counts_within_floor():
    noise = dict.fromkeys(Summary.__dataclass_fields__, 0.1)
    rep = _report(_summary(speed=0.7), _summary(speed=0.5), noise)  # speed SNR 2, rest 0
    assert rep.loss() == pytest.approx(np.sqrt(4.0 / 10.0))
    n, of = rep.n_matched()
    assert (n, of) == (9, 10)
    assert "9 of 10" in rep.table()


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
