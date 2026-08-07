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

"""Scoring plumbing: the physics patch, the SNR weighting, and the loss."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.motion.simulation import model as go2_model
from dimos.navigation.motion.simulation.evaluate import (
    FOOT_GEOMS,
    LEG_DOFS,
    NOT_COMPARABLE,
    Report,
    _physics,
    virtual_tracker,
)
from dimos.navigation.motion.simulation.metrics import Summary


def _summary(**kw):
    base = dict(
        speed=0.4,
        speed_gain=0.9,
        yaw_rate_gain=0.5,
        height_mean=0.3,
        height_std=0.03,
        gait_hz=2.0,
        speed_lag=0.2,
        yaw_lag=0.1,
        pitch_std=0.02,
        roll_std=0.02,
        tilt_p99=0.06,
    )
    base.update(kw)
    return Summary(**base)


def _report(sim, real, noise):
    return Report(sim=sim, real=real, noise=noise, seconds=10.0, start=6.0)


def test_snr_divides_by_each_statistic_own_noise():
    r = _report(
        _summary(speed=0.50),
        _summary(speed=0.40),
        dict.fromkeys(_summary().as_dict(), 0.05),
    )
    assert r.snr()["speed"] == pytest.approx(2.0)
    assert r.snr()["gait_hz"] == pytest.approx(0.0)


def test_snr_excludes_the_statistics_that_cannot_be_compared():
    r = _report(_summary(), _summary(), dict.fromkeys(_summary().as_dict(), 0.1))
    for key in NOT_COMPARABLE:
        assert key not in r.snr()
    assert "height_mean" in r.sim.as_dict()  # still reported, just not scored


def test_zero_noise_is_infinite_snr_not_a_crash():
    noise = dict.fromkeys(_summary().as_dict(), 0.1)
    noise["speed"] = 0.0
    r = _report(_summary(speed=0.5), _summary(speed=0.4), noise)
    assert r.snr()["speed"] == float("inf")


def test_loss_is_zero_when_sim_matches_real():
    r = _report(_summary(), _summary(), dict.fromkeys(_summary().as_dict(), 0.1))
    assert r.loss() == pytest.approx(0.0)


def test_loss_rises_with_the_gap():
    noise = dict.fromkeys(_summary().as_dict(), 0.1)
    near = _report(_summary(speed=0.45), _summary(speed=0.40), noise).loss()
    far = _report(_summary(speed=0.80), _summary(speed=0.40), noise).loss()
    assert far > near > 0


def test_table_mentions_the_physics_it_ran_with():
    r = Report(
        sim=_summary(),
        real=_summary(),
        noise=dict.fromkeys(_summary().as_dict(), 0.1),
        seconds=10.0,
        start=6.0,
        physics={"armature": 0.03},
    )
    assert "armature=0.03" in r.table()


def test_physics_override_applies_and_restores():
    original = go2_model.load
    model, _ = go2_model.load()
    before = float(model.dof_armature[LEG_DOFS][0])

    with _physics({"armature": 0.07, "frictionloss": 1.5}):
        patched, _ = go2_model.load()
        np.testing.assert_allclose(patched.dof_armature[LEG_DOFS], 0.07)
        np.testing.assert_allclose(patched.dof_frictionloss[LEG_DOFS], 1.5)
        # untouched keys keep the MJCF value
        assert float(patched.dof_damping[LEG_DOFS][0]) == pytest.approx(2.0)

    assert go2_model.load is original
    restored, _ = go2_model.load()
    assert float(restored.dof_armature[LEG_DOFS][0]) == pytest.approx(before)


def test_virtual_tracker_sits_above_an_upright_base():
    """Inverted mount, +0.207 in tracker frame -> the tracker rides 0.207 above
    the base, so the virtual z must come out higher than the base z."""
    pos = np.array([[0.0, 0.0, 0.30]])
    quat = np.array([[1.0, 0.0, 0.0, 0.0]])
    out = virtual_tracker(pos, quat, mount_yaw=94.0, tracker_z=0.207)
    assert out[0, 2] == pytest.approx(0.30 + 0.207, abs=1e-9)
    np.testing.assert_allclose(out[0, :2], pos[0, :2])  # xy untouched


def test_virtual_tracker_swings_with_body_pitch():
    """The lever arm must show up once the body tilts — that is the point of
    comparing in sensor space."""
    half = np.sin(0.1 / 2)  # 0.1 rad pitch about y
    quat = np.array([[np.cos(0.1 / 2), 0.0, half, 0.0]])
    pos = np.array([[0.0, 0.0, 0.30]])
    out = virtual_tracker(pos, quat, mount_yaw=94.0, tracker_z=0.207)
    assert out[0, 2] == pytest.approx(0.30 + 0.207 * np.cos(0.1), abs=1e-6)


def test_physics_override_sets_foot_friction():
    import mujoco

    with _physics({"foot_friction": 1.1, "foot_friction_torsional": 0.09}):
        model, _ = go2_model.load()
        for name in FOOT_GEOMS:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert float(model.geom_friction[gid, 0]) == pytest.approx(1.1)
            assert float(model.geom_friction[gid, 1]) == pytest.approx(0.09)
            assert float(model.geom_friction[gid, 2]) == pytest.approx(0.01)  # rolling kept


def test_leg_stats_rise_when_the_front_thigh_lifts():
    """FK sanity: raising front thigh targets must raise front_lift only."""
    from dimos.navigation.motion.simulation.evaluate import leg_stats

    t = np.arange(50) * 0.02
    stand = np.tile([0.0, 0.9, -1.8] * 4, (50, 1))
    lifted = stand.copy()
    lifted[:, [1, 4]] -= 0.5  # front thighs swing forward/up (FL, FR)
    a, b = leg_stats(t, stand), leg_stats(t, lifted)
    assert b["front_lift"] > a["front_lift"] + 0.02
    assert b["rear_lift"] == pytest.approx(a["rear_lift"], abs=1e-6)


def test_report_scores_leg_statistics_when_present():
    noise = dict.fromkeys(_summary().as_dict(), 0.1)
    noise["front_lift"] = 0.01
    noise["rear_lift"] = 0.01
    r = Report(
        sim=_summary(),
        real=_summary(),
        noise=noise,
        seconds=10.0,
        start=6.0,
        sim_legs={"front_lift": 0.10, "rear_lift": 0.04},
        real_legs={"front_lift": 0.06, "rear_lift": 0.04},
    )
    assert r.snr()["front_lift"] == pytest.approx(4.0)
    assert r.snr()["rear_lift"] == pytest.approx(0.0)
    assert "front_lift" in r.table()


def test_report_skips_legs_when_the_recording_has_none():
    r = _report(_summary(), _summary(), dict.fromkeys(_summary().as_dict(), 0.1))
    assert "front_lift" not in r.snr()


def test_physics_override_reaches_the_ghost_loader():
    """--view --ghost builds through load_with_ghost, which once went
    unpatched: --fitted showed stock physics whenever the ghost was on."""
    with _physics({"armature": 0.07}):
        model, _ = go2_model.load_with_ghost()
        np.testing.assert_allclose(model.dof_armature[LEG_DOFS], 0.07)
    restored, _ = go2_model.load_with_ghost()
    assert float(restored.dof_armature[LEG_DOFS][0]) != pytest.approx(0.07)


def test_physics_override_rejects_unknown_keys():
    with pytest.raises(ValueError, match="unknown physics override"), _physics({"mass": 1.0}):
        pass


def test_empty_physics_is_a_no_op():
    original = go2_model.load
    with _physics(None):
        assert go2_model.load is original
    with _physics({}):
        assert go2_model.load is original
