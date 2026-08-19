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

"""The sensitivity instrument: nudge, analyse, resolve — and say where from."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.ranges import CONTACT_DEFAULTS, KNOBS
from dimos.robot.unitree.go2.sim.sysid.identify import (
    SegmentRows,
    Sensitivity,
    analyse,
    format_report,
    format_resolution,
    format_segments,
    nudge,
    resolution,
)
from dimos.robot.unitree.go2.sim.sysid.regimes import Segment, sample_segments

# ---------------------------------------------------------------- nudge


def test_a_log_knob_steps_in_log_space_symmetrically():
    """A bound is judged in the knob's own metric, and so is a perturbation:
    a linear step on armature's 50x range would be 25x the value at the low
    end and negligible at the high end."""
    knob = KNOBS["armature"]  # log, [0.001, 0.05]
    base = {"armature": 0.005}
    up = nudge(base, "armature", knob, 0.05, +1)["armature"]
    dn = nudge(base, "armature", knob, 0.05, -1)["armature"]
    assert up / 0.005 == pytest.approx(0.005 / dn)  # symmetric in log space
    assert up > 0.005 > dn


def test_a_linear_knob_steps_by_a_fraction_of_its_range():
    knob = KNOBS["trunk_mass_scale"]  # linear, [0.8, 1.4]
    got = nudge({"trunk_mass_scale": 1.0}, "trunk_mass_scale", knob, 0.05, +1)
    assert got["trunk_mass_scale"] == pytest.approx(1.0 + 0.05 * (1.4 - 0.8))


def test_a_nudge_clamps_to_the_knobs_range():
    knob = KNOBS["trunk_inertia_scale"]  # [0.8, 1.3]
    got = nudge({"trunk_inertia_scale": 1.29}, "trunk_inertia_scale", knob, 0.5, +1)
    assert got["trunk_inertia_scale"] == knob.hi


def test_an_absent_contact_knob_centres_on_the_menagerie_default():
    """No early preset carries a contact value — the centre of the difference
    is the engine default, which is what keeps old rollouts identical."""
    knob = KNOBS["foot_solimp_dmin"]
    got = nudge({}, "foot_solimp_dmin", knob, 0.05, +1)
    assert got["foot_solimp_dmin"] > CONTACT_DEFAULTS["foot_solimp_dmin"]
    # and one default sits exactly AT its range top: the up-nudge clamps there,
    # the central difference lives off the down side alone
    got = nudge({}, "foot_solref_time", KNOBS["foot_solref_time"], 0.05, +1)
    assert got["foot_solref_time"] == KNOBS["foot_solref_time"].hi


def test_a_knob_with_no_value_and_no_default_is_an_error_not_a_guess():
    with pytest.raises(KeyError, match="no value"):
        nudge({}, "leg_mass_scale", KNOBS["leg_mass_scale"], 0.05, +1)


def test_a_nudge_returns_a_copy_and_leaves_the_base_alone():
    base = {"armature": 0.005}
    nudge(base, "armature", KNOBS["armature"], 0.05, +1)
    assert base == {"armature": 0.005}


# ---------------------------------------------------------------- analyse


def test_identical_columns_are_reported_degenerate():
    col = np.random.default_rng(0).normal(size=100)
    J = np.stack([col, col, np.random.default_rng(1).normal(size=100)], axis=1)
    a = analyse(J, ["a", "b", "c"])
    assert abs(a["corr"][0, 1]) == pytest.approx(1.0)
    assert "a" in format_report(a) and "cos 1.000" in format_report(a)


def test_a_zero_column_means_blind():
    rng = np.random.default_rng(0)
    J = np.stack([rng.normal(size=50), np.zeros(50)], axis=1)
    a = analyse(J, ["seen", "blind"])
    assert a["sensitivity"][1] == 0.0
    # one informative direction: the second eigenvalue is numerically zero
    assert a["evals"][1] == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------- resolution


def _sens(J: np.ndarray, residual: np.ndarray, names: list[str]) -> Sensitivity:
    seg = Segment(0.0, 1.0)
    return Sensitivity(
        J=J,
        names=names,
        channel="joint",
        residual=residual,
        segments=[SegmentRows(seg, slice(0, len(J)))],
    )


def test_resolution_is_the_range_fraction_that_matches_one_residual_rms():
    """Sensitivity 2.0 per full range against residual RMS 1.0: moving 0.5 of
    the range moves the prediction by one RMS — resolved."""
    n = 100
    J = np.stack([np.full(n, 2.0), np.full(n, 0.1)], axis=1)
    r = resolution(_sens(J, np.ones(n), ["strong", "weak"]))
    assert r[0] == pytest.approx(0.5)
    assert r[1] == pytest.approx(10.0)  # 10x its whole range: not resolved
    report = format_resolution(_sens(J, np.ones(n), ["strong", "weak"]))
    assert "resolves 1 of 2" in report


def test_resolution_without_a_measured_side_is_nan_not_a_claim():
    J = np.ones((10, 2))
    r = resolution(_sens(J, np.full(10, np.nan), ["a", "b"]))
    assert np.all(np.isnan(r))
    assert "no measured side" in format_resolution(_sens(J, np.full(10, np.nan), ["a", "b"]))


# ---------------------------------------------------------------- segments


def test_segments_are_seeded_and_reproducible():
    a = sample_segments(0.0, 60.0, n=6, seed=7)
    b = sample_segments(0.0, 60.0, n=6, seed=7)
    assert a == b
    assert a != sample_segments(0.0, 60.0, n=6, seed=8)


def test_segments_spread_across_the_recording_not_cluster():
    """Stratified on purpose: the information is concentrated (t=50-55 carried
    the trunk-inertia answer; t=25 and t=40 were near-motionless), so the
    sampler must look everywhere — one start per stratum."""
    segs = sample_segments(0.0, 100.0, n=10, length=(2.0, 4.0), seed=3)
    starts = [s.t0 for s in segs]
    assert starts == sorted(starts)
    for i, s in enumerate(segs):
        room = 100.0 - s.duration
        assert i / 10 * room <= s.t0 - 0.0 <= (i + 1) / 10 * room


def test_segments_fit_inside_the_recording():
    for seg in sample_segments(10.0, 30.0, n=8, length=(4.0, 8.0), seed=0):
        assert 10.0 <= seg.t0 and seg.t1 <= 30.0 + 1e-9
        assert 4.0 <= seg.duration <= 8.0


def test_a_segment_longer_than_the_recording_is_clamped_to_it():
    (seg,) = sample_segments(0.0, 3.0, n=1, length=(5.0, 8.0), seed=0)
    assert seg.t0 == 0.0 and seg.duration == 3.0


def test_bad_segment_arguments_raise():
    with pytest.raises(ValueError, match="at least one"):
        sample_segments(0.0, 10.0, n=0)
    with pytest.raises(ValueError, match="lo <= hi"):
        sample_segments(0.0, 10.0, length=(2.0, 1.0))
    with pytest.raises(ValueError, match="empty"):
        sample_segments(5.0, 5.0)


# ---------------------------------------------------------------- reports


def test_the_segment_report_names_where_the_information_came_from():
    """A user whose recording resolves 7 of 14 needs to know it is because the
    robot never did anything hard, not because the method failed."""
    from dimos.robot.unitree.go2.sim.sysid.recording import Streams

    n = 200
    t = np.linspace(0.0, 20.0, n)
    st = Streams(
        lt=t,
        lq=np.zeros((n, 12)),
        ldq=np.ones((n, 12)),
        ltau=np.zeros((n, 12)),
        lquat=np.tile([1.0, 0, 0, 0], (n, 1)),
        lgyro=np.zeros((n, 3)),
        lacc=np.tile([0.0, 0.0, 9.8], (n, 1)),
        ct=t,
        cq=np.zeros((n, 12)),
        ckp=np.zeros((n, 12)),
        ckd=np.zeros((n, 12)),
        ctau=np.zeros((n, 12)),
        cdq=np.zeros((n, 12)),
    )
    hot = np.zeros((100, 1))
    cold = np.full((100, 1), 1e-6)
    sens = Sensitivity(
        J=np.concatenate([cold, hot + 3.0]),
        names=["k"],
        channel="joint",
        residual=np.ones(200),
        segments=[
            SegmentRows(Segment(0.0, 5.0), slice(0, 100)),
            SegmentRows(Segment(10.0, 5.0), slice(100, 200)),
        ],
    )
    report = format_segments(sens, st)
    assert "t=  10.0..  15.0" in report
    lines = [ln for ln in report.splitlines() if "t=" in ln]
    assert "100.0%" in lines[1] and "0.0%" in lines[0]


# ------------------------------------------------- the instrument, end to end


MIXED = __import__("pathlib").Path.home() / (
    "recordings/rubber_floor/20260816-015155_policy-mixed_vive.mcap"
)

# Recordings are not vendored, so they may legitimately be absent; the go2
# scene IS (data/go2_menagerie), so no menagerie condition here — a missing
# scene fails instead of skipping.
needs_rig = pytest.mark.skipif(
    __import__("importlib.util", fromlist=["util"]).find_spec("mujoco") is None
    or not MIXED.is_file(),
    reason="needs mujoco + the mixed recording",
)


@pytest.fixture(scope="module")
def mixed_streams():
    pytest.importorskip("mcap")
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    return read_streams(MIXED)


@needs_rig
@pytest.mark.go2sim
def test_the_jacobian_is_deterministic_and_shares_its_schedules(mixed_streams):
    """Same recording, same seed, twice: bit-identical J and residual. The
    clip schedule is a pure function per segment and the backend is
    deterministic, so any drift here means a rollout leaked state."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.identify import jacobian
    from dimos.robot.unitree.go2.sim.sysid.regimes import sample_segments

    st = mixed_streams
    t_lo = max(float(st.lt[0]), float(st.ct[0]))
    segs = sample_segments(t_lo, t_lo + 12.0, n=2, length=(1.5, 2.5), seed=0)
    values = {**MEASURED.physics, "actuator_tau": MEASURED.actuator_tau}
    kw = dict(frac=0.05, window=0.4, seed=0, params=("armature", "trunk_mass_scale"))
    a = jacobian(st, segs, MujocoBackend(), values, channel="joint", **kw)
    b = jacobian(st, segs, MujocoBackend(), values, channel="joint", **kw)
    assert np.array_equal(a.J, b.J)
    assert np.array_equal(a.residual, b.residual)
    assert a.names == ["armature", "trunk_mass_scale"]
    # rows partition the Jacobian, one block per segment
    assert a.segments[0].rows.start == 0
    assert a.segments[-1].rows.stop == a.J.shape[0]
    # the knobs move the prediction, and the residual is measured (no NaN fill)
    assert np.linalg.norm(a.J, axis=0).min() > 0
    assert np.all(np.isfinite(a.residual))


@needs_rig
@pytest.mark.go2sim
def test_every_channel_produces_a_scorable_jacobian(mixed_streams):
    from dimos.robot.unitree.go2.sim.backend import CHANNELS
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.identify import (
        analyse,
        format_report,
        format_resolution,
        format_segments,
        jacobian,
        resolution,
    )
    from dimos.robot.unitree.go2.sim.sysid.regimes import sample_segments

    st = mixed_streams
    t_lo = max(float(st.lt[0]), float(st.ct[0]))
    segs = sample_segments(t_lo, t_lo + 10.0, n=1, length=(1.5, 2.0), seed=1)
    values = {**MEASURED.physics, "actuator_tau": MEASURED.actuator_tau}
    for channel in CHANNELS:
        s = jacobian(
            st,
            segs,
            MujocoBackend(),
            values,
            channel=channel,
            window=0.4,
            params=("trunk_mass_scale",),
        )
        assert s.J.shape[1] == 1 and s.J.shape[0] > 0, channel
        r = resolution(s)
        assert np.all(np.isfinite(r)), channel  # tracker present: every side measured
        # the reports render for every channel
        format_report(analyse(s.J, s.names))
        format_resolution(s)
        format_segments(s, st)
