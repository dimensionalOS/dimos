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

"""One span-labelling pass, and a clip schedule that respects span edges."""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.recording import Declarations, Streams
from dimos.robot.unitree.go2.sim.sysid.regimes import (
    Span,
    clip_schedule,
    contaminated_spans,
    flight_spans,
    propose_suspended,
    protected,
    regimes,
)


def _streams(t: np.ndarray, acc: np.ndarray, tau: np.ndarray | None = None) -> Streams:
    z12 = np.zeros((len(t), 12))
    return Streams(
        lt=t,
        lq=z12,
        ldq=z12,
        ltau=np.full((len(t), 12), 3.0) if tau is None else tau,
        lquat=np.zeros((len(t), 4)),
        lgyro=np.zeros((len(t), 3)),
        lacc=acc,
        ct=t,
        cq=z12,
        ckp=z12,
        ckd=z12,
        ctau=z12,
        cdq=z12,
    )


def _grounded(n: int) -> np.ndarray:
    return np.tile([0.0, 0.0, 9.5], (n, 1))


def test_no_window_never_reinitialises():
    assert len(clip_schedule(0.0, 10.0, None)) == 0


def test_a_scalar_window_is_a_fixed_grid():
    assert np.allclose(clip_schedule(5.0, 2.0, 0.4), [5.4, 5.8, 6.2, 6.6])


def test_the_last_clip_is_not_cut_by_a_reinit_past_the_end():
    assert np.allclose(clip_schedule(0.0, 10.0, 2.5), [2.5, 5.0, 7.5])


def test_sampled_clips_stay_inside_their_bounds():
    got = clip_schedule(0.0, 200.0, (0.05, 0.8), seed=3)
    clips = np.diff(np.concatenate([[0.0], got]))
    assert clips.min() >= 0.05 and clips.max() <= 0.8
    assert got.max() < 200.0


def test_the_schedule_is_reproducible_because_a_fit_compares_across_plants():
    a = clip_schedule(0.0, 50.0, (0.05, 0.8), seed=7)
    b = clip_schedule(0.0, 50.0, (0.05, 0.8), seed=7)
    assert np.array_equal(a, b)


def test_a_different_seed_is_a_different_schedule():
    a = clip_schedule(0.0, 50.0, (0.05, 0.8), seed=7)
    b = clip_schedule(0.0, 50.0, (0.05, 0.8), seed=8)
    n = min(len(a), len(b))
    assert not np.array_equal(a[:n], b[:n])


def test_a_degenerate_range_is_the_fixed_grid():
    assert np.allclose(clip_schedule(0.0, 2.0, (0.4, 0.4)), clip_schedule(0.0, 2.0, 0.4))


@pytest.mark.parametrize("bad", [0.0, -1.0, (0.0, 1.0), (2.0, 1.0), (-0.5, 0.5)])
def test_a_window_that_cannot_advance_is_rejected_not_looped_forever(bad):
    with pytest.raises(ValueError):
        clip_schedule(0.0, 10.0, bad)


def _spans(*pairs):
    return np.array(pairs, dtype=float).reshape(-1, 2)


def test_a_clip_boundary_never_falls_inside_a_flight_span():
    """A re-initialisation inside a ballistic arc snaps the sim back to
    measured state part-way through, and the trunk counter-rotation never
    accumulates: trunk_inertia_scale drops from 74% to 4% sensitivity."""
    got = clip_schedule(0.0, 10.0, 0.4, protect=_spans((2.05, 2.35)))
    assert not ((got > 2.05) & (got < 2.35)).any()


def test_each_protected_span_opens_its_own_clip_at_take_off():
    got = clip_schedule(0.0, 10.0, 0.4, protect=_spans((2.05, 2.35), (5.11, 5.29)))
    assert 2.05 in got and 5.11 in got


def test_protection_displaces_only_the_grid_points_it_covers():
    plain = clip_schedule(0.0, 10.0, 0.4)
    got = clip_schedule(0.0, 10.0, 0.4, protect=_spans((1.9, 2.1)))
    lost = set(np.round(plain, 6)) - set(np.round(got, 6))
    assert lost == {2.0}
    assert 1.9 in got


def test_a_span_outside_the_rollout_is_ignored():
    got = clip_schedule(0.0, 2.0, 0.4, protect=_spans((90.0, 90.2)))
    assert np.allclose(got, clip_schedule(0.0, 2.0, 0.4))


def test_flight_is_where_the_accelerometer_goes_quiet():
    """The IMU is the contact sensor. It has to be: foot_force on this Go2 Air
    reads 40-61 counts and never drops, even mid-jump."""
    t = np.arange(0, 1.0, 0.002)
    acc = _grounded(len(t))
    acc[(t >= 0.30) & (t < 0.45)] = [0.0, 0.0, 0.05]  # 150 ms of free fall
    acc[(t >= 0.70) & (t < 0.71)] = [0.0, 0.0, 0.05]  # 10 ms blip, too short
    got = flight_spans(_streams(t, acc))
    assert len(got) == 1
    assert abs(got[0, 0] - 0.30) < 0.01 and abs(got[0, 1] - 0.45) < 0.01


def test_no_accelerometer_means_no_spans_not_a_crash():
    st = _streams(np.zeros(0), np.zeros((0, 3)), tau=np.zeros((0, 12)))
    assert flight_spans(st).shape == (0, 2)


def test_a_jump_recording_is_floor_with_flight_inside_it():
    t = np.arange(0, 1.0, 0.002)
    acc = _grounded(len(t))
    acc[(t >= 0.30) & (t < 0.45)] = [0.0, 0.0, 0.05]
    got = regimes(_streams(t, acc))
    kinds = [s.kind for s in got]
    assert kinds == ["floor", "flight", "floor"]


def test_the_spans_tile_the_recording_without_gaps():
    t = np.arange(0, 1.0, 0.002)
    acc = _grounded(len(t))
    acc[(t >= 0.30) & (t < 0.45)] = [0.0, 0.0, 0.05]
    got = regimes(_streams(t, acc))
    assert got[0].t0 == t[0] and got[-1].t1 == t[-1]
    for a, b in itertools.pairwise(got):
        assert a.t1 == b.t0


def test_a_declared_suspension_labels_the_whole_file():
    t = np.arange(0, 2.0, 0.002)
    st = _streams(t, _grounded(len(t)), tau=np.full((len(t), 12), 0.3))
    got = regimes(st, Declarations(suspended=True))
    assert [s.kind for s in got] == ["suspended"]


def test_an_undeclared_recording_with_unloaded_legs_is_refused_not_guessed():
    """A hanging robot reads ~1 g with unloaded legs — and so does one lying
    down. Ambiguous in principle, so ingest refuses rather than guesses."""
    t = np.arange(0, 2.0, 0.002)
    st = _streams(t, _grounded(len(t)), tau=np.full((len(t), 12), 0.3))
    with pytest.raises(ValueError, match="declares"):
        regimes(st)


def test_a_declared_not_suspended_overrides_the_detector():
    """The detector cross-checks; it never decides."""
    t = np.arange(0, 2.0, 0.002)
    st = _streams(t, _grounded(len(t)), tau=np.full((len(t), 12), 0.3))
    got = regimes(st, Declarations(suspended=False))
    assert all(s.kind != "suspended" for s in got)


def test_unloaded_legs_are_what_the_detector_proposes_on():
    t = np.arange(0, 1.0, 0.002)
    hanging = _streams(t, _grounded(len(t)), tau=np.full((len(t), 12), 0.45))
    walking = _streams(t, _grounded(len(t)), tau=np.full((len(t), 12), 2.32))
    assert propose_suspended(hanging) and not propose_suspended(walking)


def test_a_rope_tug_needs_both_signatures_to_fire():
    """|a| deviation 3.9-4.1 against ~1.0 AND torque p90 19-26 against 2-7.
    Either alone (a jump landing spikes accel; a stumble spikes torque) must
    not contaminate the span."""
    t = np.arange(0, 10.0, 0.002)
    n = len(t)
    acc = np.tile([0.0, 0.0, 9.6], (n, 1))
    tau = np.full((n, 12), 0.3)
    tug = (t >= 4.0) & (t < 5.0)
    acc[tug, 2] = 9.6 + 6.0 * np.sin(40.0 * t[tug])
    tau[tug, :6] = 20.0
    both = contaminated_spans(_streams(t, acc, tau=tau))
    assert len(both) == 1
    assert both[0, 0] <= 4.0 and both[0, 1] >= 5.0

    accel_only = contaminated_spans(_streams(t, acc, tau=np.full((n, 12), 0.3)))
    quiet_acc = np.tile([0.0, 0.0, 9.6], (n, 1))
    torque_only = contaminated_spans(_streams(t, quiet_acc, tau=tau))
    assert len(accel_only) == 0 and len(torque_only) == 0


def test_a_suspended_recording_cuts_its_rope_tugs_out():
    t = np.arange(0, 10.0, 0.002)
    n = len(t)
    acc = np.tile([0.0, 0.0, 9.6], (n, 1))
    tau = np.full((n, 12), 0.3)
    tug = (t >= 4.0) & (t < 5.0)
    acc[tug, 2] = 9.6 + 6.0 * np.sin(40.0 * t[tug])
    tau[tug, :6] = 20.0
    got = regimes(_streams(t, acc, tau=tau), Declarations(suspended=True))
    kinds = [s.kind for s in got]
    assert kinds == ["suspended", "contaminated", "suspended"]
    assert got[0].t0 == t[0] and got[-1].t1 == t[-1]


def test_protected_extracts_what_a_clip_must_not_straddle():
    spans = [
        Span("floor", 0.0, 2.0),
        Span("flight", 2.0, 2.2),
        Span("floor", 2.2, 5.0),
        Span("contaminated", 5.0, 6.0),
    ]
    got = protected(spans)
    assert got.tolist() == [[2.0, 2.2], [5.0, 6.0]]
