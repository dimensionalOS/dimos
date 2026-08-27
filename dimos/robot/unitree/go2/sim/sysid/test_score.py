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

"""The weight-vector score: intersection, normalisation, no special modes."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.backend import CHANNELS, Prediction
from dimos.robot.unitree.go2.sim.sysid.regimes import Span
from dimos.robot.unitree.go2.sim.sysid.replay import ReplayResult
from dimos.robot.unitree.go2.sim.sysid.score import (
    PERMITTED,
    sample_errors,
    scales_from,
    score_terms,
    segment_terms,
)


def _result(
    *,
    t0: float = 0.0,
    dur: float = 1.0,
    n: int = 10,
    k: int = 20,
    tracker: bool = True,
    joint_err: float = 0.1,
    accel_err: float | tuple[float, float] = 3.0,
) -> ReplayResult:
    """A synthetic rollout with exactly controlled per-sample errors.

    ``accel_err`` as a pair puts one value on the first half of the physics
    samples and the other on the second — for regime masking tests.
    """
    t = t0 + dur * np.arange(n) / n
    at = t0 + dur * np.arange(k) / k
    eye = np.tile(np.eye(3), (n, 1, 1))
    pred = Prediction(
        t=t,
        q=np.zeros((n, 12)),
        dq=np.zeros((n, 12)),
        body_pos=np.zeros((n, 3)),
        body_rot=eye,
        at=at,
        imu_accel=np.zeros((k, 3)),
        imu_gyro=np.zeros((k, 3)),
        tau=np.zeros((k, 12)),
    )
    a_real = np.empty((k, 3))
    lo, hi = accel_err if isinstance(accel_err, tuple) else (accel_err, accel_err)
    a_real[: k // 2] = lo
    a_real[k // 2 :] = hi
    return ReplayResult(
        prediction=pred,
        q_real=np.full((n, 12), joint_err),
        p_real=np.zeros((n, 3)) if tracker else None,
        r_real=eye.copy() if tracker else None,
        a_real=a_real,
        w_real=np.zeros((k, 3)),
        tau_real=np.zeros((k, 12)),
        dq_real=np.zeros((n, 12)),
    )


def test_every_channel_has_a_sample_error_and_its_own_time_base():
    r = _result()
    for c in CHANNELS:
        te = sample_errors(r, c)
        assert te is not None, c
        t, err = te
        assert len(t) == len(err) > 0
    with pytest.raises(ValueError, match="unknown channel"):
        sample_errors(r, "warp")


def test_no_tracker_means_pos_and_rot_are_absent_not_zero():
    r = _result(tracker=False)
    assert sample_errors(r, "pos") is None
    assert sample_errors(r, "rot") is None
    # ... and everything else proceeds: the intersection rule, not a VR mode
    terms = segment_terms(r, [Span("floor", 0.0, 1.0)], ["accel", "pos", "rot", "joint"])
    keys = {c for c, _r in terms.sums}
    assert keys == {"accel", "joint"}


def test_a_suspended_span_permits_only_the_joint_space_channels():
    """A held trunk makes accel the weld's reaction and gyro/pos/rot echoes of
    the mocap track — scoring them would grade the bookkeeping."""
    r = _result()
    terms = segment_terms(r, [Span("suspended", 0.0, 1.0)], list(CHANNELS))
    kinds = {(c, k) for c, k in terms.sums}
    assert kinds == {("joint", "suspended"), ("dq", "suspended"), ("tau", "suspended")}
    assert PERMITTED["suspended"] == frozenset({"joint", "dq", "tau"})


def test_contaminated_spans_contribute_to_nothing():
    r = _result(accel_err=(2.0, 4.0))
    spans = [Span("floor", 0.0, 0.5), Span("contaminated", 0.5, 1.0)]
    terms = segment_terms(r, spans, ["accel"])
    s, n = terms.sums[("accel", "floor")]
    assert n == 10  # only the first half of 20 physics samples
    assert s / n == pytest.approx(2.0)
    # the scale pool excludes them too
    sq, nsq = terms.sq["accel"]
    assert nsq == 10


def test_regime_masking_splits_the_terms_where_the_spans_say():
    r = _result(accel_err=(2.0, 6.0))
    spans = [Span("floor", 0.0, 0.5), Span("flight", 0.5, 1.0)]
    terms = segment_terms(r, spans, ["accel"])
    assert terms.sums[("accel", "floor")][0] / terms.sums[("accel", "floor")][1] == pytest.approx(
        2.0
    )
    assert terms.sums[("accel", "flight")][0] / terms.sums[("accel", "flight")][1] == pytest.approx(
        6.0
    )


def test_the_weight_vector_subsumes_w_flight():
    """w[accel, floor] = w[accel, flight] = 0.5 IS the frozen tool's
    channel="accel", w_flight=0.5 — one vector, not two knobs."""
    r = _result(accel_err=(2.0, 6.0))
    spans = [Span("floor", 0.0, 0.5), Span("flight", 0.5, 1.0)]
    terms = [segment_terms(r, spans, ["accel"])]
    scales = {"accel": 1.0}
    s = score_terms(terms, {("accel", "floor"): 0.5, ("accel", "flight"): 0.5}, scales)
    assert s.total == pytest.approx(0.5 * 2.0 + 0.5 * 6.0)
    # all the weight on flight scores only flight
    s = score_terms(terms, {("accel", "flight"): 1.0}, scales)
    assert s.total == pytest.approx(6.0)


def test_missing_terms_renormalise_instead_of_poisoning_the_total():
    """A segment with no flight has no flight term, and 0.0 * nan silently
    failing every trial over grounded data is a measured failure mode."""
    r = _result(accel_err=3.0)
    terms = [segment_terms(r, [Span("floor", 0.0, 1.0)], ["accel"])]
    s = score_terms(terms, {("accel", "floor"): 0.5, ("accel", "flight"): 0.5}, {"accel": 1.0})
    assert s.total == pytest.approx(3.0)  # renormalised over what exists


def test_scales_make_the_terms_addable():
    r = _result(joint_err=0.1, accel_err=3.0)
    spans = [Span("floor", 0.0, 1.0)]
    terms = [segment_terms(r, spans, ["accel", "joint"])]
    scales = scales_from(terms)
    assert scales["accel"] == pytest.approx(3.0)
    assert scales["joint"] == pytest.approx(0.1)
    s = score_terms(terms, {("accel", "floor"): 0.5, ("joint", "floor"): 0.5}, scales)
    # both channels normalised to 1.0 under the baseline: units cannot leak in
    assert s.total == pytest.approx(1.0)


def test_a_weight_vector_that_selects_nothing_is_an_error_not_a_score():
    r = _result(tracker=False)
    terms = [segment_terms(r, [Span("suspended", 0.0, 1.0)], list(CHANNELS))]
    with pytest.raises(ValueError, match="selects nothing"):
        score_terms(terms, {("accel", "floor"): 1.0}, {"accel": 1.0})


def test_the_total_is_the_mean_over_segments():
    a = segment_terms(_result(accel_err=2.0), [Span("floor", 0.0, 1.0)], ["accel"])
    b = segment_terms(_result(t0=1.0, accel_err=4.0), [Span("floor", 1.0, 2.0)], ["accel"])
    s = score_terms([a, b], {("accel", "floor"): 1.0}, {"accel": 1.0})
    assert s.per_segment == pytest.approx((2.0, 4.0))
    assert s.total == pytest.approx(3.0)
