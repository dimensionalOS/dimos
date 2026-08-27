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

"""The score is a weight vector over (channel, regime).

    score = mean over segments of  Σ  w[c, r] · residual[c, r] / scale[c]

``scale[c]`` makes the terms addable — m/s², rad and N·m cannot be summed, and
without it the weights would encode unit choices rather than importance. It is
each channel's residual RMS under the BASELINE plant, computed once on the
shared schedule and then FROZEN: a scale that moved with the candidate would
make the search see a moving objective, exactly as an unshared clip schedule
would. (The noise floor replaces the baseline RMS once a repeat recording
exists.)

Scoring compares the INTERSECTION of what the recording has, what the backend
predicts (:meth:`~dimos.simulation.sysid.backend.Backend.channels`), and
what the regime permits. No tracker → ``pos``/``rot`` are absent and the rest
proceeds. A suspended recording has a held trunk, so every trunk-frame signal
is an echo of the boundary condition and drops out. Adding a tracker adds a
channel and changes nothing else — no second code path, no "VR mode".

This subsumes the frozen tool's ``w_flight``: "which channel do we score" and
"how much does flight count" are entries in one vector. That vector is a
hyperparameter of the METHOD, not a property of the robot — loop 2 selects it.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from dimos.simulation.sysid.backend import CHANNELS
from dimos.simulation.sysid.regimes import Span
from dimos.simulation.sysid.replay import ReplayResult
from dimos.simulation.sysid.rotations import rotation_angle

# One (channel, regime) weight vector. Keys are (channel, span kind).
WeightVector = Mapping[tuple[str, str], float]

# THE PARTITION (README 4): loop 1 scores only the joint-level channels;
# body-level ones (`accel`, `gyro`, `pos`, `rot`) are the referee's and
# carry zero weight — `accel` deliberately so despite resolving 11 of 14
# knobs vs dq's 4, because any accel weight re-couples the fit to the
# quantity loop 2 judges (the suspected anti-transfer overlap). Zero
# weight never means unreported. Within the family, identifiability
# (sysid.identify, 194142 + jumps): dq resolves 4 of 7 searched knobs
# (only channel to resolve actuator_tau), joint 3, tau the same 3 from an
# ESTIMATE — so tau gets half weight. w_flight 0.25 emphasises the 2.5 s
# of flight ~6x its sample share. Chosen from the spectrum, never
# validated — loop 2's outer study owns these numbers.
DEFAULT_WEIGHTS: dict[tuple[str, str], float] = {
    ("joint", "floor"): 0.30,
    ("joint", "flight"): 0.10,
    ("dq", "floor"): 0.30,
    ("dq", "flight"): 0.10,
    ("tau", "floor"): 0.15,
    ("tau", "flight"): 0.05,
}

# The one honest choice for a hanging recording: the trunk is held, so only
# the joint-space channels measure physics there (see PERMITTED), and `joint`
# is the channel `damping` was fitted on.
SUSPENDED_WEIGHTS: dict[tuple[str, str], float] = {("joint", "suspended"): 1.0}

# What each regime permits. A held trunk makes every trunk-frame signal an
# echo of the boundary condition — `accel` is the weld's reaction force,
# `gyro`/`pos`/`rot` follow the mocap track the sim was TOLD, so scoring them
# would grade the bookkeeping. Only the joint-space channels measure physics
# on a suspended span.
PERMITTED: dict[str, frozenset[str]] = {
    "floor": frozenset(CHANNELS),
    "flight": frozenset(CHANNELS),
    "suspended": frozenset({"joint", "dq", "tau"}),
}


def predicted(r: ReplayResult, channel: str) -> np.ndarray:
    """The prediction whose movement a Jacobian measures, flattened.

    WHICH CHANNEL MATTERS MORE THAN THE KNOBS. Trunk inertia hardly moves a
    joint angle — it moves the BODY, by counter-rotating it against the legs;
    scoring only ``joint`` reports it unidentifiable no matter how good the
    data is. ``accel`` resolves 11-12 of 14 knobs against ``joint``'s 3-4 —
    and is scored at zero anyway (:data:`DEFAULT_WEIGHTS`): resolution is
    not correctness, and the body-level channels are the referee's
    territory. The unresolved knobs ship as spread, not as fiction.
    """
    p = r.prediction
    if channel == "joint":
        return p.q.ravel()
    if channel == "dq":
        return p.dq.ravel()
    if channel == "rot":
        return p.body_rot.reshape(len(p.body_rot), 9).ravel()  # entries move ~1:1 with radians
    if channel == "pos":
        return p.body_pos.ravel()
    if channel == "accel":
        return p.imu_accel.ravel()
    if channel == "gyro":
        return p.imu_gyro.ravel()
    if channel == "tau":
        return p.tau.ravel()
    raise ValueError(f"unknown channel {channel!r}: expected one of {CHANNELS}")


def residual(r: ReplayResult, channel: str) -> np.ndarray | None:
    """Flat residual (sim - real) on the channel; ``None`` without a real side."""
    p = r.prediction
    out: np.ndarray
    if channel == "joint":
        out = (p.q - r.q_real).ravel()
    elif channel == "dq":
        out = (p.dq - r.dq_real).ravel()
    elif channel == "rot":
        if r.r_real is None:
            return None
        out = (p.body_rot - r.r_real).reshape(len(p.body_rot), 9).ravel()
    elif channel == "pos":
        if r.p_real is None:
            return None
        out = (p.body_pos - r.p_real).ravel()
    elif channel == "accel":
        out = (p.imu_accel - r.a_real).ravel()
    elif channel == "gyro":
        out = (p.imu_gyro - r.w_real).ravel()
    elif channel == "tau":
        out = (p.tau - r.tau_real).ravel()
    else:
        raise ValueError(f"unknown channel {channel!r}: expected one of {CHANNELS}")
    return out


def sample_errors(r: ReplayResult, channel: str) -> tuple[np.ndarray, np.ndarray] | None:
    """``(t, err)`` per-sample error on the channel's own time base.

    ``None`` when the recording has no real side for the channel (``pos`` /
    ``rot`` without a tracker) — the caller drops the term and proceeds.
    Vector channels reduce to a per-sample root-mean-square across components,
    so every channel's error is one number per sample in its own unit; ``pos``
    is the planar error and ``rot`` the rotation angle, the conventions
    :meth:`~dimos.simulation.sysid.replay.ReplayResult.body_err`
    already uses.
    """
    p = r.prediction

    def rms_rows(diff: np.ndarray) -> np.ndarray:
        out: np.ndarray = np.sqrt(np.mean(diff * diff, axis=1))
        return out

    if channel == "joint":
        return p.t, rms_rows(p.q - r.q_real)
    if channel == "dq":
        return p.t, rms_rows(p.dq - r.dq_real)
    if channel == "tau":
        return p.at, rms_rows(p.tau - r.tau_real)
    if channel == "accel":
        return p.at, rms_rows(p.imu_accel - r.a_real)
    if channel == "gyro":
        return p.at, rms_rows(p.imu_gyro - r.w_real)
    if channel == "pos":
        if r.p_real is None:
            return None
        return p.t, np.linalg.norm(p.body_pos[:, :2] - r.p_real[:, :2], axis=1)
    if channel == "rot":
        if r.r_real is None:
            return None
        return p.t, rotation_angle(p.body_rot, r.r_real)
    raise ValueError(f"unknown channel {channel!r}: expected one of {CHANNELS}")


def _kind_of(spans: Sequence[Span], t: np.ndarray) -> np.ndarray:
    """The span kind containing each sample time, as an index into ``spans``.

    -1 for samples outside every span (before the first / after the last).
    Spans are the contiguous, ordered output of :func:`regimes`.
    """
    if not spans:
        return np.full(len(t), -1)
    starts = np.array([s.t0 for s in spans])
    idx = np.searchsorted(starts, t, "right") - 1
    ends = np.array([s.t1 for s in spans])
    idx = np.asarray(idx)
    out_of_span = (idx < 0) | (t > ends[np.clip(idx, 0, len(spans) - 1)])
    return np.where(out_of_span, -1, idx)


@dataclass(frozen=True)
class SegmentTerms:
    """One segment's per-(channel, regime) error sums, ready to be pooled.

    ``sums[c, r] = (Σ per-sample error, n)`` over the segment's samples lying
    in spans of kind ``r``; ``sq[c] = (Σ error², n)`` over ALL scored regimes,
    which is what the baseline scales are computed from. Contaminated spans
    contribute to neither.
    """

    sums: dict[tuple[str, str], tuple[float, int]]
    sq: dict[str, tuple[float, int]]


def segment_terms(r: ReplayResult, spans: Sequence[Span], channels: Iterable[str]) -> SegmentTerms:
    """Reduce one rollout to its scoreable terms, masked by regime.

    Only ``channels`` are computed; a channel the recording cannot answer
    (no tracker) is silently absent, and a (channel, regime) pair the regime
    does not permit (:data:`PERMITTED`) is never produced — the intersection
    rule, enforced here and nowhere else.
    """
    sums: dict[tuple[str, str], tuple[float, int]] = {}
    sq: dict[str, tuple[float, int]] = {}
    for c in channels:
        te = sample_errors(r, c)
        if te is None:
            continue
        t, err = te
        kind_idx = _kind_of(spans, t)
        s_sq = 0.0
        n_sq = 0
        for si in np.unique(kind_idx):
            if si < 0:
                continue
            kind = spans[int(si)].kind
            if kind not in PERMITTED or c not in PERMITTED[kind]:
                continue
            e = err[kind_idx == si]
            prev = sums.get((c, kind), (0.0, 0))
            sums[(c, kind)] = (prev[0] + float(e.sum()), prev[1] + len(e))
            s_sq += float((e * e).sum())
            n_sq += len(e)
        if n_sq:
            sq[c] = (s_sq, n_sq)
    return SegmentTerms(sums=sums, sq=sq)


def scales_from(terms: Sequence[SegmentTerms]) -> dict[str, float]:
    """Per-channel residual RMS pooled over segments — the term normaliser.

    Computed from the BASELINE plant's rollouts and then frozen for the whole
    fit; see the module docstring for why it must not move with the candidate.
    """
    acc: dict[str, tuple[float, int]] = {}
    for st in terms:
        for c, (s, n) in st.sq.items():
            prev = acc.get(c, (0.0, 0))
            acc[c] = (prev[0] + s, prev[1] + n)
    return {c: float(np.sqrt(s / n)) for c, (s, n) in acc.items() if n > 0 and s > 0}


@dataclass(frozen=True)
class Score:
    """One candidate's score: the total, and what it is made of.

    ``total`` is the mean over segments of each segment's weighted term sum,
    renormalised over the terms that EXIST there — a segment with no flight
    has no flight term, and ``0.0 * nan`` silently failing every trial over
    grounded data is a measured failure mode, not a hypothetical.
    ``per_segment`` feeds the harvest tolerance: the loss is a mean over
    segments, so its own sampling noise is the SE of that mean.
    """

    total: float
    terms: dict[tuple[str, str], float]  # pooled mean per-sample error, natural units
    per_segment: tuple[float, ...]


def score_terms(
    seg_terms: Sequence[SegmentTerms],
    weights: WeightVector,
    scales: Mapping[str, float],
) -> Score:
    """Combine per-segment terms under the weight vector. Pure arithmetic."""
    per_segment: list[float] = []
    for st in seg_terms:
        num = den = 0.0
        for (c, kind), w in weights.items():
            if w <= 0 or (c, kind) not in st.sums or c not in scales:
                continue
            s, n = st.sums[(c, kind)]
            num += w * (s / n) / scales[c]
            den += w
        if den > 0:
            per_segment.append(num / den)
    if not per_segment:
        raise ValueError(
            "the weight vector selects nothing this data can score: no scored "
            "segment has any weighted (channel, regime) term. A suspended "
            "recording needs joint/dq/tau weights; a tracker-less one has no "
            "pos/rot."
        )
    pooled: dict[tuple[str, str], tuple[float, int]] = {}
    for st in seg_terms:
        for key, (s, n) in st.sums.items():
            prev = pooled.get(key, (0.0, 0))
            pooled[key] = (prev[0] + s, prev[1] + n)
    return Score(
        total=float(np.mean(per_segment)),
        terms={k: s / n for k, (s, n) in pooled.items() if n},
        per_segment=tuple(per_segment),
    )
