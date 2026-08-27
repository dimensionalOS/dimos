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

"""ONE span-labelling pass, and the clip schedule that respects span edges.

The distinguishing axis across recordings is WHAT IS HOLDING THE TRUNK,
because that sets the boundary condition the sim must impose: ``floor`` (the
feet), ``flight`` (nothing), ``suspended`` (a rope — declared, never
detected), ``contaminated`` (excluded). Most parameters are only observable in
one regime and invisible elsewhere: ``trunk_inertia_scale`` is 74% of the
sensitivity in flight and exactly 0.0% of the contact parameters' is.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from dimos.simulation.sysid.recording import Declarations, Streams

SpanKind = Literal["floor", "flight", "suspended", "contaminated"]


@dataclass(frozen=True)
class Span:
    kind: SpanKind
    t0: float
    t1: float


def flight_spans(st: Streams, *, threshold: float = 3.0, min_s: float = 0.03) -> np.ndarray:
    """``(n,2)`` start/end times of free fall, from the IMU. No contact in these.

    ``|specific force|`` is ~9.5 m/s2 standing and collapses to near zero the
    instant the feet leave the ground, so the accelerometer IS the contact
    sensor. It has to be: ``foot_force`` on this Go2 Air reads 40-61 counts
    and never drops, even mid-jump.
    """
    if len(st.lacc) == 0:
        return np.zeros((0, 2))
    air = np.linalg.norm(st.lacc, axis=1) < threshold
    edge = np.diff(air.astype(np.int8))
    starts = list(st.lt[1:][edge == 1])
    ends = list(st.lt[1:][edge == -1])
    if air[0]:
        starts.insert(0, float(st.lt[0]))
    if air[-1]:
        ends.append(float(st.lt[-1]))
    return np.array([(a, b) for a, b in zip(starts, ends, strict=False) if b - a >= min_s]).reshape(
        -1, 2
    )


def propose_suspended(st: Streams) -> bool:
    """Cross-check a suspension declaration; NEVER a decision.

    Unloaded legs are obvious — hanging ``|tau|`` p50 is 0.45 Nm against 2.32
    walking, a 5x separation — but a robot lying down reads the same, which is
    why suspension is declared, and this only warns on disagreement.
    """
    if len(st.ltau) == 0:
        return False
    return float(np.percentile(np.abs(st.ltau), 50)) < 1.0


def contaminated_spans(
    st: Streams,
    *,
    window_s: float = 1.0,
    accel_dev: float = 3.0,
    torque_p90: float = 12.0,
) -> np.ndarray:
    """``(n,2)`` spans matching the rope-tug signature, PROPOSED, not decided.

    Measured on the 2026-08-16 hanging file: tugs show ``|a|`` deviation
    3.9-4.1 against a ~1.0 baseline together with torque p90 19-26 Nm against
    2-7. Both must fire — jump landings spike one but not the other. Only
    validated inside suspended recordings; :func:`regimes` applies it there
    and nowhere else.
    """
    if len(st.lt) < 3:
        return np.zeros((0, 2))
    amag = np.linalg.norm(st.lacc, axis=1)
    baseline = float(np.median(amag))
    t0, t1 = float(st.lt[0]), float(st.lt[-1])
    edges = np.arange(t0, t1, window_s / 2.0)
    hot: list[tuple[float, float]] = []
    for edge in edges:
        a = float(edge)
        m = (st.lt >= a) & (st.lt < a + window_s)
        if m.sum() < 8:
            continue
        dev = float(np.sqrt(np.mean((amag[m] - baseline) ** 2)))
        tq = float(np.percentile(np.abs(st.ltau[m]), 90))
        if dev > accel_dev and tq > torque_p90:
            hot.append((a, min(a + window_s, t1)))
    if not hot:
        return np.zeros((0, 2))
    merged = [list(hot[0])]
    for a, b in hot[1:]:
        if a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return np.array(merged, dtype=float).reshape(-1, 2)


def regimes(st: Streams, declared: Declarations | None = None) -> list[Span]:
    """Label the whole recording, one pass. Suspension is a label like any other.

    A jump recording is ``floor`` spans with ``flight`` spans inside them —
    the normal case, not a special one. A declared-suspended recording is one
    ``suspended`` span with detector-proposed ``contaminated`` (rope-tug)
    spans cut out of it; flight detection is meaningless there (a hanging
    robot reads ~1 g throughout).
    """
    declared = declared or Declarations()
    t0, t1 = float(st.lt[0]), float(st.lt[-1])
    if declared.suspended:
        cut = contaminated_spans(st)
        out: list[Span] = []
        cursor = t0
        for a, b in cut:
            if a > cursor:
                out.append(Span("suspended", cursor, float(a)))
            out.append(Span("contaminated", float(a), float(b)))
            cursor = float(b)
        if cursor < t1:
            out.append(Span("suspended", cursor, t1))
        return out
    if declared.suspended is None and propose_suspended(st):
        raise ValueError(
            "this recording looks suspended (unloaded legs) but nothing declares it: "
            "add {'suspended': true|false} to the mcap's go2sim metadata or a "
            "<recording>.meta.json sidecar. Refusing to guess."
        )
    out = []
    cursor = t0
    for a, b in flight_spans(st):
        if a > cursor:
            out.append(Span("floor", cursor, float(a)))
        out.append(Span("flight", float(a), float(b)))
        cursor = float(b)
    if cursor < t1:
        out.append(Span("floor", cursor, t1))
    return out


@dataclass(frozen=True)
class Segment:
    """One stretch of the recording that gets scored at all.

    Not a clip. A CLIP is the multiple-shooting interval INSIDE a segment —
    how often the sim snaps back to measured state (:func:`clip_schedule`). A
    SEGMENT is which part of the recording contributes rows to the objective.
    Conflating the two is how "window" came to mean both, and how every fit
    inherited whatever one contiguous slice happened to contain.
    """

    t0: float
    duration: float

    @property
    def t1(self) -> float:
        return self.t0 + self.duration


def sample_segments(
    t0: float,
    t1: float,
    *,
    n: int = 8,
    length: tuple[float, float] = (4.0, 8.0),
    seed: int = 0,
) -> list[Segment]:
    """``n`` scored segments spread across ``[t0, t1]``, seeded.

    One contiguous slice is a measured flaw: the same recording resolved
    ``trunk_inertia_scale`` at 5.08 from its first 20 s and 0.40 from its
    first 50 s — a 10x swing on a step change, because the information is
    concentrated in the most dynamic stretch and t=25/t=40 are
    near-motionless. Sampling makes the objective an average over the
    recording instead of a sample of one part of it.

    Stratified, not plain-uniform: segment ``i``'s start is drawn inside the
    ``i``-th of ``n`` equal strata of the start domain, so the draws cannot
    cluster and every part of the recording is looked at. Lengths come from
    ``U(*length)``. Pure function of its arguments — shared across candidate
    plants exactly as clip schedules are, and for the same reason: segments
    that move between candidates score the sampling, not the physics.
    """
    if n < 1:
        raise ValueError(f"need at least one segment, got n={n}")
    lo, hi = length
    if lo <= 0 or hi < lo:
        raise ValueError(f"segment length must satisfy 0 < lo <= hi, got {length!r}")
    span = t1 - t0
    if span <= 0:
        raise ValueError(f"empty recording range [{t0}, {t1}]")
    rng = np.random.default_rng(seed)
    out: list[Segment] = []
    for i in range(n):
        dur = min(float(rng.uniform(lo, hi)), span)
        room = span - dur  # the start domain, stratified n ways
        start = t0 + (i + float(rng.uniform())) / n * room
        out.append(Segment(start, dur))
    return out


def clip_schedule(
    t0: float,
    duration: float,
    window: float | tuple[float, float] | None,
    *,
    seed: int = 0,
    protect: np.ndarray | None = None,
) -> np.ndarray:
    """When the sim gets snapped back to measured state, as absolute times.

    A scalar ``window`` is a fixed clip length; a ``(lo, hi)`` pair samples
    each clip from ``U(lo, hi)`` (SPI-Active, arXiv 2505.14266: a sampled
    range beats any fixed length; U(0.05, 0.8) rather than the paper's
    U(0.05, 2.0), which drowns the signal in long-clip tail variance).

    SEEDED AND PRECOMPUTED ON PURPOSE: the boundaries must not move between
    candidate plants, or the search optimises the schedule instead of the
    physics. Pure function of ``(t0, duration, window, seed, protect)``.

    ``protect`` is ``(n,2)`` spans a boundary must NEVER fall inside — flight
    spans above all: a re-initialisation inside a ballistic arc snaps the sim
    back to measured state part-way through and the trunk counter-rotation
    never accumulates (``trunk_inertia_scale`` drops from 74% to 4%
    sensitivity). Boundaries inside a span are dropped and a fresh clip opens
    at the span's start instead.
    """
    if window is None:
        return np.zeros(0)
    lo, hi = window if isinstance(window, tuple) else (window, window)
    if lo <= 0 or hi < lo:
        raise ValueError(f"window must satisfy 0 < lo <= hi, got {window!r}")
    rng = np.random.default_rng(seed) if hi > lo else None
    out: list[float] = []
    t, end = t0, t0 + duration
    while True:
        t += float(rng.uniform(lo, hi)) if rng is not None else lo
        if t >= end:
            break
        out.append(t)
    if protect is None or len(protect) == 0:
        return np.array(out)
    grid = np.array(out)
    inside = np.zeros(len(grid), bool)
    for a, b in protect:
        inside |= (grid >= a) & (grid <= b)
    kept = list(grid[~inside])
    kept += [float(a) for a, _b in protect if t0 < a < end]
    return np.unique(np.array(sorted(kept)))


def protected(spans: list[Span]) -> np.ndarray:
    """The ``(n,2)`` spans a clip boundary must not straddle: flight + contaminated."""
    keep = [(s.t0, s.t1) for s in spans if s.kind in ("flight", "contaminated")]
    return np.array(keep, dtype=float).reshape(-1, 2)
