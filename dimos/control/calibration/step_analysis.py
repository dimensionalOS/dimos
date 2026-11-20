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

"""Step-response metrics for one joint, and the gains they argue for.

Everything here works on plain arrays in degrees and seconds, keyed by joint
name, so it can be tested on synthetic responses and reused by the offline
report for any arm. A joint rests short of its target by its friction over kp
(the drive has no integrator), so each step's rest limit is
``max(0.1 deg, 1.5 x friction / kp)``, and its settle time is measured to
within that limit of where it comes to rest. A step that moved less than half
its size is stuck: its overshoot and settle time are not measured. A step
fails when its hold was cut short, a sample or metric is not finite, it is
stuck, it overshoots more than 20 % or it buzzes at rest. ``propose_gains``
ranks, per joint, the candidates whose steps all pass in both directions by
worst rest error over its limit plus worst settle time over 0.5 s, and picks
the lowest; it says why the others lost. A stuck step smaller than a
candidate's largest does not reject it: it is listed and left out of its
score.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass, field
import json
import math
from typing import Any

import numpy as np

FIRST_MOTION_DEG = 0.05
SETTLE_BAND_DEG = 0.1
FRICTION_REST_FACTOR = 1.5
SETTLE_TIME_LIMIT_S = 0.5
OVERSHOOT_LIMIT_PCT = 20.0
STUCK_FRACTION = 0.5
KD_LIMIT = 5.0
KD_PER_KP_LIMIT = 0.25
# RMS joint velocity over the last part of the hold; a joint at rest reads
# encoder noise only, a buzzing one oscillates at a few deg/s.
BUZZ_RMS_LIMIT_DEG_S = 1.0
REST_WINDOW_S = 0.5
# A hold is complete when its samples reach this close to its end (a few
# samples at the 100 Hz joint state).
HOLD_COMPLETE_TOLERANCE_S = 0.05
# The metrics the pass criteria compare; a NaN or infinite one fails the step.
CRITERIA_METRICS = ("settle_time_s", "overshoot_pct", "steady_state_error_deg", "buzz_rms_deg_s")


@dataclass(frozen=True)
class StepEvent:
    """Marker published by the step test module when a step starts."""

    joint: str
    delta_deg: float
    kp: float
    kd: float
    t0: float
    hold_s: float
    target_before_deg: float
    target_after_deg: float

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, text: str) -> StepEvent:
        data = json.loads(text)
        return cls(**{key: data[key] for key in cls.__dataclass_fields__})


@dataclass(frozen=True)
class StepMetrics:
    """Metrics of one recorded step; times are from the step command.

    ``hold_s`` is the hold the step asked for, ``recorded_s`` the time of its
    last recorded sample; a hold cut short (by the next step or the end of the
    recording) leaves the rest metrics unreliable, and the step fails.
    ``samples`` counts the finite samples the metrics come from;
    ``nonfinite_samples`` those left out for a NaN or infinite value, which
    fail the step too, as does a step with no samples at all. ``moved_deg``
    is how far the joint came to rest from where it started, along the step;
    a step that moved less than ``STUCK_FRACTION`` of its size is stuck, and
    its ``overshoot_pct`` and ``settle_time_s`` are ``None``. The settle time
    is to within ``rest_limit_deg`` (from the joint's ``friction_nm``) of the
    rest, ``None`` too when the last sample is outside it.
    """

    joint: str
    delta_deg: float
    kp: float
    kd: float
    samples: int
    first_motion_s: float | None
    settle_time_s: float | None
    overshoot_pct: float | None
    steady_state_error_deg: float
    buzz_rms_deg_s: float
    hold_s: float
    recorded_s: float
    nonfinite_samples: int = 0
    moved_deg: float = math.nan
    rest_limit_deg: float = SETTLE_BAND_DEG
    friction_nm: float = 0.0

    @property
    def hold_complete(self) -> bool:
        """Whether the samples cover the whole hold."""
        return self.recorded_s >= self.hold_s - HOLD_COMPLETE_TOLERANCE_S

    @property
    def stuck(self) -> bool:
        return self.samples > 0 and self.moved_deg < STUCK_FRACTION * abs(self.delta_deg)

    @property
    def passed(self) -> bool:
        return not self.failures()

    @property
    def settled_s(self) -> float:
        return self.recorded_s if self.settle_time_s is None else self.settle_time_s

    def failures(self, *, stuck: bool = True) -> list[str]:
        """Why this step fails the pass criteria, in plain words; empty when it passes.

        With ``stuck`` False a stuck step is not one of the reasons.
        """
        reasons: list[str] = []
        step = f"{self.delta_deg:+g} deg step"
        if self.nonfinite_samples:
            reasons.append(f"{step} has {self.nonfinite_samples} non-finite sample(s)")
        if self.samples == 0:
            reasons.append(
                f"{step} has no {'finite' if self.nonfinite_samples else 'recorded'} samples"
            )
            return reasons
        if not self.hold_complete:
            reasons.append(
                f"{step} hold was cut short: {self.recorded_s:.2f} of {self.hold_s:g} s recorded"
            )
        finite: dict[str, bool] = {}
        for name in CRITERIA_METRICS:
            value = getattr(self, name)
            finite[name] = value is None or math.isfinite(value)
            if not finite[name]:
                reasons.append(f"{step} metric {name} is not finite")
        if stuck and self.stuck:
            reasons.append(
                f"{step} is stuck: moved {self.moved_deg:+.2f} deg "
                f"(less than {STUCK_FRACTION:.0%} of the step)"
            )
        if (
            self.overshoot_pct is not None
            and finite["overshoot_pct"]
            and self.overshoot_pct > OVERSHOOT_LIMIT_PCT
        ):
            reasons.append(
                f"{step} overshot {self.overshoot_pct:.0f} % (limit {OVERSHOOT_LIMIT_PCT:g} %)"
            )
        if finite["buzz_rms_deg_s"] and self.buzz_rms_deg_s > BUZZ_RMS_LIMIT_DEG_S:
            reasons.append(
                f"{step} buzzes at {self.buzz_rms_deg_s:.1f} deg/s RMS "
                f"(limit {BUZZ_RMS_LIMIT_DEG_S:g} deg/s)"
            )
        return reasons

    def as_row(self) -> dict[str, Any]:
        row = asdict(self)
        row["hold_complete"] = self.hold_complete
        row["passed"] = self.passed
        return row


def first_motion_time(
    t: np.ndarray, y: np.ndarray, y0: float, threshold_deg: float = FIRST_MOTION_DEG
) -> float | None:
    """Time of the first sample further than ``threshold_deg`` from the start value."""
    moved = np.flatnonzero(np.abs(y - y0) > threshold_deg)
    return float(t[moved[0]]) if moved.size else None


def settle_time(
    t: np.ndarray, y: np.ndarray, target: float, band_deg: float = SETTLE_BAND_DEG
) -> float | None:
    """Time after which every remaining sample stays within ``band_deg`` of ``target``.

    ``None`` when the last sample is still outside the band.
    """
    outside = np.abs(y - target) > band_deg
    if outside.size == 0 or outside[-1]:
        return None
    last_outside = np.flatnonzero(outside)
    if last_outside.size == 0:
        return float(t[0])
    return float(t[last_outside[-1] + 1])


def overshoot_pct(y: np.ndarray, y0: float, target: float) -> float:
    """Peak excursion past the target, as a percentage of the step size (0 if none)."""
    delta = target - y0
    if abs(delta) < 1e-12:
        return 0.0
    past = (y - target) * math.copysign(1.0, delta)
    return float(max(0.0, past.max() / abs(delta) * 100.0))


def rest_window(t: np.ndarray, window_s: float = REST_WINDOW_S) -> np.ndarray:
    """Mask of the last ``window_s`` of the record, at most its second half."""
    span = min(window_s, 0.5 * (t[-1] - t[0]))
    return t >= t[-1] - span  # type: ignore[no-any-return]


def steady_state_error(
    t: np.ndarray, y: np.ndarray, target: float, window_s: float = REST_WINDOW_S
) -> float:
    return float(y[rest_window(t, window_s)].mean() - target)


def rest_limit_deg(friction_nm: float, kp: float) -> float:
    return max(SETTLE_BAND_DEG, math.degrees(FRICTION_REST_FACTOR * friction_nm / kp))


def buzz_rms(t: np.ndarray, dy: np.ndarray, window_s: float = REST_WINDOW_S) -> float:
    tail = dy[rest_window(t, window_s)]
    return float(np.sqrt(np.mean(np.square(tail))))


def analyze_step(
    event: StepEvent,
    t: np.ndarray,
    y_deg: np.ndarray,
    dy_deg_s: np.ndarray,
    y0_deg: float | None = None,
    friction_nm: float = 0.0,
) -> StepMetrics:
    """Metrics of one step from samples ``t`` (seconds since the command), ``y`` and ``dy``.

    A sample with a non-finite value is left out of the metrics and fails the
    step; with no samples left the step fails and its metrics are NaN. The
    joint's ``friction_nm`` sets the rest limit (``rest_limit_deg``).
    """
    t = np.asarray(t, dtype=np.float64)
    y_deg = np.asarray(y_deg, dtype=np.float64)
    dy_deg_s = np.asarray(dy_deg_s, dtype=np.float64)
    finite = np.isfinite(t) & np.isfinite(y_deg) & np.isfinite(dy_deg_s)
    nonfinite = int(finite.size - finite.sum())
    t, y_deg, dy_deg_s = t[finite], y_deg[finite], dy_deg_s[finite]
    limit = rest_limit_deg(friction_nm, event.kp)
    if t.size == 0:
        return StepMetrics(
            joint=event.joint,
            delta_deg=event.delta_deg,
            kp=event.kp,
            kd=event.kd,
            samples=0,
            first_motion_s=None,
            settle_time_s=None,
            overshoot_pct=math.nan,
            steady_state_error_deg=math.nan,
            buzz_rms_deg_s=math.nan,
            hold_s=event.hold_s,
            recorded_s=0.0,
            nonfinite_samples=nonfinite,
            rest_limit_deg=limit,
            friction_nm=friction_nm,
        )
    y0 = event.target_before_deg if y0_deg is None else y0_deg
    target = event.target_after_deg
    error = steady_state_error(t, y_deg, target)
    moved = (target + error - y0) * math.copysign(1.0, event.delta_deg)
    stuck = moved < STUCK_FRACTION * abs(event.delta_deg)
    return StepMetrics(
        joint=event.joint,
        delta_deg=event.delta_deg,
        kp=event.kp,
        kd=event.kd,
        samples=int(t.size),
        first_motion_s=first_motion_time(t, y_deg, y0),
        settle_time_s=None if stuck else settle_time(t, y_deg, target + error, limit),
        overshoot_pct=None if stuck else overshoot_pct(y_deg, y0, target),
        steady_state_error_deg=error,
        buzz_rms_deg_s=buzz_rms(t, dy_deg_s),
        hold_s=event.hold_s,
        recorded_s=float(t[-1]),
        nonfinite_samples=nonfinite,
        moved_deg=moved,
        rest_limit_deg=limit,
        friction_nm=friction_nm,
    )


def sweep_friction_nm(steps: Iterable[StepMetrics]) -> dict[str, float]:
    first: dict[str, tuple[float, float]] = {}
    largest: dict[str, dict[int, StepMetrics]] = defaultdict(dict)
    rests: dict[str, list[float]] = defaultdict(list)
    for step in steps:
        gains = first.setdefault(step.joint, (step.kp, step.kd))
        if (step.kp, step.kd) != gains or step.samples == 0:
            continue
        if not step.stuck:
            rests[step.joint].append(abs(step.steady_state_error_deg))
        sign = int(math.copysign(1, step.delta_deg))
        held = largest[step.joint].get(sign)
        if held is None or abs(step.delta_deg) > abs(held.delta_deg):
            largest[step.joint][sign] = step
    friction: dict[str, float] = {}
    for joint, by_sign in largest.items():
        offsets = list(rests[joint])
        if 1 in by_sign and -1 in by_sign:
            band = by_sign[-1].steady_state_error_deg - by_sign[1].steady_state_error_deg
            offsets.append(band / 2.0)
        if offsets:
            friction[joint] = first[joint][0] * math.radians(max(0.0, *offsets))
    return friction


def capped_kd(kp: float, kd: float) -> float:
    return min(kd, KD_LIMIT, KD_PER_KP_LIMIT * kp)


def gain_candidates(
    kp0: float, kd0: float, kp_scales: Sequence[float], kd_scales: Sequence[float]
) -> list[tuple[float, float]]:
    baseline = (kp0, capped_kd(kp0, kd0))
    grid = [(kp0 * p, capped_kd(kp0 * p, kd0 * d)) for p in kp_scales for d in kd_scales]
    return list(dict.fromkeys(sorted(grid, key=lambda pair: pair != baseline)))


def segment_steps(events: list[StepEvent], t_end: float) -> list[tuple[StepEvent, float, float]]:
    """Window ``(event, start, stop)`` per step: the hold, cut short by the next step."""
    ordered = sorted(events, key=lambda event: event.t0)
    windows: list[tuple[StepEvent, float, float]] = []
    for index, event in enumerate(ordered):
        stop = event.t0 + event.hold_s
        if index + 1 < len(ordered):
            stop = min(stop, ordered[index + 1].t0)
        windows.append((event, event.t0, min(stop, t_end)))
    return windows


def gain_label(kp: float, kd: float) -> str:
    return f"kp={kp:g} kd={kd:g}"


def format_table(metrics: list[StepMetrics]) -> str:
    """Markdown table, one row per step, grouped by joint then gains."""
    header = (
        "| joint | kp | kd | step (deg) | moved (deg) | first motion (s) | settle (s) "
        "| overshoot (%) | rest error (deg) | rest limit (deg) | buzz (deg/s) | pass |"
    )
    lines = [header, "|" + "---|" * 12]
    for m in sorted(metrics, key=lambda m: (m.joint, m.kp, m.kd, -abs(m.delta_deg), m.delta_deg)):
        settle = "-" if m.stuck else _fmt(m.settle_time_s)
        overshoot = "-" if m.overshoot_pct is None else f"{m.overshoot_pct:.1f}"
        verdict = "stuck" if m.stuck else "ok" if m.passed else "FAIL"
        lines.append(
            f"| {m.joint} | {m.kp:g} | {m.kd:g} | {m.delta_deg:+.1f} | {m.moved_deg:+.2f} "
            f"| {_fmt(m.first_motion_s)} | {settle} | {overshoot} "
            f"| {m.steady_state_error_deg:+.3f} | {m.rest_limit_deg:.2f} | {m.buzz_rms_deg_s:.2f} "
            f"| {verdict} |"
        )
    lines.append("")
    lines.append(
        f"Pass: none of the largest steps stuck (moved less than {STUCK_FRACTION:.0%} of its "
        f"size), overshoot at most {OVERSHOOT_LIMIT_PCT:g} %, buzz RMS at most "
        f"{BUZZ_RMS_LIMIT_DEG_S:g} deg/s over the last {REST_WINDOW_S} s. Passing gains are "
        f"ranked, over the steps that moved, by worst rest error over the rest limit, "
        f"max({SETTLE_BAND_DEG:g} deg, {FRICTION_REST_FACTOR:g} x friction / kp), plus worst "
        f"settle time, to within that limit of the rest, over {SETTLE_TIME_LIMIT_S:g} s; the "
        "lowest wins."
    )
    return "\n".join(lines)


def _fmt(value: float | None) -> str:
    return "never" if value is None else f"{value:.3f}"


@dataclass(frozen=True)
class GainProposal:
    """The gains ``propose_gains`` picked for one joint, and why.

    Attributes:
        joint: The joint.
        kp: The chosen position gain, or ``None`` when no candidate passed.
        kd: The chosen velocity gain, or ``None`` when no candidate passed.
        reason: One line on the choice.
        rejected: Why each candidate that failed, was not tested or lost on
            its score was passed over, by ``gain_label``.
        friction_nm: The joint's friction its rest limits come from.
    """

    joint: str
    kp: float | None
    kd: float | None
    reason: str
    rejected: dict[str, str] = field(default_factory=dict)
    friction_nm: float = 0.0

    @property
    def gains(self) -> tuple[float, float] | None:
        if self.kp is None or self.kd is None:
            return None
        return (self.kp, self.kd)


def propose_gains(
    step_results: Iterable[StepMetrics],
    candidates: Sequence[tuple[float, float]] | None = None,
) -> dict[str, GainProposal]:
    """Per joint, the passing candidate among ``candidates`` with the lowest score.

    Candidates are ``(kp, kd)`` pairs; by default every pair with a recorded
    step. A candidate passes when it has at least one positive and one
    negative step on the joint and every one of its steps passes (whole hold
    recorded, finite samples and metrics, not stuck, overshoots at most 20 %
    where measured, does not buzz), but for a stuck step smaller than its
    largest, which is listed and left out. Its score, over the steps that
    moved, is its worst rest error over its rest limit plus its worst settle
    time over 0.5 s; the lowest wins, a
    tie going to the higher kp, then the lower kd. When none passes and every
    candidate at the highest kd tried, the driver's ``KD_LIMIT``, overshoots,
    the reason says to lower kp. ``rejected`` explains every other candidate.
    """
    by_joint: dict[str, dict[tuple[float, float], list[StepMetrics]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for metrics in step_results:
        by_joint[metrics.joint][(metrics.kp, metrics.kd)].append(metrics)
    proposals: dict[str, GainProposal] = {}
    for joint, by_gains in by_joint.items():
        pairs = sorted(
            set(by_gains) if candidates is None else {(float(p), float(d)) for p, d in candidates}
        )
        friction = next(iter(by_gains.values()))[0].friction_nm
        rejected: dict[str, str] = {}
        scored: list[tuple[float, float, float]] = []
        for kp, kd in pairs:
            steps = by_gains.get((kp, kd), [])
            if not steps:
                rejected[gain_label(kp, kd)] = "not tested"
            elif why := _gain_failures(steps):
                rejected[gain_label(kp, kd)] = why
            else:
                scored.append((_score(steps), kp, kd))
        if not scored:
            reason = "no candidate passed every step"
            if _overshoots_at_kd_limit(by_gains):
                reason = (
                    f"overshoots more than {OVERSHOOT_LIMIT_PCT:g} % at every kp; kd at the "
                    f"driver limit {KD_LIMIT:g}; lower kp"
                )
            proposals[joint] = GainProposal(joint, None, None, reason, rejected, friction)
            continue
        score, kp, kd = min(scored, key=lambda entry: (entry[0], -entry[1], entry[2]))
        for other, other_kp, other_kd in scored:
            if (other_kp, other_kd) != (kp, kd):
                rejected[gain_label(other_kp, other_kd)] = (
                    f"passes, but scores {other:.2f} against {score:.2f}"
                    f"{_stuck_note(by_gains[(other_kp, other_kd)])}"
                )
        steps = [s for s in by_gains[(kp, kd)] if not s.stuck]
        overshoots = [s.overshoot_pct for s in steps if s.overshoot_pct is not None]
        reason = (
            f"{gain_label(kp, kd)}: lowest score {score:.2f} of {len(scored)} passing "
            f"candidate(s): rests up to {max(abs(s.steady_state_error_deg) for s in steps):.2f} "
            f"deg off target (limit {steps[0].rest_limit_deg:.2f} deg, friction "
            f"{friction:.2f} Nm), settles in {max(s.settled_s for s in steps):.2f} s, "
            f"overshoot {max(overshoots, default=0.0):.0f} %, "
            f"buzz {max(s.buzz_rms_deg_s for s in steps):.2f} deg/s"
            f"{_stuck_note(by_gains[(kp, kd)])}"
        )
        proposals[joint] = GainProposal(joint, kp, kd, reason, rejected, friction)
    return proposals


def _score(steps: Sequence[StepMetrics]) -> float:
    moved = [step for step in steps if not step.stuck]
    rest = max(abs(step.steady_state_error_deg) / step.rest_limit_deg for step in moved)
    return rest + max(step.settled_s for step in moved) / SETTLE_TIME_LIMIT_S


def _stuck_note(steps: Sequence[StepMetrics]) -> str:
    stuck = [f"{step.delta_deg:+g}" for step in steps if step.stuck]
    return f"; the {', '.join(stuck)} deg step(s) stuck and do not count" if stuck else ""


def _overshoots_at_kd_limit(by_gains: dict[tuple[float, float], list[StepMetrics]]) -> bool:
    stiffest = max(kd for _, kd in by_gains)
    if stiffest < KD_LIMIT - 1e-9:
        return False
    return all(
        any(s.overshoot_pct is not None and s.overshoot_pct > OVERSHOOT_LIMIT_PCT for s in steps)
        for (_, kd), steps in by_gains.items()
        if kd == stiffest
    )


def _gain_failures(steps: Sequence[StepMetrics]) -> str:
    """Why one gain pair's steps on a joint do not pass; empty when they do."""
    largest = max(abs(step.delta_deg) for step in steps)
    reasons = [
        reason for step in steps for reason in step.failures(stuck=abs(step.delta_deg) >= largest)
    ]
    if not any(step.delta_deg > 0.0 for step in steps):
        reasons.append("no positive step recorded")
    if not any(step.delta_deg < 0.0 for step in steps):
        reasons.append("no negative step recorded")
    return "; ".join(reasons)


def format_proposals(proposals: dict[str, GainProposal]) -> str:
    """Markdown list: the chosen gains per joint, its friction, and why the others lost."""
    if not proposals:
        return "(no steps)"
    lines: list[str] = []
    for joint in sorted(proposals):
        proposal = proposals[joint]
        lines.append(f"- {joint} (friction {proposal.friction_nm:.2f} Nm): {proposal.reason}")
        for label, why in proposal.rejected.items():
            lines.append(f"  - {label} rejected: {why}")
    return "\n".join(lines)
