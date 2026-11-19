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
report for any arm. A step passes when its whole hold was recorded, every
metric is finite, and it settles within 0.5 s to within 0.1 deg, overshoots
less than 20 % and leaves the joint quiet at rest. ``propose_gains`` picks,
per joint, the stiffest tried gains whose steps all pass in both directions
and below which every tried candidate (every lower kp scale, whatever its kd)
passed too; it says why the others did not.
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
SETTLE_TIME_LIMIT_S = 0.5
OVERSHOOT_LIMIT_PCT = 20.0
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
    """

    joint: str
    delta_deg: float
    kp: float
    kd: float
    samples: int
    first_motion_s: float | None
    settle_time_s: float | None
    overshoot_pct: float
    steady_state_error_deg: float
    buzz_rms_deg_s: float
    hold_s: float
    recorded_s: float

    @property
    def hold_complete(self) -> bool:
        """Whether the samples cover the whole hold."""
        return self.recorded_s >= self.hold_s - HOLD_COMPLETE_TOLERANCE_S

    @property
    def passed(self) -> bool:
        return not self.failures()

    def failures(self) -> list[str]:
        """Why this step fails the pass criteria, in plain words; empty when it passes."""
        reasons: list[str] = []
        step = f"{self.delta_deg:+g} deg step"
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
        if self.settle_time_s is None:
            reasons.append(f"{step} never settled within {SETTLE_BAND_DEG:g} deg")
        elif finite["settle_time_s"] and self.settle_time_s > SETTLE_TIME_LIMIT_S:
            reasons.append(
                f"{step} settled in {self.settle_time_s:.2f} s (limit {SETTLE_TIME_LIMIT_S:g} s)"
            )
        if finite["overshoot_pct"] and self.overshoot_pct >= OVERSHOOT_LIMIT_PCT:
            reasons.append(
                f"{step} overshot {self.overshoot_pct:.0f} % (limit {OVERSHOOT_LIMIT_PCT:g} %)"
            )
        if finite["steady_state_error_deg"] and abs(self.steady_state_error_deg) > SETTLE_BAND_DEG:
            reasons.append(
                f"{step} rests {self.steady_state_error_deg:+.2f} deg off target "
                f"(limit {SETTLE_BAND_DEG:g} deg)"
            )
        if finite["buzz_rms_deg_s"] and self.buzz_rms_deg_s >= BUZZ_RMS_LIMIT_DEG_S:
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


def buzz_rms(t: np.ndarray, dy: np.ndarray, window_s: float = REST_WINDOW_S) -> float:
    tail = dy[rest_window(t, window_s)]
    return float(np.sqrt(np.mean(np.square(tail))))


def analyze_step(
    event: StepEvent,
    t: np.ndarray,
    y_deg: np.ndarray,
    dy_deg_s: np.ndarray,
    y0_deg: float | None = None,
) -> StepMetrics:
    """Metrics of one step from samples ``t`` (seconds since the command), ``y`` and ``dy``."""
    t = np.asarray(t, dtype=np.float64)
    y_deg = np.asarray(y_deg, dtype=np.float64)
    dy_deg_s = np.asarray(dy_deg_s, dtype=np.float64)
    if t.size == 0:
        raise ValueError(f"no samples for the {event.joint} step at t0={event.t0}")
    y0 = event.target_before_deg if y0_deg is None else y0_deg
    target = event.target_after_deg
    return StepMetrics(
        joint=event.joint,
        delta_deg=event.delta_deg,
        kp=event.kp,
        kd=event.kd,
        samples=int(t.size),
        first_motion_s=first_motion_time(t, y_deg, y0),
        settle_time_s=settle_time(t, y_deg, target),
        overshoot_pct=overshoot_pct(y_deg, y0, target),
        steady_state_error_deg=steady_state_error(t, y_deg, target),
        buzz_rms_deg_s=buzz_rms(t, dy_deg_s),
        hold_s=event.hold_s,
        recorded_s=float(t[-1]),
    )


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
        "| joint | kp | kd | step (deg) | first motion (s) | settle (s) | overshoot (%) "
        "| ss error (deg) | buzz (deg/s) | pass |"
    )
    lines = [header, "|" + "---|" * 10]
    for m in sorted(metrics, key=lambda m: (m.joint, m.kp, m.kd, -abs(m.delta_deg), m.delta_deg)):
        lines.append(
            f"| {m.joint} | {m.kp:g} | {m.kd:g} | {m.delta_deg:+.1f} "
            f"| {_fmt(m.first_motion_s)} | {_fmt(m.settle_time_s)} | {m.overshoot_pct:.1f} "
            f"| {m.steady_state_error_deg:+.3f} | {m.buzz_rms_deg_s:.2f} "
            f"| {'ok' if m.passed else 'FAIL'} |"
        )
    lines.append("")
    lines.append(
        f"Pass: settle within {SETTLE_TIME_LIMIT_S} s to {SETTLE_BAND_DEG} deg, "
        f"overshoot under {OVERSHOOT_LIMIT_PCT:g} %, |ss error| <= {SETTLE_BAND_DEG} deg, "
        f"buzz RMS under {BUZZ_RMS_LIMIT_DEG_S:g} deg/s over the last {REST_WINDOW_S} s."
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
        rejected: Why each candidate that failed, was not tested or was not
            accepted was passed over, by ``gain_label``.
    """

    joint: str
    kp: float | None
    kd: float | None
    reason: str
    rejected: dict[str, str] = field(default_factory=dict)

    @property
    def gains(self) -> tuple[float, float] | None:
        if self.kp is None or self.kd is None:
            return None
        return (self.kp, self.kd)


def propose_gains(
    step_results: Iterable[StepMetrics],
    candidates: Sequence[tuple[float, float]] | None = None,
) -> dict[str, GainProposal]:
    """Per joint, the highest kp (with its kd) among ``candidates`` that passes, monotonically.

    Candidates are ``(kp, kd)`` pairs; by default every pair with a recorded
    step. A candidate passes when it has at least one positive and one
    negative step on the joint and every one of its steps passes (whole hold
    recorded, finite metrics, settles under 0.5 s to 0.1 deg, overshoots under
    20 %, rests on target and does not buzz). It is accepted only when every
    tried candidate with a lower kp passed too, whatever its kd: the sweep
    scales kd with the square root of the kp scale, so the candidates are
    ordered by kp alone, and a lucky stiff run above a failing softer one is
    not chosen. Among accepted candidates the highest kp wins; when several
    kd values are accepted at that kp, the one with the shortest worst settle
    time wins, then the lower kd. ``rejected`` explains every candidate that
    failed, was not tried or was not accepted.
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
        # Every tried pair counts for the monotone rule, candidate or not.
        failed = {pair: why for pair, steps in by_gains.items() if (why := _gain_failures(steps))}
        rejected: list[tuple[float, float, str]] = []
        accepted: list[tuple[float, float, float]] = []
        for kp, kd in pairs:
            steps = by_gains.get((kp, kd), [])
            if not steps:
                rejected.append((kp, kd, "not tested"))
                continue
            if (kp, kd) in failed:
                rejected.append((kp, kd, failed[(kp, kd)]))
                continue
            softer = sorted(pair for pair in failed if pair[0] < kp)
            if softer:
                rejected.append(
                    (kp, kd, f"its steps pass, but {gain_label(*softer[-1])} below it failed")
                )
                continue
            worst_settle = max(step.settle_time_s or 0.0 for step in steps)
            accepted.append((kp, kd, worst_settle))
        if not accepted:
            proposals[joint] = GainProposal(
                joint,
                None,
                None,
                "no candidate passed every step",
                {gain_label(kp, kd): why for kp, kd, why in rejected},
            )
            continue
        best_kp = max(kp for kp, _, _ in accepted)
        kp, kd, settle = min(
            (entry for entry in accepted if entry[0] == best_kp), key=lambda e: (e[2], e[1])
        )
        steps = by_gains[(kp, kd)]
        reason = (
            f"{gain_label(kp, kd)}: highest kp whose {len(steps)} step(s) all pass, both ways, "
            f"with every softer kp tried passing (worst settle {settle:.2f} s, "
            f"overshoot {max(s.overshoot_pct for s in steps):.0f} %, "
            f"buzz {max(s.buzz_rms_deg_s for s in steps):.2f} deg/s)"
        )
        proposals[joint] = GainProposal(
            joint, kp, kd, reason, {gain_label(p, d): why for p, d, why in rejected}
        )
    return proposals


def _gain_failures(steps: Sequence[StepMetrics]) -> str:
    """Why one gain pair's steps on a joint do not pass; empty when they do."""
    reasons = [reason for step in steps for reason in step.failures()]
    if not any(step.delta_deg > 0.0 for step in steps):
        reasons.append("no positive step recorded")
    if not any(step.delta_deg < 0.0 for step in steps):
        reasons.append("no negative step recorded")
    return "; ".join(reasons)


def format_proposals(proposals: dict[str, GainProposal]) -> str:
    """Markdown list: the chosen gains per joint and why stiffer candidates lost."""
    if not proposals:
        return "(no steps)"
    lines: list[str] = []
    for joint in sorted(proposals):
        proposal = proposals[joint]
        lines.append(f"- {joint}: {proposal.reason}")
        for label, why in proposal.rejected.items():
            lines.append(f"  - {label} rejected: {why}")
    return "\n".join(lines)
