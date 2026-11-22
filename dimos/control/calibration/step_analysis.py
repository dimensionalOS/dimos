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

"""Step-response metrics of one joint, judged against its noise at rest and its friction, and the gains they argue for.

Plain arrays in degrees and seconds. The first-motion threshold comes from a rest window the
caller measured (``rest_noise``), widened to the encoder's step when the rest reads one value
throughout; a step buzzes when its rest moves faster (RMS) than ``BUZZ_MIN_DEG_S`` and three
times that window's peak speed. The drive has no integrator, so a joint rests short of its
target by its friction over kp: a step's rest limit is ``rest_limit_deg`` of the joint's
friction (``sweep_friction_nm``), and it settles once it stays within that limit of where it came to
rest. A step that moved less than a share of its size is stuck: its overshoot and settle are not
measured. ``propose_gains`` ranks the candidates whose steps all pass by their worst rest error
over its limit plus their worst settle over a reference time, and picks the lowest.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import math

import numpy as np

BUZZ_MIN_DEG_S = 1.0


@dataclass(frozen=True)
class StepEvent:
    """One commanded step: ``delta_deg`` from ``target_before_deg`` at ``t0``, held ``hold_s``."""

    joint: str
    delta_deg: float
    kp: float
    kd: float
    t0: float
    hold_s: float
    target_before_deg: float
    target_after_deg: float


@dataclass(frozen=True)
class RestNoise:
    """A joint at rest: the window's length, the largest sample deviation and the largest speed."""

    window_s: float
    band_deg: float
    speed_peak_deg_s: float


@dataclass(frozen=True)
class StepMetrics:
    """One step's metrics (times from the command) and why it fails, if it does.

    ``moved_deg`` is how far the joint came to rest from where it started, along the step;
    ``stuck`` when that is less than the caller's share of the step, and then
    ``settle_time_s`` and ``overshoot_pct`` are None. ``failures`` leave stuck out: whether a
    stuck step fails its gains is ``propose_gains``' call.
    """

    joint: str
    delta_deg: float
    kp: float
    kd: float
    samples: int
    band_deg: float
    first_motion_s: float | None
    moved_deg: float
    stuck: bool
    settle_time_s: float | None
    overshoot_pct: float | None
    steady_state_error_deg: float
    rest_limit_deg: float
    buzz_rms_deg_s: float
    buzzing: bool
    failures: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.failures and not self.stuck


@dataclass(frozen=True)
class GainProposal:
    """The gains ``propose_gains`` picked for one joint (None when nothing passed), and why."""

    joint: str
    kp: float | None
    kd: float | None
    reason: str
    rejected: dict[str, str] = field(default_factory=dict)


def _arrays(*values: Sequence[float] | np.ndarray) -> list[np.ndarray]:
    arrays = [np.asarray(value, dtype=np.float64) for value in values]
    if len({a.shape for a in arrays}) != 1 or arrays[0].ndim != 1 or arrays[0].size < 2:
        raise ValueError(
            f"need matching 1-D sample arrays of two or more, got {[a.shape for a in arrays]}"
        )
    if not all(np.isfinite(a).all() for a in arrays):
        raise ValueError("samples hold NaN or infinite values")
    return arrays


def _quantum(values: np.ndarray) -> float:
    """The smallest nonzero change between samples (the encoder's step), 0 when there is none."""
    steps = np.abs(np.diff(values))
    nonzero = steps[steps > 0]
    return float(nonzero.min()) if nonzero.size else 0.0


def rest_noise(
    t: Sequence[float] | np.ndarray,
    q_deg: Sequence[float] | np.ndarray,
    dq_deg_s: Sequence[float] | np.ndarray,
) -> RestNoise:
    t_, q, dq = _arrays(t, q_deg, dq_deg_s)
    return RestNoise(
        window_s=float(t_[-1] - t_[0]),
        band_deg=float(np.abs(q - q.mean()).max()),
        speed_peak_deg_s=float(np.abs(dq).max()),
    )


def rest_window(t: np.ndarray, end: float, window_s: float) -> np.ndarray:
    """Mask of the samples in the last ``window_s`` before ``end``.

    Raises ``ValueError`` when the samples stop short of ``end`` by more than their largest gap.
    """
    if t[-1] < end - float(np.diff(t).max()):
        raise ValueError(f"the samples stop {end - t[-1]:.3f} s before the hold ends")
    mask: np.ndarray = t >= end - window_s
    return mask


def rest_limit_deg(
    friction_nm: float, kp: float, *, smallest_deg: float, friction_factor: float
) -> float:
    """How far a joint may rest from its target: ``friction_factor`` times its friction over kp."""
    return max(smallest_deg, math.degrees(friction_factor * friction_nm / kp))


def analyze_step(
    event: StepEvent,
    t: Sequence[float] | np.ndarray,
    q_deg: Sequence[float] | np.ndarray,
    dq_deg_s: Sequence[float] | np.ndarray,
    noise: RestNoise,
    *,
    max_overshoot_pct: float,
    rest_limit_deg: float,
    stuck_fraction: float,
) -> StepMetrics:
    """Metrics of one step from its samples (``t`` in seconds, the clock of ``event.t0``).

    The step's rest is its last ``noise.window_s``, its start its first sample. It is stuck when
    it moved less than ``stuck_fraction`` of the step; it settles once it stays within
    ``rest_limit_deg`` of its rest (at the end of the hold when it never does). It fails when it
    overshoots ``max_overshoot_pct`` (an operator setting) or more of the step, or moves faster at
    rest (RMS) than ``BUZZ_MIN_DEG_S`` and three times the noise's peak speed. Raises
    ``ValueError`` for non-finite samples, a hold no longer than the noise window, or samples
    that stop short.
    """
    t_, q, dq = _arrays(t, q_deg, dq_deg_s)
    end = event.t0 + event.hold_s
    if noise.window_s >= event.hold_s:
        raise ValueError(
            f"the {event.hold_s:g} s hold must outlast the {noise.window_s:g} s rest window"
        )
    hold = (t_ >= event.t0) & (t_ <= end)
    t_, q, dq = _arrays(t_[hold] - event.t0, q[hold], dq[hold])
    rest = rest_window(t_, event.hold_s, noise.window_s)
    band = max(noise.band_deg, _quantum(q) / 2.0)
    moved = np.flatnonzero(np.abs(q - q[0]) > 2.0 * band)
    resting, sign = float(q[rest].mean()), math.copysign(1.0, event.delta_deg)
    travel = (resting - float(q[0])) * sign
    stuck = travel < stuck_fraction * abs(event.delta_deg)
    outside = np.flatnonzero(np.abs(q - resting) > rest_limit_deg)
    settle = float(t_[min(outside[-1] + 1, t_.size - 1)]) if outside.size else 0.0
    overshoot = max(0.0, float(((q - event.target_after_deg) * sign).max()))
    overshoot_pct = 100.0 * overshoot / abs(event.delta_deg)
    buzz = float(np.sqrt(np.mean(np.square(dq[rest]))))
    buzz_limit = max(BUZZ_MIN_DEG_S, 3.0 * noise.speed_peak_deg_s)
    step = f"{event.delta_deg:+g} deg step"
    failures = []
    if not stuck and overshoot_pct >= max_overshoot_pct:
        failures.append(f"{step} overshot {overshoot_pct:.0f} % (limit {max_overshoot_pct:g} %)")
    if buzz > buzz_limit:
        failures.append(f"{step} buzzes at {buzz:.2f} deg/s RMS (limit {buzz_limit:.2f})")
    return StepMetrics(
        joint=event.joint,
        delta_deg=event.delta_deg,
        kp=event.kp,
        kd=event.kd,
        samples=int(t_.size),
        band_deg=band,
        first_motion_s=float(t_[moved[0]]) if moved.size else None,
        moved_deg=travel,
        stuck=stuck,
        settle_time_s=None if stuck else settle,
        overshoot_pct=None if stuck else overshoot_pct,
        steady_state_error_deg=resting - event.target_after_deg,
        rest_limit_deg=rest_limit_deg,
        buzz_rms_deg_s=buzz,
        buzzing=buzz > buzz_limit,
        failures=tuple(failures),
    )


def sweep_friction_nm(steps: Iterable[StepMetrics]) -> dict[str, float]:
    """Per joint, its friction from its steps at one gain pair (its baseline): kp times the
    larger of half the rest band between its largest steps either way and its largest rest
    error of a step that moved."""
    by_joint: dict[str, list[StepMetrics]] = defaultdict(list)
    for step in steps:
        by_joint[step.joint].append(step)
    friction: dict[str, float] = {}
    for joint, joint_steps in by_joint.items():
        offsets = [abs(s.steady_state_error_deg) for s in joint_steps if not s.stuck]
        up = [s for s in joint_steps if s.delta_deg > 0.0]
        down = [s for s in joint_steps if s.delta_deg < 0.0]
        if up and down:
            largest_up = max(up, key=lambda s: s.delta_deg)
            largest_down = min(down, key=lambda s: s.delta_deg)
            offsets.append(
                (largest_down.steady_state_error_deg - largest_up.steady_state_error_deg) / 2.0
            )
        if offsets:
            friction[joint] = joint_steps[0].kp * math.radians(max(0.0, *offsets))
    return friction


def _gain_failures(steps: Sequence[StepMetrics]) -> str:
    """Why one gain pair's steps fail; a stuck step smaller than its largest does not count."""
    largest = max(abs(step.delta_deg) for step in steps)
    reasons = [reason for step in steps for reason in step.failures]
    reasons += [
        f"{step.delta_deg:+g} deg step is stuck: moved {step.moved_deg:+.2f} deg"
        for step in steps
        if step.stuck and abs(step.delta_deg) >= largest
    ]
    if not any(step.delta_deg > 0 for step in steps):
        reasons.append("no positive step")
    if not any(step.delta_deg < 0 for step in steps):
        reasons.append("no negative step")
    return "; ".join(reasons)


def _score(steps: Sequence[StepMetrics], settle_s: float) -> float:
    moved = [step for step in steps if not step.stuck]
    rest = max(abs(step.steady_state_error_deg) / step.rest_limit_deg for step in moved)
    return rest + max(step.settle_time_s or 0.0 for step in moved) / settle_s


def _stuck_note(steps: Sequence[StepMetrics]) -> str:
    stuck = [f"{step.delta_deg:+g}" for step in steps if step.stuck]
    return f"; its {', '.join(stuck)} deg steps stuck and do not count" if stuck else ""


def propose_gains(
    step_results: Iterable[StepMetrics],
    candidates: Sequence[tuple[float, float]] | None = None,
    stopped: Mapping[str, Mapping[tuple[float, float], str]] | None = None,
    *,
    settle_s: float,
    kd_limit: float,
    max_overshoot_pct: float,
) -> dict[str, GainProposal]:
    """Per joint, the passing ``(kp, kd)`` among ``candidates`` (default: every one tried) with
    the lowest score.

    A pair passes when it has a positive and a negative step on the joint and none fails or,
    among its largest, is stuck (a stuck smaller step is listed and left out). Its score is its
    worst rest error over its rest limit plus its worst settle over ``settle_s``; a tie goes to
    the higher kp, then the lower kd. A pair in ``stopped`` (by joint) is rejected with the
    reason given there. When none passes and every pair tried at ``kd_limit``, the driver's,
    overshoots ``max_overshoot_pct``, the reason says to lower kp.
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
        halted = (stopped or {}).get(joint, {})
        rejected: dict[str, str] = {}
        scored: list[tuple[float, float, float]] = []
        for kp, kd in pairs:
            label, steps = f"kp={kp:g} kd={kd:g}", by_gains.get((kp, kd), [])
            if (kp, kd) in halted:
                rejected[label] = halted[(kp, kd)]
            elif not steps:
                rejected[label] = "not tested"
            elif why := _gain_failures(steps):
                rejected[label] = why
            else:
                scored.append((_score(steps, settle_s), kp, kd))
        if not scored:
            reason = "no candidate passed every step"
            at_limit = [steps for (_, kd), steps in by_gains.items() if kd >= kd_limit]
            if at_limit and all(
                any(
                    s.overshoot_pct is not None and s.overshoot_pct >= max_overshoot_pct
                    for s in steps
                )
                for steps in at_limit
            ):
                reason += f"; it overshoots with kd at the driver limit {kd_limit:g}; lower kp"
            proposals[joint] = GainProposal(joint, None, None, reason, rejected)
            continue
        score, kp, kd = min(scored, key=lambda entry: (entry[0], -entry[1], entry[2]))
        for other, other_kp, other_kd in scored:
            if (other_kp, other_kd) != (kp, kd):
                rejected[f"kp={other_kp:g} kd={other_kd:g}"] = (
                    f"passes, but scores {other:.2f} against {score:.2f}"
                    f"{_stuck_note(by_gains[(other_kp, other_kd)])}"
                )
        steps = by_gains[(kp, kd)]
        moved = [step for step in steps if not step.stuck]
        reason = (
            f"kp={kp:g} kd={kd:g}: the lowest score {score:.2f} of {len(scored)} passing: rests "
            f"up to {max(abs(s.steady_state_error_deg) for s in moved):.2f} deg off target "
            f"(limit {moved[0].rest_limit_deg:.2f}), settles in "
            f"{max(s.settle_time_s or 0.0 for s in moved):.2f} s, overshoot "
            f"{max(s.overshoot_pct or 0.0 for s in moved):.0f} %{_stuck_note(steps)}"
        )
        proposals[joint] = GainProposal(joint, kp, kd, reason, rejected)
    return proposals
