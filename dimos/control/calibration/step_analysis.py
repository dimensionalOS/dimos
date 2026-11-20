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

"""Step-response metrics of one joint, judged against its own noise at rest, and the gains they argue for.

Plain arrays in degrees and seconds. The settle band, the first-motion threshold and the buzz
limit come from a rest window the caller measured (``rest_noise``), widened to the encoder's
step when the rest reads one value throughout. A step settles when it stays inside the band
around where it came to rest; given the joint's inertia it must do so no slower than an ideal
spring-damper with its kp and kd would.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.special import lambertw


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
    """One step's metrics (times from the command) and why it fails, if it does."""

    joint: str
    delta_deg: float
    kp: float
    kd: float
    samples: int
    band_deg: float
    first_motion_s: float | None
    settle_time_s: float
    settle_limit_s: float | None
    overshoot_pct: float
    steady_state_error_deg: float
    buzz_rms_deg_s: float
    failures: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.failures


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


def settle_time_limit_s(
    inertia_kgm2: float, kp: float, kd: float, step_deg: float, band_deg: float
) -> float:
    """When an ideal spring-damper (inertia, kp, kd) is sure to be within ``band_deg`` of a step's end.

    The decay envelope of its error: ``A e^(-zeta wn t) / sqrt(1 - zeta^2)`` when underdamped,
    ``A (1 + p t) e^(-p t)`` with ``p`` its slower pole otherwise.
    """
    ratio = band_deg / abs(step_deg)
    if ratio >= 1.0:
        return 0.0
    wn = math.sqrt(kp / inertia_kgm2)
    zeta = kd / (2.0 * math.sqrt(kp * inertia_kgm2))
    if ratio == 0.0 or zeta == 0.0:
        return math.inf
    if zeta < 1.0:
        return max(0.0, -math.log(ratio * math.sqrt(1.0 - zeta**2)) / (zeta * wn))
    pole = wn * (zeta - math.sqrt(zeta**2 - 1.0))
    return float(-1.0 - lambertw(-ratio / math.e, -1).real) / pole


def analyze_step(
    event: StepEvent,
    t: Sequence[float] | np.ndarray,
    q_deg: Sequence[float] | np.ndarray,
    dq_deg_s: Sequence[float] | np.ndarray,
    noise: RestNoise,
    *,
    max_overshoot_pct: float,
    inertia_kgm2: float | None,
) -> StepMetrics:
    """Metrics of one step from its samples (``t`` in seconds, the clock of ``event.t0``).

    The step's rest is its last ``noise.window_s``. It fails when the joint never moved past
    the band, settles slower than ``settle_time_limit_s`` (only with ``inertia_kgm2``),
    overshoots ``max_overshoot_pct`` (an operator setting) or more of the step, or moves
    faster at rest (RMS) than at the rest the noise came from. Raises ``ValueError`` for
    non-finite samples, a hold no longer than the noise window, or samples that stop short.
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
    outside = np.flatnonzero((q < q[rest].min() - band) | (q > q[rest].max() + band))
    settle = float(t_[outside[-1] + 1]) if outside.size else 0.0
    moved = np.flatnonzero(np.abs(q - q[0]) > 2.0 * band)
    overshoot = max(
        0.0, float(((q - event.target_after_deg) * math.copysign(1.0, event.delta_deg)).max())
    )
    overshoot_pct = 100.0 * overshoot / abs(event.delta_deg)
    buzz = float(np.sqrt(np.mean(np.square(dq[rest]))))
    buzz_limit = max(noise.speed_peak_deg_s, _quantum(dq))
    limit = (
        None
        if inertia_kgm2 is None
        else settle_time_limit_s(inertia_kgm2, event.kp, event.kd, event.delta_deg, band)
    )
    step = f"{event.delta_deg:+g} deg step"
    failures = []
    if not moved.size:
        failures.append(f"{step} never moved past twice its {band:.3f} deg noise band")
    if limit is not None and settle > limit:
        failures.append(
            f"{step} settled in {settle:.2f} s; an ideal spring-damper takes {limit:.2f} s"
        )
    if overshoot_pct >= max_overshoot_pct:
        failures.append(f"{step} overshot {overshoot_pct:.0f} % (limit {max_overshoot_pct:g} %)")
    if buzz > buzz_limit:
        failures.append(
            f"{step} buzzes at {buzz:.2f} deg/s RMS; at rest it peaked at {buzz_limit:.2f}"
        )
    return StepMetrics(
        joint=event.joint,
        delta_deg=event.delta_deg,
        kp=event.kp,
        kd=event.kd,
        samples=int(t_.size),
        band_deg=band,
        first_motion_s=float(t_[moved[0]]) if moved.size else None,
        settle_time_s=settle,
        settle_limit_s=limit,
        overshoot_pct=overshoot_pct,
        steady_state_error_deg=float(q[rest].mean() - event.target_after_deg),
        buzz_rms_deg_s=buzz,
        failures=tuple(failures),
    )


def propose_gains(
    step_results: Iterable[StepMetrics], candidates: Sequence[tuple[float, float]] | None = None
) -> dict[str, GainProposal]:
    """Per joint, the stiffest ``(kp, kd)`` among ``candidates`` (default: every one tried) that passes.

    A pair passes when it has a positive and a negative step on the joint and they all pass.
    It is accepted only when every tried pair with a lower kp (whatever its kd) passed too, so a
    lucky stiff run above a failing softer one is not chosen. At the highest accepted kp the
    shortest worst settle wins, then the lower kd.
    """
    by_joint: dict[str, dict[tuple[float, float], list[StepMetrics]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for metrics in step_results:
        by_joint[metrics.joint][(metrics.kp, metrics.kd)].append(metrics)
    proposals: dict[str, GainProposal] = {}
    for joint, by_gains in by_joint.items():
        failed: dict[tuple[float, float], str] = {}
        for pair, steps in by_gains.items():
            reasons = [reason for step in steps for reason in step.failures]
            if not any(step.delta_deg > 0 for step in steps):
                reasons.append("no positive step")
            if not any(step.delta_deg < 0 for step in steps):
                reasons.append("no negative step")
            if reasons:
                failed[pair] = "; ".join(reasons)
        pairs = sorted(
            set(by_gains) if candidates is None else {(float(p), float(d)) for p, d in candidates}
        )
        rejected: dict[str, str] = {}
        accepted: list[tuple[float, float, float]] = []
        for kp, kd in pairs:
            label = f"kp={kp:g} kd={kd:g}"
            softer = sorted(pair for pair in failed if pair[0] < kp)
            if (kp, kd) not in by_gains:
                rejected[label] = "not tested"
            elif (kp, kd) in failed:
                rejected[label] = failed[(kp, kd)]
            elif softer:
                rejected[label] = (
                    f"its steps pass, but kp={softer[-1][0]:g} kd={softer[-1][1]:g} below it failed"
                )
            else:
                accepted.append((kp, kd, max(step.settle_time_s for step in by_gains[(kp, kd)])))
        if not accepted:
            proposals[joint] = GainProposal(
                joint, None, None, "no candidate passed every step", rejected
            )
            continue
        best_kp = max(kp for kp, _, _ in accepted)
        kp, kd, settle = min((a for a in accepted if a[0] == best_kp), key=lambda a: (a[2], a[1]))
        steps = by_gains[(kp, kd)]
        reason = (
            f"kp={kp:g} kd={kd:g}: the stiffest whose {len(steps)} steps all pass both ways, every "
            f"softer kp passing too (worst settle {settle:.2f} s, overshoot "
            f"{max(s.overshoot_pct for s in steps):.0f} %)"
        )
        proposals[joint] = GainProposal(joint, kp, kd, reason, rejected)
    return proposals
