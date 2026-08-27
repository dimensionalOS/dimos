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

"""Fit the plant: pins with provenance, seeded restarts, a median and a spread.

    python -m dimos.robot.unitree.go2.sim.sysid.fit REC.mcap --workers 20

ONE RUN IS NOT A RESULT. Four studies differing only in seed agree on the
loss to within ~3 points and disagree on the parameters by up to 8.8x — the
knobs trade against each other, so the data locates a REGION and cannot locate
a point in it. So the fit restarts, harvests every near-optimal trial as a
free sample OF the region, ships the per-parameter MEDIAN of the pooled cloud
(never the best draw — picking it spends the held-out set to make the choice)
and the 10th-90th percentile spread, and stops when the SPREAD is stable under
leave-one-study-out, not when a counter runs out. Hitting the study cap is a
RESULT to report — the region is wider than the data can pin — not a failure.

A knob is either PINNED with provenance or SEARCHED within its declared range;
:mod:`~dimos.robot.unitree.go2.sim.anchors` supplies pins derived from
``robot.json``. Pinning does not delete a range — it stops the search using
it. The objective — segments, clip schedules, weights, scales — is decided
ONCE, before any search, and shared by every candidate and every study: an
objective that moved between candidates would be compared as much as the
physics (changing only which window was scored once swung a headline 10x).

Parallelism: segments fan out across worker processes
(:mod:`~dimos.robot.unitree.go2.sim.sysid.rollouts`), studies run in batches
sized to the core budget, and trials within a study stay SEQUENTIAL — CMA-ES
learns from its history, and parallel trials would stop a study being
reproducible from its seed.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import concurrent.futures
from dataclasses import dataclass
import itertools
import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.unitree.go2.sim.anchors import RobotSpec, derive
from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
from dimos.robot.unitree.go2.sim.ranges import ENGINE_DEFAULTS, Knob, Preset, load_preset
from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts, RolloutSpec
from dimos.robot.unitree.go2.sim.sysid.score import (
    DEFAULT_WEIGHTS,
    SUSPENDED_WEIGHTS,
    Score,
    SegmentTerms,
    WeightVector,
    scales_from,
    score_terms,
    segment_terms,
)
from dimos.simulation.sysid.recording import read_declarations
from dimos.simulation.sysid.regimes import (
    Segment,
    Span,
    protected,
    regimes,
    sample_segments,
)

# Natural units per channel, for the judgement tables. `rot` entries move
# ~1:1 with radians (score.predicted).
CHANNEL_UNITS = {
    "joint": "rad",
    "dq": "rad/s",
    "tau": "N*m",
    "accel": "m/s^2",
    "gyro": "rad/s",
    "pos": "m",
    "rot": "rad",
}

# ------------------------------------------------------------------ knob plan


@dataclass(frozen=True)
class Pin:
    """A knob held at a known value. A pin is a claim like any other, so it
    carries its provenance the same way a searched range does."""

    value: float
    why: str


@dataclass(frozen=True)
class KnobPlan:
    """Every knob's disposition: pinned with provenance, or searched in range.

    The two sets are disjoint by construction. A knob in neither keeps
    whatever the base preset (or the engine default) says — visible in the
    report as neither measured nor searched.
    """

    pinned: dict[str, Pin]
    searched: dict[str, Knob]

    def __post_init__(self) -> None:
        both = set(self.pinned) & set(self.searched)
        if both:
            raise ValueError(f"knob(s) both pinned and searched: {sorted(both)}")


# The default search set: the knobs the 2026-08-16 fits searched — the ones
# the identifiability spectrum resolves. Everything else is anchored by a
# measurement (see default_plan).
DEFAULT_SEARCH: tuple[str, ...] = (
    "armature",
    "actuator_tau",
    "frictionloss",
    "leg_mass_scale",
    "foot_solref_time",
    "foot_solref_damp",
    "foot_solimp_width",
)


def default_plan(
    backend_knobs: Mapping[str, Knob],
    robot: RobotSpec | None = None,
    *,
    floor_mu: float = 0.9,
    search: Sequence[str] | None = None,
) -> KnobPlan:
    """The anchoring discipline as code: derive pins from the robot spec,
    search the rest of the measured-informative set.

    ``damping`` is pinned because only the suspended regime resolves it (330x
    the information of walking) — fit it there, on the joint channel, never
    here. ``foot_friction`` is a floor property, not a robot one.
    """
    spec = robot or RobotSpec(mass_kg=16.500)
    derived = derive(spec, floor_mu=floor_mu)
    why = {
        "trunk_mass_scale": f"(trunk + weighed surplus)/trunk at {spec.mass_kg} kg",
        "trunk_com_x": "parallel-axis consequence of the weighed payload",
        "trunk_inertia_scale": "parallel-axis interpolation at the weighed mass",
        "foot_friction_torsional": f"(3pi/16)*mu*a for the {spec.foot.radius_m * 1e3:.0f} mm foot",
    }
    pinned = {k: Pin(v, why[k]) for k, v in derived.items()}
    pinned["damping"] = Pin(
        0.03808, "only the suspended regime resolves it; fitted there on the joint channel"
    )
    pinned["foot_friction"] = Pin(
        floor_mu, "a floor property, not a robot one; rubber pads grip both measured surfaces"
    )
    names = tuple(search) if search is not None else DEFAULT_SEARCH
    missing = [n for n in names if n not in backend_knobs]
    if missing:
        raise KeyError(f"backend exposes no knob(s) {missing}")
    searched = {n: backend_knobs[n] for n in names}
    for n in searched:
        pinned.pop(n, None)  # an explicit search wins over a default pin
    return KnobPlan(pinned=pinned, searched=searched)


def load_knob_plan(path: str | Path, backend_knobs: Mapping[str, Knob]) -> KnobPlan:
    """A knobs.json: ``{"pin": v, "why": ...}`` or ``{"search": [lo, hi] | true}``.

    ``"search": true`` uses the backend's declared range; an explicit
    ``[lo, hi]`` (with optional ``"log"``) narrows or widens it, with the
    reason recorded in ``"why"`` like any other range.
    """
    d = json.loads(Path(path).read_text())
    pinned: dict[str, Pin] = {}
    searched: dict[str, Knob] = {}
    for name, entry in d.items():
        if name not in backend_knobs:
            raise KeyError(f"knobs.json names {name!r}, which this backend does not expose")
        if "pin" in entry:
            pinned[name] = Pin(float(entry["pin"]), str(entry.get("why", "knobs.json, no why")))
        elif "search" in entry:
            base = backend_knobs[name]
            if entry["search"] is True:
                searched[name] = base
            else:
                lo, hi = entry["search"]
                searched[name] = Knob(
                    float(lo),
                    float(hi),
                    log=bool(entry.get("log", base.log)),
                    unit=base.unit,
                    why=str(entry.get("why", "knobs.json, no why")),
                )
        else:
            raise ValueError(f"knobs.json entry {name!r} needs 'pin' or 'search'")
    return KnobPlan(pinned=pinned, searched=searched)


def base_values(preset: str = "measured") -> dict[str, float]:
    """The incumbent's complete knob values — the search's start point.

    Early presets predate the contact and solver keys, so the engine defaults
    fill in: this changes no physics, it just gives every knob a start value
    — and the preset a fit SAVES then carries them explicitly.
    """
    p = load_preset(preset)
    return {**ENGINE_DEFAULTS, **p.physics, "actuator_tau": p.actuator_tau}


def merged(
    base: Mapping[str, float], plan: KnobPlan, params: Mapping[str, float]
) -> dict[str, float]:
    """One candidate's complete values: base, then pins, then the trial's draws."""
    return {**base, **{k: p.value for k, p in plan.pinned.items()}, **params}


# ------------------------------------------------------------------ objective


class Objective:
    """The fixed objective every candidate and every study sees.

    Segments, per-segment clip schedules, regime spans, weights and scales are
    all decided here, before any search — the scales by :meth:`calibrate`
    against the baseline plant, then FROZEN. After calibration this object is
    immutable and thread-safe: studies in flight share it.

    Terms are computed for EVERY backend channel, not only the weighted
    ones (zero weight never means unreported — README 4); the extras cost
    array arithmetic, not physics. Normalisation is always on: raw
    residual summation is a unit choice, and its toggle retired with the
    frozen scorer.
    """

    def __init__(
        self,
        rollouts: Rollouts,
        *,
        segments: Sequence[Segment],
        spans: Sequence[Span],
        weights: WeightVector,
        backend_channels: frozenset[str],
        window: float | tuple[float, float] | None = (0.05, 0.8),
        schedule_seed: int = 0,
        suspended: bool = False,
    ) -> None:
        self.rollouts = rollouts
        self.segments = list(segments)
        self.spans = list(spans)
        self.weights = dict(weights)
        self.window = window
        self.schedule_seed = schedule_seed
        self.suspended = suspended
        self._protect = protected(list(spans))
        self.channels = tuple(sorted(backend_channels))
        wanted = {c for (c, _r), w in self.weights.items() if w > 0}
        dropped = sorted(wanted - backend_channels)
        if dropped:
            print(f"note: backend predicts no {dropped}; those weights are dead here")
        self.scales: dict[str, float] | None = None

    def _specs(self, values: Mapping[str, float]) -> list[RolloutSpec]:
        return [
            RolloutSpec(
                values=dict(values),
                t0=seg.t0,
                duration=seg.duration,
                window=self.window,
                seed=self.schedule_seed + i,  # per-segment, shared across candidates
                suspended=self.suspended,
                protect=self._protect,
            )
            for i, seg in enumerate(self.segments)
        ]

    def terms(self, values: Mapping[str, float]) -> list[SegmentTerms]:
        """Every segment's per-(channel, regime) terms under ``values``."""
        results = self.rollouts.run(self._specs(values))
        return [segment_terms(r, self.spans, self.channels) for r in results]

    def calibrate(self, baseline: Mapping[str, float]) -> Score:
        """Set the scales from the baseline plant's residuals, then freeze them."""
        terms = self.terms(baseline)
        self.scales = scales_from(terms)
        return score_terms(terms, self.weights, self.scales)

    def evaluate(self, values: Mapping[str, float]) -> Score:
        if self.scales is None:
            raise RuntimeError("calibrate() the objective on the baseline before evaluating")
        return score_terms(self.terms(values), self.weights, self.scales)


class PooledObjective:
    """Several recordings, ONE objective: terms pool, scales are shared.

    How a flight-bearing recording joins the floor one (README 4). Scales
    come from the pooled baseline residuals so a channel means one thing
    across recordings; the total is the mean over all parts' segments;
    weights must be identical across parts.
    """

    def __init__(self, parts: Sequence[Objective]) -> None:
        if not parts:
            raise ValueError("PooledObjective needs at least one part")
        self.parts = list(parts)
        self.weights = dict(parts[0].weights)
        for p in parts[1:]:
            if dict(p.weights) != self.weights:
                raise ValueError("every pooled part must share one weight vector")
        self.scales: dict[str, float] | None = None

    def terms(self, values: Mapping[str, float]) -> list[SegmentTerms]:
        return [t for p in self.parts for t in p.terms(values)]

    def calibrate(self, baseline: Mapping[str, float]) -> Score:
        terms = self.terms(baseline)
        self.scales = scales_from(terms)
        return score_terms(terms, self.weights, self.scales)

    def evaluate(self, values: Mapping[str, float]) -> Score:
        if self.scales is None:
            raise RuntimeError("calibrate() the objective on the baseline before evaluating")
        return score_terms(self.terms(values), self.weights, self.scales)


# ------------------------------------------------------------------- studies


@dataclass
class StudyOutcome:
    """One seeded study: its best draw (diagnostic only) and its harvest."""

    seed: int
    n_trials: int
    best_total: float
    best_params: dict[str, float]
    tol: float  # harvest tolerance: 1 SE of the loss across scored segments
    harvested: dict[str, np.ndarray]  # per searched knob, near-optimal draws
    n_harvested: int


def run_study(
    objective: Objective | PooledObjective,
    plan: KnobPlan,
    base: Mapping[str, float],
    *,
    seed: int,
    trials: int = 90,
) -> StudyOutcome:
    """One CMA-ES study, fully reproducible from its seed. Trials SEQUENTIAL.

    Harvests every trial statistically indistinguishable from the best — a
    study makes ~90 trials and keeps 1; the near-optimal ones are free samples
    OF the region. The tolerance is one standard error of the loss's own
    sampling noise, measured PAIRED: every candidate scores the identical
    segments, so segment difficulty is common mode and the distinguishable
    quantity is the per-segment DIFFERENCE from the best trial — its SE across
    segments is the resolution at which two totals differ at all. (The
    unpaired SE is ~16% of the loss and harvests nearly every trial, i.e. the
    search's whole path rather than the region; the principled ceiling — the
    loss's noise floor from a repeat recording — does not exist yet.)
    """
    import optuna

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    start = {k: float(base[k]) for k in plan.searched}
    records: list[tuple[float, dict[str, float], tuple[float, ...]]] = []

    def objective_fn(trial: optuna.Trial) -> float:
        params = {
            name: trial.suggest_float(name, k.lo, k.hi, log=k.log)
            for name, k in plan.searched.items()
        }
        s = objective.evaluate(merged(base, plan, params))
        records.append((s.total, params, s.per_segment))
        return s.total

    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.CmaEsSampler(
            seed=seed,
            # The fallback sampler must be seeded too, or the one trial it
            # draws makes the study irreproducible — silently.
            independent_sampler=optuna.samplers.RandomSampler(seed=seed),
        ),
    )
    # Seed trial 0 with the incumbent: a search that cannot beat what we have
    # should say so, not rediscover it and claim an improvement.
    study.enqueue_trial(start)
    study.optimize(objective_fn, n_trials=trials)

    best_total, best_params, best_segs = min(records, key=lambda r: r[0])
    keep: list[dict[str, float]] = []
    tols: list[float] = []
    for total, p, segs in records:
        d = np.array(segs) - np.array(best_segs)
        n = len(d)
        tol_i = float(np.std(d, ddof=1) / math.sqrt(n)) if n > 1 else 0.0
        tols.append(tol_i)
        if total - best_total <= tol_i:  # mean(d) == total - best_total exactly
            keep.append(p)
    return StudyOutcome(
        seed=seed,
        n_trials=len(records),
        best_total=best_total,
        best_params=dict(best_params),
        tol=float(np.median(tols)),  # the study's typical 1-SE resolution
        harvested={k: np.array([p[k] for p in keep]) for k in plan.searched},
        n_harvested=len(keep),
    )


# ------------------------------------------------- pooling, spread, stopping


def _positions(knob: Knob, v: np.ndarray) -> np.ndarray:
    """Vectorised :meth:`Knob.position` — quantile drift is judged in the
    knob's OWN metric, for the same reason bounds are."""
    out: np.ndarray
    if knob.log:
        out = (np.log(v) - math.log(knob.lo)) / (math.log(knob.hi) - math.log(knob.lo))
    else:
        out = (v - knob.lo) / (knob.hi - knob.lo)
    return out


def pooled_cloud(outcomes: Sequence[StudyOutcome], names: Sequence[str]) -> dict[str, np.ndarray]:
    return {n: np.concatenate([o.harvested[n] for o in outcomes]) for n in names}


def point_and_spread(
    cloud: Mapping[str, np.ndarray],
) -> tuple[dict[str, float], dict[str, tuple[float, float]]]:
    """Per-parameter MEDIAN of the pooled cloud, and its 10th-90th percentiles.

    NEVER the best draw: quantiles are order statistics, so they are the same
    in linear and log space, and the median of a region every point of which
    scores about the same is the honest centre.
    """
    point = {n: float(np.median(v)) for n, v in cloud.items()}
    spread = {
        n: (float(np.percentile(v, 10)), float(np.percentile(v, 90))) for n, v in cloud.items()
    }
    return point, spread


def loo_drift(outcomes: Sequence[StudyOutcome], searched: Mapping[str, Knob]) -> float:
    """Leave-one-study-out stability of the SPREAD, in each knob's own metric.

    The max over left-out studies and searched knobs of |quantile change| /
    the full pool's spread, for both the 10th and 90th percentiles. The
    median's precision does not matter — every point in the region scores
    about the same, which is the finding; the spread is what training
    consumes, so the spread is what must converge.
    """
    drift = 0.0
    for name, knob in searched.items():
        full = _positions(knob, pooled_cloud(outcomes, [name])[name])
        q10, q90 = np.percentile(full, 10), np.percentile(full, 90)
        spread = q90 - q10
        for j in range(len(outcomes)):
            rest = [o for i, o in enumerate(outcomes) if i != j]
            loo = _positions(knob, pooled_cloud(rest, [name])[name])
            for q, ref in ((10, q10), (90, q90)):
                change = abs(float(np.percentile(loo, q)) - float(ref))
                if spread > 0:
                    drift = max(drift, change / spread)
                elif change > 0:
                    return float("inf")
    return drift


# ------------------------------------------------------------------ the fit


@dataclass
class FitResult:
    plan: KnobPlan
    point: dict[str, float]
    spread: dict[str, tuple[float, float]]
    cloud: dict[str, np.ndarray]
    studies: list[StudyOutcome]
    drift_trace: list[tuple[int, float]]  # (k studies, drift) after each batch
    stopped: str  # "stable" | "cap" | "diagnostic"
    baseline: Score
    point_score: Score

    def unresolved(self) -> list[str]:
        """Knobs whose spread covers most of their range: nothing pinned them."""
        out = []
        for n, (lo, hi) in self.spread.items():
            k = self.plan.searched[n]
            if k.position(hi) - k.position(lo) > 0.8:
                out.append(n)
        return out


def fit(
    objective: Objective | PooledObjective,
    plan: KnobPlan,
    base: Mapping[str, float],
    *,
    trials: int = 90,
    min_studies: int = 3,
    max_studies: int = 12,
    drift_tol: float = 0.10,
    batch: int = 1,
    on_study: Any = None,  # callable(StudyOutcome), progress reporting
) -> FitResult:
    """Restarts until the spread is stable under leave-one-study-out.

    Studies run in BATCHES sized to the core budget (they are independent by
    design), the rule is evaluated between batches, and overshooting is
    accepted — extra samples of the region are what the spread estimate wants
    anyway. Below 3 studies there is no leave-one-out and the result is
    explicitly diagnostic.
    """
    baseline = objective.calibrate(merged(base, plan, {}))
    outcomes: list[StudyOutcome] = []
    drift_trace: list[tuple[int, float]] = []
    consecutive = 0
    stopped = "diagnostic"
    while len(outcomes) < max_studies:
        n_new = (min_studies if not outcomes else batch) if min_studies > 0 else batch
        n_new = min(max(n_new, 1), max_studies - len(outcomes))
        seeds = [len(outcomes) + i for i in range(n_new)]
        with concurrent.futures.ThreadPoolExecutor(max_workers=n_new) as tp:
            batch_out = list(
                tp.map(
                    lambda s: run_study(objective, plan, base, seed=s, trials=trials),
                    seeds,
                )
            )
        for o in batch_out:
            outcomes.append(o)
            if on_study is not None:
                on_study(o)
        if len(outcomes) < 3:
            continue
        d = loo_drift(outcomes, plan.searched)
        drift_trace.append((len(outcomes), d))
        consecutive = consecutive + 1 if d < drift_tol else 0
        if consecutive >= 2:
            stopped = "stable"
            break
    else:
        stopped = "cap" if len(outcomes) >= 3 else "diagnostic"
    cloud = pooled_cloud(outcomes, list(plan.searched))
    point, spread = point_and_spread(cloud)
    point_score = objective.evaluate(merged(base, plan, point))
    return FitResult(
        plan=plan,
        point=point,
        spread=spread,
        cloud=cloud,
        studies=outcomes,
        drift_trace=drift_trace,
        stopped=stopped,
        baseline=baseline,
        point_score=point_score,
    )


# ------------------------------------------------------------------- report


def _pct(now: float, was: float) -> str:
    return f"{100.0 * (now - was) / was:+.1f}%"


def channel_table(
    baseline: Score,
    candidate: Score | None,
    weights: WeightVector,
    scales: Mapping[str, float] | None,
) -> list[str]:
    """Every computed (channel, regime) residual, scored or not — zero
    weight never means unreported (README 4: the de-weighted rows ARE the
    misspecification map). ``share`` approximates each scored row's part
    of the weighted total."""
    who = candidate or baseline
    keys = sorted(set(baseline.terms) | set(who.terms))
    scored = {k: w for k, w in weights.items() if w > 0}
    denom = sum(
        w * who.terms.get(k, 0.0) / (scales or {}).get(k[0], 1.0)
        for k, w in scored.items()
        if k in who.terms and (scales or {}).get(k[0], 0.0) > 0
    )
    out = [
        "CHANNELS  every residual the data can answer, scored or not — the",
        "zero-weight rows are the misspecification map, not omissions",
        f"  {'channel/regime':<20s} {'weight':>7s} {'baseline':>10s} {'-> value':>10s} "
        f"{'unit':<7s} {'/scale':>7s} {'share':>6s}  scored?",
    ]
    for key in keys:
        c, r = key
        w = weights.get(key, 0.0)
        b = baseline.terms.get(key)
        v = who.terms.get(key)
        scale = (scales or {}).get(c)
        vs = f"{v:10.4f}" if v is not None else f"{'—':>10s}"
        bs = f"{b:10.4f}" if b is not None else f"{'—':>10s}"
        rel = f"{v / scale:7.2f}" if v is not None and scale else f"{'—':>7s}"
        share = (
            f"{100 * w * (v / scale) / denom:5.1f}%"
            if w > 0 and v is not None and scale and denom > 0
            else f"{'—':>6s}"
        )
        tag = "yes" if w > 0 else "SHOWN, not scored"
        out.append(
            f"  {c + '/' + r:<20s} {w:7.2f} {bs} {vs} {CHANNEL_UNITS.get(c, ''):<7s}"
            f" {rel} {share}  {tag}"
        )
    return out


def format_report(
    res: FitResult,
    *,
    header: str = "",
    weights: WeightVector | None = None,
    scales: Mapping[str, float] | None = None,
) -> str:
    out: list[str] = []
    if header:
        out += [header, ""]
    out.append("PINNED (not searched — provenance, not optimism)")
    for n, p in sorted(res.plan.pinned.items()):
        out.append(f"  {n:<26s} {p.value:>10.5f}   {p.why}")
    out.append("")
    out.append("SEARCHED  within the knob's declared range, in its own metric")
    for n, k in res.plan.searched.items():
        scale = "log" if k.log else "lin"
        out.append(f"  {n:<26s} [{k.lo:g}, {k.hi:g}] {scale}   {k.why}")
    out.append("")
    n_pool = len(next(iter(res.cloud.values()))) if res.cloud else 0
    out.append(
        f"POINT AND SPREAD  median and 10th-90th of {n_pool} pooled near-optimal "
        f"trials from {len(res.studies)} studies — never the best draw"
    )
    out.append(f"  {'knob':<26s} {'point':>10s}  {'p10':>10s} .. {'p90':<10s} {'of range':>9s}")
    unresolved = set(res.unresolved())
    for n in res.plan.searched:
        lo, hi = res.spread[n]
        k = res.plan.searched[n]
        frac = k.position(hi) - k.position(lo)
        tag = "  UNRESOLVED: the data does not pin this" if n in unresolved else ""
        out.append(
            f"  {n:<26s} {res.point[n]:>10.5f}  {lo:>10.5f} .. {hi:<10.5f} {100 * frac:>8.1f}%{tag}"
        )
    out.append("")
    out.append(
        "STUDIES  harvest = trials within 1 paired SE of the study's best; "
        "per-seed bests are DIAGNOSTIC — the shipped point is the median"
    )
    out.append(
        f"  {'seed':>4s} {'trials':>6s} {'best':>10s} {'vs base':>8s} {'harvest':>7s} {'tol':>9s}"
    )
    for o in res.studies:
        out.append(
            f"  {o.seed:>4d} {o.n_trials:>6d} {o.best_total:>10.5f} "
            f"{_pct(o.best_total, res.baseline.total):>8s} {o.n_harvested:>7d} {o.tol:>9.5f}"
        )
    if len(res.studies) > 1:
        out.append("")
        out.append("  seed sensitivity of the best draw (the reason the median ships):")
        for n in res.plan.searched:
            vals = [o.best_params[n] for o in res.studies]
            ratio = max(vals) / min(vals) if min(vals) > 0 else float("inf")
            out.append(
                f"    {n:<24s} {min(vals):>10.5f} .. {max(vals):<10.5f} ({ratio:.1f}x across seeds)"
            )
    out.append("")
    trace = "  ".join(f"k={k}: {d:.3f}" for k, d in res.drift_trace)
    out.append("STOPPING  leave-one-study-out spread drift (below tolerance twice in a row stops)")
    out.append(f"  {trace or '(fewer than 3 studies: no leave-one-out)'}")
    if res.stopped == "cap":
        out.append(
            "  HIT THE STUDY CAP without stabilising: the region is wider than this"
            " data can pin. That is the result — ship the spread, not a point."
        )
    elif res.stopped == "diagnostic":
        out.append("  DIAGNOSTIC RUN (under 3 studies): not a result, do not ship it.")
    out.append("")
    out.append("SCORE  weighted (channel, regime) residuals / baseline scales")
    out.append(
        f"  baseline {res.baseline.total:.5f} -> point {res.point_score.total:.5f}   "
        f"{_pct(res.point_score.total, res.baseline.total)}"
    )
    out.append("")
    out += channel_table(res.baseline, res.point_score, weights or {}, scales)
    return "\n".join(out)


def judgement(
    objective: Objective | PooledObjective,
    base: Mapping[str, float],
    plan: KnobPlan,
    target: Preset,
    args: argparse.Namespace,
) -> str:
    """Loop 1's whole judgement of ONE plant, no search: what was compared,
    with what weight, against what, and what came out (README 4). Scales
    calibrate on the ``--preset`` incumbent exactly as a fit's would."""
    baseline = objective.calibrate(merged(base, plan, {}))
    tvals = {**ENGINE_DEFAULTS, **target.physics, "actuator_tau": target.actuator_tau}
    cand = objective.evaluate(tvals)
    out = [
        f"LOOP-1 JUDGEMENT  plant {target.name!r}  (scales: baseline {args.preset!r})",
        f"  weighted total  baseline {baseline.total:.5f} -> {target.name} {cand.total:.5f}   "
        f"{_pct(cand.total, baseline.total)}",
        f"  envelope {target.envelope or 'none'} (fitted-with; honoured by every rollout)",
        "",
        "KNOBS  value and provenance; [pinned]/[searched] shows the fit plan's view",
    ]
    for name in sorted(tvals):
        disp = (
            "[searched]"
            if name in plan.searched
            else ("[pinned]" if name in plan.pinned else "[base]")
        )
        why = target.provenance.get(name, "")
        if not why and name in plan.pinned:
            why = plan.pinned[name].why
        if not why and name in ENGINE_DEFAULTS and tvals[name] == ENGINE_DEFAULTS[name]:
            why = "menagerie default (never overridden)"
        out.append(f"  {name:<26s} {tvals[name]:>10.5f}  {disp:<10s} {why}")
    out.append("")
    out += channel_table(baseline, cand, objective.weights, objective.scales)
    return "\n".join(out)


# ---------------------------------------------------------------------- CLI


def _parse_weights(arg: str | None, suspended: bool) -> dict[tuple[str, str], float]:
    if arg is None:
        return dict(SUSPENDED_WEIGHTS) if suspended else dict(DEFAULT_WEIGHTS)
    text = Path(arg).read_text() if Path(arg).is_file() else arg
    d = json.loads(text)
    out: dict[tuple[str, str], float] = {}
    for c, per_regime in d.items():
        for r, w in per_regime.items():
            out[(c, r)] = float(w)
    return out


def _overlap_seconds(segments: Sequence[Segment]) -> float:
    ordered = sorted(segments, key=lambda s: s.t0)
    return sum(max(0.0, a.t1 - b.t0) for a, b in itertools.pairwise(ordered))


def _out_file(prefix: Path, ending: str) -> Path:
    """``prefix`` + ``ending`` by string append — dots in the stem survive."""
    return prefix.parent / (prefix.name + ending)


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.fit")
    ap.add_argument(
        "recordings",
        nargs="+",
        help="one or more recordings; several POOL into one objective with "
        "shared scales (how a flight-bearing recording joins the floor one)",
    )
    ap.add_argument("--preset", default="measured", help="the incumbent the search starts from")
    ap.add_argument("--robot", default=None, help="robot.json; pins derive from it")
    ap.add_argument("--knobs", default=None, help="knobs.json: per-knob pin/search")
    ap.add_argument("--search", default=None, help="comma-separated knobs to search")
    ap.add_argument(
        "--weights",
        default=None,
        help='JSON {"channel": {"regime": w}}, inline or a file. '
        "Default: the joint-level partition (joint/dq/tau, README 4); "
        "suspended recordings score joint",
    )
    ap.add_argument(
        "--judge",
        nargs="?",
        const="",
        default=None,
        metavar="PLANT",
        help="no search: print the full loop-1 judgement of PLANT (default: "
        "--preset) under this objective — every channel's residual, scored "
        "or not, with weights, scales and per-knob provenance",
    )
    ap.add_argument("--segments", type=int, default=8)
    ap.add_argument(
        "--segment-length", type=float, nargs=2, default=(4.0, 8.0), metavar=("LO", "HI")
    )
    ap.add_argument(
        "--window",
        type=float,
        nargs="+",
        default=[0.05, 0.8],
        metavar="S",
        help="clip length, s: one value = fixed, two = U(lo, hi)",
    )
    ap.add_argument(
        "--schedule-seed",
        type=int,
        default=0,
        help="seeds segments AND clip schedules; shared by every candidate and every study",
    )
    ap.add_argument(
        "--envelope",
        default=None,
        choices=sorted(TORQUE_ENVELOPES),
        help="fit WITH this measured torque envelope in the actuator chain; the "
        "name is recorded on the saved preset so the plant is never run without "
        "it (fitting without the envelope absorbs its average effect into the "
        "viscous/inertial knobs — README 9)",
    )
    ap.add_argument("--trials", type=int, default=90)
    ap.add_argument("--min-studies", type=int, default=3)
    ap.add_argument("--max-studies", type=int, default=12)
    ap.add_argument("--drift-tol", type=float, default=0.10)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--held-out", default=None, help="score base vs point on this recording too")
    ap.add_argument(
        "--out", default=None, help="prefix: writes .plant.json/.ranges.json/.report.md"
    )
    ap.add_argument("--name", default=None, help="preset name for the plant JSON")
    args = ap.parse_args()

    from contextlib import ExitStack

    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

    backend = MujocoBackend(envelope=TORQUE_ENVELOPES[args.envelope] if args.envelope else None)
    base = base_values(args.preset)
    robot = RobotSpec.from_json(args.robot) if args.robot else None
    search = tuple(args.search.split(",")) if args.search else None
    if args.knobs:
        plan = load_knob_plan(args.knobs, backend.knobs())
    else:
        plan = default_plan(backend.knobs(), robot, search=search)

    declarations = [read_declarations(r) for r in args.recordings]
    all_suspended = all(bool(d.suspended) for d in declarations)
    weights = _parse_weights(args.weights, all_suspended)
    if all_suspended and args.weights is None:
        print("suspended recording(s): scoring joint on the suspended regime")

    window: float | tuple[float, float] = (
        float(args.window[0])
        if len(args.window) == 1
        else (float(args.window[0]), float(args.window[1]))
    )
    part_workers = max(1, args.workers // len(args.recordings))
    parts: list[Objective] = []
    n_segments = 0
    with ExitStack() as stack:
        for idx, (rec, declared) in enumerate(zip(args.recordings, declarations, strict=True)):
            suspended = bool(declared.suspended)
            rollouts = stack.enter_context(Rollouts(rec, backend, workers=part_workers))
            st = rollouts.streams
            spans = regimes(st, declared)
            t_lo = max(float(st.lt[0]), float(st.ct[0]))
            t_hi = min(float(st.lt[-1]), float(st.ct[-1]))
            if args.start is not None:
                t_lo = max(t_lo, args.start)
            if args.seconds is not None:
                t_hi = min(t_hi, t_lo + args.seconds)
            seed = args.schedule_seed + 100 * idx  # disjoint per-segment seed blocks
            segments = sample_segments(
                t_lo,
                t_hi,
                n=args.segments,
                length=(float(args.segment_length[0]), float(args.segment_length[1])),
                seed=seed,
            )
            parts.append(
                Objective(
                    rollouts,
                    segments=segments,
                    spans=spans,
                    weights=weights,
                    backend_channels=backend.channels(),
                    window=window,
                    schedule_seed=seed,
                    suspended=suspended,
                )
            )
            n_segments += len(segments)
            kinds = {s.kind for s in spans}
            print(
                f"{Path(rec).name}  {len(segments)} segments over t={t_lo:.1f}..{t_hi:.1f}s"
                f" ({_overlap_seconds(segments):.1f}s double-covered)  regimes {sorted(kinds)}"
            )
        objective = PooledObjective(parts)
        wtxt = ", ".join(f"{c}/{r}={w:g}" for (c, r), w in weights.items())
        print(
            f"weights {wtxt}"
            + (f"  envelope {args.envelope}" if args.envelope else "")
            + f"  channels {parts[0].channels} (all reported; only weighted ones scored)"
        )

        if args.judge is not None:
            target = load_preset(args.judge or args.preset)
            print()
            print(judgement(objective, base, plan, target, args))
            return

        batch = max(1, args.workers // max(n_segments, 1))
        print(
            f"{args.trials} trials/study, studies {args.min_studies}..{args.max_studies} in "
            f"batches of {batch}, {args.workers} workers\n"
        )

        def progress(o: StudyOutcome) -> None:
            print(
                f"  study seed {o.seed}: best {o.best_total:.5f}, "
                f"harvested {o.n_harvested}/{o.n_trials} within tol {o.tol:.5f}"
            )

        res = fit(
            objective,
            plan,
            base,
            trials=args.trials,
            min_studies=args.min_studies,
            max_studies=args.max_studies,
            drift_tol=args.drift_tol,
            batch=batch,
            on_study=progress,
        )
    # After the fit's pool is CLOSED: the held-out evaluation spawns its own
    # workers, and running both pools at once doubles the process population
    # for the two cheapest evaluations of the run (measured: the overlap
    # killed a 40-minute fit at its last step).
    held_out_lines: list[str] = []
    if args.held_out:
        held_out_lines = _held_out(args, base, plan, res, weights, backend)

    names = "+".join(Path(r).name for r in args.recordings)
    header = f"FIT  {names}  preset {args.preset}  schedule seed {args.schedule_seed}"
    text = format_report(res, header=header, weights=weights, scales=objective.scales)
    if held_out_lines:
        text += "\n\n" + "\n".join(held_out_lines)
    print()
    print(text)

    if args.out:
        # APPEND to the prefix, never with_suffix: a dotted stem like
        # "sweep_lam0.75" would have its ".75" REPLACED, colliding every
        # sweep point onto one file (last writer wins — it happened).
        prefix = Path(str(args.out))
        name = args.name or f"fit-{Path(args.recordings[0]).stem.split('_')[0]}"
        values = merged(base, plan, res.point)
        tau = values.pop("actuator_tau", 0.0)
        Preset(name=name, physics=values, actuator_tau=tau, envelope=args.envelope).save(
            _out_file(prefix, ".plant.json")
        )
        _out_file(prefix, ".ranges.json").write_text(
            json.dumps(
                {
                    "point": res.point,
                    "spread": {k: list(v) for k, v in res.spread.items()},
                    # min/max over the pooled cloud — the statistic the frozen
                    # 4-seed "8.8x" spreads were quoted in. Over n draws it
                    # covers (n-1)/(n+1) of the distribution: too NARROW.
                    "cloud_min_max": {
                        k: [float(v.min()), float(v.max())] for k, v in res.cloud.items()
                    },
                    # The pooled cloud itself, INDEX-ALIGNED across knobs:
                    # row i of every array is one harvested trial, i.e. one
                    # JOINT draw from the region. Draw selection (ground a
                    # draw, ship the best) needs the joint rows — the knobs
                    # trade against each other, so per-knob resampling
                    # fabricates off-valley points the fit never visited.
                    "cloud": {k: [float(x) for x in v] for k, v in res.cloud.items()},
                    "pinned": {
                        k: {"value": p.value, "why": p.why} for k, p in res.plan.pinned.items()
                    },
                    "stopped": res.stopped,
                    "studies": len(res.studies),
                    "pooled_trials": len(next(iter(res.cloud.values()))) if res.cloud else 0,
                },
                indent=2,
            )
        )
        _out_file(prefix, ".report.md").write_text(text + "\n")
        print(f"\nwrote {prefix}.plant.json, .ranges.json, .report.md")


def _held_out(
    args: argparse.Namespace,
    base: Mapping[str, float],
    plan: KnobPlan,
    res: FitResult,
    weights: WeightVector,
    backend: Any,
) -> list[str]:
    """Baseline vs the fitted point on a recording the fit never saw.

    Its own segments, schedules and baseline-calibrated scales — the
    comparison is honest because BOTH plants see the identical objective.
    """
    declared = read_declarations(args.held_out)
    ho = Rollouts(args.held_out, backend, workers=args.workers)
    with ho:
        st = ho.streams
        spans = regimes(st, declared)
        t_lo = max(float(st.lt[0]), float(st.ct[0]))
        t_hi = min(float(st.lt[-1]), float(st.ct[-1]))
        segments = sample_segments(t_lo, t_hi, n=args.segments, seed=args.schedule_seed)
        window: float | tuple[float, float] = (
            float(args.window[0])
            if len(args.window) == 1
            else (float(args.window[0]), float(args.window[1]))
        )
        obj = Objective(
            ho,
            segments=segments,
            spans=spans,
            weights=weights,
            backend_channels=backend.channels(),
            window=window,
            schedule_seed=args.schedule_seed,
            suspended=bool(declared.suspended),
        )
        b = obj.calibrate(merged(base, plan, {}))
        p = obj.evaluate(merged(base, plan, res.point))
    return [
        f"HELD OUT  {Path(args.held_out).name}  (never part of the objective)",
        f"  baseline {b.total:.5f} -> point {p.total:.5f}   {_pct(p.total, b.total)}",
    ]


if __name__ == "__main__":
    main()
