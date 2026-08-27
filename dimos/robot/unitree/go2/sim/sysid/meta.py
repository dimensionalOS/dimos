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

"""Loop 2's meta-search: the grounding adjudicates loop-1's hyperparameters.

The outer space is the loop-1 WEIGHT VECTOR's four live axes (README 4):
``w_accel`` (the accel share — the measured principal axis of the
misspecification map, and what keeps the incumbent's objective REACHABLE),
``w_flight`` (real now that a flight-bearing recording is in the fit set)
and the ``dq``/``tau`` weights relative to ``joint``. The axes the two
retired scorers once disagreed on — stratified segment sampling and
per-channel normalisation — are gone: normalisation is always on (raw
residual summation is a unit choice, not a hypothesis), and segment
sampling is the fit CLI's ``--segments``/``--segment-length``, a coverage
question loop 1's own identifiability instrument answers more cheaply
than the referee can.

    # ground candidate plants under one shared floor:
    python -m dimos.robot.unitree.go2.sim.sysid.meta experiment REC.mcap \
        NET.bin --presets measured stock results/fit.plant.json

    # the full nested search (EXPENSIVE: every trial is a whole inner fit):
    python -m dimos.robot.unitree.go2.sim.sysid.meta outer FIT.mcap [FIT2...] \
        VAL.mcap NET.bin --trials 10 --workers 16 --out results/outer.json

Three design constraints, each load-bearing:

* OUTER TRIALS ARE EXPENSIVE — each runs a full inner fit (seeded restarts,
  median + spread). Ten or twenty, not hundreds; TPE, not CMA-ES.
* THREE SPLITS. Inner fits on the fit recordings, the outer study selects on
  a validation recording, and the final number is quoted on a THIRD recording
  neither has touched. Selecting hyperparameters on the recording you then
  report is overfitting one storey up, with the added insult that it looks
  rigorous. This module never reads the reserved final-quote recording.
* THE OUTER DIMENSION STAYS TINY. Loop 2 yields ~11 statistics per
  recording; that supports two or three decisions. Everything else stays at
  a documented default.

One obligation: EVERY trial's fitted plant persists with its loop-2 score.
The trials the referee cannot distinguish from the winner (within the MDD)
are samples of a SECOND DR component — misspecification spread, which no
amount of data shrinks — beside the fit's own p10-p90. Reported
separately, never merged.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.backend import ClosedLoopBackend
from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.ranges import Preset, load_preset
from dimos.robot.unitree.go2.sim.sysid.fit import (
    FitResult,
    Objective,
    PooledObjective,
    base_values,
    default_plan,
    fit,
    merged,
)
from dimos.robot.unitree.go2.sim.sysid.ground import (
    ObsNoise,
    Report,
    ground,
    sim_noise,
    usable_floor,
)
from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
from dimos.robot.unitree.go2.sim.sysid.loop import (
    LATENCY_BAND_S,
    sensor_noise,
    timing_of,
    transport_leg,
)
from dimos.robot.unitree.go2.sim.sysid.probe import FOCUS, Probe, Spectrum, spectrum
from dimos.robot.unitree.go2.sim.sysid.real import real_summary, robot_noise
from dimos.robot.unitree.go2.sim.sysid.recording import Streams, read_declarations
from dimos.robot.unitree.go2.sim.sysid.regimes import regimes, sample_segments
from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts
from dimos.robot.unitree.go2.sim.sysid.stats import Summary

# Loop 2's minimum detectable differences (README 8, bootstrap, 95%),
# re-measured 2026-08-17 on the CURRENT loss and loop (tracking areas,
# measured-state seeding — whose honest draw tail is what coarsens n=8;
# plant-selection verdicts want --replicates 16). Outer trials within
# the matching MDD of the best are ties, their fitted plants admissible
# DR samples (module docstring).
MDD_N8 = 0.571
MDD_N16 = 0.260


@dataclass(frozen=True)
class OuterPoint:
    """One loop-1 hyperparameter setting — a point the outer study evaluates.

    Four decisions: ``w_accel`` (the fraction of channel mass on ``accel``
    — the MEASURED principal axis of the misspecification map, worth
    0.6-0.9 of referee loss between its endpoints), ``w_flight``, and the
    ``dq``/``tau`` weights relative to ``joint`` within the joint family's
    (1 - w_accel) share. One more axis than the two-or-three rule wants,
    kept because pinning it at zero confines the search to the family the
    referee just rejected while the incumbent's objective sits outside the
    reachable set (README 4). ``w_accel=1, w_flight=0.5`` reproduces the
    incumbent's objective exactly; any nonzero ``w_accel`` re-couples
    loop 1 to a referee quantity, so it must justify itself on the
    held-out RECORDING, not merely on held-out segments.
    """

    name: str
    w_accel: float = 0.0  # fraction of channel mass on accel
    w_flight: float = 0.25  # share of every channel's weight on the flight regime
    w_dq: float = 1.0  # dq weight relative to joint, within the family share
    w_tau: float = 0.5  # tau weight relative to joint, within the family share

    def weights(self) -> dict[tuple[str, str], float]:
        family = {"joint": 1.0, "dq": self.w_dq, "tau": self.w_tau}
        total = sum(family.values()) or 1.0
        out: dict[tuple[str, str], float] = {}
        for channel, wc in family.items():
            mass = (1.0 - self.w_accel) * wc / total
            if mass > 0:
                out[(channel, "floor")] = mass * (1.0 - self.w_flight)
                out[(channel, "flight")] = mass * self.w_flight
        if self.w_accel > 0:
            out[("accel", "floor")] = self.w_accel * (1.0 - self.w_flight)
            out[("accel", "flight")] = self.w_accel * self.w_flight
        return out


# The two seed points: the PARTITION's default (DEFAULT_WEIGHTS' shape —
# joint .3/.1, dq .3/.1, tau .15/.05) and the INCUMBENT's objective (the
# accel scorer the shipped plant was actually fitted under, which grounds
# at 1.61 against the partition point's 2.21) — the search must be able to
# reach the winner, not only the principle.
DEFAULT_POINT = OuterPoint("partition-default")
INCUMBENT_POINT = OuterPoint("incumbent-accel", w_accel=1.0, w_flight=0.5)


def inner_fit(
    recordings: Sequence[str | Path],
    point: OuterPoint,
    backend: ClosedLoopBackend,
    *,
    workers: int = 1,
    n_segments: int = 8,
    trials: int = 90,
    max_studies: int = 12,
    schedule_seed: int = 0,
) -> tuple[dict[str, float], FitResult]:
    """One full loop-1 fit under ``point``'s hyperparameters.

    Returns the complete knob values of the fitted POINT (median, never the
    best draw) and the full result. ``recordings`` pool into one objective
    with shared scales (fit.PooledObjective) — the fit set is expected to
    carry both `floor` and `flight` so the regime axis refers to something.
    """
    from contextlib import ExitStack

    base = base_values("measured")
    plan = default_plan(backend.knobs())
    part_workers = max(1, workers // len(list(recordings)))
    parts = []
    n_total = 0
    with ExitStack() as stack:
        for idx, rec in enumerate(recordings):
            declared = read_declarations(rec)
            rollouts = stack.enter_context(Rollouts(rec, backend, workers=part_workers))
            st = rollouts.streams
            spans = regimes(st, declared)
            t_lo = max(float(st.lt[0]), float(st.ct[0]))
            t_hi = min(float(st.lt[-1]), float(st.ct[-1]))
            seed = schedule_seed + 100 * idx
            segments = sample_segments(t_lo, t_hi, n=n_segments, seed=seed)
            parts.append(
                Objective(
                    rollouts,
                    segments=segments,
                    spans=spans,
                    weights=point.weights(),
                    backend_channels=backend.channels(),
                    schedule_seed=seed,
                    suspended=bool(declared.suspended),
                )
            )
            n_total += len(segments)
        res = fit(
            PooledObjective(parts),
            plan,
            base,
            trials=trials,
            max_studies=max_studies,
            batch=max(1, workers // max(n_total, 1)),
        )
    return merged(base, plan, res.point), res


def ground_values(
    validation: str | Path,
    policy: FreePolicy,
    values: dict[str, float],
    backend: ClosedLoopBackend,
    *,
    name: str,
    noise: dict[str, float],
    floor_source: str,
    start: float = 6.0,
    replicates: int = 8,
) -> Report:
    """Score a fitted point on the grounding — the outer study's objective.

    Replicated (README 4a): the returned report's loss is the median over
    ``replicates`` rollouts, and its spread bounds what a single draw would
    have read — outer trials separated by less than that spread are ties.
    """
    tau = values.get("actuator_tau", 0.0)
    physics = {k: v for k, v in values.items() if k != "actuator_tau"}
    preset = Preset(name=name, physics=physics, actuator_tau=tau)
    st = read_streams(validation)
    return ground(
        st,
        policy,
        preset,
        backend,
        start=start,
        noise=noise,
        floor_source=floor_source,
        replicates=replicates,
        with_ghost=False,
    )


def shared_floor(
    validation: str | Path,
    policy: FreePolicy,
    backend: ClosedLoopBackend,
    *,
    baseline: str = "measured",
    seeds: int = 4,
    start: float = 6.0,
    seconds: float | None = None,
) -> tuple[dict[str, float], str]:
    """ONE noise floor for every candidate, measured on the baseline plant.

    Per-candidate floors would let a plant buy small SNRs with its own chaos;
    a shared floor keeps the comparison about the physics. Measured on the
    incumbent and clamped (:func:`usable_floor`) — swap in
    :func:`~dimos.robot.unitree.go2.sim.sysid.real.robot_noise` when repeat
    recordings exist.
    """
    st = read_streams(validation)
    base = load_preset(baseline)
    raw = sim_noise(st, policy, base, backend, seeds=seeds, start=start, seconds=seconds)
    span = float(st.wt[-1]) - start if seconds is None else seconds
    real = real_summary(st, start=start, seconds=span)
    return usable_floor(raw, real.as_dict()), f"sim-perturb ({baseline} plant, shared)"


# ------------------------------------------------------------ the experiment


def experiment(
    recording: str | Path,
    policy_bin: str | Path,
    presets: list[str],
    backend: ClosedLoopBackend,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    seeds: int = 4,
    replicates: int = 8,
    workers: int = 1,
) -> str:
    """Ground each plant under one shared floor and say which the referee prefers.

    The referee's plant-vs-plant instrument: candidates share one floor (a
    per-candidate floor would let a plant buy small SNRs with its own
    chaos), losses closer than the widest draw range are declared a tie,
    and the ordering — not any loop-1 score — is what promotes a plant
    (README 4's converse rule).

    ``start``/``seconds`` window the comparison — for a mixed recording, run
    it on the span the verified net actually drove and nothing else.
    """
    policy = FreePolicy.load(policy_bin)
    noise, source = shared_floor(
        recording, policy, backend, seeds=seeds, start=start, seconds=seconds
    )
    st = read_streams(recording)
    reports: list[Report] = []
    lines: list[str] = [
        f"GROUNDING  {Path(str(recording)).name}  floor: {source}",
        "",
    ]
    for name in presets:
        p = load_preset(name)
        rep = ground(
            st,
            policy,
            p,
            backend,
            start=start,
            seconds=seconds,
            noise=noise,
            floor_source=source,
            replicates=replicates,
            workers=workers,
            with_ghost=False,
        )
        reports.append(rep)
        lines += [rep.table(), ""]
    order = sorted(reports, key=lambda r: r.loss())
    lines.append("VERDICT  (loss = RMS over SNRs; lower = closer to the real run)")
    for r in order:
        n, of = r.n_matched()
        lo, hi = r.loss_range()
        lines.append(
            f"  {r.preset:<28s} loss {r.loss():6.2f} (draws {lo:.2f}-{hi:.2f})   "
            f"{n} of {of} within the floor"
        )
    lines.append("  Losses closer than the widest draw range above are a TIE, not an ordering.")
    return "\n".join(lines)


# ------------------------------------------- the measured-mechanism grounding


@dataclass(frozen=True)
class MeasuredLoop:
    """One recording span's measured loop mechanisms — no fitted values."""

    noise: ObsNoise  # the >20 Hz sensor floor, per channel
    intervals: tuple[float, ...]  # the executor's inter-command sequence, s
    transport_ms: float | None  # median command-transport leg, when measurable


def measured_loop(
    recording: str | Path, st: Streams, *, start: float, seconds: float
) -> MeasuredLoop:
    """Measure every loop mechanism this recording span can parameterise."""
    try:
        leg: float | None = transport_leg(recording).median_ms
    except ValueError:
        leg = None
    return MeasuredLoop(
        noise=sensor_noise(st, t0=start, t1=start + seconds).obs_noise(),
        intervals=tuple(timing_of(st, t0=start, t1=start + seconds).intervals),
        transport_ms=leg,
    )


def _replicated(
    name: str, replicates: int, base: dict[str, float], tau: float, **kw: object
) -> list[Probe]:
    return [
        Probe(f"{name} #{i}", dict(base), tau, perturb_seed=10 + i, noise_seed=i, **kw)  # type: ignore[arg-type]
        for i in range(replicates)
    ]


def mechanism_probes(
    base_physics: dict[str, float],
    base_tau: float,
    ml: MeasuredLoop,
    *,
    replicates: int = 3,
    fitted: tuple[float, float] = (0.010, 1.0),
    envelope: str | None = None,
) -> list[Probe]:
    """Each loop mechanism at its MEASURED value, alone and stacked, plus the
    previously FITTED point (latency 10 ms + training noise) for comparison.

    ``envelope`` is the base PLANT's own envelope (a fitted-with-envelope
    preset), applied to every probe — it is part of the plant here, not one of
    the mechanisms under test."""
    lo, hi = LATENCY_BAND_S
    configs: list[tuple[str, dict[str, object]]] = [
        ("ideal loop", {}),
        (
            f"FITTED lat={fitted[0] * 1e3:g}ms+noise x{fitted[1]:g}",
            {"action_latency": fitted[0], "noise_scale": fitted[1]},
        ),
        (f"lat={lo * 1e3:g}ms (cmd leg)", {"action_latency": lo}),
        (f"lat={hi * 1e3:g}ms (band top)", {"action_latency": hi}),
        ("noise=measured", {"noise": ml.noise}),
        ("timing=measured", {"timing": ml.intervals}),
        ("timing+noise", {"timing": ml.intervals, "noise": ml.noise}),
        (
            f"timing+noise+lat={lo * 1e3:g}ms",
            {"timing": ml.intervals, "noise": ml.noise, "action_latency": lo},
        ),
        (
            f"timing+noise+lat={hi * 1e3:g}ms",
            {"timing": ml.intervals, "noise": ml.noise, "action_latency": hi},
        ),
    ]
    out: list[Probe] = []
    for name, kw in configs:
        out += _replicated(name, replicates, base_physics, base_tau, envelope=envelope, **kw)
    return out


def grouped_losses(spec: Spectrum) -> dict[str, tuple[float, float, list[Summary]]]:
    """Mean loss, half-spread and the summaries per ``name #i`` probe group."""
    groups: dict[str, list[Summary]] = {}
    for probe, s in spec.results:
        groups.setdefault(probe.name.rsplit(" #", 1)[0], []).append(s)
    out = {}
    for name, ss in groups.items():
        losses = [spec.loss(s) for s in ss]
        out[name] = (
            float(np.mean(losses)),
            float((max(losses) - min(losses)) / 2),
            ss,
        )
    return out


def mechanism_table(spec: Spectrum) -> str:
    """The measured-mechanism comparison, aggregated over replicates."""
    real = spec.real.as_dict()
    rows = grouped_losses(spec)
    lines = [
        f"MEASURED MECHANISMS  base={spec.preset}  {spec.seconds:.0f}s from "
        f"t={spec.start:.0f}s  ({len(next(iter(rows.values()))[2])} replicates, "
        "loss = grounding RMS-SNR over the shared base floor)",
        "",
        f"{'config':<28}{'loss':>12}  " + "".join(f"{k:>11}" for k in FOCUS),
        f"{'(real)':<28}{'':>12}  " + "".join(f"{real[k]:11.3f}" for k in FOCUS),
        f"{'(base, unperturbed)':<28}{spec.loss(spec.base):>12.2f}  "
        + "".join(f"{spec.base.as_dict()[k]:11.3f}" for k in FOCUS),
    ]
    for name, (loss, spread, ss) in rows.items():
        means = {k: float(np.mean([s.as_dict()[k] for s in ss])) for k in FOCUS}
        lines.append(
            f"{name:<28}{loss:>7.2f} ±{spread:4.2f}  " + "".join(f"{means[k]:11.3f}" for k in FOCUS)
        )
    return "\n".join(lines)


def run_mechanisms(
    recording: str | Path,
    policy_bin: str | Path,
    backend: ClosedLoopBackend,
    *,
    preset: str = "measured",
    start: float = 6.0,
    seconds: float | None = None,
    replicates: int = 3,
    workers: int = 1,
) -> None:
    """Ground each mechanism at its measured value, not a fitted one (README 9)."""
    st = read_streams(recording)
    span = float(st.wt[-1]) - start if seconds is None else seconds
    ml = measured_loop(recording, st, start=start, seconds=span)
    p = load_preset(preset)
    probes = mechanism_probes(
        dict(p.physics), p.actuator_tau, ml, replicates=replicates, envelope=p.envelope
    )
    spec = spectrum(
        recording,
        policy_bin,
        probes,
        backend,
        preset=preset,
        start=start,
        seconds=seconds,
        workers=workers,
    )
    n = ml.noise
    print(
        f"measured: transport {ml.transport_ms and round(ml.transport_ms, 2)} ms | "
        f"timing mean {np.mean(ml.intervals) * 1e3:.2f} ms "
        f"({len(ml.intervals)} intervals) | noise dq ±{n.dq:.3f} gyro ±{n.gyro:.4f} "
        f"q ±{n.q:.5f} grav ±{n.gravity:.5f}"
    )
    print(mechanism_table(spec))


# --------------------------------------------------- the latency proxy search


def latency_probes(
    base_physics: dict[str, float],
    base_tau: float,
    grid_s: tuple[float, ...],
    ml: MeasuredLoop | None,
    *,
    replicates: int = 3,
    envelope: str | None = None,
) -> list[Probe]:
    out: list[Probe] = []
    extra: dict[str, object] = {"timing": ml.intervals, "noise": ml.noise} if ml is not None else {}
    for lat in grid_s:
        out += _replicated(
            f"lat={lat * 1e3:g}ms",
            replicates,
            base_physics,
            base_tau,
            action_latency=lat,
            envelope=envelope,
            **extra,
        )
    return out


def run_latency(
    select_rec: str | Path,
    report_rec: str | Path,
    policy_bin: str | Path,
    backend: ClosedLoopBackend,
    *,
    preset: str = "measured",
    grid_ms: tuple[float, ...] = (0.0, 1.5, 3.0, 4.5, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0),
    with_measured: bool = True,
    replicates: int = 3,
    workers: int = 1,
    select_start: float = 6.0,
    select_seconds: float | None = None,
    report_start: float = 6.0,
    report_seconds: float | None = None,
) -> None:
    """``action_latency`` as a loop-2 knob, selected/reported on SPLIT recordings.

    The train/select/report split is what makes tuning against the grounding
    legitimate: the grid is scored on ``select_rec``, the argmin (once bounded
    to the measured band, once free) is then quoted on ``report_rec``, which
    the selection never read. If the UNBOUNDED search pushes far above
    :data:`LATENCY_BAND_S`, the fitted 10 ms was a PROXY — the one available
    knob standing in for a mechanism that is not a transport delay.
    """

    def one(
        rec: str | Path, start: float, seconds: float | None, grid: tuple[float, ...]
    ) -> dict[float, tuple[float, float]]:
        st = read_streams(rec)
        span = float(st.wt[-1]) - start if seconds is None else seconds
        ml = measured_loop(rec, st, start=start, seconds=span) if with_measured else None
        p = load_preset(preset)
        probes = latency_probes(
            dict(p.physics), p.actuator_tau, grid, ml, replicates=replicates, envelope=p.envelope
        )
        spec = spectrum(
            rec,
            policy_bin,
            probes,
            backend,
            preset=preset,
            start=start,
            seconds=seconds,
            workers=workers,
        )
        return {
            float(name.split("=")[1].removesuffix("ms")) / 1e3: (loss, spread)
            for name, (loss, spread, _ss) in grouped_losses(spec).items()
        }

    mech = "measured timing+noise ON" if with_measured else "bare (no other mechanisms)"
    print(f"SELECT  {Path(str(select_rec)).name}  {mech}")
    sel = one(select_rec, select_start, select_seconds, tuple(x / 1e3 for x in grid_ms))
    for lat, (loss, spread) in sorted(sel.items()):
        print(f"  lat {lat * 1e3:5.1f} ms  loss {loss:6.2f} ±{spread:4.2f}")
    bounded = min((lat for lat in sel if lat <= LATENCY_BAND_S[1]), key=lambda k: sel[k][0])
    unbounded = min(sel, key=lambda k: sel[k][0])
    print(f"  selected: bounded-to-measured {bounded * 1e3:g} ms, unbounded {unbounded * 1e3:g} ms")

    grid_rep = tuple(sorted({0.0, bounded, unbounded}))
    print(f"\nREPORT  {Path(str(report_rec)).name}  (held out from the selection)")
    rep = one(report_rec, report_start, report_seconds, grid_rep)
    for lat, (loss, spread) in sorted(rep.items()):
        tag = (
            " <- bounded" * (lat == bounded and lat != 0.0)
            + " <- unbounded" * (lat == unbounded and lat != 0.0)
            + " (reference)" * (lat == 0.0)
        )
        print(f"  lat {lat * 1e3:5.1f} ms  loss {loss:6.2f} ±{spread:4.2f}{tag}")

    band = f"{LATENCY_BAND_S[0] * 1e3:g}-{LATENCY_BAND_S[1] * 1e3:g} ms"
    if unbounded > LATENCY_BAND_S[1]:
        how = f"{unbounded / LATENCY_BAND_S[1]:.1f}x the measured band's top ({band})"
        print(
            f"\nVERDICT: the unbounded search wants {unbounded * 1e3:g} ms — {how}. "
            "A latency the loop cannot physically contain is a PROXY: it moves the right "
            "statistics for the wrong reason, and the missing mechanism is still missing."
        )
    else:
        print(
            f"\nVERDICT: the unbounded search stays within the measured band ({band}) — "
            "the latency mechanism is real and its measured value carries it."
        )


# ------------------------------------------------------------ the outer study


def second_dr_component(log: Sequence[dict[str, object]], mdd: float = MDD_N8) -> dict[str, object]:
    """The misspecification DR component: per-knob spread over the trials
    tied with the best (within ``mdd`` — the fit's 1-SE harvest, one
    storey up). Small tied sets make it coarse; the output says so."""
    losses = [float(e["ground_loss"]) for e in log]  # type: ignore[arg-type]
    best = min(losses)
    tied = [e for e, loss in zip(log, losses, strict=True) if loss - best <= mdd]
    knobs: dict[str, list[float]] = {}
    for e in tied:
        for k, v in dict(e["values"]).items():  # type: ignore[call-overload]
            knobs.setdefault(k, []).append(float(v))
    return {
        "admissible_trials": [e["trial"] for e in tied],
        "mdd": mdd,
        "caveat": f"n={len(tied)} tied trials: a coarse estimate, not a distribution",
        "spread": {k: [min(v), max(v)] for k, v in knobs.items() if max(v) > min(v)},
    }


def run_outer(
    fit_recordings: Sequence[str | Path],
    validation: str | Path,
    policy_bin: str | Path,
    backend: ClosedLoopBackend,
    *,
    trials: int = 10,
    workers: int = 1,
    inner_trials: int = 90,
    max_studies: int = 12,
    replicates: int = 16,
    floor_from: Sequence[str | Path] | None = None,
    seed_points: Sequence[OuterPoint] = (INCUMBENT_POINT, DEFAULT_POINT),
    out: str | Path | None = None,
) -> None:
    """Nested Optuna: TPE over the LIVE axes, each trial a full inner fit.

    The searched axes are ``w_accel`` and the ``w_dq``/``w_tau`` ratios;
    ``w_flight`` is FIXED at the default — measured inert (0.03/0.10 across
    full refits under two judges), so searching it would spend trials on a
    dead dial. ``floor_from`` builds the robot-repeat floor from repeat
    recordings (the sim-perturb ``shared_floor`` is the fallback);
    ``replicates`` defaults to 16 — the honest seed's draw tail puts the
    n=8 MDD at 0.571 vs 0.260 at n=16. The final number must be quoted on
    a recording this function never sees. Every trial's fitted plant
    persists in the ``--out`` log with its grounding score — the losers
    within the MDD of the winner are the samples of the second DR
    component (:func:`second_dr_component`), not discards.
    """
    import optuna

    policy = FreePolicy.load(policy_bin)
    if floor_from is not None:
        st_v = read_streams(validation)
        span = float(st_v.wt[-1]) - 6.0
        raw = robot_noise([read_streams(r) for r in floor_from])
        noise = usable_floor(raw, real_summary(st_v, start=6.0, seconds=span).as_dict())
        source = "robot-repeat"
    else:
        noise, source = shared_floor(validation, policy, backend)
    mdd = MDD_N16 if replicates >= 16 else MDD_N8
    log: list[dict[str, object]] = []

    def objective(trial: optuna.Trial) -> float:
        point = OuterPoint(
            name=f"trial-{trial.number}",
            w_accel=trial.suggest_float("w_accel", 0.0, 1.0),
            w_flight=DEFAULT_POINT.w_flight,
            w_dq=trial.suggest_float("w_dq", 0.0, 2.0),
            w_tau=trial.suggest_float("w_tau", 0.0, 2.0),
        )
        values, res = inner_fit(
            fit_recordings,
            point,
            backend,
            workers=workers,
            trials=inner_trials,
            max_studies=max_studies,
        )
        rep = ground_values(
            validation,
            policy,
            values,
            backend,
            name=point.name,
            noise=noise,
            floor_source=source,
            replicates=replicates,
        )
        n, of = rep.n_matched()
        lo, hi = rep.loss_range()
        log.append(
            {
                "trial": trial.number,
                "point": point.__dict__,
                # EVERY trial's fitted plant, spread included — the tied
                # ones are DR samples, not discards (module docstring).
                "values": values,
                "spread": {k: list(v) for k, v in res.spread.items()},
                "inner_stopped": res.stopped,
                "ground_loss": rep.loss(),
                "ground_loss_draws": [lo, hi],
                "matched": [n, of],
            }
        )
        if out is not None:
            Path(out).write_text(
                json.dumps({"trials": log, "second_dr": second_dr_component(log, mdd)}, indent=2)
            )
        return rep.loss()

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=0))
    for p in seed_points:
        study.enqueue_trial({"w_accel": p.w_accel, "w_dq": p.w_dq, "w_tau": p.w_tau})
    study.optimize(objective, n_trials=trials)
    best = study.best_trial
    print(f"best outer point: {best.params}  ground loss {best.value:.3f}")
    dr2 = second_dr_component(log, mdd)
    print(
        f"second DR component (misspecification spread over {len(dr2['admissible_trials'])} "  # type: ignore[arg-type]
        f"trials tied within MDD {mdd}): {dr2['spread']}"
    )
    print(f"  caveat: {dr2['caveat']}")
    print(
        "Selected on the validation recording — quote the final number on a "
        "third recording this run never touched."
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.meta")
    sub = ap.add_subparsers(dest="cmd", required=True)

    ex = sub.add_parser("experiment", help="ground the candidate plants; which scorer survives?")
    ex.add_argument("recording")
    ex.add_argument("policy_bin")
    ex.add_argument(
        "--presets",
        nargs="+",
        default=["measured", "accel"],
        help="preset names or plant JSON paths",
    )
    ex.add_argument("--start", type=float, default=6.0)
    ex.add_argument("--seconds", type=float, default=None)
    ex.add_argument("--seeds", type=int, default=4, help="floor rollouts (shared_floor)")
    ex.add_argument("--replicates", type=int, default=8, help="verdict rollouts per preset")
    ex.add_argument(
        "--workers", type=int, default=1, help="fan each preset's replicate rollouts out"
    )

    ot = sub.add_parser("outer", help="the nested search (EXPENSIVE: trials are inner fits)")
    ot.add_argument(
        "fit_recordings",
        nargs="+",
        help="fit-set recordings (pooled; include a flight-bearing one), then "
        "the validation recording, then the policy blob",
    )
    ot.add_argument("validation")
    ot.add_argument("policy_bin")
    ot.add_argument("--trials", type=int, default=10)
    ot.add_argument("--workers", type=int, default=1)
    ot.add_argument("--inner-trials", type=int, default=90)
    ot.add_argument("--max-studies", type=int, default=12)
    ot.add_argument("--replicates", type=int, default=16, help="verdict rollouts per trial")
    ot.add_argument(
        "--floor-from",
        nargs="+",
        default=None,
        metavar="REC",
        help="repeat recordings for a robot-repeat verdict floor (else sim-perturb)",
    )
    ot.add_argument(
        "--seed",
        action="append",
        default=None,
        metavar="JSON",
        help='extra seed point, e.g. {"w_accel": 0.5, "w_dq": 1.0, "w_tau": 0.5}; '
        "repeatable. Default seeds: the incumbent objective and the partition point",
    )
    ot.add_argument("--out", default=None, help="JSON log path, written after every trial")

    me = sub.add_parser("mechanisms", help="ground each loop mechanism at its MEASURED value")
    me.add_argument("recording")
    me.add_argument("policy_bin")
    me.add_argument("--preset", default="measured")
    me.add_argument("--start", type=float, default=6.0)
    me.add_argument("--seconds", type=float, default=None)
    me.add_argument("--replicates", type=int, default=3)
    me.add_argument("--workers", type=int, default=1)

    la = sub.add_parser("latency", help="the proxy question: select on one, report on another")
    la.add_argument("select_recording")
    la.add_argument("report_recording")
    la.add_argument("policy_bin")
    la.add_argument("--preset", default="measured")
    la.add_argument(
        "--grid-ms",
        type=float,
        nargs="+",
        default=[0.0, 1.5, 3.0, 4.5, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0],
    )
    la.add_argument(
        "--bare",
        action="store_true",
        help="search latency ALONE (no measured timing/noise) — the fitted regime",
    )
    la.add_argument("--replicates", type=int, default=3)
    la.add_argument("--workers", type=int, default=1)
    la.add_argument("--select-start", type=float, default=6.0)
    la.add_argument("--select-seconds", type=float, default=None)
    la.add_argument("--report-start", type=float, default=6.0)
    la.add_argument("--report-seconds", type=float, default=None)

    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

    backend = MujocoBackend()
    if args.cmd == "experiment":
        print(
            experiment(
                args.recording,
                args.policy_bin,
                args.presets,
                backend,
                start=args.start,
                seconds=args.seconds,
                seeds=args.seeds,
                replicates=args.replicates,
                workers=args.workers,
            )
        )
    elif args.cmd == "mechanisms":
        run_mechanisms(
            args.recording,
            args.policy_bin,
            backend,
            preset=args.preset,
            start=args.start,
            seconds=args.seconds,
            replicates=args.replicates,
            workers=args.workers,
        )
    elif args.cmd == "latency":
        run_latency(
            args.select_recording,
            args.report_recording,
            args.policy_bin,
            backend,
            preset=args.preset,
            grid_ms=tuple(args.grid_ms),
            with_measured=not args.bare,
            replicates=args.replicates,
            workers=args.workers,
            select_start=args.select_start,
            select_seconds=args.select_seconds,
            report_start=args.report_start,
            report_seconds=args.report_seconds,
        )
    else:
        seeds: list[OuterPoint] = [INCUMBENT_POINT, DEFAULT_POINT]
        for i, blob in enumerate(args.seed or []):
            seeds.append(OuterPoint(name=f"seed-{i}", **json.loads(blob)))
        run_outer(
            args.fit_recordings,
            args.validation,
            args.policy_bin,
            backend,
            trials=args.trials,
            workers=args.workers,
            inner_trials=args.inner_trials,
            max_studies=args.max_studies,
            replicates=args.replicates,
            floor_from=args.floor_from,
            seed_points=seeds,
            out=args.out,
        )


if __name__ == "__main__":
    main()
