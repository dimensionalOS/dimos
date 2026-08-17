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

Two self-consistent scorers disagree about which plant is better: the frozen
one (raw residual, one contiguous window) prefers ``accel`` over ``measured``
by 11-18% on held-out jumps; this package's (per-channel normalisation,
stratified segments) says ``accel`` is marginally worse. Those are not rival
philosophies — they are TWO POINTS in the hyperparameter space this module
searches. Loop 1 cannot arbitrate, because that would judge a scorer with a
scorer; the grounding (:mod:`~dimos.robot.unitree.go2.sim.sysid.ground`) is
the referee that lives outside both.

    # the experiment that decides which scorer survives (run this first):
    python -m dimos.robot.unitree.go2.sim.sysid.meta experiment REC.mcap \
        NET.bin --presets measured accel results/fit5-freewalk.plant.json

    # the full nested search (EXPENSIVE: every trial is a whole inner fit):
    python -m dimos.robot.unitree.go2.sim.sysid.meta outer FIT.mcap VAL.mcap \
        NET.bin --trials 10 --workers 16

Three design constraints, each load-bearing:

* OUTER TRIALS ARE EXPENSIVE — each runs a full inner fit (seeded restarts,
  median + spread). Ten or twenty, not hundreds; TPE, not CMA-ES.
* THREE SPLITS. Inner fits on the fit recording, the outer study selects on
  a validation recording, and the final number is quoted on a THIRD recording
  neither has touched. Selecting hyperparameters on the recording you then
  report is overfitting one storey up, with the added insult that it looks
  rigorous. This module never reads the reserved final-quote recording.
* THE OUTER DIMENSION STAYS TINY. Loop 2 yields ~11 statistics per
  recording; that supports two or three decisions. Everything else stays at
  a documented default.
"""

from __future__ import annotations

import argparse
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
from dimos.robot.unitree.go2.sim.sysid.real import real_summary
from dimos.robot.unitree.go2.sim.sysid.recording import Streams, read_declarations
from dimos.robot.unitree.go2.sim.sysid.regimes import Segment, regimes, sample_segments
from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts
from dimos.robot.unitree.go2.sim.sysid.stats import Summary


@dataclass(frozen=True)
class OuterPoint:
    """One loop-1 hyperparameter setting — a point the outer study evaluates.

    Deliberately three decisions and no more. ``stratified`` and
    ``normalised`` are the axes the two incumbent scorers actually disagree
    on; ``w_flight`` is the invented constant the README flags as loop 2's
    first selection. Everything else in the inner objective stays at its
    documented default.
    """

    name: str
    stratified: bool  # sampled segments across the recording vs one contiguous window
    normalised: bool  # per-channel baseline scales vs raw residual
    w_flight: float = 0.5

    def weights(self) -> dict[tuple[str, str], float]:
        return {("accel", "floor"): 1.0 - self.w_flight, ("accel", "flight"): self.w_flight}


# The two known settings — the seeds of the outer study, and the whole of the
# first experiment. FROZEN_STYLE reproduces the frozen scorer's SHAPE (raw
# accel residual over one contiguous stretch), not its byte-level behaviour.
FROZEN_STYLE = OuterPoint("frozen-style", stratified=False, normalised=False)
GO2SIM_STYLE = OuterPoint("go2sim-style", stratified=True, normalised=True)


def inner_fit(
    recording: str | Path,
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
    best draw) and the full result. The non-stratified variant scores one
    contiguous segment — the frozen convention — which also collapses the
    paired-SE harvest to exact ties; the restart median still pools across
    studies.
    """
    base = base_values("measured")
    plan = default_plan(backend.knobs())
    declared = read_declarations(recording)
    rollouts = Rollouts(recording, backend, workers=workers)
    st = rollouts.streams
    spans = regimes(st, declared)
    t_lo = max(float(st.lt[0]), float(st.ct[0]))
    t_hi = min(float(st.lt[-1]), float(st.ct[-1]))
    if point.stratified:
        segments = sample_segments(t_lo, t_hi, n=n_segments, seed=schedule_seed)
    else:
        segments = [Segment(t_lo, t_hi - t_lo)]
    objective = Objective(
        rollouts,
        segments=segments,
        spans=spans,
        weights=point.weights(),
        backend_channels=backend.channels(),
        schedule_seed=schedule_seed,
        suspended=bool(declared.suspended),
        normalise=point.normalised,
    )
    with rollouts:
        res = fit(
            objective,
            plan,
            base,
            trials=trials,
            max_studies=max_studies,
            batch=max(1, workers // max(len(segments), 1)),
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
) -> Report:
    """Score a fitted point on the grounding — the outer study's objective."""
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
) -> str:
    """Ground each plant under one shared floor and say which the referee prefers.

    THE FIRST DELIVERABLE, and it is an experiment, not a feature: the frozen
    scorer's winner (``accel``) and this package's (``measured``, plus the
    phase-4 fitted point) disagree; whichever plant the grounding prefers
    tells us which scorer survives into the outer study's seed.

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
            with_ghost=False,
        )
        reports.append(rep)
        lines += [rep.table(), ""]
    order = sorted(reports, key=lambda r: r.loss())
    lines.append("VERDICT  (loss = RMS over SNRs; lower = closer to the real run)")
    for r in order:
        n, of = r.n_matched()
        lines.append(f"  {r.preset:<28s} loss {r.loss():6.2f}   {n} of {of} within the floor")
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
    """Ground each mechanism at its measured value — §5b redone without fits."""
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


def run_outer(
    fit_recording: str | Path,
    validation: str | Path,
    policy_bin: str | Path,
    backend: ClosedLoopBackend,
    *,
    trials: int = 10,
    workers: int = 1,
    inner_trials: int = 90,
    max_studies: int = 12,
    out: str | Path | None = None,
) -> None:
    """Nested Optuna: TPE over :class:`OuterPoint`, each trial a full inner fit.

    Seeded with the two known settings. The final number must be quoted on a
    THIRD recording this function never sees — quoting the validation score
    as the result is the one-storey-up overfit this module exists to prevent.
    """
    import optuna

    policy = FreePolicy.load(policy_bin)
    noise, source = shared_floor(validation, policy, backend)
    log: list[dict[str, object]] = []

    def objective(trial: optuna.Trial) -> float:
        point = OuterPoint(
            name=f"trial-{trial.number}",
            stratified=bool(trial.suggest_categorical("stratified", [True, False])),
            normalised=bool(trial.suggest_categorical("normalised", [True, False])),
            w_flight=trial.suggest_float("w_flight", 0.0, 1.0),
        )
        values, res = inner_fit(
            fit_recording,
            point,
            backend,
            workers=workers,
            trials=inner_trials,
            max_studies=max_studies,
        )
        rep = ground_values(
            validation, policy, values, backend, name=point.name, noise=noise, floor_source=source
        )
        n, of = rep.n_matched()
        log.append(
            {
                "trial": trial.number,
                "point": point.__dict__,
                "values": values,
                "inner_stopped": res.stopped,
                "ground_loss": rep.loss(),
                "matched": [n, of],
            }
        )
        if out is not None:
            Path(out).write_text(json.dumps(log, indent=2))
        return rep.loss()

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=0))
    for p in (FROZEN_STYLE, GO2SIM_STYLE):
        study.enqueue_trial(
            {"stratified": p.stratified, "normalised": p.normalised, "w_flight": p.w_flight}
        )
    study.optimize(objective, n_trials=trials)
    best = study.best_trial
    print(f"best outer point: {best.params}  ground loss {best.value:.3f}")
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
    ex.add_argument("--seeds", type=int, default=4)

    ot = sub.add_parser("outer", help="the nested search (EXPENSIVE: trials are inner fits)")
    ot.add_argument("fit_recording")
    ot.add_argument("validation")
    ot.add_argument("policy_bin")
    ot.add_argument("--trials", type=int, default=10)
    ot.add_argument("--workers", type=int, default=1)
    ot.add_argument("--inner-trials", type=int, default=90)
    ot.add_argument("--max-studies", type=int, default=12)
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

    from dimos.robot.unitree.go2.sim.model import MujocoBackend

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
        run_outer(
            args.fit_recording,
            args.validation,
            args.policy_bin,
            backend,
            trials=args.trials,
            workers=args.workers,
            inner_trials=args.inner_trials,
            max_studies=args.max_studies,
            out=args.out,
        )


if __name__ == "__main__":
    main()
