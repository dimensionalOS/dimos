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
    Report,
    ground,
    real_summary,
    sim_noise,
    usable_floor,
)
from dimos.robot.unitree.go2.sim.sysid.ingest import read_declarations, read_streams
from dimos.robot.unitree.go2.sim.sysid.regimes import Segment, regimes, sample_segments
from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts


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
    from dimos.robot.unitree.go2.sim.model import MujocoBackend

    backend = MujocoBackend()
    base = base_values("measured")
    plan = default_plan(backend.knobs())
    declared = read_declarations(recording)
    rollouts = Rollouts(recording, workers=workers)
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
        st, policy, preset, start=start, noise=noise, floor_source=floor_source, with_ghost=False
    )


def shared_floor(
    validation: str | Path,
    policy: FreePolicy,
    *,
    baseline: str = "measured",
    seeds: int = 4,
    start: float = 6.0,
) -> tuple[dict[str, float], str]:
    """ONE noise floor for every candidate, measured on the baseline plant.

    Per-candidate floors would let a plant buy small SNRs with its own chaos;
    a shared floor keeps the comparison about the physics. Measured on the
    incumbent and clamped (:func:`usable_floor`) — swap in
    :func:`~dimos.robot.unitree.go2.sim.sysid.ground.robot_noise` when the
    repeat recording exists.
    """
    st = read_streams(validation)
    base = load_preset(baseline)
    raw = sim_noise(st, policy, base, seeds=seeds, start=start)
    real = real_summary(st, start=start, seconds=float(st.wt[-1]) - start)
    return usable_floor(raw, real.as_dict()), f"sim-perturb ({baseline} plant, shared)"


# ------------------------------------------------------------ the experiment


def experiment(
    recording: str | Path,
    policy_bin: str | Path,
    presets: list[str],
    *,
    start: float = 6.0,
    seeds: int = 4,
) -> str:
    """Ground each plant under one shared floor and say which the referee prefers.

    THE FIRST DELIVERABLE, and it is an experiment, not a feature: the frozen
    scorer's winner (``accel``) and this package's (``measured``, plus the
    phase-4 fitted point) disagree; whichever plant the grounding prefers
    tells us which scorer survives into the outer study's seed.
    """
    policy = FreePolicy.load(policy_bin)
    noise, source = shared_floor(recording, policy, seeds=seeds, start=start)
    st = read_streams(recording)
    reports: list[Report] = []
    lines: list[str] = [
        f"GROUNDING  {Path(str(recording)).name}  floor: {source}",
        "",
    ]
    for name in presets:
        p = load_preset(name)
        rep = ground(st, policy, p, start=start, noise=noise, floor_source=source, with_ghost=False)
        reports.append(rep)
        lines += [rep.table(), ""]
    order = sorted(reports, key=lambda r: r.loss())
    lines.append("VERDICT  (loss = RMS over SNRs; lower = closer to the real run)")
    for r in order:
        n, of = r.n_matched()
        lines.append(f"  {r.preset:<28s} loss {r.loss():6.2f}   {n} of {of} within the floor")
    return "\n".join(lines)


# ------------------------------------------------------------ the outer study


def run_outer(
    fit_recording: str | Path,
    validation: str | Path,
    policy_bin: str | Path,
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
    noise, source = shared_floor(validation, policy)
    log: list[dict[str, object]] = []

    def objective(trial: optuna.Trial) -> float:
        point = OuterPoint(
            name=f"trial-{trial.number}",
            stratified=bool(trial.suggest_categorical("stratified", [True, False])),
            normalised=bool(trial.suggest_categorical("normalised", [True, False])),
            w_flight=trial.suggest_float("w_flight", 0.0, 1.0),
        )
        values, res = inner_fit(
            fit_recording, point, workers=workers, trials=inner_trials, max_studies=max_studies
        )
        rep = ground_values(
            validation, policy, values, name=point.name, noise=noise, floor_source=source
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

    args = ap.parse_args()
    if args.cmd == "experiment":
        print(
            experiment(
                args.recording,
                args.policy_bin,
                args.presets,
                start=args.start,
                seeds=args.seeds,
            )
        )
    else:
        run_outer(
            args.fit_recording,
            args.validation,
            args.policy_bin,
            trials=args.trials,
            workers=args.workers,
            inner_trials=args.inner_trials,
            max_studies=args.max_studies,
            out=args.out,
        )


if __name__ == "__main__":
    main()
