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

"""Search leg-joint physics for the values that make sim behave like hardware.

    python -m dimos.navigation.motion.simulation.search DATASET POLICY

Uses Optuna's CMA-ES sampler. The objective is continuous, low-dimensional and
noisy, which is what CMA-ES is built for; TPE is the better default when
parameters are categorical or conditional, and neither applies here.

The noise floor is measured once and reused for every trial. It costs four
extra rollouts, and holding it fixed is what makes losses comparable between
trials -- recomputing it per trial would let a trial win by getting noisier.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.navigation.motion.simulation.evaluate import (
    LEG_STATS,
    evaluate,
    measure_noise,
)

# name -> (low, high, log). Menagerie defaults are armature 0.01, damping 2.0,
# frictionloss 0.2; the ranges bracket them by about a decade each way.
SPACE: dict[str, tuple[float, float, bool]] = {
    "armature": (0.001, 0.2, True),
    # Floor at 0.05: the previous front crowded the old 0.2 floor, and a bound
    # a search leans on is a bound hiding the optimum.
    "damping": (0.05, 10.0, True),
    "frictionloss": (0.0, 2.0, False),
    # Not a physics property: how long the command takes to reach the policy on
    # hardware. Genuine transport latency only -- the streams now share an
    # epoch (see vive.read_vive_pose), and epoch-corrected cross-correlation
    # puts the real turn lag at ~0.17 s, so 0.3 brackets it comfortably. The
    # old 0.8 ceiling existed to cover a 0.31 s stream-misalignment artifact.
    "command_delay": (0.0, 0.3, False),
    # Competing explanation for the same lag: a heavier / more rotationally
    # sluggish trunk. Inertia delays and smooths the response; transport delay
    # only shifts it. Letting both into one space lets the data choose.
    "trunk_mass_scale": (0.6, 2.0, False),
    "trunk_inertia_scale": (0.4, 4.0, True),
    # Motor time constant. A MuJoCo motor delivers the requested torque on the
    # same step; a real BLDC through a gearbox takes a few milliseconds. The
    # Pareto front says no scalar on an existing term can match gait and
    # rotation at once, which points at a missing mechanism like this one.
    "actuator_tau": (0.0, 0.05, False),
    # Foot-floor contact friction; menagerie ships 0.8 tangential and 0.02
    # torsional (go2.xml, priority 1, condim 6). Torsional is what the stance
    # feet pivot against in a turn, and neither value is measured.
    "foot_friction": (0.3, 1.5, False),
    "foot_friction_torsional": (0.002, 0.2, True),
    # Payload *placement*: the lidar and mounts sit forward/top of the trunk,
    # which a pure mass scale at the stock com cannot express. Metres.
    "trunk_com_x": (-0.06, 0.06, False),
    # Covers and cabling the MJCF omits; swing inertia shapes how high a foot
    # flies for a given action.
    "leg_mass_scale": (0.7, 2.0, False),
}

# Statistics grouped into a few objectives. Seven separate objectives would make
# almost every trial non-dominated -- NSGA-II degrades badly past three or four
# -- and these three are the ones that actually trade against each other: the
# physics-only search bought gait accuracy with friction, the delay search bought
# rotation accuracy and gave gait back.
OBJECTIVES: dict[str, tuple[str, ...]] = {
    # tilt_p99 is the stability tail -- a config that occasionally almost
    # falls pays here even when its oscillation statistics look right.
    "gait": ("gait_hz", "height_std", "pitch_std", "roll_std", "tilt_p99"),
    "translation": ("speed", "speed_gain", "speed_lag"),
    "rotation": ("yaw_rate_gain", "yaw_lag"),
    # Scored against policy/lowcmd: the base statistics cannot see the legs,
    # and the sim matched all of them while high-stepping its front feet.
    "legs": LEG_STATS,
}


# Statistics a recording cannot measure honestly, per recording stem. On v11
# the body-bob gait_hz locks onto the sway/height envelope rather than the
# steps (1.0 Hz against 2.9 Hz in the joint commands -- cmd_gait_hz is the
# valid one there), and height_std is dominated by the commanded crouches.
INVALID_STATS: dict[str, frozenset[str]] = {
    "unitree_v11_gait_height01": frozenset({"gait_hz", "height_std"}),
}


def _invalid_for(dataset: str | Path) -> frozenset[str]:
    return INVALID_STATS.get(Path(dataset).stem, frozenset())


def usable_floor(
    raw: dict[str, float], report: Any, cross: dict[str, float] | None = None
) -> dict[str, float]:
    """Clamp a noise floor that collapsed, so a resolved statistic cannot dominate.

    The floor is a *resolution limit*, and a well-stabilized policy can drive
    it to ~0 -- v11 does, which sends its SNRs to infinity and makes the loss
    meaningless. Clamping to the same statistic's floor on another recording,
    plus 5% of the real value, keeps every term finite and comparable.
    """
    real = {**report.real.as_dict(), **report.real_legs}
    cross = cross or {}
    return {
        k: max(v, cross.get(k, 0.0), 0.05 * abs(real.get(k, 0.0)), 1e-4) for k, v in raw.items()
    }


def joint_loss(
    runs: list[tuple[str | Path, str | Path]],
    floors: list[dict[str, float]],
    physics: dict[str, float] | None,
    command_delay: float,
    actuator_tau: float,
    start: float = 6.0,
) -> tuple[float, list[Any]]:
    """One loss over several recordings: RMS of every valid SNR from all of them.

    Fitting one recording leaves the weakly-identified parameters free to
    absorb that run's own style -- which is exactly what the v11 held-out
    check caught. Scoring every trial on both recordings is what separates
    platform from policy.
    """
    reports, snrs = [], []
    for (dataset, policy_bin), floor in zip(runs, floors, strict=True):
        report = evaluate(
            dataset,
            policy_bin,
            start=start,
            physics=physics,
            command_delay=command_delay,
            actuator_tau=actuator_tau,
            noise=floor,
        )
        reports.append(report)
        invalid = _invalid_for(dataset)
        snrs += [v for k, v in report.snr().items() if k not in invalid and np.isfinite(v)]
    return float(np.sqrt(np.mean([v * v for v in snrs]))), reports


def run_joint(
    runs: list[tuple[str | Path, str | Path]],
    *,
    trials: int = 300,
    start: float = 6.0,
    seeds: int = 4,
    space: dict[str, tuple[float, float, bool]] | None = None,
    seed_params: dict[str, float] | None = None,
    storage: str | None = None,
    study_name: str = "go2-joint",
) -> dict[str, Any]:
    """Fit one physics configuration against several recordings at once.

    ``seed_params`` is enqueued as trial 0, so a known-good configuration is
    the bar the search has to clear rather than a point it has to rediscover.
    """
    import optuna

    space = space or SPACE
    floors: list[dict[str, float]] = []
    raws: list[dict[str, float]] = []
    for dataset, policy_bin in runs:
        raw = measure_noise(dataset, policy_bin, start=start, seeds=seeds)
        base = evaluate(dataset, policy_bin, start=start, noise=raw)
        # Cross-clamp against the first recording's floor: the same statistic
        # on the same simulator cannot really be orders of magnitude sharper.
        floors.append(usable_floor(raw, base, raws[0] if raws else None))
        raws.append(raw)

    def objective(trial: optuna.Trial) -> float:
        params = {
            name: trial.suggest_float(name, lo, hi, log=log)
            for name, (lo, hi, log) in space.items()
        }
        delay = params.pop("command_delay", 0.0)
        tau = params.pop("actuator_tau", 0.0)
        loss, reports = joint_loss(runs, floors, params, delay, tau, start)
        for run_i, report in enumerate(reports):
            for key, value in report.snr().items():
                trial.set_user_attr(f"{run_i}:{key}", value)
        return loss

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.CmaEsSampler(seed=0),
        storage=storage,
        study_name=study_name,
        load_if_exists=storage is not None,
    )
    if seed_params:
        study.enqueue_trial(dict(seed_params))
    study.optimize(objective, n_trials=trials, show_progress_bar=True)

    best = dict(study.best_params)
    best_delay = best.pop("command_delay", 0.0)
    best_tau = best.pop("actuator_tau", 0.0)
    _loss, reports = joint_loss(runs, floors, best, best_delay, best_tau, start)
    return {
        "best_loss": study.best_value,
        "best_params": study.best_params,
        "tables": [r.table() for r in reports],
    }


def _objective_values(snr: dict[str, float], groups: dict[str, tuple[str, ...]]) -> list[float]:
    """Root-mean-square SNR within each group, so groups of different size compare."""
    return [
        float(np.sqrt(np.mean([snr[k] ** 2 for k in keys if k in snr]))) for keys in groups.values()
    ]


def run(
    dataset: str | Path,
    policy_bin: str | Path,
    *,
    trials: int = 100,
    start: float = 6.0,
    seconds: float | None = None,
    seeds: int = 4,
    space: dict[str, tuple[float, float, bool]] | None = None,
    storage: str | None = None,
    study_name: str = "go2-physics",
) -> dict[str, Any]:
    """Minimize the noise-weighted sim-vs-real loss over physics parameters."""
    import optuna

    space = space or SPACE
    noise = measure_noise(dataset, policy_bin, start=start, seconds=seconds, seeds=seeds)

    baseline = evaluate(dataset, policy_bin, start=start, seconds=seconds, noise=noise)

    def objective(trial: optuna.Trial) -> float:
        params = {
            name: trial.suggest_float(name, lo, hi, log=log)
            for name, (lo, hi, log) in space.items()
        }
        delay = params.pop("command_delay", 0.0)
        tau = params.pop("actuator_tau", 0.0)
        report = evaluate(
            dataset,
            policy_bin,
            start=start,
            seconds=seconds,
            physics=params,
            command_delay=delay,
            actuator_tau=tau,
            noise=noise,
        )
        for key, value in report.snr().items():
            trial.set_user_attr(key, value)
        return report.loss()

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.CmaEsSampler(seed=0),
        storage=storage,
        study_name=study_name,
        load_if_exists=storage is not None,
    )
    study.optimize(objective, n_trials=trials, show_progress_bar=True)

    best_params = dict(study.best_params)
    best_delay = best_params.pop("command_delay", 0.0)
    best_tau = best_params.pop("actuator_tau", 0.0)
    best = evaluate(
        dataset,
        policy_bin,
        start=start,
        seconds=seconds,
        physics=best_params,
        command_delay=best_delay,
        actuator_tau=best_tau,
        noise=noise,
    )
    return {
        "baseline_loss": baseline.loss(),
        "best_loss": study.best_value,
        "best_params": study.best_params,
        "baseline_table": baseline.table(),
        "best_table": best.table(),
    }


def run_multi(
    dataset: str | Path,
    policy_bin: str | Path,
    *,
    trials: int = 200,
    start: float = 6.0,
    seconds: float | None = None,
    seeds: int = 4,
    space: dict[str, tuple[float, float, bool]] | None = None,
    groups: dict[str, tuple[str, ...]] | None = None,
) -> dict[str, Any]:
    """Multi-objective search; returns the Pareto front rather than one winner.

    A single scalar loss hides the trade: gait accuracy and rotation accuracy
    pull the parameters in different directions, and averaging them just picks
    an arbitrary point on that curve. NSGA-II keeps the whole curve.
    """
    import optuna

    space = space or SPACE
    groups = groups or OBJECTIVES
    noise = measure_noise(dataset, policy_bin, start=start, seconds=seconds, seeds=seeds)

    def objective(trial: optuna.Trial) -> tuple[float, ...]:
        params = {
            name: trial.suggest_float(name, lo, hi, log=log)
            for name, (lo, hi, log) in space.items()
        }
        delay = params.pop("command_delay", 0.0)
        tau = params.pop("actuator_tau", 0.0)
        report = evaluate(
            dataset,
            policy_bin,
            start=start,
            seconds=seconds,
            physics=params,
            command_delay=delay,
            actuator_tau=tau,
            noise=noise,
        )
        snr = report.snr()
        for key, value in snr.items():
            trial.set_user_attr(key, value)
        return tuple(_objective_values(snr, groups))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        directions=["minimize"] * len(groups),
        sampler=optuna.samplers.NSGAIISampler(seed=0),
    )
    study.optimize(objective, n_trials=trials, show_progress_bar=True)

    front = sorted(
        (
            {
                "objectives": dict(zip(groups, t.values, strict=True)),
                "params": t.params,
                "snr": {k: v for k, v in t.user_attrs.items()},
            }
            for t in study.best_trials
        ),
        key=lambda row: sum(row["objectives"].values()),
    )
    return {"groups": list(groups), "front": front, "n_trials": trials}


def format_front(result: dict[str, Any], limit: int = 12) -> str:
    groups = result["groups"]
    head = "  ".join(f"{g:>11}" for g in groups)
    lines = [
        f"Pareto front ({len(result['front'])} of {result['n_trials']} trials)",
        f"{head}   parameters",
    ]
    for row in result["front"][:limit]:
        vals = "  ".join(f"{row['objectives'][g]:11.2f}" for g in groups)
        params = " ".join(f"{k}={v:.4g}" for k, v in sorted(row["params"].items()))
        lines.append(f"{vals}   {params}")
    if len(result["front"]) > limit:
        lines.append(f"... {len(result['front']) - limit} more")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(prog="motion.simulation.search")
    ap.add_argument("dataset")
    ap.add_argument("policy")
    ap.add_argument("--trials", type=int, default=100)
    ap.add_argument("--start", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=None, help="default: the whole recording")
    ap.add_argument("--storage", default=None, help="e.g. sqlite:///search.db to resume")
    ap.add_argument("--json", default="", help="write the result here")
    ap.add_argument(
        "--multi",
        action="store_true",
        help="multi-objective (NSGA-II); print the Pareto front instead of one winner",
    )
    ap.add_argument(
        "--also",
        nargs=2,
        metavar=("DATASET", "POLICY"),
        action="append",
        default=[],
        help="fit against this recording too (repeatable); scores every trial "
        "on all of them, which is what stops one run's style being absorbed",
    )
    ap.add_argument(
        "--seed-fitted",
        action="store_true",
        help="start from the 'fitted' preset, so the search must beat it",
    )
    ap.add_argument(
        "--seed-preset",
        default=None,
        help="start from this preset instead: a built-in name or a preset JSON",
    )
    ap.add_argument(
        "--save-preset",
        default=None,
        help="write the winner as a NEW named preset, <name>.json. Built-in "
        "names are refused -- a validated tune is never overwritten by a search",
    )
    args = ap.parse_args()

    from dimos.navigation.motion.simulation.evaluate import Preset, load_preset

    def save_winner(params: dict[str, float]) -> None:
        """Persist a search result as a preset that stands next to the built-ins."""
        if not args.save_preset:
            return
        out = Preset.from_params(args.save_preset, params).save(f"{args.save_preset}.json")
        print(f"\nwrote preset {args.save_preset!r} -> {out}")
        print(f"run it with:  --preset {out}")

    if args.also:
        seed = (
            load_preset(args.seed_preset).params()
            if (args.seed_fitted or args.seed_preset)
            else None
        )
        result = run_joint(
            [(args.dataset, args.policy), *(tuple(a) for a in args.also)],  # type: ignore[list-item]
            trials=args.trials,
            start=args.start,
            seed_params=seed,
            storage=args.storage,
        )
        for table in result["tables"]:
            print(f"\n{table}")
        print(f"\njoint loss {result['best_loss']:.3f}")
        print("params:", json.dumps(result["best_params"], indent=2))
        if args.json:
            Path(args.json).write_text(json.dumps(result, indent=2))
        save_winner(result["best_params"])
        return

    if args.multi:
        result = run_multi(
            args.dataset,
            args.policy,
            trials=args.trials,
            start=args.start,
            seconds=args.seconds,
        )
        print(format_front(result))
        if args.json:
            Path(args.json).write_text(json.dumps(result, indent=2))
        return

    result = run(
        args.dataset,
        args.policy,
        trials=args.trials,
        start=args.start,
        seconds=args.seconds,
        storage=args.storage,
    )
    print("\n=== baseline ===")
    print(result["baseline_table"])
    print("\n=== best ===")
    print(result["best_table"])
    print(f"\nloss {result['baseline_loss']:.2f} -> {result['best_loss']:.2f}")
    print("params:", json.dumps(result["best_params"], indent=2))
    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=2))
    save_winner(result["best_params"])


if __name__ == "__main__":
    main()
