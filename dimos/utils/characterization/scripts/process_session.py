# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""CLIs for the post-collection processing pipeline.

Five entry points; each takes a session directory (or pair of dir lists)
and emits a derived artifact alongside the raw data — never mutating
existing files (with the one documented exception of appending
``noise_floor`` to each ``run.json``).

Pipeline order:

    1. python -m dimos.utils.characterization.scripts.process_session validate      <session>             # validation.json + noise_floor in run.json
    2. python -m dimos.utils.characterization.scripts.analyze_run           <run> ... (existing)  # metrics.json + plot.svg per run
    3. python -m dimos.utils.characterization.scripts.process_session aggregate     <session>             # session_summary.json + .csv
    4. python -m dimos.utils.characterization.scripts.process_session deadtime        <session>             # deadtime_stats.json (E8 only)
    5. python -m dimos.utils.characterization.scripts.process_session coupling        <session>             # coupling_stats.json (E7 only)
    6. python -m dimos.utils.characterization.scripts.process_session envelope       <session> [<session> ...] --mode default|rage
    7. python -m dimos.utils.characterization.scripts.process_session compare-modes         --default <s> <s> ... --rage <s> <s> ...

Steps 1–3 should be run on every session. 4–7 are downstream of those.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)


def main_validate(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session validate`` — per-run validation + noise floor for one session."""
    parser = argparse.ArgumentParser(description="Validate every run in a session and compute noise floors.")
    parser.add_argument("session_dir")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.utils.characterization.processing.noise import compute_session
    from dimos.utils.characterization.processing.validate import validate_session

    s = Path(args.session_dir)
    print(f"validating: {s}")
    v = validate_session(s)
    print(f"  passed: {v['passed']}/{v['total_runs']}  failed: {v['failed']}")
    for f in v["failures"]:
        print(f"    {f['run_id']}: {','.join(f['failed_checks'])}")

    print("computing noise floors...")
    n = compute_session(s)
    n_with_floor = sum(1 for r in n["runs"] if r.get("vx_std") is not None)
    print(f"  noise floor computed for {n_with_floor}/{len(n['runs'])} runs")
    print(f"  artifacts: {s / 'validation_summary.json'}")
    print(f"             noise_floor appended to each run.json")
    return 0


def main_aggregate(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session aggregate`` — per-recipe metric aggregation for one session."""
    parser = argparse.ArgumentParser(description="Aggregate per-recipe metrics across runs in a session.")
    parser.add_argument("session_dir")
    parser.add_argument("--sigma", type=float, default=2.0,
                        help="Reject runs whose steady_state is > N σ from group mean (default: 2.0)")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.processing.aggregate import aggregate_session
    s = Path(args.session_dir)
    summary = aggregate_session(s, sigma=args.sigma)
    print(f"session: {s}")
    print(f"groups: {summary['n_recipes']}  total runs: {summary['n_runs_total']}")
    for g in summary["groups"]:
        ss = g["metrics"]["steady_state"]
        print(
            f"  {g['recipe']}: kept={g['n_runs_kept']}/{g['n_runs_total']}  "
            f"steady={ss['mean']}±{ss['std']}  rejected={g['rejected_run_ids'] or '-'}"
        )
    print(f"artifacts: {s / 'session_summary.json'}, {s / 'session_summary.csv'}")
    return 0


def main_deadtime(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session deadtime`` — E8 deadtime distribution for one session."""
    parser = argparse.ArgumentParser(description="Compute deadtime statistics for E8-style runs.")
    parser.add_argument("session_dir")
    parser.add_argument("--threshold-k", type=float, default=3.0,
                        help="Onset threshold = K × σ_noise (default: 3.0)")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.processing.deadtime import deadtime_stats_session
    s = Path(args.session_dir)
    out = deadtime_stats_session(s, threshold_k=args.threshold_k)
    summary = out["summary"]
    if summary.get("n", 0) == 0:
        print(f"no usable E8 deadtime data in {s}")
        return 1
    print(f"session: {s}")
    print(f"n={summary['n']}  threshold_k={args.threshold_k}")
    print(f"  mean={summary['mean_s']}s  median={summary['median_s']}s")
    print(f"  p5={summary['p5_s']}s  p95={summary['p95_s']}s")
    print(f"  jitter σ={summary['std_s']}s  range=[{summary['min_s']}, {summary['max_s']}]s")
    print(f"artifact: {s / 'deadtime_stats.json'}")
    return 0


def main_coupling(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session coupling`` — E7 cross-coupling for one session."""
    parser = argparse.ArgumentParser(description="Compute cross-coupling for E7-style runs.")
    parser.add_argument("session_dir")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.processing.coupling import coupling_stats_session
    s = Path(args.session_dir)
    out = coupling_stats_session(s)
    print(f"session: {s}")
    print(f"n_runs={out['n_runs']}  overall decision: {out['overall_decision']}")
    for g in out["groups"]:
        print(
            f"  {g['recipe']}: n={g['n_runs']}  decision={g['decision']}  "
            f"leak%={g['leak_pct_mean']}"
        )
    print(f"artifact: {s / 'coupling_stats.json'}")
    return 0


def main_envelope(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session envelope`` — operational envelope markdown for one mode."""
    parser = argparse.ArgumentParser(description="Build per-mode operational envelope markdown.")
    parser.add_argument("session_dirs", nargs="+",
                        help="One or more session directories (same mode)")
    parser.add_argument("--mode", default="default",
                        help="Mode label for the report (default: 'default'; use 'rage' for rage sessions)")
    parser.add_argument("--out", default=None,
                        help="Output markdown path (default: ./envelope_<mode>.md)")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.processing.envelope import envelope_report
    out = Path(args.out) if args.out else Path(f"./envelope_{args.mode}.md")
    md = envelope_report([Path(s) for s in args.session_dirs], mode_label=args.mode, out_path=out)
    print(f"wrote {out}  ({len(md)} chars)")
    return 0


def main_fit(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session fit`` — FOPDT plant-model fit for one session."""
    parser = argparse.ArgumentParser(description="Fit FOPDT plant model to step recipes in one session.")
    parser.add_argument("session_dir")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip overlay / parameter plots (faster; markdown + JSON only)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.utils.characterization.modeling.session import fit_session
    s = Path(args.session_dir)
    print(f"fitting FOPDT: {s}")
    summary = fit_session(s, write_plots_enabled=not args.no_plots)

    print(f"  mode: {summary['mode']}")
    for channel, ch in sorted(summary.get("channels", {}).items()):
        pooled = ch.get("pooled", {})
        K = pooled.get("K") or {}
        tau = pooled.get("tau") or {}
        L = pooled.get("L") or {}
        asym = "asymmetric" if ch.get("direction_asymmetric") else "symmetric"
        print(
            f"  {channel}: K={K.get('mean')} τ={tau.get('mean')} L={L.get('mean')} "
            f"({asym}, n_groups_K={K.get('n_groups')})"
        )
    out_dir = s / "modeling"
    print(f"artifacts: {out_dir / 'model_summary.json'}, {out_dir / 'model_report.md'}, "
          f"{out_dir / 'plots'}/")
    return 0


def main_fit_all(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session fit-all`` — batch FOPDT fits for every session under a parent dir."""
    parser = argparse.ArgumentParser(description="Fit FOPDT plant model to every session under a parent directory.")
    parser.add_argument("parent_dir",
                        help="Directory containing session_* subdirs (e.g. ~/char_runs)")
    parser.add_argument("--force", action="store_true",
                        help="Refit sessions that already have model_summary.json")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip overlay / parameter plots (faster)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.utils.characterization.modeling.session import fit_all_sessions
    parent = Path(args.parent_dir).expanduser()
    print(f"fitting every session under: {parent}")
    index = fit_all_sessions(
        parent, force=args.force, write_plots_enabled=not args.no_plots,
    )
    print(f"  {index['n_sessions']} sessions discovered")
    print()
    print(f"  {'session':<32} {'mode':<8} {'runs':>5} {'fits':>5}  status")
    for row in index["sessions"]:
        print(
            f"  {row['session']:<32} {row['mode']:<8} "
            f"{row['n_runs']:>5} {row['n_groups_with_fit']:>5}  {row['status']}"
        )
    print()
    print(f"index: {parent / 'models_index.json'}")
    return 0


def main_tune(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session tune`` — lambda-tune PI gains for each channel.

    Loads ``model_summary.json`` from the session, runs a multiplier sweep
    (λ ∈ [τ, 3τ]) per channel through a battery of reference signals,
    picks the lowest-cost candidate, runs robustness sweeps on K (±15%)
    and on τ / L (±20%), and writes ``tuning_summary.json`` +
    ``tuning_report.md`` + plots under ``<session>/modeling/tuning/``.
    """
    parser = argparse.ArgumentParser(
        description="Lambda-tune PI controller gains per channel.")
    parser.add_argument("session_dir")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip step / sweep plots (faster)")
    parser.add_argument("--control-rate-hz", type=float, default=50.0,
                        help="Discrete control rate (default 50 Hz)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.utils.characterization.modeling.tune_session import tune_session
    s = Path(args.session_dir)
    print(f"tuning PI gains: {s}")
    out = tune_session(
        s,
        write_plots_enabled=not args.no_plots,
        control_dt_s=1.0 / args.control_rate_hz,
    )
    for channel, ch in sorted((out.get("channels") or {}).items()):
        if "skip_reason" in ch:
            print(f"  {channel}: SKIPPED - {ch['skip_reason']}")
            continue
        gains = (ch.get("tuning") or {}).get("best_gains") or {}
        print(
            f"  {channel}: verdict={ch.get('verdict','?').upper():<8s} "
            f"Kp={gains.get('Kp', 0):.3f}, Ki={gains.get('Ki', 0):.3f}, "
            f"Kt={gains.get('Kt', 0):.3f} (λ-mult={gains.get('multiplier', 0):.2f})"
        )
    out_dir = s / "modeling" / "tuning"
    print(f"artifacts: {out_dir / 'tuning_report.md'}, "
          f"{out_dir / 'tuning_summary.json'}, {out_dir / 'plots'}/")
    return 0


def main_validate_model(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session validate-model`` — direction-holdout validation of the FOPDT fit.

    Refits the model on forward-direction runs only, then predicts the
    held-out reverse-direction runs and reports per-channel pass/marginal/
    fail verdicts. Outputs land in ``<session>/modeling/validation/``.
    """
    parser = argparse.ArgumentParser(
        description="Validate FOPDT fit on held-out reverse-direction runs.")
    parser.add_argument("session_dir")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip overlay/distribution plots (faster)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.utils.characterization.modeling.validate_session import (
        validate_session_direction_holdout,
    )
    s = Path(args.session_dir)
    print(f"validating FOPDT model: {s}")
    summary = validate_session_direction_holdout(
        s, write_plots_enabled=not args.no_plots,
    )
    print(f"  mode: {summary.get('mode')}")
    print(f"  trained on {summary.get('n_train_runs_forward', 0)} forward runs; "
          f"validated on {summary.get('n_validate_runs_reverse', 0)} reverse runs")
    for channel, cs in sorted((summary.get("channels") or {}).items()):
        rn = (cs.get("rise_norm_rmse") or {}).get("median")
        fn = (cs.get("fall_norm_rmse") or {}).get("median") if cs.get("fall_norm_rmse") else None
        rn_str = f"{rn:.1%}" if isinstance(rn, float) else "—"
        fn_str = f"{fn:.1%}" if isinstance(fn, float) else "—"
        print(
            f"  {channel}: {cs.get('verdict','?').upper():<8} "
            f"({cs.get('n_pass',0)}/{cs.get('n_total',0)} pass; "
            f"rise nRMSE med={rn_str}, fall nRMSE med={fn_str})"
        )
    out_dir = s / "modeling" / "validation"
    print(f"artifacts: {out_dir / 'validation_report.md'}, "
          f"{out_dir / 'validation_summary.json'}, {out_dir / 'plots'}/")
    return 0


def main_compare_models(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session compare-models`` — default vs rage FOPDT comparison.

    Accepts multiple sessions per mode; pools RunFits across all sessions
    on each side before comparing.
    """
    parser = argparse.ArgumentParser(description="Compare default-mode and rage-mode FOPDT models (pooled across sessions).")
    parser.add_argument("--default", nargs="+", required=True,
                        help="Default-mode session dir(s)")
    parser.add_argument("--rage", nargs="+", required=True,
                        help="Rage-mode session dir(s)")
    parser.add_argument("--out", default="./model_compare.md")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.modeling.session import compare_pooled
    verdict = compare_pooled(
        [Path(p) for p in args.default],
        [Path(p) for p in args.rage],
        out_path=Path(args.out),
    )
    print(f"wrote {args.out}")
    for channel, cv in sorted(verdict.get("channels", {}).items()):
        print(f"  {channel}: {cv.get('verdict')}")
    return 0


def main_compare_modes(argv: list[str] | None = None) -> int:
    """``python -m dimos.utils.characterization.scripts.process_session compare-modes`` — default vs rage comparison."""
    parser = argparse.ArgumentParser(description="Compare default-mode and rage-mode session metrics.")
    parser.add_argument("--default", nargs="+", required=True,
                        help="Default-mode session dirs")
    parser.add_argument("--rage", nargs="+", required=True,
                        help="Rage-mode session dirs")
    parser.add_argument("--out", default="./compare_default_vs_rage.md")
    args = parser.parse_args(argv)

    from dimos.utils.characterization.processing.compare_modes import compare_modes
    md = compare_modes(
        [Path(p) for p in args.default],
        [Path(p) for p in args.rage],
        out_path=Path(args.out),
    )
    print(f"wrote {args.out}  ({len(md)} chars)")
    return 0


_SUBCOMMANDS: dict[str, callable] = {
    "validate": main_validate,
    "aggregate": main_aggregate,
    "deadtime": main_deadtime,
    "coupling": main_coupling,
    "envelope": main_envelope,
    "compare-modes": main_compare_modes,
    "fit": main_fit,
    "fit-all": main_fit_all,
    "validate-model": main_validate_model,
    "tune": main_tune,
    "compare-models": main_compare_models,
}


def main(argv: list[str] | None = None) -> int:
    """Subcommand dispatcher for ``python -m dimos.utils.characterization.scripts.process_session``.

    Usage:
        python -m dimos.utils.characterization.scripts.process_session <subcommand> [args]

    Subcommands: validate, aggregate, deadtime, coupling, envelope, compare-modes.
    """
    import sys
    args = sys.argv[1:] if argv is None else argv
    if not args or args[0] in {"-h", "--help"}:
        print("Usage: python -m dimos.utils.characterization.scripts.process_session <subcommand> [args]")
        print(f"Subcommands: {', '.join(_SUBCOMMANDS)}")
        return 0 if args else 2
    sub, rest = args[0], args[1:]
    if sub not in _SUBCOMMANDS:
        print(f"Unknown subcommand: {sub!r}. Available: {', '.join(_SUBCOMMANDS)}")
        return 2
    return _SUBCOMMANDS[sub](rest)


if __name__ == "__main__":
    raise SystemExit(main())
