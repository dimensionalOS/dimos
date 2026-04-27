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
