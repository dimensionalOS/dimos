#!/usr/bin/env python3

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

"""Benchmark harness for the ``agent_encode`` autoresearch loop.

Speaks evo's inline instrumentation contract: read ``EVO_RESULT_PATH``,
``EVO_TRACES_DIR`` and ``EVO_EXPERIMENT_ID`` from the environment, write one
JSON object with a numeric ``score``, drop one trace file per case, and exit 0
whenever the measurement itself completed — a candidate encoder that scores
0.0 is a measurement, not a failure. Non-zero exit means the bench could not
measure (missing dataset, unimportable suite) or a gate threshold was breached.

    python -m dimos.evals.tool_evo_bench                    # the benchmark
    python -m dimos.evals.tool_evo_bench --dry-run          # wiring, no spend
    python -m dimos.evals.tool_evo_bench --slice holdout --gate --min-score 0.62
    python -m dimos.evals.tool_evo_bench --write-floors     # record the baseline

The default ``bench`` slice runs the train rows *and* the frozen geometry
suite in one pass, but scores only the families under optimization: the
regression floors then come free from the same run, read back by
``tool_evo_gate floors``. See docs/development/autoresearch.md.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from datetime import datetime, timezone
import importlib
from itertools import zip_longest
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from dimos.evals.types import EvalCase, EvalResult

SUITES = (
    "dimos.evals.suites.go2_pointcloud_clearance",
    "dimos.evals.suites.go2_pointcloud_route",
    "dimos.evals.suites.go2_pointcloud_glass",
    "dimos.evals.suites.go2_pointcloud",
)

SCORED_FAMILIES = ("clearance", "route", "crossing")
"""Families the loop optimizes. Geometry families ride along in the same run
so the regression gate costs no extra model calls, but they cannot buy score."""

SLICE_TAGS: dict[str, frozenset[str]] = {
    "bench": frozenset({"train", "frozen"}),
    "train": frozenset({"train"}),
    "holdout": frozenset({"holdout"}),
    "frozen": frozenset({"frozen"}),
    "spare": frozenset({"spare"}),
    "all": frozenset(),
}

# Suites tag a case with its suite, family, row type and slice; whatever is
# left over is the family.
_NON_FAMILY = frozenset({"pointcloud", "numeric", "mcq", "train", "holdout", "spare", "frozen"})

ARTIFACTS = Path(".evo_bench")  # gitignored; where gates read the last run from
FLOORS = Path(__file__).parent / "evo_floors.json"

EXIT_GATE = 1
EXIT_INFRA = 2


def family_of(case: EvalCase) -> str:
    leftover = case.tags - _NON_FAMILY
    return sorted(leftover)[0] if leftover else "other"


def select(slice_: str, *, limit: int = 0) -> list[EvalCase]:
    """Every case of the four pointcloud suites in this slice, suite order.

    ``limit`` takes cases round-robin across families rather than off the top,
    so a smoke run touches every family and every dataset it would touch for
    real.
    """
    tags = SLICE_TAGS[slice_]
    cases: list[EvalCase] = []
    for name in SUITES:
        suite = importlib.import_module(name).SUITE
        cases += [c for c in suite if not tags or tags & c.tags]
    if not limit or limit >= len(cases):
        return cases
    by_family: dict[str, list[EvalCase]] = {}
    for case in cases:
        by_family.setdefault(family_of(case), []).append(case)
    order = [c for row in zip_longest(*by_family.values()) for c in row if c is not None]
    keep = {c.id for c in order[:limit]}
    return [c for c in cases if c.id in keep]


def means(pairs: Sequence[tuple[str, float]]) -> dict[str, float]:
    """Per-family mean score."""
    totals: dict[str, list[float]] = {}
    for family, score in pairs:
        totals.setdefault(family, []).append(score)
    return {f: sum(v) / len(v) for f, v in sorted(totals.items())}


def pool(families: dict[str, float]) -> float:
    """One number for evo: the mean of the scored families' means.

    Family-weighted, not case-weighted — otherwise 40 already-solved geometry
    rows outvote the 22 glass rows the loop exists to fix.
    """
    scored = {f: s for f, s in families.items() if f in SCORED_FAMILIES}
    chosen = scored or families
    return sum(chosen.values()) / len(chosen) if chosen else 0.0


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _git_sha() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True
        )
        return out.stdout.strip()
    except Exception:
        return ""


def _write_traces(
    results: Sequence[EvalResult], cases: Sequence[EvalCase], families: dict[str, str]
) -> None:
    """One ``task_<id>.json`` per case, for the subagents reading failures.

    Written after the run rather than as each case lands: the runner hands back
    results in one batch, and evo reads traces once the benchmark exits.
    """
    traces = os.environ.get("EVO_TRACES_DIR")
    if not traces:
        return
    directory = Path(traces)
    directory.mkdir(parents=True, exist_ok=True)
    expected = {c.id: getattr(c, "expected", "") for c in cases}
    experiment = os.environ.get("EVO_EXPERIMENT_ID", "unknown")
    for r in results:
        (directory / f"task_{r.case_id}.json").write_text(
            json.dumps(
                {
                    "experiment_id": experiment,
                    "task_id": r.case_id,
                    "score": r.score,
                    "status": "error" if r.error else ("pass" if r.passed else "fail"),
                    "summary": f"[{families.get(r.case_id, '?')}] "
                    f"expected {expected.get(r.case_id, '')!r}, answered {r.outputs[:120]!r}",
                    "failure_reason": r.error,
                    "ended_at": _now(),
                },
                indent=2,
            )
        )


def _publish(payload: dict[str, Any]) -> None:
    """Claim ``EVO_RESULT_PATH`` exclusively, stage, rename — the evo contract."""
    target = os.environ.get("EVO_RESULT_PATH")
    if not target:
        print(json.dumps(payload, indent=2))
        return
    path = Path(target)
    path.parent.mkdir(parents=True, exist_ok=True)
    os.close(os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644))  # claim
    staged = path.with_suffix(path.suffix + ".tmp")
    staged.write_text(json.dumps(payload, indent=2))
    staged.replace(path)


def measure(
    args: argparse.Namespace,
) -> tuple[dict[str, Any], list[EvalResult], list[EvalCase], dict[str, str]]:
    """Run the slice and shape the result payload. Raises on infra failure."""
    cases = select(args.slice, limit=args.limit)
    if not cases:
        raise RuntimeError(f"slice {args.slice!r} selected no cases")
    families = {c.id: family_of(c) for c in cases}
    started = _now()

    if args.dry_run:
        results = [EvalResult(case_id=c.id) for c in cases]
        run_dir = ""
    else:
        from dimos.evals.runner import EvalRunner

        overrides: dict[str, Any] = {"blind": args.blind}
        if args.model:
            overrides["model"] = args.model
        runner = EvalRunner(**overrides)
        results = runner.run(cases)
        run_dir = str(runner.run_dir)

    preflight = [r for r in results if r.error.startswith("preflight:")]
    if preflight:  # datasets or streams missing — the bench cannot measure
        raise RuntimeError(f"{len(preflight)} preflight failures, e.g. {preflight[0].error}")

    # An errored case scores 0. A candidate encoder that raises is a dead
    # branch, not a broken harness, and evo should not spend retries on it.
    per_family = means([(families[r.case_id], r.score) for r in results])
    return (
        {
            "score": pool(per_family),
            "tasks": {r.case_id: r.score for r in results},
            "families": per_family,
            "started_at": started,
            "ended_at": _now(),
            "slice": args.slice,
            "blind": args.blind,
            "errors": sum(1 for r in results if r.error),
            "n": len(results),
            "run_dir": run_dir,
            "git": _git_sha(),
        },
        results,
        cases,
        families,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slice", choices=sorted(SLICE_TAGS), default="bench")
    parser.add_argument("--limit", type=int, default=0, help="run at most N cases")
    parser.add_argument("--model", default="", help="override the chat model")
    parser.add_argument("--blind", action="store_true", help="withhold observations")
    parser.add_argument("--dry-run", action="store_true", help="select cases, score 0, no spend")
    parser.add_argument("--gate", action="store_true", help="never write the evo result artifact")
    parser.add_argument("--min-score", type=float, default=None, help="exit 1 below this")
    parser.add_argument("--max-score", type=float, default=None, help="exit 1 above this")
    parser.add_argument(
        "--write-floors", action="store_true", help=f"record this run's families in {FLOORS.name}"
    )
    args = parser.parse_args(argv)

    try:
        payload, results, cases, families = measure(args)
    except Exception as e:
        print(f"BENCH INFRA FAILURE: {e!r}", file=sys.stderr)
        return EXIT_INFRA

    if not args.gate:
        _write_traces(results, cases, families)
        _publish(payload)
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    (ARTIFACTS / f"{args.slice}.json").write_text(json.dumps(payload, indent=2))

    for family, score in payload["families"].items():
        marker = " " if family in SCORED_FAMILIES else "."
        print(f"{marker} {family:10} {score:.3f}")
    print(
        f"score {payload['score']:.4f} | slice {args.slice} | {payload['n']} cases "
        f"| {payload['errors']} errors | {payload['run_dir']}"
    )

    if args.write_floors:
        FLOORS.write_text(
            json.dumps(
                {
                    "note": "per-family floors for tool_evo_gate floors; regenerate with "
                    "tool_evo_bench --write-floors on a known-good encoder",
                    "families": payload["families"],
                    "score": payload["score"],
                    "slice": args.slice,
                    "model": args.model or "default",
                    "git": payload["git"],
                    "recorded_at": payload["ended_at"],
                },
                indent=2,
            )
            + "\n"
        )
        print(f"wrote {FLOORS}")

    if args.min_score is not None and payload["score"] < args.min_score:
        print(f"GATE FAIL: {payload['score']:.4f} < {args.min_score}", file=sys.stderr)
        return EXIT_GATE
    if args.max_score is not None and payload["score"] > args.max_score:
        print(f"GATE FAIL: {payload['score']:.4f} > {args.max_score}", file=sys.stderr)
        return EXIT_GATE
    return 0


if __name__ == "__main__":
    sys.exit(main())
