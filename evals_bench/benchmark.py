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

"""Pointcloud VQA benchmark over go2_short via the dimos evals framework.

Score = mean [0,1] credit across generated cases (rows.json). The surface
under test is the encoding the agent receives for PointCloud2 observations
(``agent_encode()`` when present, ``str(data)`` fallback) — the evo target is
``dimos/msgs/sensor_msgs/PointCloud2.py``.

Flags:
  --blind         withhold context (guessing ablation)
  --max-mean X    exit 1 if mean score exceeds X (blind-gate mode)
  --min-mean X    exit 1 if mean score falls below X (floor-gate mode)
  --limit N       run only the first N cases
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

MAX_CONTEXT_CHARS = 120_000  # anti-gaming + cost ceiling per case context
TRANSIENT = re.compile(r"429|rate.?limit|timeout|connection|temporar", re.IGNORECASE)


def pick(choices: list[str]):
    ordered = sorted(choices, key=len, reverse=True)
    pattern = re.compile(r"\b(" + "|".join(ordered) + r")\b", re.IGNORECASE)

    def parse(text: str) -> str:
        matches = pattern.findall(text)
        return matches[-1].lower() if matches else ""

    return parse


def build_cases(rows: list[dict]):
    from dimos.evals.scorers import exact, first_number, within
    from dimos.evals.types import PassiveEval

    def lidar_select(window):
        return lambda s, w=tuple(window): s.streams.lidar.range_time(*w)

    def odom_select(window):
        return lambda s, w=tuple(window): s.streams.odom.range_time(*w)

    cases = []
    for row in rows:
        context = [lidar_select(row["window"])]
        if row["ctx"] == "lidar+odom":
            context.append(odom_select(row["odom_window"]))
        if row["type"] == "numeric":
            case = PassiveEval(
                id=row["id"],
                inputs=row["q"] + " Answer with a single number.",
                expected=float(row["a"]),
                parse=first_number,
                score=within(float(row["band"])),
                context=tuple(context),
                dataset=row["dataset"],
                tags=frozenset({row["family"], "numeric"}),
            )
        else:
            case = PassiveEval(
                id=row["id"],
                inputs=row["q"],
                expected=str(row["a"]),
                parse=pick(list(row["choices"])),
                score=exact,
                context=tuple(context),
                dataset=row["dataset"],
                tags=frozenset({row["family"], "mcq"}),
            )
        cases.append(case)
    return cases


def check_context_budget(runner, cases) -> None:
    """Fail fast if any case's encoded context exceeds the ceiling."""
    for case in cases:
        store = runner.open_dataset(case.dataset)
        try:
            total = sum(
                len(json.dumps(block, default=str))
                for select in case.context
                for block in runner.encode(select(store))
            )
        finally:
            store.stop()
        if total > MAX_CONTEXT_CHARS:
            print(
                f"context for {case.id} is {total} chars > {MAX_CONTEXT_CHARS};"
                " encoding too verbose — refusing to run",
                file=sys.stderr,
            )
            sys.exit(2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--blind", action="store_true")
    parser.add_argument("--max-mean", type=float, default=None)
    parser.add_argument("--min-mean", type=float, default=None)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    from evo_agent import Run

    from dimos.evals.runner import EvalRunner

    rows = json.loads((Path(__file__).parent / "rows.json").read_text())
    if args.limit:
        rows = rows[: args.limit]
    by_id = {row["id"]: row for row in rows}
    cases = build_cases(rows)

    runner = EvalRunner(blind=args.blind)
    if not args.blind:
        check_context_budget(runner, cases)
    results = list(runner.run(cases))

    # one retry pass for transport-flavored failures; API blips are not
    # encoding regressions
    retry = [
        c for c, r in zip(cases, results, strict=False) if r.error and TRANSIENT.search(r.error)
    ]
    if retry:
        print(f"retrying {len(retry)} transient failures", file=sys.stderr)
        retried = {r.case_id: r for r in EvalRunner(blind=args.blind).run(retry)}
        results = [retried.get(r.case_id, r) for r in results]

    infra = re.compile(
        r"Authentication|RateLimit|quota|APIConnection|APIError|NotFound|Timeout",
        re.IGNORECASE,
    )
    outage = [r for r in results if r.error and infra.search(r.error)]
    if len(outage) / len(results) >= 0.5:
        print(
            f"FAIL: {len(outage)}/{len(results)} cases hit API/infra errors — "
            "refusing to report a fake 0.0",
            file=sys.stderr,
        )
        sys.exit(3)

    run = Run()
    try:
        for result in results:
            row = by_id[result.case_id]
            run.report(
                result.case_id,
                score=result.score,
                summary=f"{row['family']} expected={row['a']} score={result.score:.2f}",
                failure_reason=(result.error or None) if result.score < 1.0 else None,
                extras={
                    "question": row["q"],
                    "expected": row["a"],
                    "family": row["family"],
                    "kind": row["type"],
                    "blind": args.blind,
                    "output": result.outputs[:800],
                    "error": result.error,
                },
            )
    finally:
        run.finish()

    mean = sum(r.score for r in results) / len(results)
    print(f"mean={mean:.4f} n={len(results)} blind={args.blind}")
    if args.max_mean is not None and mean > args.max_mean:
        print(f"FAIL: mean {mean:.3f} > ceiling {args.max_mean}", file=sys.stderr)
        sys.exit(1)
    if args.min_mean is not None and mean < args.min_mean:
        print(f"FAIL: mean {mean:.3f} < floor {args.min_mean}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
