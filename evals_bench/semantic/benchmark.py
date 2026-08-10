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

"""Semantic-map VQA benchmark over go2_bigoffice via the dimos evals framework.

Score = mean [0,1] credit across generated cases (rows.json). The surface
under test is the encoding the agent receives for the raw semantic detection
stream: ``detections3d_stream()`` (dimos/memory2/objects.py) materializes the
frozen detections (detections.json) as the canonical perception payload —
``ImageDetections3DPC`` observations on a memory2 stream named
``"detections3d"``, the tool_localize.py convention — and the eval runner
encodes each observation (``agent_encode()`` when the type provides it,
``str(data)`` fallback). The evo target is the file where that
``agent_encode`` lives:
``dimos/perception/detection/type/detection3d/imageDetections3DPC.py``.

CONTEXT PARITY CONTRACT (load-bearing — generate_rows.py and this harness
must agree; do not change one side without the other):

  1. Ground truth in generate_rows.py is computed from the FULL recording's
     detections (all of detections.json, minus the documented class filters).
     The student's objects context is therefore the FULL-span detections3d
     stream: every frame, every detection, no curation and no truncation.
     ``build_cases`` asserts frame count <= CONTEXT_BUDGET so the runner's
     evenly-spaced subsampler never drops a frame.
  2. For time-conditioned families (nearest / egoside), truth is the robot
     odom pose at the question timestamp. The odom context window therefore
     ENDS EXACTLY at the question timestamp (row ``odom_window[1] == t``):
     "your current pose is the last odom observation shown". The objects
     context stays full-span for these rows too — truth ranks the full map.
  3. Odom may be subsampled by the runner (evenly spaced, ~19 Hz source);
     the subsampler always keeps the last observation, so the pose the
     question is conditioned on survives subsampling.

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

# Anti-gaming + cost ceiling per case context. The uncurated full-span
# detections3d stream str()-encodes to ~156k chars (rich table + ANSI), so the
# ceiling sits above the raw baseline while still refusing point-dump blowups.
MAX_CONTEXT_CHARS = 200_000
CONTEXT_BUDGET = 256  # >= frame count (161): full detection stream, no subsampling
TRANSIENT = re.compile(r"429|rate.?limit|timeout|connection|temporar", re.IGNORECASE)


def pick(choices: list[str]):
    ordered = sorted(choices, key=len, reverse=True)
    pattern = re.compile(r"\b(" + "|".join(ordered) + r")\b", re.IGNORECASE)

    def parse(text: str) -> str:
        matches = pattern.findall(text)
        return matches[-1].lower() if matches else ""

    return parse


def class_set(vocab: list[str]):
    """Parse every class name mentioned in a reply into a set.

    ``vocab`` = mapped classes + verified-absent probes, so hallucinated
    extras are caught and penalized via F1 precision.
    """
    ordered = sorted(vocab, key=len, reverse=True)
    pattern = re.compile(r"\b(" + "|".join(re.escape(v) for v in ordered) + r")\b", re.IGNORECASE)

    def parse(text: str) -> frozenset[str]:
        return frozenset(m.lower() for m in pattern.findall(text))

    return parse


def set_f1(expected: frozenset[str], got: frozenset[str]) -> float:
    """F1 between predicted and truth class sets ([0,1]; listing every class
    loses precision, listing nothing scores 0)."""
    tp = len(expected & got)
    return 2 * tp / (len(expected) + len(got)) if tp else 0.0


def build_cases(rows: list[dict]):
    from dimos.evals.scorers import exact, first_number, within
    from dimos.evals.types import PassiveEval
    from dimos.memory2.objects import detections3d_stream

    detections = json.loads((Path(__file__).parent / "detections.json").read_text())
    # parity contract #1: full stream must fit the runner's context budget
    n_frames = len({d["ts"] for d in detections})
    assert n_frames <= CONTEXT_BUDGET, (
        f"{n_frames} detection frames > CONTEXT_BUDGET {CONTEXT_BUDGET}: "
        "the runner would subsample the objects context — parity broken"
    )

    def objects_select(store):
        return detections3d_stream(detections)

    def odom_select(window):
        return lambda s, w=tuple(window): s.streams.odom.range_time(*w)

    cases = []
    for row in rows:
        context = [objects_select]
        if row["ctx"] == "objects+odom":
            context.append(odom_select(row["odom_window"]))
        if row["type"] == "numeric":
            case = PassiveEval(
                id=row["id"],
                inputs=row["q"],
                expected=float(row["a"]),
                parse=first_number,
                score=within(float(row["band"])),
                context=tuple(context),
                dataset=row["dataset"],
                tags=frozenset({row["family"], "numeric"}),
            )
        elif row["type"] == "set":
            case = PassiveEval(
                id=row["id"],
                inputs=row["q"],
                expected=frozenset(row["a"]),
                parse=class_set(list(row["vocab"])),
                score=set_f1,
                context=tuple(context),
                dataset=row["dataset"],
                tags=frozenset({row["family"], "set"}),
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
    """Fail fast if any case's encoded context exceeds the ceiling.

    Text blocks are measured as the raw text the model receives (json.dumps
    would count every ANSI/box-drawing char as a 6-char \\uXXXX escape and
    overstate rich-table encodings ~2.6x); other blocks by their JSON size.
    """
    for case in cases:
        store = runner.open_dataset(case.dataset)
        try:
            total = sum(
                len(block["text"])
                if block.get("type") == "text"
                else len(json.dumps(block, default=str))
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

    runner = EvalRunner(blind=args.blind, context_budget=CONTEXT_BUDGET)
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
        retried = {
            r.case_id: r
            for r in EvalRunner(blind=args.blind, context_budget=CONTEXT_BUDGET).run(retry)
        }
        results = [retried.get(r.case_id, r) for r in results]

    infra = re.compile(
        r"Authentication|RateLimit|quota|APIConnection|APIError|NotFound|Timeout"
        r"|api.?key|OpenAIError",
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
