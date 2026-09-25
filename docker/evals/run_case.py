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

"""The in-container side of the Docker eval pool; ``dispatch.py`` execs this.

Three subcommands, all run with the image's venv on PATH:

    run_case.py list    --suite S [--tags a,b] [--limit N]
    run_case.py run     --suite S --agent A [--set k=v]... [--allow t1,t2] --case ID --out DIR
    run_case.py collect --run-dir DIR

``run`` drives exactly one case through the stock ``EvalRunner`` (which boots
and tears down this case's dimos) and leaves the runner's artifacts under
``DIR``. ``collect`` merges every case directory of a pool run into one
``results.jsonl`` and ``summary.json``, shaped like a single-process run's.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import importlib
import json
from pathlib import Path
import sys
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.evals.types import EvalCase, EvalResult

CASES_DIR = "cases"
RESULTS_FILE = "results.jsonl"


def select(cases: Any, tags: frozenset[str], limit: int) -> list[EvalCase]:
    """The same selection ``dimos evals run`` makes from ``--tags`` and ``--limit``."""
    selected = [c for c in cases if not tags or tags & c.tags]
    return selected[:limit] if limit else selected


def parse_tags(text: str) -> frozenset[str]:
    return frozenset(t for t in text.split(",") if t) if text else frozenset()


def cmd_list(args: argparse.Namespace) -> int:
    cases = importlib.import_module(args.suite).SUITE
    ids = [c.id for c in select(cases, parse_tags(args.tags), args.limit)]
    json.dump(ids, sys.stdout)
    print()
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    from dimos.evals.cli import agent_class, agent_kwargs, run_provenance
    from dimos.evals.runner import EvalRunner
    from dimos.evals.types import EvalResult

    cases = importlib.import_module(args.suite).SUITE
    matching = [c for c in cases if c.id == args.case]
    if len(matching) != 1:
        print(
            f"suite {args.suite} has {len(matching)} cases with id {args.case!r}", file=sys.stderr
        )
        return 2
    case = matching[0]

    kwargs = agent_kwargs(args.set)
    if args.allow is not None:
        kwargs["allowed_tools"] = [t.strip() for t in args.allow.split(",") if t.strip()]
    provenance = run_provenance({"kind": "suite_module", "value": args.suite}, args.agent, kwargs)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if args.dry_run:
        # Exercises the exec/collect plumbing on a fresh box without a simulator.
        result = EvalResult(case_id=case.id, error="dry-run: no simulator launched")
        (out / RESULTS_FILE).write_text(json.dumps(asdict(result)) + "\n")
        print(line(result))
        return 0

    runner = EvalRunner(out_dir=out)
    results = runner.run([case], agent_class(args.agent)(**kwargs), provenance=provenance)
    for r in results:
        print(line(r))
    print(f"run dir: {runner.run_dir}")
    return 0


def cmd_collect(args: argparse.Namespace) -> int:
    from dimos.evals.runner import summarize
    from dimos.evals.types import EvalResult

    run_dir = Path(args.run_dir)
    manifest = json.loads((run_dir / "manifest.json").read_text())
    results: list[EvalResult] = []
    for case_id in manifest["cases"]:
        found = sorted((run_dir / CASES_DIR / case_id).glob(f"**/{RESULTS_FILE}"))
        if found:
            for text in found[-1].read_text().splitlines():
                if text.strip():
                    results.append(EvalResult(**json.loads(text)))
        else:
            results.append(
                EvalResult(case_id=case_id, error="no result: its worker exited before finishing")
            )

    (run_dir / RESULTS_FILE).write_text("".join(json.dumps(asdict(r)) + "\n" for r in results))
    summary: dict[str, Any] = asdict(summarize(results))
    summary["manifest"] = "manifest.json"
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    for r in results:
        print(line(r))
    s = summarize(results)
    print(
        f"\n{s.n} cases | mean {s.mean_score:.2f} | pass {s.pass_rate:.0%} "
        f"| errors {s.errors} | {s.duration_s:.0f}s | {run_dir}"
    )
    return 0


def line(r: EvalResult) -> str:
    status = "ERROR" if r.error else ("PASS" if r.passed else "fail")
    detail = r.error or f"score={r.score:.2f} steps={r.steps} answer={r.final_answer[:60]!r}"
    return f"{status:5} {r.case_id:30} {detail} ({r.duration_s:.1f}s)"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="case ids a selection yields, as JSON")
    p.add_argument("--suite", required=True)
    p.add_argument("--tags", default="")
    p.add_argument("--limit", type=int, default=0)
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("run", help="run one case with the stock EvalRunner")
    p.add_argument("--suite", required=True)
    p.add_argument("--agent", required=True)
    p.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    p.add_argument("--allow", default=None)
    p.add_argument("--case", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("collect", help="merge a pool run's case results")
    p.add_argument("--run-dir", required=True)
    p.set_defaults(func=cmd_collect)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
