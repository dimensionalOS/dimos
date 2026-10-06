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

"""Acquire SPACE, bound a normal EvalRunner job, or score its saved responses."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

from langchain_core.language_models.fake_chat_models import FakeListChatModel

from dimos.constants import STATE_DIR
from dimos.evals.agents.text_question import TextQuestion
from dimos.evals.cli import run_provenance
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.lib.space.constants import (
    DATA_SHA256,
    SELECTED_INDICES,
    SMOKE_INDEX,
    SPACE_REVISION,
    TASK,
)
from dimos.evals.suites.lib.space.data import SpacePaths, load_examples, setup
from dimos.evals.suites.lib.space.process import run_process
from dimos.evals.suites.lib.space.report import score_run
from dimos.evals.suites.lib.space.suite import load_cases

AGENT_MODULE = "dimos.evals.agents.text_question"
JOB_TIMEOUT_S = 1800.0


def execute(job_path: Path) -> None:
    """Child process: the standard runner owns every case and its artifacts."""
    job = json.loads(job_path.read_text())
    paths = SpacePaths(Path(job["cache_root"]))
    mode = job["mode"]
    kwargs: dict[str, Any] = {"model": job["model"]}
    if mode == "offline-smoke":
        kwargs = {"chat_model": FakeListChatModel(responses=['{"answer":1}'])}
    agent = TextQuestion(**kwargs)
    config = agent.config.model_dump(mode="json", exclude={"chat_model"})
    if mode == "offline-smoke":
        config["model"] = "FakeListChatModel"
    source = {
        "kind": "space",
        "task": TASK,
        "space_revision": SPACE_REVISION,
        "data_sha256": DATA_SHA256,
        "mode": mode,
        "cases": job["cases"],
    }
    runner = EvalRunner(out_dir=job_path.parent / "runs")
    runner.run(
        load_cases(paths, smoke=mode != "evaluation"),
        agent,
        provenance=run_provenance(source, AGENT_MODULE, config),
    )


def run_job(
    paths: SpacePaths,
    out_dir: Path,
    *,
    model: str | None,
    smoke: bool = False,
    offline_smoke: bool = False,
    timeout_s: float = JOB_TIMEOUT_S,
) -> dict[str, Any]:
    """Retain every attempt; an interrupted job is incomplete even if traces exist."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Job deadline must be finite and positive")
    if offline_smoke and model is not None:
        raise ValueError("Offline smoke cannot select a provider model")
    if not offline_smoke and (model is None or not model.strip()):
        raise ValueError("A provider model is required unless --offline-smoke is selected")
    mode = "offline-smoke" if offline_smoke else ("smoke" if smoke else "evaluation")
    examples = load_examples(paths, (SMOKE_INDEX,) if mode != "evaluation" else SELECTED_INDICES)
    out_dir.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="space-", dir=out_dir)).resolve()
    job: dict[str, Any] = {
        "schema_version": 1,
        "mode": mode,
        "model": model,
        "cache_root": str(paths.root.resolve()),
        "timeout_s": timeout_s,
        "cases": [example.identity() for example in examples],
        "job_dir": str(directory),
        "complete": False,
        "returncode": None,
        "deadline_exceeded": False,
        "scored_denominator": 0,
        "counts": {"unreported": len(examples)},
    }
    job_path = directory / "job.json"
    job_path.write_text(json.dumps(job, indent=2) + "\n")
    started = time.monotonic()
    try:
        job["returncode"] = run_process(
            [
                sys.executable,
                "-m",
                "dimos.evals.suites.lib.space.commands",
                "execute",
                str(job_path),
            ],
            directory / "runner.log",
            timeout_s,
        )
        job["deadline_exceeded"] = False
    except subprocess.TimeoutExpired:
        job["returncode"] = None
        job["deadline_exceeded"] = True
    except KeyboardInterrupt:
        job["cancelled"] = True
        job["error"] = "Runner interrupted"
        raise
    except OSError as exc:
        job["error"] = f"Runner could not start: {exc}"
        raise
    finally:
        job["runner_duration_s"] = time.monotonic() - started
        job_path.write_text(json.dumps(job, indent=2) + "\n")
    runs = list((directory / "runs").glob("run-*"))
    if len(runs) == 1 and (runs[0] / "manifest.json").is_file():
        try:
            report = score_run(paths, runs[0])
        except KeyboardInterrupt:
            job["cancelled"] = True
            job["error"] = "Scoring interrupted"
            raise
        except (OSError, ValueError) as exc:
            job["error"] = f"Saved run could not be scored: {type(exc).__name__}: {exc}"
        else:
            job["report_path"] = report["report_path"]
            job["complete"] = job["returncode"] == 0 and report["complete"]
            job["official_percent"] = report["official_percent"]
            job["scored_denominator"] = report["scored_denominator"]
            job["counts"] = report["counts"]
        finally:
            job_path.write_text(json.dumps(job, indent=2) + "\n")
    else:
        job["complete"] = False
        job["error"] = "Runner did not produce exactly one manifest; inspect runner.log"
    job_path.write_text(json.dumps(job, indent=2) + "\n")
    return job


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    subcommands.add_parser("setup", help="Acquire pinned source and one verified task file")
    run = subcommands.add_parser("run", help="Run the frozen subset or a reserved smoke case")
    run.add_argument("--model", help="Explicit model identifier accepted by the DimOS factory")
    run.add_argument("--smoke", action="store_true")
    run.add_argument(
        "--offline-smoke", action="store_true", help="One canned reply; no model calls"
    )
    run.add_argument("--timeout", type=float, default=JOB_TIMEOUT_S, help="Whole runner deadline")
    run.add_argument("--out-dir", type=Path, default=STATE_DIR / "evals" / "space")
    score = subcommands.add_parser("score", help="Replay saved replies without model calls")
    score.add_argument("run_dir", type=Path)
    child = subcommands.add_parser("execute", help="Internal runner subprocess")
    child.add_argument("job", type=Path)
    args = parser.parse_args(argv)
    paths = SpacePaths()
    if args.command == "setup":
        result = setup(paths)
    elif args.command == "execute":
        execute(args.job)
        return 0
    elif args.command == "score":
        result = score_run(paths, args.run_dir)
    else:
        result = run_job(
            paths,
            args.out_dir,
            model=args.model,
            smoke=args.smoke,
            offline_smoke=args.offline_smoke,
            timeout_s=args.timeout,
        )
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result.get("complete", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
