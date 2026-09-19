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

"""One ``dimos evals run`` per case, in parallel, each in its own container.

A live case owns a host (MCP port, multicast bus, simulator ports, viewer port), so
parallelism is a scheduler around the sequential runner, not a change to it: each job
is ``dimos evals run <suite> --agent <agent> --case <id>`` with ``XDG_STATE_HOME`` under
the job directory, in a container when one is given. The parent merges the rows.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
from typing import Any

from dimos.constants import STATE_DIR
from dimos.evals.types import EvalCase, EvalResult
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

KEY_VARS = ("OPENAI_API_KEY", "ANTHROPIC_API_KEY", "TYPESAFE_API_KEY")
LAUNCH_MARGIN_S = 1500.0  # simulator launch timeout plus teardown and grading


def _job_command(
    suite: str,
    agent: str,
    overrides: list[str],
    case: EvalCase,
    job: Path,
    container: str,
    run_id: str,
) -> tuple[list[str], dict[str, str] | None]:
    child = ["dimos", "evals", "run", suite, "--agent", agent, "--case", case.id]
    child += [arg for o in overrides for arg in ("--set", o)]
    if not container:
        return child, {**os.environ, "XDG_STATE_HOME": str(job / "state")}
    keys = [arg for k in KEY_VARS if k in os.environ for arg in ("-e", k)]
    return [
        "docker", "run", "--rm", "--name", f"dimos-eval-{run_id}-{job.name}",
        "--label", f"dimos-eval={run_id}", "--gpus", "all", "--shm-size", "2g",
        "-e", "XDG_STATE_HOME=/out/state", *keys, "-v", f"{job}:/out", container, *child,
    ], None  # fmt: skip


def _run_job(
    suite: str,
    agent: str,
    overrides: list[str],
    case: EvalCase,
    job: Path,
    container: str,
    run_id: str,
) -> dict[str, Any]:
    job.mkdir(parents=True, exist_ok=True)
    cmd, env = _job_command(suite, agent, overrides, case, job, container, run_id)
    t0 = time.monotonic()
    with (job / "job.log").open("w") as log:
        try:
            proc = subprocess.run(
                cmd,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=case.timeout_s + LAUNCH_MARGIN_S,
            )
            exit_code = proc.returncode
        except subprocess.TimeoutExpired:
            if container:
                subprocess.run(
                    ["docker", "rm", "-f", f"dimos-eval-{run_id}-{job.name}"], check=False
                )
            exit_code = -1
    rows = list((job / "state" / "dimos" / "evals").glob("run-*/results.jsonl"))
    if exit_code == 0 and rows:
        row: dict[str, Any] = json.loads(rows[0].read_text().splitlines()[0])
        if container:
            row["trajectory"] = row["trajectory"].replace("/out", str(job), 1)
    else:
        row = {"case_id": case.id, "error": f"job exit {exit_code}: {job / 'job.log'}"}
    row["duration_s"] = time.monotonic() - t0
    row["job_dir"] = str(job)
    return row


def run_parallel(
    suite: str,
    agent: str,
    overrides: list[str],
    cases: list[EvalCase],
    *,
    parallel: int,
    container: str,
    repeat: int,
    manifest: dict[str, Any],
) -> tuple[Path, list[EvalResult]]:
    out_dir = STATE_DIR / "evals"
    out_dir.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=time.strftime("run-%Y%m%d-%H%M%S-"), dir=out_dir))
    run_id = run_dir.name.split("-")[-1]
    (run_dir / "manifest.json").write_text(
        json.dumps(
            {**manifest, "parallel": parallel, "container": container, "repeat": repeat}, indent=2
        )
    )
    jobs = [(case, run_dir / f"{case.id}-{k}") for k in range(1, repeat + 1) for case in cases]
    try:
        with ThreadPoolExecutor(parallel) as pool:
            rows = list(
                pool.map(
                    lambda j: _run_job(suite, agent, overrides, j[0], j[1], container, run_id), jobs
                )
            )
    finally:
        if container:  # reap stragglers after an interrupt
            left = subprocess.run(
                ["docker", "ps", "-aq", "--filter", f"label=dimos-eval={run_id}"],
                capture_output=True, text=True, check=False,
            ).stdout.split()  # fmt: skip
            if left:
                subprocess.run(["docker", "rm", "-f", *left], check=False)
    (run_dir / "results.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    known = {f.name for f in fields(EvalResult)}
    results = [EvalResult(**{k: v for k, v in r.items() if k in known}) for r in rows]
    for r in results:
        logger.info("eval job done", case=r.case_id, score=round(r.score, 3), error=r.error or None)
    return run_dir, results
