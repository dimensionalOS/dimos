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

"""Inline instrumentation for Python benchmarks. Paste into the benchmark
and call `log_task()` per task + `write_result()` once at the end.

Contract:
- Reads EVO_TRACES_DIR, EVO_EXPERIMENT_ID, EVO_RESULT_PATH from env.
- Writes traces/task_<id>.json per task.
- Writes the final result JSON to EVO_RESULT_PATH, or stdout if unset.

**Per-task emission is the load-bearing discipline.** If your benchmark
evaluates N independent items (per-question math, per-test-case unit
tests, per-document QA, per-sample reasoning trace), call `log_task`
ONCE PER ITEM with as much detail as you have (problem text, model
output, expected answer, intermediate reasoning) -- not one aggregate
call with the rolled-up score. The dashboard's per-task panel and the
verifier's reproducibility spot-check both rely on per-item traces;
emitting one aggregate `log_task("eval_total", score)` makes both
useless and loses the diagnostic value of the run. `write_result()`
aggregates from the per-task scores automatically -- you don't need to
roll up yourself. See the USAGE EXAMPLE at the bottom of this file.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_TRACES_DIR = Path(os.environ["EVO_TRACES_DIR"]) if os.environ.get("EVO_TRACES_DIR") else None
_EXPERIMENT_ID = os.environ.get("EVO_EXPERIMENT_ID", "unknown")
_RESULT_PATH = os.environ.get("EVO_RESULT_PATH")
_SCORES: dict[str, float] = {}
_TASK_META: dict[str, dict[str, Any]] = {}
_STARTED_AT = datetime.now(timezone.utc).isoformat(timespec="seconds")

if _TRACES_DIR:
    _TRACES_DIR.mkdir(parents=True, exist_ok=True)


def log_task(
    task_id: str,
    score: float,
    *,
    summary: str | None = None,
    failure_reason: str | None = None,
    log: list[Any] | None = None,
    direction: str | None = None,
    **extra: Any,
) -> None:
    """Record the result for one task. Writes task_<id>.json immediately.

    *direction* is "max" (higher is better, default) or "min" (lower is
    better, e.g. latency). Only set it when this task's direction differs
    from the benchmark's top-level `--metric`. Propagates to `tasks_meta`
    in the final result JSON for downstream selection strategies.
    """
    task_id = str(task_id)
    if direction is not None and direction not in ("max", "min"):
        raise ValueError(f"direction must be 'max' or 'min', got {direction!r}")
    _SCORES[task_id] = score
    if direction is not None:
        _TASK_META[task_id] = {"direction": direction}
    if _TRACES_DIR is None:
        return
    trace: dict[str, Any] = {
        "experiment_id": _EXPERIMENT_ID,
        "task_id": task_id,
        "status": "passed" if score >= 0.5 else "failed",
        "score": score,
        "ended_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    if direction is not None:
        trace["direction"] = direction
    if summary is not None:
        trace["summary"] = summary
    if failure_reason is not None:
        trace["failure_reason"] = failure_reason
    if log is not None:
        trace["log"] = log
    trace.update(extra)
    (_TRACES_DIR / f"task_{task_id}.json").write_text(
        json.dumps(trace, indent=2), encoding="utf-8"
    )


def write_result(score: float | None = None) -> float:
    """Write the final score JSON to $EVO_RESULT_PATH (or stdout if unset)
    and return the score. The return lets callers gate on --min-score
    without recomputing the aggregate.
    """
    if score is None:
        score = sum(_SCORES.values()) / len(_SCORES) if _SCORES else 0.0
    score = round(score, 4)
    result = {
        "score": score,
        "tasks": dict(_SCORES),
        "started_at": _STARTED_AT,
        "ended_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    if _TASK_META:
        result["tasks_meta"] = {k: dict(v) for k, v in _TASK_META.items()}
    payload = json.dumps(result, indent=2)
    if _RESULT_PATH:
        target = Path(_RESULT_PATH)
        target.parent.mkdir(parents=True, exist_ok=True)
        # Claim + tmp+rename: duplicate writers fail-fast; crash mid-publish
        # leaves an empty file (caught by load_result) not a partial write.
        try:
            os.close(os.open(target, os.O_CREAT | os.O_EXCL | os.O_WRONLY))
        except FileExistsError:
            raise RuntimeError(
                f"{target} already exists; only one write_result() per attempt"
            ) from None
        tmp = target.with_name(target.name + ".tmp")
        tmp.write_text(payload, encoding="utf-8")
        os.replace(tmp, target)
    else:
        print(payload)
    return score


# === USAGE EXAMPLE (copy + adapt) ===
#
# For a benchmark scoring N independent items, emit one log_task per item.
# write_result() with no arg aggregates _SCORES into the final score.
#
# def main():
#     problems = load_aime_problems()              # list of N items
#     model = load_model()
#     for i, problem in enumerate(problems):
#         output = model.generate(problem.question)
#         correct = extract_answer(output) == problem.expected
#         log_task(
#             f"aime_q{i:02d}",                    # stable id per item
#             score=1.0 if correct else 0.0,       # per-item score
#             question=problem.question,           # everything else goes as **extra
#             expected=problem.expected,           # and lands in the trace JSON
#             model_output=output,                 # for diagnosis later
#             tokens_used=len(model.last_tokens),
#         )
#     # write_result() with no arg returns mean(_SCORES) over all logged tasks --
#     # no need to compute the average yourself
#     final_score = write_result()
#     print(f"final: {final_score:.4f}")
#
# Anti-pattern: ONE log_task or write_result call with the aggregate. The
# dashboard's per-task panel + the verifier's reproducibility spot-check
# both need per-item traces; aggregate-only emission makes them useless.
#
#     # DO NOT do this:
#     score = sum(per_problem_correct) / len(per_problem_correct)
#     log_task("eval_total", score)               # <-- aggregate; loses detail
#     write_result(score)
#
# Exception: if the benchmark really is a single indivisible measurement
# (one e2e workflow, one perf number), emit one task with that score AND
# include every observable as **extra (timings, allocations, error log).
