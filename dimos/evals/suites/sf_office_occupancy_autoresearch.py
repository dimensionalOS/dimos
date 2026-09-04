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

"""Frozen entry point for occupancy encoder autoresearch.

Run with::

    python -m dimos.evals.suites.sf_office_occupancy_autoresearch

Under Evo, results and per-case traces are written to its requested artifact
paths. Outside Evo, the result JSON is printed to stdout.
"""

from __future__ import annotations

from collections.abc import Sequence
import hashlib
import json
import os
from pathlib import Path
from typing import Any

from dimos.evals.agents.pi import Pi
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.sf_office_occupancy_research import SUITE
from dimos.evals.types import EvalResult

MODEL = "gpt-5.6-luna"
THINKING = "medium"
MAX_STEPS = 40
INSTRUCTIONS = (
    "This is a timed spatial-understanding benchmark. Use as many small or large tool calls as "
    "the task genuinely needs, but manage the wall-clock deadline. Stop investigating early "
    "enough to return your best estimate in the exact JSON shape requested, even when uncertain. "
    "Always reserve a final model response for the answer. Do not add Markdown or commentary."
)

_HERE = Path(__file__).parent
FROZEN_FILES = (
    _HERE / "sf_office_occupancy_research.py",
    _HERE / "sf_office_occupancy_grading.py",
    _HERE / "sf_office_occupancy_answers.json",
)
EXPECTED_BENCHMARK_DIGEST = "939eef6e1255fd3b7549790a3518f86f88665e6dab81f3458b6d22b0c25eab75"

CATEGORIES = {
    "semantics": frozenset(
        {
            "sf_office_occupancy_room_count",
            "sf_office_occupancy_doorways",
            "sf_office_occupancy_hide_location",
            "sf_office_occupancy_doorway_bottleneck",
        }
    ),
    "temporal": frozenset(
        {
            "sf_office_occupancy_movement_square",
            "sf_office_occupancy_possible_person_motion",
            "sf_office_occupancy_first_reachable_time",
        }
    ),
    "routing": frozenset(
        {
            "sf_office_occupancy_three_point_loop",
            "sf_office_occupancy_max_robot_radius",
            "sf_office_occupancy_blocked_opening_reachability",
            "sf_office_occupancy_independent_routes",
        }
    ),
    "geometry": frozenset(
        {
            "sf_office_occupancy_largest_free_circle",
            "sf_office_occupancy_constant_twist_collision",
            "sf_office_occupancy_known_free_area",
            "sf_office_occupancy_forward_clearance",
        }
    ),
}


def benchmark_digest(paths: Sequence[Path] = FROZEN_FILES) -> str:
    """Hash frozen files and the fixed Pi profile with unambiguous framing."""
    digest = hashlib.sha256()
    inputs = [(path.name.encode(), path.read_bytes()) for path in paths]
    profile = json.dumps(
        {
            "model": MODEL,
            "thinking": THINKING,
            "max_steps": MAX_STEPS,
            "instructions": INSTRUCTIONS,
            "tools": ["read", "bash"],
        },
        sort_keys=True,
    ).encode()
    inputs.append((b"pi-profile.json", profile))
    for name, content in inputs:
        digest.update(len(name).to_bytes(4, "big"))
        digest.update(name)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def verify_benchmark() -> str:
    """Reject evaluation after any question, reference, grader, or profile change."""
    actual = benchmark_digest()
    if actual != EXPECTED_BENCHMARK_DIGEST:
        raise RuntimeError(
            "occupancy benchmark integrity check failed: "
            f"expected {EXPECTED_BENCHMARK_DIGEST}, got {actual}"
        )
    return actual


def benchmark_agent() -> Pi:
    """The fixed Pi profile used for every comparable research iteration."""
    return Pi(
        model=MODEL,
        thinking=THINKING,
        max_steps=MAX_STEPS,
        instructions=INSTRUCTIONS,
        tools=("read", "bash"),
    )


def objective(results: Sequence[EvalResult], run_dir: Path, digest: str) -> dict[str, Any]:
    """Produce stable aggregate and per-case feedback for Evo tree search."""
    by_id = {result.case_id: result for result in results}
    expected_ids = {case.id for case in SUITE}
    category_ids = set().union(*CATEGORIES.values())
    if set(by_id) != expected_ids or category_ids != expected_ids:
        raise RuntimeError("autoresearch result/category IDs do not match the frozen suite")

    category_scores = {
        name: sum(by_id[case_id].score for case_id in case_ids) / len(case_ids)
        for name, case_ids in CATEGORIES.items()
    }
    return {
        "schema_version": 1,
        "score": sum(result.score for result in results) / len(results),
        "tasks": {result.case_id: result.score for result in results},
        "category_scores": category_scores,
        "completion_rate": sum(bool(result.final_answer) for result in results) / len(results),
        "error_count": sum(bool(result.error) for result in results),
        "benchmark_digest": digest,
        "run_dir": str(run_dir),
    }


def publish_result(results: Sequence[EvalResult], payload: dict[str, Any]) -> None:
    """Publish Evo's atomic result file and one diagnostic trace per case."""
    if traces_dir_value := os.environ.get("EVO_TRACES_DIR"):
        traces_dir = Path(traces_dir_value)
        traces_dir.mkdir(parents=True, exist_ok=True)
        experiment_id = os.environ.get("EVO_EXPERIMENT_ID", "unknown")
        for result in results:
            trace = {
                "experiment_id": experiment_id,
                "task_id": result.case_id,
                "score": result.score,
                "status": "passed" if result.passed else "failed",
                "summary": result.final_answer[:1000],
                "failure_reason": result.error or result.ended_by,
                "steps": result.steps,
            }
            path = traces_dir / f"task_{result.case_id}.json"
            path.write_text(json.dumps(trace, indent=2, allow_nan=False))

    serialized = json.dumps(payload, sort_keys=True, allow_nan=False)
    result_path_value = os.environ.get("EVO_RESULT_PATH")
    if not result_path_value:
        print(serialized)
        return
    result_path = Path(result_path_value)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.open("x").close()
    temporary = result_path.with_name(f"{result_path.name}.tmp")
    temporary.write_text(serialized)
    temporary.replace(result_path)


def main() -> None:
    digest = verify_benchmark()
    runner = EvalRunner()
    profile = {
        "model": MODEL,
        "thinking": THINKING,
        "max_steps": MAX_STEPS,
        "tools": ["read", "bash"],
        "instructions": INSTRUCTIONS,
    }
    results = runner.run(
        SUITE,
        benchmark_agent(),
        provenance={
            "source": {"kind": "frozen_occupancy_autoresearch", "digest": digest},
            "agent": {"module": "dimos.evals.agents.pi", "kwargs": profile},
        },
    )
    payload = objective(results, runner.run_dir, digest)
    publish_result(results, payload)


if __name__ == "__main__":
    main()
