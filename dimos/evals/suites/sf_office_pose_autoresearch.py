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

"""Frozen entry point for PoseStamped encoder autoresearch."""

from __future__ import annotations

from collections.abc import Sequence
import hashlib
import json
import os
from pathlib import Path
from typing import Any

from dimos.evals.agents.pi import Pi
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.sf_office_pose_research import (
    EXPECTED_POSE_COUNT,
    EXPECTED_RECORDING_SHA256,
    RECORDING_PATH,
    SUITE,
)
from dimos.evals.types import EvalResult
from dimos.memory.store.sqlite import SqliteStore

MODEL = "gpt-5.6-luna"
THINKING = "medium"
MAX_STEPS = 40
INSTRUCTIONS = (
    "This is a timed trajectory-analysis benchmark. Use as many small or large tool calls as "
    "the task genuinely needs, but manage the wall-clock deadline. Stop investigating early "
    "enough to return your best estimate in the exact JSON shape requested, even when uncertain. "
    "Always reserve a final model response for the answer. Do not add Markdown or commentary."
)

_HERE = Path(__file__).parent
FROZEN_FILES = (
    _HERE / "sf_office_pose_autoresearch.py",
    _HERE / "sf_office_pose_research.py",
    _HERE / "sf_office_pose_grading.py",
    _HERE / "sf_office_pose_answers.json",
    _HERE.parents[1] / "msgs/geometry_msgs/PoseStamped.py",
)
EXPECTED_BENCHMARK_DIGEST = "54d4fe0879930d6d2e9a21e75b0e1118a764c8b0d7925510dd666ff1a6171f31"

CATEGORIES = {
    "kinematics": frozenset(
        {
            "sf_office_pose_return_distance",
            "sf_office_pose_stationary_percentage",
            "sf_office_pose_backward_intervals",
            "sf_office_pose_least_aligned",
        }
    ),
    "topology": frozenset(
        {
            "sf_office_pose_self_intersections",
            "sf_office_pose_opposite_retrace",
            "sf_office_pose_longest_elapsed_return",
        }
    ),
    "planning": frozenset({"sf_office_pose_path_compression"}),
    "patterns": frozenset({"sf_office_pose_repeated_patrol_cycle"}),
}


def benchmark_digest(paths: Sequence[Path] = FROZEN_FILES) -> str:
    """Hash frozen files, source identity, and the fixed Pi profile."""
    digest = hashlib.sha256()
    inputs = []
    for path in paths:
        content = path.read_bytes()
        if path.resolve() == Path(__file__).resolve():
            content = content.replace(
                EXPECTED_BENCHMARK_DIGEST.encode(), b"<expected-benchmark-digest>"
            )
        inputs.append((path.name.encode(), content))
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
    inputs.extend(
        (
            (b"pi-profile.json", profile),
            (b"recording.sha256", EXPECTED_RECORDING_SHA256.encode()),
        )
    )
    for name, content in inputs:
        digest.update(len(name).to_bytes(4, "big"))
        digest.update(name)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def verify_benchmark() -> str:
    """Reject evaluation after any question, reference, grader, wrapper, or profile change."""
    actual = benchmark_digest()
    if actual != EXPECTED_BENCHMARK_DIGEST:
        raise RuntimeError(
            "pose benchmark integrity check failed: "
            f"expected {EXPECTED_BENCHMARK_DIGEST}, got {actual}"
        )
    return actual


def verify_recording() -> str:
    """Reject evaluation if the external source recording changed."""
    digest = hashlib.sha256()
    with RECORDING_PATH.open("rb") as recording:
        while chunk := recording.read(1024 * 1024):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != EXPECTED_RECORDING_SHA256:
        raise RuntimeError(
            "pose recording integrity check failed: "
            f"expected {EXPECTED_RECORDING_SHA256}, got {actual}"
        )
    return actual


def benchmark_agent() -> Pi:
    """Return the fixed Pi profile used for every comparable iteration."""
    return Pi(
        model=MODEL,
        thinking=THINKING,
        max_steps=MAX_STEPS,
        instructions=INSTRUCTIONS,
        tools=("read", "bash"),
    )


def _expected_pose_timestamps() -> list[float]:
    with SqliteStore(path=RECORDING_PATH, must_exist=True) as store:
        timestamps = [
            float(observation.data.ts) for observation in store.streams.odom.order_by("ts")
        ]
    if len(timestamps) != EXPECTED_POSE_COUNT:
        raise RuntimeError(
            f"expected {EXPECTED_POSE_COUNT} odom poses in the frozen recording, "
            f"found {len(timestamps)}"
        )
    return timestamps


def _pose_encode_timestamps(run_dir: Path, case_id: str) -> list[float]:
    path = run_dir / case_id / "agent-activity/events.jsonl"
    if not path.is_file():
        return []
    timestamps: list[float] = []
    for line in path.read_text().splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event.get("event") != "agent_encode" or event.get("message_type") != "PoseStamped":
            continue
        timestamp = event.get("output", {}).get("source_timestamp_s")
        if isinstance(timestamp, int | float) and not isinstance(timestamp, bool):
            timestamps.append(float(timestamp))
    return timestamps


def objective(results: Sequence[EvalResult], run_dir: Path, digest: str) -> dict[str, Any]:
    """Produce stable aggregate and per-case feedback for Evo tree search."""
    by_id = {result.case_id: result for result in results}
    expected_ids = {case.id for case in SUITE}
    category_ids = set().union(*CATEGORIES.values())
    if set(by_id) != expected_ids or category_ids != expected_ids:
        raise RuntimeError("pose autoresearch result/category IDs do not match the frozen suite")
    if all(result.error for result in results):
        raise RuntimeError("every pose autoresearch case failed before producing a valid result")

    expected_timestamps = _expected_pose_timestamps()
    activity_timestamps = {
        case_id: _pose_encode_timestamps(run_dir, case_id) for case_id in expected_ids
    }
    activity_counts = {
        case_id: len(timestamps) for case_id, timestamps in activity_timestamps.items()
    }
    evidence_complete = {
        case_id: timestamps == expected_timestamps
        for case_id, timestamps in activity_timestamps.items()
    }
    task_scores = {
        case_id: result.score if evidence_complete[case_id] else 0.0
        for case_id, result in by_id.items()
    }
    category_scores = {
        name: sum(task_scores[case_id] for case_id in case_ids) / len(case_ids)
        for name, case_ids in CATEGORIES.items()
    }
    return {
        "schema_version": 1,
        "score": sum(task_scores.values()) / len(task_scores),
        "tasks": task_scores,
        "category_scores": category_scores,
        "completion_rate": sum(bool(result.final_answer) for result in results) / len(results),
        "evidence_completion_rate": sum(evidence_complete.values()) / len(evidence_complete),
        "activity_counts": activity_counts,
        "error_count": sum(bool(result.error) for result in results),
        "benchmark_digest": digest,
        "run_dir": str(run_dir),
    }


def publish_result(results: Sequence[EvalResult], payload: dict[str, Any]) -> None:
    """Publish Evo's result file and one diagnostic trace per case."""
    if traces_dir_value := os.environ.get("EVO_TRACES_DIR"):
        traces_dir = Path(traces_dir_value)
        traces_dir.mkdir(parents=True, exist_ok=True)
        experiment_id = os.environ.get("EVO_EXPERIMENT_ID", "unknown")
        for result in results:
            score = payload["tasks"][result.case_id]
            trace = {
                "experiment_id": experiment_id,
                "task_id": result.case_id,
                "score": score,
                "status": "passed" if score >= 1.0 and not result.error else "failed",
                "summary": result.final_answer[:1000],
                "failure_reason": result.error
                or (result.ended_by if result.ended_by != "answer" else ""),
                "steps": result.steps,
                "benchmark_digest": payload["benchmark_digest"],
                "pose_encode_count": payload["activity_counts"][result.case_id],
            }
            (traces_dir / f"task_{result.case_id}.json").write_text(
                json.dumps(trace, indent=2, allow_nan=False)
            )

    serialized = json.dumps(payload, sort_keys=True, allow_nan=False)
    if not (result_path_value := os.environ.get("EVO_RESULT_PATH")):
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
    verify_recording()
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
            "source": {"kind": "frozen_pose_autoresearch", "digest": digest},
            "agent": {"module": "dimos.evals.agents.pi", "kwargs": profile},
        },
    )
    publish_result(results, objective(results, runner.run_dir, digest))


if __name__ == "__main__":
    main()
