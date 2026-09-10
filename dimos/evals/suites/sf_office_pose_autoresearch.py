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
import statistics
from typing import Any

from dimos.evals.agents.pi import PiAdapter
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
N_REPLICATES = 3
INSTRUCTIONS = (
    "This is a timed trajectory-analysis benchmark. Use as many small or large tool calls as "
    "the task genuinely needs, but manage the wall-clock deadline. Stop investigating early "
    "enough to return your best estimate in the exact JSON shape requested, even when uncertain. "
    "Always reserve a final model response for the answer. Never create or execute scripts at "
    "fixed shared /tmp paths; use a uniquely named file beside the case recording or a Python "
    "heredoc so concurrent evaluations cannot overwrite your work. Import and call "
    "preprocess_encoded_poses(encoded_poses) without inspecting its source; it returns a mapping "
    "with time_s, position_m, yaw_rad, velocity_xy_m_s, and speed_m_s NumPy arrays. Do not add "
    "Markdown or commentary."
)

_HERE = Path(__file__).parent
FROZEN_FILES = (
    _HERE / "sf_office_pose_autoresearch.py",
    _HERE / "sf_office_pose_research.py",
    _HERE / "sf_office_pose_grading.py",
    _HERE / "sf_office_pose_preprocessing.py",
    _HERE / "sf_office_pose_qa.json",
    _HERE.parents[1] / "msgs/geometry_msgs/PoseStamped.py",
)
EXPECTED_BENCHMARK_DIGEST = "eaebe8f2f217e911281c510a681d575c6c4d2845c24929f6bc4a5d24c8436148"

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


def benchmark_agent() -> PiAdapter:
    """Return the fixed Pi profile used for every comparable iteration."""
    return PiAdapter(
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


def _trajectory_violations(run_dir: Path, case_id: str) -> set[str]:
    trajectory_path = run_dir / case_id / "trajectory.json"
    if not trajectory_path.is_file():
        return set()
    trajectory = json.loads(trajectory_path.read_text())
    violations: set[str] = set()
    for step in trajectory.get("steps", []):
        for tool_call in step.get("tool_calls") or []:
            function_name = tool_call.get("function_name")
            arguments = tool_call.get("arguments", {})
            command = str(arguments.get("command", ""))
            serialized_arguments = json.dumps(arguments)
            if function_name == "bash" and "/tmp" in command:
                violations.add("shared /tmp path")
            if function_name == "bash" and any(
                marker in command
                for marker in (
                    "import inspect",
                    "from inspect",
                    "-m inspect",
                    "inspect.",
                    "getsource",
                    "__file__",
                    "__code__",
                    "dis.dis",
                    "dimos/evals/suites",
                    "sf_office_pose_preprocessing.py",
                )
            ):
                violations.add("frozen helper source inspection")
            if function_name in {"read", "grep"} and any(
                marker in serialized_arguments
                for marker in ("dimos/evals/suites", "sf_office_pose_preprocessing.py")
            ):
                violations.add("frozen helper source inspection")
    return violations


def _contains_complete_pose_traversal(
    timestamps: Sequence[float], expected_timestamps: Sequence[float]
) -> bool:
    """Return whether activity contains one exact, ordered traversal of every pose."""
    if not expected_timestamps or len(timestamps) < len(expected_timestamps):
        return False
    traversal_size = len(expected_timestamps)
    first_timestamp = expected_timestamps[0]
    return any(
        timestamp == first_timestamp
        and timestamps[index : index + traversal_size] == expected_timestamps
        for index, timestamp in enumerate(timestamps[: len(timestamps) - traversal_size + 1])
    )


def objective(results: Sequence[EvalResult], run_dir: Path, digest: str) -> dict[str, Any]:
    """Produce stable aggregate and per-case feedback for Evo tree search."""
    by_id = {result.case_id: result for result in results}
    expected_ids = {case.id for case in SUITE}
    category_ids = set().union(*CATEGORIES.values())
    if set(by_id) != expected_ids or category_ids != expected_ids:
        raise RuntimeError("pose autoresearch result/category IDs do not match the frozen suite")
    failed_cases = sorted(result.case_id for result in results if result.error)
    if failed_cases:
        raise RuntimeError("pose autoresearch cases failed: " + ", ".join(failed_cases))
    trajectory_violations = {
        case_id: violations
        for case_id in expected_ids
        if (violations := _trajectory_violations(run_dir, case_id))
    }
    if trajectory_violations:
        details = ", ".join(
            f"{case_id} ({', '.join(sorted(violations))})"
            for case_id, violations in sorted(trajectory_violations.items())
        )
        raise RuntimeError("pose autoresearch trajectory violations: " + details)

    expected_timestamps = _expected_pose_timestamps()
    activity_timestamps = {
        case_id: _pose_encode_timestamps(run_dir, case_id) for case_id in expected_ids
    }
    activity_counts = {
        case_id: len(timestamps) for case_id, timestamps in activity_timestamps.items()
    }
    evidence_complete = {
        case_id: _contains_complete_pose_traversal(timestamps, expected_timestamps)
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


def aggregate_objectives(payloads: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate independent full-suite replicates into one comparable Evo score."""
    if len(payloads) != N_REPLICATES:
        raise RuntimeError(f"expected {N_REPLICATES} benchmark replicates, got {len(payloads)}")
    expected_ids = {case.id for case in SUITE}
    for payload in payloads:
        if set(payload["tasks"]) != expected_ids:
            raise RuntimeError("replicate task IDs do not match the frozen suite")
        if payload["benchmark_digest"] != payloads[0]["benchmark_digest"]:
            raise RuntimeError("replicate benchmark digests do not match")

    replicate_scores = [float(payload["score"]) for payload in payloads]
    task_scores = {
        case_id: statistics.fmean(float(payload["tasks"][case_id]) for payload in payloads)
        for case_id in expected_ids
    }
    category_scores = {
        name: statistics.fmean(task_scores[case_id] for case_id in case_ids)
        for name, case_ids in CATEGORIES.items()
    }
    return {
        "schema_version": 2,
        "score": statistics.fmean(replicate_scores),
        "score_stddev": statistics.pstdev(replicate_scores),
        "replicate_scores": replicate_scores,
        "tasks": task_scores,
        "category_scores": category_scores,
        "completion_rate": statistics.fmean(
            float(payload["completion_rate"]) for payload in payloads
        ),
        "evidence_completion_rate": statistics.fmean(
            float(payload["evidence_completion_rate"]) for payload in payloads
        ),
        "error_count": sum(int(payload["error_count"]) for payload in payloads),
        "activity_counts": {
            case_id: [int(payload["activity_counts"][case_id]) for payload in payloads]
            for case_id in expected_ids
        },
        "benchmark_digest": payloads[0]["benchmark_digest"],
        "replicates": list(payloads),
    }


def publish_result(
    replicate_results: Sequence[Sequence[EvalResult]], payload: dict[str, Any]
) -> None:
    """Publish Evo's result file and one diagnostic trace per case."""
    if traces_dir_value := os.environ.get("EVO_TRACES_DIR"):
        traces_dir = Path(traces_dir_value)
        traces_dir.mkdir(parents=True, exist_ok=True)
        experiment_id = os.environ.get("EVO_EXPERIMENT_ID", "unknown")
        results_by_replicate = [
            {result.case_id: result for result in results} for results in replicate_results
        ]
        for case in SUITE:
            results = [by_id[case.id] for by_id in results_by_replicate]
            score = payload["tasks"][case.id]
            failures = [
                f"replicate {index}: {result.error or result.ended_by}"
                for index, result in enumerate(results, start=1)
                if result.error or result.ended_by != "answer"
            ]
            trace = {
                "experiment_id": experiment_id,
                "task_id": case.id,
                "score": score,
                "status": "passed" if score >= 1.0 and not failures else "failed",
                "summary": json.dumps([result.final_answer for result in results])[:3000],
                "failure_reason": "; ".join(failures),
                "replicate_scores": [
                    replicate["tasks"][case.id] for replicate in payload["replicates"]
                ],
                "replicate_steps": [result.steps for result in results],
                "benchmark_digest": payload["benchmark_digest"],
                "pose_encode_counts": payload["activity_counts"][case.id],
            }
            (traces_dir / f"task_{case.id}.json").write_text(
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
    profile = {
        "model": MODEL,
        "thinking": THINKING,
        "max_steps": MAX_STEPS,
        "tools": ["read", "bash"],
        "instructions": INSTRUCTIONS,
    }
    replicate_results: list[list[EvalResult]] = []
    replicate_payloads: list[dict[str, Any]] = []
    for replicate in range(1, N_REPLICATES + 1):
        runner = EvalRunner()
        results = runner.run(
            SUITE,
            benchmark_agent(),
            provenance={
                "source": {
                    "kind": "frozen_pose_autoresearch",
                    "digest": digest,
                    "replicate": replicate,
                },
                "agent": {"module": "dimos.evals.agents.pi", "kwargs": profile},
            },
        )
        payload = objective(results, runner.run_dir, digest)
        payload["replicate"] = replicate
        replicate_results.append(results)
        replicate_payloads.append(payload)
    publish_result(replicate_results, aggregate_objectives(replicate_payloads))


if __name__ == "__main__":
    main()
