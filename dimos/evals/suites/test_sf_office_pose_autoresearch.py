# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
from pathlib import Path

import pytest

from dimos.evals.suites import sf_office_pose_autoresearch
from dimos.evals.suites.sf_office_pose_autoresearch import (
    EXPECTED_BENCHMARK_DIGEST,
    MAX_STEPS,
    MODEL,
    N_REPLICATES,
    THINKING,
    _expected_pose_timestamps,
    aggregate_objectives,
    benchmark_agent,
    benchmark_digest,
    objective,
    publish_result,
    verify_benchmark,
    verify_recording,
)
from dimos.evals.suites.sf_office_pose_research import EXPECTED_POSE_COUNT, SUITE
from dimos.evals.types import EvalResult


def test_benchmark_profile_is_fixed() -> None:
    agent = benchmark_agent()

    assert (agent.model, agent.thinking, agent.max_steps) == (MODEL, THINKING, MAX_STEPS)
    assert agent.max_steps == 40
    assert tuple(agent.tools) == ("read", "bash")
    assert N_REPLICATES == 3


def test_frozen_benchmark_digest_matches_files() -> None:
    assert verify_benchmark() == EXPECTED_BENCHMARK_DIGEST
    assert benchmark_digest() == EXPECTED_BENCHMARK_DIGEST


def test_source_recording_is_frozen() -> None:
    assert len(verify_recording()) == 64


def test_expected_pose_evidence_is_complete_and_time_ordered() -> None:
    timestamps = _expected_pose_timestamps()

    assert len(timestamps) == EXPECTED_POSE_COUNT
    assert timestamps == sorted(timestamps)


def test_digest_changes_with_file_content(tmp_path: Path) -> None:
    path = tmp_path / "input"
    path.write_text("before")
    initial = benchmark_digest((path,))

    path.write_text("after")

    assert benchmark_digest((path,)) != initial


def test_objective_reports_all_cases_and_categories(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    event = json.dumps(
        {
            "event": "agent_encode",
            "message_type": "PoseStamped",
            "output": {"source_timestamp_s": 12.5},
        }
    )
    for case in SUITE:
        activity_dir = tmp_path / case.id / "agent-activity"
        activity_dir.mkdir(parents=True)
        (activity_dir / "events.jsonl").write_text(event)
    results = [
        EvalResult(case_id=case.id, score=0.5, final_answer="{}", duration_s=1.0) for case in SUITE
    ]

    monkeypatch.setattr(sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5])
    result = objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)

    assert result["score"] == 0.5
    assert result["completion_rate"] == 1.0
    assert result["error_count"] == 0
    assert result["evidence_completion_rate"] == 1.0
    assert result["tasks"] == {case.id: 0.5 for case in SUITE}


def test_objective_zeros_scores_without_complete_pose_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    results = [
        EvalResult(case_id=case.id, score=1.0, final_answer="{}", duration_s=1.0) for case in SUITE
    ]

    monkeypatch.setattr(sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5])
    result = objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)

    assert result["score"] == 0.0
    assert result["evidence_completion_rate"] == 0.0


@pytest.mark.parametrize(
    "command",
    ["python /tmp/analysis.py", "cd /tmp && python analysis.py", "TMPDIR=/tmp; python -V"],
)
def test_objective_rejects_shared_tmp_scripts(
    command: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    results = [
        EvalResult(case_id=case.id, score=1.0, final_answer="{}", duration_s=1.0) for case in SUITE
    ]
    case_dir = tmp_path / SUITE[0].id
    case_dir.mkdir()
    (case_dir / "trajectory.json").write_text(
        json.dumps(
            {
                "steps": [
                    {
                        "tool_calls": [
                            {
                                "function_name": "bash",
                                "arguments": {"command": command},
                            }
                        ]
                    }
                ]
            }
        )
    )
    monkeypatch.setattr(sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5])

    with pytest.raises(RuntimeError, match="shared /tmp path"):
        objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)


@pytest.mark.parametrize(
    ("trajectory_step", "violation"),
    [
        (
            {
                "tool_calls": [
                    {
                        "function_name": "bash",
                        "arguments": {"command": "print(inspect.getsource(helper))"},
                    }
                ]
            },
            "frozen helper source inspection",
        ),
        (
            {
                "tool_calls": [
                    {
                        "function_name": "read",
                        "arguments": {
                            "filePath": "dimos/evals/suites/sf_office_pose_preprocessing.py"
                        },
                    }
                ]
            },
            "frozen helper source inspection",
        ),
        (
            {
                "observation": {
                    "results": [{"content": "Traceback (most recent call last):\nValueError"}]
                }
            },
            "tool execution failure",
        ),
        (
            {
                "observation": {
                    "results": [{"content": "bash: syntax error near unexpected token `)'"}]
                }
            },
            "tool execution failure",
        ),
        (
            {"observation": {"results": [{"content": "jq: parse error: Invalid literal"}]}},
            "tool execution failure",
        ),
        (
            {"observation": {"results": [{"content": "Command timed out after 30 seconds"}]}},
            "tool execution failure",
        ),
    ],
)
def test_objective_rejects_trajectory_violations(
    trajectory_step: dict[str, object],
    violation: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    results = [
        EvalResult(case_id=case.id, score=1.0, final_answer="{}", duration_s=1.0) for case in SUITE
    ]
    case_dir = tmp_path / SUITE[0].id
    case_dir.mkdir()
    (case_dir / "trajectory.json").write_text(json.dumps({"steps": [trajectory_step]}))
    monkeypatch.setattr(sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5])

    with pytest.raises(RuntimeError, match=violation):
        objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)


def test_objective_rejects_case_errors(tmp_path: Path) -> None:
    results = [
        EvalResult(
            case_id=case.id,
            score=0.0,
            final_answer="",
            duration_s=1.0,
            error="failed",
        )
        for case in SUITE
    ]

    with pytest.raises(RuntimeError, match="cases failed"):
        objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)


def test_objective_rejects_repeated_pose_instead_of_ordered_coverage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    event = json.dumps(
        {
            "event": "agent_encode",
            "message_type": "PoseStamped",
            "output": {"source_timestamp_s": 12.5},
        }
    )
    results = [
        EvalResult(case_id=case.id, score=1.0, final_answer="{}", duration_s=1.0) for case in SUITE
    ]
    for case in SUITE:
        activity_dir = tmp_path / case.id / "agent-activity"
        activity_dir.mkdir(parents=True)
        (activity_dir / "events.jsonl").write_text(f"{event}\n{event}\n")
    monkeypatch.setattr(
        sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5, 13.0]
    )

    result = objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)

    assert set(result["activity_counts"].values()) == {2}
    assert result["score"] == 0.0
    assert result["evidence_completion_rate"] == 0.0


def test_objective_accepts_complete_ordered_coverage_after_exploratory_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def event(timestamp: float) -> str:
        return json.dumps(
            {
                "event": "agent_encode",
                "message_type": "PoseStamped",
                "output": {"source_timestamp_s": timestamp},
            }
        )

    results = [
        EvalResult(case_id=case.id, score=1.0, final_answer="{}", duration_s=1.0) for case in SUITE
    ]
    for case in SUITE:
        activity_dir = tmp_path / case.id / "agent-activity"
        activity_dir.mkdir(parents=True)
        (activity_dir / "events.jsonl").write_text(
            "\n".join([event(12.5), event(12.5), event(13.0)])
        )
    monkeypatch.setattr(
        sf_office_pose_autoresearch, "_expected_pose_timestamps", lambda: [12.5, 13.0]
    )

    result = objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)

    assert set(result["activity_counts"].values()) == {3}
    assert result["score"] == 1.0
    assert result["evidence_completion_rate"] == 1.0


def test_publish_result_writes_evo_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("EVO_RESULT_PATH", str(tmp_path / "result.json"))
    monkeypatch.setenv("EVO_TRACES_DIR", str(tmp_path / "traces"))
    monkeypatch.setenv("EVO_EXPERIMENT_ID", "exp_test")
    results = [
        EvalResult(case_id=case.id, score=0.85, final_answer="{}", ended_by="answer")
        for case in SUITE
    ]
    result = results[0]
    replicate = {
        "score": 0.85,
        "tasks": {case.id: 0.85 for case in SUITE},
        "benchmark_digest": EXPECTED_BENCHMARK_DIGEST,
        "activity_counts": {case.id: 8568 for case in SUITE},
    }
    payload = {
        "score": 0.85,
        "tasks": {case.id: 0.85 for case in SUITE},
        "benchmark_digest": EXPECTED_BENCHMARK_DIGEST,
        "activity_counts": {case.id: [8568, 8568, 8568] for case in SUITE},
        "replicates": [replicate, replicate, replicate],
    }

    publish_result([results, results, results], payload)

    assert json.loads((tmp_path / "result.json").read_text()) == payload
    trace = json.loads((tmp_path / "traces" / f"task_{result.case_id}.json").read_text())
    assert (trace["experiment_id"], trace["task_id"], trace["score"]) == (
        "exp_test",
        result.case_id,
        0.85,
    )
    assert trace["replicate_scores"] == [0.85, 0.85, 0.85]


def test_aggregate_objectives_reports_mean_and_variance() -> None:
    payloads = []
    for replicate, score in enumerate((0.6, 0.8, 1.0), start=1):
        payloads.append(
            {
                "score": score,
                "tasks": {case.id: score for case in SUITE},
                "category_scores": {},
                "completion_rate": 1.0,
                "evidence_completion_rate": 1.0,
                "error_count": 0,
                "activity_counts": {case.id: 8568 for case in SUITE},
                "benchmark_digest": EXPECTED_BENCHMARK_DIGEST,
                "run_dir": f"run-{replicate}",
                "replicate": replicate,
            }
        )

    result = aggregate_objectives(payloads)

    assert result["score"] == pytest.approx(0.8)
    assert result["score_stddev"] == pytest.approx(0.1632993162)
    assert result["replicate_scores"] == [0.6, 0.8, 1.0]
    assert result["evidence_completion_rate"] == 1.0
    assert all(score == pytest.approx(0.8) for score in result["tasks"].values())


def test_publish_result_refuses_an_existing_result_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    result_path = tmp_path / "result.json"
    result_path.write_text("existing")
    monkeypatch.setenv("EVO_RESULT_PATH", str(result_path))

    with pytest.raises(FileExistsError):
        publish_result([], {"benchmark_digest": EXPECTED_BENCHMARK_DIGEST})

    assert result_path.read_text() == "existing"
