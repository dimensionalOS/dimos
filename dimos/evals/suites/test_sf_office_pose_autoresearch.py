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
    THINKING,
    _expected_pose_timestamps,
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
    result = EvalResult(
        case_id=SUITE[0].id,
        score=0.85,
        final_answer='{"remaining_distance_m": 0.22}',
        ended_by="answer",
    )
    payload = {
        "score": 0.85,
        "tasks": {result.case_id: 0.85},
        "benchmark_digest": EXPECTED_BENCHMARK_DIGEST,
        "activity_counts": {result.case_id: 8568},
    }

    publish_result([result], payload)

    assert json.loads((tmp_path / "result.json").read_text()) == payload
    trace = json.loads((tmp_path / "traces" / f"task_{result.case_id}.json").read_text())
    assert (trace["experiment_id"], trace["task_id"], trace["score"]) == (
        "exp_test",
        result.case_id,
        0.85,
    )


def test_publish_result_refuses_an_existing_result_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    result_path = tmp_path / "result.json"
    result_path.write_text("existing")
    monkeypatch.setenv("EVO_RESULT_PATH", str(result_path))

    with pytest.raises(FileExistsError):
        publish_result([], {"benchmark_digest": EXPECTED_BENCHMARK_DIGEST})

    assert result_path.read_text() == "existing"
