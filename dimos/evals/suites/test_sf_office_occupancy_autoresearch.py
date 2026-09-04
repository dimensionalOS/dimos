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

import json
from pathlib import Path

import pytest

from dimos.evals.suites.sf_office_occupancy_autoresearch import (
    EXPECTED_BENCHMARK_DIGEST,
    MAX_STEPS,
    MODEL,
    THINKING,
    benchmark_agent,
    benchmark_digest,
    objective,
    publish_result,
    verify_benchmark,
)
from dimos.evals.suites.sf_office_occupancy_research import SUITE
from dimos.evals.types import EvalResult


def test_benchmark_profile_is_fixed_and_requires_a_timely_answer() -> None:
    agent = benchmark_agent()

    assert (agent.model, agent.thinking, agent.max_steps) == (MODEL, THINKING, MAX_STEPS)
    assert agent.max_steps == 40
    assert tuple(agent.tools) == ("read", "bash")
    assert "Use as many small or large tool calls" in agent.instructions
    assert "manage the wall-clock deadline" in agent.instructions
    assert "return your best estimate" in agent.instructions


def test_frozen_benchmark_digest_matches_files() -> None:
    assert verify_benchmark() == EXPECTED_BENCHMARK_DIGEST
    assert benchmark_digest() == EXPECTED_BENCHMARK_DIGEST


def test_digest_changes_with_file_content(tmp_path: Path) -> None:
    left = tmp_path / "left"
    right = tmp_path / "right"
    left.write_text("same")
    right.write_text("same")
    initial = benchmark_digest((left, right))

    right.write_text("changed")

    assert benchmark_digest((left, right)) != initial


def test_objective_emits_only_aggregate_feedback(tmp_path: Path) -> None:
    results = [
        EvalResult(case_id=case.id, score=0.5, final_answer="{}", duration_s=1.0)
        for case in SUITE
    ]

    result = objective(results, tmp_path, EXPECTED_BENCHMARK_DIGEST)

    assert result["score"] == 0.5
    assert result["category_scores"] == {
        "semantics": 0.5,
        "temporal": 0.5,
        "routing": 0.5,
        "geometry": 0.5,
    }
    assert result["completion_rate"] == 1.0
    assert result["error_count"] == 0
    assert result["tasks"] == {case.id: 0.5 for case in SUITE}


def test_publish_result_writes_evo_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("EVO_RESULT_PATH", str(tmp_path / "result.json"))
    monkeypatch.setenv("EVO_TRACES_DIR", str(tmp_path / "traces"))
    monkeypatch.setenv("EVO_EXPERIMENT_ID", "exp_test")
    result = EvalResult(
        case_id=SUITE[0].id,
        score=0.85,
        final_answer='{"room_count": 3}',
        ended_by="answer",
    )
    payload = {"score": 0.85, "tasks": {result.case_id: 0.85}}

    publish_result([result], payload)

    assert json.loads((tmp_path / "result.json").read_text()) == payload
    trace = json.loads((tmp_path / "traces" / f"task_{result.case_id}.json").read_text())
    assert (trace["experiment_id"], trace["task_id"], trace["score"]) == (
        "exp_test",
        result.case_id,
        0.85,
    )
