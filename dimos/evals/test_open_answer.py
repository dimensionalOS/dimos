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

"""Open-phrase grading: the 1-5 map, the calibration set, and the run manifest."""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.agents.base import EvalSubject
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.open_answer import (
    JUDGE_MODEL,
    OPEN_ANSWER_PASS,
    OPEN_EQA_RUBRIC,
    agreement,
    credit,
    grade_open_answer,
    load_checks,
    mark_of,
)
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.go2_open import SUITE
from dimos.evals.types import EvalCase, RunningEnvironment, Trajectory


def test_credit_maps_the_openeqa_scale() -> None:
    assert [credit(mark) for mark in (1, 2, 3, 4, 5)] == [0.0, 0.25, 0.5, 0.75, 1.0]
    assert credit(4) == OPEN_ANSWER_PASS
    assert credit(3) < OPEN_ANSWER_PASS
    with pytest.raises(ValueError, match="1, 2, 3, 4, or 5"):
        mark_of(0)
    with pytest.raises(ValueError, match="1, 2, 3, 4, or 5"):
        credit(3.5)


def test_grade_sends_the_question_and_maps_four_to_the_pass_line(
    mocker: MockerFixture,
) -> None:
    seen: dict[str, Any] = {}

    def evaluate(*, inputs: str, outputs: str, reference_outputs: str) -> dict[str, float]:
        seen["triple"] = (inputs, outputs, reference_outputs)
        return {"score": 4}

    def factory(**kwargs: Any) -> Any:
        seen["factory"] = kwargs
        return evaluate

    mocker.patch("openevals.llm.create_llm_as_judge", side_effect=factory)
    grade = grade_open_answer("Which furniture is most numerous?", "chairs")
    outcome = SimpleNamespace(trajectory=SimpleNamespace(final_answer="office chairs"))

    assert grade(outcome) == OPEN_ANSWER_PASS
    assert seen["triple"] == ("Which furniture is most numerous?", "office chairs", "chairs")
    assert seen["factory"]["model"] == JUDGE_MODEL
    assert seen["factory"]["choices"] == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert seen["factory"]["prompt"] == OPEN_EQA_RUBRIC
    assert "{inputs}" in OPEN_EQA_RUBRIC
    assert "{outputs}" in OPEN_EQA_RUBRIC
    assert "{reference_outputs}" in OPEN_EQA_RUBRIC


def test_calibration_covers_paraphrases_and_wrong_answers() -> None:
    checks = load_checks()
    assert len(checks) >= 30
    humans = {check.human for check in checks}
    assert {1, 5} <= humans
    assert agreement(checks, [check.human for check in checks]) == 1.0
    assert agreement(checks, [1 for _ in checks]) < 0.5

    suite_questions = {case.inputs.splitlines()[0] for case in SUITE}
    assert suite_questions <= {check.question for check in checks}
    for case in SUITE:
        assert "There are no choices." in case.inputs
        assert "Choices:" not in case.inputs
        assert case.threshold == OPEN_ANSWER_PASS
        assert case.judge_model == JUDGE_MODEL
        assert case.tags & {"open"}
    assert {letter for case in SUITE for letter in case.tags if letter in "DFGL"} == {
        "D",
        "F",
        "G",
        "L",
    }


class _Quiet(Environment):
    def start(self, modules: Sequence[str], *, requires_mcp: bool = False) -> RunningEnvironment:
        return RunningEnvironment(mcp_url="", streams=(), artifacts={})


class _Reply(EvalSubject):
    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        built = TrajectoryBuilder(inputs, name="fake", model="fake")
        built.step(message="chairs", request=run_dir / "r", response=run_dir / "s")
        return built.build("answer")


def test_run_manifest_records_the_judge_model(tmp_path: Path, mocker: MockerFixture) -> None:
    mocker.patch(
        "openevals.llm.create_llm_as_judge",
        return_value=lambda **kwargs: {"score": 5},
    )
    case = EvalCase(
        id="open_chairs",
        inputs="Which furniture?\nAnswer in a short phrase. There are no choices.",
        environment=_Quiet(),
        grade=grade_open_answer("Which furniture?", "chairs"),
        threshold=OPEN_ANSWER_PASS,
        judge_model=JUDGE_MODEL,
    )
    runner = EvalRunner(out_dir=tmp_path)
    result = runner.run([case], _Reply())[0]

    assert result.passed and result.score == 1.0
    manifest = json.loads((runner.run_dir / "manifest.json").read_text())
    assert manifest["runner"]["judge_models"] == [JUDGE_MODEL]
    assert manifest["runner"]["strict"] is False
