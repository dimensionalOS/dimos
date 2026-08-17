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

from pathlib import Path

import pytest
from typer.testing import CliRunner

from dimos.evals import runner as runner_module
from dimos.evals.cli import app
from dimos.evals.types import EvalResult
from dimos.evals.vqa import generate as generate_module, suite as suite_module
from dimos.evals.vqa.generate import GenerationRequest, GenerationResult, PublicCase


def test_vqa_cli_exposes_generate_and_run() -> None:
    result = CliRunner().invoke(app, ["vqa", "--help"])

    assert result.exit_code == 0
    assert "generate" in result.stdout
    assert "run" in result.stdout


def test_vqa_generate_cli_declares_single_image_input() -> None:
    result = CliRunner().invoke(app, ["vqa", "generate", "--help"])

    assert result.exit_code == 0
    assert "DATASET" in result.stdout
    assert "--image-index" in result.stdout
    assert "--output" in result.stdout


def test_vqa_run_cli_declares_standalone_dataset_input() -> None:
    result = CliRunner().invoke(app, ["vqa", "run", "--help"])

    assert result.exit_code == 0
    assert "DATASET" in result.stdout
    assert "--model" in result.stdout


def test_vqa_generate_cli_runs_generation(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    seen: list[GenerationRequest] = []

    def fake_generate(request: GenerationRequest) -> GenerationResult:
        seen.append(request)
        return GenerationResult(
            output=request.output,
            cases=(
                PublicCase(
                    id="q",
                    image="assets/frame.jpg",
                    question="Is there a chair?",
                    choices=("yes", "no"),
                ),
            ),
        )

    monkeypatch.setattr(generate_module, "generate_dataset", fake_generate)

    result = CliRunner().invoke(
        app,
        ["vqa", "generate", "recording.db", "--image-index", "3", "--output", str(tmp_path)],
    )

    assert result.exit_code == 0
    assert seen == [GenerationRequest(dataset="recording.db", image_index=3, output=tmp_path)]
    assert "Generated 1 VQA case" in result.stdout


def test_vqa_run_cli_runs_shared_evaluator(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    class FakeRunner:
        def __init__(self, **kwargs: object) -> None:
            assert kwargs == {"model": "test-model"}
            self.run_dir = tmp_path / "results"

        def run(self, cases: object) -> list[EvalResult]:
            assert cases == ("case",)
            return [EvalResult(case_id="q", outputs="yes", score=1.0, passed=True)]

    monkeypatch.setattr(suite_module, "load_suite", lambda dataset: ("case",))
    monkeypatch.setattr(runner_module, "EvalRunner", FakeRunner)

    result = CliRunner().invoke(
        app,
        ["vqa", "run", str(tmp_path), "--model", "test-model"],
    )

    assert result.exit_code == 0
    assert "PASS" in result.stdout
    assert "mean 1.00" in result.stdout
