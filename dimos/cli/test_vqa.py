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
from typing import Any

from click import unstyle
import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main as app
from dimos.evals import runner as runner_module
from dimos.evals.types import EvalResult
from dimos.evals.vqa import generate as generate_module, suite as suite_module
from dimos.evals.vqa.generate import (
    GenerationRequest,
    GenerationResult,
    PublicCase,
    VqaGenerationConfig,
)


def test_vqa_cli_exposes_generate_and_run() -> None:
    result = CliRunner().invoke(app, ["evals", "vqa", "--help"])
    output = unstyle(result.stdout)

    assert result.exit_code == 0
    assert "generate" in output
    assert "run" in output


def test_vqa_generate_cli_declares_single_image_input() -> None:
    result = CliRunner().invoke(app, ["evals", "vqa", "generate", "--help"])
    output = unstyle(result.stdout)

    assert result.exit_code == 0
    assert "DATASET" in output
    assert "--image-index" in output
    assert "--start" in output
    assert "--stop" in output
    assert "--stride" in output
    assert "--sync-tolerance" not in output
    assert "--output" in output
    assert "absent or empty" in output


def test_vqa_run_cli_declares_standalone_dataset_input() -> None:
    result = CliRunner().invoke(app, ["evals", "vqa", "run", "--help"])
    output = unstyle(result.stdout)

    assert result.exit_code == 0
    assert "DATASET" in output
    assert "--model" in output


def test_vqa_generate_cli_runs_generation(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    seen: list[tuple[GenerationRequest, VqaGenerationConfig | None]] = []

    def fake_generate(
        request: GenerationRequest, config: VqaGenerationConfig | None = None
    ) -> GenerationResult:
        seen.append((request, config))
        return GenerationResult(
            output=request.output_directory(),
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
        [
            "evals",
            "vqa",
            "generate",
            "recording.db",
            "--image-index",
            "3",
            "--output",
            str(tmp_path),
        ],
    )

    assert result.exit_code == 0
    assert seen == [
        (
            GenerationRequest(dataset="recording.db", image_index=3, output=tmp_path),
            None,
        )
    ]
    assert "Generated 1 VQA case" in result.stdout


def test_vqa_generate_cli_accepts_frame_range(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    seen: list[tuple[GenerationRequest, VqaGenerationConfig | None]] = []

    def fake_generate(
        request: GenerationRequest, config: VqaGenerationConfig | None = None
    ) -> GenerationResult:
        seen.append((request, config))
        return GenerationResult(output=request.output_directory(), cases=())

    monkeypatch.setattr(generate_module, "generate_dataset", fake_generate)

    result = CliRunner().invoke(
        app,
        [
            "evals",
            "vqa",
            "generate",
            "recording.db",
            "--start",
            "2",
            "--stop",
            "9",
            "--stride",
            "3",
            "--output",
            str(tmp_path),
        ],
    )

    assert result.exit_code == 0
    assert seen == [
        (
            GenerationRequest(
                dataset="recording.db",
                start=2,
                stop=9,
                stride=3,
                output=tmp_path,
            ),
            None,
        )
    ]


def test_vqa_generate_cli_formats_expected_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_generate(
        request: GenerationRequest, config: VqaGenerationConfig | None = None
    ) -> GenerationResult:
        raise ValueError("dataset has no 'tf' stream")

    monkeypatch.setattr(generate_module, "generate_dataset", fake_generate)

    result = CliRunner().invoke(
        app,
        ["evals", "vqa", "generate", "recording.db", "--image-index", "0"],
    )

    assert result.exit_code != 0
    assert "dataset has no 'tf' stream" in result.output
    assert "Traceback" not in result.output


def test_vqa_run_cli_formats_dataset_errors(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    def fail_load(dataset: Path) -> tuple[()]:
        raise ValueError(f"invalid VQA dataset file: {dataset / 'cases.jsonl'}")

    monkeypatch.setattr(suite_module, "load_suite", fail_load)

    result = CliRunner().invoke(app, ["evals", "vqa", "run", str(tmp_path)])

    assert result.exit_code != 0
    assert "invalid VQA dataset file" in result.output
    assert "Traceback" not in result.output


def test_vqa_run_cli_runs_shared_evaluator(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    assets = tmp_path / "assets"
    assets.mkdir()
    (tmp_path / "cases.jsonl").write_text('{"id":"q","image":"assets/q.png"}\n')
    (tmp_path / "labels.jsonl").write_text('{"id":"q","answer":"yes"}\n')
    (assets / "q.png").write_bytes(b"image")

    class FakeRunner:
        def __init__(self, **kwargs: object) -> None:
            assert kwargs == {}
            self.run_dir = tmp_path / "results"

        def run(self, cases: object, agent: Any, *, provenance: dict[str, Any]) -> list[EvalResult]:
            assert cases == ("case",)
            assert agent.model == "test-model"
            assert type(agent).__name__ == "QuestionAnswer"
            assert provenance["source"]["kind"] == "vqa_dataset"
            assert provenance["source"]["path"] == str(tmp_path.resolve())
            assert set(provenance["source"]["fingerprints"]) == {
                "cases.jsonl",
                "labels.jsonl",
                "assets/q.png",
            }
            assert provenance["agent"] == {
                "module": "dimos.evals.agents.question_answer",
                "kwargs": {"model": "test-model"},
            }
            return [EvalResult(case_id="q", final_answer="yes", score=1.0, passed=True)]

    monkeypatch.setattr(suite_module, "load_suite", lambda dataset: ("case",))
    monkeypatch.setattr(runner_module, "EvalRunner", FakeRunner)

    result = CliRunner().invoke(
        app,
        ["evals", "vqa", "run", str(tmp_path), "--model", "test-model"],
    )

    assert result.exit_code == 0
    assert "PASS" in result.stdout
    assert "mean 1.00" in result.stdout


def test_vqa_source_fingerprints_change_with_reload_inputs(tmp_path: Path) -> None:
    assets = tmp_path / "assets"
    assets.mkdir()
    (tmp_path / "cases.jsonl").write_text("cases")
    (tmp_path / "labels.jsonl").write_text("labels")
    image = assets / "frame.png"
    image.write_bytes(b"first")

    before = suite_module.source_record(tmp_path)
    image.write_bytes(b"second")
    after = suite_module.source_record(tmp_path)

    assert before["path"] == str(tmp_path.resolve())
    assert set(before["fingerprints"]) == {"cases.jsonl", "labels.jsonl", "assets/frame.png"}
    assert before["fingerprints"]["assets/frame.png"] != after["fingerprints"]["assets/frame.png"]
