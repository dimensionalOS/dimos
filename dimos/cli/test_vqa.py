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

from collections.abc import Callable
from pathlib import Path
import sys
from typing import Any

from click import unstyle
import numpy as np
import pytest
from pytest_mock import MockerFixture
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
from dimos.evals.vqa.pointcloud_frame import PointCloudFrameUnavailableError
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


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
    assert "--quality-window" in output
    assert "--min-brightness" in output
    assert "--sharp-stride" not in output
    assert "--sync-tolerance" not in output
    assert "--output" in output
    assert "absent or" in output and "empty" in output


def test_vqa_run_cli_declares_standalone_dataset_input() -> None:
    result = CliRunner().invoke(app, ["evals", "vqa", "run", "--help"])
    output = unstyle(result.stdout)

    assert result.exit_code == 0
    assert "DATASET" in output
    assert "--model" in output


def test_vqa_generate_cli_runs_generation(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    seen: list[tuple[GenerationRequest, VqaGenerationConfig | None]] = []

    def fake_generate(
        request: GenerationRequest,
        config: VqaGenerationConfig | None = None,
        *,
        progress: Callable[[str], None] | None = None,
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
        request: GenerationRequest,
        config: VqaGenerationConfig | None = None,
        *,
        progress: Callable[[str], None] | None = None,
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


def test_vqa_generate_cli_accepts_quality_window_options(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    seen: list[GenerationRequest] = []

    def fake_generate(
        request: GenerationRequest,
        config: VqaGenerationConfig | None = None,
        *,
        progress: Callable[[str], None] | None = None,
    ) -> GenerationResult:
        seen.append(request)
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
            "0",
            "--stop",
            "100",
            "--quality-window",
            "0.5",
            "--min-brightness",
            "0.1",
            "--output",
            str(tmp_path),
        ],
    )

    assert result.exit_code == 0
    assert seen == [
        GenerationRequest(
            dataset="recording.db",
            start=0,
            stop=100,
            quality_window=0.5,
            min_brightness=0.1,
            output=tmp_path,
        )
    ]


def test_vqa_generate_cli_formats_expected_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_generate(
        request: GenerationRequest,
        config: VqaGenerationConfig | None = None,
        *,
        progress: Callable[[str], None] | None = None,
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
            assert agent.config.model == "test-model"
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


@pytest.mark.parametrize(
    ("selection", "indices"),
    [
        (["--start", "20", "--stop", "300", "--stride", "100"], (20, 120, 220)),
        (["--image-index", "20"], (20,)),
        (["--start", "0", "--stop", "300", "--quality-window", "0.5"], (12, 22)),
    ],
)
def test_vqa_generate_reports_progress_before_slow_work(
    selection: list[str], indices: tuple[int, ...], tmp_path: Path, mocker: MockerFixture
) -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    loader_factory = mocker.patch.object(generate_module, "PointCloudFrameLoader")
    loader = loader_factory.return_value.__enter__.return_value
    loader.image_count = 300
    loader.load_image.return_value = image
    mocker.patch.object(generate_module, "select_quality_indices", return_value=indices)
    author = mocker.patch("dimos.models.vl.openai.OpenAIVlModel").return_value
    author.config.model_name = "test-author"
    author.query_json.return_value = [{"family": "presence", "object_names": ["chair"]}]
    detector = mocker.patch("dimos.models.vl.moondream.MoondreamVlModel").return_value
    detector.config.model_name = "test-detector"
    detector.query_detections.return_value = ImageDetections2D(
        image,
        [
            Detection2DBBox(
                bbox=(0, 0, 2, 2),
                name="chair",
                image=image,
                track_id=1,
                class_id=1,
                confidence=1.0,
                ts=1.0,
            )
        ],
    )

    def open_recording(*args: Any, **kwargs: Any) -> Any:
        assert "Preparing VQA generation..." in sys.stdout.buffer.getvalue().decode()
        assert "Loading recording recording.db..." in sys.stdout.buffer.getvalue().decode()
        return loader_factory.return_value

    def load_frame(index: int) -> None:
        # Check the captured stream while work is running, not only after CLI completion.
        output = sys.stdout.buffer.getvalue().decode()
        assert f"Selected {len(indices)} frame(s)." in output
        assert "Initializing VQA models" in output
        number = indices.index(index) + 1
        assert f"Processing frame {number}/{len(indices)} (color_image index {index})" in output
        assert "Generated " not in output
        raise PointCloudFrameUnavailableError("no point cloud")

    loader_factory.side_effect = open_recording
    loader.load.side_effect = load_frame
    output = tmp_path / "vqa"
    result = CliRunner().invoke(
        app, ["evals", "vqa", "generate", "recording.db", *selection, "--output", str(output)]
    )

    assert result.exit_code == 0, result.output
    assert loader.load.call_args_list == [mocker.call(index) for index in indices]
    assert author.query_json.call_count == len(indices)
    assert f"Generated {len(indices)} VQA case(s) in {output}" in result.stdout
    assert (output / "cases.jsonl").is_file()
