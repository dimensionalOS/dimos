# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np

from dimos.benchmark.evaluation.models import ArtifactNativeResult
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.benchmark.vqa.evaluation import MultipleChoiceVqaEvaluation, VqaEvaluationConfig


class _VisionModel:
    def query(self, image: Any, prompt: str) -> str:
        assert "Choices: left, right." in prompt
        return "ANSWER: left"


def test_vqa_evaluation_uses_only_public_case_image_and_private_label(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    assert cv2.imwrite(str(dataset / "image.jpg"), np.zeros((1, 1, 3), dtype=np.uint8))
    (dataset / "cases.jsonl").write_text(
        json.dumps(
            {
                "id": "case-1",
                "image": "image.jpg",
                "question": "Which side?",
                "choices": ["left", "right"],
            }
        )
        + "\n"
    )
    (dataset / "labels.jsonl").write_text(json.dumps({"id": "case-1", "answer": "left"}) + "\n")
    workspace = tmp_path / "workspace"
    workspace.mkdir()

    report = MultipleChoiceVqaEvaluation(lambda _: cast("Any", _VisionModel())).run(
        VqaEvaluationConfig(dataset=str(dataset)),
        EvaluationContext("run", tmp_path, workspace, cast("Any", None), None),
    )

    assert report.summary[1].value == 1
    assert isinstance(report.native_result, ArtifactNativeResult)
    assert json.loads((workspace / "vqa-results.json").read_text())[0]["passed"] is True
