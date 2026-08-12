# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np
import pytest

from dimos.benchmark.evaluation.models import ArtifactNativeResult
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.benchmark.vqa.evaluation import (
    MultipleChoiceVqaEvaluation,
    VqaEvaluationConfig,
    _parse_choice,
)


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


def test_vqa_evaluation_rejects_duplicate_label_ids(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    (dataset / "cases.jsonl").write_text(
        json.dumps({"id": "case-1", "image": "image.jpg", "question": "Q?", "choices": ["a", "b"]})
        + "\n"
    )
    label = json.dumps({"id": "case-1", "answer": "a"}) + "\n"
    (dataset / "labels.jsonl").write_text(label + label)
    workspace = tmp_path / "workspace"
    workspace.mkdir()

    with pytest.raises(ValueError, match="duplicate VQA label ID"):
        MultipleChoiceVqaEvaluation(lambda _: cast("Any", _VisionModel())).run(
            VqaEvaluationConfig(dataset=str(dataset)),
            EvaluationContext("run", tmp_path, workspace, cast("Any", None), None),
        )


def test_vqa_evaluation_normalizes_boolean_choices() -> None:
    assert _parse_choice("ANSWER: TRUE", ("yes", "no")) == "yes"
    assert _parse_choice("ANSWER: No", ("yes", "no")) == "no"
