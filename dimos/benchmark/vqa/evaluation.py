# Copyright 2026 Dimensional Inc.
"""Multiple-choice VQA evaluation plugin using public image artifacts only."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import re

import cv2
from pydantic import BaseModel, ConfigDict, Field

from dimos.benchmark.evaluation.models import (
    ArtifactNativeResult,
    ArtifactReference,
    EvaluationReport,
    SummaryItem,
)
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image


class VqaEvaluationConfig(BaseModel):
    """Location and vision model for a generated VQA evaluation dataset."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    dataset: str = Field(min_length=1)
    model: str = Field(default="gpt-4o-mini", min_length=1)


class MultipleChoiceVqaEvaluation:
    """Score a vision model against generated image-question-choice VQA cases."""

    name = "point-cloud-vqa"
    config_model: type[BaseModel] = VqaEvaluationConfig

    def __init__(self, vision_factory: Callable[[str], OpenAIVlModel] | None = None) -> None:
        self._vision_factory = vision_factory or (lambda model: OpenAIVlModel(model_name=model))

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        if not isinstance(config, VqaEvaluationConfig):
            raise TypeError("point-cloud-vqa received the wrong configuration type")
        dataset = Path(config.dataset).expanduser()
        if not dataset.is_absolute():
            dataset = context.spec_dir / dataset
        dataset = dataset.resolve()
        cases = _load_jsonl(dataset / "cases.jsonl")
        labels = {item["id"]: item["answer"] for item in _load_jsonl(dataset / "labels.jsonl")}
        model = self._vision_factory(config.model)
        results = [_evaluate_case(dataset, model, case, labels) for case in cases]
        artifact = context.workspace / "vqa-results.json"
        artifact.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        passed = sum(item["passed"] for item in results)
        total = len(results)
        return EvaluationReport(
            summary=(
                SummaryItem(key="cases", label="Cases", value=total),
                SummaryItem(key="passed", label="Passed", value=passed),
                SummaryItem(
                    key="accuracy", label="Accuracy", value=passed / total if total else 0.0
                ),
            ),
            native_result=ArtifactNativeResult(
                artifact=ArtifactReference(
                    path=artifact.relative_to(context.workspace).as_posix(),
                    label="VQA case results",
                    media_type="application/json",
                )
            ),
            artifacts=(
                ArtifactReference(
                    path=artifact.relative_to(context.workspace).as_posix(),
                    label="VQA case results",
                    media_type="application/json",
                ),
            ),
        )


def _evaluate_case(
    dataset: Path, model: OpenAIVlModel, case: dict[str, object], labels: dict[str, str]
) -> dict[str, object]:
    case_id = _required_string(case, "id")
    choices = _choices(case)
    expected = labels.get(case_id)
    if expected is None:
        raise ValueError(f"missing private label for case {case_id}")
    if expected not in choices:
        raise ValueError(f"private label for {case_id} is not an allowed choice")
    image = cv2.imread(str(dataset / _required_string(case, "image")))
    if image is None:
        raise ValueError(f"unable to load public image for case {case_id}")
    prompt = (
        f"{_required_string(case, 'question')}\n\n"
        f"Choices: {', '.join(choices)}.\n"
        "Use only the supplied image. End with exactly `ANSWER: <choice>`."
    )
    response = model.query(Image.from_numpy(image), prompt)
    answer = _parse_choice(response, choices)
    return {
        "id": case_id,
        "expected": expected,
        "answer": answer,
        "passed": answer == expected,
        "raw_response": response,
    }


def _load_jsonl(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        raise ValueError(f"missing VQA dataset file: {path}")
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _required_string(item: dict[str, object], key: str) -> str:
    value = item.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"VQA case requires non-empty {key}")
    return value


def _choices(case: dict[str, object]) -> tuple[str, ...]:
    value = case.get("choices")
    if (
        not isinstance(value, list)
        or len(value) < 2
        or not all(isinstance(item, str) for item in value)
    ):
        raise ValueError("VQA case requires at least two string choices")
    return tuple(value)


def _parse_choice(response: str, choices: tuple[str, ...]) -> str | None:
    match = re.search(r"^ANSWER:\s*(.+?)\s*$", response, re.MULTILINE)
    if match is None:
        return None
    answer = match.group(1)
    return answer if answer in choices else None


point_cloud_vqa = MultipleChoiceVqaEvaluation()
