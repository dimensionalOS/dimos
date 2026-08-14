# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest

from dimos.benchmark.vqa.evaluation import (
    MultipleChoiceVqaCase,
    _parse_choice,
    load_suite,
)
from dimos.evals.types import EvalCase, InteractiveEval


class _Rig:
    blind = False
    mcp_url = "http://localhost:9990/mcp"

    def __init__(self, answer: str = "ANSWER: left") -> None:
        self.answer = answer
        self.context: Sequence[dict[str, Any]] = ()
        self.question = ""

    def ask(self, context: Sequence[dict[str, Any]], question: str) -> str:
        self.context = context
        self.question = question
        return self.answer

    def open_dataset(self, name: str) -> Any:
        raise NotImplementedError

    def live_store(self) -> Any:
        raise NotImplementedError

    def encode(self, stream: Any) -> list[dict[str, Any]]:
        raise NotImplementedError

    def call_skill(self, name: str, args: Mapping[str, object]) -> str:
        raise NotImplementedError

    def agent_loop(self, case: EvalCase) -> str:
        raise NotImplementedError

    def mcp_ready(self) -> bool:
        return False

    def setup_env(self, case: InteractiveEval) -> None:
        raise NotImplementedError

    def check_env(self, case: InteractiveEval) -> None:
        raise NotImplementedError

    def instruct(self, text: str) -> None:
        raise NotImplementedError

    def sample(
        self, score: Callable[[Any], float], interval_s: float, timeout_s: float
    ) -> list[tuple[float, float]]:
        raise NotImplementedError


def _dataset(tmp_path: Path) -> Path:
    dataset = tmp_path / "dataset"
    assets = dataset / "assets"
    assets.mkdir(parents=True)
    assert cv2.imwrite(str(assets / "image.jpg"), np.zeros((1, 1, 3), dtype=np.uint8))
    (dataset / "cases.jsonl").write_text(
        json.dumps(
            {
                "id": "case-1",
                "image": "assets/image.jpg",
                "question": "Which side?",
                "choices": ["left", "right"],
            }
        )
        + "\n"
    )
    (dataset / "labels.jsonl").write_text(json.dumps({"id": "case-1", "answer": "left"}) + "\n")
    return dataset


def test_vqa_suite_uses_only_public_image_question_and_choices(tmp_path: Path) -> None:
    case = load_suite(_dataset(tmp_path))[0]
    assert isinstance(case, MultipleChoiceVqaCase)
    rig = _Rig()

    result = case.evaluate(rig)

    assert result.score == 1.0
    assert len(rig.context) == 1
    assert rig.context[0]["type"] == "image_url"
    assert "Choices: left, right." in rig.question
    assert "ANSWER: <choice>" in rig.question


def test_vqa_suite_runs_through_standard_eval_runner(tmp_path: Path) -> None:
    from langchain_core.language_models.fake_chat_models import FakeListChatModel

    from dimos.evals.runner import EvalRunner

    runner = EvalRunner(
        chat_model=FakeListChatModel(responses=["ANSWER: left"]),
        out_dir=tmp_path / "evals",
    )

    results = runner.run(load_suite(_dataset(tmp_path)))

    assert len(results) == 1
    assert results[0].passed
    assert results[0].outputs == "ANSWER: left"
    assert (runner.run_dir / "results.jsonl").is_file()
    assert (runner.run_dir / "summary.json").is_file()


def test_vqa_suite_rejects_duplicate_label_ids(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path)
    label = json.dumps({"id": "case-1", "answer": "left"}) + "\n"
    (dataset / "labels.jsonl").write_text(label + label)

    with pytest.raises(ValueError, match="duplicate VQA label ID"):
        load_suite(dataset)


def test_vqa_suite_rejects_images_outside_dataset(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path)
    (dataset / "cases.jsonl").write_text(
        json.dumps(
            {
                "id": "case-1",
                "image": "../private.jpg",
                "question": "Which side?",
                "choices": ["left", "right"],
            }
        )
        + "\n"
    )

    with pytest.raises(ValueError, match="escapes the dataset directory"):
        load_suite(dataset)


def test_vqa_evaluation_normalizes_boolean_choices() -> None:
    assert _parse_choice("ANSWER: TRUE", ("yes", "no")) == "yes"
    assert _parse_choice("ANSWER: No", ("yes", "no")) == "no"
