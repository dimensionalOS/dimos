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

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.evals.types import EvalRig
from dimos.evals.vqa.author import OpenAIQuestionAuthor, QuestionAuthor
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    FamilyAnswer,
    FamilySpec,
    QuestionProposal,
    answer_question,
)
from dimos.evals.vqa.generate import (
    GenerationRequest,
    PrivateLabel,
    PublicCase,
    generate_image_dataset,
    load_memory_image,
)
from dimos.evals.vqa.suite import load_suite
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.models.vl.base import VlModel


class _Author:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (QuestionProposal(family="presence", object_name="chair"),)


class _Detector:
    def __init__(self, present: bool) -> None:
        self._present = present

    def detect(self, image: Image, object_name: str) -> ImageDetections2D:
        detections = []
        if self._present:
            detections.append(
                Detection2DBBox(
                    bbox=(0.0, 0.0, 2.0, 2.0),
                    track_id=0,
                    class_id=-1,
                    confidence=1.0,
                    name=object_name,
                    ts=image.ts,
                    image=image,
                )
            )
        return ImageDetections2D(image, detections)


class _QuestionModel:
    def query_json(self, image: Image, prompt: str) -> dict[str, str]:
        assert "presence" in prompt
        return {"family": "presence", "object_name": "chair"}


class _Rig:
    blind = False

    def ask(self, context: object, question: str) -> str:
        assert context
        assert 'Choices: ["yes", "no"]' in question
        return "ANSWER: yes"


def test_presence_proposal_matches_available_family() -> None:
    proposal = QuestionProposal(family="presence", object_name="  chair  ")

    assert proposal.object_name == "chair"
    assert [family.name for family in AVAILABLE_FAMILIES] == [proposal.family]


def test_question_proposal_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError):
        QuestionProposal.model_validate(
            {"family": "presence", "object_name": "chair", "answer": "yes"}
        )


def test_family_answer_requires_an_allowed_choice() -> None:
    with pytest.raises(ValidationError, match="answer must be one of the choices"):
        FamilyAnswer(
            question="Is there a chair in the image?",
            choices=("yes", "no"),
            answer="unknown",
        )


def test_presence_family_derives_answer_from_detector_evidence() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="presence", object_name="chair")

    present = answer_question(proposal, image, _Detector(present=True))

    assert present.answer == "yes"
    assert present.question == "Does the image contain any chair?"
    assert present.choices == ("yes", "no")
    assert present.evidence["detection_count"] == 1
    with pytest.raises(ValueError, match="did not confirm"):
        answer_question(proposal, image, _Detector(present=False))


def test_openai_author_parses_one_constrained_proposal() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(cast("VlModel", _QuestionModel()))

    proposals = author.propose(image, AVAILABLE_FAMILIES)

    assert proposals == (QuestionProposal(family="presence", object_name="chair"),)


def test_generation_request_selects_one_memory_image() -> None:
    request = GenerationRequest(dataset="go2_short", image_index=4, output=Path("dataset"))

    assert request.image_index == 4
    assert request.output == Path("dataset")


def test_load_memory_image_selects_by_stream_index(tmp_path: Path) -> None:
    dataset = tmp_path / "recording.db"
    store = SqliteStore(path=str(dataset))
    try:
        images = store.stream("color_image", Image)
        images.append(Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8)), ts=1.0)
        images.append(Image.from_numpy(np.full((2, 2, 3), 42, dtype=np.uint8)), ts=2.0)
    finally:
        store.stop()

    image = load_memory_image(str(dataset), 1)

    assert image.ts == 2.0
    assert np.all(image.data == 42)


def test_standalone_rows_match_public_private_contract() -> None:
    case = PublicCase(
        id="frame-000004-chair-presence",
        image="assets/frame-000004.jpg",
        question="Is there a chair in the image?",
        choices=("yes", "no"),
    )
    label = PrivateLabel(id=case.id, answer="yes")

    assert case.model_dump(mode="json") == {
        "id": "frame-000004-chair-presence",
        "image": "assets/frame-000004.jpg",
        "question": "Is there a chair in the image?",
        "choices": ["yes", "no"],
    }
    assert label.model_dump(mode="json") == {
        "id": "frame-000004-chair-presence",
        "answer": "yes",
    }


def test_generate_and_evaluate_one_image(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_image_dataset(
        request,
        image,
        cast("QuestionAuthor", _Author()),
        _Detector(present=True),
    )

    assert result.cases[0].id == "frame-000004-chair-presence"
    assert (output / "assets" / "frame-000004.jpg").is_file()
    assert (output / "cases.jsonl").is_file()
    assert (output / "labels.jsonl").is_file()
    assert (output / "audit" / "frame-000004" / "ground_truth.json").is_file()

    suite = load_suite(output)
    evaluation = suite[0].evaluate(cast("EvalRig", _Rig()))

    assert evaluation.case_id == "frame-000004-chair-presence"
    assert evaluation.score == 1.0
