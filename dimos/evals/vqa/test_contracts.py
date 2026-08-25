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

from collections.abc import Sequence
import json
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.constants import STATE_DIR
from dimos.evals.types import EvalRig
from dimos.evals.vqa.author import OpenAIQuestionAuthor, QuestionAuthor
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    FamilyAnswer,
    FamilySpec,
    InsufficientEvidenceError,
    QuestionProposal,
    answer_question,
)
from dimos.evals.vqa.generate import (
    GenerationRequest,
    PrivateLabel,
    PublicCase,
    generate_frames_dataset,
)
from dimos.evals.vqa.suite import load_suite
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel


class _Author:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (QuestionProposal(family="presence", object_name="chair"),)


class _MixedAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(family="horizontal_direction", object_name="chair"),
            QuestionProposal(family="presence", object_name="chair"),
        )


class _EmptyThenAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        if image.ts == 1.0:
            return ()
        return (QuestionProposal(family="presence", object_name="chair"),)


class _CollidingAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(family="presence", object_name="Chair"),
            QuestionProposal(family="presence", object_name="chair"),
            QuestionProposal(family="presence", object_name="Chair"),
        )


class _Detector:
    def __init__(self, present: bool) -> None:
        self._present = present

    def query_detections(self, image: Image, query: str) -> ImageDetections2D[Detection2DBBox]:
        detections = []
        if self._present:
            detections.append(
                Detection2DBBox(
                    bbox=(0.0, 0.0, 2.0, 2.0),
                    track_id=0,
                    class_id=-1,
                    confidence=1.0,
                    name=query,
                    ts=image.ts,
                    image=image,
                )
            )
        return ImageDetections2D(image, detections)


class _BoxesDetector:
    def __init__(self, boxes: tuple[tuple[float, float, float, float], ...]) -> None:
        self._boxes = boxes

    def query_detections(self, image: Image, query: str) -> ImageDetections2D[Detection2DBBox]:
        return ImageDetections2D(
            image,
            [
                Detection2DBBox(
                    bbox=box,
                    track_id=index,
                    class_id=-1,
                    confidence=1.0,
                    name=query,
                    ts=image.ts,
                    image=image,
                )
                for index, box in enumerate(self._boxes)
            ],
        )


class _QuestionModel:
    def query_json(self, image: Image, prompt: str) -> list[dict[str, str]]:
        assert "presence" in prompt
        assert "horizontal_direction" in prompt
        assert "object_count" in prompt
        assert "JSON array" in prompt
        return [
            {"family": "presence", "object_name": "chair"},
            {"family": "horizontal_direction", "object_name": "robot"},
            {"family": "object_count", "object_name": "box"},
        ]


class _PartlyInvalidQuestionModel:
    def query_json(self, image: Image, prompt: str) -> list[object]:
        return [
            {"family": "presence", "object_name": "chair"},
            {"family": "unsupported", "object_name": "table"},
            {"family": "presence", "object_name": "door", "answer": "yes"},
        ]


class _BrokenDetector:
    def query_detections(self, image: Image, query: str) -> ImageDetections2D[Detection2DBBox]:
        raise ValueError("detector broke")


class _FailsSecondDetector(_Detector):
    def __init__(self) -> None:
        super().__init__(present=True)
        self._calls = 0

    def query_detections(self, image: Image, query: str) -> ImageDetections2D[Detection2DBBox]:
        self._calls += 1
        if self._calls == 2:
            raise ValueError("detector broke")
        return super().query_detections(image, query)


class _Rig:
    blind = False

    def __init__(self, answer: str = "yes") -> None:
        self.answer = answer

    def ask(self, context: object, question: str) -> str:
        assert context
        assert 'Choices: ["yes", "no"]' in question
        return self.answer

    def ask_structured(self, context: object, question: str, schema: type[object]) -> object:
        assert context
        assert 'Choices: ["yes", "no"]' in question
        return schema(answer=self.answer)  # type: ignore[call-arg, return-value]


def test_presence_proposal_matches_available_family() -> None:
    proposal = QuestionProposal(family="presence", object_name="  chair  ")

    assert proposal.object_name == "chair"
    assert proposal.family in [family.name for family in AVAILABLE_FAMILIES]


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


def test_family_answer_requires_case_insensitive_unique_choices() -> None:
    with pytest.raises(ValidationError, match="choices must be unique"):
        FamilyAnswer(
            question="Is there a chair in the image?",
            choices=("yes", "YES"),
            answer="yes",
        )


def test_presence_family_derives_answer_from_detector_evidence() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="presence", object_name="chair")

    present = answer_question(proposal, image, cast("VlModel", _Detector(present=True)))

    assert present.answer == "yes"
    assert present.question == "Does the image contain any chair?"
    assert present.choices == ("yes", "no")
    assert present.evidence["detection_count"] == 1
    with pytest.raises(ValueError, match="did not confirm"):
        answer_question(proposal, image, cast("VlModel", _Detector(present=False)))


@pytest.mark.parametrize(
    ("box", "expected"),
    (
        ((0.0, 0.0, 20.0, 20.0), "left"),
        ((20.0, 0.0, 40.0, 20.0), "center"),
        ((35.0, 0.0, 55.0, 20.0), "center"),
        ((50.0, 0.0, 70.0, 20.0), "right"),
        ((70.0, 0.0, 90.0, 20.0), "right"),
    ),
)
def test_horizontal_direction_uses_detection_center(
    box: tuple[float, float, float, float], expected: str
) -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="horizontal_direction", object_name="robot")

    answer = answer_question(proposal, image, cast("VlModel", _BoxesDetector((box,))))

    assert answer.answer == expected
    assert answer.choices == ("left", "center", "right")
    assert answer.evidence["detection_count"] == 1


def test_horizontal_direction_rejects_ambiguous_instances() -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="horizontal_direction", object_name="chair")
    detector = cast(
        "VlModel",
        _BoxesDetector(
            (
                (0.0, 0.0, 20.0, 20.0),
                (70.0, 0.0, 90.0, 20.0),
            )
        ),
    )

    with pytest.raises(ValueError, match="exactly one"):
        answer_question(proposal, image, detector)


@pytest.mark.parametrize(
    ("count", "expected"),
    ((1, "one"), (2, "two"), (3, "three"), (4, "four or more"), (5, "four or more")),
)
def test_object_count_buckets_detected_instances(count: int, expected: str) -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_count", object_name="box")
    boxes = tuple((float(index), 0.0, float(index + 1), 1.0) for index in range(count))

    answer = answer_question(proposal, image, cast("VlModel", _BoxesDetector(boxes)))

    assert answer.answer == expected
    assert answer.question == "How many instances of box are visible in the image?"
    assert answer.choices == ("one", "two", "three", "four or more")
    assert answer.evidence["detection_count"] == count


def test_object_count_requires_at_least_one_detection() -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_count", object_name="box")

    with pytest.raises(InsufficientEvidenceError, match="to count"):
        answer_question(proposal, image, cast("VlModel", _BoxesDetector(())))


def test_openai_author_parses_constrained_proposals() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(cast("VlModel", _QuestionModel()))

    proposals = author.propose(image, AVAILABLE_FAMILIES)

    assert proposals == (
        QuestionProposal(family="presence", object_name="chair"),
        QuestionProposal(family="horizontal_direction", object_name="robot"),
        QuestionProposal(family="object_count", object_name="box"),
    )


def test_openai_author_keeps_valid_items_from_partly_invalid_response() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(cast("VlModel", _PartlyInvalidQuestionModel()))

    proposals = author.propose(image, AVAILABLE_FAMILIES)

    assert proposals == (QuestionProposal(family="presence", object_name="chair"),)


def test_generation_request_selects_one_memory_image() -> None:
    request = GenerationRequest(dataset="go2_short", image_index=4, output=Path("dataset"))

    assert request.image_index == 4
    assert request.output == Path("dataset")
    assert request.frame_indices() == range(4, 5)


def test_generation_request_defaults_output_under_state_directory() -> None:
    request = GenerationRequest(dataset="recordings/go2_short.db", image_index=4)

    assert request.output_directory() == STATE_DIR / "datasets" / "vqa" / "go2_short-frames"


def test_generation_request_selects_frame_range() -> None:
    request = GenerationRequest(
        dataset="go2_short",
        start=2,
        stop=10,
        stride=3,
        output=Path("dataset"),
    )

    assert request.frame_indices() == range(2, 10, 3)


@pytest.mark.parametrize(
    "selection",
    (
        {},
        {"image_index": 1, "start": 0, "stop": 2},
        {"start": 2, "stop": 2},
        {"start": 0},
    ),
)
def test_generation_request_rejects_invalid_selection(selection: dict[str, int]) -> None:
    with pytest.raises(ValidationError):
        GenerationRequest(dataset="go2_short", output=Path("dataset"), **selection)


def test_standalone_rows_match_public_private_contract() -> None:
    case = PublicCase(
        id="frame-000004-chair-presence",
        image="assets/frame-000004.png",
        question="Is there a chair in the image?",
        choices=("yes", "no"),
    )
    label = PrivateLabel(id=case.id, answer="yes")

    assert case.model_dump(mode="json") == {
        "id": "frame-000004-chair-presence",
        "image": "assets/frame-000004.png",
        "question": "Is there a chair in the image?",
        "choices": ["yes", "no"],
    }
    assert label.model_dump(mode="json") == {
        "id": "frame-000004-chair-presence",
        "answer": "yes",
    }


@pytest.mark.parametrize("choices", (("yes",), ("yes", "YES")))
def test_public_case_requires_multiple_case_insensitive_unique_choices(
    choices: tuple[str, ...],
) -> None:
    with pytest.raises(ValidationError, match="at least two unique choices"):
        PublicCase(
            id="q",
            image="frame.png",
            question="Is there a chair?",
            choices=choices,
        )


def test_suite_loads_jsonl_without_reading_whole_files(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    (tmp_path / "frame.png").write_bytes(b"image")
    (tmp_path / "cases.jsonl").write_text(
        "\n"
        + json.dumps(
            {
                "id": "q",
                "image": "frame.png",
                "question": "Is there a chair?",
                "choices": ["yes", "no"],
            }
        )
        + "\n\n"
    )
    (tmp_path / "labels.jsonl").write_text(json.dumps({"id": "q", "answer": "yes"}) + "\n")

    def fail_read_text(*args: object, **kwargs: object) -> str:
        raise AssertionError("load_suite must stream JSONL instead of reading the whole file")

    monkeypatch.setattr(Path, "read_text", fail_read_text)

    suite = load_suite(tmp_path)

    assert len(suite) == 1
    assert suite[0].id == "q"


def test_suite_rejects_malformed_jsonl(tmp_path: Path) -> None:
    (tmp_path / "labels.jsonl").write_text("")
    (tmp_path / "cases.jsonl").write_text("{not-json}\n")

    with pytest.raises(ValueError, match="invalid VQA dataset file"):
        load_suite(tmp_path)


def test_suite_rejects_case_insensitive_duplicate_choices(tmp_path: Path) -> None:
    case = {
        "id": "q",
        "image": "missing.png",
        "question": "Is there a chair?",
        "choices": ["yes", "YES"],
    }
    (tmp_path / "cases.jsonl").write_text(json.dumps(case) + "\n")
    (tmp_path / "labels.jsonl").write_text(json.dumps({"id": "q", "answer": "yes"}) + "\n")

    with pytest.raises(ValueError, match="at least two unique choices"):
        load_suite(tmp_path)


def test_suite_validates_all_case_ids_before_images(tmp_path: Path) -> None:
    case = {
        "id": "duplicate",
        "image": "missing.png",
        "question": "Is there a chair?",
        "choices": ["yes", "no"],
    }
    (tmp_path / "cases.jsonl").write_text(f"{json.dumps(case)}\n{json.dumps(case)}\n")
    (tmp_path / "labels.jsonl").write_text(json.dumps({"id": "duplicate", "answer": "yes"}) + "\n")

    with pytest.raises(ValueError, match="duplicate case IDs"):
        load_suite(tmp_path)


def test_suite_reports_empty_cases_before_duplicate_labels(tmp_path: Path) -> None:
    label = json.dumps({"id": "duplicate", "answer": "yes"})
    (tmp_path / "cases.jsonl").write_text("")
    (tmp_path / "labels.jsonl").write_text(f"{label}\n{label}\n")

    with pytest.raises(ValueError, match="contains no cases"):
        load_suite(tmp_path)


def test_generate_and_evaluate_one_image(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.arange(48, dtype=np.uint8).reshape(4, 4, 3), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        ((4, image),),
        cast("QuestionAuthor", _Author()),
        cast("VlModel", _Detector(present=True)),
    )

    assert result.cases[0].id == "frame-000004-chair-presence"
    image_path = output / "assets" / "frame-000004.png"
    assert image_path.is_file()
    assert np.array_equal(Image.from_file(image_path).data, image.data)
    assert (output / "cases.jsonl").is_file()
    assert (output / "labels.jsonl").is_file()
    assert (output / "audit" / "frame-000004" / "ground_truth.json").is_file()

    suite = load_suite(output)
    evaluation = suite[0].evaluate(cast("EvalRig", _Rig()))

    assert evaluation.case_id == "frame-000004-chair-presence"
    assert json.loads(evaluation.outputs) == {"answer": "yes"}
    assert evaluation.score == 1.0

    invalid = suite[0].evaluate(cast("EvalRig", _Rig(answer="maybe")))
    assert invalid.score == 0.0


def test_generate_multiple_images_aggregates_frame_artifacts(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    request = GenerationRequest(
        dataset="recording.db",
        start=1,
        stop=5,
        stride=2,
        output=output,
    )
    frames = (
        (1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        (3, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=3.0)),
    )

    result = generate_frames_dataset(
        request,
        frames,
        cast("QuestionAuthor", _Author()),
        cast("VlModel", _Detector(present=True)),
        model_names={"author": "gpt-4o-mini", "detector": "vikhyatk/moondream2"},
    )

    assert [case.id for case in result.cases] == [
        "frame-000001-chair-presence",
        "frame-000003-chair-presence",
    ]
    assert (output / "assets" / "frame-000001.png").is_file()
    assert (output / "assets" / "frame-000003.png").is_file()
    assert (output / "audit" / "frame-000001" / "frame.json").is_file()
    assert (output / "audit" / "frame-000003" / "frame.json").is_file()
    run = json.loads((output / "audit" / "run.json").read_text())
    assert run["frame_count"] == 2
    assert run["question_count"] == 2
    assert run["generation"]["start"] == 1
    assert run["generation"]["stop"] == 5
    assert run["generation"]["stride"] == 2
    assert run["models"] == {
        "author": "gpt-4o-mini",
        "detector": "vikhyatk/moondream2",
    }


def test_empty_frame_does_not_count_as_rejected_question(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    request = GenerationRequest(dataset="recording.db", start=1, stop=3, output=output)
    frames = (
        (1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        (2, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=2.0)),
    )

    generate_frames_dataset(
        request,
        frames,
        cast("QuestionAuthor", _EmptyThenAuthor()),
        cast("VlModel", _Detector(present=True)),
    )

    run = json.loads((output / "audit" / "run.json").read_text())
    frame = json.loads((output / "audit" / "frame-000001" / "frame.json").read_text())
    assert run["rejected_question_count"] == 0
    assert frame["rejected_question_count"] == 0


def test_generation_keeps_answered_questions_and_audits_rejections(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)
    detector = cast(
        "VlModel",
        _BoxesDetector(
            (
                (0.0, 0.0, 20.0, 20.0),
                (70.0, 0.0, 90.0, 20.0),
            )
        ),
    )

    result = generate_frames_dataset(
        request,
        ((4, image),),
        cast("QuestionAuthor", _MixedAuthor()),
        detector,
    )

    assert [case.id for case in result.cases] == ["frame-000004-chair-presence"]
    ground_truth = json.loads((output / "audit" / "frame-000004" / "ground_truth.json").read_text())
    assert [row["status"] for row in ground_truth] == ["rejected", "answered"]
    assert "exactly one" in ground_truth[0]["reason"]


def test_generation_propagates_detector_failures(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    with pytest.raises(ValueError, match="detector broke"):
        generate_frames_dataset(
            request,
            ((4, image),),
            cast("QuestionAuthor", _Author()),
            cast("VlModel", _BrokenDetector()),
        )

    assert not output.exists()


def test_later_frame_failure_does_not_publish_partial_dataset(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    request = GenerationRequest(dataset="recording.db", start=1, stop=3, output=output)
    frames = (
        (1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        (2, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=2.0)),
    )

    with pytest.raises(ValueError, match="detector broke"):
        generate_frames_dataset(
            request,
            frames,
            cast("QuestionAuthor", _Author()),
            cast("VlModel", _FailsSecondDetector()),
        )

    assert not output.exists()


def test_generation_deduplicates_proposals_and_suffixes_id_collisions(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        ((4, image),),
        cast("QuestionAuthor", _CollidingAuthor()),
        cast("VlModel", _Detector(present=True)),
    )

    assert [case.id for case in result.cases] == [
        "frame-000004-chair-presence",
        "frame-000004-chair-presence-2",
    ]
