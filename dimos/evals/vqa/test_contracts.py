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

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.constants import STATE_DIR
from dimos.evals.vqa.author import OpenAIQuestionAuthor
from dimos.evals.vqa.contracts import (
    FamilyAnswer,
    FamilySpec,
    InsufficientEvidenceError,
    QuestionProposal,
)
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    CLOSEST_OBJECT_FAMILY,
    LARGEST_VISIBLE_AREA_FAMILY,
    PRESENCE_FAMILY,
    answer_question,
)
from dimos.evals.vqa.generate import (
    GenerationFrame,
    GenerationRequest,
    PrivateLabel,
    PublicCase,
    VqaGenerationConfig,
    generate_frames_dataset,
)
from dimos.evals.vqa.primitives.edge_tam import ObjectMaskEvidence
from dimos.evals.vqa.primitives.range import ObjectRangeEvidence
from dimos.evals.vqa.suite import load_suite
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class _TestVlModel(VlModel):
    def __init__(self) -> None:
        pass

    def query(self, image: Image, query: str, **kwargs: object) -> str:
        raise NotImplementedError

    def stop(self) -> None:
        pass


class _Author:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (QuestionProposal(family="presence", object_names=("chair",)),)


class _MixedAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(family="horizontal_direction", object_names=("chair",)),
            QuestionProposal(family="presence", object_names=("chair",)),
        )


class _InvalidThenValidAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(family="presence", object_names=("chair", "table")),
            QuestionProposal(family="presence", object_names=("chair",)),
        )


class _DistanceAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (QuestionProposal(family="object_distance", object_names=("chair",)),)


class _ClosestObjectAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(
                family="closest_object",
                object_names=("left person", "right person", "chair"),
            ),
        )


class _CoverageAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (QuestionProposal(family="image_coverage", object_names=("chair",)),)


class _DuplicateLargestVisibleAreaAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(
                family="largest_visible_area",
                object_names=("chair", "table", "box"),
            ),
            QuestionProposal(
                family="largest_visible_area",
                object_names=("box", "chair", "table"),
            ),
        )


class _DuplicateClosestObjectAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(
                family="closest_object",
                object_names=("left person", "right person", "chair"),
            ),
            QuestionProposal(
                family="closest_object",
                object_names=("chair", "right person", "left person"),
            ),
        )


class _RecordingAuthor(_Author):
    def __init__(self) -> None:
        self.family_names: tuple[str, ...] = ()

    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        self.family_names = tuple(family.name for family in families)
        return super().propose(image, families)


class _EmptyThenAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        if image.ts == 1.0:
            return ()
        return (QuestionProposal(family="presence", object_names=("chair",)),)


class _CollidingAuthor:
    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return (
            QuestionProposal(family="presence", object_names=("Chair",)),
            QuestionProposal(family="presence", object_names=("chair",)),
            QuestionProposal(family="presence", object_names=("Chair",)),
        )


class _Detector(_TestVlModel):
    def __init__(self, present: bool) -> None:
        self._present = present

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
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


class _BoxesDetector(_TestVlModel):
    def __init__(self, boxes: tuple[Bbox, ...]) -> None:
        self._boxes = boxes

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
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
    def query_json(self, image: Image, prompt: str) -> list[dict[str, object]]:
        assert "presence" in prompt
        assert "horizontal_direction" in prompt
        assert "object_count" in prompt
        assert "image_coverage" in prompt
        assert "largest_visible_area" in prompt
        assert "object_distance" in prompt
        assert "closest_object" in prompt
        assert "JSON array" in prompt
        assert "'left box' and 'right box'" in prompt
        assert "Do not call an object 'closest'" in prompt
        return [
            {"family": "presence", "object_names": ["chair"]},
            {"family": "horizontal_direction", "object_names": ["robot"]},
            {"family": "object_count", "object_names": ["box"]},
            {"family": "image_coverage", "object_names": ["chair"]},
            {
                "family": "largest_visible_area",
                "object_names": ["chair", "table", "box"],
            },
            {"family": "object_distance", "object_names": ["chair"]},
            {
                "family": "closest_object",
                "object_names": ["left person", "right person", "chair"],
            },
        ]


class _PartlyInvalidQuestionModel:
    def query_json(self, image: Image, prompt: str) -> list[object]:
        return [
            {"family": "presence", "object_names": ["chair"]},
            {"family": "object_count", "object_names": ["table"]},
            {"family": "unsupported", "object_names": ["table"]},
            {"family": "presence", "object_names": ["door"], "answer": "yes"},
        ]


class _UnavailableFamilyQuestionModel:
    def query_json(self, image: Image, prompt: str) -> list[dict[str, object]]:
        assert "For closest_object" not in prompt
        return [{"family": "object_distance", "object_names": ["chair"]}]


class _BrokenDetector(_TestVlModel):
    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        raise ValueError("detector broke")


class _FailsSecondDetector(_Detector):
    def __init__(self) -> None:
        super().__init__(present=True)
        self._calls = 0

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        self._calls += 1
        if self._calls == 2:
            raise ValueError("detector broke")
        return super().query_detections(image, query, **kwargs)


class _RangeEstimator:
    def __init__(self, range_m: float, quartiles: tuple[float, float, float] | None = None) -> None:
        self._range_m = range_m
        self._quartiles = quartiles or (range_m, range_m, range_m)

    def estimate(self, frame: object, object_name: str) -> ObjectRangeEvidence:
        return ObjectRangeEvidence(
            object_name=object_name,
            camera_range_m=self._range_m,
            supporting_point_count=7,
            prompt_bbox_xyxy=(0.0, 0.0, 2.0, 2.0),
            mask_bbox_xyxy=(0.0, 0.0, 2.0, 2.0),
            mask_area_px=4,
            range_quartiles_m=self._quartiles,
            synchronization_delta_s=0.02,
            calibration_source="test",
        )

    def estimate_many(
        self, frame: object, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]:
        return tuple(self.estimate(frame, object_name) for object_name in object_names)


class _MaskEstimator:
    def __init__(self, areas: dict[str, int]) -> None:
        self._areas = areas

    def estimate(self, image: Image, object_name: str) -> ObjectMaskEvidence:
        mask = np.zeros((image.height, image.width), dtype=bool)
        mask.flat[: self._areas[object_name]] = True
        box = (0.0, 0.0, float(image.width), float(image.height))
        detection = Detection2DBBox(
            bbox=box,
            track_id=0,
            class_id=-1,
            confidence=1.0,
            name=object_name,
            ts=image.ts,
            image=image,
        )
        return ObjectMaskEvidence(
            object_name=object_name,
            prompt_bbox_xyxy=box,
            detection=detection,
            mask=mask,
        )

    def estimate_many(
        self, image: Image, object_names: tuple[str, ...]
    ) -> tuple[ObjectMaskEvidence, ...]:
        return tuple(self.estimate(image, object_name) for object_name in object_names)


class _ClosestRangeEstimator:
    def __init__(self, *quartiles: tuple[float, float, float]) -> None:
        self._quartiles = quartiles

    def estimate(self, frame: object, object_name: str) -> ObjectRangeEvidence:
        quartiles = self._quartiles[0]
        box = (0.0, 0.0, 2.0, 2.0)
        return ObjectRangeEvidence(
            object_name=object_name,
            camera_range_m=quartiles[1],
            supporting_point_count=7,
            prompt_bbox_xyxy=box,
            mask_bbox_xyxy=box,
            mask_area_px=4,
            range_quartiles_m=quartiles,
            synchronization_delta_s=0.02,
            calibration_source="test",
        )

    def estimate_many(
        self, frame: object, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]:
        return tuple(
            ObjectRangeEvidence(
                object_name=object_name,
                camera_range_m=quartiles[1],
                supporting_point_count=7,
                prompt_bbox_xyxy=(0.0, 0.0, 2.0, 2.0),
                mask_bbox_xyxy=(0.0, 0.0, 2.0, 2.0),
                mask_area_px=4,
                range_quartiles_m=quartiles,
                synchronization_delta_s=0.02,
                calibration_source="test",
            )
            for object_name, quartiles in zip(
                object_names,
                self._quartiles,
                strict=True,
            )
        )


class _ReorderedClosestRangeEstimator(_ClosestRangeEstimator):
    def estimate_many(
        self, frame: object, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]:
        return tuple(reversed(super().estimate_many(frame, object_names)))


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
    proposal = QuestionProposal(family="presence", object_names=("  chair  ",))

    assert proposal.object_names == ("chair",)
    assert proposal.model_dump(mode="json") == {
        "family": "presence",
        "object_names": ["chair"],
    }
    assert proposal.family in [family.name for family in AVAILABLE_FAMILIES]


def test_closest_object_proposal_requires_two_to_five_distinct_references() -> None:
    proposal = QuestionProposal(
        family="closest_object",
        object_names=("  left person ", "right person", "chair"),
    )

    assert proposal.object_names == ("left person", "right person", "chair")
    assert proposal.model_dump(mode="json") == {
        "family": "closest_object",
        "object_names": ["left person", "right person", "chair"],
    }
    assert CLOSEST_OBJECT_FAMILY in AVAILABLE_FAMILIES
    with pytest.raises(ValueError, match="distinct object names"):
        CLOSEST_OBJECT_FAMILY.validate(
            QuestionProposal(
                family="closest_object",
                object_names=("person", "Person"),
            )
        )
    with pytest.raises(ValueError, match="2 to 5"):
        CLOSEST_OBJECT_FAMILY.validate(
            QuestionProposal(family="closest_object", object_names=("chair",))
        )
    with pytest.raises(ValueError, match="2 to 5"):
        CLOSEST_OBJECT_FAMILY.validate(
            QuestionProposal(
                family="closest_object",
                object_names=("a", "b", "c", "d", "e", "f"),
            )
        )
    with pytest.raises(ValidationError):
        QuestionProposal.model_validate(
            {
                "family": "closest_object",
                "object_name": "chair",
                "object_names": ["chair", "table"],
            }
        )
    with pytest.raises(ValueError, match="exactly 1 object name"):
        PRESENCE_FAMILY.validate(
            QuestionProposal(family="presence", object_names=("chair", "table"))
        )


def test_object_distance_requires_range_evidence() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_distance", object_names=("chair",))

    with pytest.raises(InsufficientEvidenceError, match="image-aligned point-cloud"):
        answer_question(proposal, image, _Detector(present=True))


@pytest.mark.parametrize(
    ("range_m", "expected"),
    (
        (0.99, "under 1 meter"),
        (1.0, "1 to under 2 meters"),
        (2.0, "2 to under 3 meters"),
        (3.0, "3 meters or more"),
    ),
)
def test_object_distance_buckets_range_evidence(range_m: float, expected: str) -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_distance", object_names=("chair",))

    answer = answer_question(
        proposal,
        image,
        _Detector(present=True),
        object(),
        _RangeEstimator(range_m),
    )

    assert answer.answer == expected
    assert answer.evidence["camera_range_m"] == range_m
    assert answer.evidence["supporting_point_count"] == 7


def test_object_distance_rejects_evidence_crossing_bucket_boundary() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_distance", object_names=("chair",))

    with pytest.raises(InsufficientEvidenceError, match="uncertainty crosses"):
        answer_question(
            proposal,
            image,
            _Detector(present=True),
            object(),
            _RangeEstimator(2.01, (1.8, 2.01, 2.2)),
        )


@pytest.mark.parametrize(
    ("quartiles", "expected"),
    (
        (
            ((1.0, 1.1, 1.2), (2.0, 2.1, 2.2), (3.0, 3.1, 3.2)),
            "left person",
        ),
        (
            ((3.0, 3.1, 3.2), (2.0, 2.1, 2.2), (1.0, 1.1, 1.2)),
            "chair",
        ),
    ),
)
def test_closest_object_compares_multiple_named_references(
    quartiles: tuple[tuple[float, float, float], ...],
    expected: str,
) -> None:
    image = Image.from_numpy(np.zeros((4, 10, 3), dtype=np.uint8))
    proposal = QuestionProposal(
        family="closest_object",
        object_names=("left person", "right person", "chair"),
    )

    answer = answer_question(
        proposal,
        image,
        _Detector(present=True),
        object(),
        _ClosestRangeEstimator(*quartiles),
    )

    assert answer.answer == expected
    assert answer.choices == ("left person", "right person", "chair")
    assert answer.question == "Which object is closest to the camera?"
    assert answer.evidence["comparison_rule"] == "closest_non_overlapping_interquartile_range"


def test_closest_object_rejects_overlapping_range_intervals() -> None:
    image = Image.from_numpy(np.zeros((4, 10, 3), dtype=np.uint8))
    proposal = QuestionProposal(
        family="closest_object",
        object_names=("left person", "right person", "chair"),
    )

    with pytest.raises(InsufficientEvidenceError, match="does not establish"):
        answer_question(
            proposal,
            image,
            _Detector(present=True),
            object(),
            _ClosestRangeEstimator(
                (1.0, 1.1, 1.2),
                (1.1, 1.3, 1.5),
                (3.0, 3.1, 3.2),
            ),
        )


def test_closest_object_rejects_reordered_range_results() -> None:
    proposal = QuestionProposal(
        family="closest_object",
        object_names=("left person", "right person"),
    )

    with pytest.raises(ValueError, match="requested object order"):
        answer_question(
            proposal,
            Image.from_numpy(np.zeros((4, 10, 3), dtype=np.uint8)),
            _Detector(present=True),
            object(),
            _ReorderedClosestRangeEstimator((1.0, 1.1, 1.2), (2.0, 2.1, 2.2)),
        )


def test_question_proposal_rejects_unknown_fields() -> None:
    with pytest.raises(ValidationError):
        QuestionProposal.model_validate(
            {"family": "presence", "object_names": ["chair"], "answer": "yes"}
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
    proposal = QuestionProposal(family="presence", object_names=("chair",))

    present = answer_question(proposal, image, _Detector(present=True))

    assert present.answer == "yes"
    assert present.question == "Does the image contain any chair?"
    assert present.choices == ("yes", "no")
    assert present.evidence["detection_count"] == 1
    with pytest.raises(ValueError, match="did not confirm"):
        answer_question(proposal, image, _Detector(present=False))


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
def test_horizontal_direction_uses_detection_center(box: Bbox, expected: str) -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="horizontal_direction", object_names=("robot",))

    answer = answer_question(proposal, image, _BoxesDetector((box,)))

    assert answer.answer == expected
    assert answer.choices == ("left", "center", "right")
    assert answer.evidence["detection_count"] == 1


def test_horizontal_direction_rejects_ambiguous_instances() -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="horizontal_direction", object_names=("chair",))
    detector = _BoxesDetector(
        (
            (0.0, 0.0, 20.0, 20.0),
            (70.0, 0.0, 90.0, 20.0),
        )
    )

    with pytest.raises(ValueError, match="exactly one"):
        answer_question(proposal, image, detector)


@pytest.mark.parametrize(
    ("count", "expected"),
    ((1, "one"), (2, "two"), (3, "three"), (4, "four or more"), (5, "four or more")),
)
def test_object_count_buckets_detected_instances(count: int, expected: str) -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_count", object_names=("box",))
    boxes = tuple((float(index), 0.0, float(index + 1), 1.0) for index in range(count))

    answer = answer_question(proposal, image, _BoxesDetector(boxes))

    assert answer.answer == expected
    assert answer.question == "How many instances of box are visible in the image?"
    assert answer.choices == ("one", "two", "three", "four or more")
    assert answer.evidence["detection_count"] == count


def test_object_count_requires_at_least_one_detection() -> None:
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="object_count", object_names=("box",))

    with pytest.raises(InsufficientEvidenceError, match="to count"):
        answer_question(proposal, image, _BoxesDetector(()))


@pytest.mark.parametrize(
    ("area", "expected", "boundaries", "margin"),
    (
        (24, "15% to under 35%", [15.0, 35.0, 65.0, 85.0], 9.0),
        (40, "25% to under 50%", [25.0, 50.0, 75.0], 10.0),
    ),
)
def test_image_coverage_chooses_scheme_with_largest_boundary_margin(
    area: int,
    expected: str,
    boundaries: list[float],
    margin: float,
) -> None:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))
    proposal = QuestionProposal(family="image_coverage", object_names=("chair",))

    answer = answer_question(
        proposal,
        image,
        _Detector(present=True),
        mask_estimator=_MaskEstimator({"chair": area}),
    )

    assert answer.answer == expected
    assert answer.evidence["coverage_percent"] == float(area)
    assert answer.evidence["bucket_boundaries_percent"] == boundaries
    assert answer.evidence["nearest_boundary_margin_percent"] == margin


def test_image_coverage_requires_mask_evidence() -> None:
    proposal = QuestionProposal(family="image_coverage", object_names=("chair",))

    with pytest.raises(InsufficientEvidenceError, match="segmentation-mask"):
        answer_question(
            proposal,
            Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8)),
            _Detector(present=True),
        )


def test_largest_visible_area_requires_distinct_references() -> None:
    with pytest.raises(ValueError, match="distinct object names"):
        LARGEST_VISIBLE_AREA_FAMILY.validate(
            QuestionProposal(
                family="largest_visible_area",
                object_names=("chair", "Chair"),
            )
        )


def test_largest_visible_area_accepts_winner_at_twenty_percent_margin() -> None:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))
    proposal = QuestionProposal(
        family="largest_visible_area",
        object_names=("chair", "table", "box"),
    )

    answer = answer_question(
        proposal,
        image,
        _Detector(present=True),
        mask_estimator=_MaskEstimator({"chair": 50, "table": 60, "box": 10}),
    )

    assert answer.answer == "table"
    assert answer.choices == ("chair", "table", "box")
    assert answer.evidence["comparison_rule"] == "winner_at_least_20_percent_larger"


def test_largest_visible_area_rejects_close_mask_areas() -> None:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))
    proposal = QuestionProposal(
        family="largest_visible_area",
        object_names=("chair", "table"),
    )

    with pytest.raises(InsufficientEvidenceError, match="at least 20% larger"):
        answer_question(
            proposal,
            image,
            _Detector(present=True),
            mask_estimator=_MaskEstimator({"chair": 50, "table": 59}),
        )


def test_openai_author_parses_constrained_proposals() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(_QuestionModel())

    proposals = author.propose(image, AVAILABLE_FAMILIES)

    assert proposals == (
        QuestionProposal(family="presence", object_names=("chair",)),
        QuestionProposal(family="horizontal_direction", object_names=("robot",)),
        QuestionProposal(family="object_count", object_names=("box",)),
        QuestionProposal(family="image_coverage", object_names=("chair",)),
        QuestionProposal(
            family="largest_visible_area",
            object_names=("chair", "table", "box"),
        ),
        QuestionProposal(family="object_distance", object_names=("chair",)),
        QuestionProposal(
            family="closest_object",
            object_names=("left person", "right person", "chair"),
        ),
    )


def test_openai_author_keeps_valid_items_from_partly_invalid_response() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(_PartlyInvalidQuestionModel())

    proposals = author.propose(image, AVAILABLE_FAMILIES)

    assert proposals == (
        QuestionProposal(family="presence", object_names=("chair",)),
        QuestionProposal(family="object_count", object_names=("table",)),
        QuestionProposal(family="object_distance", object_names=("chair",)),
        QuestionProposal(family="closest_object", object_names=("chair", "table")),
    )


def test_openai_author_skips_unavailable_family() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    author = OpenAIQuestionAuthor(_UnavailableFamilyQuestionModel())

    proposals = author.propose(image, (AVAILABLE_FAMILIES[0],))

    assert proposals == ()


def test_generation_request_selects_one_memory_image() -> None:
    request = GenerationRequest(dataset="go2_short", image_index=4, output=Path("dataset"))

    assert request.image_index == 4
    assert request.output == Path("dataset")
    assert request.frame_indices() == range(4, 5)


def test_generation_request_defaults_output_under_state_directory() -> None:
    request = GenerationRequest(dataset="recordings/go2_short.db", image_index=4)

    assert request.output_directory() == STATE_DIR / "datasets" / "vqa" / "go2_short-frames"


def test_generation_config_validates_synchronization_tolerance() -> None:
    assert VqaGenerationConfig().synchronization_tolerance_s == 0.1
    with pytest.raises(ValidationError):
        VqaGenerationConfig(synchronization_tolerance_s=0.0)


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
    output = tmp_path / "nested" / "vqa"
    image = Image.from_numpy(np.arange(48, dtype=np.uint8).reshape(4, 4, 3), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image),),
        _Author(),
        _Detector(present=True),
    )

    assert result.cases[0].id == "frame-000004-chair-presence"
    image_path = output / "assets" / "frame-000004.png"
    assert image_path.is_file()
    assert np.array_equal(Image.from_file(image_path).data, image.data)
    assert (output / "cases.jsonl").is_file()
    assert (output / "labels.jsonl").is_file()
    assert (output / "audit" / "frame-000004" / "ground_truth.json").is_file()

    suite = load_suite(output)
    evaluation = suite[0].evaluate(_Rig())

    assert evaluation.case_id == "frame-000004-chair-presence"
    assert json.loads(evaluation.outputs) == {"answer": "yes"}
    assert evaluation.score == 1.0

    invalid = suite[0].evaluate(_Rig(answer="maybe"))
    assert invalid.score == 0.0


def test_generation_rejects_nonempty_output_without_modifying_it(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    output.mkdir()
    existing = output / "existing.txt"
    existing.write_text("keep me")
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))

    with pytest.raises(FileExistsError, match="output directory is not empty"):
        generate_frames_dataset(
            request,
            (GenerationFrame(4, image),),
            _Author(),
            _Detector(present=True),
        )

    assert existing.read_text() == "keep me"
    assert list(output.iterdir()) == [existing]


def test_generation_audits_invalid_family_proposals_and_continues(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image),),
        _InvalidThenValidAuthor(),
        _Detector(present=True),
    )

    assert len(result.cases) == 1
    ground_truth = json.loads((output / "audit" / "frame-000004" / "ground_truth.json").read_text())
    assert [row["status"] for row in ground_truth] == ["rejected", "answered"]
    assert ground_truth[0]["reason"] == "presence requires exactly 1 object name"


def test_generate_distance_case_from_pointcloud_frame(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image, object()),),
        _DistanceAuthor(),
        _Detector(present=True),
        _RangeEstimator(1.5),
    )

    assert result.cases[0].id == "frame-000004-chair-object_distance"
    assert result.cases[0].choices == (
        "under 1 meter",
        "1 to under 2 meters",
        "2 to under 3 meters",
        "3 meters or more",
    )
    labels = [json.loads(line) for line in (output / "labels.jsonl").read_text().splitlines()]
    assert labels == [{"id": "frame-000004-chair-object_distance", "answer": "1 to under 2 meters"}]


def test_generate_closest_object_case_from_pointcloud_frame(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 10, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image, object()),),
        _ClosestObjectAuthor(),
        _Detector(present=True),
        _ClosestRangeEstimator(
            (1.0, 1.1, 1.2),
            (2.0, 2.1, 2.2),
            (3.0, 3.1, 3.2),
        ),
    )

    case_id = "frame-000004-chair-vs-left-person-vs-right-person-closest_object"
    assert result.cases[0].id == case_id
    assert result.cases[0].choices == ("left person", "right person", "chair")
    labels = [json.loads(line) for line in (output / "labels.jsonl").read_text().splitlines()]
    assert labels == [
        {
            "id": case_id,
            "answer": "left person",
        }
    ]


def test_generate_image_coverage_case_without_pointcloud(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8), ts=12.5)

    result = generate_frames_dataset(
        GenerationRequest(dataset="recording.db", image_index=4, output=output),
        (GenerationFrame(4, image),),
        _CoverageAuthor(),
        _Detector(present=True),
        mask_estimator=_MaskEstimator({"chair": 24}),
    )

    assert result.cases[0].id == "frame-000004-chair-image_coverage"
    labels = [json.loads(line) for line in (output / "labels.jsonl").read_text().splitlines()]
    assert labels == [{"id": "frame-000004-chair-image_coverage", "answer": "15% to under 35%"}]


def test_generation_deduplicates_reordered_closest_object_references(tmp_path: Path) -> None:
    result = generate_frames_dataset(
        GenerationRequest(dataset="recording.db", image_index=4, output=tmp_path / "vqa"),
        (
            GenerationFrame(
                4,
                Image.from_numpy(np.zeros((4, 10, 3), dtype=np.uint8)),
                object(),
            ),
        ),
        _DuplicateClosestObjectAuthor(),
        _Detector(present=True),
        _ClosestRangeEstimator(
            (1.0, 1.1, 1.2),
            (2.0, 2.1, 2.2),
            (3.0, 3.1, 3.2),
        ),
    )

    assert len(result.cases) == 1
    assert result.cases[0].id == (
        "frame-000004-chair-vs-left-person-vs-right-person-closest_object"
    )


def test_generation_deduplicates_reordered_largest_area_references(tmp_path: Path) -> None:
    result = generate_frames_dataset(
        GenerationRequest(dataset="recording.db", image_index=4, output=tmp_path / "vqa"),
        (GenerationFrame(4, Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))),),
        _DuplicateLargestVisibleAreaAuthor(),
        _Detector(present=True),
        mask_estimator=_MaskEstimator({"chair": 60, "table": 40, "box": 10}),
    )

    assert len(result.cases) == 1
    assert result.cases[0].id == "frame-000004-box-vs-chair-vs-table-largest_visible_area"


def test_frame_without_pointcloud_does_not_expose_distance_family(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)
    author = _RecordingAuthor()

    generate_frames_dataset(
        request,
        (GenerationFrame(4, image),),
        author,
        _Detector(present=True),
        _RangeEstimator(1.5),
    )

    assert author.family_names == ("presence", "horizontal_direction", "object_count")
    run = json.loads((output / "audit" / "run.json").read_text())
    assert run["families"] == ["presence", "horizontal_direction", "object_count"]


def test_frame_without_pointcloud_exposes_mask_families_when_masks_are_available(
    tmp_path: Path,
) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8), ts=12.5)
    author = _RecordingAuthor()

    generate_frames_dataset(
        GenerationRequest(dataset="recording.db", image_index=4, output=output),
        (GenerationFrame(4, image),),
        author,
        _Detector(present=True),
        mask_estimator=_MaskEstimator({"chair": 24}),
    )

    assert author.family_names == (
        "presence",
        "horizontal_direction",
        "object_count",
        "image_coverage",
        "largest_visible_area",
    )


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
        GenerationFrame(1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        GenerationFrame(3, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=3.0)),
    )

    result = generate_frames_dataset(
        request,
        frames,
        _Author(),
        _Detector(present=True),
        config=VqaGenerationConfig(synchronization_tolerance_s=0.025),
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
    assert run["configuration"] == {"synchronization_tolerance_s": 0.025}
    assert run["models"] == {
        "author": "gpt-4o-mini",
        "detector": "vikhyatk/moondream2",
    }


def test_empty_frame_does_not_count_as_rejected_question(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    request = GenerationRequest(dataset="recording.db", start=1, stop=3, output=output)
    frames = (
        GenerationFrame(1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        GenerationFrame(2, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=2.0)),
    )

    generate_frames_dataset(
        request,
        frames,
        _EmptyThenAuthor(),
        _Detector(present=True),
    )

    run = json.loads((output / "audit" / "run.json").read_text())
    frame = json.loads((output / "audit" / "frame-000001" / "frame.json").read_text())
    assert run["rejected_question_count"] == 0
    assert frame["rejected_question_count"] == 0


def test_generation_keeps_answered_questions_and_audits_rejections(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((60, 90, 3), dtype=np.uint8), ts=12.5)
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)
    detector = _BoxesDetector(
        (
            (0.0, 0.0, 20.0, 20.0),
            (70.0, 0.0, 90.0, 20.0),
        )
    )

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image),),
        _MixedAuthor(),
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
            (GenerationFrame(4, image),),
            _Author(),
            _BrokenDetector(),
        )

    assert not output.exists()


def test_later_frame_failure_does_not_publish_partial_dataset(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    request = GenerationRequest(dataset="recording.db", start=1, stop=3, output=output)
    frames = (
        GenerationFrame(1, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)),
        GenerationFrame(2, Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), ts=2.0)),
    )

    with pytest.raises(ValueError, match="detector broke"):
        generate_frames_dataset(
            request,
            frames,
            _Author(),
            _FailsSecondDetector(),
        )

    assert not output.exists()


def test_generation_deduplicates_proposals_and_suffixes_id_collisions(tmp_path: Path) -> None:
    output = tmp_path / "vqa"
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    request = GenerationRequest(dataset="recording.db", image_index=4, output=output)

    result = generate_frames_dataset(
        request,
        (GenerationFrame(4, image),),
        _CollidingAuthor(),
        _Detector(present=True),
    )

    assert [case.id for case in result.cases] == [
        "frame-000004-chair-presence",
        "frame-000004-chair-presence-2",
    ]
