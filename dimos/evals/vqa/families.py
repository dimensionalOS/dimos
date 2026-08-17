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

"""Contracts for deterministic VQA question families."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Annotated, Literal, Protocol, cast

from pydantic import BaseModel, ConfigDict, Field, JsonValue, StringConstraints, model_validator

if TYPE_CHECKING:
    from dimos.evals.vqa.preprocessing import CalibratedFrame
    from dimos.evals.vqa.primitives.edgetam import ObjectRangeEstimator
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

NonEmptyString = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]


class QuestionProposal(BaseModel):
    """A model-authored request constrained to a deterministic family."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    family: Literal["presence", "horizontal_direction", "object_count", "object_distance"]
    object_name: NonEmptyString


class FamilyAnswer(BaseModel):
    """A public question and its privately derived choice answer."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    question: NonEmptyString
    choices: tuple[NonEmptyString, ...]
    answer: NonEmptyString
    evidence: dict[str, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def answer_is_a_choice(self) -> FamilyAnswer:
        if len(self.choices) < 2:
            raise ValueError("a VQA question requires at least two choices")
        if len(set(self.choices)) != len(self.choices):
            raise ValueError("VQA choices must be unique")
        if self.answer not in self.choices:
            raise ValueError("the answer must be one of the choices")
        return self


@dataclass(frozen=True)
class FamilySpec:
    """The proposal shape exposed to the image-only question author."""

    name: str
    required_fields: tuple[str, ...]
    description: str


PRESENCE_FAMILY = FamilySpec(
    name="presence",
    required_fields=("object_name",),
    description="Ask whether a clearly visible object category is present.",
)
HORIZONTAL_DIRECTION_FAMILY = FamilySpec(
    name="horizontal_direction",
    required_fields=("object_name",),
    description="Use only for one clearly visible object instance.",
)
OBJECT_COUNT_FAMILY = FamilySpec(
    name="object_count",
    required_fields=("object_name",),
    description="Count a small number of clearly visible, distinct object instances.",
)
OBJECT_DISTANCE_FAMILY = FamilySpec(
    name="object_distance",
    required_fields=("object_name",),
    description="Estimate the range to one clearly visible object instance.",
)
AVAILABLE_FAMILIES = (
    PRESENCE_FAMILY,
    HORIZONTAL_DIRECTION_FAMILY,
    OBJECT_COUNT_FAMILY,
    OBJECT_DISTANCE_FAMILY,
)


class InsufficientEvidenceError(ValueError):
    """A family cannot derive an answer from the available primitive evidence."""


class ObjectDetector(Protocol):
    """Object evidence required by visual question families."""

    def query_detections(self, image: Image, query: str) -> ImageDetections2D: ...


def answer_question(
    proposal: QuestionProposal,
    image: Image,
    detector: ObjectDetector,
    calibrated_frame: CalibratedFrame | None = None,
    range_estimator: ObjectRangeEstimator | None = None,
) -> FamilyAnswer:
    """Dispatch one constrained proposal to its deterministic family."""
    if proposal.family == "presence":
        return _answer_presence(proposal, image, detector)
    if proposal.family == "horizontal_direction":
        return _answer_horizontal_direction(proposal, image, detector)
    if proposal.family == "object_count":
        return _answer_object_count(proposal, image, detector)
    if proposal.family == "object_distance":
        if calibrated_frame is None or range_estimator is None:
            raise InsufficientEvidenceError(
                "object distance requires calibrated point-cloud evidence"
            )
        return _answer_object_distance(proposal, calibrated_frame, range_estimator)
    raise ValueError(f"unsupported VQA family: {proposal.family}")


def _answer_presence(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    detections = detector.query_detections(image, proposal.object_name)
    if not detections.detections:
        raise InsufficientEvidenceError(
            f"object detector did not confirm visible {proposal.object_name!r}"
        )
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"Does the image contain any {proposal.object_name}?",
        choices=("yes", "no"),
        answer="yes",
        evidence={
            "primitive": "object_detector",
            "object_name": proposal.object_name,
            "detection_count": len(detections),
            "boxes": cast("JsonValue", boxes),
        },
    )


def _answer_horizontal_direction(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    detections = detector.query_detections(image, proposal.object_name)
    if len(detections) != 1:
        raise InsufficientEvidenceError(
            f"horizontal direction requires exactly one detected {proposal.object_name!r}, "
            f"got {len(detections)}"
        )

    box = detections.detections[0].bbox
    center_x = (box[0] + box[2]) / 2
    center_fraction = center_x / image.width
    if center_fraction < 1 / 3:
        answer = "left"
    elif center_fraction < 2 / 3:
        answer = "center"
    else:
        answer = "right"
    return FamilyAnswer(
        question=f"Which horizontal region contains the visible {proposal.object_name}?",
        choices=("left", "center", "right"),
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": proposal.object_name,
            "detection_count": 1,
            "box": cast("JsonValue", list(map(float, box))),
            "center_x_px": center_x,
            "center_x_fraction": center_fraction,
        },
    )


def _answer_object_count(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    detections = detector.query_detections(image, proposal.object_name)
    count = len(detections)
    if count == 0:
        raise InsufficientEvidenceError(
            f"object detector did not confirm visible {proposal.object_name!r} to count"
        )

    choices = ("one", "two", "three", "four or more")
    answer = choices[min(count, 4) - 1]
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"How many instances of {proposal.object_name} are visible in the image?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": proposal.object_name,
            "detection_count": count,
            "boxes": cast("JsonValue", boxes),
        },
    )


def _answer_object_distance(
    proposal: QuestionProposal,
    calibrated_frame: CalibratedFrame,
    range_estimator: ObjectRangeEstimator,
) -> FamilyAnswer:
    """Answer an object-distance proposal from EdgeTAM and point-cloud evidence."""
    evidence = range_estimator.estimate(calibrated_frame, proposal.object_name)
    choices = (
        "under 1 meter",
        "1 to under 2 meters",
        "2 to under 3 meters",
        "3 meters or more",
    )
    lower_quartile, _, upper_quartile = evidence.range_quartiles_m
    if _distance_bucket(lower_quartile) != _distance_bucket(upper_quartile):
        raise InsufficientEvidenceError(
            "object range uncertainty crosses a distance answer boundary"
        )
    answer = choices[_distance_bucket(evidence.camera_range_m)]
    return FamilyAnswer(
        question=f"Approximately how far is the visible {proposal.object_name} from the camera?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "edgetam_lidar_range",
            **evidence.model_dump(mode="json"),
        },
    )


def _distance_bucket(distance_m: float) -> int:
    if distance_m < 1:
        return 0
    if distance_m < 2:
        return 1
    if distance_m < 3:
        return 2
    return 3
