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

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    StringConstraints,
    model_validator,
)

if TYPE_CHECKING:
    from dimos.evals.vqa.preprocessing import CalibratedFrame
    from dimos.evals.vqa.primitives.range import ObjectRangeEstimator
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

NonEmptyString = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]
FamilyName = Literal[
    "presence",
    "horizontal_direction",
    "object_count",
    "object_distance",
    "closest_object",
]


class QuestionProposal(BaseModel):
    """A model-authored request constrained to a deterministic family."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    family: FamilyName
    object_names: tuple[NonEmptyString, ...] = Field(min_length=1)


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

    name: FamilyName
    description: str
    min_objects: int = 1
    max_objects: int = 1
    distinct_objects: bool = False
    requires_pointcloud: bool = False

    def validate(self, proposal: QuestionProposal) -> None:
        """Validate a structurally parsed proposal against this family contract."""
        if proposal.family != self.name:
            raise ValueError(
                f"proposal family {proposal.family!r} does not match specification {self.name!r}"
            )
        count = len(proposal.object_names)
        if not self.min_objects <= count <= self.max_objects:
            if self.min_objects == self.max_objects:
                requirement = f"exactly {self.min_objects} object name"
            else:
                requirement = f"{self.min_objects} to {self.max_objects} object names"
            raise ValueError(f"{self.name} requires {requirement}")
        if (
            self.distinct_objects
            and len({name.casefold() for name in proposal.object_names}) != count
        ):
            raise ValueError(f"{self.name} requires distinct object names")


PRESENCE_FAMILY = FamilySpec(
    name="presence",
    description="Ask whether a clearly visible object category is present.",
)
HORIZONTAL_DIRECTION_FAMILY = FamilySpec(
    name="horizontal_direction",
    description="Use only for one clearly visible object instance.",
)
OBJECT_COUNT_FAMILY = FamilySpec(
    name="object_count",
    description="Count a small number of clearly visible, distinct object instances.",
)
OBJECT_DISTANCE_FAMILY = FamilySpec(
    name="object_distance",
    description="Estimate the range to one clearly visible object instance.",
    requires_pointcloud=True,
)
CLOSEST_OBJECT_FAMILY = FamilySpec(
    name="closest_object",
    description=(
        "Choose the closest of two to five distinct object references with exactly one visible "
        "match each. Spatial descriptions such as 'left person' and 'right person' may distinguish "
        "instances of one category. Do not infer which reference is closest."
    ),
    min_objects=2,
    max_objects=5,
    distinct_objects=True,
    requires_pointcloud=True,
)
AVAILABLE_FAMILIES = (
    PRESENCE_FAMILY,
    HORIZONTAL_DIRECTION_FAMILY,
    OBJECT_COUNT_FAMILY,
    OBJECT_DISTANCE_FAMILY,
    CLOSEST_OBJECT_FAMILY,
)

_FAMILIES_BY_NAME = {family.name: family for family in AVAILABLE_FAMILIES}


def _family_spec(name: FamilyName) -> FamilySpec:
    return _FAMILIES_BY_NAME[name]


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
    _family_spec(proposal.family).validate(proposal)
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
    if proposal.family == "closest_object":
        if calibrated_frame is None or range_estimator is None:
            raise InsufficientEvidenceError(
                "closest object requires calibrated point-cloud evidence"
            )
        return _answer_closest_object(proposal, calibrated_frame, range_estimator)
    raise ValueError(f"unsupported VQA family: {proposal.family}")


def _answer_presence(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    if not detections.detections:
        raise InsufficientEvidenceError(f"object detector did not confirm visible {object_name!r}")
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"Does the image contain any {object_name}?",
        choices=("yes", "no"),
        answer="yes",
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
            "detection_count": len(detections),
            "boxes": cast("JsonValue", boxes),
        },
    )


def _answer_horizontal_direction(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    if len(detections) != 1:
        raise InsufficientEvidenceError(
            f"horizontal direction requires exactly one detected {object_name!r}, "
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
        question=f"Which horizontal region contains the visible {object_name}?",
        choices=("left", "center", "right"),
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
            "detection_count": 1,
            "box": cast("JsonValue", list(map(float, box))),
            "center_x_px": center_x,
            "center_x_fraction": center_fraction,
        },
    )


def _answer_object_count(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    count = len(detections)
    if count == 0:
        raise InsufficientEvidenceError(
            f"object detector did not confirm visible {object_name!r} to count"
        )

    choices = ("one", "two", "three", "four or more")
    answer = choices[min(count, 4) - 1]
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"How many instances of {object_name} are visible in the image?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
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
    object_name = proposal.object_names[0]
    evidence = range_estimator.estimate(calibrated_frame, object_name)
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
        question=f"Approximately how far is the visible {object_name} from the camera?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "edgetam_lidar_range",
            **evidence.model_dump(mode="json"),
        },
    )


def _answer_closest_object(
    proposal: QuestionProposal,
    calibrated_frame: CalibratedFrame,
    range_estimator: ObjectRangeEstimator,
) -> FamilyAnswer:
    """Choose the closest named object when its range interval is unambiguous."""
    object_names = proposal.object_names
    estimate_many = getattr(range_estimator, "estimate_many", None)
    if estimate_many is None:
        ranges = tuple(
            range_estimator.estimate(calibrated_frame, object_name) for object_name in object_names
        )
    else:
        ranges = estimate_many(calibrated_frame, object_names)
    if len(ranges) != len(object_names):
        raise ValueError("range estimator returned the wrong number of object ranges")
    if tuple(evidence.object_name for evidence in ranges) != object_names:
        raise ValueError("range estimator results do not match the requested object order")

    winner_index = min(range(len(ranges)), key=lambda index: ranges[index].camera_range_m)
    winner = ranges[winner_index]
    winner_upper = winner.range_quartiles_m[2]
    if any(
        winner_upper >= evidence.range_quartiles_m[0]
        for index, evidence in enumerate(ranges)
        if index != winner_index
    ):
        raise InsufficientEvidenceError(
            "object range uncertainty does not establish a closest object"
        )

    return FamilyAnswer(
        question="Which object is closest to the camera?",
        choices=object_names,
        answer=object_names[winner_index],
        evidence={
            "primitive": "edgetam_lidar_closest_range",
            "comparison_rule": "closest_non_overlapping_interquartile_range",
            "objects": cast(
                "JsonValue",
                [
                    {"choice": object_name, **evidence.model_dump(mode="json")}
                    for object_name, evidence in zip(object_names, ranges, strict=True)
                ],
            ),
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
