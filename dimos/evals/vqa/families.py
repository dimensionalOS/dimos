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
from typing import TYPE_CHECKING, Annotated, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, JsonValue, StringConstraints, model_validator

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs.Image import Image

NonEmptyString = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]


class QuestionProposal(BaseModel):
    """A model-authored request constrained to a deterministic family."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    family: Literal["presence", "horizontal_direction", "object_count"]
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
        unique_choices = {choice.casefold() for choice in self.choices}
        if len(unique_choices) != len(self.choices):
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
AVAILABLE_FAMILIES = (PRESENCE_FAMILY, HORIZONTAL_DIRECTION_FAMILY, OBJECT_COUNT_FAMILY)


class InsufficientEvidenceError(ValueError):
    """A family cannot derive an answer from the available primitive evidence."""


def answer_question(proposal: QuestionProposal, image: Image, detector: VlModel) -> FamilyAnswer:
    """Dispatch one constrained proposal to its deterministic family."""
    if proposal.family == "presence":
        return _answer_presence(proposal, image, detector)
    if proposal.family == "horizontal_direction":
        return _answer_horizontal_direction(proposal, image, detector)
    if proposal.family == "object_count":
        return _answer_object_count(proposal, image, detector)
    raise ValueError(f"unsupported VQA family: {proposal.family}")


def _answer_presence(proposal: QuestionProposal, image: Image, detector: VlModel) -> FamilyAnswer:
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
    proposal: QuestionProposal, image: Image, detector: VlModel
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
    proposal: QuestionProposal, image: Image, detector: VlModel
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
