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
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

NonEmptyString = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]


class QuestionProposal(BaseModel):
    """A model-authored request constrained to a deterministic family."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    family: Literal["presence"]
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


PRESENCE_FAMILY = FamilySpec(name="presence", required_fields=("object_name",))
AVAILABLE_FAMILIES = (PRESENCE_FAMILY,)


class ObjectDetector(Protocol):
    """Object evidence required by the presence family."""

    def detect(self, image: Image, object_name: str) -> ImageDetections2D: ...


def answer_question(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    """Dispatch one constrained proposal to its deterministic family."""
    if proposal.family == "presence":
        return _answer_presence(proposal, image, detector)
    raise ValueError(f"unsupported VQA family: {proposal.family}")


def _answer_presence(
    proposal: QuestionProposal, image: Image, detector: ObjectDetector
) -> FamilyAnswer:
    detections = detector.detect(image, proposal.object_name)
    if not detections.detections:
        raise ValueError(f"object detector did not confirm visible {proposal.object_name!r}")
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
