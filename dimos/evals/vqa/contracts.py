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

"""Shared domain contracts for deterministic VQA generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Annotated, Literal, Protocol

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    StringConstraints,
    model_validator,
)

if TYPE_CHECKING:
    from dimos.evals.vqa.pointcloud_frame import PointCloudFrame
    from dimos.evals.vqa.primitives.edge_tam import ObjectMaskEvidence
    from dimos.evals.vqa.primitives.range import ObjectRangeEvidence
    from dimos.msgs.sensor_msgs.Image import Image

NonEmptyString = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]
FamilyName = Literal[
    "presence",
    "horizontal_direction",
    "object_count",
    "image_coverage",
    "largest_visible_area",
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
        unique_choices = {choice.casefold() for choice in self.choices}
        if len(unique_choices) != len(self.choices):
            raise ValueError("VQA choices must be unique")
        if self.answer not in self.choices:
            raise ValueError("the answer must be one of the choices")
        return self


class InvalidQuestionProposalError(ValueError):
    """A proposal does not satisfy its selected question family contract."""


@dataclass(frozen=True)
class FamilySpec:
    """The proposal shape exposed to the image-only question author."""

    name: FamilyName
    description: str
    min_objects: int = 1
    max_objects: int = 1
    distinct_objects: bool = False
    requires_masks: bool = False
    requires_pointcloud: bool = False
    object_order_matters: bool = True

    def validate(self, proposal: QuestionProposal) -> None:
        """Validate a structurally parsed proposal against this family contract."""
        if proposal.family != self.name:
            raise InvalidQuestionProposalError(
                f"proposal family {proposal.family!r} does not match specification {self.name!r}"
            )
        count = len(proposal.object_names)
        if not self.min_objects <= count <= self.max_objects:
            if self.min_objects == self.max_objects:
                requirement = f"exactly {self.min_objects} object name"
            else:
                requirement = f"{self.min_objects} to {self.max_objects} object names"
            raise InvalidQuestionProposalError(f"{self.name} requires {requirement}")
        if (
            self.distinct_objects
            and len({name.casefold() for name in proposal.object_names}) != count
        ):
            raise InvalidQuestionProposalError(f"{self.name} requires distinct object names")


class InsufficientEvidenceError(ValueError):
    """A family cannot derive an answer from the available primitive evidence."""


class ObjectMaskEstimator(Protocol):
    """Return validated image-sized masks in requested object order."""

    def estimate(self, image: Image, object_name: str) -> ObjectMaskEvidence: ...

    def estimate_many(
        self, image: Image, object_names: tuple[str, ...]
    ) -> tuple[ObjectMaskEvidence, ...]: ...


class ObjectRangeEstimator(Protocol):
    """Return object ranges in requested order from an explicit point-cloud frame."""

    def estimate(self, frame: PointCloudFrame, object_name: str) -> ObjectRangeEvidence: ...

    def estimate_many(
        self, frame: PointCloudFrame, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]: ...
