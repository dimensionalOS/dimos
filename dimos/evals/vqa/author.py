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

"""Image-only author for constrained VQA family proposals."""

from __future__ import annotations

from collections.abc import Sequence
import json
from typing import TYPE_CHECKING, Protocol

from pydantic import ValidationError

from dimos.evals.vqa.contracts import FamilySpec, QuestionProposal

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs.Image import Image


class QuestionAuthor(Protocol):
    """Proposes family inputs from an image without private evidence access."""

    def propose(
        self, image: Image, families: Sequence[FamilySpec]
    ) -> Sequence[QuestionProposal]: ...


class OpenAIQuestionAuthor:
    """Constrain an image VLM to the available deterministic family inputs."""

    def __init__(self, model: VlModel) -> None:
        self._model = model

    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        available = {family.name: family for family in families}
        family_shapes = [
            {
                "family": family.name,
                "required_fields": ("object_names",),
                "min_objects": family.min_objects,
                "max_objects": family.max_objects,
                "distinct_objects": family.distinct_objects,
                "description": family.description,
            }
            for family in families
        ]
        prompt = (
            "Generate useful questions about objects clearly visible in this image. "
            "Use any applicable families and object names, preferring specific families when their "
            "requirements are clearly satisfied. "
            "Return only a JSON array of objects matching the available deterministic families. "
            "Populate exactly the required fields. Do not duplicate proposals, answer questions, "
            "or add fields. "
            f"Available families: {json.dumps(family_shapes)}"
        )
        payload = self._model.query_json(image, prompt)
        if not isinstance(payload, list):
            raise ValueError("question author response must be a JSON array")
        proposals: list[QuestionProposal] = []
        for item in payload:
            try:
                proposal = QuestionProposal.model_validate(item)
            except ValidationError:
                continue
            family = available.get(proposal.family)
            if family is None:
                continue
            try:
                family.validate(proposal)
            except ValueError:
                continue
            proposals.append(proposal)
        return tuple(proposals)
