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

"""Image-only visual-question-answering evaluation."""

from __future__ import annotations

import re

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.vqa.models import VisualQuestionAnswerer, VqaEvaluation, VqaExample


def evaluate_examples(
    image: Image, examples: list[VqaExample], answerer: VisualQuestionAnswerer
) -> list[VqaEvaluation]:
    """Ask an agent questions from an image and compare closed answers."""
    evaluations: list[VqaEvaluation] = []
    for example in examples:
        raw_response = answerer.answer(image, example.question)
        normalized = _normalize_response(raw_response, example.expected_answer)
        evaluations.append(
            VqaEvaluation(
                example_id=example.id,
                expected_answer=example.expected_answer,
                raw_response=raw_response,
                normalized_response=normalized,
                passed=normalized == example.expected_answer,
            )
        )
    return evaluations


def _normalize_response(response: str, expected: str) -> str | None:
    tokens = re.findall(r"[a-z]+", response.lower())
    return expected if expected.lower() in tokens else None
