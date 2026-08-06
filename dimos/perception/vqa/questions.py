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

"""Deterministic closed-answer questions from grounded objects."""

from __future__ import annotations

from dimos.perception.vqa.models import GroundedObject, VqaExample


def generate_questions(
    frame_id: str, objects: list[GroundedObject], queries: list[str], *, distance_m: float = 3.0
) -> list[VqaExample]:
    """Generate presence, range, and direction questions for one frame."""
    if distance_m <= 0:
        raise ValueError("distance_m must be positive")

    examples: list[VqaExample] = []
    for query in queries:
        matches = sorted(
            (item for item in objects if item.label == query), key=lambda item: item.range_m
        )
        nearest = matches[0] if matches else None
        examples.append(
            VqaExample(
                id=f"{frame_id}-{query}-presence",
                question=f"Is there a {query} in the image? Answer yes or no.",
                expected_answer="yes" if nearest is not None else "no",
                answer_type="boolean",
                object_ids=(nearest.id,) if nearest is not None else (),
            )
        )
        if nearest is None:
            continue
        examples.extend(
            [
                VqaExample(
                    id=f"{frame_id}-{query}-direction",
                    question=f"Where is the nearest {query}: left, center, or right?",
                    expected_answer=nearest.horizontal_direction,
                    answer_type="choice",
                    object_ids=(nearest.id,),
                ),
                VqaExample(
                    id=f"{frame_id}-{query}-range",
                    question=f"Is the nearest {query} within {distance_m:g} meters? Answer yes or no.",
                    expected_answer="yes" if nearest.range_m <= distance_m else "no",
                    answer_type="boolean",
                    object_ids=(nearest.id,),
                ),
            ]
        )
    return examples
