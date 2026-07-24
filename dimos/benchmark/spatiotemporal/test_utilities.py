# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for deterministic serialization helpers."""

import json

from dimos.benchmark.spatiotemporal.models import Question, QuestionKind, SpatialPredicate
from dimos.benchmark.spatiotemporal.utilities import canonical_model_json, stable_question_id


def test_serializes_strict_records_to_identical_canonical_json() -> None:
    question = Question(
        question_id=stable_question_id(
            episode_id="episode_1",
            object_ids=("obj_red", "obj_blue"),
            predicate=SpatialPredicate.LEFT_OF.value,
            question_kind=QuestionKind.SPATIAL.value,
        ),
        episode_id="episode_1",
        text="Is the mug left of the laptop at the end?",
        question_kind=QuestionKind.SPATIAL,
        predicate=SpatialPredicate.LEFT_OF,
        object_ids=("obj_red", "obj_blue"),
    )

    first = canonical_model_json(question)
    second = canonical_model_json(question)

    assert first == second
    assert json.loads(first) == question.model_dump(mode="json")
    assert ": " not in first
    assert ", " not in first
