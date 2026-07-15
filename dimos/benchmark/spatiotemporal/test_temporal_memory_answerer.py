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

"""Tests for the public-only TemporalMemory candidate adapter."""

from pathlib import Path

import pytest

from dimos.benchmark.spatiotemporal.models import Question, QuestionKind, SpatialPredicate
from dimos.benchmark.spatiotemporal.temporal_memory_answerer import TemporalMemoryAnswerer
from dimos.benchmark.spatiotemporal.utilities import SCHEMA_VERSION, stable_id


def test_temporal_memory_answerer_enforces_public_candidate_lifecycle(tmp_path: Path) -> None:
    calls: list[tuple[str, object]] = []

    def ingest(video_path: Path) -> int:
        calls.append(("ingest", video_path))
        return 12

    def query(question_text: str) -> str:
        calls.append(("query", question_text))
        return "yes"

    def cleanup() -> None:
        calls.append(("cleanup", None))

    answerer = TemporalMemoryAnswerer(ingest=ingest, query=query, cleanup=cleanup)
    question = Question(
        question_id=stable_id(
            "question",
            {
                "object_ids": ("obj_red", "obj_blue"),
                "predicate": SpatialPredicate.LEFT_OF.value,
                "question_kind": QuestionKind.SPATIAL.value,
                "reference_ids": (),
                "schema_version": SCHEMA_VERSION,
            },
        ),
        episode_id="episode_1",
        text="Is the red object left of the blue object?",
        question_kind=QuestionKind.SPATIAL,
        predicate=SpatialPredicate.LEFT_OF,
        object_ids=("obj_red", "obj_blue"),
    )
    video_path = tmp_path / "public-video.mp4"

    with pytest.raises(RuntimeError, match="ingest"):
        answerer.answer(question)

    readiness = answerer.ingest_video(video_path)
    assert readiness.ready is True
    assert readiness.ingested_frame_count == 12
    assert answerer.answer(question) == "yes"

    answerer.close()
    answerer.close()

    assert calls == [
        ("ingest", video_path),
        ("query", question.text),
        ("cleanup", None),
    ]
    with pytest.raises(RuntimeError, match="closed"):
        answerer.ingest_video(video_path)
