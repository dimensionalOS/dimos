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

"""Behavioral tests for dataset question generation."""

from collections import Counter

import pytest

from dimos.benchmark.spatiotemporal.generation import (
    generate_spatial_questions,
    generate_temporal_question_cases,
)
from dimos.benchmark.spatiotemporal.models import (
    RelationFact,
    RelationInterval,
    SpatialPredicate,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import (
    SCHEMA_VERSION,
    JsonValue,
    stable_id,
)


def _interval(
    subject_id: str,
    predicate: SpatialPredicate,
    object_id: str,
    start_frame_id: int,
    end_frame_id: int,
    start_timestamp_s: float | None = None,
    end_timestamp_s: float | None = None,
    episode_id: str = "episode_1",
) -> RelationInterval:
    if start_timestamp_s is None:
        start_timestamp_s = start_frame_id / 10
    if end_timestamp_s is None:
        end_timestamp_s = end_frame_id / 10
    relation_id = stable_id(
        "relation",
        {
            "object_ids": (subject_id, object_id),
            "predicate": predicate.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    contract: dict[str, JsonValue] = {
        "end_frame_id": end_frame_id,
        "end_timestamp_s": end_timestamp_s,
        "episode_id": episode_id,
        "object_id": object_id,
        "predicate": predicate.value,
        "relation_id": relation_id,
        "schema_version": SCHEMA_VERSION,
        "start_frame_id": start_frame_id,
        "start_timestamp_s": start_timestamp_s,
        "subject_id": subject_id,
    }
    return RelationInterval(
        interval_id=stable_id("interval", contract),
        relation_id=relation_id,
        episode_id=episode_id,
        subject_id=subject_id,
        predicate=predicate,
        object_id=object_id,
        start_frame_id=start_frame_id,
        end_frame_id=end_frame_id,
        start_timestamp_s=start_timestamp_s,
        end_timestamp_s=end_timestamp_s,
        evidence_frame_ids=tuple(range(start_frame_id, end_frame_id + 1)),
    )


def test_generates_balanced_spatial_question_cases() -> None:
    from dimos.benchmark.spatiotemporal.generation import generate_spatial_question_cases

    relation_id = stable_id(
        "relation",
        {
            "object_ids": ("obj_red", "obj_blue"),
            "predicate": SpatialPredicate.LEFT_OF.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    fact = RelationFact(
        relation_id=relation_id,
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        subject_id="obj_red",
        predicate=SpatialPredicate.LEFT_OF,
        object_id="obj_blue",
        evidence_frame_ids=(12,),
    )

    cases = generate_spatial_question_cases((fact,))

    assert len(cases) == 2
    assert Counter(answer.expected for _, answer in cases) == {True: 1, False: 1}
    assert {(question.predicate, answer.expected) for question, answer in cases} == {
        (SpatialPredicate.LEFT_OF, True),
        (SpatialPredicate.RIGHT_OF, False),
    }
    assert all(answer.evidence_frame_ids == (12,) for _, answer in cases)


def test_spatial_cases_are_order_independent_when_relation_flips() -> None:
    def fact(predicate: SpatialPredicate, frame_id: int) -> RelationFact:
        return RelationFact(
            relation_id=stable_id(
                "relation",
                {
                    "object_ids": ("obj_a", "obj_b"),
                    "predicate": predicate.value,
                    "schema_version": SCHEMA_VERSION,
                },
            ),
            episode_id="episode_1",
            frame_id=frame_id,
            timestamp_s=frame_id / 10,
            subject_id="obj_a",
            predicate=predicate,
            object_id="obj_b",
            evidence_frame_ids=(frame_id,),
        )

    from dimos.benchmark.spatiotemporal.generation import generate_spatial_question_cases

    facts = (fact(SpatialPredicate.LEFT_OF, 1), fact(SpatialPredicate.RIGHT_OF, 2))
    forward = generate_spatial_question_cases(facts)
    backward = generate_spatial_question_cases(tuple(reversed(facts)))

    assert tuple(
        (question.model_dump_json(), answer.model_dump_json()) for question, answer in forward
    ) == tuple(
        (question.model_dump_json(), answer.model_dump_json()) for question, answer in backward
    )
    assert Counter(answer.expected for _, answer in forward) == {True: 2}


def test_generates_one_public_spatial_question_per_accepted_relation() -> None:
    relation_id = stable_id(
        "relation",
        {
            "object_ids": ("obj_red", "obj_blue"),
            "predicate": SpatialPredicate.LEFT_OF.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    facts = tuple(
        RelationFact(
            relation_id=relation_id,
            episode_id="episode_1",
            frame_id=frame_id,
            timestamp_s=timestamp_s,
            subject_id="obj_red",
            predicate=SpatialPredicate.LEFT_OF,
            object_id="obj_blue",
            evidence_frame_ids=(frame_id,),
        )
        for frame_id, timestamp_s in ((12, 0.5), (13, 0.6))
    )

    questions = generate_spatial_questions(facts)

    assert len(questions) == 1
    question = questions[0]
    assert (
        question.question_id
        == "question_6c4562bfb3f21f38d6086b9331af250ebd473ce9162af6bf6cb8a28d94978f83"
    )
    assert question.text == "Did obj_red ever appear left of obj_blue?"
    assert question.object_ids == ("obj_red", "obj_blue")
    public_record = question.model_dump(mode="json")
    assert (
        not {
            "relation_id",
            "frame_id",
            "timestamp_s",
            "evidence_frame_ids",
            "expected",
        }
        & public_record.keys()
    )


def test_spatial_question_ids_are_scoped_to_episode() -> None:
    relation_id = stable_id(
        "relation",
        {
            "object_ids": ("obj_red", "obj_blue"),
            "predicate": SpatialPredicate.LEFT_OF.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    fact = RelationFact(
        relation_id=relation_id,
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        subject_id="obj_red",
        predicate=SpatialPredicate.LEFT_OF,
        object_id="obj_blue",
        evidence_frame_ids=(12,),
    )

    first = generate_spatial_questions((fact,))[0]
    second = generate_spatial_questions((fact.model_copy(update={"episode_id": "episode_2"}),))[0]

    assert first.question_id != second.question_id


def test_rejects_facts_from_multiple_episodes() -> None:
    relation_id = stable_id(
        "relation",
        {
            "object_ids": ("obj_red", "obj_blue"),
            "predicate": SpatialPredicate.LEFT_OF.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    facts = tuple(
        RelationFact(
            relation_id=relation_id,
            episode_id=episode_id,
            frame_id=frame_id,
            timestamp_s=timestamp_s,
            subject_id="obj_red",
            predicate=SpatialPredicate.LEFT_OF,
            object_id="obj_blue",
            evidence_frame_ids=(frame_id,),
        )
        for episode_id, frame_id, timestamp_s in (
            ("episode_1", 12, 0.5),
            ("episode_2", 13, 0.6),
        )
    )

    with pytest.raises(ValueError, match="one episode"):
        generate_spatial_questions(facts)


def test_generates_balanced_temporal_questions_in_byte_stable_order() -> None:
    first = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 1, 2)
    second = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 4, 5)

    cases = generate_temporal_question_cases((second, first))

    assert tuple(
        (question.model_dump_json(), answer.model_dump_json()) for question, answer in cases
    ) == tuple(
        (question.model_dump_json(), answer.model_dump_json())
        for question, answer in generate_temporal_question_cases((first, second))
    )
    assert {(question.predicate, question.reference_ids) for question, _ in cases} == {
        (TemporalPredicate.BEFORE, (first.relation_id, second.relation_id)),
        (TemporalPredicate.AFTER, (second.relation_id, first.relation_id)),
        (TemporalPredicate.AFTER, (first.relation_id, second.relation_id)),
        (TemporalPredicate.BEFORE, (second.relation_id, first.relation_id)),
    }
    assert Counter((question.predicate, answer.expected) for question, answer in cases) == {
        (TemporalPredicate.BEFORE, True): 1,
        (TemporalPredicate.BEFORE, False): 1,
        (TemporalPredicate.AFTER, True): 1,
        (TemporalPredicate.AFTER, False): 1,
    }


def test_temporal_questions_use_human_readable_relation_descriptions() -> None:
    first = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 1, 2)
    second = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 4, 5)

    questions = tuple(question for question, _ in generate_temporal_question_cases((first, second)))

    assert all("relation_" not in question.text for question in questions)
    assert all("obj_red left of obj_blue" in question.text for question in questions)
    assert all("obj_green above obj_yellow" in question.text for question in questions)


def test_temporal_question_ids_are_scoped_to_episode() -> None:
    def question_ids(episode_id: str) -> set[str]:
        first = _interval(
            "obj_red",
            SpatialPredicate.LEFT_OF,
            "obj_blue",
            1,
            2,
            episode_id=episode_id,
        )
        second = _interval(
            "obj_green",
            SpatialPredicate.ABOVE,
            "obj_yellow",
            4,
            5,
            episode_id=episode_id,
        )
        return {
            question.question_id
            for question, _ in generate_temporal_question_cases((first, second))
        }

    assert question_ids("episode_1").isdisjoint(question_ids("episode_2"))


def test_omits_temporal_questions_when_any_interval_pair_is_non_strict() -> None:
    relation_a_early = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 1, 2)
    relation_a_overlap = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 4, 7)
    relation_b = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 5, 6)

    assert (
        generate_temporal_question_cases((relation_a_early, relation_a_overlap, relation_b)) == ()
    )


def test_omits_temporal_questions_with_bidirectional_interval_proofs() -> None:
    relation_a_early = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 1, 2)
    relation_b_early = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 4, 5)
    relation_b_late = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 7, 8)
    relation_a_late = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 10, 11)

    assert (
        generate_temporal_question_cases(
            (relation_a_late, relation_b_early, relation_a_early, relation_b_late)
        )
        == ()
    )


def test_requires_strict_frame_and_timestamp_interval_order() -> None:
    first = _interval("obj_red", SpatialPredicate.LEFT_OF, "obj_blue", 1, 2)
    touching = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 2, 4)
    overlapping = _interval("obj_green", SpatialPredicate.ABOVE, "obj_yellow", 1, 4)
    disagreeing = _interval(
        "obj_green",
        SpatialPredicate.ABOVE,
        "obj_yellow",
        4,
        5,
        start_timestamp_s=-0.2,
        end_timestamp_s=-0.1,
    )

    for unproven_second in (touching, overlapping, disagreeing):
        assert generate_temporal_question_cases((first, unproven_second)) == ()
