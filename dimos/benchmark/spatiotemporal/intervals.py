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

"""Deterministic construction of relation intervals."""

from collections.abc import Sequence
from itertools import pairwise

from dimos.benchmark.spatiotemporal.models import (
    RelationFact,
    RelationId,
    RelationInterval,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import SCHEMA_VERSION, JsonValue, stable_id


def _validate_sample_schedules(facts: Sequence[RelationFact]) -> None:
    by_episode: dict[str, dict[int, float]] = {}
    timestamps_by_episode: dict[str, dict[float, int]] = {}
    for fact in facts:
        frames = by_episode.setdefault(fact.episode_id, {})
        timestamps = timestamps_by_episode.setdefault(fact.episode_id, {})
        if (fact.frame_id in frames and frames[fact.frame_id] != fact.timestamp_s) or (
            fact.timestamp_s in timestamps and timestamps[fact.timestamp_s] != fact.frame_id
        ):
            raise ValueError("conflicting sample schedule")
        frames[fact.frame_id] = fact.timestamp_s
        timestamps[fact.timestamp_s] = fact.frame_id

    for frames in by_episode.values():
        ordered_samples = sorted(frames.items())
        if any(
            current_timestamp >= next_timestamp
            for (_, current_timestamp), (_, next_timestamp) in pairwise(ordered_samples)
        ):
            raise ValueError("conflicting sample schedule")


def _build_interval(facts: Sequence[RelationFact]) -> RelationInterval:
    first = facts[0]
    last = facts[-1]
    preimage: dict[str, JsonValue] = {
        "end_frame_id": last.frame_id,
        "end_timestamp_s": last.timestamp_s,
        "episode_id": first.episode_id,
        "object_id": first.object_id,
        "predicate": first.predicate.value,
        "relation_id": first.relation_id,
        "schema_version": SCHEMA_VERSION,
        "start_frame_id": first.frame_id,
        "start_timestamp_s": first.timestamp_s,
        "subject_id": first.subject_id,
    }
    return RelationInterval(
        interval_id=stable_id("interval", preimage),
        relation_id=first.relation_id,
        episode_id=first.episode_id,
        subject_id=first.subject_id,
        predicate=first.predicate,
        object_id=first.object_id,
        start_frame_id=first.frame_id,
        end_frame_id=last.frame_id,
        start_timestamp_s=first.timestamp_s,
        end_timestamp_s=last.timestamp_s,
        evidence_frame_ids=tuple(fact.frame_id for fact in facts),
    )


def build_relation_intervals(facts: Sequence[RelationFact]) -> tuple[RelationInterval, ...]:
    """Coalesce consecutive facts that share episode and relation identity."""
    _validate_sample_schedules(facts)
    ordered = sorted(
        facts,
        key=lambda fact: (
            fact.episode_id,
            fact.relation_id,
            fact.frame_id,
            fact.timestamp_s,
        ),
    )
    groups: list[list[RelationFact]] = []
    for fact in ordered:
        if (
            groups
            and fact.episode_id == groups[-1][-1].episode_id
            and fact.relation_id == groups[-1][-1].relation_id
            and fact.frame_id == groups[-1][-1].frame_id + 1
        ):
            groups[-1].append(fact)
        else:
            groups.append([fact])
    return tuple(_build_interval(group) for group in groups)


def derive_temporal_predicate(
    first_relation_id: RelationId,
    second_relation_id: RelationId,
    intervals: Sequence[RelationInterval],
) -> TemporalPredicate | None:
    """Return a strict order only when all matching interval evidence agrees."""
    first = [interval for interval in intervals if interval.relation_id == first_relation_id]
    second = [interval for interval in intervals if interval.relation_id == second_relation_id]
    first_episodes = {interval.episode_id for interval in first}
    second_episodes = {interval.episode_id for interval in second}
    if not first or not second or first_episodes != second_episodes:
        return None

    predicates: set[TemporalPredicate] = set()
    for first_interval in first:
        for second_interval in second:
            if first_interval.episode_id != second_interval.episode_id:
                continue
            if (
                first_interval.end_frame_id < second_interval.start_frame_id
                and first_interval.end_timestamp_s < second_interval.start_timestamp_s
            ):
                predicates.add(TemporalPredicate.BEFORE)
            elif (
                second_interval.end_frame_id < first_interval.start_frame_id
                and second_interval.end_timestamp_s < first_interval.start_timestamp_s
            ):
                predicates.add(TemporalPredicate.AFTER)
            else:
                return None

    return predicates.pop() if len(predicates) == 1 else None
