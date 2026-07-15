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

"""Tests for deterministic relation interval construction."""

import pytest

from dimos.benchmark.spatiotemporal.models import (
    RelationFact,
    RelationInterval,
    SpatialPredicate,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import SCHEMA_VERSION, stable_id


def _relation_id(subject_id: str, predicate: SpatialPredicate, object_id: str) -> str:
    return stable_id(
        "relation",
        {
            "object_ids": (subject_id, object_id),
            "predicate": predicate.value,
            "schema_version": SCHEMA_VERSION,
        },
    )


def _fact(
    frame_id: int,
    timestamp_s: float,
    *,
    episode_id: str = "episode_1",
    subject_id: str = "mug_1",
    predicate: SpatialPredicate = SpatialPredicate.LEFT_OF,
    object_id: str = "laptop_1",
) -> RelationFact:
    return RelationFact(
        relation_id=_relation_id(subject_id, predicate, object_id),
        episode_id=episode_id,
        frame_id=frame_id,
        timestamp_s=timestamp_s,
        subject_id=subject_id,
        predicate=predicate,
        object_id=object_id,
        evidence_frame_ids=(frame_id,),
    )


def test_coalesces_relations_at_consecutive_sparse_samples() -> None:
    from dimos.benchmark.spatiotemporal.intervals import build_relation_intervals

    intervals = build_relation_intervals(
        (_fact(0, 0.0), _fact(150, 5.0)),
        sample_schedule=((0, 0.0), (150, 5.0), (300, 10.0)),
    )

    assert len(intervals) == 1
    assert intervals[0].start_frame_id == 0
    assert intervals[0].end_frame_id == 150
    assert intervals[0].evidence_frame_ids == (0, 150)


def test_relation_absent_at_intervening_sample_breaks_interval() -> None:
    from dimos.benchmark.spatiotemporal.intervals import build_relation_intervals

    intervals = build_relation_intervals(
        (_fact(0, 0.0), _fact(300, 10.0)),
        sample_schedule=((0, 0.0), (150, 5.0), (300, 10.0)),
    )

    assert [(interval.start_frame_id, interval.end_frame_id) for interval in intervals] == [
        (0, 0),
        (300, 300),
    ]


def test_coalesces_only_consecutive_samples_with_stable_relation_identity() -> None:
    from dimos.benchmark.spatiotemporal.intervals import build_relation_intervals

    intervals = build_relation_intervals(
        (
            _fact(1, 0.1),
            _fact(2, 0.2),
            _fact(
                2,
                0.2,
                subject_id="lamp_1",
                predicate=SpatialPredicate.ABOVE,
                object_id="table_1",
            ),
            _fact(4, 0.4),
            _fact(1, 0.1, episode_id="episode_2"),
        )
    )

    assert [
        (
            interval.episode_id,
            interval.subject_id,
            interval.predicate,
            interval.object_id,
            interval.start_frame_id,
            interval.end_frame_id,
            interval.evidence_frame_ids,
        )
        for interval in intervals
    ] == [
        (
            "episode_1",
            "lamp_1",
            SpatialPredicate.ABOVE,
            "table_1",
            2,
            2,
            (2,),
        ),
        (
            "episode_1",
            "mug_1",
            SpatialPredicate.LEFT_OF,
            "laptop_1",
            1,
            2,
            (1, 2),
        ),
        (
            "episode_1",
            "mug_1",
            SpatialPredicate.LEFT_OF,
            "laptop_1",
            4,
            4,
            (4,),
        ),
        (
            "episode_2",
            "mug_1",
            SpatialPredicate.LEFT_OF,
            "laptop_1",
            1,
            1,
            (1,),
        ),
    ]


@pytest.mark.parametrize(
    "facts",
    [
        (
            _fact(1, 0.1),
            _fact(
                1,
                0.2,
                subject_id="lamp_1",
                predicate=SpatialPredicate.ABOVE,
                object_id="table_1",
            ),
        ),
        (_fact(1, 0.1), _fact(2, 0.1)),
        (_fact(1, 0.2), _fact(2, 0.1)),
    ],
)
def test_rejects_conflicting_episode_sample_schedules(
    facts: tuple[RelationFact, ...],
) -> None:
    from dimos.benchmark.spatiotemporal.intervals import build_relation_intervals

    with pytest.raises(ValueError, match="conflicting sample schedule"):
        build_relation_intervals(facts)


def test_derives_only_unanimous_strict_temporal_ordering() -> None:
    from dimos.benchmark.spatiotemporal.intervals import (
        build_relation_intervals,
        derive_temporal_predicate,
    )

    earlier = _relation_id("mug_1", SpatialPredicate.LEFT_OF, "laptop_1")
    later = _relation_id("lamp_1", SpatialPredicate.ABOVE, "table_1")

    def paired_intervals(
        first_frames: tuple[int, ...], second_frames: tuple[int, ...]
    ) -> tuple[RelationInterval, ...]:
        return build_relation_intervals(
            tuple(_fact(frame, frame / 10) for frame in first_frames)
            + tuple(
                _fact(
                    frame,
                    frame / 10,
                    subject_id="lamp_1",
                    predicate=SpatialPredicate.ABOVE,
                    object_id="table_1",
                )
                for frame in second_frames
            )
        )

    ordered_facts = (
        _fact(1, 0.1),
        _fact(2, 0.2),
        _fact(
            4,
            0.4,
            subject_id="lamp_1",
            predicate=SpatialPredicate.ABOVE,
            object_id="table_1",
        ),
        _fact(
            5,
            0.5,
            subject_id="lamp_1",
            predicate=SpatialPredicate.ABOVE,
            object_id="table_1",
        ),
    )
    ordered = build_relation_intervals(ordered_facts)

    assert derive_temporal_predicate(earlier, later, ordered) is TemporalPredicate.BEFORE
    assert derive_temporal_predicate(later, earlier, ordered) is TemporalPredicate.AFTER
    assert (
        derive_temporal_predicate(earlier, later, tuple(reversed(ordered)))
        is TemporalPredicate.BEFORE
    )

    touching = paired_intervals((1, 2), (2, 3))
    overlapping = paired_intervals((1, 2, 3), (2, 3, 4))
    containing = paired_intervals((1, 2, 3, 4, 5), (2, 3, 4))
    contradictory = build_relation_intervals(
        (
            *ordered_facts,
            _fact(4, 0.4, episode_id="episode_2"),
            _fact(
                1,
                0.1,
                episode_id="episode_2",
                subject_id="lamp_1",
                predicate=SpatialPredicate.ABOVE,
                object_id="table_1",
            ),
        )
    )
    coordinate_contradiction = build_relation_intervals((_fact(1, 0.4),)) + (
        build_relation_intervals(
            (
                _fact(
                    4,
                    0.1,
                    subject_id="lamp_1",
                    predicate=SpatialPredicate.ABOVE,
                    object_id="table_1",
                ),
            )
        )
    )

    assert derive_temporal_predicate(earlier, later, touching) is None
    assert derive_temporal_predicate(earlier, later, overlapping) is None
    assert derive_temporal_predicate(earlier, later, containing) is None
    assert derive_temporal_predicate(earlier, "relation_" + "0" * 64, ordered) is None
    assert derive_temporal_predicate(earlier, later, contradictory) is None
    assert derive_temporal_predicate(earlier, later, coordinate_contradiction) is None
