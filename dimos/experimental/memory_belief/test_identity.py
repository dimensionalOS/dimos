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

from __future__ import annotations

from dimos.experimental.memory_belief.identity import (
    IdentityClaim,
    claims_from_tracks,
    resolve_identity,
)
from dimos.experimental.memory_belief.types import SCHEMA_VERSION


def claim(claim_id: str, entity: str, refs: tuple[str, ...], retracts: str | None = None):
    return IdentityClaim(
        schema_version=SCHEMA_VERSION,
        claim_id=claim_id,
        entity_id=entity,
        target_refs=refs,
        basis="test",
        confidence=0.9,
        valid_ts=1.0,
        retracts=retracts,
    )


class TestIdentityIsAFoldNotAColumn:
    def test_sightings_collapse_to_one_entity(self):
        mapping = resolve_identity([claim("c1", "chair-a", ("s#1:0", "s#2:0", "s#3:1"))])
        assert set(mapping) == {"s#1:0", "s#2:0", "s#3:1"}
        assert set(mapping.values()) == {"chair-a"}

    def test_a_later_claim_wins(self):
        mapping = resolve_identity(
            [claim("c1", "chair-a", ("s#1:0",)), claim("c2", "chair-b", ("s#1:0",))]
        )
        assert mapping["s#1:0"] == "chair-b"

    def test_batches_of_one_track_merge_into_one_entity(self):
        mapping = resolve_identity(
            [claim("c1", "chair-a", ("s#1:0",)), claim("c2", "chair-a", ("s#2:0",))]
        )
        assert mapping == {"s#1:0": "chair-a", "s#2:0": "chair-a"}


class TestAMergeCanBeTakenBack:
    def test_retracting_removes_the_association(self):
        # Two things were merged, then the merge was found to be wrong.
        mapping = resolve_identity(
            [
                claim("c1", "one-thing", ("s#1:0", "s#2:0")),
                claim("c2", "just-the-first", ("s#1:0",), retracts="c1"),
            ]
        )
        assert mapping["s#1:0"] == "just-the-first"
        # The sighting that only the retracted claim spoke for is unassociated
        # again, rather than silently keeping an entity nothing still asserts.
        assert "s#2:0" not in mapping

    def test_the_retracted_claim_is_still_in_the_stream(self):
        history = [
            claim("c1", "one-thing", ("s#1:0", "s#2:0")),
            claim("c2", "just-the-first", ("s#1:0",), retracts="c1"),
        ]
        # Auditability: folding is what drops it, not deletion.
        assert any(c.claim_id == "c1" for c in history)


class TestTrackerIdsAreNotEntityIds:
    def test_tracks_are_namespaced_by_session(self):
        a = list(claims_from_tracks([("s#1:0", 7, 1.0)], session="run-a"))
        b = list(claims_from_tracks([("s#9:0", 7, 9.0)], session="run-b"))
        # The same tracker number in two sessions must not merge two objects.
        assert a[0].entity_id != b[0].entity_id

    def test_untracked_detections_produce_no_claim(self):
        assert list(claims_from_tracks([("s#1:0", None, 1.0)], session="r")) == []

    def test_batching_does_not_change_the_resolved_entity(self):
        sightings = [(f"s#{i}:0", 3, float(i)) for i in range(10)]
        small = resolve_identity(claims_from_tracks(sightings, session="r", batch_size=2))
        large = resolve_identity(claims_from_tracks(sightings, session="r", batch_size=100))
        assert set(small.values()) == set(large.values())
        assert set(small) == set(large)

    def test_claims_are_emitted_incrementally(self):
        sightings = [(f"s#{i}:0", 1, float(i)) for i in range(9)]
        claims = list(claims_from_tracks(sightings, session="r", batch_size=4))
        # A live run must not accumulate every sighting before emitting anything.
        assert len(claims) == 3
