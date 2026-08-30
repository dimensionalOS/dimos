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

from __future__ import annotations

from dimos.experimental.memory_belief.entity import fold_entities
from dimos.experimental.memory_belief.identity import IdentityClaim
from dimos.experimental.memory_belief.types import SCHEMA_VERSION, BeliefObservation, EvidenceRef


def sighting(ref: str, **overrides) -> BeliefObservation:
    return BeliefObservation(
        **{
            "schema_version": SCHEMA_VERSION,
            "target_ref": ref,
            "label": "bottle",
            "visibility": "present",
            "valid_ts": 10.0,
            "observed_ts": 10.0,
            "source": "test",
            "evidence": (EvidenceRef(stream="color_image", observation_id=1, ts=10.0),),
            **overrides,
        }
    )


def claim(*refs: str) -> IdentityClaim:
    return IdentityClaim(
        schema_version=SCHEMA_VERSION,
        claim_id="c1",
        entity_id="e1",
        target_refs=refs,
        basis="tracker:test",
        confidence=0.6,
        valid_ts=10.0,
    )


class TestWhereTheSensorWasIsNotWhereTheThingIs:
    """The one confusion this layer exists to refuse.

    A sighting knows where the camera stood, never where the object stands.
    Letting ``capture_place_ref`` stand in for a missing ``place_ref`` turns
    "the robot was in the kitchen when it saw a bottle" into "there is a bottle
    in the kitchen". The two fields are one line apart in the record, so nothing
    but a test keeps them apart.

    This used to be asserted at the query layer, where an ``in_region`` clause
    on raw sightings was rejected outright. That query no longer exists -- the
    only select is entities -- so the guarantee now has to hold in the fold.
    """

    def test_a_capture_place_never_becomes_an_entity_place(self):
        sightings = [sighting("s1", capture_place_ref="kitchen", place_ref=None)]

        (entity,) = fold_entities(sightings, [claim("s1")])

        assert entity.place_ref is None

    def test_an_entity_place_is_carried_when_one_exists(self):
        sightings = [sighting("s1", capture_place_ref="kitchen", place_ref="cell(3,4)")]

        (entity,) = fold_entities(sightings, [claim("s1")])

        assert entity.place_ref == "cell(3,4)"


class TestFoldingIsByIdentityNotByLabel:
    def test_a_sighting_no_claim_covers_is_skipped(self):
        """ "Seen once" and "followed and lost" are different states.

        A singleton entity per unassociated detection would bury the second
        under a crowd of the first.
        """
        folded = list(fold_entities([sighting("s1"), sighting("s2")], [claim("s1")]))

        assert len(folded) == 1
        assert folded[0].support == 1
