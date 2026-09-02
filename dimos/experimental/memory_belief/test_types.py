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

"""Contract tests for belief records.

These lock the distinctions that make a negative answer trustworthy: evidence
that can be re-read, ignorance that cannot pass as absence, and a target's
position that cannot be confused with the vantage point it was seen from.
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path
import sqlite3
import tempfile

import pytest

from dimos.experimental.memory_belief.types import (
    SCHEMA_VERSION,
    BeliefObservation,
    Confidence,
    EvidenceRef,
    belief_tags,
)
from dimos.experimental.memory_belief.write import append_belief, belief_stream
from dimos.experimental.world_belief.absence import (
    ABSENT,
    OCCLUDED,
    OUT_OF_VIEW,
    PRESENT,
)
from dimos.memory.codecs.json import JsonCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

EV = (EvidenceRef(stream="color_image", observation_id=42, ts=100.0),)


def rec(**overrides) -> BeliefObservation:
    kwargs = {
        "schema_version": SCHEMA_VERSION,
        "target_ref": "chair-17",
        "visibility": "present",
        "valid_ts": 100.0,
        "observed_ts": 100.0,
        "source": "yoloe+depth",
        "evidence": EV,
        **overrides,
    }
    return BeliefObservation(**kwargs)


@pytest.fixture
def store():
    with tempfile.TemporaryDirectory() as tmp:
        s = SqliteStore(path=str(Path(tmp) / "belief.db"))
        try:
            yield s
        finally:
            s.stop()


class TestEvidenceIsMandatory:
    def test_a_record_without_evidence_is_rejected(self):
        """An untraceable claim is an assertion, not a belief."""
        with pytest.raises(ValueError, match="evidence"):
            rec(evidence=())

    def test_evidence_points_at_a_re_readable_observation(self, store):
        stream = belief_stream(store)
        append_belief(stream, rec())

        ref = stream.to_list()[0].data.evidence[0]

        assert (ref.stream, ref.observation_id) == ("color_image", 42)


class TestVisibilitySemantics:
    """absent is a claim; occluded and out_of_view are admissions."""

    def test_the_four_verdicts_stay_in_step_with_the_geometry(self):
        """These strings must equal the ones classify_visibility() emits.

        The two definitions are deliberately separate -- memory must not depend
        on an experimental perception module -- so this test is what catches
        drift between them.
        """

        for verdict in (PRESENT, ABSENT, OCCLUDED, OUT_OF_VIEW):
            rec(visibility=verdict)  # must not raise


class TestVantagePointIsNotLocation:
    def test_target_pose_requires_a_declared_frame(self):
        """Coordinates with no frame cannot be compared with anything."""
        with pytest.raises(ValueError, match="frame"):
            rec(target_pose=(1.0, 2.0, 3.0))

    def test_frame_without_a_pose_is_also_rejected(self):
        with pytest.raises(ValueError, match="frame"):
            rec(frame_id="map")

    def test_capture_pose_lives_on_the_envelope_not_the_payload(self, store):
        """Structural separation: the payload has no field to put it in."""

        stream = belief_stream(store)
        append_belief(
            stream,
            rec(target_pose=(5.0, 6.0, 7.0), frame_id="map"),
            capture_pose=PoseStamped(position=(1.0, 2.0, 3.0)),
        )

        obs = stream.to_list()[0]

        assert obs.pose.position.x == pytest.approx(1.0)  # where the robot looked from
        assert obs.data.target_pose == (5.0, 6.0, 7.0)  # where the thing is
        assert not hasattr(obs.data, "capture_pose")

    def test_a_record_may_carry_no_position_at_all(self):
        """No depth means no target position -- that is a valid, honest record."""
        assert rec().target_pose is None


class TestVocabularyAndSource:
    def test_source_is_mandatory(self):
        with pytest.raises(ValueError, match="source"):
            rec(source="")

    def test_closed_vocabulary_is_recorded_with_the_record(self, store):
        """A detector only asked about chairs cannot support "there is no mug"."""
        stream = belief_stream(store)
        append_belief(stream, rec(vocabulary=("chair", "person")))

        assert stream.to_list()[0].data.vocabulary == ("chair", "person")

    def test_open_vocabulary_records_none(self):
        assert rec(source="vlm").vocabulary is None


class TestConfidenceStaysSeparate:
    def test_the_three_confidences_are_independent(self):
        c = rec(confidence=Confidence(detection=0.9, association=0.4)).confidence

        assert (c.detection, c.association, c.attribute) == (0.9, 0.4, None)

    def test_confidence_defaults_to_all_unknown(self):
        c = rec().confidence

        assert (c.detection, c.association, c.attribute) == (None, None, None)


class TestTwoTimeAxes:
    def test_valid_and_observed_time_can_differ(self):
        """Replay and late correction: learned at 5pm what happened at 2pm."""
        r = rec(valid_ts=100.0, observed_ts=10_000.0)

        assert r.valid_ts != r.observed_ts

    def test_the_envelope_is_indexed_by_valid_time(self, store):
        """ "What was here at 2pm" is the query that has to push down."""
        stream = belief_stream(store)
        append_belief(stream, rec(valid_ts=100.0, observed_ts=10_000.0))

        obs = stream.to_list()[0]

        assert obs.ts == pytest.approx(100.0)
        assert obs.data.observed_ts == pytest.approx(10_000.0)

    def test_time_range_query_uses_valid_time(self, store):
        stream = belief_stream(store)
        append_belief(stream, rec(target_ref="a", valid_ts=100.0, observed_ts=9_000.0))
        append_belief(stream, rec(target_ref="b", valid_ts=500.0, observed_ts=9_000.0))

        found = stream.time_range(0.0, 200.0).to_list()

        assert [o.data.target_ref for o in found] == ["a"]


class TestTagProjection:
    def test_tags_are_derived_not_supplied(self, store):
        """No parameter exists to pass tags, so they cannot disagree."""

        assert "tags" not in inspect.signature(append_belief).parameters

    def test_tags_mirror_the_payload(self):
        r = rec(place_ref="kitchen", label="chair", identity_status="confirmed")
        tags = belief_tags(r)

        assert tags["target_ref"] == r.target_ref
        assert tags["visibility"] == r.visibility
        assert tags["source"] == r.source
        assert tags["place_ref"] == r.place_ref
        assert tags["identity_status"] == r.identity_status

    def test_absent_optional_fields_are_omitted_not_null(self):
        """A null tag would index as a value and match "place_ref is null" oddly."""
        assert "place_ref" not in belief_tags(rec())

    @pytest.mark.parametrize("tag", ["target_ref", "visibility", "source", "place_ref"])
    def test_tags_push_down_as_queries(self, store, tag):
        kept = rec(
            target_ref="chair-17", visibility="present", place_ref="kitchen", source="yoloe+depth"
        )
        other = rec(
            target_ref="mug-3",
            visibility="absent",
            place_ref="pantry",
            source="vlm",
            valid_ts=101.0,
        )
        stream = belief_stream(store)
        append_belief(stream, kept)
        append_belief(stream, other)

        wanted = belief_tags(kept)[tag]
        found = stream.tags(**{tag: wanted}).to_list()

        assert [o.data.target_ref for o in found] == ["chair-17"]


class TestPersistence:
    def test_records_survive_reopening_the_store(self, tmp_path):
        """The whole point of JSON over pickle -- and of not opening a fifth store."""
        path = str(tmp_path / "belief.db")
        s1 = SqliteStore(path=path)
        append_belief(belief_stream(s1), rec(vocabulary=("chair",)))
        s1.stop()

        s2 = SqliteStore(path=path, must_exist=True)
        try:
            reread = belief_stream(s2).to_list()[0].data
        finally:
            s2.stop()

        assert reread == rec(vocabulary=("chair",))

    def test_stored_bytes_are_readable_json(self, tmp_path):
        path = str(tmp_path / "belief.db")
        s = SqliteStore(path=path)
        append_belief(belief_stream(s), rec())
        s.stop()

        con = sqlite3.connect(path)
        try:
            row = con.execute('SELECT data FROM "belief_observation_blob" LIMIT 1').fetchone()
        finally:
            con.close()

        assert json.loads(bytes(row[0]))["target_ref"] == "chair-17"

    def test_unknown_field_from_a_newer_writer_is_reported(self, tmp_path):
        """A field this build does not know must surface, not vanish."""

        codec = JsonCodec(BeliefObservation)
        payload = codec.encode(rec()).replace(b"{", b'{"from_the_future":1,', 1)

        with pytest.raises(Exception, match="from_the_future"):
            codec.decode(payload)
