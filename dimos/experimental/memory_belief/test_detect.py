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

"""Contract tests for the 2D detection producer.

Mostly about what it refuses to claim. A flat image supports far less than it
appears to, and each of those limits has to survive into the record rather than
living in someone's memory of how the data was made.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from dimos.experimental.memory_belief.detect import bright_enough, detect_to_belief
from dimos.experimental.memory_belief.types import BeliefObservation
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

NOW = 1000.0


@dataclass
class FakeDetection:
    name: str
    confidence: float


@dataclass
class FakeResult:
    detections: list[FakeDetection]


class FakeDetector:
    """Returns a scripted result, and records what it was shown."""

    def __init__(self, detections: list[FakeDetection] | None = None) -> None:
        self.detections = detections if detections is not None else [FakeDetection("chair", 0.9)]
        self.seen = 0

    def process_image(self, image):
        self.seen += 1
        return FakeResult(list(self.detections))


def frame(oid: int = 1, ts: float = NOW, *, mean: int = 200, pose=True) -> Observation[Image]:
    array = np.full((16, 16, 3), mean, np.uint8)
    return Observation(
        id=oid,
        ts=ts,
        pose=PoseStamped(position=(2.0, 7.0, 0.0)) if pose else None,
        _data=Image(data=array, format=ImageFormat.RGB, frame_id="cam", ts=ts),
    )


def produce(frames, detector=None, **overrides) -> list[BeliefObservation]:
    kwargs = {
        "stream_name": "color_image",
        "vocabulary": ("chair", "person"),
        "source": "fake",
        **overrides,
    }
    return list(detect_to_belief(frames, detector or FakeDetector(), **kwargs))


class TestNoDepthMeansNoPosition:
    """A bounding box says a thing was in frame, not where it is."""

    def test_target_pose_is_never_filled(self):
        assert all(r.target_pose is None for r in produce([frame()]))

    def test_entity_place_is_never_filled(self):
        assert all(r.place_ref is None for r in produce([frame()]))

    def test_capture_place_is_filled_from_the_pose(self):
        (record,) = produce([frame()])

        assert record.capture_place_ref == "cell(0,1)"  # (2.0, 7.0) at 5m cells

    def test_a_frame_without_a_pose_records_no_place_at_all(self):
        assert produce([frame(pose=False)])[0].capture_place_ref is None


class TestNoAssociationMeansNoIdentity:
    def test_identity_status_is_none(self):
        assert all(r.identity_status == "none" for r in produce([frame()]))

    def test_the_same_object_in_two_frames_gets_two_targets(self):
        """Honest until a re-identification step exists."""
        records = produce([frame(oid=1), frame(oid=2)])

        assert len({r.target_ref for r in records}) == 2

    def test_target_refs_point_back_at_their_frame(self):
        (record,) = produce([frame(oid=42)])

        assert record.target_ref.startswith("color_image#42")


class TestOnlyPresenceIsClaimed:
    def test_every_record_is_present(self):
        assert all(r.visibility == "present" for r in produce([frame()]))

    def test_a_frame_with_no_detections_writes_nothing(self):
        """Silence from a detector is not evidence of absence."""
        records = produce([frame()], FakeDetector(detections=[]))

        assert records == []


class TestVocabularyTravelsWithTheRecord:
    def test_a_prompted_run_records_what_it_was_asked(self):
        (record,) = produce([frame()], vocabulary=("chair", "person"))

        assert record.vocabulary == ("chair", "person")

    def test_an_unprompted_run_records_none(self):
        (record,) = produce([frame()], vocabulary=None)

        assert record.vocabulary is None


class TestConfidence:
    def test_detection_confidence_is_kept_separate(self):
        (record,) = produce([frame()], FakeDetector([FakeDetection("chair", 0.77)]))

        assert record.confidence.detection == pytest.approx(0.77)
        assert record.confidence.association is None

    def test_low_confidence_detections_are_dropped_not_recorded(self):
        """Anything that exists at all is treated as evidence downstream."""
        detector = FakeDetector([FakeDetection("chair", 0.2), FakeDetection("door", 0.9)])

        records = produce([frame()], detector, min_confidence=0.5)

        assert [r.label for r in records] == ["door"]


class TestFrameSelection:
    def test_dark_frames_can_be_filtered_out(self):
        detector = FakeDetector()

        produce([frame(mean=5), frame(mean=200)], detector, frame_filter=bright_enough)

        assert detector.seen == 1

    def test_bright_enough_thresholds_on_mean_luminance(self):
        assert not bright_enough(frame(mean=5))
        assert bright_enough(frame(mean=200))

    def test_no_filter_means_every_frame_is_processed(self):
        detector = FakeDetector()

        produce([frame(mean=5), frame(mean=200)], detector)

        assert detector.seen == 2


class TestTimeAxes:
    def test_valid_time_comes_from_the_frame(self):
        (record,) = produce([frame(ts=123.0)])

        assert record.valid_ts == 123.0

    def test_observed_time_defaults_to_valid_time(self):
        (record,) = produce([frame(ts=123.0)])

        assert record.observed_ts == record.valid_ts


class TestEvidence:
    def test_each_record_points_back_at_its_frame(self):
        (record,) = produce([frame(oid=7, ts=5.0)])
        (ref,) = record.evidence

        assert (ref.stream, ref.observation_id, ref.ts) == ("color_image", 7, 5.0)

    def test_two_detections_in_one_frame_share_its_evidence(self):
        detector = FakeDetector([FakeDetection("chair", 0.9), FakeDetection("door", 0.9)])

        records = produce([frame(oid=3)], detector)

        assert {r.evidence[0].observation_id for r in records} == {3}


class TestPlaceResolution:
    def test_place_is_derived_from_coordinates_not_a_room_list(self):
        """Naming a place is a claim that needs its own evidence."""
        (record,) = produce([frame()], place_size_m=1.0)

        assert record.capture_place_ref == "cell(2,7)"
