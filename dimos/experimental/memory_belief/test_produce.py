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

"""Frame selection and darkness skipping, which lived in a CLI until now.

Both decide how many records a run produces and neither had a test: they were
closures over ``argparse`` results inside ``detect_recording.main``. A stride
off by one, or a brightness filter that counted the wrong frames, would have
shown up as "the robot saw less than expected" with nothing to point at.
"""

from __future__ import annotations

import numpy as np

from dimos.experimental.memory_belief.produce import DetectParams, detect_stream
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class FakeDetection:
    def __init__(self) -> None:
        self.name = "chair"
        self.confidence = 0.9
        self.box = (0.0, 0.0, 10.0, 10.0)
        self.track_id = None


class FakeResult:
    def __init__(self, detections: list[FakeDetection]) -> None:
        self.detections = detections


class FakeDetector:
    """Yields one detection per frame, and records how many frames it saw."""

    def __init__(self) -> None:
        self.seen = 0

    def process_image(self, image):  # type: ignore[no-untyped-def]
        self.seen += 1
        return FakeResult([FakeDetection()])


def frame(oid: int, ts: float, *, mean: int = 200) -> Observation[Image]:
    array = np.full((16, 16, 3), mean, np.uint8)
    return Observation(
        id=oid, ts=ts, _data=Image(data=array, format=ImageFormat.RGB, frame_id="cam", ts=ts)
    )


class TestFrameSelection:
    def test_stride_keeps_every_nth_frame(self):
        detector = FakeDetector()

        list(
            detect_stream(
                [frame(i, float(i)) for i in range(10)],
                detector,
                stream_name="color_image",
                vocabulary=None,
                source="test",
                stride=3,
            )
        )

        # 0, 3, 6, 9 -- counted from the first frame, not from the first kept
        # one, so two runs over the same recording select the same frames.
        assert detector.seen == 4

    def test_limit_counts_selected_frames_not_offered_ones(self):
        detector = FakeDetector()

        list(
            detect_stream(
                [frame(i, float(i)) for i in range(20)],
                detector,
                stream_name="color_image",
                vocabulary=None,
                source="test",
                stride=2,
                limit=3,
            )
        )

        assert detector.seen == 3


class TestDarknessIsReportedNotHidden:
    """A dark frame yields nothing, which reads exactly like an empty room."""

    def test_dark_frames_are_skipped_and_counted(self):
        skipped: list[int] = []
        detector = FakeDetector()

        records = list(
            detect_stream(
                [frame(1, 1.0, mean=5), frame(2, 2.0, mean=200), frame(3, 3.0, mean=5)],
                detector,
                stream_name="color_image",
                vocabulary=None,
                source="test",
                params=DetectParams(min_brightness=40.0),
                on_skip=lambda: skipped.append(1),
            )
        )

        assert len(records) == 1
        assert len(skipped) == 2

    def test_no_callback_is_not_an_error(self):
        records = list(
            detect_stream(
                [frame(1, 1.0, mean=5)],
                FakeDetector(),
                stream_name="color_image",
                vocabulary=None,
                source="test",
            )
        )

        assert records == []


class TestWithoutACameraNothingIsPlaced:
    def test_records_carry_no_position(self):
        records = list(
            detect_stream(
                [frame(1, 1.0)],
                FakeDetector(),
                stream_name="color_image",
                vocabulary=None,
                source="test",
            )
        )

        # A box says a thing was in frame, never where it is. Without a camera
        # to project through there is no depth, and inventing one is the failure
        # this layer exists to refuse.
        assert records
        assert all(r.target_pose is None for r in records)
        assert all(r.place_ref is None for r in records)
