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

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.identity.mot import BotSortMot, assign_track_ids


def _image(ts: float = 1.0) -> Image:
    return Image.from_numpy(np.zeros((32, 32, 3), dtype=np.uint8), ts=ts)


def _box(image: Image, name: str = "person", track_id: int = -1) -> Detection2DBBox:
    return Detection2DBBox(
        bbox=(4.0, 4.0, 20.0, 20.0),
        track_id=track_id,
        class_id=0,
        confidence=0.9,
        name=name,
        ts=image.ts,
        image=image,
    )


def test_assign_track_ids_writes_ids_and_drops_negatives() -> None:
    image = _image()
    a = _box(image, "person")
    b = _box(image, "car")
    out = assign_track_ids([a, b], [7, -1, 8], [0, 1, 1])
    assert [det.track_id for det in out] == [7, 8]
    assert [det.name for det in out] == ["person", "car"]


def test_assign_track_ids_reorders_by_input_index() -> None:
    image = _image()
    a = _box(image, "person")
    b = _box(image, "car")
    out = assign_track_ids([a, b], [3, 4], [1, 0])
    assert [det.name for det in out] == ["car", "person"]
    assert [det.track_id for det in out] == [3, 4]


def test_botsort_mot_passes_frame_and_writes_ids() -> None:
    import pytest

    pytest.importorskip("supervision")
    image = _image()
    detections = [_box(image, "person"), _box(image, "car")]

    class Tracker:
        def __init__(self) -> None:
            self.frame = None

        def update(self, sv_dets, frame=None):  # type: ignore[no-untyped-def]
            self.frame = frame
            n = len(sv_dets)
            sv_dets.tracker_id = np.arange(1, n + 1, dtype=np.int32)
            return sv_dets

    mot = BotSortMot(tracker=Tracker())
    frame = image.to_opencv()
    out = mot.update(detections, frame)
    assert mot._tracker.frame is frame
    assert [det.track_id for det in out] == [1, 2]
    assert [det.name for det in out] == ["person", "car"]
