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

from collections.abc import Sequence

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.identity.reid import AppearanceReId
from dimos.perception.identity.test_reid import FakeEmbedder
from dimos.perception.identity.tracker import IdentityTracker


def _image(ts: float) -> Image:
    return Image.from_numpy(np.full((32, 32, 3), 80, dtype=np.uint8), ts=ts)


def _box(image: Image, name: str = "person") -> Detection2DBBox:
    return Detection2DBBox(
        bbox=(4.0, 4.0, 20.0, 20.0),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name=name,
        ts=image.ts,
        image=image,
    )


class FakeDetector:
    def __init__(self, names: Sequence[str]) -> None:
        self.names = list(names)
        self.queries: list[str] | None = None

    def query_detections(
        self, image: Image, queries: list[str], threshold: float = 0.3
    ) -> ImageDetections2D:
        self.queries = queries
        _ = threshold
        return ImageDetections2D(image, [_box(image, name) for name in self.names])


class FakeMot:
    def __init__(self, ids: Sequence[int]) -> None:
        self.ids = list(ids)
        self.frames = 0

    def update(
        self, detections: Sequence[Detection2DBBox], frame_bgr: np.ndarray
    ) -> list[Detection2DBBox]:
        self.frames += 1
        assert frame_bgr.shape[0] == 32
        from dataclasses import replace

        return [replace(det, track_id=self.ids[i]) for i, det in enumerate(detections)]


def test_identity_tracker_asks_omdet_then_writes_mot_ids() -> None:
    image = _image(1.0)
    tracker = IdentityTracker(FakeDetector(["person", "car"]), FakeMot([7, 8]))
    out = tracker.update(image)
    assert [det.track_id for det in out] == [7, 8]
    assert [det.name for det in out] == ["person", "car"]


def test_identity_tracker_runs_reid_after_mot() -> None:
    vec = np.array([1.0, 0.0], dtype=np.float32)
    reid = AppearanceReId(FakeEmbedder([vec]), threshold=0.9, update_every=100)
    tracker = IdentityTracker(FakeDetector(["person"]), FakeMot([4]), reid)
    first = tracker.update(_image(1.0))
    second = tracker.update(_image(1.1))
    assert first[0].track_id == second[0].track_id == 1
