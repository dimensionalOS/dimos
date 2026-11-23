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

from dimos.models.embedding.base import Embedding
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.identity.reid import AppearanceReId, ema_embedding


class FakeEmbedder:
    device = "cpu"

    def __init__(self, vectors: list[np.ndarray]) -> None:
        self.vectors = vectors
        self.calls = 0

    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        out: list[Embedding] = []
        for image in images:
            vec = self.vectors[min(self.calls, len(self.vectors) - 1)]
            self.calls += 1
            out.append(Embedding(vector=vec, timestamp=image.ts))
        return out[0] if len(out) == 1 else out


def _image(ts: float) -> Image:
    return Image.from_numpy(np.full((32, 32, 3), 40, dtype=np.uint8), ts=ts)


def _box(image: Image, track_id: int, name: str = "person") -> Detection2DBBox:
    return Detection2DBBox(
        bbox=(4.0, 4.0, 20.0, 20.0),
        track_id=track_id,
        class_id=0,
        confidence=0.9,
        name=name,
        ts=image.ts,
        image=image,
    )


def test_new_botsort_id_reuses_lost_identity_when_crop_matches() -> None:
    same = np.array([1.0, 0.0], dtype=np.float32)
    reid = AppearanceReId(FakeEmbedder([same, same]), threshold=0.9, update_every=100)
    first = _image(1.0)
    gap = _image(2.0)
    again = _image(3.0)
    a = reid.assign([_box(first, 1)], first)
    assert a[0].track_id == 1
    reid.assign([], gap)
    b = reid.assign([_box(again, 99)], again)
    assert b[0].track_id == 1


def test_new_botsort_id_stays_new_when_crop_does_not_match() -> None:
    reid = AppearanceReId(
        FakeEmbedder(
            [np.array([1.0, 0.0], dtype=np.float32), np.array([0.0, 1.0], dtype=np.float32)]
        ),
        threshold=0.9,
        update_every=100,
    )
    first = _image(1.0)
    gap = _image(2.0)
    again = _image(3.0)
    reid.assign([_box(first, 1)], first)
    reid.assign([], gap)
    out = reid.assign([_box(again, 99)], again)
    assert out[0].track_id == 2


def test_motion_match_keeps_identity_without_gallery_search() -> None:
    embedder = FakeEmbedder([np.array([1.0, 0.0], dtype=np.float32)])
    reid = AppearanceReId(embedder, threshold=0.9, update_every=100)
    t0 = _image(1.0)
    t1 = _image(1.1)
    first = reid.assign([_box(t0, 4)], t0)
    second = reid.assign([_box(t1, 4)], t1)
    assert first[0].track_id == second[0].track_id == 1
    assert embedder.calls == 1


def test_reset_treats_the_next_frame_as_a_new_track() -> None:
    vec = np.array([1.0, 0.0], dtype=np.float32)
    embedder = FakeEmbedder([vec])
    reid = AppearanceReId(embedder, threshold=0.9, update_every=100)
    t0 = _image(1.0)
    t1 = _image(2.0)
    reid.assign([_box(t0, 4)], t0)
    reid.assign([_box(t1, 4)], t1)
    assert embedder.calls == 1
    reid.reset()
    reid.assign([_box(t1, 4)], t1)
    assert embedder.calls == 2


def test_ema_embedding_stays_normalized_and_moves_toward_the_new_vector() -> None:
    old = Embedding(vector=np.array([1.0, 0.0], dtype=np.float32))
    new = Embedding(vector=np.array([0.0, 1.0], dtype=np.float32), timestamp=2.0)
    mixed = ema_embedding(old, new, alpha=0.75)
    value = mixed.to_numpy()
    assert abs(float(np.linalg.norm(value)) - 1.0) < 1e-5
    assert value[0] > value[1]
