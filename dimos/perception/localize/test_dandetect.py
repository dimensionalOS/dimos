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

from collections.abc import Generator
import math
from typing import Any

import numpy as np
import pytest

from dimos.memory.store.memory import MemoryStore
from dimos.message_utils.camera_info import camera_info_from_intrinsics
from dimos.message_utils.image import image_from_array
from dimos.message_utils.time import time_from_seconds, to_seconds
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.models.embedding.base import Embedding
from dimos.perception.localize import dandetect
from dimos.perception.localize.dandetect import DanDetector, NoSegmenter, Option
from dimos.perception.localize.localize import embed_index
from dimos.perception.localize.rig import Rig, RigCamera
from dimos.perception.localize.types import LocalizeParams
from dimos.protocol.tf.tf import MultiTBuffer

ROOM_PARAMS = LocalizeParams(candidate_floor=0.18, accept_score=0.32)


class _StubEmbedder:
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        vecs = [Embedding(np.array([1.0, 0.0], dtype=np.float32)) for _ in images]
        return vecs[0] if len(vecs) == 1 else vecs

    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        vecs = [Embedding(np.array([1.0, 0.0], dtype=np.float32)) for _ in texts]
        return vecs[0] if len(vecs) == 1 else vecs

    def stop(self) -> None:
        pass


class _TwoViewScoreDetector:
    def __init__(self, scores: tuple[float, float]) -> None:
        self.scores = {1.0: scores[0], 2.0: scores[1]}

    def query_score_rows(
        self, image: Image, queries: list[str], threshold: float
    ) -> tuple[np.ndarray, np.ndarray]:
        score = self.scores[to_seconds(image.header.stamp)]
        if score < threshold:
            return np.empty((0, 4)), np.empty((0, len(queries)))
        boxes = np.array([[10.0, 10.0, 30.0, 30.0]], dtype=np.float32)
        rows = np.full((1, len(queries)), score, dtype=np.float32)
        return boxes, rows

    def stop(self) -> None:
        pass


@pytest.fixture
def two_view_rig() -> Generator[tuple[Rig, Any], None, None]:
    store = MemoryStore()
    color = store.stream("color_image", Image)
    depth = store.stream("depth_image", Image)
    tf = MultiTBuffer(buffer_size=math.inf)
    info = camera_info_from_intrinsics(
        40, 40, 20, 20, 40, 40, header=Header(stamp=time_from_seconds(0), frame_id="cam")
    )

    for k, (ts, x_offset) in enumerate(((1.0, 0.0), (2.0, 0.02))):
        rgb = np.full((40, 40, 3), 120 + k, dtype=np.uint8)
        rgb[10:30, 10:30] = 220
        color.append(
            image_from_array(
                rgb, encoding="rgb8", header=Header(frame_id="cam", stamp=time_from_seconds(ts))
            ),
            ts=ts,
        )
        depth_mm = np.full((40, 40), 500, dtype=np.uint16)
        depth_mm[10:30, 10:30] = 450
        depth.append(
            image_from_array(
                depth_mm,
                encoding="16UC1",
                header=Header(frame_id="cam", stamp=time_from_seconds(ts)),
            ),
            ts=ts,
        )
        tf.receive_transform(
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(ts)),
                child_frame_id="cam",
                transform=Transform(
                    translation=Vector3(x_offset, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            )
        )

    rig = Rig(
        cameras={"cam": RigCamera(info=info, color=color, depth=depth)},
        tf=tf,
        world_frame="world",
    )
    index = embed_index(rig, _StubEmbedder(), 0.0, 3.0)  # type: ignore[arg-type]
    try:
        yield rig, index
    finally:
        store.stop()


@pytest.mark.parametrize(
    ("detector_name", "scores", "base", "fields", "expected_hits"),
    [
        # accept_score shift on non-default room params (floor 0.18, accept 0.32):
        # 0.25 fails owlv2 (0.32) but passes omdet's -0.10 shift (0.22).
        ("owlv2", (0.25, 0.25), ROOM_PARAMS, None, 0),
        ("omdet", (0.25, 0.25), ROOM_PARAMS, None, 1),
        # 0.40 passes owlv2 (0.32) but fails gdino's +0.15 shift (0.47).
        ("owlv2", (0.40, 0.40), ROOM_PARAMS, None, 1),
        ("gdino", (0.40, 0.40), ROOM_PARAMS, None, 0),
        ("gdino", (0.50, 0.50), ROOM_PARAMS, None, 1),
        # candidate_floor shift on non-default room params (floor 0.18, accept 0.32):
        # View 1 clears accept_score (0.60); view 2 is needed for min_views=2.
        # View 2 at 0.12 is dropped by owlv2 (0.18) but kept by omdet's -0.10 shift (0.08).
        ("owlv2", (0.60, 0.12), ROOM_PARAMS, None, 0),
        ("omdet", (0.60, 0.12), ROOM_PARAMS, None, 1),
        # View 2 at 0.22 is kept by owlv2 (0.18) but dropped by gdino's +0.10 shift (0.28).
        ("owlv2", (0.60, 0.22), ROOM_PARAMS, None, 1),
        ("gdino", (0.60, 0.22), ROOM_PARAMS, None, 0),
        # Per-call field overrides win over both accept_score and candidate_floor shifts.
        ("gdino", (0.40, 0.40), ROOM_PARAMS, {"accept_score": 0.38}, 1),
        ("gdino", (0.60, 0.22), ROOM_PARAMS, {"candidate_floor": 0.20}, 1),
        # Default desk params LocalizeParams() (floor 0.25, accept 0.40):
        # accept_score shifts:
        ("omdet", (0.32, 0.32), LocalizeParams(), None, 1),
        ("gdino", (0.50, 0.50), LocalizeParams(), None, 0),
        ("gdino", (0.56, 0.56), LocalizeParams(), None, 1),
        # candidate_floor shifts:
        ("owlv2", (0.60, 0.20), LocalizeParams(), None, 0),
        ("omdet", (0.60, 0.20), LocalizeParams(), None, 1),
        ("owlv2", (0.60, 0.30), LocalizeParams(), None, 1),
        ("gdino", (0.60, 0.30), LocalizeParams(), None, 0),
    ],
)
def test_shifted_detector_scales_gate_localization(
    monkeypatch: pytest.MonkeyPatch,
    two_view_rig: tuple[Rig, Any],
    detector_name: str,
    scores: tuple[float, float],
    base: LocalizeParams,
    fields: dict[str, Any] | None,
    expected_hits: int,
) -> None:
    rig, index = two_view_rig

    def fake_build(option: Option) -> Any:
        if option is dandetect.EMBEDDERS["siglip"]:
            return _StubEmbedder()
        if option is dandetect.SEGMENTERS["none"]:
            return NoSegmenter()
        return _TwoViewScoreDetector(scores)

    monkeypatch.setattr(dandetect, "_build", fake_build)

    with DanDetector(embedder="siglip", detector=detector_name, segmenter="none") as dan:
        (hits,) = dan.localize(["box"], index=index, rig=rig, params=base, fields=fields)

    assert len(hits) == expected_hits
