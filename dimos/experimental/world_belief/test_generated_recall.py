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

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.experimental.world_belief.recall import build_frame_clip_index, index_stream_name
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.image import image_from_array, image_resize_to_fit, image_view
from dimos.msgs.time import time_from_seconds


class _EmbeddingModel:
    def __init__(self) -> None:
        self.seen: list[Image] = []

    def embed(self, *images: Image) -> list[np.ndarray]:
        self.seen.extend(images)
        return [np.array([1.0, 0.0], dtype=np.float32) for _ in images]


def test_index_selects_sharp_generated_frame_and_preserves_stamp_and_thumbnail() -> None:
    source, destination = MemoryStore(), MemoryStore()
    try:
        frames = source.stream("color_image", Image)
        sharp = np.tile(np.array([[64, 255], [255, 64]], dtype=np.uint8), (4, 8))
        sharp = np.repeat(sharp[:, :, None], 3, axis=2)
        image = image_from_array(
            sharp, encoding="rgb8", header=Header(stamp=time_from_seconds(0.2), frame_id="camera")
        )
        identity = Pose(
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0), position=Point(x=0.0, y=0.0, z=0.0)
        )
        frames.append(
            image_from_array(np.full_like(sharp, 150), encoding="rgb8", header=image.header),
            ts=0.1,
            pose=identity,
        )
        frames.append(image, ts=0.2, pose=identity)
        # A bright pose-less camera frame and a dark posed frame are ineligible.
        frames.append(image, ts=1.1)
        frames.append(
            image_from_array(np.zeros_like(sharp), encoding="rgb8", header=image.header),
            ts=2.1,
            pose=identity,
        )
        model = _EmbeddingModel()
        assert (
            build_frame_clip_index(
                source, model=model, index_store=destination, thumbnail_px=4, source_tag="test"
            )
            == 1
        )
        assert len(model.seen) == 1
        assert np.array_equal(image_view(model.seen[0]), sharp)
        result = destination.stream(
            index_stream_name("openai/clip-vit-base-patch32"), Image
        ).first()
        assert (result.data.width, result.data.height) == (4, 2)
        assert result.data.header == image.header
        assert result.ts == 0.2
        assert result.tags["rec"] == "test"
        assert (
            build_frame_clip_index(
                source, model=model, index_store=destination, thumbnail_px=4, source_tag="test"
            )
            == 0
        )
    finally:
        source.stop()
        destination.stop()


def test_generated_resize_keeps_small_images_and_rejects_invalid_dimensions() -> None:
    image = image_from_array(np.ones((2, 3, 3), dtype=np.uint8), encoding="bgr8")
    assert image_resize_to_fit(image, 5, 5) == (image, 1.0)
    with pytest.raises(ValueError, match="positive"):
        image_resize_to_fit(image, 0, 5)
