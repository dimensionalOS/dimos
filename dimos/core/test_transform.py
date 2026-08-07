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

from collections.abc import Callable
import pickle
from typing import Any

import numpy as np

from dimos.core.stream import Out, Stream, Transport
from dimos.core.transform import ResizeImage, Throttle, TransformTransport, VoxelDownsample
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class RecordingTransport(Transport[Any]):
    """Records broadcasts; subscribe hands back a marker unsubscribe."""

    def __init__(self) -> None:
        self.broadcasts: list[Any] = []
        self.started = False

    def broadcast(self, selfstream: Out[Any] | None, value: Any) -> None:
        self.broadcasts.append(value)

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        return lambda: None

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False


def add_one(msg: int) -> int:
    return msg + 1


def drop_odd(msg: int) -> int | None:
    return None if msg % 2 else msg


def test_transforms_apply_in_order() -> None:
    inner = RecordingTransport()
    transport = TransformTransport(inner, add_one, drop_odd)
    transport.publish(1)  # 1 -> 2 -> kept
    transport.publish(2)  # 2 -> 3 -> dropped
    assert inner.broadcasts == [2]


def test_none_drops_before_later_transforms() -> None:
    inner = RecordingTransport()
    transport = TransformTransport(inner, drop_odd, add_one)
    transport.publish(3)
    assert inner.broadcasts == []


def test_lifecycle_and_subscribe_passthrough() -> None:
    inner = RecordingTransport()
    transport = TransformTransport(inner, add_one)
    transport.start()
    assert inner.started
    assert callable(transport.subscribe(lambda msg: None))
    transport.stop()
    assert not inner.started


def test_throttle_keeps_first_of_interval() -> None:
    throttle = Throttle(max_hz=1000.0)
    assert throttle("a") == "a"
    assert throttle("b") is None  # within the 1ms interval


def test_resize_image() -> None:
    image = Image(data=np.zeros((1000, 2000, 3), dtype=np.uint8), format=ImageFormat.RGB)
    resized = ResizeImage(max_width=640, max_height=480)(image)
    assert (resized.width, resized.height) == (640, 320)


def test_pickle_roundtrip() -> None:
    transport = TransformTransport(RecordingTransport(), add_one, VoxelDownsample(0.05))
    restored = pickle.loads(pickle.dumps(transport))
    assert restored._transforms[1] == VoxelDownsample(0.05)
