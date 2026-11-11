# Copyright 2025-2026 Dimensional Inc.
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

"""Benchmark-only image codec for comparing raw and JPEG transport costs."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from dimos.core.stream import Stream
from dimos.core.transport import PubSubTransport
from dimos.message_utils import image as image_utils
from dimos_messages.sensor_msgs.msg import CompressedImage, Image


class CompressedCodec(PubSubTransport[Image]):
    """Image→CompressedImage jpeg codec over any inner transport.

    Bench-only utility: rejected as public API in #2831 (transports shouldn't
    peek into messages), kept here so raw-vs-codec cells stay comparable.
    The wire carries a typed sensor_msgs.CompressedImage; ts/frame_id survive.
    Subscribers always receive a decoded Image.
    """

    def __init__(self, inner: PubSubTransport[CompressedImage], quality: int = 75) -> None:
        super().__init__(inner.topic)
        self.inner = inner
        self.quality = quality

    def cbor_encode(self) -> dict[str, Any]:
        return {"inner": self.inner, "quality": self.quality}

    @classmethod
    def cbor_decode(cls, data: dict[str, Any]) -> CompressedCodec:
        return cls(data["inner"], data["quality"])

    def broadcast(self, stream: Stream[Image] | None, msg: Image) -> None:
        encoded = image_utils.to_compressed(msg, quality=self.quality)
        self.inner.broadcast(None, encoded)

    def subscribe(
        self, callback: Callable[[Image], Any], selfstream: Stream[Image] | None = None
    ) -> Callable[[], None]:
        return self.inner.subscribe(lambda m: callback(image_utils.from_compressed(m)))

    def start(self) -> None:
        self.inner.start()

    def stop(self) -> None:
        self.inner.stop()
