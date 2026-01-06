# Copyright 2025 Dimensional Inc.
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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat
from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.protocol.service.lcmservice import LCMConfig, LCMService, autoconf
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    import threading

logger = setup_logger()


@runtime_checkable
class LCMMsg(Protocol):
    msg_name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> LCMMsg:
        """Decode bytes into an LCM message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    topic: str = ""
    lcm_type: type[LCMMsg] | None = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.msg_name}"


class LCMPubSubBase(LCMService, PubSub[Topic, Any]):
    default_config = LCMConfig
    _stop_event: threading.Event
    _thread: threading.Thread | None
    _callbacks: dict[str, list[Callable[[Any], None]]]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._callbacks = {}

    def publish(self, topic: Topic, message: bytes) -> None:
        """Publish a message to the specified channel."""
        if self.l is None:
            logger.error("Tried to publish after LCM was closed")
            return

        self.l.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        if self.l is None:
            logger.error("Tried to subscribe after LCM was closed")

            def noop() -> None:
                pass

            return noop

        lcm_subscription = self.l.subscribe(str(topic), lambda _, msg: callback(msg, topic))

        # Set queue capacity to 10000 to handle high-volume bursts
        lcm_subscription.set_queue_capacity(10000)

        def unsubscribe() -> None:
            if self.l is None:
                return
            self.l.unsubscribe(lcm_subscription)

        return unsubscribe


class LCMEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(self, msg: LCMMsg, _: Topic) -> bytes:
        return msg.lcm_encode()

    def decode(self, msg: bytes, topic: Topic) -> LCMMsg:
        if topic.lcm_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no lcm_type specified"
            )
        return topic.lcm_type.lcm_decode(msg)


class JpegEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(self, msg: LCMMsg, _: Topic) -> bytes:
        return msg.lcm_jpeg_encode()  # type: ignore[attr-defined, no-any-return]

    def decode(self, msg: bytes, topic: Topic) -> LCMMsg:
        if topic.lcm_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no lcm_type specified"
            )
        return topic.lcm_type.lcm_jpeg_decode(msg)  # type: ignore[attr-defined, no-any-return]


class JpegSharedMemoryEncoderMixin(PubSubEncoderMixin[str, Image]):
    def __init__(self, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self.jpeg = TurboJPEG()
        self.quality = quality

    def encode(self, msg: Any, _topic: str) -> bytes:
        if not isinstance(msg, Image):
            raise ValueError("Can only encode images.")

        bgr_image = msg.to_bgr().to_opencv()
        return self.jpeg.encode(bgr_image, quality=self.quality)  # type: ignore[no-any-return]

    def decode(self, msg: bytes, _topic: str) -> Image:
        bgr_array = self.jpeg.decode(msg)
        return Image(data=bgr_array, format=ImageFormat.BGR)


class LCM(
    LCMEncoderMixin,
    LCMPubSubBase,
): ...


class PickleLCM(
    PickleEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...


class JpegLCM(
    JpegEncoderMixin,
    LCMPubSubBase,
): ...


__all__ = [
    "LCM",
    "JpegLCM",
    "LCMEncoderMixin",
    "LCMMsg",
    "LCMMsg",
    "LCMPubSubBase",
    "PickleLCM",
    "autoconf",
]
