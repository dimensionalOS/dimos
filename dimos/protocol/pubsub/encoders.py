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

"""Encoder mixins for PubSub implementations."""

from __future__ import annotations

from abc import ABC, abstractmethod
import pickle
from typing import TYPE_CHECKING, Generic, Protocol, TypeVar, cast

from dimos.msgs import DimosMsg
from dimos.msgs.sensor_msgs import Image

if TYPE_CHECKING:
    from collections.abc import Callable

TopicT = TypeVar("TopicT")
MsgT = TypeVar("MsgT")
EncodingT = TypeVar("EncodingT")


class DecodingError(Exception):
    """Raised by decode() to skip a message without calling the callback."""

    pass


class PubSubEncoderMixin(Generic[TopicT, MsgT, EncodingT], ABC):
    """Mixin that encodes messages before publishing and decodes them after receiving.

    This will override publish and subscribe methods to add encoding/decoding.
    Must be mixed with a class implementing PubSubProtocol[TopicT, EncodingT].

    Usage: Just specify encoder and decoder as a subclass:

    class MyPubSubWithJSON(PubSubEncoderMixin, MyPubSub):
        def encoder(msg, topic):
            json.dumps(msg).encode('utf-8')
        def decoder(msg, topic):
            data: json.loads(data.decode('utf-8'))
    """

    # Declare expected methods from PubSubProtocol for type checking
    if TYPE_CHECKING:
        _base_publish: Callable[[TopicT, EncodingT], None]
        _base_subscribe: Callable[[TopicT, Callable[[EncodingT, TopicT], None]], Callable[[], None]]

    @abstractmethod
    def encode(self, msg: MsgT, topic: TopicT) -> EncodingT: ...

    @abstractmethod
    def decode(self, msg: EncodingT, topic: TopicT) -> MsgT: ...

    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Encode the message and publish it."""
        encoded_message = self.encode(message, topic)
        if encoded_message is None:
            return
        super().publish(topic, encoded_message)  # type: ignore[misc]

    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe with automatic decoding."""

        def wrapper_cb(encoded_data: EncodingT, topic: TopicT) -> None:
            try:
                decoded_message = self.decode(encoded_data, topic)
            except DecodingError:
                return
            callback(decoded_message, topic)

        return cast("Callable[[], None]", super().subscribe(topic, wrapper_cb))  # type: ignore[misc]


class PickleEncoderMixin(PubSubEncoderMixin[TopicT, MsgT, bytes]):
    """Encoder mixin that uses pickle for serialization. Works with any Python object."""

    def encode(self, msg: MsgT, _: TopicT) -> bytes:
        return pickle.dumps(msg)

    def decode(self, msg: bytes, _: TopicT) -> MsgT:
        return cast("MsgT", pickle.loads(msg))


class LCMTopicProto(Protocol):
    """Protocol for topics usable with LCM encoders."""

    topic: str  # At decode time, always concrete string
    lcm_type: type[DimosMsg] | None


class LCMEncoderMixin(PubSubEncoderMixin[LCMTopicProto, DimosMsg, bytes]):
    """Encoder mixin for DimosMsg using LCM binary encoding."""

    def encode(self, msg: DimosMsg | bytes, _: LCMTopicProto) -> bytes:
        if isinstance(msg, bytes):
            return msg
        return msg.lcm_encode()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> DimosMsg:
        if topic.lcm_type is None:
            raise DecodingError(f"Cannot decode: topic {topic.topic!r} has no lcm_type")
        return topic.lcm_type.lcm_decode(msg)


class JpegEncoderMixin(PubSubEncoderMixin[LCMTopicProto, Image, bytes]):
    """Encoder mixin for DimosMsg using JPEG encoding (for images)."""

    def encode(self, msg: Image, _: LCMTopicProto) -> bytes:
        return msg.lcm_jpeg_encode()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> Image:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.lcm_type is None:
            raise DecodingError(f"Cannot decode: topic {topic.topic!r} has no lcm_type")
        return cast("type[Image]", topic.lcm_type).lcm_jpeg_decode(msg)
