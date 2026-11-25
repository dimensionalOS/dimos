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
from typing import TYPE_CHECKING, Generic, Protocol, TypeVar, cast

from dimos.msgs.protocol import DimosMsg
from dimos.protocol.encode import cbor
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()

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


class CborEncoderMixin(PubSubEncoderMixin[TopicT, MsgT, bytes]):
    """Encoder mixin using the CBOR codec (dimos.protocol.encode.cbor).

    Encoding an unsupported value raises TypeError out of publish(); a message
    that does not decode is logged and dropped. RPC keeps its bodies as bytes
    inside a primitives-only envelope (see pubsubrpc) so that a body this
    process cannot decode is answered with the error instead of dropped here.
    """

    def encode(self, msg: MsgT, _: TopicT) -> bytes:
        return cbor.encode(msg)

    def decode(self, msg: bytes, topic: TopicT) -> MsgT:
        try:
            return cast("MsgT", cbor.decode(msg))
        except cbor.CborDecodeError as e:
            logger.error(
                "Dropping undecodable CBOR message: is another dimos version on the bus?",
                topic=str(topic),
                error=str(e),
            )
            raise DecodingError(str(e)) from e


class LCMTopicProto(Protocol):
    """Protocol for topics usable with LCM encoders."""

    topic: str  # At decode time, always concrete string
    lcm_type: type[DimosMsg] | None


class LCMEncoderMixin(PubSubEncoderMixin[LCMTopicProto, DimosMsg, bytes]):
    """Encoder mixin for DimosMsg using LCM binary encoding."""

    def subscribe(
        self, topic: LCMTopicProto, callback: Callable[[DimosMsg, LCMTopicProto], None]
    ) -> Callable[[], None]:
        # Import the type's heavy decode dependencies now, on the subscriber's
        # thread. Left to the first decode, the import would run on the LCM
        # handler thread and block it for seconds, dropping messages meanwhile.
        warmup = getattr(topic.lcm_type, "lcm_warmup", None)
        if warmup is not None:
            warmup()
        return super().subscribe(topic, callback)

    def encode(self, msg: DimosMsg | bytes, _: LCMTopicProto) -> bytes:
        if isinstance(msg, bytes):
            return msg
        return msg.lcm_encode()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> DimosMsg:
        if topic.lcm_type is None:
            raise DecodingError(f"Cannot decode: topic {topic.topic!r} has no lcm_type")
        return topic.lcm_type.lcm_decode(msg)
