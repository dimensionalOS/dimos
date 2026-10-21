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
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar, cast

from dimos.protocol.pubsub.codec import MessageDescriptor

if TYPE_CHECKING:
    from collections.abc import Callable

TopicT = TypeVar("TopicT")
MsgT = TypeVar("MsgT")
EncodingT = TypeVar("EncodingT")


class DecodingError(Exception):
    """Raised by decode() to skip a message without calling the callback."""


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

    def decode_received(self, payload: EncodingT, declared: TopicT, received: TopicT) -> MsgT:
        return self.decode(payload, received)

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

        declared_topic = topic

        def wrapper_cb(encoded_data: EncodingT, topic: TopicT) -> None:
            try:
                decoded_message = self.decode_received(encoded_data, declared_topic, topic)
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
    """Compatibility interface for legacy encoders such as JPEG."""

    topic: str
    lcm_type: type[Any] | None


class MessageTopic(Protocol):
    @property
    def message(self) -> MessageDescriptor[Any] | None: ...


class MessageEncoderMixin(PubSubEncoderMixin[MessageTopic, Any, bytes]):
    """Apply the topic's bound codec without changing transport behavior."""

    def subscribe(
        self, topic: MessageTopic, callback: Callable[[Any, MessageTopic], None]
    ) -> Callable[[], None]:
        descriptor = topic.message
        if descriptor is not None and descriptor.encoding == "lcm":
            warmup = getattr(descriptor.message_type, "lcm_warmup", None)
            if warmup is not None:
                warmup()

        return super().subscribe(topic, callback)

    def decode_received(
        self, payload: bytes, declared: MessageTopic, received: MessageTopic
    ) -> Any:
        descriptor = declared.message
        actual = received.message
        if descriptor is not None and descriptor.encoding == "cdr":
            if (
                actual is None
                or actual.encoding != "cdr"
                or actual.message_type is not descriptor.message_type
            ):
                raise DecodingError("Message metadata does not match the declared type")
            return self.decode(payload, declared)
        if descriptor is not None and actual is not None and actual.encoding != descriptor.encoding:
            raise DecodingError("Message encoding does not match the declared type")
        return self.decode(payload, received)

    def encode(self, msg: Any, topic: MessageTopic) -> bytes:
        descriptor = topic.message
        if isinstance(msg, bytes) and (descriptor is None or descriptor.encoding == "lcm"):
            return msg
        if descriptor is None:
            raise TypeError("Typed publication requires a declared message type")
        return descriptor.codec.encode(msg)

    def decode(self, msg: bytes, topic: MessageTopic) -> Any:
        if topic.message is None:
            raise DecodingError("Typed subscription requires a declared message type")
        try:
            return topic.message.codec.decode(msg)
        except ValueError as error:
            raise DecodingError(str(error)) from error


# Keep existing imports working while callers migrate to the generic name.
LCMEncoderMixin = MessageEncoderMixin
