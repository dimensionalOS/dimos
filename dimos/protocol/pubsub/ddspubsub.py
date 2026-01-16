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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


@runtime_checkable
class DDSMsg(Protocol):
    msg_name: str

    @classmethod
    def dds_decode(cls, data: bytes) -> DDSMsg:
        """Decode bytes into an LCM message instance."""
        ...

    def dds_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    """Represents a DDS topic with optional type information."""

    topic: str = ""
    dds_type: type[DDSMsg] | None = None

    def __str__(self) -> str:
        if self.dds_type is None:
            return self.topic
        return f"{self.topic}#{self.dds_type.__name__}"

    def __hash__(self) -> int:
        return hash((self.topic, self.dds_type))

    def __eq__(self, other: Any) -> bool:
        return (
            isinstance(other, Topic)
            and self.topic == other.topic
            and self.dds_type == other.dds_type
        )


class DDSPubSubBase(PubSub[Topic, Any]):
    """Base DDS pub/sub implementation using in-memory transport.

    This provides a transport-agnostic DDS pub/sub system that can later be
    extended to use actual DDS implementations (e.g., cyclone-dds, rti-dds).
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._callbacks: dict[Topic, list[Callable[[Any, Topic], None]]] = {}

    def publish(self, topic: Topic, message: Any) -> None:
        """Publish a message to a DDS topic."""
        # Dispatch to all subscribers
        if topic in self._callbacks:
            for callback in self._callbacks[topic]:
                try:
                    callback(message, topic)
                except Exception as e:
                    # Log but continue processing other callbacks
                    logger.error(f"Error in callback for topic {topic}: {e}")

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""
        # Add callback to our list
        if topic not in self._callbacks:
            self._callbacks[topic] = []
        self._callbacks[topic].append(callback)

        # Return unsubscribe function
        def unsubscribe() -> None:
            self.unsubscribe_callback(topic, callback)

        return unsubscribe

    def unsubscribe_callback(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> None:
        """Unsubscribe a callback from a topic."""
        try:
            if topic in self._callbacks:
                self._callbacks[topic].remove(callback)
                if not self._callbacks[topic]:
                    del self._callbacks[topic]
        except ValueError:
            pass


class DDSEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(self, msg: DDSMsg, _: Topic) -> bytes:
        return msg.dds_encode()

    def decode(self, msg: bytes, topic: Topic) -> DDSMsg:
        if topic.dds_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no dds_type specified"
            )
        return topic.dds_type.dds_decode(msg)


class DDS(
    DDSEncoderMixin,
    DDSPubSubBase,
): ...


class PickleDDS(
    PickleEncoderMixin,
    DDSPubSubBase,
): ...


__all__ = [
    "DDS",
    "DDSEncoderMixin",
    "DDSMsg",
    "DDSPubSubBase",
    "PickleDDS",
]
