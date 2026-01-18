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
import threading
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from cyclonedds.core import Listener
from cyclonedds.idl import IdlStruct
from cyclonedds.pub import DataWriter as DDSDataWriter
from cyclonedds.sub import DataReader as DDSDataReader

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.protocol.service.ddsservice import DDSConfig, DDSService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from cyclonedds.topic import Topic as DDSTopic

logger = setup_logger()


@runtime_checkable
class DDSMsg(Protocol):
    msg_name: str

    @classmethod
    def dds_decode(cls, data: bytes) -> DDSMsg:
        """Decode bytes into a DDS message instance."""
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


class DDSPubSubBase(DDSService, PubSub[Topic, Any]):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._callbacks: dict[DDSTopic, list[Callable[[Any, DDSTopic], None]]] = {}
        self._writers: dict[DDSTopic, DDSDataWriter] = {}
        self._readers: dict[str, DDSDataReader] = {}
        self._writer_lock = threading.Lock()
        self._reader_lock = threading.Lock()

    def _get_writer(self, topic: DDSTopic) -> DDSDataWriter:
        """Get a DataWriter for the given topic name, create if it does not exsist."""

        with self._writer_lock:
            if topic not in self._writers:
                writer = DDSDataWriter(self.get_participant(), topic)
                self._writers[topic] = writer
                logger.debug(f"Created DataWriter for topic: {topic.topic}")
            return self._writers[topic]

    def publish(self, topic: DDSTopic, message: Any) -> None:
        """Publish a message to a DDS topic."""

        writer = self._get_writer(topic)
        try:
            # Publish to DDS network
            writer.write(message)

        except Exception as e:
            logger.error(f"Error publishing to topic {topic}: {e}")

        # Dispatch to local subscribers
        if topic in self._callbacks:
            for callback in self._callbacks[topic]:
                try:
                    callback(message, topic)
                except Exception as e:
                    # Log but continue processing other callbacks
                    logger.error(f"Error in callback for topic {topic}: {e}")

    def _get_reader(self, topic: DDSTopic) -> DDSDataReader:
        """Get or create a DataReader for the given topic with listener."""

        with self._reader_lock:
            if topic not in self._readers:
                reader = DDSDataReader[Any](self.get_participant(), topic)
                self._readers[topic] = reader
                logger.debug(f"Created DataReader for topic: {topic.topic}")
            return self._readers[topic]

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""

        # Create a DataReader for this topic if needed
        self._get_reader(topic)

        # Add callback to our list
        if topic not in self._callbacks:
            self._callbacks[topic] = []
        self._callbacks[topic].append(callback)

        # Return unsubscribe function
        def unsubscribe() -> None:
            self.unsubscribe_callback(topic, callback)

        return unsubscribe

    def unsubscribe_callback(
        self, topic: DDSTopic, callback: Callable[[Any, DDSTopic], None]
    ) -> None:
        """Unsubscribe a callback from a topic."""
        try:
            if topic in self._callbacks:
                self._callbacks[topic].remove(callback)
                if not self._callbacks[topic]:
                    del self._callbacks[topic]
        except ValueError:
            pass


class DDSEncoderMixin(PubSubEncoderMixin[Topic, Any, IdlStruct]):
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
    "Topic",
]
