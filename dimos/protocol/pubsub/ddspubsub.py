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

from collections.abc import Callable
from dataclasses import dataclass
import threading
from typing import Any, Protocol, TypeAlias, runtime_checkable

from cyclonedds.core import Listener
from cyclonedds.pub import DataWriter as DDSDataWriter
from cyclonedds.sub import DataReader as DDSDataReader
from cyclonedds.topic import Topic as DDSTopic

from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.ddsservice import DDSService
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@runtime_checkable
class DDSMsg(Protocol):
    msg_name: str

    @classmethod
    def dds_encode(cls, msg: DDSMsg) -> bytes:
        """Encode this message instance into bytes."""
        ...

    @classmethod
    def dds_decode(cls, data: bytes) -> DDSMsg:
        """Decode bytes into a DDS message instance."""
        ...


@dataclass(frozen=True)
class Topic:
    """Represents a DDS topic."""

    name: str
    typename: type[DDSMsg]

    def __hash__(self) -> int:
        return hash((self.name, self.typename))

    def __str__(self) -> str:
        return f"{self.name}#{self.typename.__name__}"


MessageCallback: TypeAlias = Callable[[Any, Topic], None]


class _DDSMessageListener(Listener):
    """Listener for DataReader that dispatches messages to callbacks."""

    __slots__ = ("callbacks", "topic")

    def __init__(self, topic: Topic, callbacks_dict: dict[Topic, list[MessageCallback]]) -> None:
        super().__init__()
        self.topic = topic
        self.callbacks = callbacks_dict[
            topic
        ]  # Cache callbacks list to avoid dict lookup per message

    def on_data_available(self, reader: DDSDataReader) -> None:
        """Called when data is available on the reader."""
        try:
            samples = reader.take()
        except Exception:
            return
        callbacks = self.callbacks
        topic = self.topic
        for sample in samples:
            if sample is not None:
                for callback in callbacks:
                    callback(sample, topic)


class DDSPubSubBase(DDSService, PubSub[Topic, Any]):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._callbacks: dict[Topic, list[MessageCallback]] = {}
        self._writers: dict[Topic, DDSDataWriter] = {}
        self._readers: dict[Topic, DDSDataReader] = {}
        self._writer_lock = threading.Lock()
        self._reader_lock = threading.Lock()

    def _get_writer(self, topic: Topic) -> DDSDataWriter:
        """Get or create a DataWriter for the given topic."""
        writer = self._writers.get(topic)
        if writer is None:
            with self._writer_lock:
                writer = self._writers.get(topic)
                if writer is None:
                    dds_topic = DDSTopic(self.get_participant(), topic.name, topic.typename)
                    writer = DDSDataWriter(self.get_participant(), dds_topic)
                    self._writers[topic] = writer
        return writer

    def publish(self, topic: Topic, message: Any) -> None:
        """Publish a message to a DDS topic."""
        writer = self._get_writer(topic)
        try:
            writer.write(message)
        except Exception as e:
            logger.error(f"Error publishing to topic {topic}: {e}")

    def _get_reader(self, topic: Topic) -> DDSDataReader:
        """Get or create a DataReader for the given topic with listener."""
        reader = self._readers.get(topic)
        if reader is None:
            with self._reader_lock:
                reader = self._readers.get(topic)
                if reader is None:
                    dds_topic = DDSTopic(self.get_participant(), topic.name, topic.typename)
                    listener = _DDSMessageListener(topic, self._callbacks)
                    reader = DDSDataReader(self.get_participant(), dds_topic, listener=listener)
                    self._readers[topic] = reader
        return reader

    def subscribe(self, topic: Topic, callback: MessageCallback) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""
        self._callbacks.setdefault(topic, [])  # Ensure list exists before creating reader
        self._get_reader(topic)
        self._callbacks[topic].append(callback)
        return lambda: self.unsubscribe_callback(topic, callback)

    def unsubscribe_callback(self, topic: Topic, callback: MessageCallback) -> None:
        """Unsubscribe a callback from a topic."""
        if topic in self._callbacks and callback in self._callbacks[topic]:
            self._callbacks[topic].remove(callback)
            if not self._callbacks[topic]:
                del self._callbacks[topic]


class DDS(DDSPubSubBase): ...


__all__ = [
    "DDS",
    "DDSMsg",
    "DDSPubSubBase",
    "MessageCallback",
    "Topic",
]
