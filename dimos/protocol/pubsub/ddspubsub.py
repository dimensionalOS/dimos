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
from typing import TYPE_CHECKING, Any, TypeAlias

from cyclonedds.core import Listener
from cyclonedds.pub import DataWriter as DDSDataWriter
from cyclonedds.sub import DataReader as DDSDataReader
from cyclonedds.topic import Topic as DDSTopic

from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.ddsservice import DDSService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from cyclonedds.idl import IdlStruct

logger = setup_logger()


@dataclass(frozen=True)
class Topic:
    """Represents a DDS topic."""

    name: str
    typename: type[IdlStruct]

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
        self.callbacks = callbacks_dict[topic]

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
        self._callback_lock = threading.Lock()
        self._writers: dict[Topic, DDSDataWriter] = {}
        self._writer_lock = threading.Lock()
        self._readers: dict[Topic, DDSDataReader] = {}
        self._reader_lock = threading.Lock()

    def _get_writer(self, topic: Topic) -> DDSDataWriter:
        """Get or create a DataWriter for the given topic."""
        with self._writer_lock:
            if topic not in self._writers:
                dds_topic = DDSTopic(self.get_participant(), topic.name, topic.typename)
                self._writers[topic] = DDSDataWriter(self.get_participant(), dds_topic)
            return self._writers[topic]

    def publish(self, topic: Topic, message: Any) -> None:
        """Publish a message to a DDS topic."""
        writer = self._get_writer(topic)
        try:
            writer.write(message)
        except Exception as e:
            logger.error(f"Error publishing to topic {topic}: {e}")

    def _get_reader(self, topic: Topic) -> DDSDataReader:
        """Get or create a DataReader for the given topic with listener."""
        with self._reader_lock:
            if topic not in self._readers:
                dds_topic = DDSTopic(self.get_participant(), topic.name, topic.typename)
                listener = _DDSMessageListener(topic, self._callbacks)
                self._readers[topic] = DDSDataReader(
                    self.get_participant(), dds_topic, listener=listener
                )
            return self._readers[topic]

    def subscribe(self, topic: Topic, callback: MessageCallback) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""
        with self._callback_lock:
            if topic not in self._callbacks:
                self._callbacks[topic] = []
            self._callbacks[topic].append(callback)
        self._get_reader(topic)
        return lambda: self._unsubscribe_callback(topic, callback)

    def _unsubscribe_callback(self, topic: Topic, callback: MessageCallback) -> None:
        """Unsubscribe a callback from a topic."""
        with self._callback_lock:
            if topic in self._callbacks and callback in self._callbacks[topic]:
                self._callbacks[topic].remove(callback)


class DDS(DDSPubSubBase): ...


__all__ = [
    "DDS",
    "DDSPubSubBase",
    "MessageCallback",
    "Topic",
]
