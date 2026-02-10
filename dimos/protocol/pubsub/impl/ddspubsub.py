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
from cyclonedds.qos import Policy, Qos
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
    data_type: type[IdlStruct]

    def __str__(self) -> str:
        return f"{self.name}#{self.data_type.__name__}"


MessageCallback: TypeAlias = Callable[[Any, Topic], None]


class _DDSMessageListener(Listener):  # type: ignore[misc]
    """Listener for DataReader that dispatches messages to callbacks."""

    __slots__ = ("_callbacks", "_lock", "_topic")

    def __init__(self, topic: Topic) -> None:
        super().__init__()  # type: ignore[no-untyped-call]
        self._topic = topic
        self._callbacks: tuple[MessageCallback, ...] = ()
        self._lock = threading.Lock()

    def add_callback(self, callback: MessageCallback) -> None:
        """Add a callback to the listener."""
        with self._lock:
            self._callbacks = (*self._callbacks, callback)

    def remove_callback(self, callback: MessageCallback) -> None:
        """Remove a callback from the listener."""
        with self._lock:
            self._callbacks = tuple(cb for cb in self._callbacks if cb is not callback)

    def on_data_available(self, reader: DDSDataReader[Any]) -> None:
        """Called when data is available on the reader."""
        try:
            samples = reader.take()
        except Exception as e:
            logger.error(f"Error reading from topic {self._topic}: {e}", exc_info=True)
            return
        for sample in samples:
            if sample is not None:
                for callback in self._callbacks:
                    try:
                        callback(sample, self._topic)
                    except Exception as e:
                        logger.error(f"Callback error on topic {self._topic}: {e}", exc_info=True)


class DDS(DDSService, PubSub[Topic, Any]):
    def __init__(self, qos: Qos | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._qos = qos
        self._writers: dict[Topic, DDSDataWriter[Any]] = {}
        self._writer_lock = threading.Lock()
        self._readers: dict[Topic, DDSDataReader[Any]] = {}
        self._reader_lock = threading.Lock()
        self._listeners: dict[Topic, _DDSMessageListener] = {}

    @property
    def qos(self) -> Qos | None:
        """Get the QoS settings."""
        return self._qos

    def _get_writer(self, topic: Topic) -> DDSDataWriter[Any]:
        """Get or create a DataWriter for the given topic."""
        with self._writer_lock:
            if topic not in self._writers:
                dds_topic = DDSTopic(self.participant, topic.name, topic.data_type)
                self._writers[topic] = DDSDataWriter(self.participant, dds_topic, qos=self._qos)
            return self._writers[topic]

    def publish(self, topic: Topic, message: Any) -> None:
        """Publish a message to a DDS topic."""
        writer = self._get_writer(topic)
        try:
            writer.write(message)
        except Exception as e:
            logger.error(f"Error publishing to topic {topic}: {e}", exc_info=True)

    def _get_listener(self, topic: Topic) -> _DDSMessageListener:
        """Get or create a listener and reader for the given topic."""
        with self._reader_lock:
            if topic not in self._readers:
                dds_topic = DDSTopic(self.participant, topic.name, topic.data_type)
                listener = _DDSMessageListener(topic)
                self._readers[topic] = DDSDataReader(
                    self.participant, dds_topic, qos=self._qos, listener=listener
                )
                self._listeners[topic] = listener
            return self._listeners[topic]

    def subscribe(self, topic: Topic, callback: MessageCallback) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""
        listener = self._get_listener(topic)
        listener.add_callback(callback)
        return lambda: self._unsubscribe_callback(topic, callback)

    def _unsubscribe_callback(self, topic: Topic, callback: MessageCallback) -> None:
        """Unsubscribe a callback from a topic."""
        with self._reader_lock:
            listener = self._listeners.get(topic)
        if listener:
            listener.remove_callback(callback)

    def stop(self) -> None:
        """Stop the DDS service and clean up resources."""
        with self._reader_lock:
            self._readers.clear()
            self._listeners.clear()
        with self._writer_lock:
            self._writers.clear()
        super().stop()


__all__ = [
    "DDS",
    "MessageCallback",
    "Policy",
    "Qos",
    "Topic",
]
