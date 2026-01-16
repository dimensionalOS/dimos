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

from collections import defaultdict
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub

if TYPE_CHECKING:
    from collections.abc import Callable


@dataclass
class DDSTopic:
    """Represents a DDS topic with optional type information."""

    topic: str = ""
    dds_type: type[Any] | None = None

    def __str__(self) -> str:
        if self.dds_type is None:
            return self.topic
        return f"{self.topic}#{self.dds_type.__name__}"

    def __hash__(self) -> int:
        return hash((self.topic, self.dds_type))

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, DDSTopic):
            return self.topic == other.topic and self.dds_type == other.dds_type


class DDSPubSubBase(PubSub[DDSTopic, Any]):
    """Base DDS pub/sub implementation using in-memory transport.

    This provides a transport-agnostic DDS pub/sub system that can later be
    extended to use actual DDS implementations (e.g., cyclone-dds, rti-dds).
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._callbacks: dict[str, list[Callable[[Any, DDSTopic], None]]] = defaultdict(list)
        self._topics: dict[str, DDSTopic] = {}

    def publish(self, topic: DDSTopic, message: Any) -> None:
        """Publish a message to a DDS topic."""
        # Store topic for reference
        self._topics[topic.topic] = topic

        # Dispatch to all subscribers
        for cb in self._callbacks[topic.topic]:
            try:
                cb(message, topic)
            except Exception as e:
                # Log but continue processing other callbacks
                print(f"Error in callback for topic {topic.topic}: {e}")

    def subscribe(
        self, topic: DDSTopic, callback: Callable[[Any, DDSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a DDS topic with a callback."""
        # Store topic for reference
        self._topics[topic.topic] = topic

        # Add callback to our list
        self._callbacks[topic.topic].append(callback)

        # Return unsubscribe function
        def unsubscribe() -> None:
            self.unsubscribe_callback(topic, callback)

        return unsubscribe

    def unsubscribe_callback(
        self, topic: DDSTopic, callback: Callable[[Any, DDSTopic], None]
    ) -> None:
        """Unsubscribe a callback from a topic."""
        try:
            self._callbacks[topic.topic].remove(callback)
            if not self._callbacks[topic.topic]:
                del self._callbacks[topic.topic]
        except (KeyError, ValueError):
            pass


class PickleDDS(PickleEncoderMixin[DDSTopic, Any], DDSPubSubBase): ...


__all__ = [
    "DDSPubSubBase",
    "DDSTopic",
    "PickleDDS",
]
