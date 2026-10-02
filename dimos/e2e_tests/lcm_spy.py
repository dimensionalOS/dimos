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

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import math
import pickle
import re
import threading
from typing import Any

from dimos.core.global_config import global_config
from dimos.core.transport_factory import transport_topic
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.protocol import DimosMsg
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, ZenohPubSubBase
from dimos.utils.testing.waiting import wait_until


def _wire_topic(channel: str) -> ZenohTopic:
    """The active backend's topic for an LCM-style channel ('/odom#pkg.Msg').

    ZenohTopic is an LCM Topic plus zenoh's key expression, so both buses take it.
    """
    parsed = Topic.from_channel_str(transport_topic(channel))
    return ZenohTopic(parsed.topic, parsed.lcm_type)


class LcmSpy:
    """Sniffs every message on the active transport's bus.

    Topics are named the LCM way (`/odom#geometry_msgs.PoseStamped`);
    `transport_topic` maps them to the running backend's wire name.
    """

    messages: dict[str, list[bytes]]
    _bus: LCMPubSubBase | ZenohPubSubBase
    _everything: ZenohTopic
    _unsubscribe: Callable[[], None]
    _messages_lock: threading.Lock
    _saved_topics: dict[str, str]
    _saved_topics_lock: threading.Lock
    _topic_listeners: dict[str, list[Callable[[bytes], None]]]
    _topic_listeners_lock: threading.Lock

    def __init__(self) -> None:
        zenoh = global_config.transport == "zenoh"
        self._bus = ZenohPubSubBase() if zenoh else LCMPubSubBase()
        self._everything = ZenohTopic("dimos/**") if zenoh else ZenohTopic(re.compile(".*"))
        self.messages = {}
        self._messages_lock = threading.Lock()
        self._saved_topics = {}
        self._saved_topics_lock = threading.Lock()
        self._topic_listeners = {}
        self._topic_listeners_lock = threading.Lock()

    def start(self) -> None:
        self._bus.start()
        self._unsubscribe = self._bus.subscribe(self._everything, self._on_message)

    def stop(self) -> None:
        self._unsubscribe()
        self._bus.stop()

    def _on_message(self, data: bytes, topic: Topic) -> None:
        wire = str(topic)
        with self._saved_topics_lock:
            channel = self._saved_topics.get(wire)

        if channel is not None:
            with self._messages_lock:
                self.messages.setdefault(channel, []).append(data)

        with self._topic_listeners_lock:
            listeners = self._topic_listeners.get(wire)
            if listeners:
                for listener in listeners:
                    listener(data)

    def publish(self, topic: str, msg: Any) -> None:
        self._bus.publish(_wire_topic(topic), msg.lcm_encode())

    def save_topic(self, topic: str) -> None:
        with self._saved_topics_lock:
            self._saved_topics[transport_topic(topic)] = topic

    def register_topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> None:
        with self._topic_listeners_lock:
            self._topic_listeners.setdefault(transport_topic(topic), []).append(listener)

    def unregister_topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> None:
        with self._topic_listeners_lock:
            self._topic_listeners[transport_topic(topic)].remove(listener)

    @contextmanager
    def topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> Iterator[None]:
        self.register_topic_listener(topic, listener)
        try:
            yield
        finally:
            self.unregister_topic_listener(topic, listener)

    def wait_for_saved_topic(self, topic: str, timeout: float = 30.0) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return topic in self.messages

        wait_until(
            condition,
            timeout=timeout,
            message=f"Timeout waiting for topic {topic}",
        )

    def wait_for_saved_topic_content(
        self, topic: str, content_contains: bytes, timeout: float = 30.0
    ) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return any(content_contains in msg for msg in self.messages.get(topic, []))

        wait_until(
            condition,
            timeout=timeout,
            message=f"Timeout waiting for '{topic}' to contain '{content_contains!r}'",
        )

    def wait_for_message_pickle_result(
        self,
        topic: str,
        predicate: Callable[[Any], bool],
        fail_message: str,
        timeout: float = 30.0,
    ) -> None:
        event = threading.Event()

        def listener(msg: bytes) -> None:
            data = pickle.loads(msg)
            if predicate(data["res"]):
                event.set()

        with self.topic_listener(topic, listener):
            wait_until(
                event.is_set,
                timeout=timeout,
                message=fail_message,
            )

    def wait_for_message_result(
        self,
        topic: str,
        type: type[DimosMsg],
        predicate: Callable[[Any], bool],
        fail_message: str,
        timeout: float = 30.0,
    ) -> None:
        event = threading.Event()

        def listener(msg: bytes) -> None:
            data = type.lcm_decode(msg)
            if predicate(data):
                event.set()

        with self.topic_listener(topic, listener):
            wait_until(
                event.is_set,
                timeout=timeout,
                message=fail_message,
            )

    def wait_until_odom_position(
        self, x: float, y: float, threshold: float = 1, timeout: float = 60
    ) -> None:
        def predicate(msg: PoseStamped) -> bool:
            pos = msg.position
            distance = math.sqrt((pos.x - x) ** 2 + (pos.y - y) ** 2)
            return distance < threshold

        self.wait_for_message_result(
            "/odom#geometry_msgs.PoseStamped",
            PoseStamped,
            predicate,
            f"Failed to get to position x={x}, y={y}",
            timeout,
        )
