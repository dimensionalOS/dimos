# Copyright 2025 Dimensional Inc.
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
import threading
import time
from typing import Any

import lcm

from dimos.msgs.geometry_msgs import PoseStamped
from dimos.protocol.service.lcmservice import LCMMsg, LCMService


class LcmSpy(LCMService):
    l: lcm.LCM
    messages: dict[str, list[bytes]]
    _messages_lock: threading.Lock
    _saved_topics: set[str]
    _saved_topics_lock: threading.Lock
    _topic_listeners: dict[str, list[Callable[[bytes], None]]]
    _topic_listeners_lock: threading.Lock

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.l = lcm.LCM()
        self.messages = {}
        self._messages_lock = threading.Lock()
        self._saved_topics = set()
        self._saved_topics_lock = threading.Lock()
        self._topic_listeners = {}
        self._topic_listeners_lock = threading.Lock()

    def start(self) -> None:
        super().start()
        if self.l:
            self.l.subscribe(".*", self.msg)

    def stop(self) -> None:
        super().stop()

    def msg(self, topic: str, data: bytes) -> None:
        with self._saved_topics_lock:
            if topic in self._saved_topics:
                with self._messages_lock:
                    self.messages.setdefault(topic, []).append(data)

        with self._topic_listeners_lock:
            listeners = self._topic_listeners.get(topic)
            if listeners:
                for listener in listeners:
                    listener(data)

    def publish(self, topic: str, msg: Any) -> None:
        self.l.publish(topic, msg.lcm_encode())

    def save_topic(self, topic: str) -> None:
        with self._saved_topics_lock:
            self._saved_topics.add(topic)

    def register_topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> int:
        with self._topic_listeners_lock:
            listeners = self._topic_listeners.setdefault(topic, [])
            listener_index = len(listeners)
            listeners.append(listener)
            return listener_index

    def unregister_topic_listener(self, topic: str, listener_index: int) -> None:
        with self._topic_listeners_lock:
            listeners = self._topic_listeners[topic]
            listeners.pop(listener_index)

    @contextmanager
    def topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> Iterator[None]:
        listener_index = self.register_topic_listener(topic, listener)
        try:
            yield
        finally:
            self.unregister_topic_listener(topic, listener_index)

    def wait_until(
        self,
        *,
        condition: Callable[[], bool],
        timeout: float,
        error_message: str,
        poll_interval: float = 0.1,
    ) -> None:
        start_time = time.time()
        while time.time() - start_time < timeout:
            if condition():
                return
            time.sleep(poll_interval)
        raise TimeoutError(error_message)

    def wait_for_saved_topic(self, topic: str, timeout: float = 30.0) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return topic in self.messages

        self.wait_until(
            condition=condition,
            timeout=timeout,
            error_message=f"Timeout waiting for topic {topic}",
        )

    def wait_for_saved_topic_content(
        self, topic: str, content_contains: bytes, timeout: float = 30.0
    ) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return any(content_contains in msg for msg in self.messages.get(topic, []))

        self.wait_until(
            condition=condition,
            timeout=timeout,
            error_message=f"Timeout waiting for '{topic}' to contain '{content_contains!r}'",
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
            self.wait_until(
                condition=event.is_set,
                timeout=timeout,
                error_message=fail_message,
            )

    def wait_for_message_result(
        self,
        topic: str,
        type: type[LCMMsg],
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
            self.wait_until(
                condition=event.is_set,
                timeout=timeout,
                error_message=fail_message,
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
