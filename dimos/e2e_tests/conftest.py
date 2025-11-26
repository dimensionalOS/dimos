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

from collections.abc import Callable
from contextlib import contextmanager
import math
import pickle
import signal
import subprocess
import threading
import time
from typing import Any

import lcm
import pytest

from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.protocol.service.lcmservice import LCMService


def _pose(x: float, y: float, theta: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0),
        orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
        frame_id="map",
    )


class LCMSpy(LCMService):
    l: lcm.LCM
    _messages: dict[str, list[bytes]]
    _messages_lock: threading.Lock
    _saved_topics: set[str]
    _saved_topics_lock: threading.Lock
    _topic_listeners: dict[str, list[Callable[[bytes], None]]]
    _topic_listeners_lock: threading.Lock

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.l = lcm.LCM()
        self._messages = {}
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
            if topic not in self._saved_topics:
                return
            with self._messages_lock:
                self._messages.setdefault(topic, []).append(data)

        with self._topic_listeners_lock:
            listeners = self._topic_listeners.get(topic)
            if not listeners:
                return
            for listener in listeners:
                listener(data)

    def publish(self, topic: str, msg) -> None:
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
    def topic_listener(self, topic: str, listener: Callable[[bytes], None]):
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
                return topic in self._messages

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
                return any(content_contains in msg for msg in self._messages.get(topic, []))

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

    def wait_until_odom_position(
        self, x: float, y: float, threshold: float = 1, timeout: float = 60
    ) -> None:
        event = threading.Event()

        def listener(msg: bytes) -> None:
            pos = PoseStamped.lcm_decode(msg).position
            distance = math.sqrt((pos.x - x) ** 2 + (pos.y - y) ** 2)
            print("=" * 100)
            print("distance", distance)
            if distance < threshold:
                event.set()

        with self.topic_listener("/odom#geometry_msgs.PoseStamped", listener):
            self.wait_until(
                condition=event.is_set,
                timeout=timeout,
                error_message=f"Failed to get to position x={x}, y={y}",
            )


class DimosCliCall:
    process: subprocess.Popen | None
    demo_name: str | None = None

    def __init__(self) -> None:
        self.process = None

    def start(self) -> None:
        if self.demo_name is None:
            raise ValueError("Demo name must be set before starting the process.")

        self.process = subprocess.Popen(
            ["dimos", "--simulation", "run", self.demo_name],
        )

    def stop(self) -> None:
        if self.process is None:
            return

        try:
            # Send the kill signal (SIGTERM for graceful shutdown)
            self.process.send_signal(signal.SIGTERM)

            # Record the time when we sent the kill signal
            shutdown_start = time.time()

            # Wait for the process to terminate with a 30-second timeout
            try:
                self.process.wait(timeout=30)
                shutdown_duration = time.time() - shutdown_start

                # Verify it shut down in time
                assert shutdown_duration <= 30, (
                    f"Process took {shutdown_duration:.2f} seconds to shut down, "
                    f"which exceeds the 30-second limit"
                )
            except subprocess.TimeoutExpired:
                # If we reach here, the process didn't terminate in 30 seconds
                self.process.kill()  # Force kill
                self.process.wait()  # Clean up
                raise AssertionError(
                    "Process did not shut down within 30 seconds after receiving SIGTERM"
                )

        except Exception:
            # Clean up if something goes wrong
            if self.process.poll() is None:  # Process still running
                self.process.kill()
                self.process.wait()
            raise


@pytest.fixture
def lcm_spy():
    lcm_spy = LCMSpy()
    lcm_spy.start()
    yield lcm_spy
    lcm_spy.stop()


@pytest.fixture
def follow_points(lcm_spy):
    def fun(*, points, fail_message: str) -> None:
        for x, y, theta in points:
            lcm_spy.publish("/goal_request#geometry_msgs.PoseStamped", _pose(x, y, theta))
            lcm_spy.wait_for_message_pickle_result(
                "/rpc/HolonomicLocalPlanner/is_goal_reached/res",
                predicate=lambda v: bool(v),
                fail_message=fail_message,
                timeout=60.0,
            )

    return fun


@pytest.fixture
def start_blueprint():
    dimos_robot_call = DimosCliCall()

    def set_name_and_start(demo_name: str) -> DimosCliCall:
        dimos_robot_call.demo_name = demo_name
        dimos_robot_call.start()
        return dimos_robot_call

    yield set_name_and_start

    dimos_robot_call.stop()


@pytest.fixture
def human_input():
    transport = pLCMTransport("/human_input")
    transport.lcm.start()

    def send_human_input(message: str) -> None:
        transport.publish(message)

    yield send_human_input

    transport.lcm.stop()
