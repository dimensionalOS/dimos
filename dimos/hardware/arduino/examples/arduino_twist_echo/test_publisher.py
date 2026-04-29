# Copyright 2026 Dimensional Inc.
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

"""TestPublisher: emits a Twist every 500ms, listens for echoes."""

from __future__ import annotations

import threading
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TestPublisherConfig(ModuleConfig):
    publish_period_s: float = 0.5


class TestPublisher(Module):
    """Publishes a Twist on ``cmd_out`` every publish_period_s, prints any echo
    received on ``echo_in``."""

    config: TestPublisherConfig

    cmd_out: Out[Twist]
    echo_in: In[Twist]

    _thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None
    _counter: int = 0
    _echo_count: int = 0
    # `threading.Lock` is a factory function, not a class, so
    # `threading.Lock | None` fails at runtime when pydantic evaluates
    # type hints via `get_type_hints`.  Use `Any` so the annotation is
    # accepted but still carries the type intent in the comment.
    _echo_lock: Any = None  # actually threading.Lock | None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event = threading.Event()
        self._echo_lock = threading.Lock()
        self.echo_in.subscribe(self._on_echo)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info("TestPublisher started")

    @rpc
    def echo_count(self) -> int:
        """Return the number of echoes received since start().

        Exposed as an RPC so e2e tests running in the host process can
        observe progress of the QEMU-backed round-trip without having to
        subscribe to LCM directly.
        """
        if self._echo_lock is None:
            return 0
        with self._echo_lock:
            return self._echo_count

    @rpc
    def stop(self) -> None:
        # Signal the loop via Event so it wakes from the period sleep
        # immediately instead of waiting up to `publish_period_s`.
        if self._stop_event is not None:
            self._stop_event.set()
        if self._thread is not None:
            # Give the loop one full period plus a small grace window to
            # exit cleanly.
            join_timeout = max(2.0, self.config.publish_period_s + 0.5)
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                logger.warning("TestPublisher thread did not exit in time")
        super().stop()

    def _publish_loop(self) -> None:
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            self._counter += 1
            twist = Twist(
                linear=Vector3(self._counter * 0.1, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, self._counter * 0.05),
            )
            self.cmd_out.publish(twist)
            logger.info(
                "Published twist",
                n=self._counter,
                linear_x=twist.linear.x,
                angular_z=twist.angular.z,
            )
            # `Event.wait(timeout=...)` returns True when the event is set,
            # so stop() wakes the loop immediately instead of waiting for
            # the full period.
            self._stop_event.wait(timeout=self.config.publish_period_s)

    def _on_echo(self, msg: Twist) -> None:
        if self._echo_lock is not None:
            with self._echo_lock:
                self._echo_count += 1
        logger.info(
            "Received echo",
            linear_x=msg.linear.x,
            linear_y=msg.linear.y,
            linear_z=msg.linear.z,
            angular_z=msg.angular.z,
        )
