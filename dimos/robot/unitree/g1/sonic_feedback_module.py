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

"""Read-only telemetry from an independently operated C++ SONIC controller."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import suppress
from dataclasses import asdict
import threading
import time
from typing import Any

from pydantic import Field
import zmq
import zmq.asyncio

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.sonic_feedback import (
    MAX_PACKET_BYTES,
    TOPIC,
    FeedbackBuffer,
    SonicFeedback,
    decode_feedback,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SonicFeedbackConfig(ModuleConfig):
    # Required: choose the telemetry endpoint explicitly, including its port.
    endpoint: str = Field(min_length=1)
    stale_after: float = Field(default=0.2, gt=0, allow_inf_nan=False)


class SonicFeedbackModule(Module):
    """Subscribe to g1_debug; expose measurements without owning robot control.

    No command socket, DDS participant, motion RPC, controller launch or stop.
    Streams emit only newly accepted samples. Use get_feedback()/get_status()
    for current freshness; a stream consumer must not cache a sample forever.
    After the controller resets its index, create a new module session.
    """

    config: SonicFeedbackConfig
    feedback: Out[SonicFeedback]
    body_joint_state: Out[JointState]
    left_hand_joint_state: Out[JointState]
    right_hand_joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._running = False
        self._error: str | None = None
        self._invalid_packets = 0
        super().__init__(**kwargs)
        self._buffer = FeedbackBuffer(stale_after=self.config.stale_after)

    @rpc
    def start(self) -> None:
        with self._lock:
            if self._running:
                return
        try:
            super().start()
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            raise

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._lock:
                self._running = False
                self._error = None

    async def main(self) -> AsyncIterator[None]:
        with self._lock:
            if self._running:
                raise RuntimeError("SONIC feedback module is already running")
        # Socket creation, receive and close all occur on the module event loop.
        with zmq.asyncio.Context() as context, context.socket(zmq.SUB) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.CONFLATE, 1)
            socket.setsockopt(zmq.MAXMSGSIZE, MAX_PACKET_BYTES)
            socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
            socket.connect(self.config.endpoint)
            with self._lock:
                self._buffer = FeedbackBuffer(stale_after=self.config.stale_after)
                self._invalid_packets = 0
                self._error = None
                self._running = True
            task = asyncio.create_task(self._receive(socket))
            try:
                yield
            finally:
                with self._lock:
                    self._running = False
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task

    async def _receive(self, socket: zmq.asyncio.Socket) -> None:
        try:
            while True:
                parts = await socket.recv_multipart()
                try:
                    if len(parts) != 1:
                        raise ValueError("g1_debug requires a single-part message")
                    state = decode_feedback(
                        parts[0], received_at=time.time(), received_monotonic=time.monotonic()
                    )
                except ValueError:
                    with self._lock:
                        self._invalid_packets += 1
                    continue
                with self._lock:
                    accepted = self._buffer.accept(state)
                if accepted:
                    body, left, right = state.joint_states()
                    self.body_joint_state.publish(body)
                    self.left_hand_joint_state.publish(left)
                    self.right_hand_joint_state.publish(right)
                    self.feedback.publish(state)
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            logger.exception("SONIC feedback receiver failed")

    @rpc
    def get_feedback(self) -> SonicFeedback | None:
        """Return fresh measurements, or None when waiting, stale, stopped or faulted."""
        with self._lock:
            if not self._running or self._error is not None:
                return None
            return self._buffer.latest(now=time.monotonic())

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Report stream freshness and optional controller-reported reference status.

        reference_guard is None unless fresh feedback explicitly includes it.
        A confirmed planner handover is not a task completion/settling result.
        """
        with self._lock:
            now = time.monotonic()
            age = self._buffer.age(now=now)
            sample = self._buffer.latest(now=now)
            if self._error is not None:
                state = "fault"
            elif not self._running:
                state = "stopped"
            elif age is None:
                state = "waiting"
            elif sample is None:
                state = "stale"
            else:
                state = "fresh"
            return {
                "state": state,
                "endpoint": self.config.endpoint,
                "age_s": age,
                "stale_after_s": self.config.stale_after,
                "accepted_packets": self._buffer.accepted,
                "invalid_packets": self._invalid_packets,
                "rejected_indices": self._buffer.rejected_indices,
                "error": self._error,
                "reference_guard": (
                    asdict(sample.reference_guard)
                    if state == "fresh"
                    and sample is not None
                    and sample.reference_guard is not None
                    else None
                ),
            }
