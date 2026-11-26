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

"""Bounded, asynchronous capture of the R1 Pro ROS control boundary."""

from __future__ import annotations

from collections.abc import Callable
from itertools import count
import json
import queue
import threading
import time
from typing import Any

from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _xyz(vector: Any) -> list[float]:
    return [vector.x, vector.y, vector.z]


def _record(direction: str, topic: str, msg: Any, mono: int, unix: int, seq: int) -> String:
    header = getattr(msg, "header", None)
    stamp = header.stamp if header is not None else None
    if hasattr(msg, "twist"):
        fields = {"linear": _xyz(msg.twist.linear), "angular": _xyz(msg.twist.angular)}
    elif hasattr(msg, "position"):
        fields = {
            key: list(getattr(msg, key)) for key in ("name", "position", "velocity", "effort")
        }
    elif hasattr(msg, "errors"):
        fields = {"errors": [[e.name, e.error_code, e.error_description] for e in msg.errors]}
    else:  # sensor_msgs/Imu
        fields = {
            "orientation": [*_xyz(msg.orientation), msg.orientation.w],
            "angular_velocity": _xyz(msg.angular_velocity),
            "linear_acceleration": _xyz(msg.linear_acceleration),
            **{
                key: list(getattr(msg, key))
                for key in (
                    "orientation_covariance",
                    "angular_velocity_covariance",
                    "linear_acceleration_covariance",
                )
            },
        }
    return String(
        json.dumps(
            {
                "schema": 1,
                "direction": direction,
                "topic": topic,
                "seq": seq,
                "monotonic_ns": mono,
                "unix_ns": unix,
                "header_ns": None if stamp is None else stamp.sec * 10**9 + stamp.nanosec,
                "frame_id": None if header is None else header.frame_id,
                "values": fields,
            },
            separators=(",", ":"),
        )
    )


class DiagnosticTap:
    def __init__(self, publish: Callable[[String], None]) -> None:
        self._publish = publish
        self._pending: queue.Queue[tuple[str, str, Any, int, int, int]] = queue.Queue(1024)
        self._sequence = count()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="r1pro-diagnostic-tap", daemon=True)
        self._thread.start()

    def offer(self, direction: str, topic: str, msg: Any, mono: int, unix: int) -> None:
        # ROS gives these callbacks their own messages, and neither the bridge
        # nor its control publishers mutate the objects after this boundary.
        try:
            self._pending.put_nowait((direction, topic, msg, mono, unix, next(self._sequence)))
        except queue.Full:
            pass  # A sequence gap, never backpressure on robot I/O.

    def wrap_rx(self, callback: Callable[[Any, Any], None]) -> Callable[[Any, Any], None]:
        def traced(msg: Any, topic: Any) -> None:
            self.offer("rx", topic.topic, msg, time.monotonic_ns(), time.time_ns())
            callback(msg, topic)

        return traced

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while True:
            try:
                event = self._pending.get(timeout=0.05)
            except queue.Empty:
                if self._stop.is_set():
                    return
                continue
            try:
                self._publish(_record(*event))
            except Exception:
                logger.exception("R1 Pro diagnostic event dropped")
