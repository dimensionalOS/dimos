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

"""Shared WebRTC session for Unitree robots.

Owns the asyncio event loop, background thread, and LegionConnection
lifecycle for one robot. Used by composition from the per-robot Module
classes (Go2WebRtcConnection, G1WebRtcConnection) and from the fleet-member
helper.
"""

from __future__ import annotations

import asyncio
from threading import Event, Thread, Timer
import time
from typing import Any

from unitree_webrtc_connect.constants import RTC_TOPIC
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection as LegionConnection,
    WebRTCConnectionMethod,
)

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import callback_to_observable

logger = setup_logger()


class UnitreeWebRtcSession:
    """Asyncio loop + LegionConnection lifecycle for one Unitree robot.

    Construction is cheap (no network). `start()` runs the WebRTC handshake
    and blocks until ready. `stop()` disconnects cleanly and joins the
    background thread.

    The underlying `loop` and `conn` are exposed for callers that need
    direct asyncio/WebRTC access (e.g., video track subscription).
    """

    def __init__(self, ip: str, *, mode_name: str = "ai", cmd_vel_timeout: float = 0.2) -> None:
        assert ip, "IP address must be provided"
        self.ip = ip
        self.mode_name = mode_name
        self.cmd_vel_timeout = cmd_vel_timeout

        self.loop = asyncio.new_event_loop()
        self.conn = LegionConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)

        self._task: asyncio.Task[None] | None = None
        self._thread: Thread | None = None
        self._connection_ready = Event()
        self._stop_timer: Timer | None = None

    def start(self) -> None:
        async def async_connect() -> None:
            await self.conn.connect()
            await self.conn.datachannel.disableTrafficSaving(True)
            self.conn.datachannel.set_decoder(decoder_type="native")
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": self.mode_name}},
            )
            self._connection_ready.set()
            while True:
                await asyncio.sleep(1)

        def start_background_loop() -> None:
            asyncio.set_event_loop(self.loop)
            self._task = self.loop.create_task(async_connect())
            self.loop.run_forever()

        self._thread = Thread(target=start_background_loop, daemon=True)
        self._thread.start()
        self._connection_ready.wait()

    def stop(self) -> None:
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

        if self._task:
            self._task.cancel()

        async def async_disconnect() -> None:
            try:
                self.conn.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data={"lx": 0, "ly": 0, "rx": 0, "ry": 0},
                )
                await self.conn.disconnect()
            except Exception:
                pass

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)
            self.loop.call_soon_threadsafe(self.loop.stop)

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def _stop_movement(self) -> None:
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a Twist as a WIRELESS_CONTROLLER command, auto-stopping after cmd_vel_timeout."""
        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z

        # WebRTC coordinate mapping:
        # x - positive right, negative left
        # y - positive forward, negative backwards
        # yaw - positive rotate right, negative rotate left
        async def async_move() -> None:
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": -y, "ly": x, "rx": -yaw, "ry": 0},
            )

        async def async_move_duration() -> None:
            start_time = time.time()
            while time.time() - start_time < duration:
                await async_move()
                await asyncio.sleep(0.01)

        if self._stop_timer:
            self._stop_timer.cancel()

        self._stop_timer = Timer(self.cmd_vel_timeout, self._stop_movement)
        self._stop_timer.daemon = True
        self._stop_timer.start()

        try:
            if duration > 0:
                future = asyncio.run_coroutine_threadsafe(async_move_duration(), self.loop)
                future.result()
                self._stop_movement()
            else:
                future = asyncio.run_coroutine_threadsafe(async_move(), self.loop)
                future.result()
            return True
        except Exception as e:
            logger.error(f"Failed to send movement command: {e}")
            return False

    def publish_request(self, topic: str, data: dict[Any, Any]) -> Any:
        """Synchronous wrapper around publish_request_new running on the session loop."""
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(topic, data), self.loop
        )
        return future.result()

    def sub_stream(self, topic_name: str):  # type: ignore[no-untyped-def]
        """Convert a Unitree pub/sub topic into an observable stream."""

        def subscribe_in_thread(cb) -> None:  # type: ignore[no-untyped-def]
            def run_subscription() -> None:
                self.conn.datachannel.pub_sub.subscribe(topic_name, cb)

            self.loop.call_soon_threadsafe(run_subscription)

        def unsubscribe_in_thread(cb) -> None:  # type: ignore[no-untyped-def]
            def run_unsubscription() -> None:
                self.conn.datachannel.pub_sub.unsubscribe(topic_name)

            self.loop.call_soon_threadsafe(run_unsubscription)

        return callback_to_observable(
            start=subscribe_in_thread,
            stop=unsubscribe_in_thread,
        )
