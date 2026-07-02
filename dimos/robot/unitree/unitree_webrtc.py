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

"""Generic Unitree WebRTC connection (connection only; robot commands live in
per-robot ``*_webrtc.py`` subclasses)."""

import asyncio
import threading
from typing import Any

from unitree_webrtc_connect.constants import RTC_TOPIC
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection as LegionConnection,
    WebRTCConnectionMethod,
)

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.resource import Resource
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import callback_to_observable

logger = setup_logger()


class UnitreeWebRTCConnection(Resource):
    """Generic Unitree WebRTC transport (connection only); commands live in subclasses."""

    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        aes_128_key: str | None = None,
    ) -> None:
        self.ip = ip
        self.mode = mode
        # Per-device AES-128 key for new Unitree firmware (data2=3 handshake); omitted when unset.
        self.conn = LegionConnection(
            WebRTCConnectionMethod.LocalSTA, ip=self.ip, aes_128_key=aes_128_key
        )
        # self.loop / self.thread are created in connect() (called below).
        self.connect()

    def connect(self) -> None:
        self.loop = asyncio.new_event_loop()

        async def async_connect() -> None:
            await self.conn.connect()
            await self.conn.datachannel.disableTrafficSaving(True)

            self.conn.datachannel.set_decoder(decoder_type="native")

            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": self.mode}}
            )

        def start_background_loop() -> None:
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()

        self.thread = threading.Thread(target=start_background_loop, daemon=True)
        self.thread.start()

        # Blocks until connected; re-raises connect failures (e.g. missing AES key).
        try:
            asyncio.run_coroutine_threadsafe(async_connect(), self.loop).result()
        except Exception:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            raise

    def start(self) -> None:
        pass

    def stop(self) -> None:
        """Tear down the connection (loop, datachannel, thread). Idempotent."""

        async def async_disconnect() -> None:
            try:
                await self.conn.disconnect()
            except Exception:
                pass

        if self.loop.is_running():
            # Let the disconnect actually run (bounded) before killing the loop.
            future = asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)
            try:
                future.result(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            except Exception:
                pass
            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.thread.is_alive():
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def publish(self, topic: str, data: dict[Any, Any], msg_type: str | None = None) -> None:
        """Fire-and-forget datachannel send; safe from any thread (marshalled onto the loop)."""

        def _send() -> None:
            if msg_type is None:
                self.conn.datachannel.pub_sub.publish_without_callback(topic, data=data)
            else:
                self.conn.datachannel.pub_sub.publish_without_callback(
                    topic, data=data, msg_type=msg_type
                )

        try:
            on_loop = asyncio.get_running_loop() is self.loop
        except RuntimeError:
            on_loop = False

        if on_loop:
            _send()  # already on the loop thread (e.g. a call_later callback)
        else:

            async def _acoro() -> None:
                _send()

            asyncio.run_coroutine_threadsafe(_acoro(), self.loop).result()

    def publish_request(self, topic: str, data: dict[Any, Any]) -> Any:
        """Request/response RPC over the datachannel (blocks for the reply)."""
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(topic, data), self.loop
        )
        return future.result()

    def subscribe(self, topic_name: str):  # type: ignore[no-untyped-def]
        """Subscribe to a datachannel topic → Observable of raw messages."""

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
