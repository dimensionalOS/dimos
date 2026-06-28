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

"""Generic Unitree WebRTC connection.

This file handles the *connection only*: the datachannel + asyncio loop, the
request/response RPC (``publish_request``), fire-and-forget sends (``publish``), and
topic subscriptions (``subscribe``). It knows nothing about how any particular robot
moves or changes mode.

Each robot's own ``connection.py`` subclasses this and owns ALL of its movement and
mode commands (and sensor decoding), since the FSMs differ per robot:
  - Go2: dimos/robot/unitree/go2/connection.py  (Go2WebRTCConnection)
  - G1:  dimos/robot/unitree/g1/connection.py   (G1WebRTCConnection)
"""

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
    """Generic Unitree WebRTC transport, shared by all Unitree WebRTC robots.

    Connection only — no movement or mode commands. Subclasses (Go2WebRTCConnection,
    G1WebRTCConnection) build their robot-specific commands on the primitives here:
    ``publish_request`` (request/response), ``publish`` (fire-and-forget) and
    ``subscribe`` (topic → Observable). They may also use ``self.conn`` / ``self.loop``
    directly for tight send loops.
    """

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
        """Tear down the connection. Stopping the robot's motion is the robot
        subclass / module's responsibility, not the transport's."""

        async def async_disconnect() -> None:
            try:
                await self.conn.disconnect()
            except Exception:
                pass

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)
            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.thread.is_alive():
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def publish(self, topic: str, data: dict[Any, Any], msg_type: str | None = None) -> None:
        """Fire-and-forget send on the datachannel, marshalled onto the loop thread.

        Robot subclasses use this for low-level wire sends (e.g. joystick / SPORT Move).
        """

        async def _send() -> None:
            if msg_type is None:
                self.conn.datachannel.pub_sub.publish_without_callback(topic, data=data)
            else:
                self.conn.datachannel.pub_sub.publish_without_callback(
                    topic, data=data, msg_type=msg_type
                )

        asyncio.run_coroutine_threadsafe(_send(), self.loop).result()

    def publish_request(self, topic: str, data: dict[Any, Any]) -> Any:
        """Request/response RPC over the datachannel (blocks for the reply)."""
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(topic, data), self.loop
        )
        return future.result()

    def subscribe(self, topic_name: str):  # type: ignore[no-untyped-def]
        """Subscribe to a datachannel topic, returning an Observable of raw messages.

        Robot subclasses pipe this through their own (robot-specific) decoders to build
        typed sensor streams.
        """

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

    def disconnect(self) -> None:
        """Disconnect from the robot and clean up resources."""
        if hasattr(self, "conn"):

            async def async_disconnect() -> None:
                try:
                    await self.conn.disconnect()
                except:
                    pass

            if hasattr(self, "loop") and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)

        if hasattr(self, "loop") and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
