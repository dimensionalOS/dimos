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

from __future__ import annotations

import os
import threading
import traceback
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, runtime_checkable

import lcm

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.protocol.service.spec import Service


@dataclass
class LCMConfig:
    ttl: int = 0
    url: str | None = None
    # auto configure routing
    auto_configure_multicast: bool = True
    auto_configure_buffers: bool = False


@runtime_checkable
class LCMMsg(Protocol):
    name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> "LCMMsg":
        """Decode bytes into an LCM message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    topic: str = ""
    lcm_type: Optional[type[LCMMsg]] = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.name}"


class LCMbase(PubSub[Topic, Any], Service[LCMConfig]):
    default_config = LCMConfig
    lc: lcm.LCM
    _stop_event: threading.Event
    _thread: Optional[threading.Thread]
    _callbacks: dict[str, list[Callable[[Any], None]]]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.lc = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()
        self._stop_event = threading.Event()
        self._thread = None
        self._callbacks = {}

    def publish(self, topic: Topic, message: bytes):
        """Publish a message to the specified channel."""
        self.lc.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        lcm_subscription = self.lc.subscribe(str(topic), lambda _, msg: callback(msg, topic))

        def unsubscribe():
            self.lc.unsubscribe(lcm_subscription)

        return unsubscribe

    def start(self):
        # TODO: proper error handling/log messages for these system calls
        if self.config.auto_configure_multicast:
            try:
                os.system("sudo ifconfig lo multicast")
                os.system("sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo")
            except Exception as e:
                print(f"Error configuring multicast: {e}")

        if self.config.auto_configure_buffers:
            try:
                os.system("sudo sysctl -w net.core.rmem_max=2097152")
                os.system("sudo sysctl -w net.core.rmem_default=2097152")
            except Exception as e:
                print(f"Error configuring buffers: {e}")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()

    def _loop(self) -> None:
        """LCM message handling loop."""
        while not self._stop_event.is_set():
            try:
                # Use timeout to allow periodic checking of stop_event
                self.lc.handle_timeout(100)  # 100ms timeout
            except Exception as e:
                stack_trace = traceback.format_exc()
                print(f"Error in LCM handling: {e}\n{stack_trace}")
                if self._stop_event.is_set():
                    break

    def stop(self):
        """Stop the LCM loop."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()


class LCMEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(self, msg: LCMMsg, _: Topic) -> bytes:
        return msg.lcm_encode()

    def decode(self, msg: bytes, topic: Topic) -> LCMMsg:
        if topic.lcm_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no lcm_type specified"
            )
        return topic.lcm_type.lcm_decode(msg)


class LCM(
    LCMEncoderMixin,
    LCMbase,
): ...


class pickleLCM(
    PickleEncoderMixin,
    LCMbase,
): ...
