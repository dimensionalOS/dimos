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

import traceback
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

import dimos.core.colors as colors
from dimos.core.core import In, Transport
from dimos.protocol.pubsub.lcmpubsub import LCM, pickleLCM
from dimos.protocol.pubsub.lcmpubsub import Topic as LCMTopic

T = TypeVar("T")


class PubSubTransport(Transport[T]):
    topic: any

    def __init__(self, topic: any):
        self.topic = topic

    def __str__(self) -> str:
        return (
            colors.green(f"{self.__class__.__name__}(")
            + colors.blue(self.topic)
            + colors.green(")")
        )


class pLCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str):
        super().__init__(topic)
        self.lcm = pickleLCM()

    def __reduce__(self):
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _, msg):
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type):
        super().__init__(LCMTopic(topic, type))
        self.lcm = LCM()

    def __reduce__(self):
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg):
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class ZenohTransport(PubSubTransport[T]): ...
